/* ==========================================================================
   hal_bt_proxy_espidf.c  --  ESP-IDF BT proxy (Bluedroid) + NimBLE stubs

   On targets where Bluedroid is enabled (classic ESP32 with CONFIG_BT_BLUEDROID_ENABLED):
     Full implementation: GAP scan, connection slots, command queue.

   On NimBLE-only targets (ESP32-C3, C6, S3 — our primary build):
     Minimal stubs so the rest of the codebase links cleanly.
     All actual BLE is handled by jk_bms.c (NimBLE central).
   ========================================================================== */

#if PLATFORM_ESPIDF

#include "../../new_common.h"
#include "../../logging/logging.h"

/* =========================================================================
   PATH A: Bluedroid available
   ========================================================================= */
#if defined(CONFIG_BT_BLUEDROID_ENABLED) && CONFIG_BT_BLUEDROID_ENABLED

#include "../hal_bt_proxy.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_timer.h"
#include "esp_err.h"

static int s_bt_proxy_init_done      = 0;
static int s_bt_proxy_health_tick    = 0;
static int s_bt_scan_active          = 0;
static int s_bt_scan_total_packets   = 0;
static int s_bt_scan_dropped_packets = 0;

#define BT_SCAN_RING_SIZE 32
typedef struct {
	uint8_t  bda[6];
	int      rssi;
	int      adv_len;
	uint8_t  addr_type;
	int      evt_type;
	uint8_t  data[62];
	uint32_t ts_ms;
} bt_scan_entry_t;

static bt_scan_entry_t s_bt_scan_ring[BT_SCAN_RING_SIZE];
static int s_bt_scan_head  = 0;
static int s_bt_scan_tail  = 0;
static int s_bt_scan_count = 0;

static QueueHandle_t s_bt_cmd_queue = NULL;
#define BT_CMD_QUEUE_SIZE 10

static bt_proxy_conn_slot_t s_bt_connections[BT_PROXY_MAX_CONNECTIONS] = {0};

static const esp_ble_scan_params_t s_ble_scan_params = {
	.scan_type          = BLE_SCAN_TYPE_ACTIVE,
	.own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
	.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
	.scan_interval      = 0x50,
	/* scan_window reduced from 0x30 → 0x10 (10ms/50ms = 20% duty cycle).
	   BLE and WiFi share the radio on ESP32; a 60% duty cycle (0x30/0x50)
	   caused significant WiFi packet loss and made the web UI sluggish.
	   20% is more than enough to catch nearby advertisements. */
	.scan_window        = 0x10,
	/* DUPLICATE_ENABLE: only report each peer once per scan period instead
	   of every advertisement packet. With DUPLICATE_DISABLE the GAP callback
	   fires thousands of times/sec in a populated BLE environment, eating CPU
	   time and contributing to watchdog panics. */
	.scan_duplicate     = BLE_SCAN_DUPLICATE_ENABLE
};

static void HAL_BTProxy_LogHealth(const char *stage)
{
	if (!stage) stage = "unknown";
	ADDLOG_INFO(LOG_FEATURE_GENERAL,
		"BT proxy health: stage=%s init=%d scan=%d total=%d dropped=%d buffered=%d ctrl_status=%d free_heap=%u",
		stage,
		s_bt_proxy_init_done,
		s_bt_scan_active,
		s_bt_scan_total_packets,
		s_bt_scan_dropped_packets,
		s_bt_scan_count,
		(int)esp_bt_controller_get_status(),
		(unsigned int)xPortGetFreeHeapSize());
}

static void HAL_BTProxy_RecordScan(const esp_ble_gap_cb_param_t *param)
{
	int pos;
	bt_scan_entry_t *e;
	int copy_len;

	if (!param) return;

	pos = s_bt_scan_head;
	e   = &s_bt_scan_ring[pos];
	memcpy(e->bda, param->scan_rst.bda, sizeof(e->bda));
	e->rssi      = param->scan_rst.rssi;
	e->addr_type = param->scan_rst.ble_addr_type;
	e->adv_len   = param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len;
	e->evt_type  = param->scan_rst.ble_evt_type;
	copy_len     = e->adv_len > 62 ? 62 : e->adv_len;
	memcpy(e->data, param->scan_rst.ble_adv, copy_len);
	e->ts_ms     = (uint32_t)(esp_timer_get_time() / 1000ULL);

	s_bt_scan_head = (s_bt_scan_head + 1) % BT_SCAN_RING_SIZE;
	if (s_bt_scan_count < BT_SCAN_RING_SIZE) {
		s_bt_scan_count++;
	} else {
		s_bt_scan_tail = (s_bt_scan_tail + 1) % BT_SCAN_RING_SIZE;
		s_bt_scan_dropped_packets++;
	}
	s_bt_scan_total_packets++;
}

int HAL_BTProxy_PopScanResult(uint8_t *mac, int *rssi,
                              uint8_t *addr_type, uint8_t *data, int *data_len)
{
	bt_scan_entry_t *e;
	if (s_bt_scan_count == 0) return 0;
	e = &s_bt_scan_ring[s_bt_scan_tail];
	memcpy(mac, e->bda, 6);
	*rssi      = e->rssi;
	*addr_type = e->addr_type;
	*data_len  = e->adv_len > 62 ? 62 : e->adv_len;
	memcpy(data, e->data, *data_len);
	s_bt_scan_tail = (s_bt_scan_tail + 1) % BT_SCAN_RING_SIZE;
	s_bt_scan_count--;
	return 1;
}

static void HAL_BTProxy_StartScan(void)
{
	esp_err_t err = esp_ble_gap_start_scanning(0);
	if (err == ESP_OK) {
		s_bt_scan_active = 1;
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "BT proxy: BLE scan started");
	} else {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BT proxy: start scan failed: %s",
		             esp_err_to_name(err));
	}
}

static void HAL_BTProxy_GapCallback(esp_gap_ble_cb_event_t event,
                                    esp_ble_gap_cb_param_t *param)
{
	switch (event) {
	case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
		HAL_BTProxy_StartScan();
		break;
	case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
		if (param && param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
			ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BT proxy: scan start status=%d",
			             param->scan_start_cmpl.status);
		break;
	case ESP_GAP_BLE_SCAN_RESULT_EVT:
		if (param && param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT)
			HAL_BTProxy_RecordScan(param);
		break;
	default:
		break;
	}
}

static void HAL_BTProxy_InitController(void)
{
	esp_err_t err;
	int i;

	if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
		s_bt_proxy_init_done = 1;
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "BT proxy: controller already enabled");
		HAL_BTProxy_LogHealth("already_enabled");
		return;
	}

	err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BT proxy: mem_release failed: %s",
		             esp_err_to_name(err));
		return;
	}

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	err = esp_bt_controller_init(&bt_cfg);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BT proxy: controller init failed: %s",
		             esp_err_to_name(err));
		return;
	}

	err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BT proxy: controller enable failed: %s",
		             esp_err_to_name(err));
		return;
	}

	err = esp_bluedroid_init();
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BT proxy: bluedroid init failed: %s",
		             esp_err_to_name(err));
		return;
	}
	err = esp_bluedroid_enable();
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BT proxy: bluedroid enable failed: %s",
		             esp_err_to_name(err));
		return;
	}

	s_bt_cmd_queue = xQueueCreate(BT_CMD_QUEUE_SIZE, sizeof(bt_proxy_cmd_t));
	if (s_bt_cmd_queue == NULL) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BT proxy: failed to allocate command queue");
		return;
	}

	for (i = 0; i < BT_PROXY_MAX_CONNECTIONS; i++) {
		s_bt_connections[i].state            = BT_CONN_STATE_DISCONNECTED;
		s_bt_connections[i].last_activity_ms = 0;
	}

	err = esp_ble_gap_register_callback(HAL_BTProxy_GapCallback);
	if (err != ESP_OK) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BT proxy: gap callback register failed: %s",
		             esp_err_to_name(err));
		return;
	}

	err = esp_ble_gap_set_scan_params((esp_ble_scan_params_t*)&s_ble_scan_params);
	if (err != ESP_OK) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BT proxy: set scan params failed: %s",
		             esp_err_to_name(err));
		return;
	}

	s_bt_proxy_init_done = 1;
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "BT proxy: ESP-IDF controller initialized");
	HAL_BTProxy_LogHealth("init_ok");
}

void HAL_BTProxy_PreInit(void)
{
#if ENABLE_BT_PROXY
	HAL_BTProxy_InitController();
#else
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "BT proxy: disabled by ENABLE_BT_PROXY");
#endif
}

void HAL_BTProxy_OnEverySecond(void)
{
#if ENABLE_BT_PROXY
	if (!s_bt_proxy_init_done) return;
	s_bt_proxy_health_tick++;
	if (s_bt_proxy_health_tick >= 60) {
		s_bt_proxy_health_tick = 0;
		HAL_BTProxy_LogHealth("periodic_60s");
	}
#endif
}

int HAL_BTProxy_GetScanStats(int *init_done, int *scan_active,
                             int *total_packets, int *dropped_packets,
                             int *buffered_packets)
{
	if (init_done)        *init_done        = s_bt_proxy_init_done;
	if (scan_active)      *scan_active      = s_bt_scan_active;
	if (total_packets)    *total_packets    = s_bt_scan_total_packets;
	if (dropped_packets)  *dropped_packets  = s_bt_scan_dropped_packets;
	if (buffered_packets) *buffered_packets = s_bt_scan_count;
	return 1;
}

int HAL_BTProxy_GetScanEntry(int newest_index, char *mac_buf, int mac_buf_len,
                             int *rssi, int *adv_len, int *evt_type, int *age_ms)
{
	int idx, pos;
	bt_scan_entry_t e;
	uint32_t now_ms;

	if (newest_index < 0 || newest_index >= s_bt_scan_count) return 0;
	if (!mac_buf || mac_buf_len < 18) return 0;

	idx = s_bt_scan_head - 1 - newest_index;
	while (idx < 0) idx += BT_SCAN_RING_SIZE;
	pos = idx % BT_SCAN_RING_SIZE;
	e   = s_bt_scan_ring[pos];

	snprintf(mac_buf, mac_buf_len, "%02X:%02X:%02X:%02X:%02X:%02X",
	         e.bda[0], e.bda[1], e.bda[2], e.bda[3], e.bda[4], e.bda[5]);

	if (rssi)     *rssi     = e.rssi;
	if (adv_len)  *adv_len  = e.adv_len;
	if (evt_type) *evt_type = e.evt_type;
	if (age_ms) {
		now_ms  = (uint32_t)(esp_timer_get_time() / 1000ULL);
		*age_ms = (int)(now_ms - e.ts_ms);
	}
	return 1;
}

#if ENABLE_BT_PROXY
int HAL_BTProxy_EnqueueCommand(bt_proxy_cmd_t *cmd)
{
	if (s_bt_cmd_queue == NULL) return -1;
	if (xQueueSend(s_bt_cmd_queue, cmd, 100) != pdPASS) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BT proxy: command queue is full");
		return -2;
	}
	return 0;
}
#endif /* ENABLE_BT_PROXY */

/* =========================================================================
   PATH B: NimBLE-only target (ESP32-C3 / C6 / S3)
   BLE is handled by jk_bms.c. Provide stubs so the rest links cleanly.
   ========================================================================= */
#else /* !CONFIG_BT_BLUEDROID_ENABLED */

void HAL_BTProxy_PreInit(void)
{
	/* NimBLE target: jk_bms.c owns BLE init via nimble_port_init(). */
}

void HAL_BTProxy_OnEverySecond(void) { }

int HAL_BTProxy_GetScanStats(int *init_done, int *scan_active,
                             int *total_packets, int *dropped_packets,
                             int *buffered_packets)
{
	if (init_done)        *init_done        = 0;
	if (scan_active)      *scan_active      = 0;
	if (total_packets)    *total_packets    = 0;
	if (dropped_packets)  *dropped_packets  = 0;
	if (buffered_packets) *buffered_packets = 0;
	return 0;
}

int HAL_BTProxy_GetScanEntry(int newest_index, char *mac_buf, int mac_buf_len,
                             int *rssi, int *adv_len, int *evt_type, int *age_ms)
{
	(void)newest_index; (void)mac_buf; (void)mac_buf_len;
	(void)rssi; (void)adv_len; (void)evt_type; (void)age_ms;
	return 0;
}

int HAL_BTProxy_PopScanResult(uint8_t *mac, int *rssi,
                              uint8_t *addr_type, uint8_t *data, int *data_len)
{
	(void)mac; (void)rssi; (void)addr_type; (void)data; (void)data_len;
	return 0;
}

#endif /* CONFIG_BT_BLUEDROID_ENABLED */

#endif /* PLATFORM_ESPIDF */
