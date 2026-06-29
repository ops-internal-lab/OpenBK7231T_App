/* ==========================================================================
   hal_bt_proxy_espidf.c  --  ESP-IDF BT proxy (Bluedroid) + NimBLE stubs

   On targets where Bluedroid is enabled (classic ESP32 with CONFIG_BT_BLUEDROID_ENABLED):
     Connection-only mode: MAC is hardcoded so scanning is disabled entirely.
     No GAP scan → no BLE/WiFi radio contention → full airtime for WiFi.
     Only the connection slots and command queue are kept.

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
#include "esp_err.h"

static int s_bt_proxy_init_done   = 0;
static int s_bt_proxy_health_tick = 0;

/* Scanning is intentionally disabled: MAC address is hardcoded so discovery
   is never needed. Removing the scan eliminates BLE/WiFi radio contention
   and the GAP callback flood that was saturating the CPU. */

static QueueHandle_t s_bt_cmd_queue = NULL;
#define BT_CMD_QUEUE_SIZE 10

static bt_proxy_conn_slot_t s_bt_connections[BT_PROXY_MAX_CONNECTIONS] = {0};

static void HAL_BTProxy_LogHealth(const char *stage)
{
	if (!stage) stage = "unknown";
	ADDLOG_INFO(LOG_FEATURE_GENERAL,
		"BT proxy health: stage=%s init=%d ctrl_status=%d free_heap=%u",
		stage,
		s_bt_proxy_init_done,
		(int)esp_bt_controller_get_status(),
		(unsigned int)xPortGetFreeHeapSize());
}

/* Scan disabled: these return empty/zero so callers compile and link cleanly. */
int HAL_BTProxy_PopScanResult(uint8_t *mac, int *rssi,
                              uint8_t *addr_type, uint8_t *data, int *data_len)
{
	(void)mac; (void)rssi; (void)addr_type; (void)data; (void)data_len;
	return 0;
}

static void HAL_BTProxy_GapCallback(esp_gap_ble_cb_event_t event,
                                    esp_ble_gap_cb_param_t *param)
{
	/* Scan is disabled; only log unexpected events for diagnostics. */
	(void)param;
	switch (event) {
	case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
	case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
	case ESP_GAP_BLE_SCAN_RESULT_EVT:
		/* Should never fire — scan was never started. */
		ADDLOG_WARN(LOG_FEATURE_GENERAL, "BT proxy: unexpected scan event %d", (int)event);
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

	/* Scan is intentionally not started: MAC is hardcoded so discovery
	   is never needed. Skipping esp_ble_gap_set_scan_params() means the
	   radio is never handed to BLE, leaving full airtime for WiFi. */

	s_bt_proxy_init_done = 1;
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "BT proxy: ESP-IDF controller initialized (scan disabled)");
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
	if (scan_active)      *scan_active      = 0;  /* scan disabled */
	if (total_packets)    *total_packets    = 0;
	if (dropped_packets)  *dropped_packets  = 0;
	if (buffered_packets) *buffered_packets = 0;
	return 1;
}

int HAL_BTProxy_GetScanEntry(int newest_index, char *mac_buf, int mac_buf_len,
                             int *rssi, int *adv_len, int *evt_type, int *age_ms)
{
	/* Scan disabled — no entries. */
	(void)newest_index; (void)mac_buf; (void)mac_buf_len;
	(void)rssi; (void)adv_len; (void)evt_type; (void)age_ms;
	return 0;
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
