/* ===========================================================================
   jk_bms.c  --  JK-BMS BLE driver for ESP-IDF (NimBLE GATT client, read-only)

   Flow (NimBLE central, mirrors the esp-idf "blecent" example):
     sync -> connect(MAC) -> discover svc 0xFFE0 -> discover chr 0xFFE1
          -> try CCCD 0x2902 (optional – many JK CC2541 modules omit it)
          -> write 0x97 (device-info) then 0x96 (cell-info) to start streaming
     notify_rx -> reassemble 300-byte frames -> checksum -> decode -> callback
     A small task re-sends 0x96 every poll_ms so values keep refreshing.
   Read-only: only the 0x97/0x96 *read requests* are ever written.

   FIX NOTES (vs original):
   1. CCCD is now OPTIONAL.  Many JK BMS modules (CC2541 / HM-10 clones) do
      not expose a real 0x2902 descriptor – or expose one that rejects a write.
      The module starts streaming the moment it receives the 0x97/0x96 frames,
      regardless of CCCD state.  When CCCD discovery returns BLE_HS_EDONE with
      no handle found we now fall through and send the request frames anyway.
   2. The initial 0x97 + 0x96 kick is sent immediately after characteristic
      discovery (before any CCCD interaction) so the BMS wakes up even on
      modules that auto-notify without a CCCD write.
   3. s_subscribed is set as soon as the characteristic is discovered and the
      requests are sent, so the poll_task keeps the stream alive.
   4. Minor: disc_all_dscs start handle corrected to s_val_handle + 1.
   =========================================================================== */

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

#include "jk_bms.h"

static const char *TAG = "jk_bms";

/* nimble store init (provided by the bt component) */
void ble_store_config_init(void);

/* ---- GATT identifiers ---------------------------------------------------- */
static const ble_uuid16_t SVC_UUID  = BLE_UUID16_INIT(0xFFE0);
static const ble_uuid16_t CHR_UUID  = BLE_UUID16_INIT(0xFFE1);  /* notify+write */
static const ble_uuid16_t CCCD_UUID = BLE_UUID16_INIT(0x2902);

/* ---- request frames (final byte = additive checksum of bytes 0..18) ------ */
static const uint8_t REQ_DEVICE_INFO[20] = {
    0xAA,0x55,0x90,0xEB,0x97,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x11};
static const uint8_t REQ_CELL_INFO[20]   = {
    0xAA,0x55,0x90,0xEB,0x96,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x10};

/* ---- driver state -------------------------------------------------------- */
static jk_bms_cb_t  s_cb       = NULL;
static void        *s_ctx      = NULL;
static uint32_t     s_poll_ms  = 2000;
static ble_addr_t   s_peer;
static uint8_t      s_own_addr_type;

static uint16_t s_conn_handle  = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_svc_start, s_svc_end;
static uint16_t s_val_handle;          /* 0xFFE1 value handle (write target)  */
static uint16_t s_cccd_handle;         /* 0x2902 – 0 when absent              */
static volatile bool s_subscribed = false;

static float s_balance_start = 0.0f;   /* latched from the settings frame      */

/* ===========================================================================
   little-endian decode helpers (verified against synthetic 32S frames)
   =========================================================================== */
static inline uint16_t u16(const uint8_t* d, int i){ return d[i] | (d[i+1] << 8); }
static inline uint32_t u32(const uint8_t* d, int i){
    return (uint32_t)d[i] | ((uint32_t)d[i+1]<<8) |
           ((uint32_t)d[i+2]<<16) | ((uint32_t)d[i+3]<<24);
}
static inline int16_t s16(const uint8_t* d, int i){ return (int16_t)u16(d, i); }
static inline int32_t s32(const uint8_t* d, int i){ return (int32_t)u32(d, i); }

/* JK02_32S: cell-info uses offset 32; MOSFET temp is special (112+off = 144). */
static void decode_cell_info(const uint8_t *d)
{
    const int off = 32;
    jk_bms_data_t m = {0};

    m.total_voltage    = u32(d, 118 + off) * 0.001f;
    m.current          = s32(d, 126 + off) * 0.001f;
    m.soc              = d[141 + off];
    m.remaining_ah     = u32(d, 142 + off) * 0.001f;
    m.full_charge_ah   = u32(d, 146 + off) * 0.001f;
    m.temp_1           = s16(d, 130 + off) * 0.1f;
    m.temp_2           = s16(d, 132 + off) * 0.1f;
    m.temp_mosfet      = s16(d, 144)       * 0.1f;
    m.balance_current  = s16(d, 138 + off) * 0.001f;
    m.balancer_action  = d[140 + off];
    m.charge_enabled   = d[166 + off];
    m.discharge_enabled= d[167 + off];
    m.balancer_enabled = d[168 + off];

    float vmin = 10.0f, vmax = 0.0f; int n = 0;
    for (int i = 0; i < 32; i++) {
        float v = u16(d, 6 + i * 2) * 0.001f;
        if (v > 0) { m.cells[n] = v; n++; if (v < vmin) vmin = v; if (v > vmax) vmax = v; }
    }
    m.cell_count = n;
    m.cell_min   = (n ? vmin : 0.0f);
    m.cell_max   = vmax;
    m.balance_start_voltage = s_balance_start;

    if (s_cb) s_cb(&m, s_ctx);
}

/* ===========================================================================
   frame assembler: reassemble 300-byte frames from ~20-byte notifications
   =========================================================================== */
static uint8_t s_buf[400];
static int     s_len = 0;

static void feed(const uint8_t *data, int len)
{
    if (len >= 4 && data[0]==0x55 && data[1]==0xAA && data[2]==0xEB && data[3]==0x90)
        s_len = 0;                                   /* preamble -> new frame */
    for (int i = 0; i < len; i++)
        if (s_len < (int)sizeof(s_buf)) s_buf[s_len++] = data[i];
    if (s_len > 400) { s_len = 0; return; }

    if (s_len >= 300) {
        uint8_t sum = 0;
        for (int i = 0; i < 299; i++) sum += s_buf[i];
        if (sum == s_buf[299]) {
            if      (s_buf[4] == 0x02) decode_cell_info(s_buf);
            else if (s_buf[4] == 0x01) s_balance_start = u32(s_buf, 30) * 0.001f;
            /* 0x03 device-info: ignored */
        }
        s_len = 0;
    }
}

/* ===========================================================================
   kick_stream – send the two request frames that make the BMS start/continue
   streaming.  Called after characteristic discovery AND after any optional
   CCCD write.  Safe to call more than once.
   =========================================================================== */
static void kick_stream(uint16_t conn)
{
    ble_gattc_write_no_rsp_flat(conn, s_val_handle, REQ_DEVICE_INFO, sizeof(REQ_DEVICE_INFO));
    ble_gattc_write_no_rsp_flat(conn, s_val_handle, REQ_CELL_INFO,   sizeof(REQ_CELL_INFO));
    s_subscribed = true;
    ESP_LOGI(TAG, "stream kick sent");
}

/* ===========================================================================
   NimBLE GATT client
   =========================================================================== */
static void start_connect(void);

/* Called when the CCCD write-with-response completes (success or fail).
   Either way we kick the stream – a failed CCCD write just means the module
   auto-notifies; the BMS will still respond to the request frames.           */
static int on_cccd_write(uint16_t conn, const struct ble_gatt_error *err,
                         struct ble_gatt_attr *attr, void *arg)
{
    (void)attr; (void)arg;
    if (err->status != 0) {
        ESP_LOGW(TAG, "CCCD write failed (status=%d) – kicking stream anyway", err->status);
    } else {
        ESP_LOGI(TAG, "CCCD ack'd");
    }
    kick_stream(conn);
    return 0;
}

static int on_disc_dsc(uint16_t conn, const struct ble_gatt_error *err,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)chr_val_handle; (void)arg;
    if (err->status == 0) {
        if (ble_uuid_cmp(&dsc->uuid.u, &CCCD_UUID.u) == 0) {
            s_cccd_handle = dsc->handle;
            ESP_LOGI(TAG, "CCCD found at handle 0x%04x", s_cccd_handle);
        }
    } else if (err->status == BLE_HS_EDONE) {
        if (s_cccd_handle != 0) {
            /* CCCD exists: write it (with response so we know when it lands),
               then kick_stream from the callback.                              */
            uint8_t val[2] = { 0x01, 0x00 };
            int rc = ble_gattc_write_flat(conn, s_cccd_handle, val, sizeof(val),
                                          on_cccd_write, NULL);
            if (rc != 0) {
                ESP_LOGW(TAG, "CCCD write enqueue failed rc=%d, kicking anyway", rc);
                kick_stream(conn);
            }
        } else {
            /* No CCCD found – common on HM-10 / CC2541 clones.
               The BMS auto-notifies when it receives 0x97/0x96, so just send
               the request frames directly.                                     */
            ESP_LOGW(TAG, "no CCCD – auto-notify mode; kicking stream directly");
            kick_stream(conn);
        }
    }
    return 0;
}

static int on_disc_chr(uint16_t conn, const struct ble_gatt_error *err,
                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    if (err->status == 0) {
        s_val_handle = chr->val_handle;
        ESP_LOGI(TAG, "0xFFE1 val_handle=0x%04x props=0x%02x",
                 s_val_handle, chr->properties);
    } else if (err->status == BLE_HS_EDONE) {
        if (s_val_handle == 0) {
            ESP_LOGW(TAG, "0xFFE1 not found in service");
            return 0;
        }
        /* Discover descriptors so we can (optionally) write CCCD.
           FIX: start from s_val_handle + 1 (the descriptor range begins
           after the value attribute handle).                                   */
        s_cccd_handle = 0;
        ble_gattc_disc_all_dscs(conn, s_val_handle + 1, s_svc_end, on_disc_dsc, NULL);
    }
    return 0;
}

static int on_disc_svc(uint16_t conn, const struct ble_gatt_error *err,
                       const struct ble_gatt_svc *svc, void *arg)
{
    (void)arg;
    if (err->status == 0) {
        s_svc_start = svc->start_handle;
        s_svc_end   = svc->end_handle;
        ESP_LOGI(TAG, "0xFFE0 start=0x%04x end=0x%04x", s_svc_start, s_svc_end);
    } else if (err->status == BLE_HS_EDONE) {
        if (s_svc_start == 0) {
            ESP_LOGW(TAG, "service 0xFFE0 not found – is this a JK BMS?");
            return 0;
        }
        s_val_handle = 0;
        ble_gattc_disc_chrs_by_uuid(conn, s_svc_start, s_svc_end, &CHR_UUID.u,
                                    on_disc_chr, NULL);
    }
    return 0;
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "connected handle=%d; discovering 0xFFE0", s_conn_handle);
            s_svc_start = 0;
            ble_gattc_disc_svc_by_uuid(s_conn_handle, &SVC_UUID.u, on_disc_svc, NULL);
        } else {
            ESP_LOGW(TAG, "connect failed (status=%d); retrying", event->connect.status);
            start_connect();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "disconnected (reason=%d); reconnecting", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed  = false;
        s_len = 0;
        start_connect();
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        struct os_mbuf *om = event->notify_rx.om;
        uint16_t total = OS_MBUF_PKTLEN(om);
        uint8_t  chunk[256];
        uint16_t n = total > sizeof(chunk) ? sizeof(chunk) : total;
        os_mbuf_copydata(om, 0, n, chunk);
        feed(chunk, n);
        /* If notifications arrive before kick_stream was called (auto-notify
           module) accept them and mark ourselves subscribed so the poll keeps
           sending REQ_CELL_INFO to refresh the data.                          */
        if (!s_subscribed && s_val_handle != 0) {
            ESP_LOGI(TAG, "notification received before kick – auto-notify mode");
            s_subscribed = true;
        }
        return 0;
    }

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU = %d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

static void start_connect(void)
{
    /* direct connect by address (we already know the MAC; no scan needed) */
    int rc = ble_gap_connect(s_own_addr_type, &s_peer, 30000, NULL, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY)
        ESP_LOGE(TAG, "ble_gap_connect rc=%d", rc);
}

/* ---- host lifecycle ------------------------------------------------------ */
static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    ESP_LOGI(TAG, "NimBLE sync; own_addr_type=%d", s_own_addr_type);
    start_connect();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "nimble reset; reason=%d", reason);
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();                 /* blocks until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* periodically re-request cell-info so the readings keep refreshing */
static void poll_task(void *param)
{
    (void)param;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(s_poll_ms));
        if (s_subscribed && s_conn_handle != BLE_HS_CONN_HANDLE_NONE)
            ble_gattc_write_no_rsp_flat(s_conn_handle, s_val_handle,
                                        REQ_CELL_INFO, sizeof(REQ_CELL_INFO));
    }
}

/* ---- MAC string -> ble_addr_t (NimBLE wants little-endian / reversed) ----- */
static int parse_mac(const char *s, ble_addr_t *out)
{
    unsigned b[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x", &b[0],&b[1],&b[2],&b[3],&b[4],&b[5]) != 6)
        return -1;
    out->type = BLE_ADDR_PUBLIC;
    for (int i = 0; i < 6; i++) out->val[i] = (uint8_t)b[5 - i];
    return 0;
}

/* ===========================================================================
   public API
   =========================================================================== */
esp_err_t jk_bms_start(const jk_bms_config_t *cfg)
{
    if (!cfg || !cfg->mac || !cfg->on_update) return ESP_ERR_INVALID_ARG;
    if (parse_mac(cfg->mac, &s_peer) != 0)     return ESP_ERR_INVALID_ARG;

    s_cb      = cfg->on_update;
    s_ctx     = cfg->user_ctx;
    s_poll_ms = cfg->poll_ms ? cfg->poll_ms : 2000;

    esp_err_t err = nimble_port_init();        /* ESP-IDF v5.x */
    if (err != ESP_OK) { ESP_LOGE(TAG, "nimble_port_init: %d", err); return err; }

    ble_hs_cfg.reset_cb        = on_reset;
    ble_hs_cfg.sync_cb         = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();

    nimble_port_freertos_init(host_task);
    xTaskCreate(poll_task, "jk_poll", 3072, NULL, 5, NULL);
    return ESP_OK;
}

bool jk_bms_is_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_subscribed;
}
