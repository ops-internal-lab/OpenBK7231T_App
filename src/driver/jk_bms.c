/* ===========================================================================
   jk_bms.c  --  JK-BMS BLE driver for ESP-IDF (NimBLE GATT client)
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

void ble_store_config_init(void);

/* ---- GATT identifiers ---------------------------------------------------- */
static const ble_uuid16_t SVC_UUID  = BLE_UUID16_INIT(0xFFE0);
static const ble_uuid16_t CHR_UUID  = BLE_UUID16_INIT(0xFFE1);
static const ble_uuid16_t CCCD_UUID = BLE_UUID16_INIT(0x2902);

/* ---- request frames ------------------------------------------------------ */
static const uint8_t REQ_DEVICE_INFO[20] = {
    0xAA,0x55,0x90,0xEB,0x97,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x11};
static const uint8_t REQ_CELL_INFO[20]   = {
    0xAA,0x55,0x90,0xEB,0x96,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x10};

/* ---- driver state -------------------------------------------------------- */
static jk_bms_cb_t  s_cb       = NULL;
static void        *s_ctx      = NULL;
static ble_addr_t   s_peer;
static uint8_t      s_own_addr_type;

static uint16_t s_conn_handle  = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_svc_start, s_svc_end;
static uint16_t s_val_handle;          
static uint16_t s_cccd_handle;         

static volatile bool s_subscribed   = false;
static volatile bool s_kick_pending = false;
static volatile bool s_reconnect_pending = false;   // set on disconnect; cleared once poll_task reconnects
static float s_balance_start        = 0.0f;
static volatile uint32_t s_last_rx_ticks = 0; // Watchdog timer
static volatile uint32_t s_last_connect_attempt_ticks = 0; // Boot/reconnect watchdog
static volatile uint32_t s_disconnect_ticks = 0; // when the last disconnect happened
static volatile uint32_t s_connect_ticks = 0; // when the GATT link came up (handshake watchdog)
static volatile int      s_stream_rekicks = 0; // consecutive 0x96 re-kicks with no data
static volatile int      s_connect_fails  = 0; // consecutive failed connect ATTEMPTS (reset on success)

// Reconnect backoff schedule, indexed by consecutive failed connect attempts.
// A single drop of a previously-working link retries after 1 s as before; but
// if the BMS is genuinely away (attempt after attempt fails), we back off so
// the radio spends most of its time IDLE instead of initiating. This matters
// because on a single-radio part (ESP32-C3) the *initiating* state occupies
// the receiver for scan_window out of every scan_itvl (see start_connect) and
// competes with WiFi for antenna time via the SW coexistence arbiter -- a
// permanent back-to-back connect loop visibly degrades WiFi/HTTP.
static const uint32_t s_backoff_ms[] = { 1000, 2000, 5000, 15000, 30000 };
#define JKBMS_BACKOFF_STEPS (sizeof(s_backoff_ms) / sizeof(s_backoff_ms[0]))

static uint32_t jkbms_retry_delay_ms(void)
{
    int f = s_connect_fails;
    if (f < 0) f = 0;
    if (f >= (int)JKBMS_BACKOFF_STEPS) f = (int)JKBMS_BACKOFF_STEPS - 1;
    return s_backoff_ms[f];
}

// One direct-connect attempt window. 10 s of listening is plenty for a JK
// that advertises every ~1 s; keeping the window short (vs the old 30 s)
// means the backoff gaps between attempts actually free the radio.
#define JKBMS_CONNECT_ATTEMPT_MS 10000

// If we're still not connected this long after the last connect attempt,
// poll_task() will kick off another one (see poll_task below). This covers
// the case where ble_gap_connect() failed synchronously - e.g. the BLE
// controller wasn't fully up yet at boot - so no BLE_GAP_EVENT_CONNECT or
// BLE_GAP_EVENT_DISCONNECT ever fires and the normal retry paths in
// gap_event() never trigger, leaving the BMS stuck showing "Not paired".
#define JKBMS_RECONNECT_TIMEOUT_MS 60000

// Handshake watchdog: once GAP-connected, the GATT bring-up (MTU exchange,
// service/char/CCCD discovery, CCCD write, first stream kick) must finish
// within this window. If it stalls - discovery returns without finding
// 0xFFE0/0xFFE1/CCCD, a callback fires with an unexpected error status, or a
// ble_gattc_*() call fails synchronously so no callback ever runs - the
// connection stays up but s_subscribed never becomes true. None of the other
// watchdogs cover that state (the reconnect logic only runs when there is NO
// connection). When this fires we terminate the link so the normal
// disconnect -> reconnect path takes over.
#define JKBMS_HANDSHAKE_TIMEOUT_MS 15000

// Stream watchdog escalation: after this many consecutive 0x96 re-kicks with
// still no data, stop kicking a link that clearly won't stream and terminate
// it so we reconnect from scratch (a fresh connection often revives a JK that
// has silently stopped notifying).
#define JKBMS_STREAM_REKICK_LIMIT 3

/* ---- decode helpers ------------------------------------------------------ */
static inline uint16_t u16(const uint8_t* d, int i){ return d[i] | (d[i+1] << 8); }
static inline uint32_t u32(const uint8_t* d, int i){
    return (uint32_t)d[i] | ((uint32_t)d[i+1]<<8) | ((uint32_t)d[i+2]<<16) | ((uint32_t)d[i+3]<<24);
}
static inline int16_t s16(const uint8_t* d, int i){ return (int16_t)u16(d, i); }
static inline int32_t s32(const uint8_t* d, int i){ return (int32_t)u32(d, i); }

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
        // Basic safety bound to prevent struct overflow
        if (v > 0) { 
            m.cells[n] = v; 
            n++; 
            if (v < vmin) vmin = v; 
            if (v > vmax) vmax = v; 
        }
    }
    m.cell_count = n;
    m.cell_min   = (n ? vmin : 0.0f);
    m.cell_max   = vmax;
    m.balance_start_voltage = s_balance_start;

    if (s_cb) s_cb(&m, s_ctx);
}

/* ---- frame assembler ----------------------------------------------------- */
static uint8_t s_buf[400];
static int     s_len = 0;

static void feed(const uint8_t *data, int len)
{
    // Feed resets the watchdog timer
    s_last_rx_ticks = xTaskGetTickCount();
    s_stream_rekicks = 0; // healthy data clears the escalation counter

    if (len >= 4 && data[0]==0x55 && data[1]==0xAA && data[2]==0xEB && data[3]==0x90)
        s_len = 0;                                   
    for (int i = 0; i < len; i++)
        if (s_len < (int)sizeof(s_buf)) s_buf[s_len++] = data[i];
    if (s_len > 400) { s_len = 0; return; }

    if (s_len >= 300) {
        uint8_t sum = 0;
        for (int i = 0; i < 299; i++) sum += s_buf[i];
        if (sum == s_buf[299]) {
            if      (s_buf[4] == 0x02) decode_cell_info(s_buf);
            else if (s_buf[4] == 0x01) s_balance_start = u32(s_buf, 30) * 0.001f;
        }
        s_len = 0;
    }
}

/* ---- NimBLE GATT client -------------------------------------------------- */
static void start_connect(void);

static int on_subscribe(uint16_t conn, const struct ble_gatt_error *err,
                        struct ble_gatt_attr *attr, void *arg)
{
    (void)attr; (void)arg;
    if (err->status != 0) {
        ESP_LOGW(TAG, "enable notify failed (status=%d)", err->status);
        ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    
    // Kick the stream: Send 0x97 immediately. Task will send 0x96 in 300ms.
    ble_gattc_write_no_rsp_flat(conn, s_val_handle, REQ_DEVICE_INFO, sizeof(REQ_DEVICE_INFO));
    s_kick_pending = true; 
    ESP_LOGI(TAG, "Subscribed. Sent 0x97 (device-info). Kicking 0x96 in 300ms...");
    return 0;
}

static int on_disc_dsc(uint16_t conn, const struct ble_gatt_error *err,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)chr_val_handle; (void)arg;
    if (err->status == 0) {
        if (ble_uuid_cmp(&dsc->uuid.u, &CCCD_UUID.u) == 0) s_cccd_handle = dsc->handle;
    } else if (err->status == BLE_HS_EDONE) {
        if (s_cccd_handle == 0) { ESP_LOGW(TAG, "no CCCD found"); return 0; }
        uint8_t val[2] = { 0x01, 0x00 };             
        ble_gattc_write_flat(conn, s_cccd_handle, val, sizeof(val), on_subscribe, NULL);
    }
    return 0;
}

static int on_disc_chr(uint16_t conn, const struct ble_gatt_error *err,
                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    if (err->status == 0) {
        s_val_handle = chr->val_handle;              
    } else if (err->status == BLE_HS_EDONE) {
        if (s_val_handle == 0) { ESP_LOGW(TAG, "0xFFE1 not found"); return 0; }
        s_cccd_handle = 0;
        ble_gattc_disc_all_dscs(conn, s_val_handle, s_svc_end, on_disc_dsc, NULL);
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
    } else if (err->status == BLE_HS_EDONE) {
        if (s_svc_start == 0) { ESP_LOGW(TAG, "service 0xFFE0 not found"); return 0; }
        s_val_handle = 0;
        ble_gattc_disc_chrs_by_uuid(conn, s_svc_start, s_svc_end, &CHR_UUID.u,
                                    on_disc_chr, NULL);
    }
    return 0;
}

static int on_mtu(uint16_t conn, const struct ble_gatt_error *err,
                  uint16_t mtu, void *arg)
{
    ESP_LOGI(TAG, "MTU exchanged: %d", mtu);
    s_svc_start = 0;
    ble_gattc_disc_svc_by_uuid(conn, &SVC_UUID.u, on_disc_svc, NULL);
    return 0;
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connect_ticks = xTaskGetTickCount(); // arm handshake watchdog
            s_connect_fails = 0;                   // peer reachable: reset backoff
            ESP_LOGI(TAG, "Connected; exchanging MTU...");
            ble_gattc_exchange_mtu(s_conn_handle, on_mtu, NULL);
        } else {
            // Do NOT call start_connect() from here. Two reasons:
            //  1. Same host-task re-entry hazard the DISCONNECT handler below
            //     already documents -- this callback runs on the NimBLE host
            //     task, and re-entering ble_gap_connect() from inside a GAP
            //     event risks stalling the host's event processing.
            //  2. An immediate retry produces a back-to-back initiating loop
            //     (each attempt = scan_window/scan_itvl receiver duty) with no
            //     idle gap for WiFi whenever the BMS is away. Scheduling the
            //     retry through poll_task applies the backoff instead.
            ESP_LOGW(TAG, "Connect failed (status=%d); retry in %u ms",
                     event->connect.status, (unsigned)jkbms_retry_delay_ms());
            s_connect_fails++;
            s_disconnect_ticks  = xTaskGetTickCount();
            s_reconnect_pending = true;
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected (reason=%d); will reconnect shortly", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed  = false;
        s_kick_pending = false;
        s_stream_rekicks = 0;
        s_len = 0;
        // Do NOT reconnect from here: this callback runs on the NimBLE host
        // task, and blocking it (this used to vTaskDelay here) or re-entering
        // ble_gap_connect() from inside a GAP event risks stalling the host's
        // event processing - if that ever happens, this was the *only* path
        // that could trigger a reconnect, so the BMS would stay disconnected
        // forever. Instead, just schedule the retry; poll_task (a separate,
        // independent task) performs the actual reconnect once the short
        // cooldown elapses.
        s_disconnect_ticks  = xTaskGetTickCount();
        s_reconnect_pending = true;
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        struct os_mbuf *om = event->notify_rx.om;
        uint16_t total = OS_MBUF_PKTLEN(om);
        uint8_t  chunk[256];
        uint16_t n = total > sizeof(chunk) ? sizeof(chunk) : total;
        os_mbuf_copydata(om, 0, n, chunk);
        feed(chunk, n);
        return 0;
    }

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU = %d", event->mtu.value);
        return 0;
        
    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "Connection parameters updated (status=%d)", event->conn_update.status);
        return 0;
        
    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
        // Returning 0 auto-accepts the peripheral's requested connection parameters
        ESP_LOGI(TAG, "Connection parameter update requested by peripheral. Accepting.");
        return 0;

    default:
        return 0;
    }
}

static void start_connect(void)
{
    // Any disconnect-triggered reconnect that was scheduled is moot now,
    // regardless of which path (poll_task or a GAP event) got us here.
    s_reconnect_pending = false;

    // Record the attempt regardless of outcome so the poll_task watchdog
    // knows when we last tried, even if ble_gap_connect() below fails
    // synchronously and never generates a GAP event of its own.
    s_last_connect_attempt_ticks = xTaskGetTickCount();

    // Explicit connection parameters. The key pair is scan_itvl/scan_window:
    // a "direct connect by MAC" still puts the controller in the INITIATING
    // state, where it listens for the peer's advertising packets exactly like
    // a scan (just address-filtered) -- these two fields set that listening
    // duty cycle. NimBLE's defaults (NULL params) are itvl == window == 10 ms,
    // i.e. the receiver is on 100% of the attempt window, which starves WiFi
    // on a shared-radio chip. 10 ms every 60 ms (~17%) still catches a JK
    // advertising at ~1 Hz within a few seconds, but leaves the antenna free.
    //
    // The link parameters are deliberately relaxed too: a 45-90 ms connection
    // interval is far more than fast enough for a 300-byte frame every 2 s,
    // and a 5 s supervision timeout rides out WiFi-induced jitter that would
    // drop the link at NimBLE's default 640 ms -- fewer drops means fewer
    // reconnect cycles in the first place.
    struct ble_gap_conn_params cp = {
        .scan_itvl           = 0x0060,   /* 96  * 0.625 ms = 60 ms            */
        .scan_window         = 0x0010,   /* 16  * 0.625 ms = 10 ms (~17% duty)*/
        .itvl_min            = 36,       /* 36  * 1.25 ms  = 45 ms            */
        .itvl_max            = 72,       /* 72  * 1.25 ms  = 90 ms            */
        .latency             = 0,
        .supervision_timeout = 500,      /* 500 * 10 ms    = 5 s              */
        .min_ce_len          = 0,
        .max_ce_len          = 0,
    };

    int rc = ble_gap_connect(s_own_addr_type, &s_peer,
                             JKBMS_CONNECT_ATTEMPT_MS, &cp, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        // Synchronous failure: no GAP event will ever fire for this attempt,
        // so nothing else would retry until the 60 s watchdog. Schedule a
        // backoff retry ourselves (poll_task picks it up), same as an async
        // connect failure.
        ESP_LOGE(TAG, "ble_gap_connect rc=%d; retry in %u ms",
                 rc, (unsigned)jkbms_retry_delay_ms());
        s_connect_fails++;
        s_disconnect_ticks  = xTaskGetTickCount();
        s_reconnect_pending = true;
    } else {
        ESP_LOGI(TAG, "Attempting direct connect to BMS (attempt fails so far: %d)...",
                 s_connect_fails);
    }
}

/* ---- host lifecycle ------------------------------------------------------ */
static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    start_connect();
}

static void on_reset(int reason) { ESP_LOGW(TAG, "nimble reset; reason=%d", reason); }

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();                 
    nimble_port_freertos_deinit();
}

/* ---- polling watchdog task ----------------------------------------------- */
static void poll_task(void *param)
{
    (void)param;
    for (;;) {
        if (s_kick_pending) {
            vTaskDelay(pdMS_TO_TICKS(300));
            s_kick_pending = false;
            if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_val_handle != 0) {
                ble_gattc_write_no_rsp_flat(s_conn_handle, s_val_handle,
                                            REQ_CELL_INFO, sizeof(REQ_CELL_INFO));
                s_subscribed = true;
                s_last_rx_ticks = xTaskGetTickCount(); // Start the watchdog
                ESP_LOGI(TAG, "Sent 0x96 initialization -- listening for stream...");
            }
        } else if (s_subscribed && s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            // Watchdog: If no data has been received for 5 seconds, the stream stopped.
            if ((xTaskGetTickCount() - s_last_rx_ticks) > pdMS_TO_TICKS(5000)) {
                if (s_stream_rekicks >= JKBMS_STREAM_REKICK_LIMIT) {
                    // Re-kicking isn't reviving the stream; drop the link and
                    // reconnect from scratch via the disconnect path.
                    ESP_LOGW(TAG, "Stream dead after %d re-kicks; terminating to force reconnect",
                             s_stream_rekicks);
                    s_stream_rekicks = 0;
                    s_last_rx_ticks  = xTaskGetTickCount();
                    ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                } else {
                    ESP_LOGW(TAG, "Stream stalled. Re-sending 0x96 kick (attempt %d)...",
                             s_stream_rekicks + 1);
                    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_val_handle != 0) {
                        ble_gattc_write_no_rsp_flat(s_conn_handle, s_val_handle,
                                                    REQ_CELL_INFO, sizeof(REQ_CELL_INFO));
                    }
                    s_stream_rekicks++;
                    s_last_rx_ticks = xTaskGetTickCount();
                }
            }
        } else if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            // Connected at the GAP level but the GATT handshake never finished
            // (s_subscribed still false, no kick pending). This is the state
            // none of the other branches see: the reconnect logic below only
            // runs when there is NO connection, and the stream watchdog above
            // requires s_subscribed. If the handshake stalls here - service /
            // characteristic / CCCD not found, a discovery callback fired with
            // an unexpected error status, or a ble_gattc_*() call failed
            // synchronously so no callback ever ran - nothing else recovers.
            // Tear the link down after a timeout; BLE_GAP_EVENT_DISCONNECT then
            // sets s_reconnect_pending and the reconnect branch takes over.
            if ((xTaskGetTickCount() - s_connect_ticks) > pdMS_TO_TICKS(JKBMS_HANDSHAKE_TIMEOUT_MS)) {
                ESP_LOGW(TAG, "Handshake stalled (connected but not subscribed after %d s); "
                              "terminating link to force reconnect",
                         JKBMS_HANDSHAKE_TIMEOUT_MS / 1000);
                // Push the timer forward so we don't re-fire every 100 ms while
                // the disconnect event is in flight.
                s_connect_ticks = xTaskGetTickCount();
                int rc = ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                if (rc != 0) {
                    // Terminate didn't take (e.g. handle already gone). Force
                    // our own state to disconnected so the reconnect branch
                    // runs next tick instead of us spinning here forever.
                    ESP_LOGW(TAG, "terminate rc=%d; forcing disconnected state", rc);
                    s_conn_handle       = BLE_HS_CONN_HANDLE_NONE;
                    s_subscribed        = false;
                    s_kick_pending      = false;
                    s_disconnect_ticks  = xTaskGetTickCount();
                    s_reconnect_pending = true;
                }
            }
        } else /* s_conn_handle == BLE_HS_CONN_HANDLE_NONE */ {
            if (s_reconnect_pending) {
                // A disconnect or a failed connect attempt scheduled a retry
                // (see gap_event). Wait out the backoff, then reconnect --
                // from this task, not the NimBLE host task. The delay grows
                // with consecutive failed attempts (1 s ... 30 s) so a BMS
                // that is genuinely away doesn't keep the radio initiating
                // full-time; a one-off drop of a working link still retries
                // after 1 s exactly as before.
                if ((xTaskGetTickCount() - s_disconnect_ticks) >= pdMS_TO_TICKS(jkbms_retry_delay_ms())) {
                    ESP_LOGI(TAG, "Reconnecting (backoff was %u ms)...",
                             (unsigned)jkbms_retry_delay_ms());
                    start_connect();
                }
            } else if ((xTaskGetTickCount() - s_last_connect_attempt_ticks) > pdMS_TO_TICKS(JKBMS_RECONNECT_TIMEOUT_MS)) {
                // Boot/reconnect watchdog: we're not connected at the GAP
                // level at all, and no disconnect-triggered retry is queued
                // either. Normally a failed/aborted connect attempt is
                // retried from inside gap_event() (BLE_GAP_EVENT_CONNECT).
                // But if ble_gap_connect() itself failed synchronously (most
                // commonly right at boot, before the BLE controller is fully
                // ready), no GAP event ever fires and that retry path never
                // runs - the BMS is then stuck showing "Not paired" forever.
                // As a backstop, retry once a minute in that situation.
                ESP_LOGW(TAG, "Still not connected after %d s; retrying connect to BMS...",
                         JKBMS_RECONNECT_TIMEOUT_MS / 1000);
                start_connect();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Yield to freeRTOS
    }
}

/* ---- MAC string -> ble_addr_t -------------------------------------------- */
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

    esp_err_t err = nimble_port_init();        
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
