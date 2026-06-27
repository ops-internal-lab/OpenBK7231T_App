/* ===========================================================================
   jk_bms.c  --  JK-BMS BLE driver for ESP-IDF (NimBLE GATT client, read-only)

   Sequencing:
     on_sync → start_scan → [DISC: target MAC found] → disc_cancel →
     start_connect → [CONNECT] → exchange_mtu → on_mtu →
     disc_all_svcs → on_disc_svc → [EDONE] →
     disc_all_chrs → on_disc_chr → [EDONE] →
     disc_all_dscs → on_disc_dsc → [EDONE] →
     write_flat CCCD(0x0001) → on_subscribe →
     write_no_rsp REQ_DEVICE_INFO(0x97) → [poll_task: 300ms] →
     write_no_rsp REQ_CELL_INFO(0x96) → notifications → decode → callback
     poll_task re-sends REQ_CELL_INFO every poll_ms thereafter.

   FIX HISTORY:
   v1  CCCD optional: kick stream even when 0x2902 is absent (CC2541 clones).
   v2  disc_all_svcs instead of disc_svc_by_uuid.
       Some JK BMS CC2541 modules ignore ATT opcode 0x06 "Find By Type Value"
       (used by disc_svc_by_uuid) and only respond to 0x10 "Read By Group Type"
       (used by disc_all_svcs).  Symptom: connected but immediately silent, no
       beep, because s_val_handle was never set and all writes were dropped.
   v3  Race-condition fix: NimBLE allows only ONE active GATT procedure per
       connection.  v2 called disc_all_chrs from inside the on_disc_svc
       status==0 branch while disc_all_svcs was still in progress.  NimBLE
       returned BLE_HS_EALREADY; the unchecked return left s_val_handle==0 and
       the chain silently stopped.  Fix: save handles in status==0; start chr
       discovery only from BLE_HS_EDONE when disc_all_svcs has finished.
       Added s_svc_start (was missing). Added rc checks on all ble_gattc_ calls.
   v4  Three additional fixes identified by comparing against the working HTML
       implementation and the ESPHome/Bluedroid JK-BMS driver:
       (a) Passive scan before connect: resolve the real BLE address type from
           the advertisement packet rather than hardcoding BLE_ADDR_PUBLIC.
           ESPHome connects after a scan; Chrome's Web Bluetooth does the same
           via requestDevice().  Wrong address type causes silent GATT failures
           even though the link-layer connection succeeds.
       (b) Explicit MTU exchange before service discovery: ESPHome logs mtu=131
           in its connection flow; the HTML gets a modern MTU from the OS stack.
           CC2541 supports up to ~247 bytes.  Larger MTU means fewer notification
           packets per 300-byte BMS frame, reducing assembly latency.  Discovery
           starts from the on_mtu callback, guaranteeing the ATT link is sized
           before any procedure runs.
       (c) 300 ms gap between 0x97 and 0x96: the working HTML does exactly:
               await write(DEVICE_INFO); await sleep(300); await write(CELL_INFO);
           Firing both writes back-to-back lets the CC2541 silently discard the
           second one (it's still processing the first, which triggers the beep).
           on_subscribe sends 0x97; poll_task delivers 0x96 after a 300 ms sleep.
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

/* ---- GATT UUIDs ---------------------------------------------------------- */
static const ble_uuid16_t SVC_UUID  = BLE_UUID16_INIT(0xFFE0);
static const ble_uuid16_t CHR_UUID  = BLE_UUID16_INIT(0xFFE1);
static const ble_uuid16_t CCCD_UUID = BLE_UUID16_INIT(0x2902);

/* ---- request frames ------------------------------------------------------ */
static const uint8_t REQ_DEVICE_INFO[20] = {
    0xAA,0x55,0x90,0xEB,0x97,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x11};
static const uint8_t REQ_CELL_INFO[20]   = {
    0xAA,0x55,0x90,0xEB,0x96,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x10};

/* ---- driver state -------------------------------------------------------- */
static jk_bms_cb_t  s_cb      = NULL;
static void        *s_ctx     = NULL;
static uint32_t     s_poll_ms = 2000;

/* s_peer_mac: 6 bytes in big-endian (as typed in "AA:BB:CC:DD:EE:FF").
   s_peer: NimBLE ble_addr_t (little-endian val[], type updated from scan).   */
static uint8_t    s_peer_mac[6];
static ble_addr_t s_peer;
static uint8_t    s_own_addr_type;

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_svc_start;
static uint16_t s_svc_end;
static uint16_t s_val_handle;    /* 0xFFE1 value handle (write target)        */
static uint16_t s_cccd_handle;   /* 0x2902 descriptor handle                  */

static volatile bool s_connecting   = false; /* gap_connect is pending        */
static volatile bool s_subscribed   = false;
static volatile bool s_kick_pending = false; /* poll_task: send 0x96 in 300ms */

static float s_balance_start = 0.0f;

/* ---- decode helpers ------------------------------------------------------ */
static inline uint16_t u16(const uint8_t *d, int i){ return d[i]|(d[i+1]<<8); }
static inline uint32_t u32(const uint8_t *d, int i){
    return (uint32_t)d[i]|((uint32_t)d[i+1]<<8)|
           ((uint32_t)d[i+2]<<16)|((uint32_t)d[i+3]<<24);
}
static inline int16_t s16(const uint8_t *d, int i){ return (int16_t)u16(d,i); }
static inline int32_t s32(const uint8_t *d, int i){ return (int32_t)u32(d,i); }

/* JK02_32S layout: cell-info at offset 32; MOSFET temp is fixed at byte 144. */
static void decode_cell_info(const uint8_t *d)
{
    const int off = 32;
    jk_bms_data_t m = {0};
    m.total_voltage    = u32(d,118+off)*0.001f;
    m.current          = s32(d,126+off)*0.001f;
    m.soc              = d[141+off];
    m.remaining_ah     = u32(d,142+off)*0.001f;
    m.full_charge_ah   = u32(d,146+off)*0.001f;
    m.temp_1           = s16(d,130+off)*0.1f;
    m.temp_2           = s16(d,132+off)*0.1f;
    m.temp_mosfet      = s16(d,144)*0.1f;
    m.balance_current  = s16(d,138+off)*0.001f;
    m.balancer_action  = d[140+off];
    m.charge_enabled   = d[166+off];
    m.discharge_enabled= d[167+off];
    m.balancer_enabled = d[168+off];
    float vmin=10.0f, vmax=0.0f; int n=0;
    for(int i=0;i<32;i++){
        float v=u16(d,6+i*2)*0.001f;
        if(v>0){m.cells[n]=v;n++;if(v<vmin)vmin=v;if(v>vmax)vmax=v;}
    }
    m.cell_count=n; m.cell_min=(n?vmin:0.0f); m.cell_max=vmax;
    m.balance_start_voltage=s_balance_start;
    if(s_cb) s_cb(&m,s_ctx);
}

/* ---- frame assembler ----------------------------------------------------- */
static uint8_t s_buf[400];
static int     s_len = 0;

static void feed(const uint8_t *data, int len)
{
    /* 0x55 0xAA 0xEB 0x90 is the BMS→host frame preamble */
    if(len>=4 && data[0]==0x55 && data[1]==0xAA && data[2]==0xEB && data[3]==0x90)
        s_len=0;
    for(int i=0;i<len;i++)
        if(s_len<(int)sizeof(s_buf)) s_buf[s_len++]=data[i];
    if(s_len>400){s_len=0;return;}
    if(s_len>=300){
        uint8_t sum=0;
        for(int i=0;i<299;i++) sum+=s_buf[i];
        if(sum==s_buf[299]){
            if     (s_buf[4]==0x02) decode_cell_info(s_buf);
            else if(s_buf[4]==0x01) s_balance_start=u32(s_buf,30)*0.001f;
        }
        s_len=0;
    }
}

/* ---- forward declarations ------------------------------------------------ */
static int  gap_event(struct ble_gap_event *event, void *arg);
static void start_scan(void);
static void start_connect(void);

/* ---- MAC comparison: NimBLE little-endian val[] vs. big-endian bytes ----- */
static bool addr_mac_match(const uint8_t addr_val[6], const uint8_t mac_be[6])
{
    /* NimBLE stores addr LSB-first: val[0]=mac[5], val[5]=mac[0] */
    for(int i=0;i<6;i++)
        if(addr_val[i] != mac_be[5-i]) return false;
    return true;
}

/* ==========================================================================
   GATT callback chain -- innermost callback first so each is defined before
   the one that references it.
   ========================================================================== */

/* on_subscribe: CCCD write acknowledged.
   Sends 0x97 (DEVICE_INFO) which triggers the BMS beep + settings frame.
   Sets s_kick_pending so poll_task sends 0x96 (CELL_INFO) 300 ms later.
   The 300 ms gap is critical: the CC2541 silently drops 0x96 if it arrives
   while still processing 0x97.  The working HTML has an explicit sleep(300)
   between the two writes; this replicates that timing via poll_task.         */
static int on_subscribe(uint16_t conn, const struct ble_gatt_error *err,
                        struct ble_gatt_attr *attr, void *arg)
{
    (void)attr; (void)arg;
    if(err->status != 0){
        ESP_LOGW(TAG,"CCCD write failed status=%d -- disconnecting",err->status);
        ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    ble_gattc_write_no_rsp_flat(conn, s_val_handle,
                                REQ_DEVICE_INFO, sizeof(REQ_DEVICE_INFO));
    s_kick_pending = true;
    ESP_LOGI(TAG,"CCCD OK; 0x97 sent (beep expected); 0x96 follows in 300ms");
    return 0;
}

/* on_disc_dsc: descriptor discovery complete.
   Find 0x2902 (CCCD) and write 0x0001 to enable notifications.              */
static int on_disc_dsc(uint16_t conn, const struct ble_gatt_error *err,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)chr_val_handle; (void)arg;
    if(err->status == 0){
        if(ble_uuid_cmp(&dsc->uuid.u, &CCCD_UUID.u) == 0){
            s_cccd_handle = dsc->handle;
            ESP_LOGI(TAG,"CCCD at 0x%04x", s_cccd_handle);
        }
    } else if(err->status == BLE_HS_EDONE){
        if(s_cccd_handle == 0){
            ESP_LOGW(TAG,"no CCCD found -- disconnecting to retry");
            ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        uint8_t val[2] = {0x01, 0x00};   /* notifications ON */
        int rc = ble_gattc_write_flat(conn, s_cccd_handle, val, sizeof(val),
                                      on_subscribe, NULL);
        if(rc != 0){
            ESP_LOGE(TAG,"CCCD write_flat rc=%d", rc);
            ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
    return 0;
}

/* on_disc_chr: find 0xFFE1 val_handle, then discover its descriptors.        */
static int on_disc_chr(uint16_t conn, const struct ble_gatt_error *err,
                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    if(err->status == 0){
        if(ble_uuid_cmp(&chr->uuid.u, &CHR_UUID.u) == 0){
            s_val_handle = chr->val_handle;
            ESP_LOGI(TAG,"0xFFE1 val_handle=0x%04x props=0x%02x",
                     s_val_handle, chr->properties);
        }
    } else if(err->status == BLE_HS_EDONE){
        if(s_val_handle == 0){
            ESP_LOGW(TAG,"0xFFE1 not found -- disconnecting");
            ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        s_cccd_handle = 0;
        /* Descriptors live after the value handle */
        int rc = ble_gattc_disc_all_dscs(conn, s_val_handle+1, s_svc_end,
                                         on_disc_dsc, NULL);
        if(rc != 0){
            ESP_LOGE(TAG,"disc_all_dscs rc=%d", rc);
            ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
    return 0;
}

/* on_disc_svc: find 0xFFE0, then (only in EDONE) start chr discovery.
   v3 fix: disc_all_svcs fires status==0 for every service it finds, then
   EDONE when the whole table has been walked.  Starting disc_all_chrs from
   status==0 means calling it while disc_all_svcs is still active -- NimBLE
   returns BLE_HS_EALREADY.  The unchecked rc left s_val_handle==0 forever.
   Save the handles in status==0; issue disc_all_chrs only from EDONE.        */
static int on_disc_svc(uint16_t conn, const struct ble_gatt_error *err,
                       const struct ble_gatt_svc *svc, void *arg)
{
    (void)arg;
    if(err->status == 0){
        if(ble_uuid_cmp(&svc->uuid.u, &SVC_UUID.u) == 0){
            s_svc_start = svc->start_handle;
            s_svc_end   = svc->end_handle;
            ESP_LOGI(TAG,"0xFFE0 start=0x%04x end=0x%04x",
                     s_svc_start, s_svc_end);
        }
    } else if(err->status == BLE_HS_EDONE){
        if(s_svc_end == 0){
            ESP_LOGW(TAG,"0xFFE0 not found -- is this a JK BMS?");
            ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        s_val_handle = 0;
        int rc = ble_gattc_disc_all_chrs(conn, s_svc_start, s_svc_end,
                                         on_disc_chr, NULL);
        if(rc != 0){
            ESP_LOGE(TAG,"disc_all_chrs rc=%d", rc);
            ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
    return 0;
}

/* on_mtu: MTU exchanged -- now safe to start service discovery.
   v4(b): waiting for MTU before discovery ensures the ATT link is sized to
   the negotiated value before any GATT procedures run.  ESPHome achieves
   mtu=131 this way; the HTML's OS stack also negotiates before any GATT ops. */
static int on_mtu(uint16_t conn, const struct ble_gatt_error *err,
                  uint16_t mtu, void *arg)
{
    (void)arg;
    ESP_LOGI(TAG,"MTU=%d (status=%d)", mtu, err->status);
    int rc = ble_gattc_disc_all_svcs(conn, on_disc_svc, NULL);
    if(rc != 0){
        ESP_LOGE(TAG,"disc_all_svcs rc=%d", rc);
        ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}

/* ---- GAP event handler --------------------------------------------------- */
static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch(event->type){

    /* v4(a): scan result -- look for our target MAC regardless of addr type */
    case BLE_GAP_EVENT_DISC: {
        if(!addr_mac_match(event->disc.addr.val, s_peer_mac)) return 0;
        s_peer = event->disc.addr;    /* capture the real address type        */
        ESP_LOGI(TAG,"BMS found: addr_type=%d RSSI=%d -- connecting",
                 s_peer.type, event->disc.rssi);
        s_connecting = true;
        ble_gap_disc_cancel();        /* stop scan; DISC_COMPLETE follows      */
        start_connect();
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        /* Only retry scan if connect was not already initiated */
        if(!s_connecting && s_conn_handle == BLE_HS_CONN_HANDLE_NONE){
            ESP_LOGW(TAG,"scan complete: BMS not found -- retrying");
            start_scan();
        }
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        s_connecting = false;
        if(event->connect.status == 0){
            s_conn_handle = event->connect.conn_handle;
            s_svc_start = 0; s_svc_end = 0; s_val_handle = 0;
            ESP_LOGI(TAG,"connected handle=%d -- requesting MTU 247",
                     s_conn_handle);
            ble_att_set_preferred_mtu(247);
            int rc = ble_gattc_exchange_mtu(s_conn_handle, on_mtu, NULL);
            if(rc != 0){
                /* MTU exchange not available or already done -- proceed */
                ESP_LOGW(TAG,"exchange_mtu rc=%d -- starting discovery",rc);
                rc = ble_gattc_disc_all_svcs(s_conn_handle, on_disc_svc, NULL);
                if(rc != 0){
                    ESP_LOGE(TAG,"disc_all_svcs rc=%d", rc);
                    ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                }
            }
        } else {
            ESP_LOGW(TAG,"connect failed status=%d -- retrying scan",
                     event->connect.status);
            start_scan();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG,"disconnected reason=%d -- retrying scan",
                 event->disconnect.reason);
        s_conn_handle  = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed   = false;
        s_kick_pending = false;
        s_connecting   = false;
        s_len          = 0;
        s_svc_start = 0; s_svc_end = 0; s_val_handle = 0;
        start_scan();
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        struct os_mbuf *om = event->notify_rx.om;
        uint16_t total = OS_MBUF_PKTLEN(om);
        uint8_t  chunk[256];
        uint16_t n = total > sizeof(chunk) ? (uint16_t)sizeof(chunk) : total;
        os_mbuf_copydata(om, 0, n, chunk);
        feed(chunk, n);
        return 0;
    }

    case BLE_GAP_EVENT_MTU:
        /* BMS-initiated MTU exchange (fires if the remote side requests it) */
        ESP_LOGI(TAG,"MTU (remote-initiated): %d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

/* ---- connection / scan helpers ------------------------------------------- */
static void start_connect(void)
{
    int rc = ble_gap_connect(s_own_addr_type, &s_peer, 30000, NULL, gap_event, NULL);
    if(rc != 0 && rc != BLE_HS_EALREADY)
        ESP_LOGE(TAG,"ble_gap_connect rc=%d", rc);
}

static void start_scan(void)
{
    struct ble_gap_disc_params dp = {0};
    dp.passive           = 1;    /* don't send SCAN_REQ, just listen          */
    dp.filter_duplicates = 1;
    dp.itvl              = 160;  /* 160 × 0.625ms = 100ms scan interval       */
    dp.window            = 80;   /* 80  × 0.625ms = 50ms  scan window         */

    int rc = ble_gap_disc(s_own_addr_type, 10000 /*ms*/, &dp, gap_event, NULL);
    if(rc != 0 && rc != BLE_HS_EALREADY)
        ESP_LOGE(TAG,"ble_gap_disc rc=%d", rc);
    else
        ESP_LOGI(TAG,"scanning for BMS (10s)...");
}

/* ---- NimBLE host lifecycle ----------------------------------------------- */
static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    ESP_LOGI(TAG,"sync; own_addr_type=%d", s_own_addr_type);
    start_scan();
}

static void on_reset(int reason){ ESP_LOGW(TAG,"nimble reset reason=%d",reason); }

static void host_task(void *p)
{
    (void)p;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* poll_task: handles the initial 0x97→0x96 timing, then periodic polling.
   The 300ms gap in the s_kick_pending branch mirrors the HTML's sleep(300)
   between DEVICE_INFO and CELL_INFO writes -- required by the CC2541 module. */
static void poll_task(void *p)
{
    (void)p;
    for(;;){
        if(s_kick_pending){
            vTaskDelay(pdMS_TO_TICKS(300));     /* match HTML's await sleep(300) */
            s_kick_pending = false;
            if(s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_val_handle != 0){
                ble_gattc_write_no_rsp_flat(s_conn_handle, s_val_handle,
                                            REQ_CELL_INFO, sizeof(REQ_CELL_INFO));
                s_subscribed = true;
                ESP_LOGI(TAG,"0x96 sent -- stream active");
            }
        } else if(s_subscribed && s_conn_handle != BLE_HS_CONN_HANDLE_NONE){
            vTaskDelay(pdMS_TO_TICKS(s_poll_ms));
            if(s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_val_handle != 0)
                ble_gattc_write_no_rsp_flat(s_conn_handle, s_val_handle,
                                            REQ_CELL_INFO, sizeof(REQ_CELL_INFO));
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));   /* idle: wait for connection      */
        }
    }
}

/* ---- MAC string parser ---------------------------------------------------- */
static int parse_mac(const char *s, uint8_t mac_be[6], ble_addr_t *addr)
{
    unsigned b[6];
    if(sscanf(s,"%x:%x:%x:%x:%x:%x",&b[0],&b[1],&b[2],&b[3],&b[4],&b[5])!=6)
        return -1;
    for(int i=0;i<6;i++) mac_be[i]=(uint8_t)b[i];
    /* Pre-populate as PUBLIC; scan will overwrite the type if it differs */
    addr->type = BLE_ADDR_PUBLIC;
    for(int i=0;i<6;i++) addr->val[i]=(uint8_t)b[5-i]; /* NimBLE LE order   */
    return 0;
}

/* ---- public API ---------------------------------------------------------- */
esp_err_t jk_bms_start(const jk_bms_config_t *cfg)
{
    if(!cfg||!cfg->mac||!cfg->on_update) return ESP_ERR_INVALID_ARG;
    if(parse_mac(cfg->mac, s_peer_mac, &s_peer)!=0) return ESP_ERR_INVALID_ARG;
    s_cb      = cfg->on_update;
    s_ctx     = cfg->user_ctx;
    s_poll_ms = cfg->poll_ms ? cfg->poll_ms : 2000;

    esp_err_t err = nimble_port_init();
    if(err!=ESP_OK){ESP_LOGE(TAG,"nimble_port_init: %d",err);return err;}

    ble_hs_cfg.reset_cb        = on_reset;
    ble_hs_cfg.sync_cb         = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();

    nimble_port_freertos_init(host_task);
    xTaskCreate(poll_task,"jk_poll",3072,NULL,5,NULL);
    return ESP_OK;
}

bool jk_bms_is_connected(void)
{
    return s_conn_handle!=BLE_HS_CONN_HANDLE_NONE && s_subscribed;
}
