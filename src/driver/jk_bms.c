/* ===========================================================================
   jk_bms.c  --  JK-BMS BLE driver for ESP-IDF (NimBLE GATT client, read-only)
   
   MERGED VERSION:
     - Includes passive scan to resolve real BLE address type.
     - Includes explicit MTU exchange before discovery.
     - Correctly splits the 0xFFE1 characteristics into separate notify and 
       write handles (fixing the "Two Handles" problem).
     - Incorporates the 300ms gap between DEVICE_INFO and CELL_INFO requests 
       (fixing the timing drop issue).
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

static uint8_t    s_peer_mac[6];
static ble_addr_t s_peer;
static uint8_t    s_own_addr_type;

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_svc_start;
static uint16_t s_svc_end;
static uint16_t s_notify_handle; /* 0xFFE1 char with NOTIFY property */
static uint16_t s_write_handle;  /* 0xFFE1 char with WRITE property  */
static uint16_t s_cccd_handle;   /* 0x2902 descriptor handle         */

static volatile bool s_connecting   = false;
static volatile bool s_subscribed   = false;
static volatile bool s_kick_pending = false;

static float s_balance_start = 0.0f;

/* ---- decode helpers ------------------------------------------------------ */
static inline uint16_t u16(const uint8_t *d, int i){ return d[i]|(d[i+1]<<8); }
static inline uint32_t u32(const uint8_t *d, int i){
    return (uint32_t)d[i]|((uint32_t)d[i+1]<<8)|
           ((uint32_t)d[i+2]<<16)|((uint32_t)d[i+3]<<24);
}
static inline int16_t s16(const uint8_t *d, int i){ return (int16_t)u16(d,i); }
static inline int32_t s32(const uint8_t *d, int i){ return (int32_t)u32(d,i); }

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

static bool addr_mac_match(const uint8_t addr_val[6], const uint8_t mac_be[6])
{
    for(int i=0;i<6;i++)
        if(addr_val[i] != mac_be[5-i]) return false;
    return true;
}

/* ==========================================================================
   GATT callback chain
   ========================================================================== */

static int on_subscribe(uint16_t conn, const struct ble_gatt_error *err,
                        struct ble_gatt_attr *attr, void *arg)
{
    (void)attr; (void)arg;
    if(err->status != 0){
        ESP_LOGW(TAG,"CCCD write failed status=%d -- disconnecting",err->status);
        ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    
    /* Write to the correct WRITE handle */
    ble_gattc_write_no_rsp_flat(conn, s_write_handle,
                                REQ_DEVICE_INFO, sizeof(REQ_DEVICE_INFO));
    s_kick_pending = true;
    ESP_LOGI(TAG,"CCCD OK; 0x97 sent (beep expected); 0x96 follows in 300ms");
    return 0;
}

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
            ESP_LOGW(TAG,"no CCCD found -- trying direct kick anyway");
            /* fallback for clones */
            ble_gattc_write_no_rsp_flat(conn, s_write_handle, REQ_DEVICE_INFO, sizeof(REQ_DEVICE_INFO));
            s_kick_pending = true;
            return 0;
        }
        uint8_t val[2] = {0x01, 0x00};
        int rc = ble_gattc_write_flat(conn, s_cccd_handle, val, sizeof(val),
                                      on_subscribe, NULL);
        if(rc != 0){
            ESP_LOGE(TAG,"CCCD write_flat rc=%d", rc);
            ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
    return 0;
}

static int on_disc_chr(uint16_t conn, const struct ble_gatt_error *err,
                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    if(err->status == 0){
        if(ble_uuid_cmp(&chr->uuid.u, &CHR_UUID.u) == 0){
            uint8_t props = chr->properties;
            ESP_LOGI(TAG,"0xFFE1 candidate val_handle=0x%04x props=0x%02x",
                     chr->val_handle, props);
            if(!s_notify_handle && (props & (BLE_GATT_CHR_PROP_NOTIFY|BLE_GATT_CHR_PROP_INDICATE)))
                s_notify_handle = chr->val_handle;
            if(!s_write_handle  && (props & (BLE_GATT_CHR_PROP_WRITE|BLE_GATT_CHR_PROP_WRITE_NO_RSP)))
                s_write_handle  = chr->val_handle;
        }
    } else if(err->status == BLE_HS_EDONE){
        ESP_LOGI(TAG,"chr discovery done: notify=0x%04x write=0x%04x",
                 s_notify_handle, s_write_handle);
                 
        if(!s_notify_handle || !s_write_handle){
            uint16_t both = s_notify_handle ? s_notify_handle : s_write_handle;
            if(both){
                ESP_LOGW(TAG,"single 0xFFE1 char -- using for both notify and write");
                s_notify_handle = both;
                s_write_handle  = both;
            } else {
                ESP_LOGW(TAG,"0xFFE1 not found; disconnecting");
                ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
                return 0;
            }
        }
        
        s_cccd_handle = 0;
        /* Find CCCD only on the NOTIFY characteristic */
        int rc = ble_gattc_disc_all_dscs(conn, s_notify_handle + 1, s_svc_end,
                                         on_disc_dsc, NULL);
        if(rc != 0){
            ESP_LOGE(TAG,"disc_all_dscs rc=%d", rc);
            ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
    return 0;
}

static int on_disc_svc(uint16_t conn, const struct ble_gatt_error *err,
                       const struct ble_gatt_svc *svc, void *arg)
{
    (void)arg;
    if(err->status == 0){
        if(ble_uuid_cmp(&svc->uuid.u, &SVC_UUID.u) == 0){
            s_svc_start = svc->start_handle;
            s_svc_end   = svc->end_handle;
            ESP_LOGI(TAG,"0xFFE0 start=0x%04x end=0x%04x", s_svc_start, s_svc_end);
        }
    } else if(err->status == BLE_HS_EDONE){
        if(s_svc_end == 0){
            ESP_LOGW(TAG,"0xFFE0 not found -- is this a JK BMS?");
            ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        s_notify_handle = 0; s_write_handle = 0;
        int rc = ble_gattc_disc_all_chrs(conn, s_svc_start, s_svc_end,
                                         on_disc_chr, NULL);
        if(rc != 0){
            ESP_LOGE(TAG,"disc_all_chrs rc=%d", rc);
            ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
    return 0;
}

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

    case BLE_GAP_EVENT_DISC: {
        if(!addr_mac_match(event->disc.addr.val, s_peer_mac)) return 0;
        s_peer = event->disc.addr;
        ESP_LOGI(TAG,"BMS found: addr_type=%d RSSI=%d -- connecting",
                 s_peer.type, event->disc.rssi);
        s_connecting = true;
        ble_gap_disc_cancel();
        start_connect();
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        if(!s_connecting && s_conn_handle == BLE_HS_CONN_HANDLE_NONE){
            ESP_LOGW(TAG,"scan complete: BMS not found -- retrying");
            start_scan();
        }
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        s_connecting = false;
        if(event->connect.status == 0){
            s_conn_handle = event->connect.conn_handle;
            s_svc_start = 0; s_svc_end = 0; 
            s_notify_handle = 0; s_write_handle = 0;
            ESP_LOGI(TAG,"connected handle=%d -- requesting MTU 247", s_conn_handle);
            ble_att_set_preferred_mtu(247);
            int rc = ble_gattc_exchange_mtu(s_conn_handle, on_mtu, NULL);
            if(rc != 0){
                ESP_LOGW(TAG,"exchange_mtu rc=%d -- starting discovery",rc);
                rc = ble_gattc_disc_all_svcs(s_conn_handle, on_disc_svc, NULL);
                if(rc != 0){
                    ESP_LOGE(TAG,"disc_all_svcs rc=%d", rc);
                    ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                }
            }
        } else {
            ESP_LOGW(TAG,"connect failed status=%d -- retrying scan", event->connect.status);
            start_scan();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG,"disconnected reason=%d -- retrying scan", event->disconnect.reason);
        s_conn_handle  = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed   = false;
        s_kick_pending = false;
        s_connecting   = false;
        s_len          = 0;
        s_svc_start = 0; s_svc_end = 0; 
        s_notify_handle = 0; s_write_handle = 0;
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
    dp.passive           = 1;
    dp.filter_duplicates = 1;
    dp.itvl              = 160;
    dp.window            = 80;

    int rc = ble_gap_disc(s_own_addr_type, 10000, &dp, gap_event, NULL);
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

/* ---- polling task -------------------------------------------------------- */
static void poll_task(void *p)
{
    (void)p;
    for(;;){
        if(s_kick_pending){
            /* Wait 300ms between DEVICE_INFO and CELL_INFO requests */
            vTaskDelay(pdMS_TO_TICKS(300));
            s_kick_pending = false;
            if(s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_write_handle != 0){
                ble_gattc_write_no_rsp_flat(s_conn_handle, s_write_handle,
                                            REQ_CELL_INFO, sizeof(REQ_CELL_INFO));
                s_subscribed = true;
                ESP_LOGI(TAG,"0x96 sent -- stream active");
            }
        } else if(s_subscribed && s_conn_handle != BLE_HS_CONN_HANDLE_NONE){
            vTaskDelay(pdMS_TO_TICKS(s_poll_ms));
            if(s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_write_handle != 0){
                ble_gattc_write_no_rsp_flat(s_conn_handle, s_write_handle,
                                            REQ_CELL_INFO, sizeof(REQ_CELL_INFO));
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
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
    addr->type = BLE_ADDR_PUBLIC;
    for(int i=0;i<6;i++) addr->val[i]=(uint8_t)b[5-i];
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
