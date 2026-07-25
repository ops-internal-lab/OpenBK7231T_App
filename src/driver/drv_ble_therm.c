/* ===========================================================================
   drv_ble_therm.c -- passive BLE listener for two Xiaomi LYWSD03MMC
   thermometers running ATC1441 or PVVX custom firmware.

   These firmwares broadcast temperature/humidity/battery inside the BLE
   advertisement itself (Service Data, 16-bit UUID 0x181A), so we NEVER
   connect -- we only keep a passive scan running and pick out frames whose
   source MAC matches one of the two configured sensors. Both payload
   layouts are supported and auto-detected by length:

     ATC1441 (13 bytes): MAC[6] tempBE(0.1C) hum%(u8) batt%(u8) battmV cnt
     PVVX    (15 bytes): MAC[6] tempLE(0.01C) humLE(0.01%) battmV batt% cnt fl

   We match on the advertisement's source address, so the in-payload MAC is
   ignored -- works identically for both formats.

   COEXISTENCE WITH JK-BMS (same NimBLE stack): NimBLE cannot start an
   outgoing connection while a discovery is running. jk_bms.c therefore
   calls BLETherm_SuspendScan() immediately before every ble_gap_connect()
   attempt (we cancel the scan synchronously) and BLETherm_ResumeScan()
   when the attempt resolves either way. The scan restarts from our next
   one-second driver tick. Thermometers advertise every few seconds, so
   the short gaps around connect attempts lose nothing.

   The NimBLE stack itself is brought up by the JK-BMS module; this driver
   only uses it (ble_hs_synced() gates the scan start).

   OpenBeken driver "BLETherm": `startDriver BLETherm` / `stopDriver
   BLETherm`. MACs: `setThermMac 1 a4:c1:38:xx:yy:zz` (index 1 or 2,
   persisted in NVS; empty/0 clears). `listThermMacs` shows config + last
   readings + age.
   =========================================================================== */

#include "../new_common.h"
#include "drv_ble_therm.h"

#if ENABLE_BLE_THERM

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../logging/logging.h"
#include "../cmnds/cmd_local.h"
#include "../cmnds/cmd_public.h"   /* commandResult_t, CMD_RES_* */

#include "sdkconfig.h"           /* CONFIG_BT_NIMBLE_EXT_ADV must be visible
                                     HERE: the ext-vs-legacy scan choice below
                                     is a preprocessor decision in this file.  */
#include "nvs_flash.h"
#include "nvs.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"

#define THERM_COUNT       2
#define THERM_FRESH_SECS  1800    /* reading older than this = "no data" (30 min;
                                     generous for outdoor sensors that drop out) */

extern int g_secondsElapsed;

/* ---- state ---------------------------------------------------------------- */
static uint8_t  s_mac[THERM_COUNT][6];       /* NimBLE order: LSB first        */
static uint8_t  s_mac_set[THERM_COUNT];

static volatile float    s_temp[THERM_COUNT];
static volatile float    s_hum[THERM_COUNT];
static volatile int      s_batt[THERM_COUNT];
static volatile uint32_t s_seen[THERM_COUNT];        /* g_secondsElapsed stamp */
static volatile uint32_t s_ivl_q8[THERM_COUNT];      /* interval EWMA, x256 fixed */
static volatile int8_t   s_rssi[THERM_COUNT];        /* last frame RSSI, dBm      */

static volatile unsigned char s_enabled   = 0;       /* driver started         */
static volatile unsigned char s_suspended = 0;       /* jk_bms is connecting   */
static volatile unsigned char s_scanning  = 0;

/* ---- MAC helpers ----------------------------------------------------------
   Human "a4:c1:38:aa:bb:cc" is MSB-first; NimBLE addr.val[] is LSB-first.  */
static int parse_mac(const char *s, uint8_t out_lsb_first[6])
{
    unsigned b[6];
    if (!s) return 0;
    while (*s == ' ') s++;
    if (sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
        return 0;
    for (int i = 0; i < 6; i++) out_lsb_first[i] = (uint8_t)b[5 - i];
    return 1;
}

static void fmt_mac(const uint8_t *lsb_first, char *out, int outlen)
{
    snprintf(out, outlen, "%02x:%02x:%02x:%02x:%02x:%02x",
             lsb_first[5], lsb_first[4], lsb_first[3],
             lsb_first[2], lsb_first[1], lsb_first[0]);
}

/* ---- NVS ------------------------------------------------------------------ */
static const char *s_nvs_keys[THERM_COUNT] = { "thermmac1", "thermmac2" };

static void therm_nvs_load(void)
{
    nvs_handle_t h = 0;
    nvs_open("config", NVS_READONLY, &h);
    for (int i = 0; i < THERM_COUNT; i++) {
        char buf[24]; size_t len = sizeof(buf);
        s_mac_set[i] = 0;
        if (nvs_get_str(h, s_nvs_keys[i], buf, &len) == ESP_OK &&
            parse_mac(buf, s_mac[i]))
            s_mac_set[i] = 1;
    }
    nvs_close(h);
}

static void therm_nvs_save(int idx)
{
    char buf[24];
    nvs_handle_t h = 0;
    esp_err_t e = nvs_open("config", NVS_READWRITE, &h);
    if (e != ESP_OK) {
        ADDLOG_ERROR(LOG_FEATURE_DRV, "BLETherm: nvs_open failed (%d), MAC NOT persisted", (int)e);
        return;
    }
    if (s_mac_set[idx]) {
        fmt_mac(s_mac[idx], buf, sizeof(buf));
        e = nvs_set_str(h, s_nvs_keys[idx], buf);
    } else {
        e = nvs_erase_key(h, s_nvs_keys[idx]);
        if (e == ESP_ERR_NVS_NOT_FOUND) e = ESP_OK;   /* clearing an unset key is fine */
    }
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e != ESP_OK)
        ADDLOG_ERROR(LOG_FEATURE_DRV, "BLETherm: nvs write failed (%d), MAC NOT persisted", (int)e);
}

/* ---- advertisement parsing ------------------------------------------------ */
static void parse_svcdata_181a(int idx, const uint8_t *p, int len)
{
    /* p points at the service-data payload AFTER the 0x181A UUID. */
    float t, rh; int batt;
    if (len == 13) {                       /* ATC1441: big-endian             */
        int16_t raw = (int16_t)(((uint16_t)p[6] << 8) | p[7]);
        t    = raw / 10.0f;
        rh   = p[8];
        batt = p[9];
    } else if (len == 15) {                /* PVVX: little-endian             */
        int16_t raw = (int16_t)((uint16_t)p[6] | ((uint16_t)p[7] << 8));
        t    = raw / 100.0f;
        rh   = ((uint16_t)p[8] | ((uint16_t)p[9] << 8)) / 100.0f;
        batt = p[12];
    } else {
        return;                            /* unknown layout                  */
    }
    if (t < -50.0f || t > 100.0f || rh < 0.0f || rh > 100.0f)
        return;                            /* corrupt frame                   */
    s_temp[idx] = t;
    s_hum[idx]  = rh;
    s_batt[idx] = batt;
    /* Interval EWMA: seconds since the previous frame, averaged alpha=1/64
       (~1/50) in x256 fixed-point. Seeded on the second frame (needs a prior
       timestamp). Diagnostic for placement / packet loss: settles near the
       sensor's true advertising period and rises if frames are being missed. */
    {
        uint32_t now = (uint32_t)g_secondsElapsed;
        if (s_seen[idx] != 0 && now >= s_seen[idx]) {
            uint32_t gap_q8 = (now - s_seen[idx]) << 8;
            if (s_ivl_q8[idx] == 0) s_ivl_q8[idx] = gap_q8;
            else s_ivl_q8[idx] += ((int)gap_q8 - (int)s_ivl_q8[idx]) >> 6;
        }
    }
    s_seen[idx] = (uint32_t)g_secondsElapsed;
}

static void parse_adv(int idx, const uint8_t *data, int len)
{
    /* Walk the AD TLVs: [len][type][payload...] */
    int i = 0;
    while (i + 1 < len) {
        int flen = data[i];
        if (flen < 1 || i + 1 + flen > len) return;
        if (data[i + 1] == 0x16 && flen >= 3 &&
            data[i + 2] == 0x1A && data[i + 3] == 0x18) {
            parse_svcdata_181a(idx, &data[i + 4], flen - 3);
            return;
        }
        i += 1 + flen;
    }
}

/* ---- scan ----------------------------------------------------------------- */
static int therm_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        for (int i = 0; i < THERM_COUNT; i++) {
            if (s_mac_set[i] &&
                memcmp(event->disc.addr.val, s_mac[i], 6) == 0) {
                s_rssi[i] = (int8_t)event->disc.rssi;
                parse_adv(i, event->disc.data, event->disc.length_data);
                break;
            }
        }
        return 0;
#if defined(CONFIG_BT_NIMBLE_EXT_ADV)
    case BLE_GAP_EVENT_EXT_DISC:
        /* Extended report: covers BOTH legacy 1M frames and Coded-PHY
           (long range) frames when the extended scan is active. */
        for (int i = 0; i < THERM_COUNT; i++) {
            if (s_mac_set[i] &&
                memcmp(event->ext_disc.addr.val, s_mac[i], 6) == 0) {
                s_rssi[i] = (int8_t)event->ext_disc.rssi;
                parse_adv(i, event->ext_disc.data, event->ext_disc.length_data);
                break;
            }
        }
        return 0;
#endif
    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_scanning = 0;                    /* cancelled or ended: tick restarts */
        return 0;
    default:
        return 0;
    }
}

static void therm_try_start_scan(void)
{
    uint8_t own_addr_type = 0;

    if (!ble_hs_synced()) return;          /* NimBLE (jk_bms) not up yet       */
    if (ble_gap_disc_active()) { s_scanning = 1; return; }

    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) return;

#if defined(CONFIG_BT_NIMBLE_EXT_ADV)
    /* EXTENDED dual-PHY scan: listen on the normal 1M PHY *and* on Coded
       PHY ("Long Range", S=8) at the same time. A PVVX thermometer set to
       long-range mode lands in the same callback as a normal one -- no
       per-sensor configuration needed on our side. Duty kept low (25%) so
       the BMS connection sees minimal radio contention. */
    {
        struct ble_gap_ext_disc_params u, c;
        memset(&u, 0, sizeof(u)); memset(&c, 0, sizeof(c));
        u.passive = 1; u.itvl = 0x0060; u.window = 0x0018;   /* 1M PHY    */
        c.passive = 1; c.itvl = 0x0060; c.window = 0x0018;   /* Coded PHY */
        if (ble_gap_ext_disc(own_addr_type,
                             0 /* forever */, 0 /* no period */,
                             0 /* keep duplicates: we want every frame */,
                             0 /* no filter policy */, 0 /* not limited */,
                             &u, &c, therm_gap_event, NULL) == 0) {
            s_scanning = 1;
        }
    }
#else
#warning "BLETherm: CONFIG_BT_NIMBLE_EXT_ADV not set - LEGACY 1M-only scan, long-range (Coded PHY) sensors will NOT be received"
    /* Legacy 1M-only scan (build without CONFIG_BT_NIMBLE_EXT_ADV; no
       long-range reception in this mode). */
    {
        struct ble_gap_disc_params p;
        memset(&p, 0, sizeof(p));
        p.passive           = 1;           /* listen only, never scan-request  */
        p.itvl              = 0x0060;      /* one window every 96*0.625 = 60ms */
        p.window            = 0x0018;      /* 15 ms of it -> 25% radio duty    */
        p.filter_duplicates = 0;           /* we WANT every periodic frame     */
        if (ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &p,
                         therm_gap_event, NULL) == 0) {
            s_scanning = 1;
        }
    }
#endif
    /* On failure (e.g. a connect just started) we simply retry next tick. */
}

/* ---- coordination hooks (called from jk_bms.c) ---------------------------- */
void BLETherm_SuspendScan(void)
{
    s_suspended = 1;
    if (s_scanning) {
        ble_gap_disc_cancel();             /* synchronous; frees the LL for    */
        s_scanning = 0;                    /* jk's ble_gap_connect()           */
    }
}

void BLETherm_ResumeScan(void)
{
    s_suspended = 0;                       /* next driver tick restarts scan   */
}

/* ---- data accessor (used by the MQTT streamer) ---------------------------- */
int BLETherm_Get(int idx, float *temp_c, float *hum_pct, int *batt_pct)
{
    if (idx < 0 || idx >= THERM_COUNT) return 0;
    if (!s_enabled || !s_mac_set[idx]) return 0;
    if (s_seen[idx] == 0) return 0;
    if ((uint32_t)g_secondsElapsed - s_seen[idx] > THERM_FRESH_SECS) return 0;
    if (temp_c)   *temp_c   = s_temp[idx];
    if (hum_pct)  *hum_pct  = s_hum[idx];
    if (batt_pct) *batt_pct = s_batt[idx];
    return 1;
}

int BLETherm_GetMacStr(int idx, char *out, int outlen)
{
    if (outlen > 0) out[0] = 0;
    if (idx < 0 || idx >= THERM_COUNT || !s_mac_set[idx]) return 0;
    fmt_mac(s_mac[idx], out, outlen);
    return 1;
}

/* Diagnostics for the config UI. secs_since_seen = age of last frame (or -1 if
   never / not configured); avg_interval_s = EWMA of inter-arrival gap (0 until
   two frames seen). rssi via BLETherm_Get already. Returns 1 if configured. */
int BLETherm_GetStats(int idx, int *secs_since_seen, int *avg_interval_s, int *rssi_dbm)
{
    if (idx < 0 || idx >= THERM_COUNT || !s_mac_set[idx]) {
        if (secs_since_seen) *secs_since_seen = -1;
        if (avg_interval_s)  *avg_interval_s  = 0;
        if (rssi_dbm)        *rssi_dbm        = 0;
        return 0;
    }
    if (secs_since_seen)
        *secs_since_seen = s_seen[idx] ? (int)((uint32_t)g_secondsElapsed - s_seen[idx]) : -1;
    if (avg_interval_s)
        *avg_interval_s = (int)(s_ivl_q8[idx] >> 8);
    if (rssi_dbm)
        *rssi_dbm = (int)s_rssi[idx];
    return 1;
}

void BLETherm_ResetIntervalStats(void)
{
    int i;
    for (i = 0; i < THERM_COUNT; i++) s_ivl_q8[i] = 0;
}

/* ---- console commands -----------------------------------------------------
   NOTE: this fork's commandHandler_t is
     commandResult_t fn(const void*, const char*, const char*, int flags)
   and the dispatcher treats the return as a result code (0 = CMD_RES_OK).
   Returning anything else makes the console report the command as failed
   even when it ran. */
static commandResult_t cmd_set_therm_mac(const void *c, const char *cmd, const char *args, int flags)
{
    int idx; const char *p = args;
    (void)c; (void)cmd; (void)flags;
    while (p && *p == ' ') p++;
    if (!p || !*p) { ADDLOG_INFO(LOG_FEATURE_DRV, "usage: setThermMac 1|2 aa:bb:cc:dd:ee:ff (or 0 to clear)"); return CMD_RES_NOT_ENOUGH_ARGUMENTS; }
    idx = atoi(p) - 1;
    if (idx < 0 || idx >= THERM_COUNT) { ADDLOG_INFO(LOG_FEATURE_DRV, "setThermMac: index must be 1 or 2"); return CMD_RES_BAD_ARGUMENT; }
    while (*p && *p != ' ') p++;
    while (*p == ' ') p++;
    /* clear ONLY on a bare "0" argument -- a real MAC may begin with 0x0.. */
    if (*p == 0 || (p[0] == '0' && (p[1] == 0 || p[1] == ' '))) {
        s_mac_set[idx] = 0; s_seen[idx] = 0;
        therm_nvs_save(idx);
        ADDLOG_INFO(LOG_FEATURE_DRV, "Thermometer %d cleared", idx + 1);
        return CMD_RES_OK;
    }
    if (!parse_mac(p, s_mac[idx])) {
        ADDLOG_INFO(LOG_FEATURE_DRV, "setThermMac: bad MAC '%s'", p);
        return CMD_RES_BAD_ARGUMENT;
    }
    s_mac_set[idx] = 1; s_seen[idx] = 0;
    therm_nvs_save(idx);
    ADDLOG_INFO(LOG_FEATURE_DRV, "Thermometer %d MAC set to %s", idx + 1, p);
    return CMD_RES_OK;
}

static commandResult_t cmd_list_therm(const void *c, const char *cmd, const char *args, int flags)
{
    (void)c; (void)cmd; (void)args; (void)flags;
    for (int i = 0; i < THERM_COUNT; i++) {
        if (!s_mac_set[i]) {
            ADDLOG_INFO(LOG_FEATURE_DRV, "therm%d: unset", i + 1);
        } else {
            char m[24]; fmt_mac(s_mac[i], m, sizeof(m));
            if (s_seen[i])
                ADDLOG_INFO(LOG_FEATURE_DRV,
                    "therm%d: %s  %.1fC %.0f%% batt=%d%% age=%us",
                    i + 1, m, s_temp[i], s_hum[i], s_batt[i],
                    (unsigned)((uint32_t)g_secondsElapsed - s_seen[i]));
            else
                ADDLOG_INFO(LOG_FEATURE_DRV, "therm%d: %s  (no frame yet)", i + 1, m);
        }
    }
    ADDLOG_INFO(LOG_FEATURE_DRV, "scan: enabled=%d suspended=%d active=%d mode=%s",
                s_enabled, s_suspended, s_scanning,
#if defined(CONFIG_BT_NIMBLE_EXT_ADV)
                "EXT dual-PHY (long range OK)"
#else
                "LEGACY 1M ONLY (no long range!)"
#endif
                );
    return CMD_RES_OK;
}

/* ---- driver entry points -------------------------------------------------- */
void BLETherm_OnEverySecond(void)
{
    if (!s_enabled || s_suspended || s_scanning) return;
    therm_try_start_scan();
}

void BLETherm_Start(void)
{
    static int cmds_registered = 0;
    if (!cmds_registered) {
        cmds_registered = 1;
        CMD_RegisterCommand("setThermMac",   cmd_set_therm_mac, "setThermMac 1|2 aa:bb:.. (0 clears)");
        CMD_RegisterCommand("listThermMacs", cmd_list_therm,    "List BLE thermometer config + readings");
    }
    therm_nvs_load();
    for (int i = 0; i < THERM_COUNT; i++) s_seen[i] = 0;
    s_enabled = 1;
#if defined(CONFIG_BT_NIMBLE_EXT_ADV)
    ADDLOG_INFO(LOG_FEATURE_DRV, "BLETherm: armed (%s%s) scan=EXTENDED dual-PHY (1M+LongRange)",
                s_mac_set[0] ? "mac1 " : "", s_mac_set[1] ? "mac2" : "");
#else
    ADDLOG_ERROR(LOG_FEATURE_DRV, "BLETherm: armed (%s%s) scan=LEGACY 1M ONLY - long-range sensors will NOT be received!",
                s_mac_set[0] ? "mac1 " : "", s_mac_set[1] ? "mac2" : "");
#endif
}

void BLETherm_Stop(void)
{
    s_enabled = 0;
    if (s_scanning) { ble_gap_disc_cancel(); s_scanning = 0; }
    ADDLOG_INFO(LOG_FEATURE_DRV, "BLETherm: stopped");
}

#else  /* !ENABLE_BLE_THERM */

void BLETherm_Start(void) { }
void BLETherm_Stop(void) { }
void BLETherm_OnEverySecond(void) { }
void BLETherm_SuspendScan(void) { }
void BLETherm_ResumeScan(void) { }
int  BLETherm_Get(int idx, float *t, float *h, int *b) { (void)idx; (void)t; (void)h; (void)b; return 0; }
int  BLETherm_GetStats(int idx, int *s, int *a, int *r) { (void)idx; if(s)*s=-1; if(a)*a=0; if(r)*r=0; return 0; }
void BLETherm_ResetIntervalStats(void) { }

#endif /* ENABLE_BLE_THERM */
