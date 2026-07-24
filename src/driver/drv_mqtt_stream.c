/* ===========================================================================
   drv_mqtt_stream.c  --  grouped BMS + energy + system MQTT streamer

   THE ONLY MQTT PUBLISHER IN THIS FIRMWARE. All legacy OpenBeken publishing
   (BL_ProcessUpdate sensor topics, HA discovery) is removed/compiled out.

   Driven externally in two stages: UART_TCP_MeterTick() (meter loop) only
   MARKS a second as quiet — the 4 idle ticks of the 10 s meter cycle, any
   tick whose meter slot is unset (dummy placement), or every tick when the
   poller is off. The MAIN LOOP then calls MQTTStream_RunPendingTick(),
   which evaluates the item table and sends AT MOST ONE message per marked
   second — so nothing ever publishes from inside the meter code:
   lwIP's MQTT ring is tiny (~256 B); bursting fills it, returns ERR_MEM and
   stalls the stack. One message on a tick where no meter I/O happens can
   never collide with a poll or flood the outbox.

   Policy per item:
     T_FLOAT : publish when |value - last| >= deadband, else on heartbeat.
               NAN means "no data / offline" -> skipped entirely, so a frozen
               meter or a dropped BMS never republishes stale data.
     T_INT   : publish when the integer changes, else on heartbeat.
               INT_MIN means "no data / skip".
     hb_only : ignore change; publish only on the heartbeat (uptime). The
               very first publish is exempt so the topic appears at boot.

   Heartbeat (STREAM_HB_MS): every item republishes at least this often so HA
   marks nothing "unavailable" in long quiet periods. On MQTT reconnect the
   whole set is force-republished (still paced) so retained topics rebuild.

   Energy/controller values come from BL_GetPublishSnapshot() (drv_bl_shared).
   BMS values come from JKBMS_GetData()/jk_bms_is_connected() when built with
   ENABLE_JK_BMS. System values come from the HAL.
   =========================================================================== */

#include "../new_common.h"
#include "drv_mqtt_stream.h"

#if ENABLE_MQTT

#include <string.h>
#include <math.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "drv_bl_shared.h"        /* bl_pub_snapshot_t, BL_GetPublishSnapshot   */
#include "../mqtt/new_mqtt.h"     /* MQTT_PublishMain_*, MQTT_IsReady, flags     */
#include "../hal/hal_wifi.h"      /* HAL_GetWifiStrength, HAL_GetMyIPString       */
#include "../logging/logging.h"

#ifdef ENABLE_JK_BMS
#include "drv_jkbms.h"            /* JKBMS_GetData + (via jk_bms.h) is_connected  */
#endif
#if ENABLE_BLE_THERM
#include "drv_ble_therm.h"        /* BLETherm_Get: two Xiaomi thermometers        */
#endif

/* ---- tuning -------------------------------------------------------------- */
#define STREAM_HB_MS    300000    /* heartbeat: min republish period per item.
                                     Budget note: only ~4 of every 10 s are
                                     quiet ticks (0.4 msg/s ceiling). 45 items
                                     at a 120 s heartbeat would saturate that
                                     ceiling with heartbeats alone; 300 s keeps
                                     the baseline at ~0.15 msg/s and leaves the
                                     rest of the budget for real changes.
                                     Match HA `expire_after` accordingly
                                     (e.g. 660 s = 2*HB + slack).              */
#define STREAM_MIN_ITEM_MS 5000   /* min spacing between sends of the SAME item
                                     (first publish + heartbeat are exempt) so a
                                     jittery analog value can't hog the budget. */

/* Sentinel: a T_INT current value of INT_MIN means "skip this item". */
#define SKIP_I ((double)INT_MIN)

/* ---- item model ---------------------------------------------------------- */
typedef enum { T_FLOAT, T_INT } pub_type_t;

typedef struct {
    const char *name;     /* MQTT channel leaf                                 */
    pub_type_t  type;
    float       thr;      /* T_FLOAT deadband; ignored for T_INT               */
    int         dec;      /* T_FLOAT decimals                                  */
    unsigned char retain; /* 1 -> OBK_PUBLISH_FLAG_RETAIN                       */
    unsigned char hb_only;/* 1 -> publish only on heartbeat (first pub exempt) */
} pub_item_t;

/* ---- item ids (index into the table + cur[]) ----------------------------- */
enum {
    /* grid */
    G_L1_V, G_L2_V, G_L3_V, G_L1_A, G_L2_A, G_L3_A, G_POWER, G_NET,
    G_IMP_TOTAL, G_IMP_LH, G_IMP_TODAY,
    G_EXP_TOTAL, G_EXP_LH, G_EXP_TODAY,
    /* ess / controller */
    E_AC_POWER, E_CHG_MODE, E_DIV_MODE, E_INV1, E_INV2, E_CHG_PWM, E_DIV_ON,
    E_INV_GATED, E_CHG_GATED,
    /* battery 1 (JK-BMS, BLE) */
    B1_CONNECTED, B1_POWER, B1_SOC, B1_VOLT, B1_CUR, B1_MOS_T, B1_TAVG,
    B1_DELTA_MV, B1_CHG_EN, B1_DIS_EN,
    /* BMS AC side (meter slot 5; import = charging) */
    BA_IMP_NOW, BA_IMP_LH, BA_IMP_TODAY,
    BA_EXP_NOW, BA_EXP_LH, BA_EXP_TODAY,
    /* solar */
    S_POWER, S_LH, S_TODAY, S_TOTAL,
#if ENABLE_BLE_THERM
    /* two Xiaomi BLE thermometers */
    TH1_TEMP, TH1_HUM, TH1_BATT,
    TH2_TEMP, TH2_HUM, TH2_BATT,
#endif
    /* system */
    SYS_UPTIME, SYS_RSSI,
    ITEM_COUNT
};

/* Designated initializers bind name<->slot regardless of order. */
static const pub_item_t s_items[ITEM_COUNT] = {
    [G_L1_V]       = { "grid_l1_voltage",       T_FLOAT, 0.1f, 1, 0, 0 },
    [G_L2_V]       = { "grid_l2_voltage",       T_FLOAT, 0.1f, 1, 0, 0 },
    [G_L3_V]       = { "grid_l3_voltage",       T_FLOAT, 0.1f, 1, 0, 0 },
    [G_L1_A]       = { "grid_l1_current",       T_FLOAT, 0.2f, 2, 0, 0 },
    [G_L2_A]       = { "grid_l2_current",       T_FLOAT, 0.2f, 2, 0, 0 },
    [G_L3_A]       = { "grid_l3_current",       T_FLOAT, 0.2f, 2, 0, 0 },
    [G_POWER]      = { "grid_power",            T_FLOAT, 20.0f,0, 0, 0 },
    [G_NET]        = { "net_energy",            T_FLOAT, 5.0f, 0, 0, 0 },
    [G_IMP_TOTAL]  = { "grid_import_total",     T_INT,   0,    0, 1, 0 },
    [G_IMP_LH]     = { "grid_import_lasthour",  T_INT,   0,    0, 1, 0 },
    [G_IMP_TODAY]  = { "grid_import_today",     T_INT,   0,    0, 1, 0 },
    [G_EXP_TOTAL]  = { "grid_export_total",     T_INT,   0,    0, 1, 0 },
    [G_EXP_LH]     = { "grid_export_lasthour",  T_INT,   0,    0, 1, 0 },
    [G_EXP_TODAY]  = { "grid_export_today",     T_INT,   0,    0, 1, 0 },

    [E_AC_POWER]   = { "ess_ac_power",          T_FLOAT, 20.0f,0, 0, 0 },
    [E_CHG_MODE]   = { "ess_charger_mode",      T_INT,   0,    0, 1, 0 },
    [E_DIV_MODE]   = { "ess_divert_mode",       T_INT,   0,    0, 1, 0 },
    [E_INV1]       = { "ess_inverter1_on",      T_INT,   0,    0, 1, 0 },
    [E_INV2]       = { "ess_inverter2_on",      T_INT,   0,    0, 1, 0 },
    [E_CHG_PWM]    = { "ess_charger_pwm",       T_INT,   0,    0, 1, 0 },
    [E_DIV_ON]     = { "ess_divert_on",         T_INT,   0,    0, 1, 0 },
    [E_INV_GATED]  = { "ess_inverter_gated",    T_INT,   0,    0, 1, 0 },
    [E_CHG_GATED]  = { "ess_charger_gated",     T_INT,   0,    0, 1, 0 },

    [B1_CONNECTED] = { "bms1_connected",        T_INT,   0,    0, 1, 0 },
    [B1_POWER]     = { "bms1_power",            T_FLOAT, 20.0f,0, 0, 0 },
    [B1_SOC]       = { "bms1_soc",              T_INT,   0,    0, 1, 0 },
    [B1_VOLT]      = { "bms1_voltage",          T_FLOAT, 0.05f,2, 0, 0 },
    [B1_CUR]       = { "bms1_current",          T_FLOAT, 0.2f, 2, 0, 0 },
    [B1_MOS_T]     = { "bms1_mos_temp",         T_FLOAT, 0.5f, 1, 0, 0 },
    [B1_TAVG]      = { "bms1_temp_avg",         T_FLOAT, 0.5f, 1, 0, 0 },
    [B1_DELTA_MV]  = { "bms1_cell_delta_mv",    T_INT,   0,    0, 0, 0 },
    [B1_CHG_EN]    = { "bms1_charge_enabled",   T_INT,   0,    0, 1, 0 },
    [B1_DIS_EN]    = { "bms1_discharge_enabled",T_INT,   0,    0, 1, 0 },

    [BA_IMP_NOW]   = { "bms_ac_import_now",     T_FLOAT, 20.0f,0, 0, 0 },
    [BA_IMP_LH]    = { "bms_ac_import_lasthour",T_INT,   0,    0, 1, 0 },
    [BA_IMP_TODAY] = { "bms_ac_import_today",   T_INT,   0,    0, 1, 0 },
    [BA_EXP_NOW]   = { "bms_ac_export_now",     T_FLOAT, 20.0f,0, 0, 0 },
    [BA_EXP_LH]    = { "bms_ac_export_lasthour",T_INT,   0,    0, 1, 0 },
    [BA_EXP_TODAY] = { "bms_ac_export_today",   T_INT,   0,    0, 1, 0 },

    [S_POWER]      = { "solar_power",           T_FLOAT, 20.0f,0, 0, 0 },
    [S_LH]         = { "solar_lasthour",        T_INT,   0,    0, 1, 0 },
    [S_TODAY]      = { "solar_today",           T_INT,   0,    0, 1, 0 },
    [S_TOTAL]      = { "solar_total",           T_INT,   0,    0, 1, 0 },

#if ENABLE_BLE_THERM
    [TH1_TEMP]     = { "therm1_temp",           T_FLOAT, 0.2f, 1, 0, 0 },
    [TH1_HUM]      = { "therm1_hum",            T_FLOAT, 1.0f, 0, 0, 0 },
    [TH1_BATT]     = { "therm1_batt",           T_INT,   0,    0, 1, 0 },
    [TH2_TEMP]     = { "therm2_temp",           T_FLOAT, 0.2f, 1, 0, 0 },
    [TH2_HUM]      = { "therm2_hum",            T_FLOAT, 1.0f, 0, 0, 0 },
    [TH2_BATT]     = { "therm2_batt",           T_INT,   0,    0, 1, 0 },
#endif

    [SYS_UPTIME]   = { "sys_uptime",            T_INT,   0,    0, 0, 1 },
    [SYS_RSSI]     = { "sys_rssi",              T_FLOAT, 3.0f, 0, 0, 0 },
};

/* ---- per-item published state ------------------------------------------- */
static double        s_last[ITEM_COUNT];
static unsigned char s_have[ITEM_COUNT];
static TickType_t    s_lastpub[ITEM_COUNT];

/* system IP (string) tracked separately */
static char          s_last_ip[40];
static unsigned char s_have_ip;
static TickType_t    s_lastpub_ip;
static int           s_cursor = 0;  /* round-robin start so no item is starved */

/* ---- gather one full snapshot into cur[] + ipbuf ------------------------- */
static void gather(double *cur, char *ipbuf, int ipbuflen)
{
    bl_pub_snapshot_t e;
    BL_GetPublishSnapshot(&e);

    cur[G_L1_V] = e.grid_l1_v;  cur[G_L2_V] = e.grid_l2_v;  cur[G_L3_V] = e.grid_l3_v;
    cur[G_L1_A] = e.grid_l1_a;  cur[G_L2_A] = e.grid_l2_a;  cur[G_L3_A] = e.grid_l3_a;
    cur[G_POWER] = e.grid_power; cur[G_NET] = e.net_energy;
    cur[G_IMP_TOTAL] = e.grid_import_total_wh;
    cur[G_IMP_LH]    = e.grid_import_lasthour_wh;
    cur[G_IMP_TODAY] = e.grid_import_today_wh;
    cur[G_EXP_TOTAL] = e.grid_export_total_wh;
    cur[G_EXP_LH]    = e.grid_export_lasthour_wh;
    cur[G_EXP_TODAY] = e.grid_export_today_wh;

    cur[E_AC_POWER]  = e.ess_ac_power;    cur[E_CHG_MODE] = e.ess_charger_mode;
    cur[E_DIV_MODE]  = e.ess_divert_mode; cur[E_INV1]     = e.ess_inverter1_on;
    cur[E_INV2]      = e.ess_inverter2_on; cur[E_CHG_PWM] = e.ess_charger_pwm;
    cur[E_DIV_ON]    = e.ess_divert_on;   cur[E_INV_GATED] = e.ess_inverter_gated;
    cur[E_CHG_GATED] = e.ess_charger_gated;

    cur[BA_IMP_NOW]   = e.bms_ac_import_now_w;      /* NAN when slot 5 offline */
    cur[BA_IMP_LH]    = e.bms_ac_import_lasthour_wh;
    cur[BA_IMP_TODAY] = e.bms_ac_import_today_wh;
    cur[BA_EXP_NOW]   = e.bms_ac_export_now_w;
    cur[BA_EXP_LH]    = e.bms_ac_export_lasthour_wh;
    cur[BA_EXP_TODAY] = e.bms_ac_export_today_wh;

    cur[S_POWER] = e.solar_power; cur[S_LH] = e.solar_lasthour_wh;
    cur[S_TODAY] = e.solar_today_wh; cur[S_TOTAL] = e.solar_total_wh;

    /* --- Battery 1 (JK-BMS over BLE) --- */
#ifdef ENABLE_JK_BMS
    {
        jk_bms_data_t b;
        int connected = jk_bms_is_connected();
        cur[B1_CONNECTED] = connected ? 1 : 0;
        if (connected && JKBMS_GetData(&b)) {
            cur[B1_POWER]    = (double)b.total_voltage * (double)b.current;
            cur[B1_SOC]      = b.soc;
            cur[B1_VOLT]     = b.total_voltage;
            cur[B1_CUR]      = b.current;
            cur[B1_MOS_T]    = b.temp_mosfet;
            cur[B1_TAVG]     = (b.temp_1 + b.temp_2) / 2.0f;
            cur[B1_DELTA_MV] = (b.cell_max - b.cell_min) * 1000.0f;
            cur[B1_CHG_EN]   = b.charge_enabled ? 1 : 0;
            cur[B1_DIS_EN]   = b.discharge_enabled ? 1 : 0;
        } else {
            /* down / no data yet: freeze data fields, keep the flag live */
            cur[B1_POWER] = cur[B1_VOLT] = cur[B1_CUR] = NAN;
            cur[B1_MOS_T] = cur[B1_TAVG] = NAN;
            cur[B1_SOC] = cur[B1_DELTA_MV] = SKIP_I;
            cur[B1_CHG_EN] = cur[B1_DIS_EN] = SKIP_I;
        }
    }
#else
    cur[B1_CONNECTED] = 0;
    cur[B1_POWER] = cur[B1_VOLT] = cur[B1_CUR] = NAN;
    cur[B1_MOS_T] = cur[B1_TAVG] = NAN;
    cur[B1_SOC] = cur[B1_DELTA_MV] = SKIP_I;
    cur[B1_CHG_EN] = cur[B1_DIS_EN] = SKIP_I;
#endif

#if ENABLE_BLE_THERM
    /* --- two Xiaomi BLE thermometers (stale/unset -> skip) --- */
    {
        float t, h; int b;
        if (BLETherm_Get(0, &t, &h, &b)) {
            cur[TH1_TEMP] = t; cur[TH1_HUM] = h; cur[TH1_BATT] = b;
        } else {
            cur[TH1_TEMP] = cur[TH1_HUM] = NAN; cur[TH1_BATT] = SKIP_I;
        }
        if (BLETherm_Get(1, &t, &h, &b)) {
            cur[TH2_TEMP] = t; cur[TH2_HUM] = h; cur[TH2_BATT] = b;
        } else {
            cur[TH2_TEMP] = cur[TH2_HUM] = NAN; cur[TH2_BATT] = SKIP_I;
        }
    }
#endif

    /* --- system --- */
    cur[SYS_UPTIME] = g_secondsElapsed;
    cur[SYS_RSSI]   = HAL_GetWifiStrength();
    {
        const char *ip = HAL_GetMyIPString();
        if (ip && ip[0]) { strncpy(ipbuf, ip, ipbuflen - 1); ipbuf[ipbuflen - 1] = 0; }
        else             { ipbuf[0] = 0; }
    }
}

/* ---- publish one item ----------------------------------------------------
   Everything goes QoS 0 (retain flag independent) so we never consume lwIP's
   scarce in-flight QoS-1 slots; a dropped sample self-heals at the next change
   or heartbeat, and retained topics still keep their last value on broker.  */
static OBK_Publish_Result publish_item(int i, double v)
{
    const pub_item_t *it = &s_items[i];
    int flags = OBK_PUBLISH_FLAG_QOS_ZERO | (it->retain ? OBK_PUBLISH_FLAG_RETAIN : 0);
    if (it->type == T_FLOAT)
        return MQTT_PublishMain_StringFloat(it->name, (float)v, it->dec, flags);
    return MQTT_PublishMain_StringInt(it->name, (int)v, flags);
}

/* ---- evaluate the table and publish AT MOST ONE due item ----------------- */
static void eval_and_publish(void)
{
    static double cur[ITEM_COUNT];
    static char   ipbuf[40];
    TickType_t now = xTaskGetTickCount();
    int n;

    gather(cur, ipbuf, (int)sizeof(ipbuf));

    for (n = 0; n < ITEM_COUNT; n++) {
        int i = (s_cursor + n) % ITEM_COUNT;
        const pub_item_t *it = &s_items[i];
        double v = cur[i];
        int changed, hb, due;

        if (it->type == T_FLOAT) {
            if (isnan(v)) continue;                       /* offline / no data  */
            changed = !s_have[i] || fabs(v - s_last[i]) >= it->thr;
        } else {
            if ((int)v == INT_MIN) continue;              /* no data / skip     */
            changed = !s_have[i] || (int)v != (int)s_last[i];
        }

        hb  = (int)((now - s_lastpub[i]) >= pdMS_TO_TICKS(STREAM_HB_MS));
        /* hb_only items are still allowed a FIRST publish so the topic
           appears right after boot instead of one heartbeat later. */
        due = it->hb_only ? (hb || !s_have[i]) : (changed || hb);
        if (!due) continue;

        /* Per-item min spacing (first publish + heartbeat exempt). */
        if (s_have[i] && !hb &&
            (now - s_lastpub[i]) < pdMS_TO_TICKS(STREAM_MIN_ITEM_MS))
            continue;

        if (publish_item(i, v) != OBK_PUBLISH_OK) {
            s_cursor = i;                 /* retry this one first next tick */
            return;
        }
        s_last[i] = v; s_have[i] = 1; s_lastpub[i] = now;
        s_cursor = (i + 1) % ITEM_COUNT;  /* budget = 1 message per quiet tick */
        return;
    }
    s_cursor = 0;   /* completed a full pass with nothing due */

    /* system IP (string) — only reached on ticks where nothing else was due */
    if (ipbuf[0]) {
        int changed = !s_have_ip || strncmp(ipbuf, s_last_ip, sizeof(s_last_ip)) != 0;
        int hb = (int)((now - s_lastpub_ip) >= pdMS_TO_TICKS(STREAM_HB_MS));
        if ((changed || hb) &&
            MQTT_PublishMain_StringString("sys_ip", ipbuf,
                OBK_PUBLISH_FLAG_QOS_ZERO | OBK_PUBLISH_FLAG_RETAIN) == OBK_PUBLISH_OK) {
            strncpy(s_last_ip, ipbuf, sizeof(s_last_ip) - 1);
            s_last_ip[sizeof(s_last_ip) - 1] = 0;
            s_have_ip = 1; s_lastpub_ip = now;
        }
    }
}

/* ---- external entry points ------------------------------------------------ */

/* Set by the meter loop when the current second involved no meter I/O
   (idle tick, unset slot, or poller off); consumed by the main loop. */
static volatile unsigned char s_quiet_pending = 0;

/* Runtime enable switch — set/cleared by the driver framework (see
   MQTTStream_Start / MQTTStream_Stop below). */
static volatile unsigned char s_enabled = 0;

void MQTTStream_MarkQuietTick(void)
{
    s_quiet_pending = 1;
}

/* Called once per second from the MAIN LOOP (after Main_OnEverySecond).
   Publishes at most ONE message, and only if this second was marked quiet. */
void MQTTStream_RunPendingTick(void)
{
    static int was_ready = 0;
    int ready;

    if (!s_enabled) { s_quiet_pending = 0; return; }
    if (!s_quiet_pending) return;
    s_quiet_pending = 0;

    ready = MQTT_IsReady();
    if (ready) {
        if (!was_ready) {
            /* MQTT (re)connected: force a full republish so retained topics
               and HA state are rebuilt from a clean slate (still paced at
               one message per quiet tick). */
            int i;
            for (i = 0; i < ITEM_COUNT; i++) { s_have[i] = 0; s_lastpub[i] = 0; }
            s_have_ip = 0; s_lastpub_ip = 0;
            s_cursor = 0;
            ADDLOG_INFO(LOG_FEATURE_GENERAL, "MQTT stream: link up, republishing (paced)");
        }
        eval_and_publish();
    }
    was_ready = ready;
}

/* This is an OpenBeken driver (start with `startDriver MQTTStream`, stop
   with `stopDriver MQTTStream`, both also in the web UI). Stopping is
   instant and total: RunPendingTick bails before touching MQTT. Start is
   re-armable and resets all state, so a restart behaves like a fresh
   boot (full paced republish). */
void MQTTStream_Start(void)
{
    memset(s_have, 0, sizeof(s_have));
    memset((void*)s_lastpub, 0, sizeof(s_lastpub));
    s_have_ip = 0; s_lastpub_ip = 0; s_cursor = 0;
    s_quiet_pending = 0;
    s_enabled = 1;
    ADDLOG_INFO(LOG_FEATURE_GENERAL,
                "MQTT stream: armed (%d items, driven by quiet meter ticks, heartbeat %d ms)",
                ITEM_COUNT, STREAM_HB_MS);
}

void MQTTStream_Stop(void)
{
    s_enabled = 0;
    s_quiet_pending = 0;
    ADDLOG_INFO(LOG_FEATURE_GENERAL, "MQTT stream: stopped");
}

#else  /* !ENABLE_MQTT */

void MQTTStream_Start(void) { }
void MQTTStream_Stop(void) { }
void MQTTStream_MarkQuietTick(void) { }
void MQTTStream_RunPendingTick(void) { }

#endif /* ENABLE_MQTT */
