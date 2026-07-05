/* ===========================================================================
   drv_mqtt_stream.c  --  grouped BMS + energy + system MQTT streamer

   One 10 s task evaluates a table of items and publishes each to MQTT under
   the device base topic (<base>/<name>/get). Policy per item:

     T_FLOAT : publish when |value - last| >= deadband, else on heartbeat.
               A NAN value means "no data / offline" -> skipped entirely, so
               a frozen meter or a dropped BMS never republishes stale data.
     T_INT   : publish when the integer changes, else on heartbeat.
               A value of INT_MIN means "placeholder / skip" (Battery 2).
     hb_only : ignore change; publish only on the heartbeat (uptime).

   Heartbeat (STREAM_HB_MS): every item republishes at least this often so HA
   marks nothing "unavailable" during long quiet periods. On MQTT reconnect the
   whole set is force-republished so retained topics are rebuilt.

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

/* ---- tuning -------------------------------------------------------------- */
#define STREAM_TICK_MS    1000    /* one evaluation per second                   */
#define STREAM_HB_MS    120000    /* heartbeat: min republish period per item    */
#define STREAM_MAX_PER_TICK   1   /* send at most ONE message per tick -> one per
                                     second maximum. lwIP's MQTT ring is tiny
                                     (~256 B); bursting fills it, returns ERR_MEM
                                     and stalls the stack. OpenBeken paces its own
                                     channels the same way (one per second).      */
#define STREAM_MIN_ITEM_MS 5000   /* min spacing between sends of the SAME item
                                     (first publish + heartbeat are exempt) so a
                                     jittery analog value can't hog the one-per-
                                     second budget.                               */

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
    unsigned char hb_only;/* 1 -> publish only on heartbeat                    */
} pub_item_t;

/* ---- item ids (index into the table + cur[]) ----------------------------- */
enum {
    /* grid */
    G_L1_V, G_L2_V, G_L3_V, G_POWER, G_NET,
    G_IMP_TOTAL, G_EXP_TOTAL, G_IMP_LH, G_IMP_TODAY, G_EXP_LH, G_EXP_TODAY,
    /* battery 1 (live) */
    B1_CONNECTED, B1_POWER, B1_SOC, B1_VOLT, B1_CUR, B1_MOS_T, B1_TAVG,
    B1_DELTA_MV, B1_CHG_EN, B1_DIS_EN,
    /* battery 2 (placeholder) */
    B2_CONNECTED, B2_POWER, B2_SOC, B2_VOLT, B2_CUR, B2_MOS_T, B2_TAVG,
    B2_DELTA_MV, B2_CHG_EN, B2_DIS_EN,
    /* solar */
    S_POWER, S_LH, S_TODAY, S_TOTAL,
    /* ess / controller */
    E_AC_POWER, E_CHG_MODE, E_DIV_MODE, E_INV1, E_INV2, E_CHG_PWM, E_DIV_ON,
    E_INV_GATED, E_CHG_GATED,
    /* system */
    SYS_UPTIME, SYS_RSSI,
    ITEM_COUNT
};

/* Designated initializers bind name<->slot regardless of order. */
static const pub_item_t s_items[ITEM_COUNT] = {
    [G_L1_V]       = { "grid_l1_voltage",       T_FLOAT, 0.1f, 1, 0, 0 },
    [G_L2_V]       = { "grid_l2_voltage",       T_FLOAT, 0.1f, 1, 0, 0 },
    [G_L3_V]       = { "grid_l3_voltage",       T_FLOAT, 0.1f, 1, 0, 0 },
    [G_POWER]      = { "grid_power",            T_FLOAT, 20.0f,0, 0, 0 },
    [G_NET]        = { "net_energy",            T_FLOAT, 5.0f, 0, 0, 0 },
    [G_IMP_TOTAL]  = { "grid_import_total",     T_INT,   0,    0, 1, 0 },
    [G_EXP_TOTAL]  = { "grid_export_total",     T_INT,   0,    0, 1, 0 },
    [G_IMP_LH]     = { "grid_import_lasthour",  T_INT,   0,    0, 1, 0 },
    [G_IMP_TODAY]  = { "grid_import_today",     T_INT,   0,    0, 1, 0 },
    [G_EXP_LH]     = { "grid_export_lasthour",  T_INT,   0,    0, 1, 0 },
    [G_EXP_TODAY]  = { "grid_export_today",     T_INT,   0,    0, 1, 0 },

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

    [B2_CONNECTED] = { "bms2_connected",        T_INT,   0,    0, 1, 0 },
    [B2_POWER]     = { "bms2_power",            T_FLOAT, 20.0f,0, 0, 0 },
    [B2_SOC]       = { "bms2_soc",              T_INT,   0,    0, 1, 0 },
    [B2_VOLT]      = { "bms2_voltage",          T_FLOAT, 0.05f,2, 0, 0 },
    [B2_CUR]       = { "bms2_current",          T_FLOAT, 0.2f, 2, 0, 0 },
    [B2_MOS_T]     = { "bms2_mos_temp",         T_FLOAT, 0.5f, 1, 0, 0 },
    [B2_TAVG]      = { "bms2_temp_avg",         T_FLOAT, 0.5f, 1, 0, 0 },
    [B2_DELTA_MV]  = { "bms2_cell_delta_mv",    T_INT,   0,    0, 0, 0 },
    [B2_CHG_EN]    = { "bms2_charge_enabled",   T_INT,   0,    0, 1, 0 },
    [B2_DIS_EN]    = { "bms2_discharge_enabled",T_INT,   0,    0, 1, 0 },

    [S_POWER]      = { "solar_power",           T_FLOAT, 20.0f,0, 0, 0 },
    [S_LH]         = { "solar_lasthour",        T_INT,   0,    0, 1, 0 },
    [S_TODAY]      = { "solar_today",           T_INT,   0,    0, 1, 0 },
    [S_TOTAL]      = { "solar_total",           T_INT,   0,    0, 1, 0 },

    [E_AC_POWER]   = { "ess_ac_power",          T_FLOAT, 20.0f,0, 0, 0 },
    [E_CHG_MODE]   = { "ess_charger_mode",      T_INT,   0,    0, 1, 0 },
    [E_DIV_MODE]   = { "ess_divert_mode",       T_INT,   0,    0, 1, 0 },
    [E_INV1]       = { "ess_inverter1_on",      T_INT,   0,    0, 1, 0 },
    [E_INV2]       = { "ess_inverter2_on",      T_INT,   0,    0, 1, 0 },
    [E_CHG_PWM]    = { "ess_charger_pwm",        T_INT,   0,    0, 1, 0 },
    [E_DIV_ON]     = { "ess_divert_on",         T_INT,   0,    0, 1, 0 },
    [E_INV_GATED]  = { "ess_inverter_gated",    T_INT,   0,    0, 1, 0 },
    [E_CHG_GATED]  = { "ess_charger_gated",     T_INT,   0,    0, 1, 0 },

    [SYS_UPTIME]   = { "sys_uptime",            T_INT,   0,    0, 0, 1 },
    [SYS_RSSI]     = { "sys_rssi",              T_FLOAT, 3.0f, 0, 0, 0 },
};

/* ---- per-item published state ------------------------------------------- */
static double     s_last[ITEM_COUNT];
static unsigned char s_have[ITEM_COUNT];
static TickType_t s_lastpub[ITEM_COUNT];

/* system IP (string) tracked separately */
static char       s_last_ip[40];
static unsigned char s_have_ip;
static int        s_cursor = 0;   /* round-robin start so no item is starved */
static TickType_t s_lastpub_ip;

/* ---- gather one full snapshot into cur[] + ipbuf ------------------------- */
static void gather(double *cur, char *ipbuf, int ipbuflen)
{
    bl_pub_snapshot_t e;
    BL_GetPublishSnapshot(&e);

    cur[G_L1_V] = e.grid_l1_v;  cur[G_L2_V] = e.grid_l2_v;  cur[G_L3_V] = e.grid_l3_v;
    cur[G_POWER] = e.grid_power; cur[G_NET] = e.net_energy;
    cur[G_IMP_TOTAL] = e.grid_import_total_wh;
    cur[G_EXP_TOTAL] = e.grid_export_total_wh;
    cur[G_IMP_LH]    = e.grid_import_lasthour_wh;
    cur[G_IMP_TODAY] = e.grid_import_today_wh;
    cur[G_EXP_LH]    = e.grid_export_lasthour_wh;
    cur[G_EXP_TODAY] = e.grid_export_today_wh;

    cur[S_POWER] = e.solar_power; cur[S_LH] = e.solar_lasthour_wh;
    cur[S_TODAY] = e.solar_today_wh; cur[S_TOTAL] = e.solar_total_wh;

    cur[E_AC_POWER]  = e.ess_ac_power;   cur[E_CHG_MODE] = e.ess_charger_mode;
    cur[E_DIV_MODE]  = e.ess_divert_mode; cur[E_INV1]    = e.ess_inverter1_on;
    cur[E_INV2]      = e.ess_inverter2_on; cur[E_CHG_PWM] = e.ess_charger_pwm;
    cur[E_DIV_ON]    = e.ess_divert_on;  cur[E_INV_GATED] = e.ess_inverter_gated;
    cur[E_CHG_GATED] = e.ess_charger_gated;

    /* --- Battery 1 (live) --- */
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

    /* --- Battery 2 (placeholder: no live source yet) ---
       connected=0 publishes once (retained) so HA shows it offline; every
       data field is a skip sentinel, so nothing else is ever published. */
    cur[B2_CONNECTED] = 0;
    cur[B2_POWER] = cur[B2_VOLT] = cur[B2_CUR] = NAN;
    cur[B2_MOS_T] = cur[B2_TAVG] = NAN;
    cur[B2_SOC] = cur[B2_DELTA_MV] = SKIP_I;
    cur[B2_CHG_EN] = cur[B2_DIS_EN] = SKIP_I;

    /* --- system --- */
    cur[SYS_UPTIME] = g_secondsElapsed;
    cur[SYS_RSSI]   = HAL_GetWifiStrength();
    {
        const char *ip = HAL_GetMyIPString();
        if (ip && ip[0]) { strncpy(ipbuf, ip, ipbuflen - 1); ipbuf[ipbuflen - 1] = 0; }
        else             { ipbuf[0] = 0; }
    }
}

/* ---- evaluate the table and publish what is due ------------------------- */
/* Publish one item. Everything goes QoS 0 (retain flag independent) so we
   never consume lwIP's scarce in-flight QoS-1 slots; a dropped sample self-
   heals at the next change or heartbeat, and retained topics still keep their
   last value on the broker. */
static OBK_Publish_Result publish_item(int i, double v)
{
    const pub_item_t *it = &s_items[i];
    int flags = OBK_PUBLISH_FLAG_QOS_ZERO | (it->retain ? OBK_PUBLISH_FLAG_RETAIN : 0);
    if (it->type == T_FLOAT)
        return MQTT_PublishMain_StringFloat(it->name, (float)v, it->dec, flags);
    return MQTT_PublishMain_StringInt(it->name, (int)v, flags);
}

static void eval_and_publish(void)
{
    static double cur[ITEM_COUNT];
    static char   ipbuf[40];
    TickType_t now = xTaskGetTickCount();
    int n, sent = 0;

    gather(cur, ipbuf, (int)sizeof(ipbuf));

    /* Evaluate every item for change each tick, but SEND at most
       STREAM_MAX_PER_TICK of them, starting from a rotating cursor so nothing
       is starved. Stop the tick on the first publish failure so lwIP's outbox
       can drain before we push more — this is what prevents the burst that was
       freezing MQTT. */
    for (n = 0; n < ITEM_COUNT; n++) {
        int i = (s_cursor + n) % ITEM_COUNT;
        const pub_item_t *it = &s_items[i];
        double v = cur[i];
        int changed, hb, due;

        if (it->type == T_FLOAT) {
            if (isnan(v)) continue;                       /* offline / no data  */
            changed = !s_have[i] || fabs(v - s_last[i]) >= it->thr;
        } else {
            if ((int)v == INT_MIN) continue;              /* placeholder / skip */
            changed = !s_have[i] || (int)v != (int)s_last[i];
        }

        hb  = (int)((now - s_lastpub[i]) >= pdMS_TO_TICKS(STREAM_HB_MS));
        due = it->hb_only ? hb : (changed || hb);
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

        if (++sent >= STREAM_MAX_PER_TICK) {
            s_cursor = (i + 1) % ITEM_COUNT;
            return;
        }
    }
    s_cursor = 0;   /* completed a full pass with budget to spare */

    /* system IP (string) — shares the same per-tick budget */
    if (sent < STREAM_MAX_PER_TICK && ipbuf[0]) {
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

/* ---- task ---------------------------------------------------------------- */
static void stream_task(void *arg)
{
    int was_ready = 0;
    (void)arg;

    for (;;) {
        int ready = MQTT_IsReady();
        if (ready) {
            if (!was_ready) {
                /* MQTT (re)connected: force a full republish so retained
                   topics and HA state are rebuilt from a clean slate. */
                int i;
                for (i = 0; i < ITEM_COUNT; i++) { s_have[i] = 0; s_lastpub[i] = 0; }
                s_have_ip = 0; s_lastpub_ip = 0;
                s_cursor = 0;
                ADDLOG_INFO(LOG_FEATURE_GENERAL, "MQTT stream: link up, republishing (paced)");
            }
            eval_and_publish();
        }
        was_ready = ready;
        vTaskDelay(pdMS_TO_TICKS(STREAM_TICK_MS));
    }
}

void MQTTStream_Start(void)
{
    static int started = 0;
    if (started) return;
    started = 1;
    xTaskCreate(stream_task, "mqtt_stream", 4096, NULL, 4, NULL);
    ADDLOG_INFO(LOG_FEATURE_GENERAL, "MQTT stream: started (tick %d ms, heartbeat %d ms)",
                STREAM_TICK_MS, STREAM_HB_MS);
}

#else  /* !ENABLE_MQTT */

void MQTTStream_Start(void) { }

#endif /* ENABLE_MQTT */
