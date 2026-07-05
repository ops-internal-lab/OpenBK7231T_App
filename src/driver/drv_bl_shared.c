// Internal code ONLY

#include <stdlib.h>   // atof, abs
#include <stdio.h>    // snprintf
#include <string.h>   // memset, memcpy, strlen
#include <stdint.h>   // int16_t / int64_t (per-meter slot store + tick acc)
#if PLATFORM_ESPIDF
// ESP-IDF does not transitively pull FreeRTOS in via the local headers below,
// and TickType_t / xTaskGetTickCount are used before those includes (line ~167).
// Pull them in up front so the type is defined at first use.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif
#include "dash_frontend.h"

// Charger C mapping constants
#define CHARGER_MIN_PWM   10       // lowest useful duty for the supply
#define CHARGER_MAX_PWM  100

// Set to 48 slots (12-hour circular buffer to match the new 12-hour graph)
#define MATRIX_SIZE 48

static int consumption_matrix[MATRIX_SIZE] = {0}; 
static int export_matrix[MATRIX_SIZE] = {0};

// Full-precision net Wh per 15-minute period (consumption - export,
// including decimals - this is period_net, not a truncated int).
// Used for OBK_CONSUMPTION_LAST_HOUR and any other internal accounting
// that needs accuracy. Sanity-clamped to +/-9999.99, not the graph's
// display range.
static float net_matrix[MATRIX_SIZE] = {0};

// Graph-only: net_matrix values capped to -150..+300 Wh and pre-packed
// as the (val+150)/2 byte the dashboard's "net" graph expects. Kept
// separate from net_matrix so OBK_CONSUMPTION_LAST_HOUR (and anything
// else reading net_matrix) sees the true, uncapped net Wh for the
// period - only the display copy is capped/scaled. Radio payloads stay
// small (1 byte/sample) while internal calculations keep full accuracy.
static unsigned char net_graph_matrix[MATRIX_SIZE] = {0};

// --- Graph series for the two-panel dashboard chart, one value per 15-min slot ---
// TOP panel: average BATTERY power over the period, signed (+ = charge,
//   - = discharge), in W, clamped to +/-500. Served as 10-bit sign+magnitude
//   (2 bytes/slot) and split into charge/discharge traces by the browser.
// BOTTOM panel: SOLAR energy generated over the period, in Wh magnitude,
//   clamped 0..150. Always drawn as a negative (downward) yellow overlay.
static int           ess_pwr_matrix[MATRIX_SIZE]    = {0};   // avg battery W, signed
static unsigned char solar_graph_matrix[MATRIX_SIZE] = {0};  // solar Wh/period, 0..150

// Per-period sample accumulators (sampled by the 30 s sampler below). Battery
// power is signed; solar power is the (>=0) instantaneous generation in W.
static int current_ess_pwr_accum   = 0;   // sum of signed battery W samples
static int current_solar_pwr_accum = 0;   // sum of solar W samples (>=0)
static int sample_count_30s        = 0;

int solar_available = 0;

// Charger/inverter PWM state (0 = idle, 5 = inverter on, 18+ = charger on
// at that %). File-scoped (not local to the 30s control block) so the
// 15-minute rollover can save/restore them across its reset, preventing
// the inverter from cycling off at every 15-minute boundary.
static int persistent_state = 0;
static int solar_excess = 0;
static int saved_persistent_state = 0;
static int saved_solar_excess = 0;
static int rollover_just_happened = 0;

/* Daily export (generation) totals — loaded from NVS on boot, rolled over
   at midnight via NTP. Index: [0]=today, [1]=yesterday, [2]=2d ago, [3]=3d ago */
static float export_daily[4] = {0.0f};

int estimated_energy_period = 0;

// NEW GLOBAL TARGETS
static int target_export       = 20;   // export low/OFF point (Wh), global (auto+manual)
static int target_power_auto   = 100;  // AUTO  : ceiling the loop regulates up to (%)
static int target_power_manual = 100;  // MANUAL: actual charger output (%)

// Charger AUTO/MANUAL mode. charger_c_auto stays the master "is auto" flag so the
// existing control logic keeps working; charger_manual_temp marks the temporary
// (purple) manual that auto-reverts to AUTO at the next 15-minute rollover:
//   charger_c_auto==1                       -> AUTO            (blue)
//   charger_c_auto==0 && charger_manual_temp -> MANUAL temp     (purple, reverts)
//   charger_c_auto==0 && !charger_manual_temp-> MANUAL locked   (red)
static int charger_manual_temp = 0;

// Diversion (.22 load) overlay. Works the same in AUTO and MANUAL:
//   divert_user 0 = auto (follow hysteresis), 1 = force-on temp (reverts at rollover),
//               2 = force-on locked.
static int          divert_user          = 0;
static int          divert_is_on         = 0;   // last state commanded to the .22 load
static int          divert_threshold     = 60;  // ON point (Wh), kept >= target_export+10
static int          charger_was_running  = 0;
// charger_on_tick (TickType_t) is declared after the FreeRTOS headers below.

#define dump_load_relay_number 6
/* charger_c_ip removed — charger IPs now configured via setChargerIP1/2 */
#define net_metering_period 15

static int dump_load_relay[dump_load_relay_number] = {0};

static int last_matrix_index = -1; 
int charger_c_auto = 1;

// ====================================================================
// LOCAL GPIO / PWM ACTUATION  (ESP32-C3 hardware — replaces SendGet)
// ====================================================================
#define GPIO_CHARGER_ENABLE     4        // charger enable (digital output, active HIGH)
#define GPIO_CHARGER_PWM        2        // charger duty   (LEDC PWM, 8-bit, 1 kHz)
#define GPIO_RELAY_ECON         0        // relay economiser (LEDC PWM, 8-bit)
#define GPIO_INVERTER_LED       8        // onboard LED mirrors GPIO0 (inverted, active LOW)

#define LEDC_CH_CHARGER         4        // LEDC channel for GPIO2
#define LEDC_CH_RELAY           5        // LEDC channel for GPIO0
#define LEDC_CH_LED             3        // LEDC channel for GPIO8 (inverted)
#define LEDC_TIMER_ACTUATION    1        // LEDC timer index (0 may be used by OBK)
#define LEDC_FREQ_HZ_ACT        1000
#define LEDC_RES_ACT            LEDC_TIMER_8_BIT

#define RELAY_ECON_DUTY_FULL    255      // 100 % — initial relay pull-in
#define RELAY_ECON_DUTY_HOLD    204      // ~80 % — economiser hold (204/255 ≈ 80.4 %)
#define RELAY_ECON_PULSE_MS     500      // ms at 100 % before dropping to hold

// Shadow variables — last duty written to each output (available for debug/MQTT)
static int charger_pwm      = 0;   // 0-255, mirrors GPIO2
static int relay_economiser = 0;   // 0 / RELAY_ECON_DUTY_HOLD / RELAY_ECON_DUTY_FULL, mirrors GPIO0

// Economiser edge-detection state (persistent across ApplyDumpLoadGPIO calls)
static int          inverter_was_active  = 0;

// ---- BMS voltage gating of charger / inverter ----
// Per-CELL setpoints (volts). Charger cuts OFF when the MAX cell reaches its
// setpoint and resumes 50 mV below it; inverter cuts OFF when the MIN cell
// reaches its setpoint and resumes 100 mV above it. Gating is skipped entirely
// when the BMS is offline (JKBMS_GetData() == false) — we won't act on stale
// cell voltages. Setpoints come from the bat-pop sliders (SetChargerCutoff /
// SetInverterCutoff, centivolts) and persist on change.
static float charger_cutoff_v  = 3.60f;  // max-cell charge stop
static float inverter_cutoff_v = 3.30f;  // min-cell discharge stop
static int   charger_gated     = 0;      // 1 = charger latched off by BMS
static int   inverter_gated    = 0;      // 1 = inverter latched off by BMS
#define CHARGER_HYST_V   0.050f          // 50 mV
#define INVERTER_HYST_V  0.100f          // 100 mV

// inverter_engage_tick declared as static local inside ApplyDumpLoadGPIO —
// TickType_t is only available after the FreeRTOS headers are pulled in
// by the OpenBK include block below, so it cannot live here at file scope.

// ---- Dashboard "System Configuration" settings ----
// Set from the settings tab via the Set* commands below, read back via
// /api_dash?req=cfg, and flash-persisted on change (see SETTINGS_Save).
//
// Meter slaves are addressed by last octet only (full IP = device subnet +
// octet, resolved when the poller opens the socket). 0 = unset / skip. The
// six slots map fixed to their dashboard roles:
//   g_meter_ip[0] = L1   (consumption)   g_meter_ip[3] = Solar A (generation)
//   g_meter_ip[1] = L2   (consumption)   g_meter_ip[4] = Solar B (generation)
//   g_meter_ip[2] = L3   (consumption)   g_meter_ip[5] = ESS Inverter (bidir)
static unsigned char g_meter_ip[6]  = {0,0,0,0,0,0};
// Per-meter direction inversion. Some BL0942 slaves get wired backwards, so the
// signed WATT and the signed CF-CNT energy delta come out with the wrong sign.
// A 1 here flips BOTH for that slot (applied in the BL0942 driver, so display
// and energy stay consistent). Set from the meter settings page (SetMeterInvert),
// persisted to flash on Apply (keys minv0..minv5).
static unsigned char g_meter_invert[6] = {0,0,0,0,0,0};
// Per-meter calibration coefficients, same divide-mode convention as the
// onboard sensor's PwrCal (calibrated = raw / coefficient). Defaulted to the
// SAME nominal BL0942 datasheet constants the shared onboard calibration
// starts from (DEFAULT_VOLTAGE_CAL/CURRENT_CAL/POWER_CAL in drv_bl0942.c) —
// NOT 1.0, which would leave an uncalibrated slot displaying raw ADC codes.
// Set via SetMeterVoltCal/SetMeterCurrentCal/SetMeterPowerCal <slot 1-6>
// <true value>; persisted to flash on change (keys mvcal0..5/macal0..5/
// mpcal0..5). "Calibrated" (for the settings-page green/grey state) simply
// means the coefficient no longer equals its default.
#define METER_CAL_V_DEFAULT 15188.0f
#define METER_CAL_A_DEFAULT 251210.0f
#define METER_CAL_P_DEFAULT 598.0f
static float g_meter_vcal[6] = {METER_CAL_V_DEFAULT, METER_CAL_V_DEFAULT, METER_CAL_V_DEFAULT,
                                 METER_CAL_V_DEFAULT, METER_CAL_V_DEFAULT, METER_CAL_V_DEFAULT};
static float g_meter_acal[6] = {METER_CAL_A_DEFAULT, METER_CAL_A_DEFAULT, METER_CAL_A_DEFAULT,
                                 METER_CAL_A_DEFAULT, METER_CAL_A_DEFAULT, METER_CAL_A_DEFAULT};
static float g_meter_pcal[6] = {METER_CAL_P_DEFAULT, METER_CAL_P_DEFAULT, METER_CAL_P_DEFAULT,
                                 METER_CAL_P_DEFAULT, METER_CAL_P_DEFAULT, METER_CAL_P_DEFAULT};
// Latest RAW (pre-calibration) chip codes per slot, captured every read so a
// calibration command always has "what the chip said just now" to compute
// coefficient = raw / true_value against. Same idea as drv_pwrCal.c's
// latest_raw_voltage/current/power, just one triple per meter instead of one
// shared triple.
static uint32_t g_meter_raw_v[6] = {0};
static uint32_t g_meter_raw_a[6] = {0};
static int32_t  g_meter_raw_w[6] = {0};
static char          g_bms_mac[18]  = {0};   // "AA:BB:CC:DD:EE:FF"
static char          g_bms2_mac[18] = {0};
static unsigned char g_inv2_ip      = 0;     // Boost Inverter (remote), last octet
static unsigned char g_bypass_ip    = 0;     // Diversion Load (remote), last octet
static int           g_boost_power   = 10;   // Boost net-energy trigger (Wh)
static int           g_inv2_on       = 0;    // Boost Inverter desired state
static void SETTINGS_Save(void);             // defined after the NVS includes below
static void COUNTERS_Save(void);             // defined after the NVS includes below

// ---- Remote meter slots (filled once per read by the BL0942 TCP poller) ----
// Slot roles match g_meter_ip[]: [0..2]=L1/L2/L3 (bidirectional grid phases,
// signed power: +import / -export), [3..4]=Solar A/B, [5]=ESS (bidir).
// online = 1 once a good reading has ever landed (stays 1 across brief read
// failures so we can hold the last-good value); last_ok = tick of the most
// recent successful read. Display/integration freshness is derived from the
// age of last_ok, not from online alone (see BL_MeterOnlineState).
// cf_wh  = signed net Wh for THIS sweep, from the chip's CF-CNT delta
//          (+ = import/charge/generation-direction, - = the opposite).
// cf_valid = 1 when cf_wh holds a trustworthy delta this cycle, 0 otherwise
//          (first read after (re)connect, detected chip reset, or a glitch).
typedef struct { float v, a, w, freq; int online; TickType_t last_ok;
                 float cf_wh; int64_t cf_ticks; int cf_valid; } meter_slot_t;
static meter_slot_t g_meter[6];

// Read freshness windows. A slot is polled ~every 6 s, so within 9 s a good
// read is "fresh"; up to 30 s we keep showing/integrating the last-good value
// but flag it as "stale" (comms hiccup); beyond 30 s it's treated as offline.
#define METER_FRESH_TICKS ((TickType_t)( 9000 / portTICK_PERIOD_MS))
#define METER_HOLD_TICKS  ((TickType_t)(60000 / portTICK_PERIOD_MS))

// =====================================================================
// PER-METER / PER-GROUP ENERGY STORE
// =====================================================================
// Two layers (see the detailed comments at the declarations below):
//   meter_acc[6]  — per-METER lifetime ticks, diagnostic reference only.
//   g_grp[3]      — per-GROUP (grid/solar/battery) accounting store: 96-slot
//                   group-net days (today + 3), plus lifetime gross buckets.
// Net metering is applied once per 15-min interval, per group; everything
// above the slot is gross (import/export tracked independently, forever).
#define DAY_SLOTS 96
#define N_METERS  6

// ---- Per-METER reference store (diagnostic only) ----------------------
// meter_acc[6] = signed lifetime CF-CNT ticks per physical meter. "What has
// this meter seen, ever." Never used for billing/display accounting — that's
// the per-GROUP store below. Persisted (keys macc0..5), shown on the meter
// settings page. Kept for our own reference.
static int64_t meter_acc[N_METERS] = {0,0,0,0,0,0};

// ---- Per-GROUP accounting store (the real numbers) --------------------
// Net metering is applied ONCE, at the 15-min boundary, PER GROUP: the phases
// (and A/B solar) are combined into a single signed group-net for the interval
// (this is the net_metering value the charger loop uses), then that one value
// is frozen into the group's slot. Above the slot everything is GROSS: a slot's
// own sign decides import vs export, and imports never cancel a different
// slot's export. Groups: 0=grid (L1+L2+L3), 1=solar (A+B, one-way), 2=battery.
#define N_GROUPS 3
#define GRP_GRID  0
#define GRP_SOLAR 1
#define GRP_BATT  2
typedef struct {
    int16_t slots[DAY_SLOTS];   // today   (group-net ticks per 15-min)
    int16_t day1[DAY_SLOTS];    // yesterday
    int16_t day2[DAY_SLOTS];    // 2 days ago
    int16_t day3[DAY_SLOTS];    // 3 days ago
} group_day_store_t;
static group_day_store_t g_grp[N_GROUPS];

// Live per-GROUP accumulator for the interval in progress (raw signed ticks,
// int32 headroom). Folded into Today and Total on the fly; frozen into the
// current slot and rolled into the lifetime buckets at the boundary.
static int32_t g_cur_ticks[N_GROUPS] = {0};
static int    g_cur_slot = -1;      // today-slot index being filled (0..95)

// ---- Lifetime buckets (totals forever, GROSS, never reset) ------------
// Bumped once per 15-min commit by that interval's group-net, into the sign-
// appropriate bucket. Raw signed-magnitude ticks (always >= 0; each is one
// direction). Persisted as i64 (keys glImp/glExp/slLife/blChg/blDis).
static int64_t life_grid_imp = 0, life_grid_exp = 0;   // grid import / export
static int64_t life_solar    = 0;                      // solar generation
static int64_t life_ess_chg  = 0, life_ess_dis  = 0;   // battery charge / discharge

// Datasheet default tick->Wh factor (no per-meter calibration — that's a
// future job, applied at this one serve-time conversion point). Derived from
// the driver's own scaling: Wh = ticks * 1638.4 * 256 / 3600 / powerCal, at
// the datasheet default powerCal = 598  ->  ~0.19483 Wh/tick.
#define TICKS_TO_WH (1638.4f * 256.0f / 3600.0f / 598.0f)
static int ticks_to_wh(long ticks) {
    float wh = (float)ticks * TICKS_TO_WH;
    return (int)(wh >= 0 ? wh + 0.5f : wh - 0.5f);
}

// Clamp a signed tick sum to the int16 slot range defensively before storing.
static int16_t ticks_to_slot(long v) {
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

// ---- Derive helpers: everything the UI shows is summed from group slots ----
// Pick a group's day array by age: 0=today 1=yesterday 2=2d 3=3d.
static const int16_t *grp_day(int grp, int age) {
    const group_day_store_t *g = &g_grp[grp];
    switch (age) { case 0: return g->slots; case 1: return g->day1;
                   case 2: return g->day2; default: return g->day3; }
}

// GROSS import (positive slots) or export (|negative slots|) for a group/day,
// split PER SLOT — so a slot's import never nets against another slot's export.
// dir: +1 = sum positive slots, -1 = sum |negative slots|.
// For today (age 0) the in-progress interval (g_cur_ticks) is folded in with
// its own running sign, so Today reflects energy up to this instant. Slots up
// to g_cur_slot-1 are complete; the slot at g_cur_slot is still filling, so we
// use the live accumulator for it instead of the (stale/zero) slot.
static long grp_gross(int grp, int age, int dir) {
    const int16_t *d = grp_day(grp, age);
    long s = 0; int i;
    int last = (age == 0)
             ? ((g_cur_slot < 0) ? -1 : g_cur_slot - 1)   // completed slots only
             : DAY_SLOTS - 1;
    for (i = 0; i <= last; i++) {
        long v = d[i];
        if (dir > 0) { if (v > 0) s += v; }
        else         { if (v < 0) s += -v; }
    }
    if (age == 0) {                          // fold in the live interval
        long cur = g_cur_ticks[grp];
        if (dir > 0) { if (cur > 0) s += cur; }
        else         { if (cur < 0) s += -cur; }
    }
    return s;
}

// Today (with current interval folded in).
static long grid_import(int age)  { return grp_gross(GRP_GRID,  age, +1); }
static long grid_export(int age)  { return grp_gross(GRP_GRID,  age, -1); }
static long solar_gen(int age)    { return grp_gross(GRP_SOLAR, age, +1); }   // one-way
static long ess_charge(int age)   { return grp_gross(GRP_BATT,  age, +1); }
static long ess_discharge(int age){ return grp_gross(GRP_BATT,  age, -1); }

// Lifetime TOTAL = permanent bucket + the current in-progress interval, so a
// total never lags a partial interval behind (same rule as Today).
static long grid_import_total(void)  { long c = g_cur_ticks[GRP_GRID];  return (long)life_grid_imp + (c > 0 ?  c : 0); }
static long grid_export_total(void)  { long c = g_cur_ticks[GRP_GRID];  return (long)life_grid_exp + (c < 0 ? -c : 0); }
static long solar_total(void)        { long c = g_cur_ticks[GRP_SOLAR]; return (long)life_solar    + (c > 0 ?  c : 0); }
static long ess_charge_total(void)   { long c = g_cur_ticks[GRP_BATT];  return (long)life_ess_chg  + (c > 0 ?  c : 0); }
static long ess_discharge_total(void){ long c = g_cur_ticks[GRP_BATT];  return (long)life_ess_dis  + (c < 0 ? -c : 0); }

// Last hour = the 4 most-recently-COMPLETED today slots, GROSS per slot.
static long grp_lasthour(int grp, int dir) {
    const int16_t *d = g_grp[grp].slots;
    long s = 0; int k;
    int last = g_cur_slot - 1;
    for (k = 0; k < 4; k++) {
        int idx = last - k;
        if (idx < 0) break;
        long v = d[idx];
        if (dir > 0) { if (v > 0) s += v; }
        else         { if (v < 0) s += -v; }
    }
    return s;
}
static long grid_imp_lh(void)  { return grp_lasthour(GRP_GRID,  +1); }
static long grid_exp_lh(void)  { return grp_lasthour(GRP_GRID,  -1); }
static long solar_lh(void)     { return grp_lasthour(GRP_SOLAR, +1); }
static long ess_chg_lh(void)   { return grp_lasthour(GRP_BATT,  +1); }
static long ess_dis_lh(void)   { return grp_lasthour(GRP_BATT,  -1); }

#include "drv_bl_shared.h"

#include "../new_cfg.h"
#include "../new_pins.h"
#include "../hal/hal_flashVars.h"
#include "../logging/logging.h"
#include "../mqtt/new_mqtt.h"
#include "../hal/hal_ota.h"
#if PLATFORM_ESPIDF
#include "drv_uart_tcp_client.h"
// Drop a slot's CF-CNT baseline so its next read re-baselines (contributes 0)
// instead of bridging a stale gap. Used at the interval boundary for any meter
// that is currently offline (see point-2 offline policy in BL_ProcessUpdate).
void BL0942_InvalidateBaseline(int slot);
#endif
#include "drv_local.h"
#include "drv_ntp.h"
#include "drv_deviceclock.h"   // TIME_* (live device clock) — used instead of stale NTP_*
#include "drv_public.h"
#include "drv_uart.h"
#include "../hal/hal_wifi.h"     // HAL_GetMyIPString (for the .22 diversion target)
#include "../new_common.h"        // g_secondsElapsed (uptime for the SYSTEM panel)
#include "../cmnds/cmd_public.h" //for enum EventCode
#include <math.h>
#include <time.h>
#if PLATFORM_ESPIDF
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#endif

#ifdef ENABLE_JK_BMS
#include "drv_jkbms.h"   // JKBMS_GetData / JKBMS_GetMac
#include "jk_bms.h"      // jk_bms_data_t
#endif

int stat_updatesSkipped = 0;
int stat_updatesSent = 0;

static float net_energy = 0;
static float real_export = 0;
static float real_consumption = 0;

// Variables for the solar dump load timer
static byte old_time = 0;
#define dump_load_hysteresis 1 
#define max_export -3300

byte check_time = 0;                    
byte check_hour = 0;                    
              
const char UNIT_WH[] = "Wh";
struct {
    energySensorNames_t names;
    byte rounding_decimals;
    float changeSendThreshold;
    double lastReading; 
    double lastSentValue; 
    int noChangeFrame; 
} sensors[OBK__NUM_SENSORS] = { 
    {{"voltage",        "V",    "Voltage",                  "voltage",                  "0", },  0,  1,   },            
    {{"current",        "A",    "Current",                  "current",                  "1", },  2,  0.01,},            
    {{"power",          "W",    "Power",                    "power",                    "2", },  0,  10,  },            
    {{"apparent_power", "VA",   "Apparent Power",           "power_apparent",           "9", },  0,  10,  },             
    {{"reactive_power", "Wh",   "Energy Balance",           "power_reactive",           "10",},  0,  1,   },            
    {{"power_factor",   "",     "Power Factor",             "power_factor",             "11",},  1,  0.1, },            
    {{"energy",         UNIT_WH,"Total Consumption",        "energycounter",            "3", },  2,  0.1, },            
    {{"energy",         UNIT_WH,"Total Generation",         "energycounter_generation", "14",},  2,  0.1, },            
    {{"energy",         UNIT_WH,"Energy Last Hour",         "energycounter_last_hour",  "4", },  2,  0.1, },            
    {{"energy",         UNIT_WH,"Energy Today",             "energycounter_today",      "7", },  2,  0.1, },            
    {{"energy",         UNIT_WH,"Energy Yesterday",         "energycounter_yesterday",  "6", },  2,  0.1, },            
    {{"energy",         UNIT_WH,"Energy 2 Days Ago",        "energycounter_2_days_ago", "12",},  2,  0.1, },            
    {{"energy",         UNIT_WH,"Energy 3 Days Ago",        "energycounter_3_days_ago", "13",},  2,  0.1, },            
    {{"timestamp",      "",     "Energy Clear Date",        "energycounter_clear_date", "8", },  0,  86400,},            
}; 

float lastReadingFrequency = NAN;

// Crash-proof float->int conversion. Casting a non-finite (NaN/inf) or
// out-of-range float to int is undefined behaviour on ARM and can fault.
// Any energy/power value that ever goes bad (e.g. a stray meter glitch)
// would otherwise crash at one of the (int) cast sites below. This clamps
// to a wide but safe integer window and maps non-finite values to 0.
static int safe_int(double v) {
    if (!isfinite(v)) return 0;
    if (v >  1000000000.0) return  1000000000;
    if (v < -1000000000.0) return -1000000000;
    return (int)v;
}

// ====================================================================
// DIVERSION (.22 load) CONTROL
// ====================================================================
// Builds "<my-subnet>.22" from the device's own IP and fires a Tasmota
// Power ON/OFF to it. Fired only on state changes (edges) so the .22
// device isn't spammed every loop.
static void divert_send(int on) {
    const char *myip = HAL_GetMyIPString();
    char ip[24];
    char cmd[96];
    char *last_dot;
    if (!myip || !*myip) return;
    strncpy(ip, myip, sizeof(ip) - 1);
    ip[sizeof(ip) - 1] = '\0';
    last_dot = strrchr(ip, '.');
    if (!last_dot) return;
    *last_dot = '\0';                       // ip now holds the /24 prefix
    snprintf(cmd, sizeof(cmd),
             "SendGet http://%s.22/cm?cmnd=Power%%20%s",
             ip, on ? "ON" : "OFF");
    //CMD_ExecuteCommand(cmd, 0);
}

// Evaluated every loop. Hysteresis band: turn ON at >= divert_threshold Wh of
// export, OFF at <= target_export Wh. AUTO mode additionally requires the
// charger to be running and 5 s to have elapsed since it went off->on. A user
// force-on (divert_user 1/2) ignores the charger gate.
static TickType_t charger_on_tick = 0;   // tick the charger last went off->on
static void evaluate_diversion(void) {
    int charger_running = (dump_load_relay[5] >= 12);
    TickType_t now = xTaskGetTickCount();
    int want_on;

    if (charger_running && !charger_was_running) charger_on_tick = now;
    charger_was_running = charger_running;

    if (divert_user >= 1) {
        want_on = 1;                                    // force-on (temp or locked)
    } else if (!charger_running) {
        want_on = 0;                                    // auto needs charger on
    } else if ((now - charger_on_tick) < (5000 / portTICK_PERIOD_MS)) {
        want_on = divert_is_on;                         // 5 s grace after charger start
    } else {
        int export_wh = -estimated_energy_period;       // +ve when exporting
        if (export_wh >= divert_threshold)   want_on = 1;
        else if (export_wh <= target_export) want_on = 0;
        else                                 want_on = divert_is_on; // inside the band
    }

    if (want_on != divert_is_on) {
        divert_is_on = want_on;
        divert_send(want_on);
    }
}

// ====================================================================
// INSTANTANEOUS POWER (derived directly from the Wh delta of the cycle)
// ====================================================================
// Each BL_ProcessUpdate call receives the SIGNED Wh that flowed since the
// last call (from the chip's CF-CNT delta on remote meters). Dividing by the
// elapsed time gives the average wattage over that window — which, because a
// full cycle is a fixed ~10 s, already IS the smoothed value. No extra rolling
// average is applied. Signed: positive = import (consumption), negative =
// export.
static float         calc_power_w     = 0.0f;

// ====================================================================
// LOOP INTERVAL MEASUREMENT
// ====================================================================
// Tracks the wall-clock time between successive BL_ProcessUpdate calls.
// Replaces the old worst-case execution-time metric with a value that
// tells us the actual cadence at which the meter pushes readings.
static TickType_t  last_processupdate_tick = 0;
static unsigned int  loop_interval_ms        = 0;

int actual_mday = -1;
float lastSavedEnergyCounterValue = 0.0f;
float lastSavedGenerationCounterValue = 0.0f;
long ConsumptionSaveCounter = 0;
TickType_t lastConsumptionSaveStamp;
time_t ConsumptionResetTime = 0;

int changeSendAlwaysFrames = 300;
int changeDoNotSendMinFrames = 20;

// ====================================================================
// ENERGY VERSION COUNTER (global)
// ====================================================================
int energy_version = 0;
void mark_energy_dirty(void) { energy_version++; }

// ====================================================================
// MINIMAL BASE64 ENCODER (for compact graph payloads)
// ====================================================================
static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Encodes `len` bytes from `in` into base64 chars written to `out`
// (NOT null-terminated). Returns number of chars written.
// out must have space for ((len + 2) / 3) * 4 bytes.
static int base64_encode(const unsigned char *in, int len, char *out) {
    int i, o = 0;
    for (i = 0; i + 3 <= len; i += 3) {
        unsigned int v = (in[i] << 16) | (in[i+1] << 8) | in[i+2];
        out[o++] = b64_table[(v >> 18) & 0x3F];
        out[o++] = b64_table[(v >> 12) & 0x3F];
        out[o++] = b64_table[(v >> 6)  & 0x3F];
        out[o++] = b64_table[v & 0x3F];
    }
    if (len - i == 1) {
        unsigned int v = in[i] << 16;
        out[o++] = b64_table[(v >> 18) & 0x3F];
        out[o++] = b64_table[(v >> 12) & 0x3F];
        out[o++] = '=';
        out[o++] = '=';
    } else if (len - i == 2) {
        unsigned int v = (in[i] << 16) | (in[i+1] << 8);
        out[o++] = b64_table[(v >> 18) & 0x3F];
        out[o++] = b64_table[(v >> 12) & 0x3F];
        out[o++] = b64_table[(v >> 6)  & 0x3F];
        out[o++] = '=';
    }
    return o;
}

void BL09XX_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState)
{
	(void)bPreState;
    // Dashboard migrated to standalone JSON architecture on /dash
}

void BL09XX_SaveEmeteringStatistics()
{
    ENERGY_METERING_DATA data;
    memset(&data, 0, sizeof(ENERGY_METERING_DATA));

    /* TotalGeneration no longer in the struct — stored as NVS key "eExpTotal" */
    data.TotalConsumption = sensors[OBK_CONSUMPTION_TOTAL].lastReading;
    data.TodayConsumpion = sensors[OBK_CONSUMPTION_TODAY].lastReading;
    data.YesterdayConsumption = sensors[OBK_CONSUMPTION_YESTERDAY].lastReading;
    data.actual_mday = actual_mday;
    data.ConsumptionHistory[0] = sensors[OBK_CONSUMPTION_2_DAYS_AGO].lastReading;
    data.ConsumptionHistory[1] = sensors[OBK_CONSUMPTION_3_DAYS_AGO].lastReading;
    data.ConsumptionResetTime = ConsumptionResetTime;
    ConsumptionSaveCounter++;
    data.save_counter = ConsumptionSaveCounter;

    HAL_SetEnergyMeterStatus(&data);

    /* Export total and daily history stored separately so the struct stays 32 bytes */
    HAL_FlashVars_SaveEnergyExportTotal(sensors[OBK_GENERATION_TOTAL].lastReading);
    HAL_FlashVars_SaveEnergyImportTotal(sensors[OBK_CONSUMPTION_TOTAL].lastReading);
    HAL_FlashVars_SaveEnergyExportDaily(0, export_daily[0]);
    HAL_FlashVars_SaveEnergyExportDaily(1, export_daily[1]);
    HAL_FlashVars_SaveEnergyExportDaily(2, export_daily[2]);
    HAL_FlashVars_SaveEnergyExportDaily(3, export_daily[3]);
}

commandResult_t BL09XX_ResetEnergyCounter(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    float value;
    int i;

    if(args==0||*args==0) 
    {
        sensors[OBK_GENERATION_TOTAL].lastReading = 0.0;
        sensors[OBK_CONSUMPTION_TOTAL].lastReading = 0.0;
        for(i = OBK_CONSUMPTION__DAILY_FIRST; i <= OBK_CONSUMPTION__DAILY_LAST; i++)
        {
            sensors[i].lastReading = 0.0;
        }
    } else {
        value = atof(args);
        sensors[OBK_CONSUMPTION_TOTAL].lastReading = value;
    }
    ConsumptionResetTime = (time_t)TIME_GetCurrentTime();
#if WINDOWS
#elif PLATFORM_BL602
#elif PLATFORM_W600 || PLATFORM_W800
#elif PLATFORM_XR809
#elif PLATFORM_BK7231N || PLATFORM_BK7231T
    if (ota_progress()==-1)
#endif
    { 
        lastSavedEnergyCounterValue = sensors[OBK_CONSUMPTION_TOTAL].lastReading;
        lastSavedGenerationCounterValue = sensors[OBK_GENERATION_TOTAL].lastReading;
        BL09XX_SaveEmeteringStatistics();
        lastConsumptionSaveStamp = xTaskGetTickCount();
    }
    mark_energy_dirty();
    return CMD_RES_OK;
}

// ClearMeteringData — wipe ALL metering counters (grid / solar / ESS totals and
// today, per-meter accumulators, the last-hour rings, the in-progress interval,
// and the 12-hour graph matrices) and persist the zeroed state. Fired from the
// "Clear Metering Data" button on the meter settings page. Meter IPs, MACs,
// invert flags and control settings are NOT touched.
commandResult_t BL09XX_ClearMeteringData(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    int i;

    // Per-meter store: all 4 days x 6 meters, the in-progress interval, and the
    // lifetime tick accumulators.
    memset(g_grp, 0, sizeof(g_grp));
    { int g; for (g = 0; g < N_GROUPS; g++) g_cur_ticks[g] = 0; }
    { int i; for (i = 0; i < N_METERS; i++) meter_acc[i] = 0; }
    life_grid_imp = life_grid_exp = life_solar = life_ess_chg = life_ess_dis = 0;

    // Live control-loop interval accumulators + the 15-min estimate.
    real_consumption = real_export = net_energy = 0;
    estimated_energy_period = 0;

    // 12-hour graph matrices back to a flat zero line.
    for (i = 0; i < MATRIX_SIZE; i++) {
        consumption_matrix[i] = 0;
        export_matrix[i]      = 0;
        net_matrix[i]         = 0;
        net_graph_matrix[i]   = (unsigned char)((0 + 150) / 2);
        ess_pwr_matrix[i]     = 0;
        solar_graph_matrix[i] = 0;
    }

    ConsumptionResetTime = (time_t)TIME_GetCurrentTime();

    // Persist the zeroed state so a reboot doesn't restore old data.
    COUNTERS_Save();                    // per-meter store + tick accumulators
#if PLATFORM_ESPIDF
    HAL_FlashVars_SaveGraphMatrices(net_graph_matrix,
                                    solar_graph_matrix, ess_pwr_matrix,
                                    MATRIX_SIZE,
                                    (last_matrix_index < 0) ? 0 : last_matrix_index,
                                    (unsigned int)TIME_GetCurrentTime());
#endif
    mark_energy_dirty();
    return CMD_RES_OK;
}

// ====================================================================
// ApplyDumpLoadGPIO — single actuation point for all three call sites
// ====================================================================
// state <  3          : everything off  (GPIO4 LOW, GPIO2 duty 0, GPIO0 duty 0)
// state  3..5         : inverter on     (GPIO4 LOW, GPIO2 0,
//                                        GPIO0 100% for 500ms then 80% hold)
//                       Pulse fires only on the 0→active rising edge.
//                       If state remains inverter-active, holds at 80%.
// state  18..100      : charger on      (GPIO4 HIGH, GPIO2 PWM 0-255, GPIO0 0)
//                       Duty scaled linearly: 18→0, 100→255.
//
// charger_pwm and relay_economiser are updated to reflect what was last
// written to hardware (shadow state, useful for diagnostics).
static void ApplyDumpLoadGPIO(int state)
{
#if PLATFORM_ESPIDF
    // Declared static local so TickType_t is resolved after the FreeRTOS
    // headers are included above; retains value between calls like a file-
    // scope static would.
    static TickType_t inverter_engage_tick = 0;

    // ---- BMS voltage gate (skipped entirely if BMS offline) ----
    // Re-evaluates the hysteresis latches from live cell voltages, then forces
    // the requested state off if the relevant device is latched. dump_load_relay
    // is left untouched (the control logic keeps its intent); only the hardware
    // output is held off until the cell voltage recovers past the hysteresis.
#ifdef ENABLE_JK_BMS
    {
        jk_bms_data_t bd;
        // Update the latches ONLY when a fresh frame is available. On comms
        // loss the latches are left exactly as they were, so the last good gate
        // decision is HELD, not released: a charger gated-off at 3.60 V stays
        // off through a dropout instead of glitching back on, and a device that
        // was allowed stays allowed. Resume (release) happens only once fresh
        // data shows the cell voltage recovered past the hysteresis band.
        // No fail-to-off timeout on purpose — given the known nightly BMS
        // desync, forcing the inverter off after a timeout would drop the house
        // load overnight; holding the last state is the safer behaviour.
        if (JKBMS_GetData(&bd)) {
            if (bd.cell_max >= charger_cutoff_v)                        charger_gated  = 1;
            else if (bd.cell_max <= charger_cutoff_v - CHARGER_HYST_V)  charger_gated  = 0;
            if (bd.cell_min <= inverter_cutoff_v)                       inverter_gated = 1;
            else if (bd.cell_min >= inverter_cutoff_v + INVERTER_HYST_V) inverter_gated = 0;
        }
        // ALWAYS apply the (possibly held) latch state.
        if (charger_gated  && state >= 12)              state = 0;
        if (inverter_gated && state >= 3 && state <= 5) state = 0;
    }
#endif

    int inverter_active = (state >= 3 && state <= 5);
    int charger_active  = (state >= 12);
    TickType_t now    = xTaskGetTickCount();

    if (charger_active) {
        // ----- CHARGER MODE -----
        // Linear map of the 12..100 duty range onto 0..255: 12->0, 100->255.
		if (state < 12) {
        duty = 0;
	    } else {
	        // (state * 5) / 2 is the exact same as state * 2.5  but uses fast integer math
	        duty = (state * 5) / 2;
	    }
		// Optional: Keep this if your hardware still requires 
    	// the output to cap at a maximum of 255.
    	if (duty > 255) duty = 255;

        gpio_set_level(GPIO_CHARGER_ENABLE, 1);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_CHARGER, (uint32_t)duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_CHARGER);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_RELAY, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_RELAY);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_LED, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_LED);

        charger_pwm         = duty;
        relay_economiser    = 0;
        inverter_was_active = 0;

    } else if (inverter_active) {
        // ----- INVERTER MODE -----
        gpio_set_level(GPIO_CHARGER_ENABLE, 0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_CHARGER, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_CHARGER);
        charger_pwm = 0;

        if (!inverter_was_active) {
            // Rising edge (0 → active): start 100 % pull-in pulse
            inverter_engage_tick = now;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_RELAY, RELAY_ECON_DUTY_FULL);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_RELAY);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_LED, RELAY_ECON_DUTY_FULL);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_LED);
            relay_economiser = RELAY_ECON_DUTY_FULL;
        } else if ((now - inverter_engage_tick) >= (RELAY_ECON_PULSE_MS / portTICK_PERIOD_MS)) {
            // 500 ms elapsed: drop to economiser hold duty
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_RELAY, RELAY_ECON_DUTY_HOLD);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_RELAY);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_LED, RELAY_ECON_DUTY_HOLD);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_LED);
            relay_economiser = RELAY_ECON_DUTY_HOLD;
        }
        // else: still within 500 ms window — LEDC retains FULL duty, no write needed

        inverter_was_active = 1;

    } else {
        // ----- OFF (state == 0, 1, or 2) -----
        gpio_set_level(GPIO_CHARGER_ENABLE, 0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_CHARGER, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_CHARGER);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_RELAY, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_RELAY);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_LED, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_LED);

        charger_pwm         = 0;
        relay_economiser    = 0;
        inverter_was_active = 0;
    }
#endif
}

commandResult_t BL09XX_SetDumpLoad(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if (charger_c_auto == 1) return CMD_RES_OK; 
    
    if(args && *args) {
        char fallback_cmd[64];

        dump_load_relay[5] = atoi(args);

        /* Fire command to each configured charger IP */
        { int _ci; for (_ci = 0; _ci < UART_TCP_CHARGER_MAX; _ci++) {
            const char *_cip = UART_TCP_GetChargerIP(_ci);
            if (!_cip) continue;
            char fallback_cmd[96];
            snprintf(fallback_cmd, sizeof(fallback_cmd),
                     "SendGet http://%s/cm?cmnd=Channel3%%20%d",
                     _cip, dump_load_relay[5]);
            //CMD_ExecuteCommand(fallback_cmd, 0);
        }}
        ApplyDumpLoadGPIO(dump_load_relay[5]);
    }
    return CMD_RES_OK;
}

commandResult_t BL09XX_SetTargetPower(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if(args && *args) {
        int val = atoi(args);

        if (charger_c_auto == 1) {
            // AUTO: this is the ceiling the loop may regulate up to.
            if (val < 12)  val = 12;
            if (val > 100) val = 100;
            target_power_auto = val;
        } else {
            // MANUAL: this is the actual charger output, applied instantly.
            if (val > 5 && val < 12) val = 12;
            if (val > 100) val = 100;
            if (val < 0)   val = 0;
            target_power_manual = val;

            dump_load_relay[5] = target_power_manual;
            /* Fire command to each configured charger IP */
            { int _ci; for (_ci = 0; _ci < UART_TCP_CHARGER_MAX; _ci++) {
                const char *_cip = UART_TCP_GetChargerIP(_ci);
                if (!_cip) continue;
                char fallback_cmd[96];
                snprintf(fallback_cmd, sizeof(fallback_cmd),
                         "SendGet http://%s/cm?cmnd=Channel3%%20%d",
                         _cip, dump_load_relay[5]);
               // CMD_ExecuteCommand(fallback_cmd, 0);
            }}
            ApplyDumpLoadGPIO(dump_load_relay[5]);
        }
        SETTINGS_Save();
    }
    return CMD_RES_OK;
}

commandResult_t BL09XX_SetTargetExport(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if(args && *args) {
        int val = atoi(args);
        if (val < 0)   val = 0;
        if (val > 100) val = 100;
        target_export = val;
        // Diversion ON point must stay at least 10 Wh above the export level.
        if (divert_threshold < target_export + 10) divert_threshold = target_export + 10;
        SETTINGS_Save();
    }
    return CMD_RES_OK;
}

// Charger mode: 0 = AUTO, 1 = MANUAL temp (reverts at next rollover), 2 = MANUAL locked.
commandResult_t BL09XX_SetChargerMode(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    int m = (args && *args) ? atoi(args) : 0;
    if (m == 0)      { charger_c_auto = 1; charger_manual_temp = 0; }
    else if (m == 1) { charger_c_auto = 0; charger_manual_temp = 1; }
    else             { charger_c_auto = 0; charger_manual_temp = 0; }
    return CMD_RES_OK;
}

// Diversion override: 0 = auto, 1 = force-on temp (reverts at rollover), 2 = force-on locked.
commandResult_t BL09XX_SetDivertUser(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    int u = (args && *args) ? atoi(args) : 0;
    if (u < 0) u = 0;
    if (u > 2) u = 2;
    divert_user = u;
    return CMD_RES_OK;
}

// Diversion ON threshold (Wh). Clamped to >= target_export + 10.
commandResult_t BL09XX_SetDivertThreshold(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if (args && *args) {
        int v = atoi(args);
        int floor_v = target_export + 10;
        if (v < floor_v) v = floor_v;
        if (v > 255)     v = 255;
        divert_threshold = v;
        SETTINGS_Save();
    }
    return CMD_RES_OK;
}

commandResult_t BL09XX_ToggleAuto(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    charger_c_auto = !charger_c_auto;
    return CMD_RES_OK;
}

// ====================================================================
// SETTINGS PERSISTENCE  (NVS "config" namespace; keys <=15 chars)
// ====================================================================
// Persists the dashboard "System Configuration" fields + the two sliders
// to flash on change, mirroring drv_uart_tcp_client's NVS pattern. On a
// non-ESPIDF build these are no-ops (RAM-only) so the code stays portable.
#if PLATFORM_ESPIDF
static void SETTINGS_Save(void)
{
    nvs_handle_t h = 0;
    int i;
    char key[8];
    if (nvs_open("config", NVS_READWRITE, &h) != ESP_OK) return;
    for (i = 0; i < 6; i++) {
        snprintf(key, sizeof(key), "mip%d", i);
        nvs_set_u8(h, key, g_meter_ip[i]);
        snprintf(key, sizeof(key), "minv%d", i);
        nvs_set_u8(h, key, g_meter_invert[i]);
        snprintf(key, sizeof(key), "mvcal%d", i);
        nvs_set_i32(h, key, (int32_t)(g_meter_vcal[i] * 1000.0f + 0.5f));
        snprintf(key, sizeof(key), "macal%d", i);
        nvs_set_i32(h, key, (int32_t)(g_meter_acal[i] * 1000.0f + 0.5f));
        snprintf(key, sizeof(key), "mpcal%d", i);
        nvs_set_i32(h, key, (int32_t)(g_meter_pcal[i] * 1000.0f + 0.5f));
    }
    nvs_set_u8 (h, "inv2ip",  g_inv2_ip);
    nvs_set_u8 (h, "bypip",   g_bypass_ip);
    nvs_set_i32(h, "boostp",  g_boost_power);
    nvs_set_i32(h, "dthr",    divert_threshold);
    nvs_set_i32(h, "texp",    target_export);
    nvs_set_i32(h, "tpa",     target_power_auto);
    nvs_set_i32(h, "ccut",    (int)(charger_cutoff_v  * 100.0f + 0.5f));
    nvs_set_i32(h, "icut",    (int)(inverter_cutoff_v * 100.0f + 0.5f));
    nvs_set_str(h, "bmsmac",  g_bms_mac);
    nvs_set_str(h, "bms2mac", g_bms2_mac);
    nvs_commit(h);
    nvs_close(h);
}

static void SETTINGS_Load(void)
{
    nvs_handle_t h = 0;
    int i;
    char key[8];
    uint8_t  u8v;
    int32_t  i32v;
    size_t   len;
    if (nvs_open("config", NVS_READONLY, &h) != ESP_OK) return;
    for (i = 0; i < 6; i++) {
        snprintf(key, sizeof(key), "mip%d", i);
        if (nvs_get_u8(h, key, &u8v) == ESP_OK) g_meter_ip[i] = u8v;
        snprintf(key, sizeof(key), "minv%d", i);
        if (nvs_get_u8(h, key, &u8v) == ESP_OK) g_meter_invert[i] = u8v ? 1 : 0;
        snprintf(key, sizeof(key), "mvcal%d", i);
        if (nvs_get_i32(h, key, &i32v) == ESP_OK) g_meter_vcal[i] = i32v / 1000.0f;
        snprintf(key, sizeof(key), "macal%d", i);
        if (nvs_get_i32(h, key, &i32v) == ESP_OK) g_meter_acal[i] = i32v / 1000.0f;
        snprintf(key, sizeof(key), "mpcal%d", i);
        if (nvs_get_i32(h, key, &i32v) == ESP_OK) g_meter_pcal[i] = i32v / 1000.0f;
    }
    if (nvs_get_u8 (h, "inv2ip",  &u8v)  == ESP_OK) g_inv2_ip         = u8v;
    if (nvs_get_u8 (h, "bypip",   &u8v)  == ESP_OK) g_bypass_ip       = u8v;
    if (nvs_get_i32(h, "boostp",  &i32v) == ESP_OK) g_boost_power     = i32v;
    if (nvs_get_i32(h, "dthr",    &i32v) == ESP_OK) divert_threshold  = i32v;
    if (nvs_get_i32(h, "texp",    &i32v) == ESP_OK) target_export     = i32v;
    if (nvs_get_i32(h, "tpa",     &i32v) == ESP_OK) target_power_auto = i32v;
    if (nvs_get_i32(h, "ccut",    &i32v) == ESP_OK) charger_cutoff_v  = i32v / 100.0f;
    if (nvs_get_i32(h, "icut",    &i32v) == ESP_OK) inverter_cutoff_v = i32v / 100.0f;
    len = sizeof(g_bms_mac);  nvs_get_str(h, "bmsmac",  g_bms_mac,  &len);
    len = sizeof(g_bms2_mac); nvs_get_str(h, "bms2mac", g_bms2_mac, &len);
    nvs_close(h);
}

// Solar / ESS energy counters, stored as integer Wh (sub-Wh rounding is
// negligible; i32 Wh holds ~2.1 GWh of lifetime total). Saved on the 15-min
// boundary and at the midnight reset; loaded at boot.
static void COUNTERS_Save(void)
{
    nvs_handle_t h = 0;
    if (nvs_open("config", NVS_READWRITE, &h) != ESP_OK) return;
    /* Per-GROUP store: today + 3 days of 96 int16 slots, one blob per group
       (keys grp0..grp2, ~768 B each). */
    { int g; char k[8];
      for (g = 0; g < N_GROUPS; g++) {
          snprintf(k, sizeof(k), "grp%d", g);
          nvs_set_blob(h, k, &g_grp[g], sizeof(group_day_store_t));
      } }
    /* Lifetime buckets (totals forever, gross). */
    nvs_set_i64(h, "glImp",  life_grid_imp);
    nvs_set_i64(h, "glExp",  life_grid_exp);
    nvs_set_i64(h, "slLife", life_solar);
    nvs_set_i64(h, "blChg",  life_ess_chg);
    nvs_set_i64(h, "blDis",  life_ess_dis);
    /* Per-meter lifetime ticks (diagnostic reference). */
    { int i; char k[8];
      for (i = 0; i < N_METERS; i++) {
          snprintf(k, sizeof(k), "macc%d", i);
          nvs_set_i64(h, k, meter_acc[i]);
      } }
    nvs_set_i32(h, "curslot", g_cur_slot);
    nvs_commit(h);
    nvs_close(h);
}

static void COUNTERS_Load(void)
{
    nvs_handle_t h = 0;
    int32_t v; int64_t v64;
    if (nvs_open("config", NVS_READONLY, &h) != ESP_OK) return;
    { int g; char k[8]; size_t sz;
      for (g = 0; g < N_GROUPS; g++) {
          snprintf(k, sizeof(k), "grp%d", g);
          sz = sizeof(group_day_store_t);
          nvs_get_blob(h, k, &g_grp[g], &sz);    /* leaves zeroed if absent */
      } }
    if (nvs_get_i64(h, "glImp",  &v64) == ESP_OK) life_grid_imp = v64;
    if (nvs_get_i64(h, "glExp",  &v64) == ESP_OK) life_grid_exp = v64;
    if (nvs_get_i64(h, "slLife", &v64) == ESP_OK) life_solar    = v64;
    if (nvs_get_i64(h, "blChg",  &v64) == ESP_OK) life_ess_chg  = v64;
    if (nvs_get_i64(h, "blDis",  &v64) == ESP_OK) life_ess_dis  = v64;
    { int i; char k[8];
      for (i = 0; i < N_METERS; i++) {
          snprintf(k, sizeof(k), "macc%d", i);
          if (nvs_get_i64(h, k, &v64) == ESP_OK) meter_acc[i] = v64;
      } }
    if (nvs_get_i32(h, "curslot", &v) == ESP_OK) g_cur_slot = v;
    nvs_close(h);
}
#else
static void SETTINGS_Save(void) {}
static void SETTINGS_Load(void) {}
static void COUNTERS_Save(void) {}
static void COUNTERS_Load(void) {}
#endif

// ---- Settings setters (store to RAM, then flash-persist on change) ----

// SetMeterIP <slot 1..6> <octet 0..255> — assign a meter slave's last octet.
commandResult_t BL09XX_SetMeterIP(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    int slot, oct;
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2)) { return CMD_RES_NOT_ENOUGH_ARGUMENTS; }
    slot = Tokenizer_GetArgInteger(0);
    oct  = Tokenizer_GetArgInteger(1);
    if (slot < 1 || slot > 6) { return CMD_RES_BAD_ARGUMENT; }
    if (oct < 0)   oct = 0;
    if (oct > 255) oct = 255;
    g_meter_ip[slot - 1] = (unsigned char)oct;   // RAM only; flushed by SaveCfg
    return CMD_RES_OK;
}

// SetMeterInvert <slot 1..6> <0|1> — flip a reverse-wired meter's direction.
// RAM only; flushed to flash by SaveCfg (Apply button).
commandResult_t BL09XX_SetMeterInvert(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    int slot, inv;
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2)) { return CMD_RES_NOT_ENOUGH_ARGUMENTS; }
    slot = Tokenizer_GetArgInteger(0);
    inv  = Tokenizer_GetArgInteger(1);
    if (slot < 1 || slot > 6) { return CMD_RES_BAD_ARGUMENT; }
    g_meter_invert[slot - 1] = inv ? 1 : 0;
    return CMD_RES_OK;
}

// Shared calibration math for the three per-meter commands below: divide-mode,
// same as the onboard PwrCal — coefficient = latest raw chip code / the true
// value just measured externally. Self-persists immediately (unlike
// SetMeterInvert/SetMeterIP, which wait for the page's Save button) because
// the settings-page OK button turns green right after this call to confirm
// the coefficient is now live AND saved, not just staged.
static commandResult_t MeterCalibrate(const char *cmd, const char *args,
                                      uint32_t raw_table[6], int32_t raw_table_signed[6],
                                      int is_signed, float *cal_table) {
    int slot; float real, raw;
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    slot = Tokenizer_GetArgInteger(0);
    real = Tokenizer_GetArgFloat(1);
    if (slot < 1 || slot > 6) return CMD_RES_BAD_ARGUMENT;
    if (real == 0.0f) return CMD_RES_BAD_ARGUMENT;
    raw = is_signed ? (float)raw_table_signed[slot - 1] : (float)raw_table[slot - 1];
    if (raw > -0.001f && raw < 0.001f) return CMD_RES_ERROR;   // connect the load first
    cal_table[slot - 1] = raw / real;
    SETTINGS_Save();
    return CMD_RES_OK;
}

// SetMeterVoltCal <slot 1..6> <true volts> — measured with a trusted meter.
commandResult_t BL09XX_SetMeterVoltCal(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    return MeterCalibrate(cmd, args, g_meter_raw_v, NULL, 0, g_meter_vcal);
}

// SetMeterCurrentCal <slot 1..6> <true amps>
commandResult_t BL09XX_SetMeterCurrentCal(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    return MeterCalibrate(cmd, args, g_meter_raw_a, NULL, 0, g_meter_acal);
}

// SetMeterPowerCal <slot 1..6> <true watts>
commandResult_t BL09XX_SetMeterPowerCal(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    return MeterCalibrate(cmd, args, NULL, g_meter_raw_w, 1, g_meter_pcal);
}

// SetBmsMAC <AA:BB:CC:DD:EE:FF>
commandResult_t BL09XX_SetBmsMAC(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if (args && *args) { strncpy(g_bms_mac, args, sizeof(g_bms_mac) - 1); g_bms_mac[sizeof(g_bms_mac) - 1] = 0; }
    return CMD_RES_OK;
}

// SetBms2MAC <AA:BB:CC:DD:EE:FF>
commandResult_t BL09XX_SetBms2MAC(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if (args && *args) { strncpy(g_bms2_mac, args, sizeof(g_bms2_mac) - 1); g_bms2_mac[sizeof(g_bms2_mac) - 1] = 0; }
    return CMD_RES_OK;
}

// SetInv2IP <octet> — Boost Inverter last octet.
commandResult_t BL09XX_SetInv2IP(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if (args && *args) { int o = atoi(args); if (o < 0) o = 0; if (o > 255) o = 255; g_inv2_ip = (unsigned char)o; }
    return CMD_RES_OK;
}

// SetBypassIP <octet> — Diversion Load last octet.
commandResult_t BL09XX_SetBypassIP(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if (args && *args) { int o = atoi(args); if (o < 0) o = 0; if (o > 255) o = 255; g_bypass_ip = (unsigned char)o; }
    return CMD_RES_OK;
}

// SetBoostPower <Wh> — Boost net-energy trigger.
commandResult_t BL09XX_SetBoostPower(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if (args && *args) { int v = atoi(args); if (v < 0) v = 0; if (v > 500) v = 500; g_boost_power = v; }
    return CMD_RES_OK;
}

// SaveCfg — commit the settings-tab fields to flash in ONE write. The Save
// button pushes all the Set* values first, then calls this once. (The fields
// above are RAM-only on purpose: no flash write per keystroke/field.)
commandResult_t BL09XX_SaveCfg(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    SETTINGS_Save();
    return CMD_RES_OK;
}

// SetInv2 <0|1> — Boost Inverter desired state. Relayed to g_inv2_ip via SendGet
// (zero-IP-guarded) inside the control loop; not flash-persisted (runtime state).
commandResult_t BL09XX_SetInv2(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    g_inv2_on = (args && atoi(args)) ? 1 : 0;
    return CMD_RES_OK;
}

// SetChargerCutoff <centivolts> — per-cell MAX setpoint (charge stop). e.g. 360 = 3.60V.
// Slider control: persists on change.
commandResult_t BL09XX_SetChargerCutoff(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if (args && *args) {
        int cv = atoi(args);
        if (cv < 250) cv = 250;
        if (cv > 420) cv = 420;
        charger_cutoff_v = cv / 100.0f;
        SETTINGS_Save();
    }
    return CMD_RES_OK;
}

// SetInverterCutoff <centivolts> — per-cell MIN setpoint (discharge stop). e.g. 330 = 3.30V.
// Slider control: persists on change.
commandResult_t BL09XX_SetInverterCutoff(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if (args && *args) {
        int cv = atoi(args);
        if (cv < 250) cv = 250;
        if (cv > 420) cv = 420;
        inverter_cutoff_v = cv / 100.0f;
        SETTINGS_Save();
    }
    return CMD_RES_OK;
}

commandResult_t BL09XX_VCPPublishIntervals(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2)) { return CMD_RES_NOT_ENOUGH_ARGUMENTS; }
    changeDoNotSendMinFrames = Tokenizer_GetArgInteger(0);
    changeSendAlwaysFrames = Tokenizer_GetArgInteger(1);
    return CMD_RES_OK;
}

commandResult_t BL09XX_VCPPrecision(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    int i;
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) { return CMD_RES_NOT_ENOUGH_ARGUMENTS; }

    for (i = 0; i < Tokenizer_GetArgsCount(); i++) {
        int val = Tokenizer_GetArgInteger(i);
        switch(i) {
        case 0: sensors[OBK_VOLTAGE].rounding_decimals = val; break;
        case 1: sensors[OBK_CURRENT].rounding_decimals = val; break;
        case 2: 
            sensors[OBK_POWER].rounding_decimals = val;
            sensors[OBK_POWER_APPARENT].rounding_decimals = val;
            sensors[OBK_POWER_REACTIVE].rounding_decimals = val;
            break;
        case 3: 
            for (int j = OBK_CONSUMPTION__DAILY_FIRST; j <= OBK_CONSUMPTION__DAILY_LAST; j++) {
                sensors[j].rounding_decimals = val;
            };
        };
    }
    return CMD_RES_OK;
}

commandResult_t BL09XX_VCPPublishThreshold(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 3)) { return CMD_RES_NOT_ENOUGH_ARGUMENTS; }

    sensors[OBK_VOLTAGE].changeSendThreshold = Tokenizer_GetArgFloat(0);
    sensors[OBK_CURRENT].changeSendThreshold = Tokenizer_GetArgFloat(1);
    sensors[OBK_POWER].changeSendThreshold = Tokenizer_GetArgFloat(2);
    sensors[OBK_POWER_APPARENT].changeSendThreshold = Tokenizer_GetArgFloat(2);
    sensors[OBK_POWER_REACTIVE].changeSendThreshold = Tokenizer_GetArgFloat(2);

    if (Tokenizer_GetArgsCount() >= 4) {
        for (int i = OBK_CONSUMPTION_LAST_HOUR; i <= OBK_CONSUMPTION__DAILY_LAST; i++) {
            sensors[i].changeSendThreshold = Tokenizer_GetArgFloat(3);
        }
    }
    return CMD_RES_OK;
}

bool Channel_AreAllRelaysOpen() {
    int i, role, ch;
    for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
        role = g_cfg.pins.roles[i];
        ch = g_cfg.pins.channels[i];
        if (role == IOR_Relay) {
            if (CHANNEL_Get(ch)) { return false; }
        }
        if (role == IOR_Relay_n) {
            if (CHANNEL_Get(ch)==false) { return false; }
        }
        if (role == IOR_BridgeForward) {
            if (CHANNEL_Get(ch)) { return false; }
        }
    }
    return true;
}

float BL_ChangeEnergyUnitIfNeeded(float Wh) {
    if (CFG_HasFlag(OBK_FLAG_MQTT_ENERGY_IN_KWH)) { return Wh * 0.001f; }
    return Wh;
}

// ====================================================================
// REMOTE MULTI-METER INTERFACE (filled by the BL0942 TCP poller)
// ====================================================================
// Last-octet of meter `slot` (0 = unset → poller skips it). 0..5.
int BL_GetMeterOctet(int slot) {
    if (slot < 0 || slot >= 6) return 0;
    return g_meter_ip[slot];
}

// 1 if meter `slot` is reverse-wired (direction flipped). Read by the BL0942
// driver to flip that slot's signed WATT and signed CF-CNT energy together.
int BL_GetMeterInvert(int slot) {
    if (slot < 0 || slot >= 6) return 0;
    return g_meter_invert[slot];
}

// Per-meter calibration coefficients (divide-mode: calibrated = raw / this).
// Read by the BL0942 driver every frame to scale that slot's raw chip codes
// independently of the other five slots and the onboard sensor.
float BL_GetMeterVoltCal(int slot)    { return (slot < 0 || slot >= 6) ? METER_CAL_V_DEFAULT : g_meter_vcal[slot]; }
float BL_GetMeterCurrentCal(int slot) { return (slot < 0 || slot >= 6) ? METER_CAL_A_DEFAULT : g_meter_acal[slot]; }
float BL_GetMeterPowerCal(int slot)   { return (slot < 0 || slot >= 6) ? METER_CAL_P_DEFAULT : g_meter_pcal[slot]; }

// Latch this slot's latest RAW (pre-calibration) chip codes, so a calibration
// command issued right after has "what the chip just said" to compute
// coefficient = raw / true_value against.
void BL_SetMeterRaw(int slot, uint32_t raw_v, uint32_t raw_a, int32_t raw_w) {
    if (slot < 0 || slot >= 6) return;
    g_meter_raw_v[slot] = raw_v;
    g_meter_raw_a[slot] = raw_a;
    g_meter_raw_w[slot] = raw_w;
}

// Store one freshly-read GOOD reading: latch values and timestamp it.
// online=0 means a hard offline (unset IP) — clear the slot completely so it
// shows as absent. A *failed read* must NOT come through here as online=0
// (that would wipe the last-good value); use BL_MeterReadFailed instead.
void BL_SetMeterReading(int slot, float v, float a, float w, float freq, int online) {
    if (slot < 0 || slot >= 6) return;
    if (online) {
        g_meter[slot].v = v; g_meter[slot].a = a;
        g_meter[slot].w = w; g_meter[slot].freq = freq;
        g_meter[slot].online = 1;
        g_meter[slot].last_ok = xTaskGetTickCount();
    } else {
        g_meter[slot].v = 0; g_meter[slot].a = 0;
        g_meter[slot].w = 0; g_meter[slot].freq = 0;
        g_meter[slot].online = 0;
        g_meter[slot].last_ok = 0;
        g_meter[slot].cf_wh = 0.0f; g_meter[slot].cf_ticks = 0; g_meter[slot].cf_valid = 0;
    }
}

// Store a good reading plus this cycle's signed CF-CNT energy, in BOTH forms:
// cf_wh (calibrated Wh, for the live power/energy pipeline) and cf_ticks (raw
// signed pulse count, uncalibrated — the BL0942's native resolution, ~5116
// ticks/Wh). The raw counter delta / wrap / scaling is done in the BL0942
// driver; here we just latch both results and timestamp the slot.
void BL_SetMeterReadingCf(int slot, float v, float a, float w, float freq,
                          float cf_wh, int64_t cf_ticks, int cf_valid) {
    if (slot < 0 || slot >= 6) return;
    g_meter[slot].v = v; g_meter[slot].a = a;
    g_meter[slot].w = w; g_meter[slot].freq = freq;
    g_meter[slot].online = 1;
    g_meter[slot].last_ok = xTaskGetTickCount();
    g_meter[slot].cf_wh    = cf_valid ? cf_wh    : 0.0f;
    g_meter[slot].cf_ticks = cf_valid ? cf_ticks : 0;
    g_meter[slot].cf_valid = cf_valid ? 1 : 0;
}

// Signed net Wh contributed by this slot for the current sweep. Gated on both
// cf_valid AND freshness: a slot that dropped past the hold window contributes
// 0 rather than replaying a stale delta.
float BL_MeterCfWh(int slot) {
    if (slot < 0 || slot >= 6) return 0.0f;
    if (!g_meter[slot].cf_valid) return 0.0f;
    if (BL_MeterOnlineState(slot) == 0) return 0.0f;
    return g_meter[slot].cf_wh;
}

// Signed net RAW ticks contributed by this slot for the current sweep —
// uncalibrated, the BL0942's native pulse resolution. Same gating as
// BL_MeterCfWh. Used for lossless lifetime accumulation (meter_acc): integer
// ticks in, integer ticks out, no float accumulation ever, so recalibrating
// later never touches stored history — only the read-time Wh conversion does.
int64_t BL_MeterCfTicks(int slot) {
    if (slot < 0 || slot >= 6) return 0;
    if (!g_meter[slot].cf_valid) return 0;
    if (BL_MeterOnlineState(slot) == 0) return 0;
    return g_meter[slot].cf_ticks;
}

// Convert a signed raw tick count to Wh, for display only, using THIS METER's
// own calibration coefficient (divide-mode, same convention as everywhere
// else here). The stored ticks themselves are never touched by calibration —
// only this read-time conversion is, so recalibrating never rewrites history.
static float MeterAccToWh(int slot, int64_t ticks) {
    int64_t mag = (ticks < 0) ? -ticks : ticks;
    float pcal = BL_GetMeterPowerCal(slot);
    float wh = ((float)mag / pcal) * 1638.4f * 256.0f / 3600.0f;
    return (ticks < 0) ? -wh : wh;
}

// Clear per-cycle CF energy once the sweep has folded it into the totals.
void BL_MeterCfConsume(void) {
    int i;
    for (i = 0; i < 6; i++) { g_meter[i].cf_wh = 0.0f; g_meter[i].cf_ticks = 0; g_meter[i].cf_valid = 0; }
}

// A grid meter reported a chip reset mid-interval. Everything accumulated for
// the current 15-min interval is now untrustworthy (the counter it was derived
// from restarted), so wipe the interval's running net and let it re-accumulate
// from the next good delta. The lifetime totals are untouched — only the
// in-progress period is discarded, exactly as if this partial period never
// happened. The estimate re-derives from the zeroed net on the next tick.
void BL_MeterNoteReset(int slot) {
    (void)slot;
    real_consumption = 0.0f;
    real_export      = 0.0f;
    net_energy       = 0.0f;
    estimated_energy_period = 0;
}

// A read attempt failed but the meter may just have a comms hiccup: keep the
// last-good value and let it age out via last_ok. No store change needed —
// after METER_HOLD_TICKS the slot is reported offline automatically.
void BL_MeterReadFailed(int slot) {
    (void)slot;   // intentionally a no-op: do NOT overwrite last-good values
}

// Tri-state freshness for a slot: 0 = offline (never read, hard-offline, or
// last good read older than the hold window), 1 = fresh, 2 = stale-but-holding
// (within the hold window — show the value but flag a comms problem).
int BL_MeterOnlineState(int slot) {
    TickType_t age;
    if (slot < 0 || slot >= 6) return 0;
    if (!g_meter[slot].online)  return 0;
    age = xTaskGetTickCount() - g_meter[slot].last_ok;
    if (age <= METER_FRESH_TICKS) return 1;
    if (age <= METER_HOLD_TICKS)  return 2;
    return 0;
}

// Power for energy integration: last-good W while online (fresh or stale),
// else 0 so a >30 s dropout can't keep injecting phantom energy.
static float BL_MeterIntegW(int slot) {
    return BL_MeterOnlineState(slot) ? g_meter[slot].w : 0.0f;
}

// Read back a slot for the /api_dash?req=meters payload. *online returns the
// tri-state (0 offline / 1 fresh / 2 stale-holding).
int BL_GetMeter(int slot, float *v, float *a, float *w, int *online) {
    if (slot < 0 || slot >= 6) return 0;
    if (v)      *v      = g_meter[slot].v;
    if (a)      *a      = g_meter[slot].a;
    if (w)      *w      = g_meter[slot].w;
    if (online) *online = BL_MeterOnlineState(slot);
    return 1;
}

// Called once per 10 s cycle (on the poller's 10th tick). Sums each meter's
// signed CF-CNT delta for this cycle into the totals: the three grid phases
// feed the net import/export pipeline, Solar A+B feed generation, and the ESS
// slot splits charge/discharge by sign. Energy comes from the counter, not
// from power*time.
void BL_ProcessSweep(void) {
    // Energy is now taken straight from each meter's free-running signed CF-CNT
    // counter: the delta since the previous read (computed in the BL0942 driver)
    // IS the true net Wh that flowed over the ~10 s cycle, immune to the
    // instantaneous-watt-sign misattribution that power*time suffered on
    // pulsating loads. A slot with no valid delta this cycle (just (re)connected,
    // reset, or offline) contributes exactly 0 via BL_MeterCfWh().
    //
    // The instantaneous W is still summed for DISPLAY and for the 15-min
    // estimate ("instant consumption reported by the chip"); it no longer
    // drives the energy totals or the import/export split.

    // --- Accumulation, RAW CF-CNT TICKS only ---
    // Read each meter's signed tick delta for this ~10 s cycle BEFORE
    // BL_MeterCfConsume() clears it. Two independent layers:
    //   meter_acc[m]     = per-METER lifetime ticks (diagnostic reference only).
    //   g_cur_ticks[grp] = per-GROUP interval accumulator. NET METERING happens
    //     HERE: the grid phases (m0+m1+m2) net into one grid value, solar A+B
    //     (m3+m4) into one, battery (m5) its own. That single group-net is what
    //     gets frozen into the slot + bucketed at the boundary below. No Wh, no
    //     calibration here — that's a serve-time job. Offline/reset slots give 0.
    {
        int64_t t0 = BL_MeterCfTicks(0), t1 = BL_MeterCfTicks(1), t2 = BL_MeterCfTicks(2);
        int64_t t3 = BL_MeterCfTicks(3), t4 = BL_MeterCfTicks(4), t5 = BL_MeterCfTicks(5);
        meter_acc[0] += t0; meter_acc[1] += t1; meter_acc[2] += t2;
        meter_acc[3] += t3; meter_acc[4] += t4; meter_acc[5] += t5;
        g_cur_ticks[GRP_GRID]  += (int32_t)(t0 + t1 + t2);   // net across phases
        g_cur_ticks[GRP_SOLAR] += (int32_t)(t3 + t4);
        g_cur_ticks[GRP_BATT]  += (int32_t)(t5);
    }

    // --- Instantaneous W for DISPLAY + the 15-min estimate only ---
    // (No longer drives energy totals — those come purely from the slot store.)
    // Grid net watts feed BL_ProcessUpdate, which still owns the live estimate,
    // the control loop and the 12-hour graph matrices.
    {
        float cons_wh = BL_MeterCfWh(0) + BL_MeterCfWh(1) + BL_MeterCfWh(2); // signed
        float cons_w  = BL_MeterIntegW(0) + BL_MeterIntegW(1) + BL_MeterIntegW(2);
        BL_ProcessUpdate(g_meter[0].v, g_meter[0].a, cons_w, g_meter[0].freq, cons_wh);
    }

    // Fold-in done: clear this cycle's deltas so nothing is counted twice.
    BL_MeterCfConsume();

    // --- 15-min slot commit + local-midnight roll (per-meter store) ---
    // Every completed 15-min interval, each meter's accumulated calibrated Wh
    // (g_cur_ticks) is written into today's current slot and the accumulator is
    // reset. At local midnight the whole 96-slot arrays shift down a day
    // (3d<-2d<-1d<-today) and today is zeroed. The store is the single source
    // of truth: everything displayed is re-summed from these slots, so a save
    // here (COUNTERS_Save persists all 4 days x 6 meters + the tick totals)
    // fully captures state — no separate running accumulators to keep in sync.
    if (TIME_IsTimeSynced()) {
        static int last_qhr = -1;
        int msm  = TIME_GetHour() * 60 + TIME_GetMinute();
        int qhr  = msm / 15;                        // 0..95, current interval

        if (last_qhr < 0) {
            // First synced sweep after boot: adopt the current interval as the
            // fill target without committing a bogus partial slot.
            g_cur_slot = qhr;
        } else if (qhr != last_qhr) {
            int g;
            int completed = last_qhr;               // the slot we were filling

            // Freeze each GROUP's interval net into its slot, and roll that same
            // net into the sign-appropriate LIFETIME bucket (gross: import never
            // cancels export across intervals). One 15-min commit does slot +
            // lifetime together, so Today/Total/Last-Hour can never disagree.
            for (g = 0; g < N_GROUPS; g++) {
                long net = g_cur_ticks[g];
                if (completed >= 0 && completed < DAY_SLOTS)
                    g_grp[g].slots[completed] = ticks_to_slot(net);
                if (g == GRP_GRID)  { if (net > 0) life_grid_imp += net; else life_grid_exp += -net; }
                else if (g == GRP_SOLAR) { if (net > 0) life_solar += net; }   // one-way
                else /* GRP_BATT */ { if (net > 0) life_ess_chg += net; else life_ess_dis += -net; }
                g_cur_ticks[g] = 0;
            }

            // Local-midnight wrap: new interval index went backwards.
            if (qhr < last_qhr) {
                for (g = 0; g < N_GROUPS; g++) {
                    memcpy(g_grp[g].day3, g_grp[g].day2, sizeof(g_grp[g].day2));
                    memcpy(g_grp[g].day2, g_grp[g].day1, sizeof(g_grp[g].day1));
                    memcpy(g_grp[g].day1, g_grp[g].slots, sizeof(g_grp[g].slots));
                    memset(g_grp[g].slots, 0, sizeof(g_grp[g].slots));
                }
                actual_mday = TIME_GetMDay();
            }

            g_cur_slot = qhr;
            mark_energy_dirty();
            COUNTERS_Save();                        // persist store + lifetime + meter_acc

            // Persist the 12-hour visual graph matrices too (unchanged role).
#if PLATFORM_ESPIDF
            HAL_FlashVars_SaveGraphMatrices(net_graph_matrix,
                                            solar_graph_matrix, ess_pwr_matrix,
                                            MATRIX_SIZE, last_matrix_index,
                                            (unsigned int)TIME_GetCurrentTime());
#endif
        }
        last_qhr = qhr;
    }
}

void BL_ProcessUpdate(float voltage, float current, float power, float frequency, float energyWh) {
    int i;
    time_t ntpTime;
    struct tm *ltm;
    char datetime[64];
    float diff;

    // Capture tick at the very top of the function. This timestamp is used
    // for two purposes:
    //   1. loop_interval_ms  – the wall-clock gap between successive calls
    //      (replaces the old worst-case execution-time metric).
    //   2. Instantaneous power – Wh delta / elapsed time → Watts.
    TickType_t now_tick = xTaskGetTickCount();

    // ====================================================================
    // LOOP INTERVAL + INSTANTANEOUS POWER CALCULATION
    // ====================================================================
    // Both calculations are gated on having a previous timestamp to diff
    // against, so they're silently skipped on the very first call.
    if (last_processupdate_tick != 0)
    {
        // Time between this call and the previous one, in milliseconds.
        loop_interval_ms = (unsigned int)(
            (now_tick - last_processupdate_tick) * portTICK_PERIOD_MS);

        // Instantaneous power derived from the Wh the meter accumulated
        // over that same interval. Guard against zero elapsed time and
        // non-finite energyWh (stray meter glitch).
        if (loop_interval_ms > 0 && isfinite(energyWh))
        {
            float delta_s = loop_interval_ms / 1000.0f;

            // energyWh is already SIGNED (+ import / - export) — the sign now
            // comes from the CF-CNT delta, not from a single instantaneous watt
            // reading. Convert Wh → W over the interval; no smoothing needed.
            calc_power_w = (energyWh / delta_s) * 3600.0f;
        }
    }
    last_processupdate_tick = now_tick;

    if (TIME_IsTimeSynced())
    {                                          
        check_time = TIME_GetMinute();
        check_hour = TIME_GetHour();

        // ======================================================================================================
        // 30-SECOND SAMPLER (Battery power + Solar power averages)
        // ======================================================================================================
        static TickType_t last_30s_tick = 0;
        TickType_t current_sys_tick = xTaskGetTickCount();
        if ((current_sys_tick - last_30s_tick) >= (30000 / portTICK_PERIOD_MS) || last_30s_tick == 0) {
            last_30s_tick = current_sys_tick;

            // Battery (ESS = meter slot 5): signed power, + charge / - discharge.
            current_ess_pwr_accum += safe_int(BL_MeterIntegW(5));

            // Solar = meter slots 3 + 4, generation only (clamp negatives to 0).
            {
                int solar_w = safe_int(BL_MeterIntegW(3)) + safe_int(BL_MeterIntegW(4));
                if (solar_w < 0) solar_w = 0;
                current_solar_pwr_accum += solar_w;
            }
            sample_count_30s++;
        }

        // ------------------------------------------------------------------------------------------------------
        // THE 15-MINUTE RESET & CIRCULAR MATRIX LOGIC 
        // ------------------------------------------------------------------------------------------------------
        {
            int minutes_since_midnight_tracker = (check_hour * 60) + check_time;
            int interval_of_day_tracker = minutes_since_midnight_tracker / 15;
            int current_matrix_index = interval_of_day_tracker % MATRIX_SIZE; 

            if (last_matrix_index == -1) {
                last_matrix_index = current_matrix_index;
            }

            if (current_matrix_index != last_matrix_index) {
                float period_net;
                int ess_avg_w, solar_period_wh;

                consumption_matrix[last_matrix_index] = safe_int(real_consumption);
                export_matrix[last_matrix_index] = safe_int(real_export);

                // Full-precision net Wh for this period (includes decimals).
                period_net = real_consumption - real_export;

                // Store the true net Wh for the period (sanity-clamped to
                // a wide +/-9999.99 range, not the graph's display range).
                // OBK_CONSUMPTION_LAST_HOUR and other consumers need the
                // real value - only the graph gets a capped/scaled copy.
                {
                    float net_val = period_net;
                    if (net_val > 9999.99f)  net_val = 9999.99f;
                    if (net_val < -9999.99f) net_val = -9999.99f;
                    net_matrix[last_matrix_index] = net_val;
                }

                // Graph display copy: cap to -150..+300 Wh (the system
                // hovers near zero most of the time thanks to battery
                // buffering; larger swings are rare and simply clipped
                // here so the graph stays readable), then pack as
                // (val+150)/2 -> single byte 0..225.
                {
                    int graph_val = safe_int(period_net);
                    if (graph_val > 300)  graph_val = 300;
                    if (graph_val < -150) graph_val = -150;
                    net_graph_matrix[last_matrix_index] = (unsigned char)((graph_val + 150) / 2);
                }

                // TOP panel: average battery power for the interval, signed
                // (+ charge / - discharge), clamped to +/-500 W.
                ess_avg_w = sample_count_30s ? (current_ess_pwr_accum / sample_count_30s) : 0;
                if (ess_avg_w >  500) ess_avg_w =  500;
                if (ess_avg_w < -500) ess_avg_w = -500;
                ess_pwr_matrix[last_matrix_index] = ess_avg_w;

                // BOTTOM panel: solar ENERGY generated this period (Wh) =
                // average solar power (W) * 0.25 h. Clamped 0..150 to match the
                // chart's -150..+150 band; drawn as a negative yellow overlay.
                solar_period_wh = sample_count_30s
                    ? ((current_solar_pwr_accum / sample_count_30s) / 4)
                    : 0;
                if (solar_period_wh > 150) solar_period_wh = 150;
                if (solar_period_wh < 0)   solar_period_wh = 0;
                solar_graph_matrix[last_matrix_index] = (unsigned char)solar_period_wh;

                // Energy ACCOUNTING (today/history/totals/last-hour) is owned
                // entirely by the per-meter tick-slot store (committed in
                // BL_ProcessSweep) and re-summed at serve time — nothing is
                // applied to sensors[] here anymore. This block only maintains
                // the live 12-hour graph + control state.
                mark_energy_dirty();

                // Persist the graph matrices so the chart survives power cuts.
                // (Accounting persistence is COUNTERS_Save from BL_ProcessSweep.)
#if WINDOWS
#elif PLATFORM_BL602
#elif PLATFORM_W600 || PLATFORM_W800
#elif PLATFORM_XR809
#elif PLATFORM_BK7231N || PLATFORM_BK7231T
                if (ota_progress() == -1)
#endif
                {
                    lastConsumptionSaveStamp = xTaskGetTickCount();
#if PLATFORM_ESPIDF
                    HAL_FlashVars_SaveGraphMatrices(net_graph_matrix,
                                                   solar_graph_matrix, ess_pwr_matrix,
                                                   MATRIX_SIZE, last_matrix_index,
                                                   (unsigned int)TIME_GetCurrentTime());
#endif
                }

                // Preserve charger/inverter state across this reset - it will
                // be restored below so the control logic doesn't see a
                // transient net_energy near 0 and flip state spuriously.
                saved_persistent_state = persistent_state;
                saved_solar_excess = solar_excess;
                rollover_just_happened = 1;

                // Device-side auto-revert of the "temporary" (purple) overrides:
                // the temp manual charger mode falls back to AUTO, and a temp
                // force-on diversion falls back to auto control. Works even if
                // no browser is connected.
                if (charger_c_auto == 0 && charger_manual_temp) {
                    charger_c_auto = 1;
                    charger_manual_temp = 0;
                }
                if (divert_user == 1) divert_user = 0;

                real_export = 0;
                real_consumption = 0;
                net_energy = 0;
                // Point-2 offline policy: any meter still offline at this
                // boundary drops its CF-CNT baseline, so when it comes back in a
                // LATER interval it re-baselines (starts fresh from the first new
                // reading) instead of bridging a stale multi-interval gap. A brief
                // dropout WITHIN an interval still self-heals via the free-running
                // counter, so its energy is recovered when the meter returns.
#if PLATFORM_ESPIDF
                { int _m; for (_m = 0; _m < 6; _m++)
                    if (BL_MeterOnlineState(_m) == 0) BL0942_InvalidateBaseline(_m); }
#endif
                // Keep the "15min Est." tile in sync with "Now" - both
                // should drop to 0 together at the rollover, rather than
                // est. showing the previous period's value until the next
                // 30-second control tick recomputes it.
                estimated_energy_period = 0;
                
                consumption_matrix[current_matrix_index] = 0;
                export_matrix[current_matrix_index] = 0;
                net_matrix[current_matrix_index] = 0;
                net_graph_matrix[current_matrix_index] = (unsigned char)((0 + 150) / 2); // encodes 0 Wh
                ess_pwr_matrix[current_matrix_index] = 0;
                solar_graph_matrix[current_matrix_index] = 0;

                current_ess_pwr_accum   = 0;
                current_solar_pwr_accum = 0;
                sample_count_30s = 0;

                last_matrix_index = current_matrix_index;
            }
        }

        if (!(check_time == old_time))
        {
            old_time = check_time;
        }
                                                         
        net_energy = (real_consumption - real_export);                               

        // ======================================================================================================
        // CONTROL LOGIC (Target Export, Proportional-Integral Control)
        // ======================================================================================================
        static TickType_t last_control_tick = 0;
        TickType_t current_tick = xTaskGetTickCount();
        
        if ((current_tick - last_control_tick) >= (30000 / portTICK_PERIOD_MS) || last_control_tick == 0) 
        {
            int min_in_block;
            int check_time_estimate_mins;
            char fallback_cmd[64];

            last_control_tick = current_tick;
            
            min_in_block = check_time % 15; 
            check_time_estimate_mins = 15 - min_in_block; 
            if (check_time_estimate_mins <= 0) check_time_estimate_mins = 1;

            // 1. Predict total Wh accumulated by the end of the 15-minute period
            estimated_energy_period = safe_int(net_energy) + (safe_int(sensors[OBK_POWER].lastReading) * check_time_estimate_mins) / 60;

            // 2. Update Base Solar State
            if (net_energy < -((float)target_export + 10.0f)) {
                solar_available = 1;
            } else if (net_energy > 10.0f) {
                solar_available = 0;
            }

            // Consume the rollover flag here so it doesn't linger if
            // charger_c_auto is 0 (manual mode) on this tick.
            int handle_rollover = rollover_just_happened;
            rollover_just_happened = 0;

            // ====================================================================
            // ISOLATED LOGIC BLOCK (AUTO / MANUAL)
            // ====================================================================
            if (charger_c_auto == 1) {
                if (handle_rollover) {
                    // A 15-minute reset happened since the last control tick.
                    // net_energy is based on a freshly-zeroed (very short)
                    // window and isn't representative yet - skip the
                    // decision this cycle and keep whatever state the
                    // charger/inverter was already in. The next control
                    // tick (30s later) will have a real sample to evaluate.
                    persistent_state = saved_persistent_state;
                    solar_excess = saved_solar_excess;
                } else if (solar_available == 0) {
                    if (net_energy < -10.0f) {
                        persistent_state = 0;
                    } else if (net_energy > 5.0f) {
                        persistent_state = 5;
                    }
                    solar_excess = 0; 
                } 
                else {
                    if (net_energy > -((float)target_export)) {
                        solar_excess = 0; 
                    } else {
                        int excess_wh, error_w, pwm_step;

                        excess_wh = -(estimated_energy_period + target_export); 
                        
                        // Convert Wh error into Watts over the remaining time
                        error_w = (excess_wh * 60) / check_time_estimate_mins; 
                        
                        // Convert Watts to PWM step (10W = 1 PWM unit) dampened by half
                        pwm_step = (error_w / 10) / 2; 
                        
                        solar_excess += pwm_step;
                    }
                    
                    // Enforce absolute constraints
                    if (solar_excess > 88) solar_excess = 88;
                    if (solar_excess < 0) solar_excess = 0;
                    
                    persistent_state = 12 + solar_excess;
                    
                    int active_max = target_power_auto;
                    if (active_max < 12) active_max = 100;
                    
                    if (persistent_state > active_max) persistent_state = active_max;
                }
                
                dump_load_relay[5] = persistent_state;

                /* Fire command to each configured charger IP */
                { int _ci; for (_ci = 0; _ci < UART_TCP_CHARGER_MAX; _ci++) {
                    const char *_cip = UART_TCP_GetChargerIP(_ci);
                    if (!_cip) continue;
                    char fallback_cmd[96];
                    snprintf(fallback_cmd, sizeof(fallback_cmd),
                             "SendGet http://%s/cm?cmnd=Channel3%%20%d",
                             _cip, dump_load_relay[5]);
                  //  CMD_ExecuteCommand(fallback_cmd, 0);
                }}
                ApplyDumpLoadGPIO(dump_load_relay[5]);
            } // END OF AUTO BLOCK
        }

        // Diversion (.22 load) control — evaluated every loop while time is synced
        // (5 s charger delay + hysteresis handled inside).
        evaluate_diversion();
    } 

    sensors[OBK_VOLTAGE].lastReading = voltage;
    sensors[OBK_CURRENT].lastReading = current;
    sensors[OBK_POWER].lastReading = power;
    sensors[OBK_POWER_APPARENT].lastReading = sensors[OBK_VOLTAGE].lastReading * sensors[OBK_CURRENT].lastReading;
    sensors[OBK_POWER_REACTIVE].lastReading = (safe_int(net_energy));
    sensors[OBK_POWER_FACTOR].lastReading = (sensors[OBK_POWER_APPARENT].lastReading == 0 ? 1 : sensors[OBK_POWER].lastReading / sensors[OBK_POWER_APPARENT].lastReading);

    lastReadingFrequency = frequency;
// --------------------------------------
    // Final backstop: even though the BL0942 driver guards energyWh at the
    // source, never let a non-finite value into the period accumulators
    // (they feed the lifetime totals, which would be permanently poisoned).
    if (!isfinite(energyWh)) {
        energyWh = 0.0f;
    }
    // Import/export is decided by the SIGN OF THE ENERGY that actually flowed
    // this cycle (CF-CNT delta). real_export is a positive magnitude
    // (period_net = consumption - export), so add the negated value on export.
    // (The old OBK_FLAG_POWER_ALLOW_NEGATIVE gate is retired — export is always
    // recorded; a slot is just a signed net.)
    if (energyWh >= 0.0f)
    {
        real_consumption += energyWh;
    }
    else
    {
        real_export += -energyWh;
    }
//---------------------------------------

    if (TIME_IsTimeSynced()) {
        ntpTime = (time_t)TIME_GetCurrentTime();
        ltm = gmtime(&ntpTime);
        (void)ltm;
        if (ConsumptionResetTime == 0)
            ConsumptionResetTime = (time_t)ntpTime;
        if (actual_mday == -1)
            actual_mday = TIME_GetMDay();
        // Daily "today"/history rollover now happens ONCE, at local midnight,
        // in BL_ProcessSweep (unified with the grid/solar/ESS counters) - not
        // here on a separate UTC day-change.
    }

    for(i = OBK__FIRST; i <= OBK__LAST; i++)
    {
        diff = sensors[i].lastSentValue - sensors[i].lastReading;
        if ( ((fabsf(diff) > sensors[i].changeSendThreshold) &&
              (sensors[i].noChangeFrame >= changeDoNotSendMinFrames)) ||
            (sensors[i].noChangeFrame >= changeSendAlwaysFrames) )
        {
            enum EventCode eventChangeCode;
            sensors[i].noChangeFrame = 0;

            switch (i) {
            case OBK_VOLTAGE:                                   eventChangeCode = CMD_EVENT_CHANGE_VOLTAGE;                       break;
            case OBK_CURRENT:                                   eventChangeCode = CMD_EVENT_CHANGE_CURRENT;                       break;
            case OBK_POWER:                                     eventChangeCode = CMD_EVENT_CHANGE_POWER;                         break;
            case OBK_CONSUMPTION_TOTAL:                         eventChangeCode = CMD_EVENT_CHANGE_CONSUMPTION_TOTAL;             break;
            case OBK_GENERATION_TOTAL:                          eventChangeCode = CMD_EVENT_CHANGE_GENERATION_TOTAL;              break;
            case OBK_CONSUMPTION_LAST_HOUR:                     eventChangeCode = CMD_EVENT_CHANGE_CONSUMPTION_LAST_HOUR;         break;
            default:                                            eventChangeCode = CMD_EVENT_NONE;                                 break;
            }
            switch (eventChangeCode) {
            case CMD_EVENT_NONE:
                break;
            case CMD_EVENT_CHANGE_CURRENT: 
            {
                int prev_mA = sensors[i].lastSentValue * 1000;
                int now_mA = sensors[i].lastReading * 1000;
                EventHandlers_ProcessVariableChange_Integer(eventChangeCode, prev_mA,now_mA);
                break;
            }
            default:
                EventHandlers_ProcessVariableChange_Integer(eventChangeCode, sensors[i].lastSentValue, sensors[i].lastReading);
                break;
            }

            if (MQTT_IsReady() == true)
            {
                sensors[i].lastSentValue = sensors[i].lastReading;
                if (i == OBK_CONSUMPTION_CLEAR_DATE) {
                    sensors[i].lastReading = ConsumptionResetTime; 
                    ltm = gmtime(&ConsumptionResetTime);
                    if (NTP_GetTimesZoneOfsSeconds()>0)
                    {
                        snprintf(datetime, sizeof(datetime), "%04i-%02i-%02iT%02i:%02i+%02i:%02i",
                                 ltm->tm_year+1900, ltm->tm_mon+1, ltm->tm_mday, ltm->tm_hour, ltm->tm_min,
                                 NTP_GetTimesZoneOfsSeconds()/3600, (NTP_GetTimesZoneOfsSeconds()/60) % 60);
                    } else {
                        snprintf(datetime, sizeof(datetime), "%04i-%02i-%02iT%02i:%02i-%02i:%02i",
                                 ltm->tm_year+1900, ltm->tm_mon+1, ltm->tm_mday, ltm->tm_hour, ltm->tm_min,
                                 abs(NTP_GetTimesZoneOfsSeconds()/3600), (abs(NTP_GetTimesZoneOfsSeconds())/60) % 60);
                    }
                    MQTT_PublishMain_StringString(sensors[i].names.name_mqtt, datetime, 0);
                } else { 
                    float val = sensors[i].lastReading;
                    if (sensors[i].names.units == UNIT_WH) val = BL_ChangeEnergyUnitIfNeeded(val);
                    MQTT_PublishMain_StringFloat(sensors[i].names.name_mqtt, val, sensors[i].rounding_decimals, 0);
                }
                stat_updatesSent++;
            }
        } else {
            sensors[i].noChangeFrame++;
            stat_updatesSkipped++;
        }
    }       
}

void BL_Shared_Init(void)
{
    int i;
    ENERGY_METERING_DATA data;

    // Restore the dashboard "System Configuration" (meter IPs, MACs,
    // inv2/bypass octets, boost power) and the persisted sliders/threshold
    // from flash before anything reads them.
    SETTINGS_Load();
    COUNTERS_Load();

    for(i = OBK__FIRST; i <= OBK__LAST; i++)
    {
        sensors[i].noChangeFrame = 0;
        sensors[i].lastReading = 0;
    }

    // net_graph_matrix encodes Wh as (val+150)/2, so a raw 0 (the default
    // zero-init) decodes to -150 Wh, not 0 Wh. Initialize every slot to
    // the byte that represents 0 Wh so an empty history shows a flat
    // zero line instead of a full -150 Wh plateau.
    for (i = 0; i < MATRIX_SIZE; i++) {
        net_graph_matrix[i] = (unsigned char)((0 + 150) / 2);
    }

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "Read ENERGYMETER status values. sizeof(ENERGY_METERING_DATA)=%d\n", sizeof(ENERGY_METERING_DATA));

    HAL_GetEnergyMeterStatus(&data);
    sensors[OBK_CONSUMPTION_TOTAL].lastReading    = data.TotalConsumption;
    sensors[OBK_GENERATION_TOTAL].lastReading     = HAL_FlashVars_GetEnergyExportTotal();
    sensors[OBK_CONSUMPTION_TODAY].lastReading    = data.TodayConsumpion;
    sensors[OBK_CONSUMPTION_YESTERDAY].lastReading = data.YesterdayConsumption;
    actual_mday = data.actual_mday;
    lastSavedEnergyCounterValue = data.TotalConsumption;
    lastSavedGenerationCounterValue = sensors[OBK_GENERATION_TOTAL].lastReading;
    sensors[OBK_CONSUMPTION_2_DAYS_AGO].lastReading = data.ConsumptionHistory[0];
    sensors[OBK_CONSUMPTION_3_DAYS_AGO].lastReading = data.ConsumptionHistory[1];
    ConsumptionResetTime = data.ConsumptionResetTime;
    ConsumptionSaveCounter = data.save_counter;
    lastConsumptionSaveStamp = xTaskGetTickCount();

    /* Load daily export history */
    export_daily[0] = HAL_FlashVars_GetEnergyExportDaily(0);
    export_daily[1] = HAL_FlashVars_GetEnergyExportDaily(1);
    export_daily[2] = HAL_FlashVars_GetEnergyExportDaily(2);
    export_daily[3] = HAL_FlashVars_GetEnergyExportDaily(3);

    /* Restore 12-hour graph from NVS so it survives power cuts */
    {
        int saved_idx = 0;
        unsigned int saved_ts = 0;
        if (HAL_FlashVars_LoadGraphMatrices(net_graph_matrix,
                                            solar_graph_matrix, ess_pwr_matrix,
                                            MATRIX_SIZE, &saved_idx, &saved_ts)) {
            last_matrix_index = saved_idx;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER,
                      "Graph matrix restored from NVS (idx=%d)\n", saved_idx);
        }
        /* If load failed: matrices stay zero-init — correct for a fresh start */
    }

#if PLATFORM_ESPIDF
    // ---- GPIO / LEDC hardware init (charger enable + relay economiser outputs) ----
    {
        // GPIO4 — charger enable (digital output, start LOW / disabled)
        gpio_config_t io_conf;
        memset(&io_conf, 0, sizeof(io_conf));
        io_conf.pin_bit_mask = (1ULL << GPIO_CHARGER_ENABLE);
        io_conf.mode         = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type    = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level(GPIO_CHARGER_ENABLE, 0);

        // Shared LEDC timer: 1 kHz, 8-bit resolution
        ledc_timer_config_t tmr;
        memset(&tmr, 0, sizeof(tmr));
        tmr.speed_mode      = LEDC_LOW_SPEED_MODE;
        tmr.timer_num       = LEDC_TIMER_ACTUATION;
        tmr.duty_resolution = LEDC_RES_ACT;
        tmr.freq_hz         = LEDC_FREQ_HZ_ACT;
        tmr.clk_cfg         = LEDC_AUTO_CLK;
        ledc_timer_config(&tmr);

        // GPIO2 — charger PWM (LEDC channel 4, starts at 0)
        ledc_channel_config_t ch_chg;
        memset(&ch_chg, 0, sizeof(ch_chg));
        ch_chg.gpio_num   = GPIO_CHARGER_PWM;
        ch_chg.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_chg.channel    = LEDC_CH_CHARGER;
        ch_chg.timer_sel  = LEDC_TIMER_ACTUATION;
        ch_chg.duty       = 0;
        ch_chg.hpoint     = 0;
        ch_chg.intr_type  = LEDC_INTR_DISABLE;
        ledc_channel_config(&ch_chg);

        // GPIO0 — relay economiser PWM (LEDC channel 5, starts at 0)
        ledc_channel_config_t ch_rel;
        memset(&ch_rel, 0, sizeof(ch_rel));
        ch_rel.gpio_num   = GPIO_RELAY_ECON;
        ch_rel.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_rel.channel    = LEDC_CH_RELAY;
        ch_rel.timer_sel  = LEDC_TIMER_ACTUATION;
        ch_rel.duty       = 0;
        ch_rel.hpoint     = 0;
        ch_rel.intr_type  = LEDC_INTR_DISABLE;
        ledc_channel_config(&ch_rel);

        // GPIO8 — onboard LED mirrors relay economiser (LEDC channel 3, inverted)
        // LED is active-LOW (wired to 3V3), so output_invert=1 maps duty 0→off,
        // duty 255→full brightness without any logic inversion in software.
        ledc_channel_config_t ch_led;
        memset(&ch_led, 0, sizeof(ch_led));
        ch_led.gpio_num          = GPIO_INVERTER_LED;
        ch_led.speed_mode        = LEDC_LOW_SPEED_MODE;
        ch_led.channel           = LEDC_CH_LED;
        ch_led.timer_sel         = LEDC_TIMER_ACTUATION;
        ch_led.duty              = 0;
        ch_led.hpoint            = 0;
        ch_led.intr_type         = LEDC_INTR_DISABLE;
        ch_led.flags.output_invert = 1;
        ledc_channel_config(&ch_led);

        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER,
                  "GPIO actuation init: enable=GPIO%d, chargerPWM=GPIO%d ch%d, relay=GPIO%d ch%d\n",
                  GPIO_CHARGER_ENABLE, GPIO_CHARGER_PWM, LEDC_CH_CHARGER,
                  GPIO_RELAY_ECON, LEDC_CH_RELAY);
    }
#endif

    CMD_RegisterCommand("SetDumpLoad", BL09XX_SetDumpLoad, NULL);
    CMD_RegisterCommand("EnergyCntReset", BL09XX_ResetEnergyCounter, NULL);
    CMD_RegisterCommand("ToggleAuto", BL09XX_ToggleAuto, NULL);
    CMD_RegisterCommand("SetTargetPower", BL09XX_SetTargetPower, NULL);
    CMD_RegisterCommand("SetTargetExport", BL09XX_SetTargetExport, NULL);
    CMD_RegisterCommand("SetChargerMode", BL09XX_SetChargerMode, NULL);
    CMD_RegisterCommand("SetDivertUser", BL09XX_SetDivertUser, NULL);
    CMD_RegisterCommand("SetDivertThreshold", BL09XX_SetDivertThreshold, NULL);
    CMD_RegisterCommand("SetMeterIP", BL09XX_SetMeterIP, NULL);
    CMD_RegisterCommand("SetMeterInvert", BL09XX_SetMeterInvert, NULL);
    CMD_RegisterCommand("SetMeterVoltCal", BL09XX_SetMeterVoltCal, NULL);
    CMD_RegisterCommand("SetMeterCurrentCal", BL09XX_SetMeterCurrentCal, NULL);
    CMD_RegisterCommand("SetMeterPowerCal", BL09XX_SetMeterPowerCal, NULL);
    CMD_RegisterCommand("ClearMeteringData", BL09XX_ClearMeteringData, NULL);
    CMD_RegisterCommand("SetBmsMAC", BL09XX_SetBmsMAC, NULL);
    CMD_RegisterCommand("SetBms2MAC", BL09XX_SetBms2MAC, NULL);
    CMD_RegisterCommand("SetInv2IP", BL09XX_SetInv2IP, NULL);
    CMD_RegisterCommand("SetBypassIP", BL09XX_SetBypassIP, NULL);
    CMD_RegisterCommand("SetBoostPower", BL09XX_SetBoostPower, NULL);
    CMD_RegisterCommand("SetInv2", BL09XX_SetInv2, NULL);
    CMD_RegisterCommand("SaveCfg", BL09XX_SaveCfg, NULL);
    CMD_RegisterCommand("SetChargerCutoff", BL09XX_SetChargerCutoff, NULL);
    CMD_RegisterCommand("SetInverterCutoff", BL09XX_SetInverterCutoff, NULL);
    CMD_RegisterCommand("VCPPublishThreshold", BL09XX_VCPPublishThreshold, NULL);
    CMD_RegisterCommand("VCPPrecision", BL09XX_VCPPrecision, NULL);
    CMD_RegisterCommand("VCPPublishIntervals", BL09XX_VCPPublishIntervals, NULL);
}

float DRV_GetReading(energySensor_t type) 
{
    return sensors[type].lastReading;
}

energySensorNames_t* DRV_GetEnergySensorNames(energySensor_t type)
{
    return &sensors[type].names;
}

// ====================================================================
// JSON API ENDPOINT
// ====================================================================
int http_fn_api_dash(http_request_t *request) {
    const char *req_param = NULL;
    if (request->url) req_param = strstr(request->url, "req=");

    http_setup(request, "application/json");

    char buf[1024];  /* headroom for the expanded req=meters payload (per-meter mt[] + grid/gen/imp/exp with d/t/lh + 3-day h[] arrays) */
    int  pos     = 0;
    int  has_ntp = TIME_IsTimeSynced();   // endpoints gate on clock sync only

#define B(...) pos += snprintf(buf + pos, sizeof(buf) - pos, __VA_ARGS__)

    B("{");

    // ---- CORE (default or req=core) ----
    // Packed binary layout (26 bytes), little-endian, base64-encoded:
    //   bytes 0-1:  voltage   (uint16 ×10,  e.g. 2303 = 230.3 V)
    //   bytes 2-3:  current   (uint16 ×100, e.g. 1500 = 15.00 A)
    //   bytes 4-5:  power     (int16, whole W, signed)
    //   bytes 6-7:  calc_pwr  (int16, whole W, signed)
    //   bytes 8-9:  bal       (int16, whole Wh, signed)
    //   bytes 10-11:est       (int16, whole Wh, signed)
    //   byte  12:   dmp       (uint8, 0/5/18..100 — current charger output)
    //   byte  13:   mode      (uint8, 0=AUTO, 1=MANUAL temp, 2=MANUAL locked)
    //   byte  14:   t_pwr_a   (uint8, 0..100 — AUTO ceiling)
    //   byte  15:   t_exp     (uint8, 0..100 — export Wh, global)
    //   byte  16:   clk_h     (uint8, 0..23)
    //   byte  17:   clk_m     (uint8, 0..59)
    //   bytes 18-19:loop_ms   (uint16, ms between BL_ProcessUpdate calls)
    //   bytes 20-21:ev        (uint16, energy version counter)
    //   byte  22:   flags     (uint8, bit0 = has_ntp, bit1 = divert_is_on)
    //   byte  23:   t_pwr_m   (uint8, 0..100 — MANUAL charger output)
    //   byte  24:   div_user  (uint8, 0=auto, 1=force-on temp, 2=force-on locked)
    //   byte  25:   div_thr   (uint8, Wh — diversion ON threshold)
    // chg_v/chg_c/pwr_cls/bal_cls/est_cls are all derived client-side from
    // the values themselves, saving further bytes.
    if (!req_param || strncmp(req_param, "req=core", 8) == 0) {
        unsigned char raw[32];
        char          b64[((32 + 2) / 3) * 4 + 1];
        int           b64_len;
        int           dmp = dump_load_relay[5];
        int           mode_v = charger_c_auto ? 0 : (charger_manual_temp ? 1 : 2);
        int           soc_v = 255;   // 255 = BMS offline / unknown
#ifdef ENABLE_JK_BMS
        { jk_bms_data_t bd; if (JKBMS_GetData(&bd)) { soc_v = bd.soc; } }
#endif
        if (soc_v < 0)   soc_v = 0;
        if (soc_v > 254) soc_v = (soc_v == 255 ? 255 : 254);

        unsigned int volt_v  = (unsigned int)(sensors[OBK_VOLTAGE].lastReading * 10.0f  + 0.5f);
        unsigned int curr_v  = (unsigned int)(sensors[OBK_CURRENT].lastReading * 100.0f + 0.5f);
        int          pwr_v   = safe_int(sensors[OBK_POWER].lastReading);
        int          cpwr_v  = safe_int(calc_power_w);
        int          bal_v   = safe_int(sensors[OBK_POWER_REACTIVE].lastReading);
        int          est_v   = estimated_energy_period;
        unsigned int lms_v   = loop_interval_ms;
        unsigned int ev_v    = (unsigned int)(energy_version & 0xFFFF);

        if (volt_v > 0xFFFF) volt_v = 0xFFFF;
        if (curr_v > 0xFFFF) curr_v = 0xFFFF;
        if (pwr_v  >  32767) pwr_v  =  32767;
        if (pwr_v  < -32768) pwr_v  = -32768;
        if (cpwr_v >  32767) cpwr_v =  32767;
        if (cpwr_v < -32768) cpwr_v = -32768;
        if (bal_v  >  32767) bal_v  =  32767;
        if (bal_v  < -32768) bal_v  = -32768;
        if (est_v  >  32767) est_v  =  32767;
        if (est_v  < -32768) est_v  = -32768;
        if (lms_v  > 0xFFFF) lms_v  = 0xFFFF;

        raw[0]  = (unsigned char)(volt_v  & 0xFF);
        raw[1]  = (unsigned char)((volt_v  >> 8) & 0xFF);
        raw[2]  = (unsigned char)(curr_v  & 0xFF);
        raw[3]  = (unsigned char)((curr_v  >> 8) & 0xFF);
        raw[4]  = (unsigned char)((unsigned short)pwr_v   & 0xFF);
        raw[5]  = (unsigned char)(((unsigned short)pwr_v  >> 8) & 0xFF);
        raw[6]  = (unsigned char)((unsigned short)cpwr_v  & 0xFF);
        raw[7]  = (unsigned char)(((unsigned short)cpwr_v >> 8) & 0xFF);
        raw[8]  = (unsigned char)((unsigned short)bal_v   & 0xFF);
        raw[9]  = (unsigned char)(((unsigned short)bal_v  >> 8) & 0xFF);
        raw[10] = (unsigned char)((unsigned short)est_v   & 0xFF);
        raw[11] = (unsigned char)(((unsigned short)est_v  >> 8) & 0xFF);
        raw[12] = (unsigned char)(dmp < 0 ? 0 : dmp > 255 ? 255 : dmp);
        raw[13] = (unsigned char)mode_v;
        raw[14] = (unsigned char)(target_power_auto   < 0 ? 0 : target_power_auto   > 255 ? 255 : target_power_auto);
        raw[15] = (unsigned char)(target_export       < 0 ? 0 : target_export       > 255 ? 255 : target_export);
        raw[16] = (unsigned char)TIME_GetHour();
        raw[17] = (unsigned char)TIME_GetMinute();
        raw[18] = (unsigned char)(lms_v  & 0xFF);
        raw[19] = (unsigned char)((lms_v  >> 8) & 0xFF);
        raw[20] = (unsigned char)(ev_v   & 0xFF);
        raw[21] = (unsigned char)((ev_v   >> 8) & 0xFF);
        raw[22] = (unsigned char)((has_ntp ? 1 : 0) | (divert_is_on ? 2 : 0));
        raw[23] = (unsigned char)(target_power_manual < 0 ? 0 : target_power_manual > 255 ? 255 : target_power_manual);
        raw[24] = (unsigned char)(divert_user < 0 ? 0 : divert_user > 2 ? 2 : divert_user);
        raw[25] = (unsigned char)(divert_threshold < 0 ? 0 : divert_threshold > 255 ? 255 : divert_threshold);
        raw[26] = (unsigned char)soc_v;
        // ---- SYSTEM panel: WiFi RSSI (dBm) + uptime (seconds) ----
        // RSSI is negative dBm (e.g. -64); stored as a signed byte. Uptime is
        // g_secondsElapsed, little-endian uint32. Both refresh with req=core
        // (~10 s). IP is served once, statically, in req=cfg (it doesn't change).
        {
            int rssi = HAL_GetWifiStrength();               // dBm (negative)
            unsigned int up = (unsigned int)g_secondsElapsed;
            if (rssi >  127) rssi =  127;
            if (rssi < -128) rssi = -128;
            raw[27] = (unsigned char)((signed char)rssi);
            raw[28] = (unsigned char)( up        & 0xFF);
            raw[29] = (unsigned char)((up >>  8)  & 0xFF);
            raw[30] = (unsigned char)((up >> 16)  & 0xFF);
            raw[31] = (unsigned char)((up >> 24)  & 0xFF);
        }

        b64_len = base64_encode(raw, sizeof(raw), b64);
        b64[b64_len] = '\0';
        B("\"c\":\"%s\"", b64);
    }

    // ---- ENERGY TOTALS (req=energy) ----
    // Packed binary layout (19 bytes), little-endian, base64-encoded:
    //   bytes 0-3:   econs  (uint32, kWh*100)  -- lifetime import (consumption)
    //   bytes 4-7:   egen   (uint32, kWh*100)  -- lifetime export (generation)
    //   bytes 8-9:   clh    (uint16, kWh*100)  -- import last hour
    //   bytes 10-11: ctoday (uint16, kWh*100)
    //   bytes 12-13: cyest  (uint16, kWh*100)
    //   bytes 14-15: c2d    (uint16, kWh*100)
    //   bytes 16-17: c3d    (uint16, kWh*100)
    //   bytes 18-19: elh    (uint16, kWh*100)  -- export last hour
    //   bytes 20-21: etoday (uint16, kWh*100)
    //   bytes 22-23: eyest  (uint16, kWh*100)
    //   bytes 24-25: e2d    (uint16, kWh*100)
    //   bytes 26-27: e3d    (uint16, kWh*100)
    // The browser divides by 100 and renders import/export columns itself.
    else if (strncmp(req_param, "req=energy", 10) == 0 && has_ntp) {
        unsigned char raw[28];
        char          b64[((28 + 2) / 3) * 4 + 1];
        int           b64_len;

        /* Export last hour: sum of the last 4 fifteen-minute export slots,
           mirroring how OBK_CONSUMPTION_LAST_HOUR is built for import. */
        float elh_wh = 0.0f;
        { int idx = (last_matrix_index < 0) ? 0 : last_matrix_index, k;
          for (k = 0; k < 4; k++) {
              elh_wh += (float)export_matrix[idx];
              idx = (idx - 1 + MATRIX_SIZE) % MATRIX_SIZE;
          } }

        unsigned long econs_v  = (unsigned long)(0.1 * sensors[OBK_CONSUMPTION_TOTAL].lastReading + 0.5f);
        unsigned long egen_v   = (unsigned long)(0.1 * sensors[OBK_GENERATION_TOTAL].lastReading + 0.5f);
        unsigned int  clh_v    = (unsigned int)(0.1 * sensors[OBK_CONSUMPTION_LAST_HOUR].lastReading + 0.5f);
        unsigned int  ctoday_v = (unsigned int)(0.1 * sensors[OBK_CONSUMPTION_TODAY].lastReading + 0.5f);
        unsigned int  cyest_v  = (unsigned int)(0.1 * sensors[OBK_CONSUMPTION_YESTERDAY].lastReading + 0.5f);
        unsigned int  c2d_v    = (unsigned int)(0.1 * sensors[OBK_CONSUMPTION_2_DAYS_AGO].lastReading + 0.5f);
        unsigned int  c3d_v    = (unsigned int)(0.1 * sensors[OBK_CONSUMPTION_3_DAYS_AGO].lastReading + 0.5f);
        unsigned int  elh_v    = (unsigned int)(0.1 * elh_wh + 0.5f);
        unsigned int  etoday_v = (unsigned int)(0.1 * export_daily[0] + 0.5f);
        unsigned int  eyest_v  = (unsigned int)(0.1 * export_daily[1] + 0.5f);
        unsigned int  e2d_v    = (unsigned int)(0.1 * export_daily[2] + 0.5f);
        unsigned int  e3d_v    = (unsigned int)(0.1 * export_daily[3] + 0.5f);

        if (clh_v    > 0xFFFF) clh_v    = 0xFFFF;
        if (ctoday_v > 0xFFFF) ctoday_v = 0xFFFF;
        if (cyest_v  > 0xFFFF) cyest_v  = 0xFFFF;
        if (c2d_v    > 0xFFFF) c2d_v    = 0xFFFF;
        if (c3d_v    > 0xFFFF) c3d_v    = 0xFFFF;
        if (elh_v    > 0xFFFF) elh_v    = 0xFFFF;
        if (etoday_v > 0xFFFF) etoday_v = 0xFFFF;
        if (eyest_v  > 0xFFFF) eyest_v  = 0xFFFF;
        if (e2d_v    > 0xFFFF) e2d_v    = 0xFFFF;
        if (e3d_v    > 0xFFFF) e3d_v    = 0xFFFF;

        raw[0]  = (unsigned char)(econs_v & 0xFF);
        raw[1]  = (unsigned char)((econs_v >> 8) & 0xFF);
        raw[2]  = (unsigned char)((econs_v >> 16) & 0xFF);
        raw[3]  = (unsigned char)((econs_v >> 24) & 0xFF);
        raw[4]  = (unsigned char)(egen_v & 0xFF);
        raw[5]  = (unsigned char)((egen_v >> 8) & 0xFF);
        raw[6]  = (unsigned char)((egen_v >> 16) & 0xFF);
        raw[7]  = (unsigned char)((egen_v >> 24) & 0xFF);
        raw[8]  = (unsigned char)(clh_v & 0xFF);
        raw[9]  = (unsigned char)((clh_v >> 8) & 0xFF);
        raw[10] = (unsigned char)(ctoday_v & 0xFF);
        raw[11] = (unsigned char)((ctoday_v >> 8) & 0xFF);
        raw[12] = (unsigned char)(cyest_v & 0xFF);
        raw[13] = (unsigned char)((cyest_v >> 8) & 0xFF);
        raw[14] = (unsigned char)(c2d_v & 0xFF);
        raw[15] = (unsigned char)((c2d_v >> 8) & 0xFF);
        raw[16] = (unsigned char)(c3d_v & 0xFF);
        raw[17] = (unsigned char)((c3d_v >> 8) & 0xFF);
        raw[18] = (unsigned char)(elh_v & 0xFF);
        raw[19] = (unsigned char)((elh_v >> 8) & 0xFF);
        raw[20] = (unsigned char)(etoday_v & 0xFF);
        raw[21] = (unsigned char)((etoday_v >> 8) & 0xFF);
        raw[22] = (unsigned char)(eyest_v & 0xFF);
        raw[23] = (unsigned char)((eyest_v >> 8) & 0xFF);
        raw[24] = (unsigned char)(e2d_v & 0xFF);
        raw[25] = (unsigned char)((e2d_v >> 8) & 0xFF);
        raw[26] = (unsigned char)(e3d_v & 0xFF);
        raw[27] = (unsigned char)((e3d_v >> 8) & 0xFF);

        b64_len = base64_encode(raw, sizeof(raw), b64);
        b64[b64_len] = '\0';

        B("\"e\":\"%s\",\"ev\":%d", b64, energy_version);
    }

    // ---- BMS (req=bms) ----
    // {"b":"<base64 23 bytes>","mac":"AA:BB:.."}  -- iOS-5 safe (XHR + base64).
    // Packed layout (23 bytes), little-endian:
    //   byte 0:     flags (bit0 chg, bit1 dis, bit2 bal, bit3 connected)
    //   byte 1:     soc (0..100)
    //   bytes 2-3:  voltage   (uint16 ×100)
    //   bytes 4-5:  current   (int16  ×100, signed)
    //   bytes 6-7:  remaining (uint16 ×10, Ah)
    //   bytes 8-9:  full      (uint16 ×10, Ah)
    //   bytes 10-11:cell_min  (uint16 ×1000, V)
    //   bytes 12-13:cell_max  (uint16 ×1000, V)
    //   bytes 14-15:temp_1    (int16  ×10, signed)
    //   bytes 16-17:temp_2    (int16  ×10, signed)
    //   bytes 18-19:temp_mos  (int16  ×10, signed)
    //   bytes 20-21:bal_curr  (int16  ×100, signed)
    //   byte 22:    cell_count
    else if (req_param && strncmp(req_param, "req=bms", 7) == 0) {
        unsigned char raw[23];
        char          b64[((23 + 2) / 3) * 4 + 1];
        int           b64_len, connected = 0;
        const char   *mac = "--";

        memset(raw, 0, sizeof(raw));
#ifdef ENABLE_JK_BMS
        {
            jk_bms_data_t d;
            mac = JKBMS_GetMac();
            if (JKBMS_GetData(&d)) {
                int   soc   = d.soc;
                if (soc < 0)   soc = 0;
                if (soc > 100) soc = 100;
                int   volt  = (int)(d.total_voltage * 100.0f + 0.5f);
                int   rem   = (int)(d.remaining_ah   * 10.0f + 0.5f);
                int   full  = (int)(d.full_charge_ah * 10.0f + 0.5f);
                int   cmin  = (int)(d.cell_min * 1000.0f + 0.5f);
                int   cmax  = (int)(d.cell_max * 1000.0f + 0.5f);
                /* signed values: round away from zero so negatives are correct */
                int   curr  = (int)(d.current     * 100.0f + (d.current     < 0 ? -0.5f : 0.5f));
                int   t1    = (int)(d.temp_1      * 10.0f  + (d.temp_1      < 0 ? -0.5f : 0.5f));
                int   t2    = (int)(d.temp_2      * 10.0f  + (d.temp_2      < 0 ? -0.5f : 0.5f));
                int   tmos  = (int)(d.temp_mosfet * 10.0f  + (d.temp_mosfet < 0 ? -0.5f : 0.5f));
                int   bcur  = (int)(d.balance_current * 100.0f + (d.balance_current < 0 ? -0.5f : 0.5f));
                if (volt < 0) volt = 0;
                if (volt > 0xFFFF) volt = 0xFFFF;
                if (rem  < 0) rem  = 0;
                if (rem  > 0xFFFF) rem  = 0xFFFF;
                if (full < 0) full = 0;
                if (full > 0xFFFF) full = 0xFFFF;
                if (cmin < 0) cmin = 0;
                if (cmin > 0xFFFF) cmin = 0xFFFF;
                if (cmax < 0) cmax = 0;
                if (cmax > 0xFFFF) cmax = 0xFFFF;

                connected = 1;
                raw[0]  = (unsigned char)((d.charge_enabled?1:0) | (d.discharge_enabled?2:0)
                                          | (d.balancer_enabled?4:0) | 8 /*connected*/);
                raw[1]  = (unsigned char)soc;
                raw[2]  = (unsigned char)(volt & 0xFF);   raw[3]  = (unsigned char)((volt >> 8) & 0xFF);
                raw[4]  = (unsigned char)(curr & 0xFF);   raw[5]  = (unsigned char)((curr >> 8) & 0xFF);
                raw[6]  = (unsigned char)(rem & 0xFF);    raw[7]  = (unsigned char)((rem >> 8) & 0xFF);
                raw[8]  = (unsigned char)(full & 0xFF);   raw[9]  = (unsigned char)((full >> 8) & 0xFF);
                raw[10] = (unsigned char)(cmin & 0xFF);   raw[11] = (unsigned char)((cmin >> 8) & 0xFF);
                raw[12] = (unsigned char)(cmax & 0xFF);   raw[13] = (unsigned char)((cmax >> 8) & 0xFF);
                raw[14] = (unsigned char)(t1 & 0xFF);     raw[15] = (unsigned char)((t1 >> 8) & 0xFF);
                raw[16] = (unsigned char)(t2 & 0xFF);     raw[17] = (unsigned char)((t2 >> 8) & 0xFF);
                raw[18] = (unsigned char)(tmos & 0xFF);   raw[19] = (unsigned char)((tmos >> 8) & 0xFF);
                raw[20] = (unsigned char)(bcur & 0xFF);   raw[21] = (unsigned char)((bcur >> 8) & 0xFF);
                raw[22] = (unsigned char)(d.cell_count & 0xFF);
            }
        }
#endif
        (void)connected;
        b64_len = base64_encode(raw, sizeof(raw), b64);
        b64[b64_len] = '\0';
        B("\"b\":\"%s\",\"mac\":\"%s\"", b64, mac);
    }

    // ---- CONFIG (req=cfg) ----
    // Returns the RAM-stored "System Configuration" for the dashboard's
    // Retrieve button. IP fields are last-octet strings ("" = unset so the
    // input keeps its placeholder); MACs are full strings.
    else if (req_param && strncmp(req_param, "req=cfg", 7) == 0) {
        int i;
        B("\"bms1\":\"%s\",\"bms2\":\"%s\",", g_bms_mac, g_bms2_mac);
        for (i = 0; i < 6; i++) {
            if (g_meter_ip[i]) B("\"m%d\":\"%d\",", i + 1, g_meter_ip[i]);
            else               B("\"m%d\":\"\",", i + 1);
        }
        B("\"minv\":[");
        for (i = 0; i < 6; i++) B("%s%d", i ? "," : "", g_meter_invert[i]);
        B("],");
        if (g_inv2_ip)   B("\"inv2\":\"%d\",", g_inv2_ip);  else B("\"inv2\":\"\",");
        if (g_bypass_ip) B("\"byp\":\"%d\",", g_bypass_ip); else B("\"byp\":\"\",");
        B("\"boost\":%d,\"dthr\":%d", g_boost_power, divert_threshold);
        // Device IP — static, for the SYSTEM panel (served once with the config
        // the page fetches on load; it doesn't change at runtime).
        { const char *ip = HAL_GetMyIPString(); B(",\"ip\":\"%s\"", ip ? ip : ""); }
    }

    // ---- METERS (req=meters) ----
    // Per-meter live readings + all energy accounting, the latter derived
    // entirely from the per-meter tick-slot store and converted to Wh here at
    // the datasheet default rate (ticks_to_wh). mt[] slots: 0-2 = L1/L2/L3,
    // 3-4 = Solar A/B, 5 = ESS. v=volts*10 (1dp), w=signed watts, o=online,
    // e = per-meter LIFETIME Wh (from the raw tick accumulator, diagnosis).
    //
    // Energy groups (never mixed): gen = solar (one-way); grid = utility
    // import(i*)/export(e*); imp/exp = ESS charge/discharge. Each carries:
    //   d  = today          (sum of today's filled slots)
    //   t  = 3-day+today total (what we can prove from the store)
    //   lh = last hour      (last 4 completed slots, scalar)
    //   h  = [yesterday, 2 days ago, 3 days ago]   (3-element history array)
    // "today" for a group = grid_import(0) etc.; a past day = grid_import(age).
    else if (req_param && strncmp(req_param, "req=meters", 10) == 0) {
        int i;
        float v, a, w; int on;
        B("\"mt\":[");
        for (i = 0; i < 6; i++) {
            v = a = w = 0; on = 0;
            BL_GetMeter(i, &v, &a, &w, &on);
            B("%s{\"v\":%d,\"w\":%d,\"o\":%d,\"e\":%d}",
              i ? "," : "", (int)(v * 10.0f + 0.5f), (int)w, on,
              ticks_to_wh((long)meter_acc[i]));
        }
        // Solar (one-way).
        B("],\"gen\":{\"d\":%d,\"t\":%d,\"lh\":%d,\"h\":[%d,%d,%d]}",
          ticks_to_wh(solar_gen(0)),
          ticks_to_wh(solar_total()),
          ticks_to_wh(solar_lh()),
          ticks_to_wh(solar_gen(1)), ticks_to_wh(solar_gen(2)), ticks_to_wh(solar_gen(3)));
        // Grid import (i*) / export (e*).
        B(",\"grid\":{\"id\":%d,\"it\":%d,\"ilh\":%d,\"ih\":[%d,%d,%d],\"ed\":%d,\"et\":%d,\"elh\":%d,\"eh\":[%d,%d,%d]}",
          ticks_to_wh(grid_import(0)),
          ticks_to_wh(grid_import_total()),
          ticks_to_wh(grid_imp_lh()),
          ticks_to_wh(grid_import(1)), ticks_to_wh(grid_import(2)), ticks_to_wh(grid_import(3)),
          ticks_to_wh(grid_export(0)),
          ticks_to_wh(grid_export_total()),
          ticks_to_wh(grid_exp_lh()),
          ticks_to_wh(grid_export(1)), ticks_to_wh(grid_export(2)), ticks_to_wh(grid_export(3)));
        // ESS charge (imp) / discharge (exp).
        B(",\"imp\":{\"d\":%d,\"t\":%d,\"lh\":%d,\"h\":[%d,%d,%d]}",
          ticks_to_wh(ess_charge(0)),
          ticks_to_wh(ess_charge_total()),
          ticks_to_wh(ess_chg_lh()),
          ticks_to_wh(ess_charge(1)), ticks_to_wh(ess_charge(2)), ticks_to_wh(ess_charge(3)));
        B(",\"exp\":{\"d\":%d,\"t\":%d,\"lh\":%d,\"h\":[%d,%d,%d]}",
          ticks_to_wh(ess_discharge(0)),
          ticks_to_wh(ess_discharge_total()),
          ticks_to_wh(ess_dis_lh()),
          ticks_to_wh(ess_discharge(1)), ticks_to_wh(ess_discharge(2)), ticks_to_wh(ess_discharge(3)));
    }

    // ---- GRAPH ARRAYS (req=net | req=batt) ----
    // req=net: {"net":"b64_48","sol":"b64_48"} — bottom panel, bundled.
    //   net: 1 byte/slot = (clamp(net_Wh,-150,300)+150)/2. JS splits by sign:
    //        positive=total energy import (red up), negative=export (green down).
    //   sol: 1 byte/slot = solar Wh this period, 0..150. JS draws it negated
    //        (yellow, downward) as a semi-transparent overlay.
    // req=batt: {"batt":"b64_96"} — top panel, battery power.
    //   2 bytes/slot, little-endian 10-bit sign+magnitude:
    //   enc = (|W| & 0x1FF) | (W<0 ? 0x200 : 0), |W| clamped to 500.
    //   JS: mag = enc & 0x1FF; if (enc & 0x200) mag = -mag.  + = charge, - = discharge.
    else if (has_ntp && req_param) {
        unsigned int msm = TIME_GetHour() * 60 + TIME_GetMinute();

        if (strncmp(req_param, "req=net", 7) == 0) {
            unsigned char rn[MATRIX_SIZE], rs[MATRIX_SIZE];
            char          bn[((MATRIX_SIZE) + 2) / 3 * 4 + 1];
            char          bs[((MATRIX_SIZE) + 2) / 3 * 4 + 1];
            int           rn_len = 0, rs_len = 0, l;
            int           net_live   = safe_int(real_consumption - real_export);
            int           solar_live = sample_count_30s
                                       ? ((current_solar_pwr_accum / sample_count_30s) / 4) : 0;

            for (int i = 47; i >= 0; i--) {
                int idx  = (msm / net_metering_period - i + 96) % 96;
                int slot = idx % MATRIX_SIZE;
                if (i == 0) {
                    int val = net_live;
                    if (val > 300)  val = 300;
                    if (val < -150) val = -150;
                    rn[rn_len++] = (unsigned char)((val + 150) / 2);
                    if (solar_live > 150) solar_live = 150;
                    if (solar_live < 0)   solar_live = 0;
                    rs[rs_len++] = (unsigned char)solar_live;
                } else {
                    rn[rn_len++] = net_graph_matrix[slot];
                    rs[rs_len++] = solar_graph_matrix[slot];
                }
            }
            l = base64_encode(rn, rn_len, bn); bn[l] = '\0';
            l = base64_encode(rs, rs_len, bs); bs[l] = '\0';
            B("\"net\":\"%s\",\"sol\":\"%s\"", bn, bs);

        } else if (strncmp(req_param, "req=batt", 8) == 0) {
            unsigned char raw[MATRIX_SIZE * 2];
            char          b64[((MATRIX_SIZE * 2) + 2) / 3 * 4 + 1];
            int           raw_len = 0, b64_len;
            int           has_live = (sample_count_30s > 0);
            int           batt_live = has_live ? (current_ess_pwr_accum / sample_count_30s) : 0;

            for (int i = 47; i >= 0; i--) {
                int idx  = (msm / net_metering_period - i + 96) % 96;
                int slot = idx % MATRIX_SIZE;
                int w    = (i == 0 && has_live) ? batt_live : ess_pwr_matrix[slot];
                int mag, enc;
                if (w >  500) w =  500;
                if (w < -500) w = -500;
                mag = (w < 0) ? -w : w;
                enc = (mag & 0x1FF) | ((w < 0) ? 0x200 : 0);
                raw[raw_len++] = (unsigned char)(enc & 0xFF);
                raw[raw_len++] = (unsigned char)((enc >> 8) & 0x03);
            }
            b64_len = base64_encode(raw, raw_len, b64);
            b64[b64_len] = '\0';
            B("\"batt\":\"%s\"", b64);
        }
    }

    B("}");
    buf[pos] = '\0';
    poststr(request, buf);
    poststr(request, NULL);

#undef B
    return 0;
}

// Dashboard HTML/CSS/JS frontend has been moved to dash_frontend.c
// (see http_fn_custom_dash). This file only serves the JSON data
// via http_fn_api_dash, consumed by that frontend's polling JS.

/* =========================================================================
   Functions declared in drv_public.h and called by hass.c / http_fns.c.
   Our build uses a single flat sensors[] array (no ENABLE_BL_TWIN).
   ========================================================================= */

energySensorNames_t* DRV_GetEnergySensorNamesEx(int asensdatasetix, energySensor_t type)
{
    if (asensdatasetix != BL_SENSORS_IX_0) return NULL;
    if (type < OBK__FIRST || type > OBK__LAST) return NULL;
    return &sensors[type].names;
}

int BL_HasEnergySensorReadingEx(int asensdatasetix, energySensor_t type)
{
    if (asensdatasetix != BL_SENSORS_IX_0) return 0;
    if (type < OBK__FIRST || type > OBK__LAST) return 0;
    return !isnan((float)sensors[type].lastReading);
}

int BL_HasEnergySensorReading(energySensor_t type)
{
    return BL_HasEnergySensorReadingEx(BL_SENSORS_IX_0, type);
}
