// Internal code ONLY

#include <stdlib.h>   // atof, abs
#include <stdio.h>    // snprintf
#include <string.h>   // memset, strlen
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
static char          g_bms_mac[18]  = {0};   // "AA:BB:CC:DD:EE:FF"
static char          g_bms2_mac[18] = {0};
static unsigned char g_inv2_ip      = 0;     // Boost Inverter (remote), last octet
static unsigned char g_bypass_ip    = 0;     // Diversion Load (remote), last octet
static int           g_boost_power   = 10;   // Boost net-energy trigger (Wh)
static int           g_inv2_on       = 0;    // Boost Inverter desired state
static void SETTINGS_Save(void);             // defined after the NVS includes below

// ---- Remote meter slots (filled once per read by the BL0942 TCP poller) ----
// Slot roles match g_meter_ip[]: [0..2]=L1/L2/L3 (bidirectional grid phases,
// signed power: +import / -export), [3..4]=Solar A/B, [5]=ESS (bidir).
// online = 1 once a good reading has ever landed (stays 1 across brief read
// failures so we can hold the last-good value); last_ok = tick of the most
// recent successful read. Display/integration freshness is derived from the
// age of last_ok, not from online alone (see BL_MeterOnlineState).
typedef struct { float v, a, w, freq; int online; TickType_t last_ok; } meter_slot_t;
static meter_slot_t g_meter[6];

// Read freshness windows. A slot is polled ~every 6 s, so within 9 s a good
// read is "fresh"; up to 30 s we keep showing/integrating the last-good value
// but flag it as "stale" (comms hiccup); beyond 30 s it's treated as offline.
#define METER_FRESH_TICKS ((TickType_t)( 9000 / portTICK_PERIOD_MS))
#define METER_HOLD_TICKS  ((TickType_t)(30000 / portTICK_PERIOD_MS))

// ---- Solar / ESS energy counters (Wh) ----
// Today resets at local midnight; total is lifetime. Accumulated by
// BL_ProcessSweep from power*elapsed; flash-persisted on the 15-min boundary.
static float gen_today = 0, gen_total = 0;            // Solar generation (m4+m5)
static float ess_imp_today = 0, ess_imp_total = 0;    // ESS import / charge   (m6 >=0)
static float ess_exp_today = 0, ess_exp_total = 0;    // ESS export / discharge(m6 < 0)

#include "drv_bl_shared.h"

#include "../new_cfg.h"
#include "../new_pins.h"
#include "../hal/hal_flashVars.h"
#include "../logging/logging.h"
#include "../mqtt/new_mqtt.h"
#include "../hal/hal_ota.h"
#if PLATFORM_ESPIDF
#include "drv_uart_tcp_client.h"
#endif
#include "drv_local.h"
#include "drv_ntp.h"
#include "drv_deviceclock.h"   // TIME_* (live device clock) — used instead of stale NTP_*
#include "drv_public.h"
#include "drv_uart.h"
#include "../hal/hal_wifi.h"     // HAL_GetMyIPString (for the .22 diversion target)
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
    int charger_running = (dump_load_relay[5] >= 18);
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
// INSTANTANEOUS POWER (rolling 10-sample average derived from Wh delta)
// ====================================================================
// Each BL_ProcessUpdate call receives the Wh accumulated since the last
// call. Dividing by the elapsed time gives an instantaneous wattage that
// is independent of the meter's reported power field. Signed: positive =
// import (consumption), negative = export. The 10-sample rolling average
// smooths out per-call jitter without introducing significant lag.
#define INST_POWER_SAMPLES 10
static float         inst_power_buf[INST_POWER_SAMPLES] = {0};
static int           inst_power_idx   = 0;
static int           inst_power_count = 0;
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
        if (charger_gated  && state >= 18)              state = 0;
        if (inverter_gated && state >= 3 && state <= 5) state = 0;
    }
#endif

    int inverter_active = (state >= 3 && state <= 5);
    int charger_active  = (state >= 18);
    TickType_t now    = xTaskGetTickCount();

    if (charger_active) {
        // ----- CHARGER MODE -----
        int duty = ((state - 18) * 255) / 82;
        if (duty < 0)   duty = 0;
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
            if (val < 18)  val = 18;
            if (val > 100) val = 100;
            target_power_auto = val;
        } else {
            // MANUAL: this is the actual charger output, applied instantly.
            if (val > 5 && val < 18) val = 18;
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
    nvs_set_i32(h, "gtod",  (int)(gen_today     + 0.5f));
    nvs_set_i32(h, "gtot",  (int)(gen_total     + 0.5f));
    nvs_set_i32(h, "eitod", (int)(ess_imp_today + 0.5f));
    nvs_set_i32(h, "eitot", (int)(ess_imp_total + 0.5f));
    nvs_set_i32(h, "eetod", (int)(ess_exp_today + 0.5f));
    nvs_set_i32(h, "eetot", (int)(ess_exp_total + 0.5f));
    nvs_commit(h);
    nvs_close(h);
}

static void COUNTERS_Load(void)
{
    nvs_handle_t h = 0;
    int32_t v;
    if (nvs_open("config", NVS_READONLY, &h) != ESP_OK) return;
    if (nvs_get_i32(h, "gtod",  &v) == ESP_OK) gen_today     = (float)v;
    if (nvs_get_i32(h, "gtot",  &v) == ESP_OK) gen_total     = (float)v;
    if (nvs_get_i32(h, "eitod", &v) == ESP_OK) ess_imp_today = (float)v;
    if (nvs_get_i32(h, "eitot", &v) == ESP_OK) ess_imp_total = (float)v;
    if (nvs_get_i32(h, "eetod", &v) == ESP_OK) ess_exp_today = (float)v;
    if (nvs_get_i32(h, "eetot", &v) == ESP_OK) ess_exp_total = (float)v;
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
    }
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

// Called once per completed 6-meter sweep by the poller. Sums the three grid
// phases into the existing net pipeline (sign = import/export), and integrates
// the Solar and ESS energy counters from power * actual elapsed time.
void BL_ProcessSweep(void) {
    static TickType_t last_tick = 0;
    TickType_t now = xTaskGetTickCount();
    float dt_h;

    if (last_tick == 0) { last_tick = now; return; }   // seed timestamp, no integ yet
    dt_h = ((float)(now - last_tick) * (float)portTICK_PERIOD_MS) / 3600000.0f;
    last_tick = now;
    // Guard a stalled/huge gap (e.g. >72 s) so one slow sweep can't dump a
    // giant energy spike into the lifetime totals. Power is still fed.
    if (!(dt_h > 0.0f) || dt_h > 0.02f) dt_h = 0.0f;

    // --- Consumption: L1+L2+L3 (signed net) -> existing pipeline ---
    // Stale/offline phases contribute 0 W (BL_MeterIntegW) so a dropout can't
    // freeze a phantom load into the net. Voltage/freq still come from slot 0.
    {
        float cons_w  = BL_MeterIntegW(0) + BL_MeterIntegW(1) + BL_MeterIntegW(2);
        float cons_wh = (cons_w < 0.0f ? -cons_w : cons_w) * dt_h;   // magnitude
        BL_ProcessUpdate(g_meter[0].v, g_meter[0].a, cons_w, g_meter[0].freq, cons_wh);
    }

    // --- Solar generation: Solar A + Solar B ---
    {
        float gen_w  = BL_MeterIntegW(3) + BL_MeterIntegW(4);
        float gen_wh = (gen_w > 0.0f ? gen_w : 0.0f) * dt_h;
        gen_today += gen_wh; gen_total += gen_wh;
    }

    // --- ESS (m6) signed: import/charge (>=0) vs export/discharge (<0) ---
    {
        float ess_w  = BL_MeterIntegW(5);
        float ess_wh = (ess_w < 0.0f ? -ess_w : ess_w) * dt_h;
        if (ess_w >= 0.0f) { ess_imp_today += ess_wh; ess_imp_total += ess_wh; }
        else               { ess_exp_today += ess_wh; ess_exp_total += ess_wh; }
    }

    // --- 15-min flash persistence + midnight reset of "today" counters ---
    // Totals accumulate in RAM every sweep and are flushed to NVS once per
    // 15-min interval (matching the graph-matrix cadence, ~96 writes/day). At
    // the local-midnight wrap the "today" counters reset and we flush again so
    // a reboot just after midnight can't restore the old day.
    if (TIME_IsTimeSynced()) {
        static int last_msm  = -1;
        static int last_qhr  = -1;
        int msm = TIME_GetHour() * 60 + TIME_GetMinute();
        int qhr = msm / 15;                         // 15-min interval of the day

        if (last_msm >= 0 && msm < last_msm) {      // midnight wrap
            gen_today = 0; ess_imp_today = 0; ess_exp_today = 0;
            COUNTERS_Save();
        } else if (last_qhr >= 0 && qhr != last_qhr) {
            COUNTERS_Save();                        // 15-min boundary
        }
        last_msm = msm;
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

            // Convert Wh → W over the interval, then apply sign:
            //   power > 0  →  import  (positive)
            //   power <= 0 →  export  (negative)
            float inst_w = (energyWh / delta_s) * 3600.0f;
            if (power <= 0.0f) inst_w = -inst_w;

            // Push into the rolling 10-sample buffer and recompute average.
            inst_power_buf[inst_power_idx] = inst_w;
            inst_power_idx = (inst_power_idx + 1) % INST_POWER_SAMPLES;
            if (inst_power_count < INST_POWER_SAMPLES) inst_power_count++;

            {
                float sum = 0.0f;
                int k;
                for (k = 0; k < inst_power_count; k++) sum += inst_power_buf[k];
                calc_power_w = sum / (float)inst_power_count;
            }
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

                // Rolling last-hour IMPORT total: sum of the positive
                // (consumption) portions of the 4 most recent 15-minute
                // slots. Use consumption_matrix directly — always positive,
                // no sign check needed.
                {
                    float lh_sum = 0;
                    int lh_idx = last_matrix_index;
                    for (int lh_k = 0; lh_k < 4; lh_k++) {
                        lh_sum += (float)consumption_matrix[lh_idx];
                        lh_idx = (lh_idx - 1 + MATRIX_SIZE) % MATRIX_SIZE;
                    }
                    sensors[OBK_CONSUMPTION_LAST_HOUR].lastReading = lh_sum;
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

                // Apply the period's net energy to the running totals.
                if (period_net > 0) {
                    sensors[OBK_CONSUMPTION_TOTAL].lastReading += period_net;
                    sensors[OBK_CONSUMPTION_TODAY].lastReading += period_net;
                } else if (period_net < 0) {
                    float exp_wh = -period_net;
                    sensors[OBK_GENERATION_TOTAL].lastReading += exp_wh;
                    export_daily[0] += exp_wh;   /* track today's export separately */
                }
                mark_energy_dirty();

                // Single save point for the whole module: right before the
                // 15-minute accumulators are reset below.
#if WINDOWS
#elif PLATFORM_BL602
#elif PLATFORM_W600 || PLATFORM_W800
#elif PLATFORM_XR809
#elif PLATFORM_BK7231N || PLATFORM_BK7231T
                if (ota_progress() == -1)
#endif
                {
                    lastSavedEnergyCounterValue = sensors[OBK_CONSUMPTION_TOTAL].lastReading;
                    lastSavedGenerationCounterValue = sensors[OBK_GENERATION_TOTAL].lastReading;
                    BL09XX_SaveEmeteringStatistics();
                    lastConsumptionSaveStamp = xTaskGetTickCount();
#if PLATFORM_ESPIDF
                    /* Persist the graph matrices so the chart survives power
                       cuts: net (48 B) + solar (48 B) + battery power
                       (48 ints). Total NVS write: ~290 bytes every 15 min. */
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
                    if (solar_excess > 82) solar_excess = 82;
                    if (solar_excess < 0) solar_excess = 0;
                    
                    persistent_state = 18 + solar_excess;
                    
                    int active_max = target_power_auto;
                    if (active_max < 18) active_max = 100;
                    
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
    if (safe_int(power) > 0)
    {
        real_consumption += energyWh;
    }
    else
    {
        if (CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE))
        {
            real_export += energyWh;
        }
    }
//---------------------------------------

    if (TIME_IsTimeSynced()) {
        ntpTime = (time_t)TIME_GetCurrentTime();
        ltm = gmtime(&ntpTime);
        if (ConsumptionResetTime == 0)
            ConsumptionResetTime = (time_t)ntpTime;

        if (actual_mday == -1)
        {
            actual_mday = ltm->tm_mday;
        }
        if (actual_mday != ltm->tm_mday)
        {
            for (i = OBK_CONSUMPTION__DAILY_LAST; i >= OBK_CONSUMPTION__DAILY_FIRST; i--) {
                sensors[i].lastReading = sensors[i - 1].lastReading;
            }
            sensors[OBK_CONSUMPTION_TODAY].lastReading = 0.0;
            /* Roll over export daily history in parallel */
            export_daily[3] = export_daily[2];
            export_daily[2] = export_daily[1];
            export_daily[1] = export_daily[0];
            export_daily[0] = 0.0f;
            actual_mday = ltm->tm_mday;
            mark_energy_dirty();
            // Persistence for this rollover happens on the next 15-minute save.
        }
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

    char buf[512];
    int  pos     = 0;
    int  has_ntp = CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE) && TIME_IsTimeSynced();

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
        unsigned char raw[27];
        char          b64[((27 + 2) / 3) * 4 + 1];
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
        if (g_inv2_ip)   B("\"inv2\":\"%d\",", g_inv2_ip);  else B("\"inv2\":\"\",");
        if (g_bypass_ip) B("\"byp\":\"%d\",", g_bypass_ip); else B("\"byp\":\"\",");
        B("\"boost\":%d,\"dthr\":%d", g_boost_power, divert_threshold);
    }

    // ---- METERS (req=meters) ----
    // Per-meter live readings for the Sensor Data panel, fetched on the slow
    // (6/12 s) sweep timer. mt[] slots: 0-2 = L1/L2/L3, 3-4 = Solar A/B,
    // 5 = ESS (w signed; page shows import grey / export green). v=volts*10
    // (1 dp), w=signed watts, o=online. Totals (import/solar/ess) are summed
    // client-side. Also returns the Solar/ESS energy counters in Wh.
    else if (req_param && strncmp(req_param, "req=meters", 10) == 0) {
        int i;
        float v, a, w; int on;
        B("\"mt\":[");
        for (i = 0; i < 6; i++) {
            v = a = w = 0; on = 0;
            BL_GetMeter(i, &v, &a, &w, &on);
            B("%s{\"v\":%d,\"w\":%d,\"o\":%d}",
              i ? "," : "", (int)(v * 10.0f + 0.5f), (int)w, on);
        }
        B("],\"gen\":{\"d\":%d,\"t\":%d},\"imp\":{\"d\":%d,\"t\":%d},\"exp\":{\"d\":%d,\"t\":%d}",
          (int)(gen_today + 0.5f),     (int)(gen_total + 0.5f),
          (int)(ess_imp_today + 0.5f), (int)(ess_imp_total + 0.5f),
          (int)(ess_exp_today + 0.5f), (int)(ess_exp_total + 0.5f));
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
