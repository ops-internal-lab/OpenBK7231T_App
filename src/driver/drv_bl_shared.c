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

// New Averages matrices
static int charger_c_matrix[MATRIX_SIZE] = {0};
static int inverter_matrix[MATRIX_SIZE] = {0};
static int current_charger_c_accum = 0;
static int current_inverter_accum = 0;
static int sample_count_30s = 0;

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
// charger_on_tick (portTickType) is declared after the FreeRTOS headers below.

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
// inverter_engage_tick declared as static local inside ApplyDumpLoadGPIO —
// portTickType is only available after the FreeRTOS headers are pulled in
// by the OpenBK include block below, so it cannot live here at file scope.

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
static portTickType charger_on_tick = 0;   // tick the charger last went off->on
static void evaluate_diversion(void) {
    int charger_running = (dump_load_relay[5] >= 18);
    portTickType now = xTaskGetTickCount();
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
static portTickType  last_processupdate_tick = 0;
static unsigned int  loop_interval_ms        = 0;

int actual_mday = -1;
float lastSavedEnergyCounterValue = 0.0f;
float lastSavedGenerationCounterValue = 0.0f;
long ConsumptionSaveCounter = 0;
portTickType lastConsumptionSaveStamp;
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
    // Declared static local so portTickType is resolved after the FreeRTOS
    // headers are included above; retains value between calls like a file-
    // scope static would.
    static portTickType inverter_engage_tick = 0;
    int inverter_active = (state >= 3 && state <= 5);
    int charger_active  = (state >= 18);
    portTickType now    = xTaskGetTickCount();

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
    }
    return CMD_RES_OK;
}

commandResult_t BL09XX_ToggleAuto(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    charger_c_auto = !charger_c_auto;
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
    portTickType now_tick = xTaskGetTickCount();

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
        // 30-SECOND SAMPLER (Charger & Inverter Averages)
        // ======================================================================================================
        static portTickType last_30s_tick = 0;
        portTickType current_sys_tick = xTaskGetTickCount();
        if ((current_sys_tick - last_30s_tick) >= (30000 / portTICK_PERIOD_MS) || last_30s_tick == 0) {
            last_30s_tick = current_sys_tick;
            int current_dmp = dump_load_relay[5];
            
            if (current_dmp >= 18 && current_dmp <= 100) {
                current_charger_c_accum += current_dmp;
            } else if (current_dmp == 5) {
                current_inverter_accum += 100; // Represent directly as full percentage state
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
                int chg_val, inv_val;

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

                // Write averages for the interval, clamped to 0..127 so the
                // graph payload can be packed as a single byte each.
                chg_val = sample_count_30s ? (current_charger_c_accum / sample_count_30s) : 0;
                inv_val = sample_count_30s ? (current_inverter_accum / sample_count_30s) : 0;
                if (chg_val > 127) chg_val = 127;
                if (chg_val < 0)   chg_val = 0;
                if (inv_val > 127) inv_val = 127;
                if (inv_val < 0)   inv_val = 0;
                charger_c_matrix[last_matrix_index] = chg_val;
                inverter_matrix[last_matrix_index] = inv_val;

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
                    /* Persist the 48-byte net_graph_matrix so the graph
                       survives power cuts. Charger/inverter arrays also
                       saved. Total NVS write: ~150 bytes every 15 min. */
                    HAL_FlashVars_SaveGraphMatrices(net_graph_matrix,
                                                   charger_c_matrix, inverter_matrix,
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
                charger_c_matrix[current_matrix_index] = 0;
                inverter_matrix[current_matrix_index] = 0;

                current_charger_c_accum = 0;
                current_inverter_accum = 0;
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
        static portTickType last_control_tick = 0;
        portTickType current_tick = xTaskGetTickCount();
        
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
                                            charger_c_matrix, inverter_matrix,
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
        unsigned char raw[26];
        char          b64[((26 + 2) / 3) * 4 + 1];
        int           b64_len;
        int           dmp = dump_load_relay[5];
        int           mode_v = charger_c_auto ? 0 : (charger_manual_temp ? 1 : 2);

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

    // ---- GRAPH ARRAYS (req=net | req=chginv) ----
    // req=net:    {"net":"b64_48"} — original signed packing kept:
    //   1 byte per slot = (clamp(net_Wh,-150,300)+150)/2.
    //   JS splits by sign at render time: positive=import(red up),
    //   negative=export(green down). No extra bytes needed.
    // req=chginv: {"chginv":"b64_48"}
    //   int8 signed: +v=charger at v%, -v=inverter at v%, 0=neither.
    else if (has_ntp && req_param) {
        unsigned int msm = TIME_GetHour() * 60 + TIME_GetMinute();

        if (strncmp(req_param, "req=net", 7) == 0) {
            unsigned char raw[MATRIX_SIZE];
            char          b64[((MATRIX_SIZE) + 2) / 3 * 4 + 1];
            int           raw_len = 0, b64l;
            int           net_live = safe_int(real_consumption - real_export);

            for (int i = 47; i >= 0; i--) {
                int idx  = (msm / net_metering_period - i + 96) % 96;
                int slot = idx % MATRIX_SIZE;
                if (i == 0) {
                    int val = net_live;
                    if (val > 300)  val = 300;
                    if (val < -150) val = -150;
                    raw[raw_len++] = (unsigned char)((val + 150) / 2);
                } else {
                    raw[raw_len++] = net_graph_matrix[slot];
                }
            }
            b64l = base64_encode(raw, raw_len, b64); b64[b64l] = '\0';
            B("\"net\":\"%s\"", b64);

        } else if (strncmp(req_param, "req=chginv", 10) == 0) {
            unsigned char raw[MATRIX_SIZE];
            char          b64[((MATRIX_SIZE) + 2) / 3 * 4 + 1];
            int           raw_len = 0, b64_len;
            int           has_live = (sample_count_30s > 0);
            int           chg_live = 0, inv_live = 0;
            if (has_live) {
                chg_live = current_charger_c_accum / sample_count_30s;
                inv_live = current_inverter_accum / sample_count_30s;
            }
            for (int i = 47; i >= 0; i--) {
                int idx  = (msm / net_metering_period - i + 96) % 96;
                int slot = idx % MATRIX_SIZE;
                int chg_v = (i == 0 && has_live) ? chg_live : charger_c_matrix[slot];
                int inv_v = (i == 0 && has_live) ? inv_live : inverter_matrix[slot];
                int combined;
                if (chg_v > 0)      combined = chg_v;
                else if (inv_v > 0) combined = -inv_v;
                else                combined = 0;
                if (combined > 127)  combined = 127;
                if (combined < -127) combined = -127;
                raw[raw_len++] = (unsigned char)((signed char)combined);
            }
            b64_len = base64_encode(raw, raw_len, b64);
            b64[b64_len] = '\0';
            B("\"chginv\":\"%s\"", b64);
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
