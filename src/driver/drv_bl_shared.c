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

int estimated_energy_period = 0;

// NEW GLOBAL TARGETS
static int target_export = 20;
static int target_power = 100;

#define dump_load_relay_number 6
#define charger_c_ip 21
#define net_metering_period 15

static int dump_load_relay[dump_load_relay_number] = {0};

static int last_matrix_index = -1; 
int charger_c_auto = 1;

#include "drv_bl_shared.h"

#include "../new_cfg.h"
#include "../new_pins.h"
#include "../hal/hal_flashVars.h"
#include "../logging/logging.h"
#include "../mqtt/new_mqtt.h"
#include "../hal/hal_ota.h"
#include "drv_local.h"
#include "drv_ntp.h"
#include "drv_public.h"
#include "drv_uart.h"
#include "../cmnds/cmd_public.h" //for enum EventCode
#include <math.h>
#include <time.h>

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
	(void)bPreState; /* dashboard replaces the old pre/post-state HTML pattern */
    // Dashboard migrated to standalone JSON architecture on /dash
}

void BL09XX_SaveEmeteringStatistics()
{
    ENERGY_METERING_DATA data;
    memset(&data, 0, sizeof(ENERGY_METERING_DATA));

    data.TotalGeneration = sensors[OBK_GENERATION_TOTAL].lastReading;
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
    ConsumptionResetTime = (time_t)NTP_GetCurrentTime();
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

commandResult_t BL09XX_SetDumpLoad(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if (charger_c_auto == 1) return CMD_RES_OK; 
    
    if(args && *args) {
        char fallback_cmd[64];

        dump_load_relay[5] = atoi(args);
        
        snprintf(fallback_cmd, sizeof(fallback_cmd), "SendGet http://192.168.8.%d/cm?cmnd=Channel3%%20%d", charger_c_ip, dump_load_relay[5]);
        CMD_ExecuteCommand(fallback_cmd, 0);
    }
    return CMD_RES_OK;
}

commandResult_t BL09XX_SetTargetPower(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if(args && *args) {
        int val = atoi(args);
        
        // Ensure values fall into logic limits
        if (val > 5 && val < 18) val = 18;
        if (val > 100) val = 100;
        
        target_power = val;
        
        // Instant execution if manual
        if (charger_c_auto == 0) {
            char fallback_cmd[64];
            dump_load_relay[5] = target_power;

            snprintf(fallback_cmd, sizeof(fallback_cmd), "SendGet http://192.168.8.%d/cm?cmnd=Channel3%%20%d", charger_c_ip, dump_load_relay[5]);
            CMD_ExecuteCommand(fallback_cmd, 0);
        }
    }
    return CMD_RES_OK;
}

commandResult_t BL09XX_SetTargetExport(const void *context, const char *cmd, const char *args, int cmdFlags)
{
    if(args && *args) {
        target_export = atoi(args);
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

    if (NTP_IsTimeSynced())
    {                                          
        check_time = NTP_GetMinute();
        check_hour = NTP_GetHour();

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
                // net values. Export periods contribute 0 here - export
                // is tracked separately via OBK_GENERATION_TOTAL, not as
                // part of "last hour" - so this value is never negative.
                {
                    float lh_sum = 0;
                    int lh_idx = last_matrix_index;
                    for (int lh_k = 0; lh_k < 4; lh_k++) {
                        if (net_matrix[lh_idx] > 0) {
                            lh_sum += net_matrix[lh_idx];
                        }
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
                    sensors[OBK_GENERATION_TOTAL].lastReading += (-period_net);
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
                }

                // Preserve charger/inverter state across this reset - it will
                // be restored below so the control logic doesn't see a
                // transient net_energy near 0 and flip state spuriously.
                saved_persistent_state = persistent_state;
                saved_solar_excess = solar_excess;
                rollover_just_happened = 1;

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
                    
                    int active_max = target_power;
                    if (active_max < 18) active_max = 100;
                    
                    if (persistent_state > active_max) persistent_state = active_max;
                }
                
                dump_load_relay[5] = persistent_state;

                // Send Commands via process loop
                snprintf(fallback_cmd, sizeof(fallback_cmd), "SendGet http://192.168.8.%d/cm?cmnd=Channel3%%20%d", charger_c_ip, dump_load_relay[5]);
                CMD_ExecuteCommand(fallback_cmd, 0);
            } // END OF AUTO BLOCK
        }
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

    if (NTP_IsTimeSynced()) {
        ntpTime = (time_t)NTP_GetCurrentTime();
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
    sensors[OBK_CONSUMPTION_TOTAL].lastReading = data.TotalConsumption;
    sensors[OBK_GENERATION_TOTAL].lastReading = data.TotalGeneration;
    sensors[OBK_CONSUMPTION_TODAY].lastReading = data.TodayConsumpion;
    sensors[OBK_CONSUMPTION_YESTERDAY].lastReading = data.YesterdayConsumption;
    actual_mday = data.actual_mday;   
    lastSavedEnergyCounterValue = data.TotalConsumption;
    lastSavedGenerationCounterValue = data.TotalGeneration;
    sensors[OBK_CONSUMPTION_2_DAYS_AGO].lastReading = data.ConsumptionHistory[0];
    sensors[OBK_CONSUMPTION_3_DAYS_AGO].lastReading = data.ConsumptionHistory[1];
    ConsumptionResetTime = data.ConsumptionResetTime;
    ConsumptionSaveCounter = data.save_counter;
    lastConsumptionSaveStamp = xTaskGetTickCount();

    CMD_RegisterCommand("SetDumpLoad", BL09XX_SetDumpLoad, NULL);
    CMD_RegisterCommand("EnergyCntReset", BL09XX_ResetEnergyCounter, NULL);
    CMD_RegisterCommand("ToggleAuto", BL09XX_ToggleAuto, NULL);
    CMD_RegisterCommand("SetTargetPower", BL09XX_SetTargetPower, NULL);
    CMD_RegisterCommand("SetTargetExport", BL09XX_SetTargetExport, NULL);
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
    int  has_ntp = CFG_HasFlag(OBK_FLAG_POWER_ALLOW_NEGATIVE) && NTP_IsTimeSynced();

#define B(...) pos += snprintf(buf + pos, sizeof(buf) - pos, __VA_ARGS__)

    B("{");

    // ---- CORE (default or req=core) ----
    // Packed binary layout (23 bytes), little-endian, base64-encoded:
    //   bytes 0-1:  voltage   (uint16 ×10,  e.g. 2303 = 230.3 V)
    //   bytes 2-3:  current   (uint16 ×100, e.g. 1500 = 15.00 A)
    //   bytes 4-5:  power     (int16, whole W, signed)
    //   bytes 6-7:  calc_pwr  (int16, whole W, signed)
    //   bytes 8-9:  bal       (int16, whole Wh, signed)
    //   bytes 10-11:est       (int16, whole Wh, signed)
    //   byte  12:   dmp       (uint8, 0/5/18..100)
    //   byte  13:   auto      (uint8, 0 or 1)
    //   byte  14:   t_pwr     (uint8, 0..100)
    //   byte  15:   t_exp     (uint8, 0..100)
    //   byte  16:   clk_h     (uint8, 0..23)
    //   byte  17:   clk_m     (uint8, 0..59)
    //   bytes 18-19:loop_ms   (uint16, ms between BL_ProcessUpdate calls)
    //   bytes 20-21:ev        (uint16, energy version counter)
    //   byte  22:   flags     (uint8, bit0 = has_ntp)
    // chg_v/chg_c/pwr_cls/bal_cls/est_cls are all derived client-side from
    // the values themselves, saving further bytes.
    if (!req_param || strncmp(req_param, "req=core", 8) == 0) {
        unsigned char raw[23];
        char          b64[((23 + 2) / 3) * 4 + 1];
        int           b64_len;
        int           dmp = dump_load_relay[5];

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
        raw[13] = (unsigned char)(charger_c_auto ? 1 : 0);
        raw[14] = (unsigned char)(target_power  < 0 ? 0 : target_power  > 255 ? 255 : target_power);
        raw[15] = (unsigned char)(target_export < 0 ? 0 : target_export > 255 ? 255 : target_export);
        raw[16] = (unsigned char)NTP_GetHour();
        raw[17] = (unsigned char)NTP_GetMinute();
        raw[18] = (unsigned char)(lms_v  & 0xFF);
        raw[19] = (unsigned char)((lms_v  >> 8) & 0xFF);
        raw[20] = (unsigned char)(ev_v   & 0xFF);
        raw[21] = (unsigned char)((ev_v   >> 8) & 0xFF);
        raw[22] = (unsigned char)(has_ntp ? 1 : 0);

        b64_len = base64_encode(raw, sizeof(raw), b64);
        b64[b64_len] = '\0';
        B("\"c\":\"%s\"", b64);
    }

    // ---- ENERGY TOTALS (req=energy) ----
    // Packed binary layout (19 bytes), little-endian, base64-encoded:
    //   byte 0:     pf   (uint8,  value*100, 0.00-1.00)
    //   bytes 1-4:  econs (uint32, kWh*100)  -- lifetime total consumption
    //   bytes 5-8:  egen  (uint32, kWh*100)  -- lifetime total generation
    //   bytes 9-10: clh    (uint16, kWh*100) -- last hour
    //   bytes 11-12:ctoday (uint16, kWh*100)
    //   bytes 13-14:cyest  (uint16, kWh*100)
    //   bytes 15-16:c2d    (uint16, kWh*100)
    //   bytes 17-18:c3d    (uint16, kWh*100)
    // The browser divides by 100 and appends " kWh" itself.
    else if (strncmp(req_param, "req=energy", 10) == 0 && has_ntp) {
        unsigned char raw[19];
        char          b64[((19 + 2) / 3) * 4 + 1];
        int           b64_len;

        float pf_v     = sensors[OBK_POWER_FACTOR].lastReading;
        unsigned long econs_v  = (unsigned long)(0.1 * sensors[OBK_CONSUMPTION_TOTAL].lastReading + 0.5f);
        unsigned long egen_v   = (unsigned long)(0.1 * sensors[OBK_GENERATION_TOTAL].lastReading + 0.5f);
        unsigned int  clh_v    = (unsigned int)(0.1 * sensors[OBK_CONSUMPTION_LAST_HOUR].lastReading + 0.5f);
        unsigned int  ctoday_v = (unsigned int)(0.1 * sensors[OBK_CONSUMPTION_TODAY].lastReading + 0.5f);
        unsigned int  cyest_v  = (unsigned int)(0.1 * sensors[OBK_CONSUMPTION_YESTERDAY].lastReading + 0.5f);
        unsigned int  c2d_v    = (unsigned int)(0.1 * sensors[OBK_CONSUMPTION_2_DAYS_AGO].lastReading + 0.5f);
        unsigned int  c3d_v    = (unsigned int)(0.1 * sensors[OBK_CONSUMPTION_3_DAYS_AGO].lastReading + 0.5f);

        int pf_byte = safe_int(pf_v * 100.0f + 0.5f);
        if (pf_byte > 255) pf_byte = 255;
        if (pf_byte < 0)   pf_byte = 0;

        if (clh_v    > 0xFFFF) clh_v    = 0xFFFF;
        if (ctoday_v > 0xFFFF) ctoday_v = 0xFFFF;
        if (cyest_v  > 0xFFFF) cyest_v  = 0xFFFF;
        if (c2d_v    > 0xFFFF) c2d_v    = 0xFFFF;
        if (c3d_v    > 0xFFFF) c3d_v    = 0xFFFF;

        raw[0] = (unsigned char)pf_byte;

        raw[1] = (unsigned char)(econs_v & 0xFF);
        raw[2] = (unsigned char)((econs_v >> 8) & 0xFF);
        raw[3] = (unsigned char)((econs_v >> 16) & 0xFF);
        raw[4] = (unsigned char)((econs_v >> 24) & 0xFF);

        raw[5] = (unsigned char)(egen_v & 0xFF);
        raw[6] = (unsigned char)((egen_v >> 8) & 0xFF);
        raw[7] = (unsigned char)((egen_v >> 16) & 0xFF);
        raw[8] = (unsigned char)((egen_v >> 24) & 0xFF);

        raw[9]  = (unsigned char)(clh_v & 0xFF);
        raw[10] = (unsigned char)((clh_v >> 8) & 0xFF);
        raw[11] = (unsigned char)(ctoday_v & 0xFF);
        raw[12] = (unsigned char)((ctoday_v >> 8) & 0xFF);
        raw[13] = (unsigned char)(cyest_v & 0xFF);
        raw[14] = (unsigned char)((cyest_v >> 8) & 0xFF);
        raw[15] = (unsigned char)(c2d_v & 0xFF);
        raw[16] = (unsigned char)((c2d_v >> 8) & 0xFF);
        raw[17] = (unsigned char)(c3d_v & 0xFF);
        raw[18] = (unsigned char)((c3d_v >> 8) & 0xFF);

        b64_len = base64_encode(raw, sizeof(raw), b64);
        b64[b64_len] = '\0';

        B("\"e\":\"%s\",\"ev\":%d", b64, energy_version);
    }

    // ---- GRAPH ARRAYS (req=net | req=chginv) ----
    // Sent as base64 of packed binary, 1 byte per sample (48 bytes total):
    //   "net":    uint8, value = (clamp(net_Wh, -150, 300) + 150) / 2
    //             i.e. net_Wh = byte*2 - 150. Halves resolution to 2Wh
    //             steps but covers the full -150..+300 range in one byte.
    //   "chginv": int8 (signed), +v = charger at v%% (0..100),
    //             -v = inverter at v%% (0..100), 0 = neither.
    //             Charger and inverter are mutually exclusive so one
    //             signed byte replaces the previous two separate arrays.
    else if (has_ntp && req_param) {
        unsigned int msm      = NTP_GetHour() * 60 + NTP_GetMinute();
        const char  *key      = NULL;
        int          is_net   = 0;
        int          is_chginv = 0;
        int          net_live = 0, chg_live = 0, inv_live = 0;
        int          has_live = 0;

        if (strncmp(req_param, "req=net", 7) == 0) {
            key = "net"; is_net = 1;
            net_live = safe_int(real_consumption - real_export);
            has_live = 1;
        } else if (strncmp(req_param, "req=chginv", 10) == 0) {
            key = "chginv"; is_chginv = 1;
            has_live = (sample_count_30s > 0);
            if (has_live) {
                chg_live = current_charger_c_accum / sample_count_30s;
                inv_live = current_inverter_accum / sample_count_30s;
            }
        }

        if (key) {
            unsigned char raw[MATRIX_SIZE];
            char          b64[((MATRIX_SIZE) + 2) / 3 * 4 + 1];
            int           raw_len = 0;
            int           b64_len;

            for (int i = 47; i >= 0; i--) {
                int idx = (msm / net_metering_period - i + 96) % 96;
                int slot = idx % MATRIX_SIZE;

                if (is_net) {
                    if (i == 0 && has_live) {
                        int val = net_live;
                        if (val > 300)  val = 300;
                        if (val < -150) val = -150;
                        raw[raw_len++] = (unsigned char)((val + 150) / 2);
                    } else {
                        raw[raw_len++] = net_graph_matrix[slot];
                    }
                } else if (is_chginv) {
                    int chg_v = charger_c_matrix[slot];
                    int inv_v = inverter_matrix[slot];
                    int combined;
                    if (i == 0 && has_live) {
                        chg_v = chg_live;
                        inv_v = inv_live;
                    }
                    if (chg_v > 0)      combined = chg_v;
                    else if (inv_v > 0) combined = -inv_v;
                    else                combined = 0;
                    if (combined > 127)  combined = 127;
                    if (combined < -127) combined = -127;
                    raw[raw_len++] = (unsigned char)((signed char)combined);
                }
            }

            b64_len = base64_encode(raw, raw_len, b64);
            b64[b64_len] = '\0';
            B("\"%s\":\"%s\"", key, b64);
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
