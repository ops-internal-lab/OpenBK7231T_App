#pragma once

#include <stdint.h>
#include "../httpserver/new_http.h"

/* Sensor dataset indices — always 0 on single-meter builds */
#define BL_SENSORS_IX_0 0
#define BL_SENSORS_IX_1 1

void BL_Shared_Init(void);
void BL_ProcessUpdate(float voltage, float current, float power,
                      float frequency, float energyWh);

/* Remote multi-meter interface (BL0942 TCP poller <-> shared accounting). */
int  BL_GetMeterOctet(int slot);                                  /* 0 = unset */
int  BL_GetMeterInvert(int slot);        /* 1 = reverse-wired: flip W and energy */
float BL_GetMeterVoltCal(int slot);
float BL_GetMeterCurrentCal(int slot);
float BL_GetMeterPowerCal(int slot);
void  BL_SetMeterRaw(int slot, uint32_t raw_v, uint32_t raw_a, int32_t raw_w);
void BL_SetMeterReading(int slot, float v, float a, float w, float freq, int online);
/* Store a good reading PLUS this cycle's signed net energy taken from the
   chip's free-running signed CF-CNT delta. cf_valid=0 means "no usable delta
   this cycle" (first read after (re)connect, or a detected chip reset) — the
   reading is still latched, but it contributes 0 Wh to the sweep. */
void BL_SetMeterReadingCf(int slot, float v, float a, float w, float freq,
                          float cf_wh, int64_t cf_ticks, int cf_valid);
/* Signed net Wh contributed by `slot` this sweep (0 if no valid delta). */
float BL_MeterCfWh(int slot);
int64_t BL_MeterCfTicks(int slot);
/* A meter reported a CHIP RESET (MODE lost): the current 15-min interval's net
   is tainted, so discard it and let counting restart from now. */
void BL_MeterNoteReset(int slot);
/* Zero every slot's per-cycle CF energy after a sweep has consumed it, so a
   stalled/duplicate sweep cannot double-count. */
void BL_MeterCfConsume(void);
void BL_MeterReadFailed(int slot);     /* failed poll: keep last-good, age out */
int  BL_MeterOnlineState(int slot);    /* 0 offline / 1 fresh / 2 stale-holding */
int  BL_GetMeter(int slot, float *v, float *a, float *w, int *online);
void BL_ProcessSweep(void);                                       /* once per full sweep */

void BL09XX_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState);
void BL09XX_SaveEmeteringStatistics();


/* ===========================================================================
   MQTT publish snapshot  (consumed by drv_mqtt_stream.c)
   ---------------------------------------------------------------------------
   One-shot copy of every drv_bl_shared-owned value the MQTT streamer pushes,
   so the streamer never reaches into this driver's file-scope statics.
   Offline phase voltages come back as NAN so the streamer can skip them
   (a frozen/offline meter must not republish).
   =========================================================================== */
typedef struct {
    /* grid */
    float grid_l1_v, grid_l2_v, grid_l3_v;   /* NAN when that phase is offline  */
    float grid_power;                        /* signed sum of ONLINE phase W     */
    float net_energy;                        /* Wh (repurposed reactive slot)    */
    int   grid_import_total_wh, grid_export_total_wh;
    int   grid_import_lasthour_wh, grid_import_today_wh;
    int   grid_export_lasthour_wh, grid_export_today_wh;
    /* solar */
    float solar_power;                       /* signed sum of ONLINE solar W      */
    int   solar_lasthour_wh, solar_today_wh, solar_total_wh;
    /* ess / controller */
    float ess_ac_power;                      /* slot 5 W, signed (0 if offline)   */
    int   ess_charger_mode;                  /* 0 auto / 1 man-temp / 2 man-lock  */
    int   ess_divert_mode;                   /* 0 auto / 1 temp / 2 lock          */
    int   ess_inverter1_on;                  /* derived from persistent_state     */
    int   ess_inverter2_on;                  /* g_inv2_on                         */
    int   ess_charger_pwm;                   /* 0, or 10..100                     */
    int   ess_divert_on;                     /* divert_is_on                      */
    int   ess_inverter_gated;                /* BMS min-cell cut                  */
    int   ess_charger_gated;                 /* BMS max-cell cut                  */
} bl_pub_snapshot_t;

/* Fill *out with the current grouped values. Read-only; safe to call from
   another task (each field is a single-word copy). */
void BL_GetPublishSnapshot(bl_pub_snapshot_t *out);
