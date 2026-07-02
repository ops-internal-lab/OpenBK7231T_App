#pragma once

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
void BL_SetMeterReading(int slot, float v, float a, float w, float freq, int online);
/* Store a good reading PLUS this cycle's signed net energy taken from the
   chip's free-running signed CF-CNT delta. cf_valid=0 means "no usable delta
   this cycle" (first read after (re)connect, or a detected chip reset) — the
   reading is still latched, but it contributes 0 Wh to the sweep. */
void BL_SetMeterReadingCf(int slot, float v, float a, float w, float freq,
                          float cf_wh, int cf_valid);
/* Signed net Wh contributed by `slot` this sweep (0 if no valid delta). */
float BL_MeterCfWh(int slot);
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

