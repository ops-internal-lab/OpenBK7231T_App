#pragma once

/* ===========================================================================
   drv_ble_therm.h -- passive BLE listener for Xiaomi LYWSD03MMC thermometers
   flashed with ATC1441 or PVVX firmware (advertisement-only, never connects).

   Two sensors, selected by MAC address (console: `setThermMac 1 aa:bb:...`,
   persisted in NVS). Values are exposed to the MQTT streamer as
   therm1_temp / therm1_hum / therm1_batt and therm2_*.

   OpenBeken driver "BLETherm": start/stop at runtime like the streamer.

   BLE coexistence: NimBLE cannot start a *connection* (JK-BMS) while a scan
   is active, so jk_bms.c calls BLETherm_SuspendScan() right before each
   connect attempt and BLETherm_ResumeScan() once the attempt resolves.
   Suspend cancels the scan synchronously; resume just clears the flag and
   the scan restarts from the driver's next one-second tick.
   =========================================================================== */

void BLETherm_Start(void);          /* driver init (re-armable)               */
void BLETherm_Stop(void);           /* driver stop: cancels scan              */
void BLETherm_OnEverySecond(void);  /* driver tick: (re)starts scan if idle   */

void BLETherm_SuspendScan(void);    /* called by jk_bms before ble_gap_connect */
void BLETherm_ResumeScan(void);     /* called by jk_bms after connect resolves */

/* idx 0/1. Returns 1 and fills outputs if a frame was seen recently
   (<180 s), else 0. batt_pct may be NULL. */
int  BLETherm_Get(int idx, float *temp_c, float *hum_pct, int *batt_pct);

/* Configured MAC as "aa:bb:cc:dd:ee:ff" into out; returns 1 if set, else
   writes "" and returns 0. For the dashboard config Retrieve. */
int  BLETherm_GetMacStr(int idx, char *out, int outlen);

/* Diagnostics for the config UI. */
int  BLETherm_GetStats(int idx, int *secs_since_seen, int *avg_interval_s, int *rssi_dbm);
void BLETherm_ResetIntervalStats(void);
