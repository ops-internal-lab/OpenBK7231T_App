/* ===========================================================================
   jk_bms.h  --  JK-BMS BLE driver for ESP-IDF (NimBLE GATT client)
   ---------------------------------------------------------------------------
   Read-only driver for a JK-BMS speaking the JK02_32S protocol over BLE.
   You give it a MAC and a callback; it connects, subscribes, asks the BMS to
   stream, reassembles + decodes the frames, and hands you a filled struct.

   Usage:
       static void on_update(const jk_bms_data_t *d, void *ctx) { ... }

       jk_bms_config_t cfg = {
           .mac       = "c8:47:80:1a:18:b5",
           .poll_ms   = 2000,
           .on_update = on_update,
           .user_ctx  = NULL,
       };
       jk_bms_start(&cfg);

   Requires the NimBLE host (see sdkconfig.defaults). ESP-IDF v5.x.
   =========================================================================== */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One decoded cell-info snapshot. */
typedef struct {
    float total_voltage;          /* pack voltage, V                       */
    float current;                /* + charge / - discharge, A             */
    int   soc;                    /* state of charge, %                    */
    float remaining_ah;           /* coulomb-counted remaining, Ah         */
    float full_charge_ah;         /* BMS learned full-charge capacity, Ah  */
    float temp_1, temp_2;         /* battery sensors, deg C                */
    float temp_mosfet;            /* MOSFET temperature, deg C             */
    float balance_current;        /* A (signed)                            */
    bool  charge_enabled;         /* charge MOSFET switch                  */
    bool  discharge_enabled;      /* discharge MOSFET switch               */
    bool  balancer_enabled;       /* balancer switch                       */
    int   balancer_action;        /* 0 off / 1 charging / 2 discharging    */
    int   cell_count;             /* number of active cells                */
    float cell_min, cell_max;     /* V                                     */
    float cells[32];              /* per-cell voltages (first cell_count)  */
    float balance_start_voltage;  /* from settings frame; 0 until received */
} jk_bms_data_t;

/* Called from the driver task on every decoded cell-info frame. */
typedef void (*jk_bms_cb_t)(const jk_bms_data_t *data, void *user_ctx);

typedef struct {
    const char *mac;          /* "c8:47:80:1a:18:b5" (lowercase, public addr) */
    uint32_t    poll_ms;      /* cell-info re-poll period; 0 -> 2000 ms        */
    jk_bms_cb_t on_update;    /* required                                     */
    void       *user_ctx;     /* passed back to the callback                  */
} jk_bms_config_t;

/* Initialise NimBLE and start connecting. Returns once the host is launched;
   data arrives later via the callback. Call once. NVS must be initialised
   beforehand (nvs_flash_init()). */
esp_err_t jk_bms_start(const jk_bms_config_t *config);

/* True while a BLE link to the BMS is up. */
bool jk_bms_is_connected(void);

#ifdef __cplusplus
}
#endif
