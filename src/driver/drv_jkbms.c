/* new_common.h must come first so ENABLE_JK_BMS is defined before the guard. */
#include "../new_common.h"

/* ===========================================================================
   drv_jkbms.c  --  OpenBK integration for the JK-BMS BLE driver

     GET /bms       → live monitor HTML page (see bms_frontend.c)
     GET /api_bms   → JSON snapshot of the latest decoded BMS frame

   Auto-started from hal_main_espidf.c when ENABLE_JK_BMS is defined.
   The BMS MAC address is hard-coded to C8:47:80:1A:18:B5.
   =========================================================================== */

#ifdef ENABLE_JK_BMS

#include "drv_jkbms.h"
#include "bms_frontend.h"
#include "jk_bms.h"
#include "../httpserver/new_http.h"
#include "../logging/logging.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

/* ---- static BMS state ---------------------------------------------------- */

static jk_bms_data_t s_data;
static volatile int  s_has_data  = 0;

/* ---- BMS callback (called from the NimBLE task on every frame) ----------- */

static void on_bms_update(const jk_bms_data_t *d, void *ctx)
{
    (void)ctx;
    memcpy(&s_data, d, sizeof(s_data));
    s_has_data = 1;
}

/* ---- public: auto-start -------------------------------------------------- */

void JKBMS_AutoStart(void)
{
    static int started = 0;
    if (started) return;
    started = 1;

    jk_bms_config_t cfg = {
        .mac       = "c8:47:80:1a:18:b5",
        .poll_ms   = 2000,
        .on_update = on_bms_update,
        .user_ctx  = NULL,
    };

    int err = (int)jk_bms_start(&cfg);
    if (err != 0) {
        ADDLOG_ERROR(LOG_FEATURE_GENERAL,
            "JK-BMS: jk_bms_start failed (err=%d)", err);
    } else {
        ADDLOG_INFO(LOG_FEATURE_GENERAL,
            "JK-BMS: BLE connecting to c8:47:80:1a:18:b5");
    }
}

/* ---- /api_bms → JSON ----------------------------------------------------- */
/*
   Response when not yet connected:
     {"ok":1,"connected":0}

   Response when live data is available:
     {"ok":1,"connected":1,
      "soc":85,"v":52.400,"a":-3.200,
      "rem":85.0,"full":100.0,
      "nc":16,"vmin":3.271,"vmax":3.283,"dmv":12,
      "t1":25.3,"t2":24.8,"tmos":28.1,
      "chg":1,"dis":1,"bal":0,
      "balA":0.000,"bstart":3.500}
*/
int http_fn_api_bms(http_request_t *request)
{
    char buf[512];
    int  pos = 0;

    http_setup(request, "application/json");

    int connected = jk_bms_is_connected() && s_has_data;

    if (!connected) {
        pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "{\"ok\":1,\"connected\":0}");
    } else {
        /* Take a safe copy while the NimBLE task might update concurrently. */
        jk_bms_data_t d;
        memcpy(&d, &s_data, sizeof(d));

        int dmv = (d.cell_count > 0)
                  ? (int)((d.cell_max - d.cell_min) * 1000.0f + 0.5f)
                  : 0;

        pos += snprintf(buf + pos, sizeof(buf) - pos,
            "{\"ok\":1,\"connected\":1"
            ",\"soc\":%d"
            ",\"v\":%.3f,\"a\":%.3f"
            ",\"rem\":%.1f,\"full\":%.1f"
            ",\"nc\":%d,\"vmin\":%.3f,\"vmax\":%.3f,\"dmv\":%d"
            ",\"t1\":%.1f,\"t2\":%.1f,\"tmos\":%.1f"
            ",\"chg\":%d,\"dis\":%d,\"bal\":%d"
            ",\"balA\":%.3f,\"bstart\":%.3f"
            "}",
            d.soc,
            d.total_voltage, d.current,
            d.remaining_ah, d.full_charge_ah,
            d.cell_count, d.cell_min, d.cell_max, dmv,
            d.temp_1, d.temp_2, d.temp_mosfet,
            d.charge_enabled   ? 1 : 0,
            d.discharge_enabled? 1 : 0,
            d.balancer_enabled ? 1 : 0,
            d.balance_current,
            d.balance_start_voltage
        );
    }

    buf[pos] = '\0';
    poststr(request, buf);
    poststr(request, NULL);
    return 0;
}

/* ---- /bms → live monitor HTML page --------------------------------------- */

int http_fn_bms_page(http_request_t *request)
{
    return http_fn_serve_bms_html(request);
}

#endif /* ENABLE_JK_BMS */
