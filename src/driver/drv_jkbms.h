#pragma once

#ifdef ENABLE_JK_BMS

#include "../httpserver/new_http.h"
#include "jk_bms.h"

/* Start the BLE connection to the JK-BMS (called once at boot). */
void JKBMS_AutoStart(void);

/* Snapshot accessor for other modules (e.g. the /dash payload).
   Returns 1 and fills *out when connected with data, else 0. */
int JKBMS_GetData(jk_bms_data_t *out);

/* Static MAC string of the configured BMS, e.g. "C8:47:80:1A:18:B5". */
const char *JKBMS_GetMac(void);

/* HTTP handlers registered in new_http.c */
int http_fn_api_bms(http_request_t *request);   /* GET /api_bms  → JSON data  */
int http_fn_bms_page(http_request_t *request);  /* GET /bms      → HTML page  */

#endif /* ENABLE_JK_BMS */
