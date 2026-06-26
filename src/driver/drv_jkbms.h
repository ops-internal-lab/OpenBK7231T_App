#pragma once

#ifdef ENABLE_JK_BMS

#include "../httpserver/new_http.h"

/* Start the BLE connection to the JK-BMS (called once at boot). */
void JKBMS_AutoStart(void);

/* HTTP handlers registered in new_http.c */
int http_fn_api_bms(http_request_t *request);   /* GET /api_bms  → JSON data  */
int http_fn_bms_page(http_request_t *request);  /* GET /bms      → HTML page  */

#endif /* ENABLE_JK_BMS */
