#pragma once
#include "../httpserver/new_http.h"

/* Serves the JK-BMS live monitor HTML page at /bms */
int http_fn_serve_bms_html(http_request_t *request);
