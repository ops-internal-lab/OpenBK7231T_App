#ifndef DASH_FRONTEND_H
#define DASH_FRONTEND_H

// Make sure to include the header that defines http_request_t in your project
// (This is usually http_server.h or drv_http.h in OpenBeken)
#include "../httpserver/new_http.h"

void send_dash_javascript(http_request_t *request);

#endif // DASH_FRONTEND_H
