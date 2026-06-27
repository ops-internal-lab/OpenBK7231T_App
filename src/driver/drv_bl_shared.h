#pragma once

#include "../httpserver/new_http.h"

/* Sensor dataset indices — always 0 on single-meter builds */
#define BL_SENSORS_IX_0 0
#define BL_SENSORS_IX_1 1

void BL_Shared_Init(void);
void BL_ProcessUpdate(float voltage, float current, float power,
                      float frequency, float energyWh);
void BL09XX_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState);
void BL09XX_SaveEmeteringStatistics();

