#if PLATFORM_ESPIDF|| PLATFORM_ESP8266

#include "../../new_cfg.h"
#include "../../logging/logging.h"
#include "../../new_common.h"
#include "../hal_flashVars.h"
#include "nvs_flash.h"
#include "nvs.h"

void InitFlashIfNeeded();

void HAL_FlashVars_IncreaseBootCount()
{
	uint32_t bootc = 0;
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READWRITE, &handle);
	nvs_get_u32(handle, "bootc", &bootc);
	nvs_set_u32(handle, "bootc", ++bootc);
	nvs_commit(handle);
	nvs_close(handle);
}

int HAL_FlashVars_GetChannelValue(int ch)
{
	char channel[6];
	sprintf(channel, "ch%i", ch);
	int32_t value = 0;
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READONLY, &handle);
	nvs_get_i32(handle, channel, &value);
	nvs_close(handle);
	return value;
}

void HAL_FlashVars_SaveChannel(int index, int value)
{
	char channel[6];
	sprintf(channel, "ch%i", index);
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READWRITE, &handle);
	nvs_set_i32(handle, channel, value);
	nvs_commit(handle);
	nvs_close(handle);
}

void HAL_FlashVars_ReadLED(byte* mode, short* brightness, short* temperature, byte* rgb, byte* bEnableAll)
{
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READONLY, &handle);
	nvs_get_u8(handle, "mode", mode);
	nvs_get_i16(handle, "brs", brightness);
	nvs_get_i16(handle, "temp", temperature);
	nvs_get_u8(handle, "r", &rgb[0]);
	nvs_get_u8(handle, "g", &rgb[1]);
	nvs_get_u8(handle, "b", &rgb[2]);
	nvs_get_u8(handle, "ena", bEnableAll);
	nvs_close(handle);
}


void HAL_FlashVars_SaveLED(byte mode, short brightness, short temperature, byte r, byte g, byte b, byte bEnableAll)
{
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READWRITE, &handle);
	nvs_set_u8(handle, "mode", mode);
	nvs_set_i16(handle, "brs", brightness);
	nvs_set_i16(handle, "temp", temperature);
	nvs_set_u8(handle, "r", r);
	nvs_set_u8(handle, "g", g);
	nvs_set_u8(handle, "b", b);
	nvs_set_u8(handle, "ena", bEnableAll);
	nvs_commit(handle);
	nvs_close(handle);
}

short HAL_FlashVars_ReadUsage()
{
	short usage = 0;
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READONLY, &handle);
	nvs_get_i16(handle, "tu", &usage);
	nvs_close(handle);
	return usage;
}

void HAL_FlashVars_SaveTotalUsage(short usage)
{
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READWRITE, &handle);
	nvs_set_i16(handle, "tu", usage);
	nvs_commit(handle);
	nvs_close(handle);
}

void HAL_FlashVars_SaveBootComplete()
{
	uint32_t bootc = 0;
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READWRITE, &handle);
	nvs_get_u32(handle, "bootc", &bootc);
	nvs_set_u32(handle, "bootsc", bootc);
	nvs_commit(handle);
	nvs_close(handle);
}

// call to return the number of boots since a HAL_FlashVars_SaveBootComplete
int HAL_FlashVars_GetBootFailures()
{
	uint32_t bootc = 0, bootsc = 0;
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READONLY, &handle);
	nvs_get_u32(handle, "bootc", &bootc);
	nvs_get_u32(handle, "bootsc", &bootsc);
	nvs_close(handle);
	return bootc - bootsc;
}

int HAL_FlashVars_GetBootCount()
{
	uint32_t bootc = 0;
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READONLY, &handle);
	nvs_get_u32(handle, "bootc", &bootc);
	nvs_close(handle);
	return bootc;
}

int HAL_GetEnergyMeterStatus(ENERGY_METERING_DATA* data)
{
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READONLY, &handle);
	size_t size = sizeof(ENERGY_METERING_DATA);   /* was sizeof(data) — pointer bug fixed */
	nvs_get_blob(handle, "emd", data, &size);
	nvs_close(handle);
	return 0;
}

int HAL_SetEnergyMeterStatus(ENERGY_METERING_DATA* data)
{
	InitFlashIfNeeded();
	nvs_handle_t handle = 0;
	nvs_open("config", NVS_READWRITE, &handle);
	nvs_set_blob(handle, "emd", data, sizeof(ENERGY_METERING_DATA)); /* fixed */
	nvs_commit(handle);
	nvs_close(handle);
	return 0;
}

void HAL_FlashVars_SaveTotalConsumption(float total_consumption)
{
	/* kept for API compatibility; the full struct save via HAL_SetEnergyMeterStatus
	   is the preferred path in drv_bl_shared.c */
}

/* ---- helpers: float get/set by key ---- */
static void nvs_set_float(const char *key, float v)
{
	InitFlashIfNeeded();
	nvs_handle_t h = 0;
	nvs_open("config", NVS_READWRITE, &h);
	nvs_set_blob(h, key, &v, sizeof(float));
	nvs_commit(h);
	nvs_close(h);
}

static float nvs_get_float(const char *key)
{
	float v = 0.0f;
	InitFlashIfNeeded();
	nvs_handle_t h = 0;
	nvs_open("config", NVS_READONLY, &h);
	size_t sz = sizeof(float);
	nvs_get_blob(h, key, &v, &sz);
	nvs_close(h);
	return v;
}

/* ---- Import / Export lifetime totals ---- */
void  HAL_FlashVars_SaveEnergyImportTotal(float v) { nvs_set_float("eImpTotal", v); }
float HAL_FlashVars_GetEnergyImportTotal(void)      { return nvs_get_float("eImpTotal"); }
void  HAL_FlashVars_SaveEnergyExportTotal(float v) { nvs_set_float("eExpTotal", v); }
float HAL_FlashVars_GetEnergyExportTotal(void)      { return nvs_get_float("eExpTotal"); }

/* ---- Daily import history (daysAgo 0=today .. 3=3d ago) ---- */
static const char * const s_imp_day_keys[4] = {"eImpD0","eImpD1","eImpD2","eImpD3"};
static const char * const s_exp_day_keys[4] = {"eExpD0","eExpD1","eExpD2","eExpD3"};

void HAL_FlashVars_SaveEnergyImportDaily(int daysAgo, float v)
{
	if (daysAgo < 0 || daysAgo > 3) return;
	nvs_set_float(s_imp_day_keys[daysAgo], v);
}
float HAL_FlashVars_GetEnergyImportDaily(int daysAgo)
{
	if (daysAgo < 0 || daysAgo > 3) return 0.0f;
	return nvs_get_float(s_imp_day_keys[daysAgo]);
}
void HAL_FlashVars_SaveEnergyExportDaily(int daysAgo, float v)
{
	if (daysAgo < 0 || daysAgo > 3) return;
	nvs_set_float(s_exp_day_keys[daysAgo], v);
}
float HAL_FlashVars_GetEnergyExportDaily(int daysAgo)
{
	if (daysAgo < 0 || daysAgo > 3) return 0.0f;
	return nvs_get_float(s_exp_day_keys[daysAgo]);
}

/* ---- 12-hour graph matrix persistence ---- */
void HAL_FlashVars_SaveGraphMatrices(unsigned char *net_graph,
                                     int *chg, int *inv,
                                     int size, int idx, unsigned int ts)
{
	size_t net_sz = (size_t)size;
	size_t int_sz = (size_t)(size * (int)sizeof(int));
	InitFlashIfNeeded();
	nvs_handle_t h = 0;
	nvs_open("config", NVS_READWRITE, &h);
	nvs_set_blob(h, "grph_net", net_graph, net_sz);
	nvs_set_blob(h, "grph_chg", chg, int_sz);
	nvs_set_blob(h, "grph_inv", inv, int_sz);
	nvs_set_i32(h,  "grph_idx", (int32_t)idx);
	nvs_set_u32(h,  "grph_ts",  (uint32_t)ts);
	nvs_commit(h);
	nvs_close(h);
}

int HAL_FlashVars_LoadGraphMatrices(unsigned char *net_graph,
                                    int *chg, int *inv,
                                    int size, int *idx, unsigned int *ts)
{
	size_t net_sz = (size_t)size;
	size_t int_sz = (size_t)(size * (int)sizeof(int));
	int32_t  saved_idx = 0;
	uint32_t saved_ts  = 0;
	esp_err_t err;
	InitFlashIfNeeded();
	nvs_handle_t h = 0;
	nvs_open("config", NVS_READONLY, &h);
	err  = nvs_get_blob(h, "grph_net", net_graph, &net_sz);
	int_sz = (size_t)(size * (int)sizeof(int));
	err |= nvs_get_blob(h, "grph_chg", chg, &int_sz);
	int_sz = (size_t)(size * (int)sizeof(int));
	err |= nvs_get_blob(h, "grph_inv", inv, &int_sz);
	err |= nvs_get_i32(h,  "grph_idx", &saved_idx);
	err |= nvs_get_u32(h,  "grph_ts",  &saved_ts);
	nvs_close(h);
	if (err != ESP_OK) return 0;
	*idx = (int)saved_idx;
	*ts  = (unsigned int)saved_ts;
	return 1;
}

#endif // PLATFORM_ESPIDF
