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

/* ---- 12-hour graph matrix persistence ----
   Single consolidated blob under one key ("grph"), replacing the old five
   keys (grph_net/chg/inv/idx/ts). Two reasons:
     1. NVS cost: one ~210 B blob is ~8 entries; the old five keys were ~22
        entries live and ~22 dead entries of churn per 15-min save.
     2. Correctness: the old API declared the solar matrix as int* while the
        caller's array is unsigned char[48]; save read 192 B from a 48 B
        array and LOAD WROTE 192 B BACK INTO IT, overwriting ~144 bytes of
        adjacent globals (control-loop state) at every boot. The struct
        below fixes the types, and the battery matrix is stored as int16
        (values are clamped to +/-500 W) instead of a wasteful int32. */

#define GRAPH_BLOB_MAGIC   0x31485247u  /* 'G','R','H','1' little-endian */
#define GRAPH_BLOB_SLOTS   48

typedef struct {
	uint32_t      magic;                     /* GRAPH_BLOB_MAGIC            */
	int32_t       idx;                       /* last_matrix_index           */
	uint32_t      ts;                        /* save timestamp (epoch)      */
	unsigned char net  [GRAPH_BLOB_SLOTS];   /* packed net Wh bytes         */
	unsigned char solar[GRAPH_BLOB_SLOTS];   /* solar Wh/period, 0..150     */
	int16_t       ess  [GRAPH_BLOB_SLOTS];   /* avg battery W, +/-500       */
} graph_blob_t;                              /* 12 + 48 + 48 + 96 = 204 B   */

/* One-time cleanup of the legacy five-key layout (safe if keys are absent). */
static void graph_erase_legacy_keys(nvs_handle_t h)
{
	nvs_erase_key(h, "grph_net");
	nvs_erase_key(h, "grph_chg");
	nvs_erase_key(h, "grph_inv");
	nvs_erase_key(h, "grph_idx");
	nvs_erase_key(h, "grph_ts");
}

void HAL_FlashVars_SaveGraphMatrices(const unsigned char *net_graph,
                                     const unsigned char *solar, const int *ess_w,
                                     int size, int idx, unsigned int ts)
{
	graph_blob_t b;
	int i, n = (size < GRAPH_BLOB_SLOTS) ? size : GRAPH_BLOB_SLOTS;
	memset(&b, 0, sizeof(b));
	b.magic = GRAPH_BLOB_MAGIC;
	b.idx   = (int32_t)idx;
	b.ts    = (uint32_t)ts;
	memcpy(b.net,   net_graph, (size_t)n);
	memcpy(b.solar, solar,     (size_t)n);
	for (i = 0; i < n; i++) {
		int v = ess_w[i];
		if (v >  32767) v =  32767;
		if (v < -32768) v = -32768;
		b.ess[i] = (int16_t)v;
	}
	InitFlashIfNeeded();
	nvs_handle_t h = 0;
	if (nvs_open("config", NVS_READWRITE, &h) != ESP_OK) return;
	graph_erase_legacy_keys(h);
	esp_err_t rc = nvs_set_blob(h, "grph", &b, sizeof(b));
	if (rc == ESP_OK) rc = nvs_commit(h);
	if (rc != ESP_OK)
		ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER, "graph save failed rc=0x%x", rc);
	nvs_close(h);
}

int HAL_FlashVars_LoadGraphMatrices(unsigned char *net_graph,
                                    unsigned char *solar, int *ess_w,
                                    int size, int *idx, unsigned int *ts)
{
	graph_blob_t b;
	size_t sz = sizeof(b);
	int i, n = (size < GRAPH_BLOB_SLOTS) ? size : GRAPH_BLOB_SLOTS;
	InitFlashIfNeeded();
	nvs_handle_t h = 0;
	if (nvs_open("config", NVS_READONLY, &h) != ESP_OK) return 0;
	esp_err_t rc = nvs_get_blob(h, "grph", &b, &sz);
	nvs_close(h);
	/* Absent, wrong size, or wrong magic -> fresh start (zeroed matrices).
	   The legacy five-key layout is deliberately NOT migrated: its "chg"
	   blob contained out-of-bounds garbage (see comment above), so the old
	   data was never trustworthy. Keys are erased on the next save. */
	if (rc != ESP_OK || sz != sizeof(b) || b.magic != GRAPH_BLOB_MAGIC) return 0;
	memcpy(net_graph, b.net,   (size_t)n);
	memcpy(solar,     b.solar, (size_t)n);
	for (i = 0; i < n; i++) ess_w[i] = (int)b.ess[i];
	*idx = (int)b.idx;
	*ts  = (unsigned int)b.ts;
	return 1;
}

#endif // PLATFORM_ESPIDF
