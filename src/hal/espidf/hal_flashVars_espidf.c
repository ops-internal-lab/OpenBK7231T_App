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

#define GRAPH_BLOB_MAGIC   0x33485247u  /* 'G','R','H','3' little-endian.
   v2: ess[] switched from plain int16 W to a packed uint16 per slot:
       bits 0-8 = |W| (clamped 500), bit 9 = sign (1 = discharge),
       bits 10-15 = battery SOC as (soc%/2)+1 (1..51, 0 = no sample).
   v3: the BLE-thermometer daily range moved in here (th_hi/th_lo/th_date),
       replacing the separate "thmn" key. It fits in space this blob already
       pays for: NVS allocates blob data in 32-byte entries, so 204 B occupied
       7 entries = 224 B of capacity with 20 B unused. The range costs 10 of
       those 20, so the entry count, the page footprint and the write cost are
       all unchanged -- the temperatures ride along on a write that happens
       anyway. A v1/v2 blob fails the magic check -> one-time fresh start of
       the 12-h graph history (and an empty range) after upgrading. */
#define GRAPH_BLOB_SLOTS   48

/* Sentinel for "no reading yet today" -- -3276.8 C is not a value a sensor
   can report, so it doubles as the seeded flag at zero storage cost. */
#define THERM_TENTHS_NONE  ((int16_t)-32768)

typedef struct {
	uint32_t      magic;                     /* GRAPH_BLOB_MAGIC            */
	int32_t       idx;                       /* last_matrix_index           */
	uint32_t      ts;                        /* save timestamp (epoch)      */
	unsigned char net  [GRAPH_BLOB_SLOTS];   /* packed net Wh bytes         */
	unsigned char solar[GRAPH_BLOB_SLOTS];   /* solar Wh/period, 0..150     */
	uint16_t      ess  [GRAPH_BLOB_SLOTS];   /* packed: |W|,sign,soc6       */
	int16_t       th_hi[2];                  /* today's max, tenths degC    */
	int16_t       th_lo[2];                  /* today's min, tenths degC    */
	uint16_t      th_date;                   /* packed y7|m4|d5, 0 = none   */
} graph_blob_t;                              /* 204 + 10 = 214 -> 216 padded */

/* The range is carried in a shadow copy rather than through the Save/Load
   parameter lists. That keeps every existing graph call site untouched: the
   thermometer driver pushes new values here whenever they change, and the
   next graph save picks them up automatically. */
static int16_t  s_th_hi[2]  = { THERM_TENTHS_NONE, THERM_TENTHS_NONE };
static int16_t  s_th_lo[2]  = { THERM_TENTHS_NONE, THERM_TENTHS_NONE };
static uint16_t s_th_date   = 0;

void HAL_FlashVars_SetThermShadow(const short *hi, const short *lo, unsigned short date)
{
	s_th_hi[0] = (int16_t)hi[0]; s_th_hi[1] = (int16_t)hi[1];
	s_th_lo[0] = (int16_t)lo[0]; s_th_lo[1] = (int16_t)lo[1];
	s_th_date  = (uint16_t)date;
}

void HAL_FlashVars_GetThermShadow(short *hi, short *lo, unsigned short *date)
{
	hi[0] = (short)s_th_hi[0]; hi[1] = (short)s_th_hi[1];
	lo[0] = (short)s_th_lo[0]; lo[1] = (short)s_th_lo[1];
	if (date) *date = (unsigned short)s_th_date;
}

/* One-time cleanup of superseded keys (safe if any are absent).
   "thmn" was the short-lived standalone min/max blob, now folded into grph. */
static void graph_erase_legacy_keys(nvs_handle_t h)
{
	nvs_erase_key(h, "grph_net");
	nvs_erase_key(h, "grph_chg");
	nvs_erase_key(h, "grph_inv");
	nvs_erase_key(h, "grph_idx");
	nvs_erase_key(h, "grph_ts");
	nvs_erase_key(h, "thmn");
}

void HAL_FlashVars_SaveGraphMatrices(const unsigned char *net_graph,
                                     const unsigned char *solar, const int *ess_w,
                                     const unsigned char *socm,
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
		int w   = ess_w[i];
		int mag = (w < 0) ? -w : w;
		int s6  = socm ? (socm[i] & 0x3F) : 0;
		if (mag > 500) mag = 500;
		b.ess[i] = (uint16_t)((mag & 0x1FF) | ((w < 0) ? 0x200 : 0)
		                      | (s6 << 10));
	}
	/* Thermometer daily range rides along from the shadow copy. */
	b.th_hi[0] = s_th_hi[0]; b.th_hi[1] = s_th_hi[1];
	b.th_lo[0] = s_th_lo[0]; b.th_lo[1] = s_th_lo[1];
	b.th_date  = s_th_date;
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
                                    unsigned char *socm,
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
	/* Stage the thermometer range. This runs at boot, before NTP, so it is
	   only parked in the shadow -- the driver adopts it later, on the first
	   synced sweep, once th_date can be compared against a real date. */
	s_th_hi[0] = b.th_hi[0]; s_th_hi[1] = b.th_hi[1];
	s_th_lo[0] = b.th_lo[0]; s_th_lo[1] = b.th_lo[1];
	s_th_date  = b.th_date;
	memcpy(net_graph, b.net,   (size_t)n);
	memcpy(solar,     b.solar, (size_t)n);
	for (i = 0; i < n; i++) {
		unsigned int enc = b.ess[i];
		int mag = (int)(enc & 0x1FF);
		ess_w[i] = (enc & 0x200) ? -mag : mag;
		if (socm) socm[i] = (unsigned char)((enc >> 10) & 0x3F);
	}
	*idx = (int)b.idx;
	*ts  = (unsigned int)b.ts;
	return 1;
}


#endif // PLATFORM_ESPIDF
