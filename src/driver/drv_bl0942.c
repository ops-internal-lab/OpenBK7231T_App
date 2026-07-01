#include "drv_bl0942.h"

#include <math.h>
#include <stdint.h>

#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#if PLATFORM_ESPIDF
#include "drv_uart_tcp_client.h"
#endif
#include "../cmnds/cmd_public.h"
#include "drv_bl_shared.h"
#include "drv_pwrCal.h"
#include "drv_spi.h"
#include "drv_uart.h"

static unsigned short bl0942_baudRate = 4800;

#define BL0942_UART_RECEIVE_BUFFER_SIZE 256
#define BL0942_UART_ADDR 0 // 0 - 3
#define BL0942_UART_CMD_READ(addr) (0x58 | addr)
#define BL0942_UART_CMD_WRITE(addr) (0xA8 | addr)
#define BL0942_UART_REG_PACKET 0xAA
#define BL0942_UART_PACKET_HEAD 0x55
#define BL0942_UART_PACKET_LEN 23

// Datasheet says 900 kHz is supported, but it produced ~50% check sum errors  
#define BL0942_SPI_BAUD_RATE 800000 // 900000
#define BL0942_SPI_CMD_READ 0x58
#define BL0942_SPI_CMD_WRITE 0xA8

// Electric parameter register (read only)
#define BL0942_REG_I_RMS 0x03
#define BL0942_REG_V_RMS 0x04
#define BL0942_REG_WATT 0x06
#define BL0942_REG_CF_CNT 0x07
#define BL0942_REG_FREQ 0x08
#define BL0942_REG_USR_WRPROT 0x1D
#define BL0942_USR_WRPROT_DISABLE 0x55

// User operation register (read and write)
#define BL0942_REG_WA_CREEP 0x14	// Minimun power measurement register
#define BL0942_REG_MODE 0x19
#define BL0942_REG_CF_CNT_CLR_SEL
// Bit 6 (CF_CNT_CLR_SEL)=1: clear CF_CNT after every read, so each read is a
//   small per-interval delta (avoids the 24-bit counter ever rolling over).
// Bit 7 (CF_CNT_ADD_SEL)=0: ALGEBRAIC accumulation. The counter now nets
//   +import / -export internally, so cf_cnt is a SIGNED per-read energy delta
//   (read it two's-complement). Direction no longer needs the WATT sign.
#define BL0942_MODE_DEFAULT 0x47	
#define BL0942_MODE_RMS_UPDATE_SEL_800_MS (1 << 3)

// Minimun power measurement value. Can be in the range of 0 to 255.
// below this value, consumption is reported at zero. Ideal for parasitic loads or where
// the device measures it's own power concumption
#define DEFAULT_WA_CREEP_VAL 64		

#define DEFAULT_VOLTAGE_CAL 15188
#define DEFAULT_CURRENT_CAL 251210
#define DEFAULT_POWER_CAL 598

#define CF_CNT_INVALID (1 << 31)

// How long to wait for a full response packet before giving up.
// At 4800 baud a 23-byte packet takes ~48 ms to arrive, so 100 ms is a
// comfortable ceiling that still catches a dead/missing device quickly.
#define BL0942_RESPONSE_TIMEOUT_MS 100

// Granularity of the poll loop — how often we check the buffer.
// Small enough to return quickly when data arrives, large enough to yield
// to other RTOS tasks between checks (delay_ms yields the scheduler).
#define BL0942_POLL_INTERVAL_MS    5

typedef struct {
    uint32_t i_rms;
    uint32_t v_rms;
    int32_t watt;
    int32_t cf_cnt;
    uint32_t freq;
} bl0942_data_t;

static uint32_t PrevCfCnt = CF_CNT_INVALID;

// Counts consecutive seconds with no valid response — used only for logging
// "device offline" once and "device back online" when it recovers.
static int g_offlineSec = 0;

static int32_t Int24ToInt32(int32_t val) {
    return (val & (1 << 23) ? val | (0xFF << 24) : val);
}

static void ScaleAndUpdate(bl0942_data_t *data) {
    float voltage, current, power;
    PwrCal_Scale(data->v_rms, data->i_rms, data->watt, &voltage, &current,
                 &power);

    // Guard power against a non-finite/absurd reading (it feeds (int)power
    // downstream). Hold the last good value rather than letting NaN/inf or
    // a wild spike propagate. Voltage/current are display-only and bounded
    // by the chip's RMS registers, so they don't need the same treatment.
    #define BL0942_MAX_SANE_POWER_W 30000.0f
    static float lastGoodPower = 0.0f;
    if (!isfinite(power) || power > BL0942_MAX_SANE_POWER_W || power < -BL0942_MAX_SANE_POWER_W) {
        power = lastGoodPower;
    } else {
        lastGoodPower = power;
    }
    // data->freq can read 0 on a glitched/failed register read; avoid a
    // divide-by-zero (which would yield inf/NaN). Report 0 Hz instead.
    float frequency = (data->freq != 0) ? (2 * 500000.0f / data->freq) : 0.0f;
    float energyWh = 0;
    // cf_cnt is now a SIGNED per-read delta (algebraic accumulation, MODE bit7=0):
    // positive = import, negative = export. Keep the sign — do NOT fabsf() it.
    energyWh = PwrCal_ScalePowerOnly(data->cf_cnt) * 1638.4f * 256.0f / 3600.0f;

    // Glitch guard: cf_cnt is reset on every read, so energyWh is the energy of a
    // single ~1s sample - a small value (either sign). If a read is missed or
    // corrupted the magnitude can come back enormous or non-finite, which later
    // poisons the float->int casts downstream and can crash. If the MAGNITUDE is
    // not sane, discard the sample and reuse the last known good value (zero would
    // dip the running total). Negative is now VALID (export), so guard |energyWh|.
    #define BL0942_MAX_SANE_ENERGY_WH 50.0f   // ~180kW for 1s; far above any real load
    static float lastGoodEnergyWh = 0.0f;
    if (!isfinite(energyWh) || fabsf(energyWh) > BL0942_MAX_SANE_ENERGY_WH) {
        ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                    "BL0942 energyWh glitch (%f), holding last good value\n", energyWh);
        energyWh = lastGoodEnergyWh;
    } else {
        lastGoodEnergyWh = energyWh;
    }

    // Apply sign convention
    float signedPower = CFG_HasFlag(OBK_FLAG_POWER_INVERT_AC) ? (-1.0f * power) : power;
    // energyWh is signed (algebraic cf_cnt); apply the SAME invert as the power
    // so energy and power agree on import(+)/export(-).
    if (CFG_HasFlag(OBK_FLAG_POWER_INVERT_AC)) energyWh = -1.0f * energyWh;

    // ====================================================================
    // 10-SECOND TICK LOGIC (INSTANTANEOUS SENSORS + ACCUMULATED ENERGY)
    // ====================================================================
    #define SAMPLES_PER_UPDATE 5

    static int   sampleCount = 0;
    static float energyAccum = 0.0f;

    // Energy must always be summed so consumption data is not lost between updates
    energyAccum += energyWh;
    sampleCount++;

    if (sampleCount < SAMPLES_PER_UPDATE) {
        return; // Do nothing else until the 10th call
    }

    // On the 10th call, pass instantaneous readings from THIS exact sample,
    // alongside the total energy accumulated over the last 10 samples.
    float totalEnergyWh = energyAccum;

    BL_ProcessUpdate(voltage, current, signedPower, frequency, totalEnergyWh);

    // Reset counters for the next 10-second window
    energyAccum = 0.0f;
    sampleCount = 0;
}

static int UART_TryToGetNextPacket(void) {
	int cs;
	int i;
	int c_garbage_consumed = 0;
	byte checksum;

	cs = UART_GetDataSize();

	if(cs < BL0942_UART_PACKET_LEN) {
		return 0;
	}
	// skip garbage data (should not happen)
	while(cs > 0) {
        if (UART_GetByte(0) != BL0942_UART_PACKET_HEAD) {
			UART_ConsumeBytes(1);
			c_garbage_consumed++;
			cs--;
		} else {
			break;
		}
	}
	if(c_garbage_consumed > 0){
        ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                    "Consumed %i unwanted non-header byte in BL0942 buffer\n",
                    c_garbage_consumed);
	}
	if(cs < BL0942_UART_PACKET_LEN) {
		return 0;
	}
    if (UART_GetByte(0) != 0x55)
		return 0;
    checksum = BL0942_UART_CMD_READ(BL0942_UART_ADDR);

    for(i = 0; i < BL0942_UART_PACKET_LEN-1; i++) {
        checksum += UART_GetByte(i);
	}
	checksum ^= 0xFF;

    if (checksum != UART_GetByte(BL0942_UART_PACKET_LEN - 1)) {
        ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                    "Skipping packet with bad checksum %02X wanted %02X\n",
                    UART_GetByte(BL0942_UART_PACKET_LEN - 1), checksum);
        UART_ConsumeBytes(BL0942_UART_PACKET_LEN);
		return 1;
	}

    bl0942_data_t data;
    data.i_rms =
        (UART_GetByte(3) << 16) | (UART_GetByte(2) << 8) | UART_GetByte(1);
    data.v_rms =
        (UART_GetByte(6) << 16) | (UART_GetByte(5) << 8) | UART_GetByte(4);
    data.watt = Int24ToInt32((UART_GetByte(12) << 16) |
                             (UART_GetByte(11) << 8) | UART_GetByte(10));
    data.cf_cnt = Int24ToInt32(
        (UART_GetByte(15) << 16) | (UART_GetByte(14) << 8) | UART_GetByte(13));
    data.freq = (UART_GetByte(17) << 8) | UART_GetByte(16);

    /* ---------------------------------------------------------------
       TCP UART mode hook — DO NOT REMOVE
       When UART_TCP_GetLastSlot() >= 0 this reading came from a remote
       BK7231N via TCP, not from the local hardware UART.

       TODO (multi-device): route data to the correct sensor group:
         int slot = UART_TCP_GetLastSlot();   // 0-3, or -1 = HW UART
         if (slot >= 0) { ... update sensors[slot] ... }

       Charger command hook:
         UART_TCP_SendChargerCmd("/api/charger?cmd=xxx")
         Called here when energy conditions trigger a charge/stop action.
         UART_TCP_GetChargerIP(0) = placeholder IP (future use)
         UART_TCP_GetChargerIP(1) = active charger IP
    --------------------------------------------------------------- */

    ScaleAndUpdate(&data);

    UART_ConsumeBytes(BL0942_UART_PACKET_LEN);
	return BL0942_UART_PACKET_LEN;
}

static void UART_WriteReg(uint8_t reg, uint32_t val) {
    uint8_t send[5];
    send[0] = BL0942_UART_CMD_WRITE(BL0942_UART_ADDR);
    send[1] = reg;
    send[2] = (val & 0xFF);
    send[3] = ((val >> 8) & 0xFF);
    send[4] = ((val >> 16) & 0xFF);
    uint8_t crc = 0;

    for (int i = 0; i < sizeof(send); i++) {
        UART_SendByte(send[i]);
        crc += send[i];
    }

    UART_SendByte(crc ^ 0xFF);
}

static int SPI_ReadReg(uint8_t reg, uint32_t *val) {
	uint8_t send[2];
	uint8_t recv[4];
	send[0] = BL0942_SPI_CMD_READ;
	send[1] = reg;
	SPI_Transmit(send, sizeof(send), recv, sizeof(recv));

	uint8_t checksum = send[0] + send[1] + recv[0] + recv[1] + recv[2];
	checksum ^= 0xFF;
	if (recv[3] != checksum) {
		ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                    "Failed to read reg %02X: bad checksum %02X wanted %02X",
                    reg, recv[3], checksum);
		return -1;
	}

	*val = (recv[0] << 16) | (recv[1] << 8) | recv[2];
	return 0;
}

static int SPI_WriteReg(uint8_t reg, uint32_t val) {
    uint8_t send[6];
    send[0] = BL0942_SPI_CMD_WRITE;
    send[1] = reg;
    send[2] = ((val >> 16) & 0xFF);
    send[3] = ((val >> 8) & 0xFF);
    send[4] = (val & 0xFF);

    // checksum
    send[5] = send[0] + send[1] + send[2] + send[3] + send[4];
    send[5] ^= 0xFF;

    SPI_WriteBytes(send, sizeof(send));

    uint32_t read;
    SPI_ReadReg(reg, &read);
    if (read == val ||
        // REG_USR_WRPROT is read back as 0x1
        (reg == BL0942_REG_USR_WRPROT && val == BL0942_USR_WRPROT_DISABLE &&
         read == 0x1)) {
        return 0;
    }

    ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER,
                 "Failed to write reg %02X val %02X: read %02X", reg, val,
                 read);
    return -1;
}

static void Init(void) {
    PrevCfCnt = CF_CNT_INVALID;
    g_offlineSec = 0;

    BL_Shared_Init();

    PwrCal_Init(PWR_CAL_DIVIDE, DEFAULT_VOLTAGE_CAL, DEFAULT_CURRENT_CAL,
                DEFAULT_POWER_CAL);
}

// THIS IS called by 'startDriver BL0942' command
// You can set alternate baud with 'startDriver BL0942 9600' syntax
void BL0942_UART_Init(void) {
	Init();

	bl0942_baudRate = Tokenizer_GetArgIntegerDefault(1, 4800);

	UART_InitUART(bl0942_baudRate, 0, false);
	UART_InitReceiveRingBuffer(BL0942_UART_RECEIVE_BUFFER_SIZE);

    UART_WriteReg(BL0942_REG_USR_WRPROT, BL0942_USR_WRPROT_DISABLE);
    UART_WriteReg(BL0942_REG_MODE,
                  BL0942_MODE_DEFAULT | BL0942_MODE_RMS_UPDATE_SEL_800_MS);
    // Set the minimun power measurement
    UART_WriteReg(BL0942_REG_WA_CREEP, DEFAULT_WA_CREEP_VAL);

#if PLATFORM_ESPIDF
    // Remote-meter build: start the persistent 6-socket poller task. Calibration
    // (PwrCal_Init) ran in Init() above, so frames are scaled correctly.
    UART_TCP_StartMeterPoll();
#endif
}

#if PLATFORM_ESPIDF
// ====================================================================
// REMOTE MULTI-METER POLLER (serial-over-TCP, 6 meters)
// ====================================================================
// One meter is polled per call (per second); a full 6-meter sweep takes ~6 s.
// Each meter: zero-octet -> mark offline and skip instantly; otherwise open a
// short-lived socket, read one BL0942 frame (50 ms budget), parse + scale, and
// store to its slot. On sweep completion the shared layer integrates energy and
// feeds the consumption phases into the existing pipeline.

// Validate + parse a 23-byte 0x55 frame from a flat buffer, scale it, and store
// to meter `slot`. Returns 1 on a good frame, 0 otherwise. Checksum matches the
// local parser: (CMD_READ + sum(bytes[0..len-2])) ^ 0xFF == last byte.
static int BL0942_ParseScaleStore(const byte *b, int len, int slot) {
    int i;
    byte checksum;
    bl0942_data_t d;
    float voltage, current, power, frequency, signedPower, energyWh;

    if (len < BL0942_UART_PACKET_LEN)        return 0;
    if (b[0] != BL0942_UART_PACKET_HEAD)     return 0;

    checksum = BL0942_UART_CMD_READ(BL0942_UART_ADDR);
    for (i = 0; i < BL0942_UART_PACKET_LEN - 1; i++) checksum += b[i];
    checksum ^= 0xFF;
    if (checksum != b[BL0942_UART_PACKET_LEN - 1]) return 0;

    d.i_rms  = (b[3] << 16) | (b[2] << 8) | b[1];
    d.v_rms  = (b[6] << 16) | (b[5] << 8) | b[4];
    d.watt   = Int24ToInt32((b[12] << 16) | (b[11] << 8) | b[10]);
    d.cf_cnt = Int24ToInt32((b[15] << 16) | (b[14] << 8) | b[13]);
    d.freq   = (b[17] << 8) | b[16];

    PwrCal_Scale(d.v_rms, d.i_rms, d.watt, &voltage, &current, &power);
    if (!isfinite(power)) power = 0.0f;
    frequency   = (d.freq != 0) ? (2 * 500000.0f / d.freq) : 0.0f;
    signedPower = CFG_HasFlag(OBK_FLAG_POWER_INVERT_AC) ? (-1.0f * power) : power;

    // Signed per-read energy (algebraic cf_cnt): + import / - export. Apply the
    // SAME invert flag as the power so energy and power agree on direction, and
    // clamp an insane/non-finite magnitude to 0 to protect downstream int casts.
    energyWh = PwrCal_ScalePowerOnly(d.cf_cnt) * 1638.4f * 256.0f / 3600.0f;
    if (CFG_HasFlag(OBK_FLAG_POWER_INVERT_AC)) energyWh = -1.0f * energyWh;
    if (!isfinite(energyWh) || fabsf(energyWh) > 50.0f) energyWh = 0.0f;

    BL_SetMeterReading(slot, voltage, current, signedPower, frequency, energyWh, 1);
    return 1;
}

// Scan a flat buffer for the first checksum-valid 23-byte 0x55 frame, parse,
// scale and store it to meter `slot`. Returns the number of bytes consumed up
// to and including that frame (so the caller can keep any trailing bytes), or
// 0 if no valid frame is present yet. Tolerant of leading/stray bytes — this
// is what keeps a single junk byte (e.g. a BL0942 line-low/BREAK glitch) from
// permanently desyncing the stream the way an offset-0-only parse would.
int BL0942_TCP_ScanStore(const unsigned char *b, int len, int slot) {
    int off;
    for (off = 0; off + BL0942_UART_PACKET_LEN <= len; off++) {
        if (BL0942_ParseScaleStore(b + off, BL0942_UART_PACKET_LEN, slot))
            return off + BL0942_UART_PACKET_LEN;
    }
    return 0;
}
#endif // PLATFORM_ESPIDF

void BL0942_UART_RunEverySecond(void) {
#if PLATFORM_ESPIDF
    // Remote-meter build: polling is handled by the persistent MeterPoll_Task
    // (started in BL0942_UART_Init). Nothing to do on the per-second tick.
    return;
#endif
    // Send the read request.
    // UART_InitUART is NOT called here — it belongs in BL0942_UART_Init only.
    // Calling it every second reinitialises the peripheral, drives TX low
    // momentarily (which the BL0942 can interpret as a BREAK/reset), and
    // was the root cause of crashes when no device was connected.
    UART_SendByte(BL0942_UART_CMD_READ(BL0942_UART_ADDR));
    UART_SendByte(BL0942_UART_REG_PACKET);

    // Poll for response with timeout.
    // Returns as soon as a full packet is in the buffer — don't wait the
    // full timeout if data arrives in 8 ms. Each delay_ms() yields to the
    // RTOS scheduler so other tasks run between checks.
    // This is the standard rugged-serial pattern: poll + early exit + deadline.
    int waited = 0;
    while (waited < BL0942_RESPONSE_TIMEOUT_MS) {
        if (UART_GetDataSize() >= BL0942_UART_PACKET_LEN) {
            break;  // data is ready — no point waiting further
        }
        delay_ms(BL0942_POLL_INTERVAL_MS);
        waited += BL0942_POLL_INTERVAL_MS;
    }

    // Try to collect the response
    if (UART_TryToGetNextPacket() == BL0942_UART_PACKET_LEN) {
        // Good packet — ScaleAndUpdate was already called inside TryToGetNextPacket
        if (g_offlineSec > 0) {
            ADDLOG_INFO(LOG_FEATURE_ENERGYMETER,
                        "BL0942: device back online after %d s\n", g_offlineSec);
            g_offlineSec = 0;
        }
        return;
    }

    // No valid response received.
    // Push a zero reading so the downstream pipeline keeps running cleanly
    // and doesn't see stale or garbage data. When the device reconnects,
    // real readings resume automatically on the next successful packet.
    if (g_offlineSec == 0) {
        ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                    "BL0942: no response after %d ms — device offline, reporting zeros\n",
                    BL0942_RESPONSE_TIMEOUT_MS);
    }
    g_offlineSec++;

    bl0942_data_t zeros = {0};
    ScaleAndUpdate(&zeros);
}

void BL0942_SPI_Init(void) {
	Init();

	SPI_DriverInit();
	spi_config_t cfg;
	cfg.role = SPI_ROLE_MASTER;
	cfg.bit_width = SPI_BIT_WIDTH_8BITS;
	cfg.polarity = SPI_POLARITY_LOW;
	cfg.phase = SPI_PHASE_2ND_EDGE;
	cfg.wire_mode = SPI_3WIRE_MODE;
	cfg.baud_rate = BL0942_SPI_BAUD_RATE;
	cfg.bit_order = SPI_MSB_FIRST;
	OBK_SPI_Init(&cfg);

    SPI_WriteReg(BL0942_REG_USR_WRPROT, BL0942_USR_WRPROT_DISABLE);
    SPI_WriteReg(BL0942_REG_MODE,
                 BL0942_MODE_DEFAULT | BL0942_MODE_RMS_UPDATE_SEL_800_MS);
}

void BL0942_SPI_RunEverySecond(void) {
    bl0942_data_t data = {0};
    int err = 0;

    err |= SPI_ReadReg(BL0942_REG_I_RMS,  &data.i_rms);
    err |= SPI_ReadReg(BL0942_REG_V_RMS,  &data.v_rms);
    err |= SPI_ReadReg(BL0942_REG_WATT,   (uint32_t *)&data.watt);
    err |= SPI_ReadReg(BL0942_REG_CF_CNT, (uint32_t *)&data.cf_cnt);
    err |= SPI_ReadReg(BL0942_REG_FREQ,   &data.freq);

    if (err != 0) {
        g_offlineSec++;
        if (g_offlineSec == 1) {
            ADDLOG_WARN(LOG_FEATURE_ENERGYMETER,
                        "BL0942 SPI: read error — device offline, reporting zeros\n");
        }
        bl0942_data_t zeros = {0};
        ScaleAndUpdate(&zeros);
        return;
    }

    if (g_offlineSec > 0) {
        ADDLOG_INFO(LOG_FEATURE_ENERGYMETER,
                    "BL0942 SPI: device back online after %d s\n", g_offlineSec);
        g_offlineSec = 0;
    }

    data.watt = Int24ToInt32(data.watt);
    data.cf_cnt = Int24ToInt32(data.cf_cnt);
    ScaleAndUpdate(&data);
}
