/* ==========================================================================
   drv_e220_lora.c -- EBYTE E220-400T22S transparent-LoRa meter link (MASTER)

   Topology: this device carries one E220 on a dedicated UART; each meter
   carries one E220 wired straight to its BL0942 (4800 8N1, no MCU).
   All modules share one channel. The master module is configured to the
   monitor address 0xFFFF (hears every packet on the channel) with FIXED
   TRANSMISSION enabled, so every poll starts with [ADDH][ADDL][CH] and is
   emitted by exactly ONE slave module's UART. Slaves run plain transparent
   mode with their own address = slot+1: whatever their BL0942 answers is
   broadcast back and the monitoring master hears it. One question, one
   answerer -- collisions are impossible by construction.

   Polling mirrors drv_uart_tcp_client.c mc_service() step for step:
   MODE register verified on EVERY cycle; mismatch -> reprogram signed
   free-run, discard the cycle (never store data taken in the wrong mode);
   match -> read one checksum-valid 23-byte frame via BL0942_TCP_ScanStore.
   Timeouts are the TCP constants plus a small allowance for the two air
   hops -- sized for the 62.5k air rate this driver configures. Worst case
   per slot stays well under the 1-second tick, the ESP can never hang here.

   Master-side RSSI: the master module is configured to append one RSSI
   byte to every received packet (REG3 bit7). After a valid frame the last
   drained byte is that packet's RSSI; dBm = -(256 - byte). RAM only.
   ========================================================================== */

#include "../new_common.h"
#include "../logging/logging.h"
#include "../cmnds/cmd_public.h"
#include "drv_e220_lora.h"
#include "drv_bl0942.h"      /* BL0942_TCP_ScanStore + register constants */
#include "drv_bl_shared.h"   /* BL_MeterReadFailed / BL_MeterNoteReset    */

#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ---- timeouts: TCP-stack values + air allowance (tuned for 62.5k air) ---- */
#define LM_FRAME_SETTLE_MS 120   /* TCP 70  + ~50 ms for two air hops        */
#define LM_ATTEMPT_MS      180   /* TCP 100 + ~80 ms air allowance           */
#define LM_MAX_ATTEMPTS    3     /* same as TCP                              */
#define LM_REG_READ_MS     280   /* TCP 200 + ~80 ms air allowance           */
#define LM_REG_SETTLE_MS   80    /* TCP 50  + ~30 ms                         */
#define LM_RXCAP           64    /* frame resync buffer (same as TCP)        */
#define LM_AUX_WAIT_MS     100   /* module-busy gate when AUX is wired       */
#define LM_CFG_SETTLE_MS   60    /* post register-write settle (no AUX)      */

/* ---- E220 register map (E220-400T22S user manual) ---- */
#define E220_REG_ADDH   0x00
#define E220_REG_ADDL   0x01
#define E220_REG_REG0   0x02   /* baud[7:5] parity[4:3] air[2:0]            */
#define E220_REG_REG1   0x03   /* subpkt[7:6] rssi-ambient[5] power[1:0]    */
#define E220_REG_REG2   0x04   /* channel: 410.125 MHz + CH                 */
#define E220_REG_REG3   0x05   /* rssi-byte[7] fixed-tx[6] LBT[4] WOR[2:0]  */

#define E220_BAUD_4800  (0x02 << 5)     /* UART side: BL0942 speed          */
#define E220_AIR_DEFAULT 0x07           /* 62.5k -- required by LM_ timeouts */
#define E220_PWR_DEFAULT 0x03           /* 10 dBm: UK 433 ISM legal ceiling */
#define E220_CH_DEFAULT  23             /* 410.125 + 23 = 433.125 MHz       */

/* ---- config (NVS "config" namespace) ---- */
static int8_t  s_uart = -1, s_pin_tx = -1, s_pin_rx = -1;
static int8_t  s_pin_m0 = -1, s_pin_m1 = -1, s_pin_aux = -1;
static uint8_t s_ch  = E220_CH_DEFAULT;
static uint8_t s_air = E220_AIR_DEFAULT;
static uint8_t s_pwr = E220_PWR_DEFAULT;

static int  s_ready = 0;
static int  s_rssi[6] = {0,0,0,0,0,0};   /* dBm, 0 = unknown (RAM only) */

static uint32_t lm_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* =========================================================================
   Module plumbing
   ========================================================================= */
static void lm_uart_open(int baud)
{
    uart_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.baud_rate  = baud;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    uart_param_config((uart_port_t)s_uart, &cfg);
    uart_set_pin((uart_port_t)s_uart, s_pin_tx, s_pin_rx,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (!uart_is_driver_installed((uart_port_t)s_uart))
        uart_driver_install((uart_port_t)s_uart, 1024, 1024, 0, NULL, 0);
}

static void lm_flush_rx(void)
{
    uart_flush_input((uart_port_t)s_uart);
}

/* AUX high = module idle. Optional pin; falls back to a fixed settle. */
static void lm_wait_ready(int settle_ms)
{
    if (s_pin_aux >= 0) {
        uint32_t deadline = lm_now_ms() + LM_AUX_WAIT_MS;
        while (gpio_get_level((gpio_num_t)s_pin_aux) == 0 &&
               (int32_t)(lm_now_ms() - deadline) < 0)
            rtos_delay_milliseconds(2);
        rtos_delay_milliseconds(3);          /* datasheet: AUX high + >2 ms */
    } else {
        rtos_delay_milliseconds(settle_ms);
    }
}

static void lm_mode(int m1, int m0)
{
    gpio_set_level((gpio_num_t)s_pin_m1, m1);
    gpio_set_level((gpio_num_t)s_pin_m0, m0);
    lm_wait_ready(LM_CFG_SETTLE_MS);
}

/* ---- configuration mode register access (mode 3, always 9600 8N1) ---- */
static int lm_cfg_read(uint8_t addr, uint8_t *out, int len)
{
    uint8_t cmd[3] = { 0xC1, addr, (uint8_t)len };
    uint8_t rx[16];
    int n;
    lm_flush_rx();
    uart_write_bytes((uart_port_t)s_uart, (const char *)cmd, 3);
    uart_wait_tx_done((uart_port_t)s_uart, pdMS_TO_TICKS(50));
    n = uart_read_bytes((uart_port_t)s_uart, rx, 3 + len, pdMS_TO_TICKS(150));
    if (n != 3 + len || rx[0] != 0xC1 || rx[1] != addr || rx[2] != (uint8_t)len)
        return -1;
    memcpy(out, rx + 3, len);
    return 0;
}

static int lm_cfg_write(uint8_t addr, const uint8_t *val, int len)
{
    uint8_t cmd[16];
    uint8_t back[8];
    cmd[0] = 0xC0; cmd[1] = addr; cmd[2] = (uint8_t)len;
    memcpy(cmd + 3, val, len);
    lm_flush_rx();
    uart_write_bytes((uart_port_t)s_uart, (const char *)cmd, 3 + len);
    uart_wait_tx_done((uart_port_t)s_uart, pdMS_TO_TICKS(50));
    (void)uart_read_bytes((uart_port_t)s_uart, back, 3 + len, pdMS_TO_TICKS(150));
    lm_wait_ready(LM_CFG_SETTLE_MS);
    /* trust nothing: read back and compare */
    if (lm_cfg_read(addr, back, len) != 0) return -1;
    return memcmp(back, val, len) == 0 ? 0 : -1;
}

/* Apply a full personality; writes only the registers that differ so the
   module's config flash is never worn by routine boots. */
static int lm_apply_personality(const uint8_t regs[6], const char *tag)
{
    uint8_t cur[6];
    int rc = 0;
    lm_mode(1, 1);                            /* mode 3 = config, 9600 8N1  */
    lm_uart_open(9600);
    if (lm_cfg_read(E220_REG_ADDH, cur, 6) != 0) {
        ADDLOG_WARN(LOG_FEATURE_DRV, "E220: no response in config mode");
        rc = -1;
    } else if (memcmp(cur, regs, 6) != 0) {
        if (lm_cfg_write(E220_REG_ADDH, regs, 6) != 0) {
            ADDLOG_WARN(LOG_FEATURE_DRV, "E220: %s config write failed", tag);
            rc = -1;
        } else {
            ADDLOG_INFO(LOG_FEATURE_DRV, "E220: %s personality written", tag);
        }
    } else {
        ADDLOG_INFO(LOG_FEATURE_DRV, "E220: %s personality already set", tag);
    }
    lm_mode(0, 0);                            /* back to normal mode        */
    lm_uart_open(4800);                       /* run side matches BL0942    */
    lm_flush_rx();
    return rc;
}

static void lm_master_regs(uint8_t r[6])
{
    r[0] = 0xFF;                              /* ADDH: monitor address      */
    r[1] = 0xFF;                              /* ADDL                       */
    r[2] = (uint8_t)(E220_BAUD_4800 | (s_air & 0x07));
    r[3] = (uint8_t)(s_pwr & 0x03);
    r[4] = s_ch;
    r[5] = 0x80 | 0x40 | 0x10;                /* RSSI byte + fixed TX + LBT */
}

static void lm_slave_regs(uint8_t r[6], int id)
{
    r[0] = 0x00;
    r[1] = (uint8_t)id;                       /* slave address = slot + 1   */
    r[2] = (uint8_t)(E220_BAUD_4800 | (s_air & 0x07));
    r[3] = (uint8_t)(s_pwr & 0x03);
    r[4] = s_ch;
    r[5] = 0x10;                              /* plain transparent + LBT    */
}

/* =========================================================================
   Radio transport primitives (fixed-transmission poll, monitored replies)
   ========================================================================= */
static void lm_send(int slot, const uint8_t *payload, int len)
{
    uint8_t buf[16];
    buf[0] = 0x00;                            /* ADDH of target slave       */
    buf[1] = (uint8_t)(slot + 1);             /* ADDL = slave address       */
    buf[2] = s_ch;                            /* channel                    */
    memcpy(buf + 3, payload, len);
    lm_wait_ready(5);
    lm_flush_rx();                            /* stragglers die here        */
    uart_write_bytes((uart_port_t)s_uart, (const char *)buf, 3 + len);
    uart_wait_tx_done((uart_port_t)s_uart, pdMS_TO_TICKS(60));
}

/* Send one read-full-frame request; wait until `deadline` for a checksum-
   valid 23-byte frame; store to `slot`. Mirrors mc_read_frame(). Also
   captures the trailing per-packet RSSI byte the master module appends.
   Returns 1 stored, 0 no valid frame before deadline. */
static int lm_read_frame(int slot, int cf_reset, uint32_t deadline)
{
    static const uint8_t req[2] = { 0x58, 0xAA };
    uint8_t rx[LM_RXCAP];
    int rxlen = 0, got = 0;

    lm_send(slot, req, 2);

    /* nothing can complete the two air hops + 23 bytes at 4800 sooner */
    rtos_delay_milliseconds(LM_FRAME_SETTLE_MS);

    while ((int32_t)(lm_now_ms() - deadline) < 0) {
        int n;
        if (rxlen >= LM_RXCAP) rxlen = 0;                 /* resync         */
        n = uart_read_bytes((uart_port_t)s_uart, rx + rxlen,
                            LM_RXCAP - rxlen, pdMS_TO_TICKS(30));
        if (n > 0) {
            rxlen += n;
            if (!got && BL0942_TCP_ScanStore(rx, rxlen, slot, cf_reset) > 0) {
                got = 1;
                /* drain briefly: the packet's RSSI byte trails the frame */
                n = uart_read_bytes((uart_port_t)s_uart, rx + rxlen,
                                    (rxlen < LM_RXCAP) ? 1 : 0,
                                    pdMS_TO_TICKS(25));
                if (n > 0) rxlen += n;
                if (rxlen > 0)
                    s_rssi[slot] = -(256 - (int)rx[rxlen - 1]);
                return 1;
            }
        }
    }
    return 0;
}

/* Single-register read over the air; same wire protocol as mc_read_reg(). */
static int lm_read_reg(int slot, uint8_t reg, uint32_t *val)
{
    uint8_t req[2];
    uint8_t rx[8];
    int rxn = 0;
    uint32_t deadline = lm_now_ms() + LM_REG_READ_MS;

    req[0] = 0x58; req[1] = reg;
    lm_send(slot, req, 2);
    rtos_delay_milliseconds(LM_REG_SETTLE_MS);

    while ((int32_t)(lm_now_ms() - deadline) < 0) {
        int n = uart_read_bytes((uart_port_t)s_uart, rx + rxn,
                                (int)sizeof(rx) - rxn, pdMS_TO_TICKS(30));
        if (n > 0) {
            rxn += n;
            if (rxn >= 4) {
                uint8_t cs = (uint8_t)((0x58 + reg + rx[0] + rx[1] + rx[2]) ^ 0xFF);
                if (cs != rx[3]) return -1;               /* corrupt reply  */
                *val = (uint32_t)rx[0] | ((uint32_t)rx[1] << 8)
                     | ((uint32_t)rx[2] << 16);
                return 0;
            }
        }
    }
    return -1;                                            /* timeout        */
}

/* Fire-and-forget register write; wire format identical to mc_write_reg(). */
static void lm_write_reg(int slot, uint8_t reg, uint32_t val)
{
    uint8_t msg[6];
    uint8_t crc = 0;
    int i;
    msg[0] = 0xA8;
    msg[1] = reg;
    msg[2] = (uint8_t)( val        & 0xFF);
    msg[3] = (uint8_t)((val >>  8) & 0xFF);
    msg[4] = (uint8_t)((val >> 16) & 0xFF);
    for (i = 0; i < 5; i++) crc += msg[i];
    msg[5] = crc ^ 0xFF;
    lm_send(slot, msg, 6);
}

/* =========================================================================
   Public: service one slot -- the exact mc_service() contract
   ========================================================================= */
int LoRaMeter_Service(int slot)
{
    uint32_t mode = 0;
    int attempt, got = 0;

    if (!s_ready || slot < 0 || slot >= 6) {
        BL_SetMeterReading(slot, 0, 0, 0, 0, 0);
        return -1;
    }

    /* 1) MODE verified on EVERY cycle -- data taken in the wrong mode is
          corrupt and must never reach the stack. */
    if (lm_read_reg(slot, BL0942_REG_MODE_ADDR, &mode) != 0) {
        BL_MeterReadFailed(slot);
        BL_MeterStatsReport(slot, 0, 0, 0);
        return 0;                              /* no/broken reply this cycle */
    }

    /* 2) Wrong mode -> reprogram, discard this cycle, let the chip settle. */
    if ((mode & BL0942_MODE_MATCH_MASK) != BL0942_MODE_FREE_RUN_SIGNED) {
        lm_write_reg(slot, BL0942_REG_WRPROT_ADDR, BL0942_WRPROT_UNLOCK);
        rtos_delay_milliseconds(30);
        lm_write_reg(slot, BL0942_REG_MODE_ADDR, BL0942_MODE_FREE_RUN_SIGNED);
        ADDLOG_WARN(LOG_FEATURE_DRV,
                    "LoRa meter %d MODE was 0x%X (reset) - reprogrammed to signed",
                    slot + 1, (unsigned)mode);
        if (slot >= 0 && slot <= 2) BL_MeterNoteReset(slot);
        BL_MeterReadFailed(slot);
        BL_MeterStatsReport(slot, 0, 0, 0);
        return 0;
    }

    /* 3) Mode verified -> read the data frame (same retry ladder as TCP). */
    {
        uint32_t t0 = lm_now_ms();
        for (attempt = 0; attempt < LM_MAX_ATTEMPTS; attempt++) {
            if (lm_read_frame(slot, BL0942_CF_RESET_NONE,
                              lm_now_ms() + LM_ATTEMPT_MS)) { got = 1; break; }
        }
        if (got) {
            BL_MeterStatsReport(slot, 1, attempt + 1, (int)(lm_now_ms() - t0));
            return 1;
        }
    }
    BL_MeterReadFailed(slot);
    BL_MeterStatsReport(slot, 0, LM_MAX_ATTEMPTS, 0);
    return 0;
}

int LoRaMeter_Ready(void)        { return s_ready; }

int LoRaMeter_GetRSSI(int slot)
{
    if (slot < 0 || slot >= 6) return 0;
    return s_rssi[slot];
}

/* =========================================================================
   Config / console
   ========================================================================= */
static void lm_nvs_load(void)
{
    nvs_handle_t h = 0;
    if (nvs_open("config", NVS_READONLY, &h) != ESP_OK) return;
    int8_t v;
    if (nvs_get_i8(h, "e220_uart", &v) == ESP_OK) s_uart    = v;
    if (nvs_get_i8(h, "e220_tx",   &v) == ESP_OK) s_pin_tx  = v;
    if (nvs_get_i8(h, "e220_rx",   &v) == ESP_OK) s_pin_rx  = v;
    if (nvs_get_i8(h, "e220_m0",   &v) == ESP_OK) s_pin_m0  = v;
    if (nvs_get_i8(h, "e220_m1",   &v) == ESP_OK) s_pin_m1  = v;
    if (nvs_get_i8(h, "e220_aux",  &v) == ESP_OK) s_pin_aux = v;
    uint8_t u;
    if (nvs_get_u8(h, "e220_ch",  &u) == ESP_OK) s_ch  = u;
    if (nvs_get_u8(h, "e220_air", &u) == ESP_OK) s_air = u;
    if (nvs_get_u8(h, "e220_pwr", &u) == ESP_OK) s_pwr = u;
    nvs_close(h);
}

static void lm_nvs_save(void)
{
    nvs_handle_t h = 0;
    if (nvs_open("config", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i8(h, "e220_uart", s_uart);
    nvs_set_i8(h, "e220_tx",   s_pin_tx);
    nvs_set_i8(h, "e220_rx",   s_pin_rx);
    nvs_set_i8(h, "e220_m0",   s_pin_m0);
    nvs_set_i8(h, "e220_m1",   s_pin_m1);
    nvs_set_i8(h, "e220_aux",  s_pin_aux);
    nvs_set_u8(h, "e220_ch",   s_ch);
    nvs_set_u8(h, "e220_air",  s_air);
    nvs_set_u8(h, "e220_pwr",  s_pwr);
    nvs_commit(h); nvs_close(h);
}

static int lm_hw_init(void)
{
    gpio_config_t io;
    if (s_uart < 0 || s_pin_tx < 0 || s_pin_rx < 0 ||
        s_pin_m0 < 0 || s_pin_m1 < 0)
        return -1;

    memset(&io, 0, sizeof(io));
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL << s_pin_m0) | (1ULL << s_pin_m1);
    gpio_config(&io);
    if (s_pin_aux >= 0) {
        memset(&io, 0, sizeof(io));
        io.mode = GPIO_MODE_INPUT;
        io.pull_up_en = GPIO_PULLUP_ENABLE;
        io.pin_bit_mask = (1ULL << s_pin_aux);
        gpio_config(&io);
    }
    {
        uint8_t regs[6];
        lm_master_regs(regs);
        if (lm_apply_personality(regs, "master") != 0) return -1;
    }
    return 0;
}

static int cmd_lora_pins(const void *c, const char *cmd, const char *a)
{
    int u=-1, tx=-1, rx=-1, m0=-1, m1=-1, aux=-1;
    (void)c; (void)cmd;
    if (!a || sscanf(a, "%d %d %d %d %d %d", &u,&tx,&rx,&m0,&m1,&aux) < 5) {
        ADDLOG_WARN(LOG_FEATURE_DRV,
            "Usage: LoRaSetPins <uart 1..2> <tx> <rx> <m0> <m1> [aux]");
        return 1;
    }
    s_uart=(int8_t)u; s_pin_tx=(int8_t)tx; s_pin_rx=(int8_t)rx;
    s_pin_m0=(int8_t)m0; s_pin_m1=(int8_t)m1; s_pin_aux=(int8_t)aux;
    lm_nvs_save();
    s_ready = (lm_hw_init() == 0);
    ADDLOG_INFO(LOG_FEATURE_DRV, "E220 pins saved; ready=%d", s_ready);
    return 1;
}

static int cmd_lora_radio(const void *c, const char *cmd, const char *a)
{
    int ch=-1, air=-1, pwr=-1;
    (void)c; (void)cmd;
    if (!a || sscanf(a, "%d %d %d", &ch, &air, &pwr) < 1) {
        ADDLOG_WARN(LOG_FEATURE_DRV,
            "Usage: LoRaSetRadio <ch 0..80> [air 0..7] [pwr 0..3]  "
            "(air: 7=62.5k default; pwr: 3=10dBm default)");
        return 1;
    }
    if (ch  >= 0 && ch  <= 80) s_ch  = (uint8_t)ch;
    if (air >= 0 && air <= 7)  s_air = (uint8_t)air;
    if (pwr >= 0 && pwr <= 3)  s_pwr = (uint8_t)pwr;
    if (s_air < E220_AIR_DEFAULT)
        ADDLOG_WARN(LOG_FEATURE_DRV,
            "E220: air rate below 62.5k -- LM_ timeouts assume 62.5k; "
            "slow rates WILL miss the read deadlines");
    lm_nvs_save();
    if (s_ready || lm_hw_init() == 0) {
        uint8_t regs[6]; lm_master_regs(regs);
        s_ready = (lm_apply_personality(regs, "master") == 0);
    }
    ADDLOG_INFO(LOG_FEATURE_DRV, "E220 radio: ch=%d air=%d pwr=%d ready=%d",
                s_ch, s_air, s_pwr, s_ready);
    return 1;
}

/* Program the MODULE CURRENTLY IN THE MASTER SOCKET as slave <id>, then
   power it down and deploy it in meter plug <id>. Run LoRaProgramMaster
   (or reboot) after re-seating the master's own module. */
static int cmd_lora_prog(const void *c, const char *cmd, const char *a)
{
    int id = 0;
    (void)c; (void)cmd;
    if (a) id = atoi(a);
    if (id < 1 || id > 6) {
        ADDLOG_WARN(LOG_FEATURE_DRV, "Usage: LoRaProgram <slave id 1..6>");
        return 1;
    }
    if (s_uart < 0) { ADDLOG_WARN(LOG_FEATURE_DRV, "Set pins first"); return 1; }
    {
        uint8_t regs[6];
        lm_slave_regs(regs, id);
        if (lm_apply_personality(regs, "slave") == 0)
            ADDLOG_INFO(LOG_FEATURE_DRV,
                "E220: module programmed as SLAVE %d (addr %d, ch %d, 4800 8N1)."
                " Unplug it and deploy to meter %d.", id, id, s_ch, id);
    }
    s_ready = 0;   /* the seated module is a slave now -- not a master */
    return 1;
}

static int cmd_lora_master(const void *c, const char *cmd, const char *a)
{
    (void)c; (void)cmd; (void)a;
    if (s_uart < 0) { ADDLOG_WARN(LOG_FEATURE_DRV, "Set pins first"); return 1; }
    s_ready = (lm_hw_init() == 0);
    ADDLOG_INFO(LOG_FEATURE_DRV, "E220 master personality: ready=%d", s_ready);
    return 1;
}

static int cmd_lora_status(const void *c, const char *cmd, const char *a)
{
    int i;
    (void)c; (void)cmd; (void)a;
    ADDLOG_INFO(LOG_FEATURE_DRV,
        "E220: ready=%d uart=%d tx=%d rx=%d m0=%d m1=%d aux=%d ch=%d air=%d pwr=%d",
        s_ready, s_uart, s_pin_tx, s_pin_rx, s_pin_m0, s_pin_m1, s_pin_aux,
        s_ch, s_air, s_pwr);
    for (i = 0; i < 6; i++)
        if (s_rssi[i])
            ADDLOG_INFO(LOG_FEATURE_DRV, "  slave %d last RSSI %d dBm", i+1, s_rssi[i]);
    return 1;
}

void LoRaMeter_Init(void)
{
    lm_nvs_load();
    CMD_RegisterCommand("LoRaSetPins",      cmd_lora_pins,
        "E220 wiring: LoRaSetPins <uart> <tx> <rx> <m0> <m1> [aux]");
    CMD_RegisterCommand("LoRaSetRadio",     cmd_lora_radio,
        "E220 radio: LoRaSetRadio <ch> [air] [pwr]");
    CMD_RegisterCommand("LoRaProgram",      cmd_lora_prog,
        "Program seated module as slave N (1..6) for deployment");
    CMD_RegisterCommand("LoRaProgramMaster",cmd_lora_master,
        "Restore master personality on the seated module");
    CMD_RegisterCommand("LoRaStatus",       cmd_lora_status,
        "Show E220 config and last per-slave RSSI");
    if (s_uart >= 0)
        s_ready = (lm_hw_init() == 0);
    ADDLOG_INFO(LOG_FEATURE_DRV, "E220 LoRa meter link: %s",
                s_ready ? "ready" : "not configured (LoRaSetPins)");
}
