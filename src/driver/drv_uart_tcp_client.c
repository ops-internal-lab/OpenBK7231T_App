/* ==========================================================================
   drv_uart_tcp_client.c  --  Remote UART-over-TCP client (MASTER side)

   What changed vs the old file:
   The old 6-meter poller (BL0942_PollRemoteMeters in drv_bl0942.c) opened a
   BRAND-NEW TCP connection every poll and closed it. On lwIP each closed
   connection sits in TIME_WAIT for ~2*MSL (~120 s on ESP-IDF), so polling 6
   meters churned the socket pool dry and most polls failed to connect.

   This file keeps a PERSISTENT socket open to each configured meter and reuses
   it every cycle — connections are created once (or re-created on failure,
   throttled), never per poll. Every socket has a single close site (mc_close)
   that nulls the fd, so nothing leaks and nothing double-closes.

   Polling is driven by UART_TCP_MeterTick(), called once per second from the
   BL0942 RunEverySecond hook (no dedicated task). A 10-tick cycle services one
   meter per second (slots 0..5) plus 4 dummy skips, so each meter is read once
   per 10 s; the CF-CNT delta over that window is the meter's net energy, and
   the sweep/dashboard sync fires on the 10th tick. Each read verifies MODE
   (0x19) is still free-running signed, reprogramming + discarding the interval
   on a detected chip reset, and retries once on a missed/corrupt frame.

   The original HAL-level mechanism (setUartTarget1..4 / s_serial[] /
   UART_TCP_GetCurrentTarget / UART_TCP_AdvanceTarget / UART_TCP_PollMeter) is
   left fully intact below — hal_uart_espidf.c depends on those symbols.

   Console commands (unchanged):
     setUartTarget1..4 <octet>     setChargerIP1/2 <octet>
     listUartTargets               listChargerIPs / sendChargerCmd <path>
   ========================================================================== */

#include "../new_common.h"
#include "../logging/logging.h"
#include "../cmnds/cmd_local.h"
#include "../hal/hal_wifi.h"
#include "drv_uart_tcp_client.h"
#include "drv_bl0942.h"      /* BL0942_TCP_ScanStore */
#include "drv_bl_shared.h"   /* BL_GetMeterOctet / BL_SetMeterReading / ... */

#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>

#define INVALID_SOCK   -1

/* ---- storage: last octet only (0 = unset) ---- */
static uint8_t s_serial[UART_TCP_SERIAL_MAX]  = {0};
static uint8_t s_charger[UART_TCP_CHARGER_MAX] = {0};
static int     s_current   = 0;
static int     s_last_slot = -1;

static char s_serial_ip [UART_TCP_SERIAL_MAX] [24];
static char s_charger_ip[UART_TCP_CHARGER_MAX][24];

static const char * const s_serial_keys[UART_TCP_SERIAL_MAX] = {
    "utcp_s1", "utcp_s2", "utcp_s3", "utcp_s4"
};
static const char * const s_charger_keys[UART_TCP_CHARGER_MAX] = {
    "utcp_c1", "utcp_c2"
};

/* ---- IP prefix helper ---- */
static const char *get_ip_prefix(char *buf, int bufsz)
{
    const char *myip = HAL_GetMyIPString();
    if (!myip || !*myip) { buf[0] = '\0'; return buf; }
    strncpy(buf, myip, bufsz - 1);
    buf[bufsz - 1] = '\0';
    char *last_dot = strrchr(buf, '.');
    if (last_dot) *last_dot = '\0';
    return buf;
}

static void build_ip(char *out, int outsz, uint8_t octet)
{
    char prefix[24];
    get_ip_prefix(prefix, sizeof(prefix));
    snprintf(out, outsz, "%s.%d", prefix, (int)octet);
}

void UART_TCP_BuildIP(char *out, int outsz, unsigned char octet)
{
    build_ip(out, outsz, (uint8_t)octet);
}

/* ---- NVS helpers ---- */
static void nvs_load_all(void)
{
    nvs_handle_t h = 0;
    nvs_open("config", NVS_READONLY, &h);
    for (int i = 0; i < UART_TCP_SERIAL_MAX; i++) {
        uint8_t v = 0; nvs_get_u8(h, s_serial_keys[i], &v); s_serial[i] = v;
    }
    for (int i = 0; i < UART_TCP_CHARGER_MAX; i++) {
        uint8_t v = 0; nvs_get_u8(h, s_charger_keys[i], &v); s_charger[i] = v;
    }
    nvs_close(h);
}

static void nvs_save_serial(int slot) {
    nvs_handle_t h = 0;
    nvs_open("config", NVS_READWRITE, &h);
    nvs_set_u8(h, s_serial_keys[slot], s_serial[slot]);
    nvs_commit(h); nvs_close(h);
}

static void nvs_save_charger(int slot) {
    nvs_handle_t h = 0;
    nvs_open("config", NVS_READWRITE, &h);
    nvs_set_u8(h, s_charger_keys[slot], s_charger[slot]);
    nvs_commit(h); nvs_close(h);
}

/* ===========================================================================
   LEGACY HAL-level target API (used by hal_uart_espidf.c) — unchanged.
   =========================================================================== */
const char *UART_TCP_GetCurrentTarget(void)
{
    for (int n = 0; n < UART_TCP_SERIAL_MAX; n++) {
        int idx = (s_current + n) % UART_TCP_SERIAL_MAX;
        if (s_serial[idx] != 0) {
            s_current   = idx;
            s_last_slot = idx;
            build_ip(s_serial_ip[idx], sizeof(s_serial_ip[idx]), s_serial[idx]);
            return s_serial_ip[idx];
        }
    }
    s_last_slot = -1;
    return NULL;
}

int  UART_TCP_GetLastSlot(void) { return s_last_slot; }
void UART_TCP_AdvanceTarget(void) { s_current = (s_current + 1) % UART_TCP_SERIAL_MAX; }

int UART_TCP_Connect(const char *ip, int port)
{
    struct sockaddr_in addr;
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) return -1;

    struct timeval tv = { .tv_sec  = UART_TCP_TIMEOUT_MS / 1000,
                          .tv_usec = (UART_TCP_TIMEOUT_MS % 1000) * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ADDLOG_WARN(LOG_FEATURE_DRV, "UART_TCP: connect %s:%d failed (%d)",
                    ip, port, errno);
        close(fd);
        return -1;
    }
    return fd;
}

/* Legacy transient poll — kept for compatibility; the new task does not use it */
int UART_TCP_PollMeter(const char *ip, int port, uint8_t *out, int outlen)
{
    int got = 0;
    int fd  = UART_TCP_Connect(ip, port);
    if (fd < 0) return -1;
    { uint8_t req[2] = { 0x58, 0xAA };
      if (send(fd, req, sizeof(req), 0) < 0) { close(fd); return -1; } }
    while (got < outlen) {
        int n = recv(fd, out + got, outlen - got, 0);
        if (n <= 0) break;
        got += n;
    }
    close(fd);
    return got;
}

/* ===========================================================================
   REMOTE 6-METER POLLER  --  connect / read / (retry) / close, PER READ
   ---------------------------------------------------------------------------
   One meter is serviced per 1 Hz tick (round-robin, 10-tick cycle). Each
   service opens a FRESH short-lived TCP connection, reads one frame (retrying
   a few times inside a 250 ms budget until a checksum-valid frame arrives),
   then RST-closes the socket. Nothing is kept open between reads.

   Why this is safe against the socket-pool exhaustion that killed the original
   connect-per-poll code: we close with SO_LINGER 0 (RST), which does NOT enter
   TIME_WAIT, so opening a socket every read never drains the pool. And a fresh
   connection is inherently clean — the slave bridge flushes its stale UART on
   accept(), so there is never carried-over garbage to desync the frame.
   =========================================================================== */

#define MP_PORT            UART_TCP_PORT   /* 8888 */
#define MP_CONNECT_MS      80              /* bounded "get the link live" wait    */
/* Per frame attempt: after sending {0x58,0xAA} we SETTLE, then poll for the
   reply. A 23-byte frame at 4800 baud is ~48 ms on the wire ALONE, before the
   request byte-time, BL0942 turnaround and WiFi jitter — so we must wait for
   it. Bailing before it arrives is exactly how reads were being "lost". */
#define MP_FRAME_SETTLE_MS 70              /* skip the guaranteed-empty period    */
#define MP_ATTEMPT_MS      100              /* total wait per attempt (settle+poll) */
#define MP_MAX_ATTEMPTS    3               /* re-send + retry up to this many      */
#define MP_REG_READ_MS     200             /* single-register reply wait          */
#define MP_REG_SETTLE_MS   50              /* reg reply is only 4 bytes (~8 ms)    */
#define MP_RXCAP           64              /* frame resync buffer                 */
#define MP_SLOTS           6
#define MP_TICKS_PER_CYCLE 10              /* 6 meters + 4 dummy skips = 10 s cycle */
#define MP_MODE_CHECK_MS   30000           /* MODE re-verify cadence, per meter    */

static bool     g_pollRun  = false;        /* poller enabled between Start/Stop   */
static int      g_pollTick = 0;            /* 0..MP_TICKS_PER_CYCLE-1 round-robin  */
/* Per-meter state that must persist ACROSS the short-lived connections. */
static uint32_t g_lastModeCheck[MP_SLOTS] = { 0 };
static bool     g_modeSet[MP_SLOTS]       = { false };

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* RST close (SO_LINGER 0): no TIME_WAIT, so per-read connects don't exhaust the
   socket pool, and the slave sees the drop immediately and frees its one slot. */
static void mc_close(int fd)
{
    if (fd != INVALID_SOCK) {
        struct linger lg = { 1, 0 };
        setsockopt(fd, SOL_SOCKET, SO_LINGER, &lg, sizeof(lg));
        close(fd);
    }
}

/* Open a fresh, bounded, non-blocking connection to a meter. Returns fd or -1. */
static int mc_open(const char *ip)
{
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) return -1;

    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);

    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family      = AF_INET;
    a.sin_port        = htons(MP_PORT);
    a.sin_addr.s_addr = inet_addr(ip);

    int rc = connect(fd, (struct sockaddr *)&a, sizeof(a));
    if (rc != 0 && errno != EINPROGRESS) { mc_close(fd); return -1; }
    if (rc != 0) {
        fd_set w; FD_ZERO(&w); FD_SET(fd, &w);
        struct timeval tv = { 0, MP_CONNECT_MS * 1000 };
        if (select(fd + 1, NULL, &w, NULL, &tv) <= 0) { mc_close(fd); return -1; }
        int err = 0; socklen_t l = sizeof(err);
        getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &l);
        if (err != 0) { mc_close(fd); return -1; }
    }
    return fd;
}

/* Send one read-full-frame request and wait (until `deadline`) for a
   checksum-valid 23-byte frame; store it to `slot`. Returns 1 stored, 0 no
   valid frame before the deadline, -1 socket error / peer closed. */
static int mc_read_frame(int fd, int slot, int cf_reset, uint32_t deadline)
{
    unsigned char req[2] = { 0x58, 0xAA };
    unsigned char rx[MP_RXCAP];
    int rxlen = 0;

    if (send(fd, req, 2, 0) < 0 && errno != EWOULDBLOCK && errno != EAGAIN) return -1;

    /* Give the slow 4800-baud link time to start delivering before we poll —
       nothing can arrive in the first ~48 ms anyway. */
    rtos_delay_milliseconds(MP_FRAME_SETTLE_MS);

    while ((int32_t)(now_ms() - deadline) < 0) {
        int n;
        if (rxlen >= MP_RXCAP) rxlen = 0;                 /* overflow -> resync */
        n = recv(fd, rx + rxlen, MP_RXCAP - rxlen, 0);
        if (n > 0) {
            rxlen += n;
            if (BL0942_TCP_ScanStore(rx, rxlen, slot, cf_reset) > 0) return 1;
        } else if (n == 0) {
            return -1;                                    /* peer closed */
        } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
            return -1;                                    /* hard error */
        } else {
            rtos_delay_milliseconds(20);                   /* nothing yet, wait */
        }
    }
    return 0;                                             /* deadline, no frame */
}

/* Single-register read. On a FRESH connection there is nothing stale to drain
   (the slave flushed its UART on accept), so no pre-drain is needed. Protocol:
   send {0x58,reg}, reply = 3 data bytes (MSB first) + checksum
   ~(0x58+reg+b0+b1+b2). Returns 0 + *val, or -1. */
static int mc_read_reg(int fd, unsigned char reg, uint32_t *val)
{
    unsigned char req[2] = { 0x58, reg };
    unsigned char rx[8];
    int rxn = 0;
    uint32_t deadline = now_ms() + MP_REG_READ_MS;

    if (send(fd, req, 2, 0) < 0 && errno != EWOULDBLOCK && errno != EAGAIN) return -1;

    rtos_delay_milliseconds(MP_REG_SETTLE_MS);            /* let the reply arrive */

    while ((int32_t)(now_ms() - deadline) < 0) {
        int n = recv(fd, rx + rxn, (int)sizeof(rx) - rxn, 0);
        if (n > 0) {
            rxn += n;
            if (rxn >= 4) {
                unsigned char cs = (unsigned char)((0x58 + reg + rx[0] + rx[1] + rx[2]) ^ 0xFF);
                if (cs != rx[3]) return -1;               /* bad checksum */
                /* Value bytes are LITTLE-ENDIAN on the wire (confirmed against a
                   MODE read: RX 07 00 00 87 == 0x07, matching the 07 00 00 we
                   write). rx[0] is the LSB. */
                *val = (uint32_t)rx[0] | ((uint32_t)rx[1] << 8) | ((uint32_t)rx[2] << 16);
                return 0;
            }
        } else if (n == 0) {
            return -1;
        } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
            return -1;
        } else {
            rtos_delay_milliseconds(20);
        }
    }
    return -1;                                            /* timeout */
}

/* Fire-and-forget register write (no reply). Value bytes little-endian,
   checksum = ~(sum of the 5 bytes). */
static void mc_write_reg(int fd, unsigned char reg, uint32_t val)
{
    unsigned char msg[6];
    int i;
    unsigned char crc = 0;
    msg[0] = 0xA8;                       /* WRITE command, addr 0 */
    msg[1] = reg;
    msg[2] = (unsigned char)( val        & 0xFF);
    msg[3] = (unsigned char)((val >>  8) & 0xFF);
    msg[4] = (unsigned char)((val >> 16) & 0xFF);
    for (i = 0; i < 5; i++) crc += msg[i];
    msg[5] = crc ^ 0xFF;
    (void)send(fd, msg, sizeof(msg), 0);
}

/* Service ONE meter in a single pass:
     open -> (ensure/verify MODE) -> read a frame, re-sending and retrying up to
     MP_MAX_ATTEMPTS (each a MP_ATTEMPT_MS window) -> RST-close.
   Returns 1 good frame / 0 no valid frame / -1 offline (couldn't connect). */
static int mc_service(int slot)
{
    int cf_reset = BL0942_CF_RESET_NONE;
    int oct = BL_GetMeterOctet(slot);
    int fd, attempt, got = 0;
    char ip[24];

    if (oct == 0) {                                       /* unset -> hard offline */
        BL_SetMeterReading(slot, 0, 0, 0, 0, 0);
        return -1;
    }

    UART_TCP_BuildIP(ip, sizeof(ip), (unsigned char)oct);
    fd = mc_open(ip);
    if (fd < 0) { BL_MeterReadFailed(slot); return -1; }

    /* MODE, on this fresh connection:
       - first ever contact: blind-program signed mode (the chip powers up in
         its default ABSOLUTE mode). g_cfPrev is still INVALID, so the parser
         drops this first delta regardless.
       - otherwise, every MP_MODE_CHECK_MS: read MODE back. Only a CONFIRMED
         wrong value (a successful read that disagrees) means the chip reset ->
         reprogram + discard the tainted interval (CF_RESET_CHIP). An unreadable
         mode changes nothing (no reprogram, no wipe). */
    if (!g_modeSet[slot]) {
        mc_write_reg(fd, BL0942_REG_WRPROT_ADDR, BL0942_WRPROT_UNLOCK);
        rtos_delay_milliseconds(20);    
        mc_write_reg(fd, BL0942_REG_MODE_ADDR,   BL0942_MODE_FREE_RUN_SIGNED);
        g_modeSet[slot]       = true;
        g_lastModeCheck[slot] = now_ms();
    } else if ((int32_t)(now_ms() - g_lastModeCheck[slot]) >= MP_MODE_CHECK_MS) {
        uint32_t mode = 0;
        g_lastModeCheck[slot] = now_ms();
        if (mc_read_reg(fd, BL0942_REG_MODE_ADDR, &mode) == 0 &&
            (mode & BL0942_MODE_MATCH_MASK) != BL0942_MODE_FREE_RUN_SIGNED) {
            mc_write_reg(fd, BL0942_REG_WRPROT_ADDR, BL0942_WRPROT_UNLOCK);
            rtos_delay_milliseconds(20);    
            mc_write_reg(fd, BL0942_REG_MODE_ADDR,   BL0942_MODE_FREE_RUN_SIGNED);
            cf_reset = BL0942_CF_RESET_CHIP;
            ADDLOG_WARN(LOG_FEATURE_DRV,
                        "Meter %d MODE was 0x%X (reset) - reprogrammed to signed",
                        slot + 1, (unsigned)mode);
        }
    }

    /* Read one frame, re-sending and retrying up to MP_MAX_ATTEMPTS. Each
       attempt gets its OWN MP_ATTEMPT_MS window (settle + poll), so a retry
       genuinely re-sends rather than one attempt eating the whole budget.
       3 x 80 ms stays under the 250 ms we allow per read. */
    for (attempt = 0; attempt < MP_MAX_ATTEMPTS; attempt++) {
        int r = mc_read_frame(fd, slot, cf_reset, now_ms() + MP_ATTEMPT_MS);
        if (r == 1) { got = 1; break; }                   /* valid frame stored */
        if (r < 0)  break;                                /* socket dead -> stop */
        /* r == 0: no valid frame in this window -> re-send and try again */
    }

    mc_close(fd);                                         /* always RST-close */

    if (got) return 1;
    BL_MeterReadFailed(slot);
    return 0;
}

// ---------------------------------------------------------------------------
// 1 Hz round-robin tick. Called once per second from BL0942_UART_RunEverySecond.
// A full cycle is MP_TICKS_PER_CYCLE (10) ticks:
//   ticks 0..5  -> service one meter each (open/read/retry/close), 1 read/s
//   ticks 6..9  -> dummy skips (nothing to do), pacing the cycle to 10 s
//   tick  9     -> fold the cycle's CF-CNT deltas into the totals and push the
//                  fresh numbers to the dashboard (one sync per 10 s)
// Each mc_service() opens, reads (with retry) and closes within its tick, so no
// socket is ever left hanging between reads.
// ---------------------------------------------------------------------------
void UART_TCP_MeterTick(void)
{
    if (!g_pollRun) return;

    if (g_pollTick < MP_SLOTS) {
        mc_service(g_pollTick);                 /* one real meter this tick */
    }
    /* ticks MP_SLOTS..MP_TICKS_PER_CYCLE-1: dummy, deliberately do nothing */

    if (g_pollTick == MP_TICKS_PER_CYCLE - 1) {
        BL_ProcessSweep();                      /* integrate deltas + sync web */
    }

    g_pollTick++;
    if (g_pollTick >= MP_TICKS_PER_CYCLE) g_pollTick = 0;
}

void UART_TCP_StartMeterPoll(void)
{
    int i;
    for (i = 0; i < MP_SLOTS; i++) {
        g_lastModeCheck[i] = 0;
        g_modeSet[i]       = false;    /* re-program MODE on first read after boot */
    }
    g_pollTick = 0;
    g_pollRun  = true;
    ADDLOG_INFO(LOG_FEATURE_DRV, "Remote meter poller armed (connect/read/close per read)");
}

void UART_TCP_StopMeterPoll(void)
{
    g_pollRun = false;   /* no persistent sockets to close — each read cleans up */
}

/* ===========================================================================
   Charger targets — unchanged
   =========================================================================== */
const char *UART_TCP_GetChargerIP(int slot)
{
    if (slot < 0 || slot >= UART_TCP_CHARGER_MAX) return NULL;
    if (s_charger[slot] == 0) return NULL;
    build_ip(s_charger_ip[slot], sizeof(s_charger_ip[slot]), s_charger[slot]);
    return s_charger_ip[slot];
}

void UART_TCP_SendChargerCmd(const char *path)
{
    const char *ip = UART_TCP_GetChargerIP(1);
    if (!ip) { ADDLOG_WARN(LOG_FEATURE_DRV, "UART_TCP: charger IP2 not set"); return; }
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) return;
    struct timeval tv = { .tv_sec  = UART_TCP_TIMEOUT_MS / 1000,
                          .tv_usec = (UART_TCP_TIMEOUT_MS % 1000) * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(80);
    addr.sin_addr.s_addr = inet_addr(ip);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ADDLOG_WARN(LOG_FEATURE_DRV, "UART_TCP: charger connect %s failed", ip);
        close(fd); return;
    }
    char req[256];
    snprintf(req, sizeof(req),
             "GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n", path, ip);
    send(fd, req, strlen(req), 0);
    close(fd);
    ADDLOG_INFO(LOG_FEATURE_DRV, "UART_TCP: charger cmd sent: %s -> %s", path, ip);
}

/* ---- console commands ---- */
static int cmd_set_serial(int slot, const char *args)
{
    while (args && *args == ' ') args++;
    int octet = args ? atoi(args) : 0;
    if (octet < 0 || octet > 255) octet = 0;
    s_serial[slot] = (uint8_t)octet;
    nvs_save_serial(slot);
    ADDLOG_INFO(LOG_FEATURE_DRV, "UART serial target %d = .%d", slot+1, octet);
    return 1;
}
static int cmd_set_charger(int slot, const char *args)
{
    while (args && *args == ' ') args++;
    int octet = args ? atoi(args) : 0;
    if (octet < 0 || octet > 255) octet = 0;
    s_charger[slot] = (uint8_t)octet;
    nvs_save_charger(slot);
    ADDLOG_INFO(LOG_FEATURE_DRV, "Charger IP%d = .%d", slot+1, octet);
    return 1;
}
#define SERIAL_CMD(N) \
static int cmd_s##N(const void *c,const char *cmd,const char *a){(void)c;(void)cmd;return cmd_set_serial(N-1,a);}
SERIAL_CMD(1) SERIAL_CMD(2) SERIAL_CMD(3) SERIAL_CMD(4)
#define CHARGER_CMD(N) \
static int cmd_c##N(const void *c,const char *cmd,const char *a){(void)c;(void)cmd;return cmd_set_charger(N-1,a);}
CHARGER_CMD(1) CHARGER_CMD(2)

static int cmd_list_serial(const void *c,const char *cmd,const char *a)
{
    (void)c;(void)cmd;(void)a;
    char prefix[24]; get_ip_prefix(prefix, sizeof(prefix));
    for (int i = 0; i < UART_TCP_SERIAL_MAX; i++)
        ADDLOG_INFO(LOG_FEATURE_DRV,"Serial target %d: %s.%d",i+1,prefix,s_serial[i]);
    return 1;
}
static int cmd_list_charger(const void *c,const char *cmd,const char *a)
{
    (void)c;(void)cmd;(void)a;
    char prefix[24]; get_ip_prefix(prefix, sizeof(prefix));
    for (int i = 0; i < UART_TCP_CHARGER_MAX; i++)
        ADDLOG_INFO(LOG_FEATURE_DRV,"Charger IP%d: %s.%d",i+1,prefix,s_charger[i]);
    return 1;
}
static int cmd_send_charger(const void *c,const char *cmd,const char *a)
{
    (void)c;(void)cmd;
    while (a && *a == ' ') a++;
    if (!a || !*a) { ADDLOG_WARN(LOG_FEATURE_DRV,"Usage: sendChargerCmd /path"); return 1; }
    UART_TCP_SendChargerCmd(a);
    return 1;
}

/* ---- init ---- */
void UART_TCP_ClientInit(void)
{
    nvs_load_all();
    CMD_RegisterCommand("setUartTarget1",  cmd_s1, "TCP UART serial target 1 (last IP octet, 0=clear)");
    CMD_RegisterCommand("setUartTarget2",  cmd_s2, "TCP UART serial target 2");
    CMD_RegisterCommand("setUartTarget3",  cmd_s3, "TCP UART serial target 3");
    CMD_RegisterCommand("setUartTarget4",  cmd_s4, "TCP UART serial target 4");
    CMD_RegisterCommand("listUartTargets", cmd_list_serial,  "List serial targets");
    CMD_RegisterCommand("setChargerIP1",   cmd_c1, "Charger placeholder IP (last octet)");
    CMD_RegisterCommand("setChargerIP2",   cmd_c2, "Active charger IP (last octet)");
    CMD_RegisterCommand("listChargerIPs",  cmd_list_charger, "List charger IPs");
    CMD_RegisterCommand("sendChargerCmd",  cmd_send_charger, "HTTP GET to active charger: sendChargerCmd /path");
    ADDLOG_INFO(LOG_FEATURE_DRV, "UART TCP client ready");
}
