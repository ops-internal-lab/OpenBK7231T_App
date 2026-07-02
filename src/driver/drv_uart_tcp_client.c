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
   PERSISTENT 6-METER POLLER  (the actual fix)
   =========================================================================== */

#define MP_PORT          UART_TCP_PORT     /* 8888 */
#define MP_CONNECT_MS    250               /* bounded non-blocking connect wait */
#define MP_READ_MS       250               /* per-slot response deadline        */
#define MP_SWEEP_GAP_MS  300               /* pause between full sweeps         */
#define MP_RETRY_GAP_MS  3000              /* backoff before reconnecting a slot */
#define MP_MAX_TIMEOUTS  3                 /* consecutive timeouts -> reconnect  */
#define MP_RXCAP         64                /* per-slot resync buffer            */
#define MP_SLOTS         6

typedef struct {
    int           fd;
    int           rxlen;
    unsigned      timeouts;
    unsigned      just_connected;   /* 1 = next good read must re-baseline CF   */
    uint32_t      next_retry_ms;
    unsigned char rx[MP_RXCAP];
} meter_conn_t;

/* Round-robin cadence: 6 real meters + 4 dummy skips = a 10 s cycle at 1 Hz. */
#define MP_TICKS_PER_CYCLE 10
/* Single-register read budget (BL0942 replies with 4 bytes; a live LAN host
   answers in a few ms, so this is generous). */
#define MP_REG_READ_MS     150

static meter_conn_t g_mc[MP_SLOTS];
static bool         g_pollRun  = false;    /* poller enabled between Start/Stop */
static int          g_pollTick = 0;        /* 0..MP_TICKS_PER_CYCLE-1 round-robin */

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* the ONLY close site for a meter socket */
static void mc_close(int slot)
{
    if (g_mc[slot].fd != INVALID_SOCK) {
        struct linger lg = { 1, 0 };   /* RST close: no TIME_WAIT churn */
        setsockopt(g_mc[slot].fd, SOL_SOCKET, SO_LINGER, &lg, sizeof(lg));
        close(g_mc[slot].fd);
        g_mc[slot].fd = INVALID_SOCK;
    }
    g_mc[slot].rxlen    = 0;
    g_mc[slot].timeouts = 0;
}

/* bounded non-blocking connect so a dead slave never stalls the sweep */
static int mc_connect(int slot, const char *ip)
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
    if (rc != 0 && errno != EINPROGRESS) { close(fd); return -1; }
    if (rc != 0) {
        fd_set w; FD_ZERO(&w); FD_SET(fd, &w);
        struct timeval tv = { 0, MP_CONNECT_MS * 1000 };
        if (select(fd + 1, NULL, &w, NULL, &tv) <= 0) { close(fd); return -1; }
        int err = 0; socklen_t l = sizeof(err);
        getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &l);
        if (err != 0) { close(fd); return -1; }
    }
    g_mc[slot].fd             = fd;   /* stays non-blocking for reads */
    g_mc[slot].rxlen          = 0;
    g_mc[slot].timeouts       = 0;
    g_mc[slot].just_connected = 1;    /* next read re-baselines CF (no stale delta) */
    ADDLOG_INFO(LOG_FEATURE_DRV, "Meter %d connected (%s)", slot + 1, ip);
    return fd;
}

/* Poll one connected slot for a full data frame. Returns: 1 good frame stored,
   0 timeout/no frame, -1 hard error (caller closes the socket). cf_reset is
   forwarded to the parser so this cycle's CF delta is discarded when needed. */
static int mc_poll(int slot, int cf_reset)
{
    meter_conn_t *m = &g_mc[slot];
    unsigned char req[2] = { 0x58, 0xAA };
    uint32_t deadline;

    if (send(m->fd, req, 2, 0) < 0 &&
        errno != EWOULDBLOCK && errno != EAGAIN) return -1;

    deadline = now_ms() + MP_READ_MS;
    while ((int32_t)(now_ms() - deadline) < 0) {
        int n;
        if (m->rxlen >= MP_RXCAP) m->rxlen = 0;          /* overflow -> resync */
        n = recv(m->fd, m->rx + m->rxlen, MP_RXCAP - m->rxlen, 0);
        if (n > 0) {
            int consumed;
            m->rxlen += n;
            /* scan for the first checksum-valid 23-byte 0x55 frame anywhere
               in the buffer; tolerant of leading/stray bytes */
            consumed = BL0942_TCP_ScanStore(m->rx, m->rxlen, slot, cf_reset);
            if (consumed > 0) {
                int rem = m->rxlen - consumed;
                if (rem > 0) memmove(m->rx, m->rx + consumed, rem);
                m->rxlen = rem;
                return 1;
            }
        } else if (n == 0) {
            return -1;                                   /* peer closed */
        } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
            return -1;                                   /* hard error */
        } else {
            rtos_delay_milliseconds(5);                  /* nothing yet, wait */
        }
    }
    return 0;                                            /* deadline, no frame */
}

/* Read one BL0942 register over the persistent socket. Protocol mirrors the
   local SPI/UART read: send {0x58, reg}, reply is 3 data bytes (MSB first) +
   1 checksum = ~(0x58 + reg + b0 + b1 + b2). Returns 0 and *val on success,
   -1 on timeout / bad checksum / socket error. Drains any stray bytes first
   so a partial data frame left in flight can't corrupt the 4-byte reply. */
static int mc_read_reg(int slot, unsigned char reg, uint32_t *val)
{
    meter_conn_t *m = &g_mc[slot];
    unsigned char req[2] = { 0x58, reg };
    unsigned char rx[8];
    int rxn = 0;
    uint32_t deadline;

    /* flush anything already buffered on the socket */
    for (;;) {
        int n = recv(m->fd, rx, sizeof(rx), 0);
        if (n > 0) continue;
        break;
    }
    m->rxlen = 0;

    if (send(m->fd, req, 2, 0) < 0 &&
        errno != EWOULDBLOCK && errno != EAGAIN) return -1;

    deadline = now_ms() + MP_REG_READ_MS;
    while ((int32_t)(now_ms() - deadline) < 0) {
        int n = recv(m->fd, rx + rxn, (int)sizeof(rx) - rxn, 0);
        if (n > 0) {
            rxn += n;
            if (rxn >= 4) {
                unsigned char cs = (unsigned char)(0x58 + reg + rx[0] + rx[1] + rx[2]);
                cs ^= 0xFF;
                if (cs != rx[3]) return -1;              /* bad checksum */
                *val = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
                return 0;
            }
        } else if (n == 0) {
            return -1;                                   /* peer closed */
        } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
            return -1;                                   /* hard error */
        } else {
            rtos_delay_milliseconds(2);
        }
    }
    return -1;                                           /* timeout */
}

/* Write one BL0942 register over the persistent socket (fire-and-forget, no
   reply — matches the local UART write path). Value bytes are little-endian,
   checksum = ~(sum of the 5 bytes). */
static void mc_write_reg(int slot, unsigned char reg, uint32_t val)
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
    (void)send(g_mc[slot].fd, msg, sizeof(msg), 0);
}

/* Verify meter `slot` is in free-running signed CF mode; if not (or if the
   mode can't be read), unlock + reprogram it and report that a re-baseline is
   needed. Returns 1 when the CF delta for this cycle must be discarded. */
static int mc_mode_guard(int slot)
{
    uint32_t mode = 0;
    if (mc_read_reg(slot, BL0942_REG_MODE_ADDR, &mode) == 0 &&
        (mode & BL0942_MODE_MATCH_MASK) == BL0942_MODE_FREE_RUN_SIGNED) {
        return 0;                                        /* already correct */
    }
    /* Wrong mode (silent reset) or unreadable: unlock write-protect, rewrite
       MODE, and force a CF re-baseline so a bogus delta never lands. */
    mc_write_reg(slot, BL0942_REG_WRPROT_ADDR, BL0942_WRPROT_UNLOCK);
    mc_write_reg(slot, BL0942_REG_MODE_ADDR,   BL0942_MODE_FREE_RUN_SIGNED);
    ADDLOG_WARN(LOG_FEATURE_DRV,
                "Meter %d not in free-run signed mode (0x%X) - reprogrammed",
                slot + 1, (unsigned)mode);
    return 1;
}

/* Service ONE meter: (re)connect if needed, mode-guard, then read one frame
   with a single retry on a missed/corrupt frame. Returns 1 good / 0 no-frame
   / -1 offline-or-error (last-good is held and aged out). */
static int mc_service(int slot)
{
    int oct = BL_GetMeterOctet(slot);
    int cf_reset, r;

    if (oct == 0) {                                      /* unset -> hard offline */
        if (g_mc[slot].fd != INVALID_SOCK) mc_close(slot);
        BL_SetMeterReading(slot, 0, 0, 0, 0, 0);
        return -1;
    }

    if (g_mc[slot].fd == INVALID_SOCK) {                 /* (re)connect, honour backoff */
        if ((int32_t)(now_ms() - g_mc[slot].next_retry_ms) < 0) {
            BL_MeterReadFailed(slot);
            return -1;
        }
        char ip[24];
        UART_TCP_BuildIP(ip, sizeof(ip), (unsigned char)oct);
        if (mc_connect(slot, ip) < 0) {
            g_mc[slot].next_retry_ms = now_ms() + MP_RETRY_GAP_MS;
            BL_MeterReadFailed(slot);
            return -1;
        }
    }

    /* Mode check every read. A mode mismatch = chip reboot -> discard the whole
       interval (CHIP). A fresh connect just lost the baseline -> skip one delta
       (REBASE); the chip kept counting so the interval so far is still good. */
    cf_reset = mc_mode_guard(slot) ? BL0942_CF_RESET_CHIP : BL0942_CF_RESET_NONE;
    if (cf_reset == BL0942_CF_RESET_NONE && g_mc[slot].just_connected)
        cf_reset = BL0942_CF_RESET_REBASE;

    r = mc_poll(slot, cf_reset);
    if (r == 0) r = mc_poll(slot, cf_reset);             /* retry once on miss/corrupt */

    if (r == 1) {
        g_mc[slot].timeouts       = 0;
        g_mc[slot].just_connected = 0;                   /* baseline now established */
        return 1;
    }
    if (r < 0) {                                         /* hard error: drop + backoff */
        mc_close(slot);
        g_mc[slot].next_retry_ms = now_ms() + MP_RETRY_GAP_MS;
        BL_MeterReadFailed(slot);
        return -1;
    }
    if (++g_mc[slot].timeouts >= MP_MAX_TIMEOUTS) {      /* repeated silence: drop */
        mc_close(slot);
        g_mc[slot].next_retry_ms = now_ms() + MP_RETRY_GAP_MS;
    }
    BL_MeterReadFailed(slot);
    return 0;
}

// ---------------------------------------------------------------------------
// 1 Hz round-robin tick. Called once per second from BL0942_UART_RunEverySecond.
// A full cycle is MP_TICKS_PER_CYCLE (10) ticks:
//   ticks 0..5  -> service one meter each (slots 0..5), read once per 10 s
//   ticks 6..9  -> dummy skips (nothing to do), pacing the cycle to 10 s
//   tick  9     -> fold the cycle's CF-CNT deltas into the totals and push the
//                  fresh numbers to the dashboard (one sync per 10 s)
// No sleeps are added here: the 1 Hz cadence is the framework's, and the 4
// dummy ticks are what stretch the cycle to 10 s. Each mc_service() call is
// bounded (mode read + one frame read + at most one retry) so it comfortably
// fits the per-second budget.
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
        g_mc[i].fd             = INVALID_SOCK;
        g_mc[i].rxlen          = 0;
        g_mc[i].timeouts       = 0;
        g_mc[i].just_connected = 0;
        g_mc[i].next_retry_ms  = 0;
    }
    g_pollTick = 0;
    g_pollRun  = true;
    ADDLOG_INFO(LOG_FEATURE_DRV, "Remote meter poller armed (1 Hz round-robin)");
}

void UART_TCP_StopMeterPoll(void)
{
    int i;
    g_pollRun = false;
    for (i = 0; i < MP_SLOTS; i++) mc_close(i);
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
