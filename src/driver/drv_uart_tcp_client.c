/* ==========================================================================
   drv_uart_tcp_client.c  --  Remote UART-over-TCP client (MASTER side)

   Topology: ONE long-lived task (MeterPoll_Task) keeps a PERSISTENT socket
   open to each configured meter (no per-poll connect churn / TIME_WAIT
   exhaustion). Every socket has a single owner (the task) and a single close
   site (mc_close) that nulls the fd, so nothing leaks and nothing
   double-closes.

   Cadence: ONE meter is polled per second, round-robin through slots 0..5,
   then the cycle restarts -> each meter is read every ~6 s, and
   BL_ProcessSweep (-> BL_ProcessUpdate, the engine) runs once per completed
   cycle. All reconnect work for a slot happens inside its own 1 s window.

   Chip config: the BL0942 MODE register is VOLATILE (reverts to absolute,
   unsigned CF_CNT accumulation on chip power loss). The poller therefore
   pushes the config (WRPROT unlock + MODE 0x0F algebraic/free-running +
   WA_CREEP) through EACH meter's own socket right after every (re)connect,
   and re-sends it periodically as brown-out insurance. This is the ONLY
   reliable delivery path - the legacy HAL write path reaches at most one
   meter and is no longer used on this build (see BL0942_UART_Init).

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
#define MP_SLOT_PERIOD_MS 1000             /* ONE meter polled per second ->
                                              full 6-meter cycle every 6 s      */
#define MP_RETRY_GAP_MS  3000              /* backoff before reconnecting a slot */
#define MP_MAX_TIMEOUTS  3                 /* consecutive timeouts -> reconnect  */
#define MP_CFG_REFRESH_VISITS 100          /* re-send chip config every ~100
                                              visits (~10 min at 6 s/visit):
                                              MODE is volatile and a chip can
                                              brown-out while the bridge (and
                                              our socket) stays up             */
#define MP_RXCAP         64                /* per-slot resync buffer            */
#define MP_SLOTS         6

typedef struct {
    int           fd;
    int           rxlen;
    unsigned      timeouts;
    unsigned      cfg_visits;   /* polls since chip config was last sent */
    uint32_t      next_retry_ms;
    unsigned char rx[MP_RXCAP];
} meter_conn_t;

static meter_conn_t g_mc[MP_SLOTS];
static TaskHandle_t  g_pollTask = NULL;
static volatile bool g_pollRun = false;
static volatile bool g_pollDone = false;

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
    g_mc[slot].rxlen      = 0;
    g_mc[slot].timeouts   = 0;
    g_mc[slot].cfg_visits = 0;
}

/* Push the BL0942 config (WRPROT unlock + MODE algebraic/free-running +
   WA_CREEP) through this slot's own socket. MODE is volatile on the chip, so
   this runs after EVERY (re)connect and periodically (MP_CFG_REFRESH_VISITS).
   Without it a chip that lost power reverts to absolute accumulation and
   CF_CNT counts export as import - the signed-energy model silently breaks.
   Writes have no response frame, so a fixed settle delay is the only sync:
   3 frames x 6 B at 4800 baud ~= 38 ms on the slave's wire, +margin. */
static void mc_sendConfig(int slot)
{
    unsigned char frames[24];
    int n = BL0942_BuildConfigFrames(frames, sizeof(frames));
    if (n <= 0 || g_mc[slot].fd == INVALID_SOCK) return;
    if (send(g_mc[slot].fd, frames, n, 0) != n) {
        ADDLOG_WARN(LOG_FEATURE_DRV, "Meter %d: chip config send failed", slot + 1);
        return;
    }
    g_mc[slot].cfg_visits = 0;
    rtos_delay_milliseconds(60);
    ADDLOG_INFO(LOG_FEATURE_DRV,
                "Meter %d: chip configured (algebraic CF_CNT, creep=64)", slot + 1);
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
    g_mc[slot].fd         = fd;   /* stays non-blocking for reads */
    g_mc[slot].rxlen      = 0;
    g_mc[slot].timeouts   = 0;
    g_mc[slot].cfg_visits = 0;
    ADDLOG_INFO(LOG_FEATURE_DRV, "Meter %d connected (%s)", slot + 1, ip);
    /* Configure the chip immediately: MODE reverted to absolute-mode default
       if the chip (not just the link) went down since we last talked to it. */
    mc_sendConfig(slot);
    return fd;
}

/* Poll one connected slot. Returns: 1 good frame stored, 0 timeout/no frame,
   -1 hard error (caller closes the socket). */
static int mc_poll(int slot)
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
            consumed = BL0942_TCP_ScanStore(m->rx, m->rxlen, slot);
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

static void MeterPoll_Task(void* arg)
{
    int slot = 0;
    (void)arg;

    for (int i = 0; i < MP_SLOTS; i++) {
        g_mc[i].fd = INVALID_SOCK;
        g_mc[i].rxlen = 0;
        g_mc[i].timeouts = 0;
        g_mc[i].cfg_visits = 0;
        g_mc[i].next_retry_ms = 0;
    }

    /* ONE meter per second, round-robin 0..5, then start over. Each meter is
       therefore read every ~6 s; BL_ProcessSweep (-> BL_ProcessUpdate) runs
       once per completed cycle, so the shared layer's 2-sample averaged power
       spans ~12 s: avg_W = (E0+E1)*3600/(t0+t1). All (re)connect work for a
       slot happens inside that slot's own 1 s window (worst case: 250 ms
       connect + 60 ms config + 250 ms read < 1 s), so a dead slave costs only
       its own second, never the others'. */
    while (g_pollRun) {
        uint32_t t0  = now_ms();
        int      oct = BL_GetMeterOctet(slot);

        if (oct == 0) {                              /* unset -> hard offline */
            if (g_mc[slot].fd != INVALID_SOCK) mc_close(slot);
            BL_SetMeterReading(slot, 0, 0, 0, 0, 0, 0);
        } else {
            /* (re)connect this slot inside its own window, throttled */
            if (g_mc[slot].fd == INVALID_SOCK &&
                (int32_t)(now_ms() - g_mc[slot].next_retry_ms) >= 0) {
                char ip[24];
                UART_TCP_BuildIP(ip, sizeof(ip), (unsigned char)oct);
                if (mc_connect(slot, ip) < 0)        /* mc_connect sends config */
                    g_mc[slot].next_retry_ms = now_ms() + MP_RETRY_GAP_MS;
            }

            if (g_mc[slot].fd == INVALID_SOCK) {
                BL_MeterReadFailed(slot);            /* keep last-good, age out */
            } else {
                /* periodic chip re-config: a chip can brown-out and revert to
                   absolute accumulation while the bridge (and our socket)
                   stays up. Cheap insurance every ~10 min per slot. */
                if (++g_mc[slot].cfg_visits >= MP_CFG_REFRESH_VISITS)
                    mc_sendConfig(slot);

                int r = mc_poll(slot);
                if (r == 1) {
                    g_mc[slot].timeouts = 0;         /* frame stored, online */
                } else if (r < 0) {
                    mc_close(slot);                  /* drop; reconnect later */
                    g_mc[slot].next_retry_ms = now_ms() + MP_RETRY_GAP_MS;
                    BL_MeterReadFailed(slot);
                } else {
                    if (++g_mc[slot].timeouts >= MP_MAX_TIMEOUTS) {
                        mc_close(slot);
                        g_mc[slot].next_retry_ms = now_ms() + MP_RETRY_GAP_MS;
                    }
                    BL_MeterReadFailed(slot);
                }
            }
        }

        /* advance; end of a full 6-slot cycle -> aggregate into the engine */
        slot++;
        if (slot >= MP_SLOTS) {
            slot = 0;
            BL_ProcessSweep();
        }

        /* pace: exactly one meter per second regardless of poll duration */
        {
            uint32_t spent = now_ms() - t0;
            if (spent < MP_SLOT_PERIOD_MS)
                rtos_delay_milliseconds(MP_SLOT_PERIOD_MS - spent);
        }
    }

    for (int i = 0; i < MP_SLOTS; i++) mc_close(i);
    g_pollDone = true;
    vTaskDelete(NULL);
}

void UART_TCP_StartMeterPoll(void)
{
    if (g_pollTask != NULL) return;
    g_pollRun = true;
    g_pollDone = false;
    if (xTaskCreate((TaskFunction_t)MeterPoll_Task, "MeterPoll",
                    6144, NULL, 5, &g_pollTask) != pdPASS) {
        g_pollTask = NULL;
        g_pollRun = false;
        ADDLOG_ERROR(LOG_FEATURE_DRV, "MeterPoll: task create failed");
        return;
    }
    ADDLOG_INFO(LOG_FEATURE_DRV, "Remote meter poller started");
}

void UART_TCP_StopMeterPoll(void)
{
    int i;
    if (g_pollTask == NULL) return;
    g_pollRun = false;
    for (i = 0; i < 50 && !g_pollDone; i++) rtos_delay_milliseconds(10);
    if (!g_pollDone && g_pollTask != NULL) vTaskDelete(g_pollTask); /* force if stuck */
    g_pollTask = NULL;
    g_pollDone = false;
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
