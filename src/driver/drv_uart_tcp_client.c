/* ==========================================================================
   drv_uart_tcp_client.c  --  Remote UART-over-TCP client (MASTER side)

   What changed:
   Fixed the programming sequence based on corrected trace analysis. The meter 
   does not issue replies for writes. The code now sends the Unlock write frame, 
   waits 50ms, sends the Mode program write frame, waits 50ms to let it settle, 
   and then cleanly terminates the connection. All obsolete draining code has 
   been removed.
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
   REMOTE 6-METER POLLER  --  connect / check-mode / read / close, PER READ
   =========================================================================== */

#define MP_PORT            UART_TCP_PORT   /* 8888 */
#define MP_CONNECT_MS      80              /* bounded connection wait time */

/* Timing parameters synchronized exactly to instructions */
#define MP_MODE_WAIT_MS    20              /* wait 20ms after 58 19 */
#define MP_UNLOCK_WAIT_MS  50              /* wait 50ms gap between configuration writes */
#define MP_FRAME_WAIT_MS   80              /* wait 80ms after 58 aa */

#define MP_GATHER_SPINS    12              /* extra polls to gather network fragments */
#define MP_MAX_ATTEMPTS    3               /* up to 3 frame requests if data is invalid */
#define MP_RXCAP           64              
#define MP_SLOTS           6
#define MP_TICKS_PER_CYCLE 10              /* 6 meters + 4 dummy skips = 10s cycle */

static bool g_pollRun  = false;            
static int     g_pollTick = 0;            

static void mc_close(int fd)
{
    if (fd != INVALID_SOCK) {
        struct linger lg = { 1, 0 };
        setsockopt(fd, SOL_SOCKET, SO_LINGER, &lg, sizeof(lg));
        close(fd);
    }
}

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

/* Send 58 aa, wait 80ms, gather frame. Retries automatically driven by the service loop */
static int mc_read_frame(int fd, int slot)
{
    unsigned char req[2] = { 0x58, 0xAA };
    unsigned char rx[MP_RXCAP];
    int rxlen = 0, spins;

    if (send(fd, req, 2, 0) < 0 && errno != EWOULDBLOCK && errno != EAGAIN) return -1;

    rtos_delay_milliseconds(MP_FRAME_WAIT_MS);            /* wait 80 ms */

    for (spins = 0; spins < MP_GATHER_SPINS; spins++) {
        int n;
        if (rxlen >= MP_RXCAP) rxlen = 0;                 
        n = recv(fd, rx + rxlen, MP_RXCAP - rxlen, 0);
        if (n > 0) {
            rxlen += n;
            if (BL0942_TCP_ScanStore(rx, rxlen, slot, BL0942_CF_RESET_NONE) > 0) return 1;
            spins = 0;                                    
        } else if (n == 0) {
            return -1;                                    
        } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
            return -1;                                    
        } else {
            rtos_delay_milliseconds(3);                    
        }
    }
    return 0;                                             /* Invalid data frame */
}

/* Send 58 19, wait 20ms, read and validate 4-byte response register */
static int mc_read_reg(int fd, unsigned char reg, uint32_t *val)
{
    unsigned char req[2] = { 0x58, reg };
    unsigned char rx[8];
    int rxn = 0, spins;

    if (send(fd, req, 2, 0) < 0 && errno != EWOULDBLOCK && errno != EAGAIN) return -1;

    rtos_delay_milliseconds(MP_MODE_WAIT_MS);             /* wait 20 ms */

    for (spins = 0; spins < MP_GATHER_SPINS && rxn < 4; spins++) {
        int n = recv(fd, rx + rxn, (int)sizeof(rx) - rxn, 0);
        if (n > 0) { rxn += n; spins = 0; }
        else if (n == 0) return -1;
        else if (errno != EWOULDBLOCK && errno != EAGAIN) return -1;
        else rtos_delay_milliseconds(2);
    }
    if (rxn < 4) return -1;

    {
        unsigned char cs = (unsigned char)((0x58 + reg + rx[0] + rx[1] + rx[2]) ^ 0xFF);
        if (cs != rx[3]) return -1;                       
        *val = (uint32_t)rx[0] | ((uint32_t)rx[1] << 8) | ((uint32_t)rx[2] << 16);
    }
    return 0;
}

/* Fire-and-forget write command sequence */
static void mc_write_reg(int fd, unsigned char reg, uint32_t val)
{
    unsigned char msg[6];
    int i;
    unsigned char crc = 0;
    msg[0] = 0xA8;                       
    msg[1] = reg;
    msg[2] = (unsigned char)( val        & 0xFF);
    msg[3] = (unsigned char)((val >>  8) & 0xFF);
    msg[4] = (unsigned char)((val >> 16) & 0xFF);
    for (i = 0; i < 5; i++) crc += msg[i];
    msg[5] = crc ^ 0xFF;
    (void)send(fd, msg, sizeof(msg), 0);
}

/* Service ONE meter per sequence instructions */
static int mc_service(int slot)
{
    int oct = BL_GetMeterOctet(slot);
    int fd, attempt, got = 0;
    uint32_t mode = 0;
    char ip[24];

    if (oct == 0) {                                       
        BL_SetMeterReading(slot, 0, 0, 0, 0, 0);
        return -1;
    }

    UART_TCP_BuildIP(ip, sizeof(ip), (unsigned char)oct);
    fd = mc_open(ip);
    if (fd < 0) { BL_MeterReadFailed(slot); return -1; }

    /* 1) Send 58 19, wait 20ms, read register (Checked EVERY single time) */
    if (mc_read_reg(fd, BL0942_REG_MODE_ADDR, &mode) != 0) {
        mc_close(fd);
        BL_MeterReadFailed(slot);
        return 0;
    }

    /* 2) Check if register is correct (Expecting 07 00 00) */
    if ((mode & BL0942_MODE_MATCH_MASK) != BL0942_MODE_FREE_RUN_SIGNED) {
        
        /* Register is incorrect: Unlock and Program */
        mc_write_reg(fd, BL0942_REG_WRPROT_ADDR, BL0942_WRPROT_UNLOCK);      /* Send a8 1d 55 00 00 e5 */
        rtos_delay_milliseconds(MP_UNLOCK_WAIT_MS);                          /* Wait 50ms */
        
        mc_write_reg(fd, BL0942_REG_MODE_ADDR, BL0942_MODE_FREE_RUN_SIGNED); /* Send a8 19 07 00 00 37 */
        rtos_delay_milliseconds(MP_UNLOCK_WAIT_MS);                          /* Wait 50ms for chip safety */
        
        if (slot >= 0 && slot <= 2) BL_MeterNoteReset(slot);
        ADDLOG_WARN(LOG_FEATURE_DRV,
                    "Meter %d MODE was 0x%X (reset) - reprogrammed to signed",
                    slot + 1, (unsigned)mode);
        
        /* Terminate connection immediately after programming */
        mc_close(fd);
        BL_MeterReadFailed(slot);
        return 0;
    }

    /* 3) Register is correct: Ask for data (58 aa), wait 80ms. Request again if data invalid */
    for (attempt = 0; attempt < MP_MAX_ATTEMPTS; attempt++) {
        int r = mc_read_frame(fd, slot);
        if (r == 1) { got = 1; break; }                   /* Valid frame parsed -> break out */
        if (r < 0)  break;                                /* Socket error -> abort loops */
    }

    /* 4) Terminate connection */
    mc_close(fd);

    if (got) return 1;
    BL_MeterReadFailed(slot);
    return 0;
}

void UART_TCP_MeterTick(void)
{
    if (!g_pollRun) return;

    if (g_pollTick < MP_SLOTS) {
        mc_service(g_pollTick);                 
    }

    if (g_pollTick == MP_TICKS_PER_CYCLE - 1) {
        BL_ProcessSweep();                      
    }

    g_pollTick++;
    if (g_pollTick >= MP_TICKS_PER_CYCLE) g_pollTick = 0;
}

void UART_TCP_StartMeterPoll(void)
{
    g_pollTick = 0;
    g_pollRun  = true;
    ADDLOG_INFO(LOG_FEATURE_DRV, "Remote meter poller armed (connect/read/close per read)");
}

void UART_TCP_StopMeterPoll(void)
{
    g_pollRun = false;   
}

/* ===========================================================================
   Charger targets & Console Commands — unchanged
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
