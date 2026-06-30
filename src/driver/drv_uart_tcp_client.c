/* ==========================================================================
   drv_uart_tcp_client.c  --  Remote UART-over-TCP + charger HTTP client

   Serial targets (UART_TCP_SERIAL_MAX = 4):
     setUartTarget1..4 <last_octet>   — set/clear (0 = clear)
     listUartTargets                  — show all slots

   Charger targets (UART_TCP_CHARGER_MAX = 2):
     setChargerIP1 <last_octet>       — placeholder charger IP
     setChargerIP2 <last_octet>       — active charger (receives HTTP cmds)
     sendChargerCmd <path>            — GET /path on active charger IP
     listChargerIPs                   — show charger slots

   Full IP is built at connect time:
     prefix = first 3 octets of device's own WiFi IP
     full   = prefix + "." + stored_octet
   ========================================================================== */

#include "../new_common.h"
#include "../logging/logging.h"
#include "../cmnds/cmd_local.h"
#include "../hal/hal_wifi.h"
#include "drv_uart_tcp_client.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>

/* ---- storage: last octet only (0 = unset) ---- */
static uint8_t s_serial[UART_TCP_SERIAL_MAX]  = {0};
static uint8_t s_charger[UART_TCP_CHARGER_MAX] = {0};
static int     s_current   = 0;
static int     s_last_slot = -1;

/* assembled full IPs (rebuilt each connect from HAL_GetMyIPString) */
static char s_serial_ip [UART_TCP_SERIAL_MAX] [24];
static char s_charger_ip[UART_TCP_CHARGER_MAX][24];

static const char * const s_serial_keys[UART_TCP_SERIAL_MAX] = {
    "utcp_s1", "utcp_s2", "utcp_s3", "utcp_s4"
};
static const char * const s_charger_keys[UART_TCP_CHARGER_MAX] = {
    "utcp_c1", "utcp_c2"
};

/* ---- IP prefix helper ---- */
/* Builds "192.168.5" from "192.168.5.45" into buf (must be >=16 bytes).
   Returns buf, or empty string if IP not yet available. */
static const char *get_ip_prefix(char *buf, int bufsz)
{
    const char *myip = HAL_GetMyIPString();
    if (!myip || !*myip) { buf[0] = '\0'; return buf; }
    strncpy(buf, myip, bufsz - 1);
    buf[bufsz - 1] = '\0';
    /* Remove last octet */
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

/* Public: build "<our-subnet>.<octet>" into out. */
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
        uint8_t v = 0;
        nvs_get_u8(h, s_serial_keys[i], &v);
        s_serial[i] = v;
    }
    for (int i = 0; i < UART_TCP_CHARGER_MAX; i++) {
        uint8_t v = 0;
        nvs_get_u8(h, s_charger_keys[i], &v);
        s_charger[i] = v;
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

/* ---- public: serial targets ---- */
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

int UART_TCP_GetLastSlot(void) { return s_last_slot; }

void UART_TCP_AdvanceTarget(void)
{
    s_current = (s_current + 1) % UART_TCP_SERIAL_MAX;
}

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
    ADDLOG_INFO(LOG_FEATURE_DRV, "UART_TCP: connected to %s:%d (slot %d)",
                ip, port, s_last_slot);
    return fd;
}

/* Poll one BL0942 meter over serial-over-TCP.
   Connects to ip:port, sends the BL0942 read request, then accumulates up to
   outlen response bytes (each recv bounded by the 50 ms SO_RCVTIMEO set in
   Connect). The socket is ALWAYS closed before returning — one read, no
   lingering connection, nothing held between cycles. Returns the number of
   bytes received (the caller validates/parses the 23-byte 0x55 frame), or -1
   on connect/send failure. */
int UART_TCP_PollMeter(const char *ip, int port, uint8_t *out, int outlen)
{
    int got = 0;
    int fd  = UART_TCP_Connect(ip, port);
    if (fd < 0) return -1;

    /* BL0942 UART read request: CMD_READ(addr=0)=0x58, then REG_PACKET=0xAA. */
    {
        uint8_t req[2] = { 0x58, 0xAA };
        if (send(fd, req, sizeof(req), 0) < 0) { close(fd); return -1; }
    }

    while (got < outlen) {
        int n = recv(fd, out + got, outlen - got, 0);   /* 50 ms budget */
        if (n <= 0) break;                              /* timeout / close / error */
        got += n;
    }
    close(fd);
    return got;
}

/* ---- public: charger targets ---- */
const char *UART_TCP_GetChargerIP(int slot)
{
    if (slot < 0 || slot >= UART_TCP_CHARGER_MAX) return NULL;
    if (s_charger[slot] == 0) return NULL;
    build_ip(s_charger_ip[slot], sizeof(s_charger_ip[slot]), s_charger[slot]);
    return s_charger_ip[slot];
}

void UART_TCP_SendChargerCmd(const char *path)
{
    const char *ip = UART_TCP_GetChargerIP(1);   /* slot 1 = active charger */
    if (!ip) {
        ADDLOG_WARN(LOG_FEATURE_DRV, "UART_TCP: charger IP2 not set");
        return;
    }
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) return;

    /* Use the same 50ms timeout as serial targets.
       The old hardcoded 2s blocked the caller (main task) for 2 full seconds
       on every failed charger command, which added directly to HTTP latency. */
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
             "GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n",
             path, ip);
    send(fd, req, strlen(req), 0);
    /* fire-and-forget — don't wait for response */
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
    if (octet)
        ADDLOG_INFO(LOG_FEATURE_DRV, "UART serial target %d = .%d", slot+1, octet);
    else
        ADDLOG_INFO(LOG_FEATURE_DRV, "UART serial target %d cleared (hardware UART)", slot+1);
    return 1;
}

static int cmd_set_charger(int slot, const char *args)
{
    while (args && *args == ' ') args++;
    int octet = args ? atoi(args) : 0;
    if (octet < 0 || octet > 255) octet = 0;
    s_charger[slot] = (uint8_t)octet;
    nvs_save_charger(slot);
    if (octet)
        ADDLOG_INFO(LOG_FEATURE_DRV, "Charger IP%d = .%d", slot+1, octet);
    else
        ADDLOG_INFO(LOG_FEATURE_DRV, "Charger IP%d cleared", slot+1);
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
    for (int i = 0; i < UART_TCP_SERIAL_MAX; i++) {
        if (s_serial[i])
            ADDLOG_INFO(LOG_FEATURE_DRV,"Serial target %d: %s.%d",i+1,prefix,s_serial[i]);
        else
            ADDLOG_INFO(LOG_FEATURE_DRV,"Serial target %d: (empty — hardware UART if all empty)",i+1);
    }
    return 1;
}

static int cmd_list_charger(const void *c,const char *cmd,const char *a)
{
    (void)c;(void)cmd;(void)a;
    char prefix[24]; get_ip_prefix(prefix, sizeof(prefix));
    const char *labels[2] = {"placeholder","active charger"};
    for (int i = 0; i < UART_TCP_CHARGER_MAX; i++) {
        if (s_charger[i])
            ADDLOG_INFO(LOG_FEATURE_DRV,"Charger IP%d (%s): %s.%d",i+1,labels[i],prefix,s_charger[i]);
        else
            ADDLOG_INFO(LOG_FEATURE_DRV,"Charger IP%d (%s): (empty)",i+1,labels[i]);
    }
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
