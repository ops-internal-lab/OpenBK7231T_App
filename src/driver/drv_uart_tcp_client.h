#pragma once

/* ==========================================================================
   drv_uart_tcp_client.h

   6 configurable remote endpoints, all stored as last-octet only.
   Full IP is derived at connect time: prefix from device's own WiFi IP
   + stored octet.  e.g. device=192.168.5.45, octet=32 → 192.168.5.32

   Serial targets (4):  BL0942 UART-TCP round-robin
   Charger targets (2): HTTP GET command delivery
   ========================================================================== */

#define UART_TCP_SERIAL_MAX    4
#define UART_TCP_CHARGER_MAX   2
#define UART_TCP_PORT          8888
/* 50ms — 4 dead devices × 50ms = 200ms worst case, well under 5s watchdog.
   Aggressive but safe: a live host on a local LAN responds to TCP SYN
   in under 5ms, so 50ms is still generous for real devices.
   Applies to both SO_RCVTIMEO (data wait) and SO_SNDTIMEO (connect).
   lwIP honours SO_SNDTIMEO during blocking connect() when
   LWIP_SO_SNDTIMEO=1 (default on ESP-IDF). */
#define UART_TCP_TIMEOUT_MS    100

/* Boot init — loads NVS, registers console commands */
void        UART_TCP_ClientInit(void);

/* ---- Serial UART targets ---- */
/* Returns full IP string for current round-robin slot, NULL if none set */
const char *UART_TCP_GetCurrentTarget(void);
void        UART_TCP_AdvanceTarget(void);
/* Returns slot index 0-3 of last connection, -1 if hardware UART */
int         UART_TCP_GetLastSlot(void);

/* Open TCP connection to ip:UART_TCP_PORT with 50ms timeout.
   Returns socket fd or -1 on failure. */
int         UART_TCP_Connect(const char *ip, int port);

/* Poll one BL0942 meter: connect, send read request, recv up to outlen bytes
   (50ms budget), close. Returns bytes received, or -1 on connect/send failure. */
int         UART_TCP_PollMeter(const char *ip, int port, unsigned char *out, int outlen);

/* Build "<our-subnet>.<octet>" (e.g. "192.168.8.156") into out. */
void        UART_TCP_BuildIP(char *out, int outsz, unsigned char octet);

/* ---- Charger targets ---- */
/* Returns full IP of charger slot (0=placeholder, 1=active), NULL if unset */
const char *UART_TCP_GetChargerIP(int slot);
/* Send HTTP GET /path to the active charger (slot 1).
   Fire-and-forget — does not wait for response body. */
void        UART_TCP_SendChargerCmd(const char *path);
