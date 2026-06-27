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
/* 500ms — 4 dead devices × 500ms = 2s worst case, safe under 5s watchdog */
#define UART_TCP_TIMEOUT_MS    500

/* Boot init — loads NVS, registers console commands */
void        UART_TCP_ClientInit(void);

/* ---- Serial UART targets ---- */
/* Returns full IP string for current round-robin slot, NULL if none set */
const char *UART_TCP_GetCurrentTarget(void);
void        UART_TCP_AdvanceTarget(void);
/* Returns slot index 0-3 of last connection, -1 if hardware UART */
int         UART_TCP_GetLastSlot(void);

/* Open TCP connection to ip:UART_TCP_PORT with 500ms timeout.
   Returns socket fd or -1 on failure. */
int         UART_TCP_Connect(const char *ip, int port);

/* ---- Charger targets ---- */
/* Returns full IP of charger slot (0=placeholder, 1=active), NULL if unset */
const char *UART_TCP_GetChargerIP(int slot);
/* Send HTTP GET /path to the active charger (slot 1).
   Fire-and-forget — does not wait for response body. */
void        UART_TCP_SendChargerCmd(const char *path);
