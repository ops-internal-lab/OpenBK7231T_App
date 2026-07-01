#pragma once
#ifndef __DRV_BL0942_H__
#define __DRV_BL0942_H__

void BL0942_UART_Init(void);
void BL0942_UART_RunEverySecond(void);
void BL0942_SPI_Init(void);
void BL0942_SPI_RunEverySecond(void);
// Scan a flat buffer for the first checksum-valid 23-byte 0x55 BL0942 frame,
// scale and store it to meter `slot`. Returns bytes consumed (incl. frame), or 0.
int  BL0942_TCP_ScanStore(const unsigned char *buf, int len, int slot);
// Build the raw UART write frames that configure a BL0942 for the signed-energy
// model (WRPROT unlock, MODE = algebraic + free-running + 800ms RMS, WA_CREEP).
// Returns bytes written into `out` (18), or 0 if maxLen < 18. Sent by the TCP
// poller through each meter's own socket after every (re)connect, because the
// chip's MODE register is volatile and reverts to absolute accumulation on
// power loss.
int  BL0942_BuildConfigFrames(unsigned char *out, int maxLen);
#if ENABLE_BL_TWIN
void BL0942_AddCommands(void);
#endif
#endif