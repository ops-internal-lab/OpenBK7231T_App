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
#if ENABLE_BL_TWIN
void BL0942_AddCommands(void);
#endif
#endif