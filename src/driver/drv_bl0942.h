#pragma once
#ifndef __DRV_BL0942_H__
#define __DRV_BL0942_H__

void BL0942_UART_Init(void);
void BL0942_UART_RunEverySecond(void);
void BL0942_SPI_Init(void);
void BL0942_SPI_RunEverySecond(void);

// ---- BL0942 register / mode constants shared with the remote TCP poller ----
// MODE register (0x19) value for FREE-RUNNING SIGNED CF-CNT, confirmed against
// Belling BL0942 datasheet V1.06:
//   bits[1:0]=reserved(1,1)=0x03, [2]CF_EN=1=0x04  -> 0x07
//   [6]CF_CNT_CLR_SEL=0 -> free-running (NOT cleared on read)
//   [7]CF_CNT_ADD_SEL=0 -> algebraic (signed) accumulation, so the counter
//                          ticks up on import and down on export (24-bit
//                          two's-complement, wraps 0x000000<->0xFFFFFF).
// The chip's reset default is 0x87 (absolute-value accumulation) — detecting
// that is exactly how we spot a silent reboot.
#define BL0942_REG_MODE_ADDR        0x19
#define BL0942_MODE_FREE_RUN_SIGNED 0x07
// Only the low byte carries the accumulation/free-run/CF_EN bits; bits[9:8] are
// the UART baud select and must be ignored when verifying the mode.
#define BL0942_MODE_MATCH_MASK      0x00FF
#define BL0942_REG_WRPROT_ADDR      0x1D
#define BL0942_WRPROT_UNLOCK        0x55

// CF reset "kind" threaded into the parser (see BL0942_TCP_ScanStore):
//   NONE   = trust the delta this cycle
//   REBASE = baseline lost (fresh (re)connect / comms gap) — skip THIS delta
//            only; the chip kept counting so the interval so far is still valid
//   CHIP   = chip rebooted (MODE was not signed) — skip the delta AND discard
//            the current 15-min interval, because everything measured before
//            the reboot is tainted
#define BL0942_CF_RESET_NONE   0
#define BL0942_CF_RESET_REBASE 1
#define BL0942_CF_RESET_CHIP   2

// Scan a flat buffer for the first checksum-valid 23-byte 0x55 BL0942 frame,
// scale and store it to meter `slot`, computing the signed CF-CNT delta since
// the previous read. `cf_reset` is one of BL0942_CF_RESET_* : REBASE discards
// this cycle's delta only, CHIP additionally discards the current 15-min
// interval (grid slots). Returns bytes consumed (incl. frame), or 0.
int  BL0942_TCP_ScanStore(const unsigned char *buf, int len, int slot, int cf_reset);
#if ENABLE_BL_TWIN
void BL0942_AddCommands(void);
#endif
#endif