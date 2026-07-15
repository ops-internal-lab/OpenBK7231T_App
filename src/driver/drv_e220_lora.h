/* ==========================================================================
   drv_e220_lora.h -- EBYTE E220-400T22S transparent-LoRa meter link (MASTER)

   Replaces the WiFi serial bridge per meter with a LoRa module wired
   directly to the BL0942 UART (4800 8N1). No slave MCU: the master
   addresses one slave at a time with the module's fixed-transmission
   header, so exactly one BL0942 ever answers -- collision-free.

   Slot semantics: a meter whose configured last-octet is 255 (the LoRa
   sentinel, see SetMeterIP) is serviced by this driver instead of TCP.
   Slave module address = slot + 1.
   ========================================================================== */
#ifndef DRV_E220_LORA_H
#define DRV_E220_LORA_H

/* One-time init (NVS pins/radio settings, module personality). Safe to call
   when unconfigured -- the driver just stays disabled. */
void LoRaMeter_Init(void);

/* 1 = driver configured, module initialised, ready to poll. */
int  LoRaMeter_Ready(void);

/* Service one meter slot (0..5) over the radio. Mirrors mc_service():
   MODE verify on EVERY cycle -> reprogram+abort on mismatch, else read one
   checksum-valid frame and store it. Returns like mc_service (1 stored,
   0 failed cycle, -1 not-available). */
int  LoRaMeter_Service(int slot);

/* Master-side RSSI (dBm, negative) of the LAST frame received from `slot`,
   or 0 when unknown. RAM only -- display aid, no persistence. */
int  LoRaMeter_GetRSSI(int slot);

#endif
