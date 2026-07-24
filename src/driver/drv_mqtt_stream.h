#pragma once

/* ===========================================================================
   drv_mqtt_stream.h  --  grouped BMS + energy + system MQTT streamer

   The ONLY MQTT publisher in this firmware. Publishes a curated set of values
   under the device base topic, publish-on-change (per-item deadband) plus a
   periodic heartbeat so retained topics never go stale in Home Assistant.

   No task of its own, and it never publishes from inside the meter code:
   the once-a-second meter poll loop (UART_TCP_MeterTick) only calls
   MQTTStream_MarkQuietTick() on ticks where no meter was serviced (idle
   ticks 6..9 of the 10 s cycle, unset slots, or poller off). The MAIN LOOP
   then calls MQTTStream_RunPendingTick() once per second; it publishes AT
   MOST ONE message and only if the flag was set. So publishing happens in
   the main loop, never during a poll second, and can never burst.

   MQTTStream_Start() just resets state and logs; safe to call once at boot.
   All three are no-ops when ENABLE_MQTT is 0.
   =========================================================================== */

void MQTTStream_Start(void);           /* driver start: reset + enable        */
void MQTTStream_Stop(void);            /* driver stop: publishing halts       */
void MQTTStream_MarkQuietTick(void);   /* meter task: "this second is free"   */
void MQTTStream_RunPendingTick(void);  /* main loop: publish if marked        */
