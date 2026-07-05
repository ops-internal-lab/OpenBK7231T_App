#pragma once

/* ===========================================================================
   drv_mqtt_stream.h  --  grouped BMS + energy + system MQTT streamer

   Publishes a curated set of values to MQTT under the device base topic,
   publish-on-change (with a per-item deadband for noisy analog values) plus a
   periodic heartbeat so retained topics never go stale in Home Assistant.

   Call MQTTStream_Start() once, after MQTT_init(). No-op when ENABLE_MQTT==0.
   =========================================================================== */

void MQTTStream_Start(void);
