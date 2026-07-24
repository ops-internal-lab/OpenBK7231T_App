#if PLATFORM_ESPIDF || PLATFORM_ESP8266

#include "../../new_common.h"
#include "../../logging/logging.h"
#include "../../quicktick.h"
#include <arch/sys_arch.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#if PLATFORM_ESP8266
#include "esp_netif.h"
#endif
void app_main(void);

void Main_Init();
void Main_OnEverySecond();
#if !PLATFORM_ESP8266
void HAL_BTProxy_PreInit(void);
void HAL_BTProxy_OnEverySecond(void);
#endif
#ifdef ENABLE_JK_BMS
#include "../../driver/drv_jkbms.h"
#include "../../driver/drv_mqtt_stream.h"
#endif
#include "../../driver/drv_uart_tcp_client.h"
float g_wifi_temperature = 0;

#if !CONFIG_IDF_TARGET_ESP32 && !PLATFORM_ESP8266

#include "driver/temperature_sensor.h"

#define TEMP_STACK_SIZE 1024

temperature_sensor_handle_t temp_handle = NULL;

void temp_func(void* pvParameters)
{
    for(;;)
    {
        temperature_sensor_enable(temp_handle);
        temperature_sensor_get_celsius(temp_handle, &g_wifi_temperature);
        temperature_sensor_disable(temp_handle);
        sys_delay_ms(10000);
    }
}

#endif

void app_main(void)
{
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_netif_init();
    esp_event_loop_create_default();
#if !CONFIG_IDF_TARGET_ESP32 && !PLATFORM_ESP8266
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    temperature_sensor_install(&temp_sensor_config, &temp_handle);
    xTaskCreate(temp_func, "IntTemp", TEMP_STACK_SIZE, NULL, 16, NULL);
#endif

#if PLATFORM_ESP8266
    uint8_t mac[6];
    if(esp_base_mac_addr_get((unsigned char*)&mac) != ESP_OK)
    {
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        esp_base_mac_addr_set((unsigned char*)&mac);
    }
#endif

    Main_Init();

    /* TCP UART client — load saved targets and register console commands */
    UART_TCP_ClientInit();

#ifdef ENABLE_JK_BMS
    /* Start BLE connection to JK-BMS (NVS must be init'd by Main_Init first) */
    JKBMS_AutoStart();
#endif

    /* MQTT streamer is an OpenBeken DRIVER now — it is NOT force-started
       here. Add `startDriver MQTTStream` to autoexec.bat (or start/stop it
       from the web UI / console). Because autoexec is skipped in safe mode,
       a buggy publisher can always be recovered without a serial reflash. */

    /* Fixed-rate 1 Hz scheduling. The old sys_delay_ms(1000) + work pattern
       made the real period 1 s PLUS the work time — with meter polls taking
       150-680 ms on 6 of every 10 ticks, the "10 second" meter cycle
       stretched to ~12 s and every timer (heartbeats, uptime, ring rollover)
       drifted. vTaskDelayUntil anchors each tick to an absolute deadline so
       the work time is absorbed, not accumulated. If one tick overruns 1 s,
       FreeRTOS just runs the next one immediately and stays on grid. */
    {
        TickType_t lastWake = xTaskGetTickCount();
        while(1)
        {
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
            Main_OnEverySecond();
            /* MAIN-LOOP MQTT publish: sends at most one message, and only if
               this second's meter tick flagged itself quiet (no meter I/O). */
            MQTTStream_RunPendingTick();
        }
    }
}

#endif // PLATFORM_ESPIDF
