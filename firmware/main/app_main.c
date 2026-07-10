/**
 * @file app_main.c
 * @brief Boot orchestration: wires up the per-subsystem modules.
 *
 * Core 1 runs the hard real-time control_task (GPTimer ISR -> read MPU6050 -> PWM).
 * Core 0 runs Wi-Fi + the HTTP/WebSocket server + reporter_task, which streams
 * telemetry (/ws) and accepts OTA uploads (/ota). The robot hosts its own AP;
 * connect and open http://192.168.4.1/ for a live log + command terminal.
 *
 * Wiring: MPU6050 SDA=21 SCL=22 (addr 0x68). Motors (XY-160D) L EN=25 IN=26,27,
 * R EN=33 IN=32,14. Encoders L A=18 B=19, R A=23 B=13 (PCNT). LEDs L=17 R=16.
 */
#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_ota_ops.h"        /* OTA: report the slot we booted from */
#include "nvs_flash.h"

#include "led.h"
#include "telemetry.h"
#include "wifi_ap.h"
#include "web_server.h"
#include "control.h"

static const char *TAG = "balance_bot";

void app_main(void)
{
    printf("\n=== Self-Balancing Vehicle - Wi-Fi log + OTA ===\n");
    ESP_LOGI(TAG, "core 1 = control loop, core 0 = Wi-Fi + HTTP + reporter");

    /* Show which OTA slot we booted from - flips after each successful upload. */
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "running from partition '%s' (0x%06" PRIx32 ", %" PRIu32 " KB)",
             running->label, (uint32_t)running->address, running->size / 1024);

    /* NVS is required by the Wi-Fi driver; erase + retry on an older layout. */
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    led_init();     /* Left/Right LEDs for notifications (core 0 heartbeat only) */

    /* Encoders, motors and I2C+MPU6050 are brought up in control_task (core 1) so
     * their interrupts stay off core 0's Wi-Fi. Only the LED (above) is on core 0. */

    /* Cached telemetry snapshot shared with the web server. */
    telemetry_init();

    /* Wi-Fi access point + web server (both live on core 0). */
    wifi_init_softap();
    start_webserver();

    /* Control loop (core 1) + reporter (core 0) + the GPTimer that drives them. */
    control_start();

    /* Default "alive" heartbeat - extend with your own led_blink() calls. */
    led_heartbeat();
}
