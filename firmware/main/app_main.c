/**
 * @file app_main.c
 * @brief Boot orchestration for the self-balancing vehicle firmware.
 *
 * Architecture:
 *
 *     Core 1 (APP_CPU):  control_task  - hard real-time loop, no slow work
 *           GPTimer ISR --notify--> control_task --> read MPU6050 --> LEDC PWM
 *           control_task --(1-slot queue)--> telemetry snapshot (incl. IMU)
 *
 *     Core 0 (PRO_CPU):  Wi-Fi + HTTP/WebSocket server + reporter_task
 *           reporter_task caches the latest snapshot and, if streaming is on,
 *           broadcasts it to connected WebSocket clients
 *           /ws  : full-duplex channel - server streams telemetry JSON, browser
 *                  sends text commands (help/stats/stream on|off)
 *           /ota : POST a new .bin; it is written to the spare app slot and the
 *                  board reboots into it - no USB cable needed to reflash.
 *
 * The robot hosts its own Wi-Fi access point. Connect your PC/phone to it and
 * open http://192.168.4.1/ for a live log + command terminal - no USB cable.
 *
 * Each subsystem lives in its own module (led, motors, encoders, imu, wifi_ap,
 * web_server, control); this file just wires them together at boot.
 *
 * Wiring the MPU6050 (GY-521 module has its own pull-ups):
 *   VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22, AD0 -> GND (addr 0x68).
 *
 * Motors (XY-160D): L EN=25 IN=26,27   R EN=33 IN=32,14.
 * Encoders (quadrature A/B): L A=18 B=19   R A=23 B=13 (PCNT, internal pull-ups).
 * LEDs/notify: Left=GPIO17, Right=GPIO16.
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

    /* NVS is required by the Wi-Fi driver. Erase + retry if the partition is
     * from an older layout. */
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    led_init();     /* Left/Right LEDs for notifications (core 0 heartbeat only) */

    /* NOTE: every peripheral the control loop drives - encoders (PCNT), motors
     * (LEDC) and I2C + MPU6050 - is deliberately brought up in control_task (see
     * control.c) so their interrupts land on core 1, keeping the real-time path
     * isolated from core 0's Wi-Fi. Only the LED (above) stays on core 0. */

    /* Cached telemetry snapshot shared with the web server. */
    telemetry_init();

    /* Wi-Fi access point + web server (both live on core 0). */
    wifi_init_softap();
    start_webserver();

    /* Control loop (core 1) + reporter (core 0) + the GPTimer that drives them. */
    control_start();

    /* Default "alive" notification - replace/extend with your own led_blink()
     * calls (e.g. error codes, Wi-Fi client connected, OTA in progress). */
    led_heartbeat();
}
