#include "wifi_ap.h"

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"

static const char *TAG = "wifi_ap";

/* --- Access point (connect to this from the PC, then open ws://192.168.4.1/ws) --- */
#define WIFI_AP_SSID     "espnow-station"
#define WIFI_AP_PASS     "espnow123"    /* must be >= 8 chars; "" = open network */
#define WIFI_AP_MAX_CONN 4

void wifi_init_softap(void)
{
    /* Wi-Fi needs NVS (caller does nvs_flash_init), a network interface and the
     * default event loop. */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    /* Keep config in RAM: the AP is set up in code every boot, so persisting it to
     * NVS only adds runtime flash writes. */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t ap = {
        .ap = {
            .ssid           = WIFI_AP_SSID,
            .ssid_len       = strlen(WIFI_AP_SSID),
            .channel        = WIFI_AP_CHANNEL,   /* ESP-NOW rides this same channel */
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };
    if (strlen(WIFI_AP_PASS) == 0) {
        ap.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        strncpy((char *)ap.ap.password, WIFI_AP_PASS, sizeof(ap.ap.password) - 1);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP '%s' on channel %d - connect, then open ws://192.168.4.1/ws",
             WIFI_AP_SSID, WIFI_AP_CHANNEL);
}
