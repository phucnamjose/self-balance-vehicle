#include "wifi_ap.h"

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"

static const char *TAG = "wifi_ap";

/* --- Wi-Fi access point (connect to this, then browse to 192.168.4.1) --- */
#define WIFI_AP_SSID     "balance-bot"
#define WIFI_AP_PASS     "balance123"   /* must be >= 8 chars; "" = open network */
#define WIFI_AP_CHANNEL  1
#define WIFI_AP_MAX_CONN 4

void wifi_init_softap(void)
{
    /* Wi-Fi needs NVS (stores calibration data), a network interface and the
     * default event loop. */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    wifi_config_t ap = {
        .ap = {
            .ssid           = WIFI_AP_SSID,
            .ssid_len       = strlen(WIFI_AP_SSID),
            .channel        = WIFI_AP_CHANNEL,
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

    ESP_LOGI(TAG, "Wi-Fi AP '%s' started - connect, then open http://192.168.4.1/",
             WIFI_AP_SSID);
}
