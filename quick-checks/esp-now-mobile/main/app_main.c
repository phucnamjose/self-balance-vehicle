/* Quick-check: the moving node that talks to the station over ESP-NOW v2.
 *
 * Auto-pairs with the station (matching broadcast-beacon handshake), then:
 *   - logs every payload it receives from the station,
 *   - echoes that payload back so the PC sees a full round-trip, and
 *   - sends a periodic heartbeat so return traffic is visible even when nobody
 *     is typing on the PC.
 * The from_station task is where real movement-command parsing would live.
 *
 * This node keeps its console on the built-in USB, so `idf.py monitor` works.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"

static const char *TAG = "mobile";

#define ESPNOW_CHANNEL  1
#define ESPNOW_CHUNK    1024        // per-packet cap; ESP-NOW v2 allows up to 1470
#define RX_STREAM_BUF   4096
#define HEARTBEAT_MS    1000

static const uint8_t BCAST_MAC[ESP_NOW_ETH_ALEN] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t PAIR_MAGIC[4] = {'E','N','P','R'};

static volatile bool        s_paired;
static uint8_t              s_peer[ESP_NOW_ETH_ALEN];
static volatile bool        s_peer_pending;
static uint8_t              s_pending[ESP_NOW_ETH_ALEN];
static StreamBufferHandle_t s_from_station;

static bool mac_is_bcast(const uint8_t *m)
{
    return memcmp(m, BCAST_MAC, ESP_NOW_ETH_ALEN) == 0;
}

// Kept minimal: no esp_now_* calls here (they run in the pairing/data tasks).
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (mac_is_bcast(info->des_addr)) {
        // Pairing beacon from the station looking for us.
        if (!s_paired && !s_peer_pending &&
            len == (int)sizeof(PAIR_MAGIC) &&
            memcmp(data, PAIR_MAGIC, sizeof(PAIR_MAGIC)) == 0) {
            memcpy(s_pending, info->src_addr, ESP_NOW_ETH_ALEN);
            s_peer_pending = true;
        }
        return;
    }
    // Unicast to us == data from the station.
    if (s_paired && len > 0) {
        xStreamBufferSend(s_from_station, data, len, 0);
    }
}

static void on_sent(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    (void)info; (void)status;
}

static void radio_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));     // ESP-NOW needs the radio always on
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    uint32_t ver = 0;
    esp_now_get_version(&ver);
    ESP_LOGI(TAG, "ESP-NOW version = %u (expect 2)", (unsigned)ver);

    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_sent));

    esp_now_peer_info_t peer = {
        .channel = ESPNOW_CHANNEL,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, BCAST_MAC, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "mobile MAC %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Broadcast a beacon until paired, then adopt the discovered peer as the unicast
// target. A few extra beacons after pairing let a slightly-later peer still find us.
static void pair_task(void *arg)
{
    (void)arg;
    int post = 0;
    for (;;) {
        if (!s_paired && s_peer_pending) {
            memcpy(s_peer, s_pending, ESP_NOW_ETH_ALEN);
            esp_now_peer_info_t peer = {
                .channel = ESPNOW_CHANNEL,
                .ifidx   = WIFI_IF_STA,
                .encrypt = false,
            };
            memcpy(peer.peer_addr, s_peer, ESP_NOW_ETH_ALEN);
            if (!esp_now_is_peer_exist(s_peer)) {
                esp_now_add_peer(&peer);
            }
            s_paired = true;
            ESP_LOGI(TAG, "paired with %02x:%02x:%02x:%02x:%02x:%02x",
                     s_peer[0], s_peer[1], s_peer[2], s_peer[3], s_peer[4], s_peer[5]);
        }

        esp_now_send(BCAST_MAC, PAIR_MAGIC, sizeof(PAIR_MAGIC));
        if (s_paired && ++post >= 4) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

// Log each payload from the station and echo it back so the PC sees a round-trip.
static void from_station_task(void *arg)
{
    (void)arg;
    uint8_t *buf = malloc(ESPNOW_CHUNK);
    if (!buf) {
        ESP_LOGE(TAG, "no memory for from_station buffer");
        vTaskDelete(NULL);
    }
    for (;;) {
        size_t n = xStreamBufferReceive(s_from_station, buf, ESPNOW_CHUNK, portMAX_DELAY);
        if (n == 0) {
            continue;
        }
        ESP_LOGI(TAG, "rx %u byte(s) from station", (unsigned)n);
        ESP_LOG_BUFFER_HEXDUMP(TAG, buf, n, ESP_LOG_INFO);
        if (s_paired) {
            esp_now_send(s_peer, buf, n);   // echo back -> station -> PC
        }
    }
}

// Periodic return traffic so the PC sees data even when it sends nothing.
static void heartbeat_task(void *arg)
{
    (void)arg;
    uint32_t n = 0;
    char line[48];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_MS));
        if (s_paired) {
            int len = snprintf(line, sizeof(line), "mobile hb #%u\n", (unsigned)n++);
            esp_now_send(s_peer, (const uint8_t *)line, len);
        }
    }
}

void app_main(void)
{
    s_from_station = xStreamBufferCreate(RX_STREAM_BUF, 1);

    radio_start();

    xTaskCreate(pair_task,         "pair",   3072, NULL, 5, NULL);
    xTaskCreate(from_station_task, "rx",     3072, NULL, 8, NULL);
    xTaskCreate(heartbeat_task,    "hb",     3072, NULL, 6, NULL);

    ESP_LOGI(TAG, "mobile up on ESP-NOW ch %d - waiting to pair", ESPNOW_CHANNEL);
}
