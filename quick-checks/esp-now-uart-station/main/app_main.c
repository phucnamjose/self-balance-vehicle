/* Quick-check: WebSocket <-> ESP-NOW v2 bridge (the "station" / bridge node).
 *
 * The station hosts a Wi-Fi access point and a WebSocket server: a PC connects to
 * the AP and opens ws://192.168.4.1/ws. Bytes the PC sends on the WebSocket are
 * relayed over ESP-NOW v2 to the mobile node, and bytes from the mobile are pushed
 * back to every connected WebSocket client. The station is just the bridge.
 *
 * ESP-NOW must share the SoftAP's radio channel, so the AP runs on WIFI_AP_CHANNEL
 * and ESP-NOW runs on the AP interface at that same channel. The mobile stays a
 * plain STA on the same channel and pairs unchanged.
 *
 * Pairing is automatic: both nodes broadcast a small beacon until they hear each
 * other, then lock to that peer and talk unicast.
 */
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"

#include "wifi_ap.h"
#include "web_server.h"

static const char *TAG = "station";

#define ESPNOW_CHANNEL  WIFI_AP_CHANNEL     // ESP-NOW rides the SoftAP channel
#define ESPNOW_CHUNK    1024                // per-packet cap; ESP-NOW v2 allows up to 1470
#define PC_STREAM_BUF   8192                // mobile -> WebSocket backlog

static const uint8_t BCAST_MAC[ESP_NOW_ETH_ALEN] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t PAIR_MAGIC[4] = {'E','N','P','R'};

static volatile bool        s_paired;
static uint8_t              s_peer[ESP_NOW_ETH_ALEN];
static volatile bool        s_peer_pending;
static uint8_t              s_pending[ESP_NOW_ETH_ALEN];
static StreamBufferHandle_t s_to_pc;    // mobile -> WebSocket bytes

static bool mac_is_bcast(const uint8_t *m)
{
    return memcmp(m, BCAST_MAC, ESP_NOW_ETH_ALEN) == 0;
}

// Kept minimal: no esp_now_*/httpd calls here (they run in tasks / the server).
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (mac_is_bcast(info->des_addr)) {
        // Pairing beacon from a peer looking for us.
        if (!s_paired && !s_peer_pending &&
            len == (int)sizeof(PAIR_MAGIC) &&
            memcmp(data, PAIR_MAGIC, sizeof(PAIR_MAGIC)) == 0) {
            memcpy(s_pending, info->src_addr, ESP_NOW_ETH_ALEN);
            s_peer_pending = true;
        }
        return;
    }
    // Unicast to us == data from the locked peer -> forward to the WebSocket.
    if (s_paired && len > 0) {
        xStreamBufferSend(s_to_pc, data, len, 0);
    }
}

static void on_sent(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    (void)info; (void)status;   // best-effort bridge; nothing to do on ack/fail
}

static void espnow_start(void)
{
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));     // ESP-NOW needs the radio always on

    ESP_ERROR_CHECK(esp_now_init());
    uint32_t ver = 0;
    esp_now_get_version(&ver);
    ESP_LOGI(TAG, "ESP-NOW version = %u (expect 2)", (unsigned)ver);

    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_sent));

    // ESP-NOW runs on the AP interface so it shares the SoftAP's channel.
    esp_now_peer_info_t peer = {
        .channel = ESPNOW_CHANNEL,
        .ifidx   = WIFI_IF_AP,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, BCAST_MAC, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    ESP_LOGI(TAG, "station AP MAC %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// WebSocket client -> mobile: forward each inbound payload over ESP-NOW, sliced to
// the per-packet cap. Runs in the HTTP server task.
static void ws_rx_forward(const uint8_t *data, size_t len)
{
    if (!s_paired) return;
    for (size_t off = 0; off < len; off += ESPNOW_CHUNK) {
        size_t n = len - off;
        if (n > ESPNOW_CHUNK) n = ESPNOW_CHUNK;
        esp_now_send(s_peer, data + off, n);
    }
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
                .ifidx   = WIFI_IF_AP,
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

// mobile -> WebSocket: drain the stream buffer that on_recv fills and broadcast it.
static void radio_to_ws_task(void *arg)
{
    (void)arg;
    uint8_t *buf = malloc(ESPNOW_CHUNK);
    if (!buf) {
        ESP_LOGE(TAG, "no memory for radio_to_ws buffer");
        vTaskDelete(NULL);
    }
    for (;;) {
        size_t n = xStreamBufferReceive(s_to_pc, buf, ESPNOW_CHUNK, portMAX_DELAY);
        if (n > 0) {
            ws_broadcast_bin(buf, n);
        }
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    s_to_pc = xStreamBufferCreate(PC_STREAM_BUF, 1);

    wifi_init_softap();     // AP + netif + event loop, on WIFI_AP_CHANNEL
    espnow_start();         // ESP-NOW on the AP interface, same channel
    start_webserver(ws_rx_forward);

    xTaskCreate(pair_task,         "pair",  3072, NULL, 5, NULL);
    xTaskCreate(radio_to_ws_task,  "rx2ws", 3072, NULL, 8, NULL);

    ESP_LOGI(TAG, "bridge up: WebSocket <-> ESP-NOW ch %d", ESPNOW_CHANNEL);
}
