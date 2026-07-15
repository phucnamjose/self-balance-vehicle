/**
 * @file wifi_ap.h
 * @brief Wi-Fi SoftAP bring-up for the ESP-NOW <-> WebSocket bridge.
 *
 * The station hosts its own access point on a fixed channel so ESP-NOW (which
 * must share the AP's channel) and the PC's WebSocket connection can coexist.
 * Connect your PC to it, then open ws://192.168.4.1/ws.
 */
#pragma once

/* Channel shared by the SoftAP and ESP-NOW. The mobile must use the same one. */
#define WIFI_AP_CHANNEL 1

/* Initialize netif + event loop + Wi-Fi and start the access point on
 * WIFI_AP_CHANNEL. Call once at boot, after nvs_flash_init and before esp_now_init. */
void wifi_init_softap(void);
