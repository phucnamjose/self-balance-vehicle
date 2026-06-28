/**
 * @file wifi_ap.h
 * @brief Wi-Fi SoftAP bring-up. The robot hosts its own access point.
 *
 * Connect your PC/phone to it, then open http://192.168.4.1/ for the web UI.
 */
#pragma once

/* Initialize NVS-backed Wi-Fi, netif, event loop and start the access point.
 * Call once at boot (after nvs_flash_init). */
void wifi_init_softap(void);
