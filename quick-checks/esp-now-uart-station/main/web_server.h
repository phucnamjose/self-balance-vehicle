/**
 * @file web_server.h
 * @brief Minimal HTTP + WebSocket server used as the PC-facing side of the bridge.
 *
 *   /    : a tiny landing page pointing at the WebSocket endpoint
 *   /ws  : full-duplex byte channel. Frames from the PC are handed to the RX
 *          callback (forwarded to the mobile over ESP-NOW); bytes from the mobile
 *          are broadcast to every connected client as binary frames.
 *
 * Adapted from the robot firmware's web_server.c (OTA/telemetry/command handling
 * stripped out); the async-broadcast plumbing is kept verbatim.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/* Invoked for each payload received from a WebSocket client (PC -> station).
 * Runs in the HTTP server task. */
typedef void (*ws_rx_cb_t)(const uint8_t *data, size_t len);

/* Start the HTTP server and register the / and /ws URIs. @p on_rx receives every
 * inbound WebSocket payload (may be NULL to ignore inbound data). */
void start_webserver(ws_rx_cb_t on_rx);

/* Broadcast a binary frame (@p len bytes) to all connected WebSocket clients.
 * Non-blocking: the socket send happens in the server task, not the caller's. */
void ws_broadcast_bin(const void *data, size_t len);
