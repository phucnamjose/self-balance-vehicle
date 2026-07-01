/**
 * @file web_server.h
 * @brief HTTP + WebSocket server, command terminal and wireless OTA.
 *
 *   /     : a tiny landing page; the full control UI runs as a separate web app
 *           on the operator's PC (see ../../webapp) and connects over /ws
 *   /ws   : full-duplex channel - server streams telemetry JSON, browser sends
 *           text commands (help/stats/motor/stop/enc reset/stream/rollback)
 *   /api/telemetry : latest snapshot as JSON (CORS-enabled) for quick checks
 *   /ota  : POST a new .bin; written to the spare app slot, then reboot.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

/* Start the HTTP server and register the /, /api/telemetry, /ws and /ota URIs. */
void start_webserver(void);

/* Broadcast a text frame to all connected WebSocket clients. Non-blocking:
 * the actual socket send happens in the server task, not the caller's. */
void ws_broadcast(const char *text);

/* Broadcast a binary frame (@p len bytes) to all connected WebSocket clients.
 * Used for packed telemetry batches. Non-blocking, same as ws_broadcast. */
void ws_broadcast_bin(const void *data, size_t len);

/* Whether telemetry streaming to WebSocket clients is currently enabled. */
bool web_server_streaming(void);
