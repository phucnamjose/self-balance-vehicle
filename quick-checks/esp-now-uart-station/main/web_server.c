#include "web_server.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_http_server.h"

static const char *TAG = "web_server";

static httpd_handle_t s_httpd;      /* server handle, for WS broadcast */
static ws_rx_cb_t     s_on_rx;      /* PC -> station payload sink */

/* Landing page: the bridge only exposes a WebSocket, so just point at it. */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    static const char page[] =
        "<!doctype html><meta charset=utf-8><title>ESP-NOW bridge</title>"
        "<body style=\"font-family:system-ui;padding:24px\">"
        "<h2>ESP-NOW WebSocket bridge (station)</h2>"
        "<p>Connect a WebSocket to <code>ws://192.168.4.1/ws</code>. Bytes you send "
        "are forwarded to the mobile over ESP-NOW; bytes from the mobile arrive as "
        "binary frames.</p>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

/* Bidirectional byte channel. Inbound frames (text or binary) are handed to the
 * RX callback as raw bytes; the frame type is ignored. */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {          /* handshake */
        ESP_LOGI(TAG, "WebSocket client connected");
        return ESP_OK;
    }

    /* First call with len=0 reports the incoming frame size. */
    httpd_ws_frame_t frame = { 0 };
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) return err;
    if (frame.len == 0) return ESP_OK;          /* control/empty frame */
    if (frame.len > 8192) return ESP_FAIL;      /* sanity cap for a quick-check */

    uint8_t *buf = malloc(frame.len);
    if (!buf) return ESP_ERR_NO_MEM;
    frame.payload = buf;
    err = httpd_ws_recv_frame(req, &frame, frame.len);
    if (err == ESP_OK && s_on_rx) {
        s_on_rx(buf, frame.len);
    }
    free(buf);
    return err;
}

/* One queued WebSocket message: length + type, with the payload bytes inline.
 * Allocated by ws_enqueue, freed by ws_async_send. */
typedef struct {
    size_t          len;
    httpd_ws_type_t type;
} ws_msg_t;

/* Runs in the HTTP server task (via httpd_queue_work): sends the message to every
 * WS client. The socket send blocks here, not in the caller, so a slow client
 * can't stall the radio; a failed send drops that client. Frees @p arg. */
static void ws_async_send(void *arg)
{
    ws_msg_t *msg = (ws_msg_t *)arg;
    if (!s_httpd) { free(msg); return; }

    size_t max = CONFIG_LWIP_MAX_SOCKETS;
    int fds[CONFIG_LWIP_MAX_SOCKETS];
    if (httpd_get_client_list(s_httpd, &max, fds) != ESP_OK) { free(msg); return; }

    httpd_ws_frame_t frame = {
        .final = true, .type = msg->type,
        .payload = (uint8_t *)(msg + 1), .len = msg->len,
    };
    for (size_t i = 0; i < max; i++) {
        if (httpd_ws_get_fd_info(s_httpd, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) {
            continue;
        }
        if (httpd_ws_send_frame_async(s_httpd, fds[i], &frame) != ESP_OK) {
            httpd_sess_trigger_close(s_httpd, fds[i]);   /* drop the wedged client */
        }
    }
    free(msg);
}

/* Copy @p len bytes of @p data into a queued message of the given frame type. */
static void ws_enqueue(const void *data, size_t len, httpd_ws_type_t type)
{
    if (!s_httpd) return;
    ws_msg_t *msg = malloc(sizeof(*msg) + len);
    if (!msg) return;
    msg->len  = len;
    msg->type = type;
    memcpy(msg + 1, data, len);
    if (httpd_queue_work(s_httpd, ws_async_send, msg) != ESP_OK) {
        free(msg);   /* couldn't queue - drop this frame rather than leak */
    }
}

void ws_broadcast_bin(const void *data, size_t len)
{
    ws_enqueue(data, len, HTTPD_WS_TYPE_BINARY);
}

void start_webserver(ws_rx_cb_t on_rx)
{
    s_on_rx = on_rx;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* Fail a stuck send fast so the server recovers if a client's link goes bad;
     * reclaim the oldest socket when full. */
    cfg.send_wait_timeout = 2;
    cfg.lru_purge_enable  = true;
    ESP_ERROR_CHECK(httpd_start(&s_httpd, &cfg));

    httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
    httpd_register_uri_handler(s_httpd, &root);

    httpd_uri_t ws = { .uri = "/ws", .method = HTTP_GET, .handler = ws_handler,
                       .is_websocket = true };
    httpd_register_uri_handler(s_httpd, &ws);

    ESP_LOGI(TAG, "HTTP + WebSocket server up (ws://192.168.4.1/ws)");
}
