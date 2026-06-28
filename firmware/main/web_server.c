#include "web_server.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/param.h>          /* MIN() */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"         /* esp_restart() */
#include "esp_ota_ops.h"        /* OTA: write firmware to the spare slot */
#include "esp_http_server.h"

#include "telemetry.h"
#include "motors.h"
#include "encoders.h"

static const char *TAG = "web_server";

static httpd_handle_t   s_httpd;               /* server handle, for WS broadcast */
static volatile bool    s_stream_enabled = true;  /* toggle telemetry streaming */

bool web_server_streaming(void) { return s_stream_enabled; }

/* The web page lives in www/index.html and is embedded into the firmware by the
 * build (EMBED_TXTFILES in main/CMakeLists.txt). The linker exposes it through
 * these symbols; EMBED_TXTFILES null-terminates the data so we can treat the
 * start pointer as a C string. */
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html_start, HTTPD_RESP_USE_STRLEN);
}

/* Kept so `curl http://192.168.4.1/api/telemetry` still works for quick checks. */
static esp_err_t telemetry_get_handler(httpd_req_t *req)
{
    char json[512];
    int n = telemetry_to_json(json, sizeof(json), "telemetry");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, n);
}

/* Send one text frame to a specific client (used for command replies). */
static void ws_send_text(httpd_req_t *req, const char *text)
{
    httpd_ws_frame_t frame = {
        .final = true, .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)text, .len = strlen(text),
    };
    httpd_ws_send_frame(req, &frame);
}

/* Reply to a command with a {"type":"resp","text":"..."} frame. */
static void ws_reply(httpd_req_t *req, const char *text)
{
    char buf[256];
    snprintf(buf, sizeof(buf), "{\"type\":\"resp\",\"text\":\"%s\"}", text);
    ws_send_text(req, buf);
}

/* Interpret a text command from the terminal. Add new commands here as the
 * project grows (later: set gains, change PWM, start/stop, etc.). */
static void handle_command(httpd_req_t *req, const char *cmd)
{
    if (strcmp(cmd, "help") == 0) {
        ws_reply(req, "commands: help | stats | motor <l|r|both> <-100..100> | "
                      "stop | enc reset | stream on | stream off | rollback");
    } else if (strcmp(cmd, "enc reset") == 0) {
        encoder_reset(0);
        encoder_reset(1);
        ws_reply(req, "encoder counts reset to 0");
    } else if (strncmp(cmd, "motor ", 6) == 0) {
        /* Open-loop test: "motor l 50", "motor r -30", "motor both 40". */
        char which[8];
        int pct;
        if (sscanf(cmd + 6, "%7s %d", which, &pct) != 2) {
            ws_reply(req, "usage: motor <l|r|both> <-100..100>");
            return;
        }
        if (pct >  100) pct =  100;
        if (pct < -100) pct = -100;
        float v = pct / 100.0f;
        if (strcmp(which, "l") == 0 || strcmp(which, "left") == 0) {
            motor_cmd_set(0, v);
        } else if (strcmp(which, "r") == 0 || strcmp(which, "right") == 0) {
            motor_cmd_set(1, v);
        } else if (strcmp(which, "both") == 0 || strcmp(which, "b") == 0) {
            motor_cmd_set(0, v);
            motor_cmd_set(1, v);
        } else {
            ws_reply(req, "motor: pick l | r | both");
            return;
        }
        char msg[64];
        snprintf(msg, sizeof(msg), "motor %s = %d%%", which, pct);
        ws_reply(req, msg);
    } else if (strcmp(cmd, "stop") == 0) {
        motor_cmd_set(0, 0.0f);
        motor_cmd_set(1, 0.0f);
        ws_reply(req, "motors stopped");
    } else if (strcmp(cmd, "rollback") == 0) {
        /* Switch boot to the other OTA slot (the image we ran before the last
         * upload) and reboot. esp_ota_set_boot_partition validates the slot, so
         * if it's empty (never flashed) we fail gracefully instead of bricking. */
        const esp_partition_t *running = esp_ota_get_running_partition();
        const esp_partition_t *other   = esp_ota_get_next_update_partition(NULL);
        if (other == NULL || other == running) {
            ws_reply(req, "rollback: no other OTA slot available");
            return;
        }
        esp_err_t err = esp_ota_set_boot_partition(other);
        if (err != ESP_OK) {
            char msg[160];
            snprintf(msg, sizeof(msg),
                     "rollback failed: %s (slot '%s' may be empty)",
                     esp_err_to_name(err), other->label);
            ws_reply(req, msg);
            return;
        }
        char msg[160];
        snprintf(msg, sizeof(msg), "rolling back '%s' -> '%s', rebooting...",
                 running->label, other->label);
        ws_reply(req, msg);
        ESP_LOGW(TAG, "%s", msg);
        vTaskDelay(pdMS_TO_TICKS(500));   /* let the reply flush */
        esp_restart();
    } else if (strcmp(cmd, "stats") == 0) {
        char json[512];
        telemetry_to_json(json, sizeof(json), "resp_stats");
        ws_send_text(req, json);
    } else if (strcmp(cmd, "stream on") == 0) {
        s_stream_enabled = true;
        ws_reply(req, "streaming ON");
    } else if (strcmp(cmd, "stream off") == 0) {
        s_stream_enabled = false;
        ws_reply(req, "streaming OFF");
    } else if (cmd[0] == '\0') {
        /* empty line: ignore */
    } else {
        char msg[160];
        snprintf(msg, sizeof(msg), "unknown command: %s (try help)", cmd);
        ws_reply(req, msg);
    }
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    /* The initial GET is the WebSocket handshake - nothing to do. */
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket client connected");
        return ESP_OK;
    }

    /* First call with len=0 tells us the incoming frame size. */
    httpd_ws_frame_t frame = { .type = HTTPD_WS_TYPE_TEXT };
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) return err;
    if (frame.len == 0 || frame.len > 200) return ESP_OK;

    uint8_t buf[201] = { 0 };
    frame.payload = buf;
    err = httpd_ws_recv_frame(req, &frame, sizeof(buf) - 1);
    if (err != ESP_OK) return err;

    handle_command(req, (const char *)buf);
    return ESP_OK;
}

/* Runs in the HTTP server task (scheduled via httpd_queue_work). Sends the text
 * to every WS client. The actual socket send blocks here, NOT in reporter_task,
 * so a slow/laggy client can't stall telemetry or logging. A client whose send
 * fails is dropped so it reconnects fresh instead of wedging future sends.
 * @p arg is a malloc'd, NUL-terminated copy of the payload that we free here. */
static void ws_async_send(void *arg)
{
    char *text = (char *)arg;
    if (!s_httpd) { free(text); return; }

    size_t max = CONFIG_LWIP_MAX_SOCKETS;
    int fds[CONFIG_LWIP_MAX_SOCKETS];
    if (httpd_get_client_list(s_httpd, &max, fds) != ESP_OK) { free(text); return; }

    httpd_ws_frame_t frame = {
        .final = true, .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)text, .len = strlen(text),
    };
    for (size_t i = 0; i < max; i++) {
        if (httpd_ws_get_fd_info(s_httpd, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) {
            continue;
        }
        if (httpd_ws_send_frame_async(s_httpd, fds[i], &frame) != ESP_OK) {
            httpd_sess_trigger_close(s_httpd, fds[i]);   /* drop the wedged client */
        }
    }
    free(text);
}

void ws_broadcast(const char *text)
{
    if (!s_httpd) return;
    char *copy = strdup(text);
    if (!copy) return;
    if (httpd_queue_work(s_httpd, ws_async_send, copy) != ESP_OK) {
        free(copy);   /* couldn't queue - drop this frame rather than leak */
    }
}

/* ===================== OTA: wireless firmware upload ===================== */

/* Browser POSTs the raw .bin to /ota. We stream it straight into the *other*
 * app slot (esp_ota_get_next_update_partition), validate it, then flip the boot
 * partition and reboot. The currently running image is never touched, so a bad
 * upload can't brick the board - on failure we just don't switch slots. */
#define OTA_RECV_BUF 1024

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (target == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA slot");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OTA: receiving %d bytes -> slot '%s' (0x%06" PRIx32 ")",
             req->content_len, target->label, (uint32_t)target->address);

    esp_ota_handle_t ota = 0;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &ota);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_begin failed");
        return ESP_FAIL;
    }

    char *buf = malloc(OTA_RECV_BUF);
    if (buf == NULL) {
        esp_ota_abort(ota);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, MIN(remaining, OTA_RECV_BUF));
        if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;   /* slow link: keep waiting */
        if (got <= 0) {
            esp_ota_abort(ota);
            free(buf);
            ESP_LOGE(TAG, "OTA: recv error (%d), aborting", got);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv error");
            return ESP_FAIL;
        }
        err = esp_ota_write(ota, buf, got);
        if (err != ESP_OK) {
            esp_ota_abort(ota);
            free(buf);
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_write failed");
            return ESP_FAIL;
        }
        remaining -= got;
    }
    free(buf);

    /* esp_ota_end verifies the image (magic byte, checksum, signature if on). */
    err = esp_ota_end(ota);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid image");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set_boot failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA OK - next boot from '%s', rebooting...", target->label);
    httpd_resp_sendstr(req, "OTA OK - rebooting");

    /* Let the HTTP response flush before we pull the rug out. */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

void start_webserver(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* OTA writes flash from inside the server task; give it a bigger stack. */
    cfg.stack_size = 8192;
    /* Fail a stuck send fast (default ~5 s) so the server task recovers quickly
     * if a client's link goes bad; and reclaim the oldest socket when full. */
    cfg.send_wait_timeout = 2;
    cfg.lru_purge_enable  = true;
    ESP_ERROR_CHECK(httpd_start(&s_httpd, &cfg));

    httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
    httpd_register_uri_handler(s_httpd, &root);

    httpd_uri_t api = { .uri = "/api/telemetry", .method = HTTP_GET,
                        .handler = telemetry_get_handler };
    httpd_register_uri_handler(s_httpd, &api);

    httpd_uri_t ws = { .uri = "/ws", .method = HTTP_GET, .handler = ws_handler,
                       .is_websocket = true };
    httpd_register_uri_handler(s_httpd, &ws);

    httpd_uri_t ota = { .uri = "/ota", .method = HTTP_POST, .handler = ota_post_handler };
    httpd_register_uri_handler(s_httpd, &ota);

    ESP_LOGI(TAG, "HTTP + WebSocket + OTA server up");
}
