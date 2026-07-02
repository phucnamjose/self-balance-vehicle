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
#include "control.h"           /* control_mode(): gate OTA on STOP_CONTROL */

static const char *TAG = "web_server";

static httpd_handle_t   s_httpd;               /* server handle, for WS broadcast */
static volatile bool    s_stream_enabled = true;  /* toggle telemetry streaming */

bool web_server_streaming(void) { return s_stream_enabled; }

/* The control UI now runs as a separate web app on the operator's PC and
 * connects here over WebSocket, so it is served from a different origin. Allow
 * cross-origin reads/uploads on the HTTP endpoints. (WebSocket is exempt from
 * CORS, so /ws needs nothing.) */
static void set_cors(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
}

/* The full UI lives in ../../webapp and is hosted on the PC; this root page is
 * just a pointer to it for anyone who browses to the device directly. */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    static const char page[] =
        "<!doctype html><meta charset=utf-8><title>Balance Bot</title>"
        "<body style=\"font-family:system-ui;background:#0e1116;color:#e6edf3;padding:24px\">"
        "<h2>Balance Bot</h2>"
        "<p>This device exposes the telemetry API only. The control UI runs as a "
        "separate web app on your PC and connects here over WebSocket.</p>"
        "<ul>"
        "<li>WebSocket: <code>ws://192.168.4.1/ws</code></li>"
        "<li>Telemetry JSON: <code>/api/telemetry</code></li>"
        "<li>OTA upload: <code>POST /ota</code></li>"
        "</ul>"
        "<p>Open the web app and connect to this device's IP (default 192.168.4.1).</p>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

/* Kept so `curl http://192.168.4.1/api/telemetry` still works for quick checks. */
static esp_err_t telemetry_get_handler(httpd_req_t *req)
{
    char json[512];
    int n = telemetry_latest_json(json, sizeof(json), "telemetry");
    httpd_resp_set_type(req, "application/json");
    set_cors(req);
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

/* Report the current experiment feature flags to the terminal. */
static void reply_experiment(httpd_req_t *req)
{
    char msg[80];
    snprintf(msg, sizeof(msg), "experiment: estimation %s, controller %s",
             control_estimation_enabled() ? "ON" : "OFF",
             control_controller_enabled() ? "ON" : "OFF");
    ws_reply(req, msg);
}

/* Interpret a text command from the terminal. Add new commands here as the
 * project grows (later: set gains, change PWM, start/stop, etc.). */
static void handle_command(httpd_req_t *req, const char *cmd)
{
    if (strcmp(cmd, "help") == 0) {
        ws_reply(req, "commands: help | stats | motor <l|r|both> <-100..100> | stop | "
                      "control start | control stop | exp motors|motor-ctrl|angles | "
                      "est on|off | ctrl on|off | play | play stop | enc reset | "
                      "stream on | stream off | rollback | (flashing requires STOP_CONTROL)");
    } else if (strcmp(cmd, "exp motors") == 0) {
        control_set_experiment(TEST_MOTORS);
        reply_experiment(req);
    } else if (strcmp(cmd, "exp motor-ctrl") == 0) {
        control_set_experiment(TEST_MOTOR_CONTROLLERS);
        reply_experiment(req);
    } else if (strcmp(cmd, "exp angles") == 0) {
        control_set_experiment(TEST_ANGLES_ESTIMATION);
        reply_experiment(req);
    } else if (strcmp(cmd, "exp") == 0) {
        reply_experiment(req);
    } else if (strcmp(cmd, "est on") == 0) {
        control_set_estimation(true);
        reply_experiment(req);
    } else if (strcmp(cmd, "est off") == 0) {
        control_set_estimation(false);
        reply_experiment(req);
    } else if (strcmp(cmd, "ctrl on") == 0) {
        control_set_controller(true);
        reply_experiment(req);
    } else if (strcmp(cmd, "ctrl off") == 0) {
        control_set_controller(false);
        reply_experiment(req);
    } else if (strcmp(cmd, "play stop") == 0) {
        control_playback_stop();
        ws_reply(req, "playback stopped, motors parked");
    } else if (strcmp(cmd, "play") == 0) {
        control_playback_start();     /* no-op if nothing is loaded */
        char msg[80];
        snprintf(msg, sizeof(msg), "playback: step %d/%d, %s",
                 control_playback_pos(), control_playback_len(),
                 control_playback_active() ? "running" :
                     (control_playback_len() ? "idle" : "empty (POST /motorseq)"));
        ws_reply(req, msg);
    } else if (strcmp(cmd, "enc reset") == 0) {
        encoder_reset(0);
        encoder_reset(1);
        ws_reply(req, "encoder counts reset to 0");
    } else if (strncmp(cmd, "motor ", 6) == 0) {
        /* Open-loop test: "motor l 50", "motor r -30", "motor both 40". */
        if (control_mode() != START_CONTROL) {
            ws_reply(req, "control task stopped - send 'control start' first");
            return;
        }
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
        /* Motor-test stop: just zero the outputs. The control task keeps running
         * (this is NOT STOP_CONTROL); a new motor command re-enables movement. */
        motor_cmd_set(0, 0.0f);
        motor_cmd_set(1, 0.0f);
        ws_reply(req, "motor test stopped (outputs 0); control task still running");
    } else if (strcmp(cmd, "control stop") == 0) {
        control_set_mode(STOP_CONTROL);
        ws_reply(req, "STOP_CONTROL: control task stopped, motors off (flashing allowed)");
    } else if (strcmp(cmd, "control start") == 0) {
        control_set_mode(START_CONTROL);
        ws_reply(req, "START_CONTROL: control task running");
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
        int n = telemetry_latest_json(json, sizeof(json), "resp_stats");
        /* Splice the current modes in before the closing brace so a single
         * 'stats' shows both the loop numbers and what the loop is running. */
        if (n > 0 && n < (int)sizeof(json) && json[n - 1] == '}') {
            snprintf(json + n - 1, sizeof(json) - (n - 1),
                     ",\"cmode\":\"%s\",\"est\":%d,\"mctrl\":%d,"
                     "\"play\":%d,\"play_pos\":%d,\"play_len\":%d}",
                     control_mode() == START_CONTROL ? "START_CONTROL" : "STOP_CONTROL",
                     control_estimation_enabled() ? 1 : 0,
                     control_controller_enabled() ? 1 : 0,
                     control_playback_active() ? 1 : 0,
                     control_playback_pos(), control_playback_len());
        }
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

/* One queued WebSocket message: a length + frame type followed inline by the
 * payload bytes (so text and binary share the same path; binary may contain NUL
 * so we can't rely on strlen). Allocated by ws_enqueue, freed by ws_async_send. */
typedef struct {
    size_t                len;
    httpd_ws_type_t       type;
    /* payload bytes follow immediately after this header */
} ws_msg_t;

/* Runs in the HTTP server task (scheduled via httpd_queue_work). Sends the
 * message to every WS client. The actual socket send blocks here, NOT in
 * reporter_task, so a slow/laggy client can't stall telemetry or logging. A
 * client whose send fails is dropped so it reconnects fresh instead of wedging
 * future sends. @p arg is a malloc'd ws_msg_t (+payload) that we free here. */
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

void ws_broadcast(const char *text)
{
    ws_enqueue(text, strlen(text), HTTPD_WS_TYPE_TEXT);
}

void ws_broadcast_bin(const void *data, size_t len)
{
    ws_enqueue(data, len, HTTPD_WS_TYPE_BINARY);
}

/* ===================== OTA: wireless firmware upload ===================== */

/* Browser POSTs the raw .bin to /ota. We stream it straight into the *other*
 * app slot (esp_ota_get_next_update_partition), validate it, then flip the boot
 * partition and reboot. The currently running image is never touched, so a bad
 * upload can't brick the board - on failure we just don't switch slots. */
#define OTA_RECV_BUF 1024

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    /* Safety gate: only flash in STOP_CONTROL, where the control task is fully
     * stopped and the motors are parked. Send the refusal as a normal CORS
     * response (not httpd_resp_send_err) so the cross-origin web app can actually
     * read the 403 status and message. On a failed flash the task stays stopped;
     * the operator brings it back explicitly with 'control start'. */
    if (control_mode() != STOP_CONTROL) {
        ESP_LOGW(TAG, "OTA refused: control task is running");
        set_cors(req);
        httpd_resp_set_status(req, "403 Forbidden");
        httpd_resp_sendstr(req, "flashing blocked: stop the control task first (send 'control stop')");
        return ESP_OK;
    }

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
    set_cors(req);
    httpd_resp_sendstr(req, "OTA OK - rebooting");

    /* Let the HTTP response flush before we pull the rug out. */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/* CORS preflight for the OTA upload: the browser sends an OPTIONS request first
 * because the cross-origin POST carries a non-simple content type. */
static esp_err_t ota_options_handler(httpd_req_t *req)
{
    set_cors(req);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

/* Upper bound on the uploaded sequence text. 4000 ticks * ~24 chars/line leaves
 * generous room; refuse anything larger so a bad request can't exhaust the heap. */
#define SEQ_MAX_BODY  (96 * 1024)

/* POST /motorseq - import a step-based open-loop motor script and play it.
 *
 * Body is plain text, one step per line: "dur,mL,mR" (comma or whitespace
 * separated) where dur is how long to hold that output in seconds (> 0) and
 * mL,mR are in -1..+1. Blank lines and lines starting with '#' are ignored.
 * Only allowed in START_CONTROL + TEST_MOTORS (estimation and controller both
 * off); the loop then applies each step for its duration, then parks. */
static esp_err_t motorseq_post_handler(httpd_req_t *req)
{
    if (control_mode() != START_CONTROL ||
        control_estimation_enabled() || control_controller_enabled()) {
        set_cors(req);
        httpd_resp_set_status(req, "403 Forbidden");
        httpd_resp_sendstr(req, "motor sequence needs START_CONTROL + TEST_MOTORS "
                                "(send 'control start' and 'exp motors')");
        return ESP_OK;
    }

    int total = req->content_len;
    if (total <= 0 || total > SEQ_MAX_BODY) {
        set_cors(req);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "empty or too-large sequence file");
        return ESP_OK;
    }

    char *body = malloc(total + 1);
    if (body == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_FAIL;
    }
    int off = 0;
    while (off < total) {
        int got = httpd_req_recv(req, body + off, total - off);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (got <= 0) {
            free(body);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv error");
            return ESP_FAIL;
        }
        off += got;
    }
    body[total] = '\0';

    /* Commas -> spaces so a single "%f %f" scan handles CSV and whitespace. */
    for (int i = 0; i < total; i++) if (body[i] == ',') body[i] = ' ';

    control_playback_begin();
    int loaded = 0, bad = 0, truncated = 0;
    float total_s = 0.0f;
    char *save = NULL;
    for (char *ln = strtok_r(body, "\r\n", &save); ln; ln = strtok_r(NULL, "\r\n", &save)) {
        while (*ln == ' ' || *ln == '\t') ln++;
        if (*ln == '\0' || *ln == '#') continue;          /* blank / comment */
        float dur, mL, mR;
        if (sscanf(ln, "%f %f %f", &dur, &mL, &mR) != 3) { bad++; continue; }
        int r = control_playback_append(dur, mL, mR);
        if (r == -1) { truncated = 1; break; }             /* buffer full */
        if (r == -2) { bad++; continue; }                  /* non-positive duration */
        total_s += dur;
        loaded++;
    }
    free(body);

    char msg[160];
    if (loaded == 0) {
        control_playback_begin();
        snprintf(msg, sizeof(msg), "no valid rows (skipped %d bad line(s); "
                 "expected 'dur,mL,mR' with dur > 0)", bad);
        set_cors(req);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, msg);
        return ESP_OK;
    }

    control_playback_start();
    snprintf(msg, sizeof(msg),
             "loaded %d steps (%.2f s total), skipped %d bad; playing now%s",
             loaded, (double)total_s, bad,
             truncated ? " (truncated: max 20 steps)" : "");
    ESP_LOGI(TAG, "motorseq: %s", msg);
    set_cors(req);
    httpd_resp_sendstr(req, msg);
    return ESP_OK;
}

/* CORS preflight for /motorseq (cross-origin POST). */
static esp_err_t motorseq_options_handler(httpd_req_t *req)
{
    set_cors(req);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
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

    httpd_uri_t ota_opt = { .uri = "/ota", .method = HTTP_OPTIONS,
                            .handler = ota_options_handler };
    httpd_register_uri_handler(s_httpd, &ota_opt);

    httpd_uri_t seq = { .uri = "/motorseq", .method = HTTP_POST,
                        .handler = motorseq_post_handler };
    httpd_register_uri_handler(s_httpd, &seq);

    httpd_uri_t seq_opt = { .uri = "/motorseq", .method = HTTP_OPTIONS,
                            .handler = motorseq_options_handler };
    httpd_register_uri_handler(s_httpd, &seq_opt);

    ESP_LOGI(TAG, "HTTP + WebSocket + OTA server up");
}
