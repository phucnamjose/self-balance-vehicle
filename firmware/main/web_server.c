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
#include "wheel_pi.h"          /* wheel-speed controller: setpoint + gains */
#include "balance_pid.h"       /* balance (tilt) controller: gains + trim */
#include "estimator.h"         /* tilt estimator: alpha get/set */
#include "control.h"           /* control_mode(): gate OTA on STOP_CONTROL */

static const char *TAG = "web_server";

static httpd_handle_t   s_httpd;               /* server handle, for WS broadcast */
static volatile bool    s_stream_enabled = true;  /* toggle telemetry streaming */

bool web_server_streaming(void) { return s_stream_enabled; }

/* The control UI is a separate web app on the operator's PC (different origin),
 * so allow cross-origin reads/uploads on the HTTP endpoints. (/ws is CORS-exempt.) */
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

/* Reply to a command with a {"type":"resp","text":"..."} frame. Sized to fit the
 * longest reply (the 'help' listing) without truncation. */
static void ws_reply(httpd_req_t *req, const char *text)
{
    char buf[512];
    snprintf(buf, sizeof(buf), "{\"type\":\"resp\",\"text\":\"%s\"}", text);
    ws_send_text(req, buf);
}

/* Report the current experiment feature flags to the terminal. */
static void reply_experiment(httpd_req_t *req)
{
    char msg[112];
    snprintf(msg, sizeof(msg), "experiment: estimation %s, controller %s, balance %s",
             control_estimation_enabled() ? "ON" : "OFF",
             control_controller_enabled() ? "ON" : "OFF",
             control_balance_enabled()    ? "ON" : "OFF");
    ws_reply(req, msg);
}

/* Interpret a text command from the terminal. Add new commands here. */
static void handle_command(httpd_req_t *req, const char *cmd)
{
    if (strcmp(cmd, "help") == 0) {
        /* Newlines are the two-char JSON escape (\\n) so the reply stays valid JSON. */
        static const char help[] =
            "Balance Bot terminal.\\n"
            "Bring-up: control start -> exp motor-ctrl -> speed <rad/s>, then "
            "watch the wheels track. Flashing needs control stop first.\\n"
            "\\n"
            "help                          - this guide\\n"
            "stats                         - loop timing + current modes\\n"
            "motor <l|r|both> <-100..100>  - open-loop duty %, needs TEST_MOTORS\\n"
            "stop                          - zero motor outputs (task keeps running)\\n"
            "speed <l|r|both> <rad/s>      - wheel-speed setpoint (controller on)\\n"
            "gains <l|r|both> <kp> <ki>    - set/report PI gains (gains default resets)\\n"
            "dbcomp on|off                 - controller deadband compensation\\n"
            "ff on|off                     - controller feedforward (u=w_set/K)\\n"
            "control start|stop            - start/stop the control task\\n"
            "exp motors|motor-ctrl|angles|balance - experiment preset\\n"
            "est on|off                    - angle estimation\\n"
            "alpha <0..1>                  - tilt estimator filter weight\\n"
            "ctrl on|off                   - wheel-speed controller\\n"
            "balance on|off                - self-balance loop (also enables est+ctrl)\\n"
            "bgains <kp> <ki> <kd>         - set/report balance PID (bgains default resets)\\n"
            "btrim <rad>                   - balance upright trim (theta_ref)\\n"
            "play | play stop              - run/stop uploaded motor script\\n"
            "deadband | deadband stop      - run/stop deadband sweep\\n"
            "gyrocal                       - average gyro at rest, store zero-rate bias\\n"
            "enc reset                     - zero encoder counts\\n"
            "stream on|off                 - telemetry streaming\\n"
            "rollback                      - boot the previous OTA slot";
        char buf[1800];
        snprintf(buf, sizeof(buf), "{\"type\":\"resp\",\"text\":\"%s\"}", help);
        ws_send_text(req, buf);
    } else if (strcmp(cmd, "exp motors") == 0) {
        control_set_experiment(TEST_MOTORS);
        reply_experiment(req);
    } else if (strcmp(cmd, "exp motor-ctrl") == 0) {
        control_set_experiment(TEST_MOTOR_CONTROLLERS);
        reply_experiment(req);
    } else if (strcmp(cmd, "exp angles") == 0) {
        control_set_experiment(TEST_ANGLES_ESTIMATION);
        reply_experiment(req);
    } else if (strcmp(cmd, "exp balance") == 0) {
        control_set_experiment(TEST_BALANCE);
        reply_experiment(req);
    } else if (strcmp(cmd, "exp") == 0) {
        reply_experiment(req);
    } else if (strcmp(cmd, "est on") == 0) {
        control_set_estimation(true);
        reply_experiment(req);
    } else if (strcmp(cmd, "est off") == 0) {
        control_set_estimation(false);
        reply_experiment(req);
    } else if (strncmp(cmd, "alpha", 5) == 0) {
        /* Estimator filter weight: "alpha 0.98" sets, "alpha" reports. Higher trusts
         * the gyro more. tau = alpha*dt/(1-alpha). */
        float a;
        if (sscanf(cmd + 5, "%f", &a) == 1) {
            estimator_set_alpha(a);
        }
        float cur = estimator_alpha();
        float dt  = 1.0f / CONTROL_HZ;
        float tau = (cur < 1.0f) ? cur * dt / (1.0f - cur) : 0.0f;
        char msg[128];
        snprintf(msg, sizeof(msg), "estimator alpha=%.4f (tau=%.3f s @ %d Hz)",
                 (double)cur, (double)tau, CONTROL_HZ);
        ws_reply(req, msg);
    } else if (strcmp(cmd, "ctrl on") == 0) {
        control_set_controller(true);
        reply_experiment(req);
    } else if (strcmp(cmd, "ctrl off") == 0) {
        control_set_controller(false);
        reply_experiment(req);
    } else if (strcmp(cmd, "balance on") == 0) {
        /* The balance loop needs the estimator + inner wheel loop, so turn both on. */
        control_set_estimation(true);
        control_set_controller(true);
        control_set_balance(true);
        reply_experiment(req);
    } else if (strcmp(cmd, "balance off") == 0) {
        control_set_balance(false);
        wheel_pi_set_setpoint(0, 0.0f);   /* drop the setpoint the balancer owned */
        wheel_pi_set_setpoint(1, 0.0f);
        reply_experiment(req);
    } else if (strcmp(cmd, "balance") == 0) {
        reply_experiment(req);
    } else if (strncmp(cmd, "bgains", 6) == 0) {
        /* Live balance PID: "bgains <kp> <ki> <kd>", "bgains default", or none to report. */
        char which[8] = { 0 };
        float kp, ki, kd;
        int n = sscanf(cmd + 6, "%7s", which);
        if (n == 1 && (strcmp(which, "default") == 0 || strcmp(which, "reset") == 0)) {
            balance_pid_reset_gains();
        } else if (sscanf(cmd + 6, "%f %f %f", &kp, &ki, &kd) == 3) {
            balance_pid_set_gains(kp, ki, kd);
        }
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "balance PID: Kp=%.2f Ki=%.2f Kd=%.2f (theta_ref=%.3f rad)",
                 (double)balance_pid_kp(), (double)balance_pid_ki(),
                 (double)balance_pid_kd(), (double)balance_pid_setpoint());
        ws_reply(req, msg);
    } else if (strncmp(cmd, "btrim", 5) == 0) {
        /* Upright trim: the tilt setpoint the balancer holds (rad). "btrim" reports. */
        float t;
        if (sscanf(cmd + 5, "%f", &t) == 1) {
            balance_pid_set_setpoint(t);
        }
        char msg[80];
        snprintf(msg, sizeof(msg), "balance trim theta_ref = %.4f rad",
                 (double)balance_pid_setpoint());
        ws_reply(req, msg);
    } else if (strcmp(cmd, "deadband stop") == 0) {
        control_deadband_stop();
        ws_reply(req, "deadband sweep stopped, motors parked");
    } else if (strcmp(cmd, "deadband") == 0) {
        /* Slow ramp to find each wheel's start-moving duty; needs TEST_MOTORS. */
        if (control_mode() != START_CONTROL ||
            control_estimation_enabled() || control_controller_enabled()) {
            ws_reply(req, "deadband needs START_CONTROL + TEST_MOTORS "
                          "(send 'control start' and 'exp motors')");
            return;
        }
        control_deadband_start();
        ws_reply(req, "deadband sweep started: ramping 0->15% forward then reverse, "
                      "result follows in a few seconds...");
    } else if (strcmp(cmd, "gyrocal") == 0) {
        /* Average the gyro at rest and store the zero-rate bias. Needs the control
         * task running and the robot held still (~2 s, any orientation). */
        if (control_mode() != START_CONTROL) {
            ws_reply(req, "gyrocal needs the control task running (send 'control start')");
            return;
        }
        control_gyrocal_start();
        ws_reply(req, "gyro calibration started: hold the robot still (~2 s), "
                      "result follows...");
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
    } else if (strncmp(cmd, "speed", 5) == 0) {
        /* Per-wheel speed setpoint (rad/s), e.g. "speed both 8". Only acts with the
         * controller enabled. */
        char which[8];
        float w;
        if (sscanf(cmd + 5, "%7s %f", which, &w) != 2) {
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "usage: speed <l|r|both> <rad/s> (now L=%.3f R=%.3f)",
                     (double)wheel_pi_setpoint(0), (double)wheel_pi_setpoint(1));
            ws_reply(req, msg);
            return;
        }
        if (strcmp(which, "l") == 0 || strcmp(which, "left") == 0) {
            wheel_pi_set_setpoint(0, w);
        } else if (strcmp(which, "r") == 0 || strcmp(which, "right") == 0) {
            wheel_pi_set_setpoint(1, w);
        } else if (strcmp(which, "both") == 0 || strcmp(which, "b") == 0) {
            wheel_pi_set_setpoint(0, w);
            wheel_pi_set_setpoint(1, w);
        } else {
            ws_reply(req, "speed: pick l | r | both");
            return;
        }
        char msg[128];
        snprintf(msg, sizeof(msg), "speed setpoint: L=%.3f R=%.3f rad/s%s",
                 (double)wheel_pi_setpoint(0), (double)wheel_pi_setpoint(1),
                 control_controller_enabled() ? "" :
                     " (controller OFF - send 'ctrl on' or 'exp motor-ctrl')");
        ws_reply(req, msg);
    } else if (strncmp(cmd, "gains", 5) == 0) {
        /* Live PI gains: "gains <l|r|both> <kp> <ki>", "gains default", or none to report. */
        char which[8];
        float kp, ki;
        int n = sscanf(cmd + 5, "%7s %f %f", which, &kp, &ki);
        if (n >= 1 && (strcmp(which, "default") == 0 || strcmp(which, "reset") == 0)) {
            wheel_pi_reset_gains();
        } else if (n == 3) {
            if (strcmp(which, "l") == 0 || strcmp(which, "left") == 0) {
                wheel_pi_set_gains(0, kp, ki);
            } else if (strcmp(which, "r") == 0 || strcmp(which, "right") == 0) {
                wheel_pi_set_gains(1, kp, ki);
            } else if (strcmp(which, "both") == 0 || strcmp(which, "b") == 0) {
                wheel_pi_set_gains(0, kp, ki);
                wheel_pi_set_gains(1, kp, ki);
            } else {
                ws_reply(req, "gains: pick l | r | both");
                return;
            }
        }
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "PI gains: L Kp=%.4f Ki=%.4f  R Kp=%.4f Ki=%.4f",
                 (double)wheel_pi_kp(0), (double)wheel_pi_ki(0),
                 (double)wheel_pi_kp(1), (double)wheel_pi_ki(1));
        ws_reply(req, msg);
    } else if (strcmp(cmd, "dbcomp on") == 0 || strcmp(cmd, "dbcomp off") == 0) {
        bool on = (cmd[7] == 'o' && cmd[8] == 'n');
        wheel_pi_set_deadband(on);
        ws_reply(req, on ? "deadband compensation ON" : "deadband compensation OFF");
    } else if (strcmp(cmd, "dbcomp") == 0) {
        ws_reply(req, wheel_pi_deadband() ? "deadband compensation is ON"
                                          : "deadband compensation is OFF");
    } else if (strcmp(cmd, "ff on") == 0 || strcmp(cmd, "ff off") == 0) {
        bool on = (cmd[3] == 'o' && cmd[4] == 'n');
        wheel_pi_set_ff(on);
        ws_reply(req, on ? "feedforward ON" : "feedforward OFF");
    } else if (strcmp(cmd, "ff") == 0) {
        ws_reply(req, wheel_pi_ff() ? "feedforward is ON" : "feedforward is OFF");
    } else if (strcmp(cmd, "stop") == 0) {
        /* Motor-test stop: zero the outputs (task keeps running, not STOP_CONTROL).
         * Also park the wheel setpoint so a re-enabled controller starts from rest. */
        motor_cmd_set(0, 0.0f);
        motor_cmd_set(1, 0.0f);
        wheel_pi_set_setpoint(0, 0.0f);
        wheel_pi_set_setpoint(1, 0.0f);
        ws_reply(req, "motor test stopped (outputs 0); control task still running");
    } else if (strcmp(cmd, "control stop") == 0) {
        control_set_mode(STOP_CONTROL);
        ws_reply(req, "STOP_CONTROL: control task stopped, motors off (flashing allowed)");
    } else if (strcmp(cmd, "control start") == 0) {
        control_set_mode(START_CONTROL);
        ws_reply(req, "START_CONTROL: control task running");
    } else if (strcmp(cmd, "rollback") == 0) {
        /* Boot the other OTA slot and reboot. set_boot_partition validates it, so an
         * empty slot fails gracefully instead of bricking. */
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
        /* Splice the current modes in before the closing brace. */
        if (n > 0 && n < (int)sizeof(json) && json[n - 1] == '}') {
            snprintf(json + n - 1, sizeof(json) - (n - 1),
                     ",\"cmode\":\"%s\",\"est\":%d,\"mctrl\":%d,\"bal\":%d,"
                     "\"play\":%d,\"play_pos\":%d,\"play_len\":%d}",
                     control_mode() == START_CONTROL ? "START_CONTROL" : "STOP_CONTROL",
                     control_estimation_enabled() ? 1 : 0,
                     control_controller_enabled() ? 1 : 0,
                     control_balance_enabled() ? 1 : 0,
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

/* One queued WebSocket message: length + type, with the payload bytes inline
 * (text and binary share one path). Allocated by ws_enqueue, freed by ws_async_send. */
typedef struct {
    size_t                len;
    httpd_ws_type_t       type;
    /* payload bytes follow immediately after this header */
} ws_msg_t;

/* Runs in the HTTP server task (via httpd_queue_work): sends the message to every
 * WS client. The socket send blocks here, not in reporter_task, so a slow client
 * can't stall telemetry; a failed send drops that client. Frees @p arg. */
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

/* Browser POSTs the raw .bin to /ota. We stream it into the other app slot,
 * validate, then flip the boot partition and reboot. The running image is never
 * touched, so a bad upload can't brick the board. */
#define OTA_RECV_BUF 1024

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    /* Safety gate: only flash in STOP_CONTROL (task stopped, motors parked). The
     * refusal is a normal CORS response (not send_err) so the cross-origin app can
     * read the 403 status and message. */
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

/* Upper bound on the uploaded sequence text; refuse larger so a bad request can't
 * exhaust the heap. */
#define SEQ_MAX_BODY  (96 * 1024)

/* POST /motorseq - import + play a step-based open-loop motor script. Body is plain
 * text, one step per line "dur,mL,mR" (dur > 0 s, mL,mR in -1..+1); blank/# lines are
 * ignored. Only in START_CONTROL + TEST_MOTORS; the loop plays each step, then parks. */
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
    /* Fail a stuck send fast so the server recovers if a client's link goes bad;
     * reclaim the oldest socket when full. */
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
