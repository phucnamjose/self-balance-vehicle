/**
 * @file app_main.c
 * @brief Phase 3 / Step 1 - Read the MPU6050 IMU (raw accel/gyro over I2C).
 *
 * Architecture:
 *
 *     Core 1 (APP_CPU):  control_task  - hard real-time loop, no slow work
 *           GPTimer ISR --notify--> control_task --> read MPU6050 --> LEDC PWM
 *           control_task --(1-slot queue)--> telemetry snapshot (incl. IMU)
 *
 *     Core 0 (PRO_CPU):  Wi-Fi + HTTP/WebSocket server + reporter_task
 *           reporter_task caches the latest snapshot and, if streaming is on,
 *           broadcasts it to connected WebSocket clients
 *           /ws  : full-duplex channel - server streams telemetry JSON, browser
 *                  sends text commands (help/stats/stream on|off)
 *           /ota : POST a new .bin; it is written to the spare app slot and the
 *                  board reboots into it - no USB cable needed to reflash.
 *
 * The robot hosts its own Wi-Fi access point. Connect your PC/phone to it and
 * open http://192.168.4.1/ for a live log + command terminal - no USB cable.
 *
 * OTA needs a two-slot partition table (ota_0 / ota_1, see partitions.csv): the
 * new image lands in the inactive slot, so a bad upload can never overwrite the
 * running firmware.
 *
 * Why WebSocket (not UDP): a browser cannot send raw UDP. WebSocket is native,
 * full-duplex, and rides the same HTTP server, so it fits an in-page terminal.
 *
 * What this teaches:
 *   - Serving a WebSocket endpoint with esp_http_server.
 *   - Receiving command frames and replying; broadcasting to all clients.
 *   - Keeping the control loop untouched while comms evolve on core 0.
 *
 * Wiring the MPU6050 (GY-521 module has its own pull-ups):
 *   VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22, AD0 -> GND (addr 0x68).
 *
 * Pins: PWM output on GPIO2 (onboard LED on most DevKits; change if needed).
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/param.h>          /* MIN() */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"         /* esp_restart() */
#include "esp_ota_ops.h"        /* OTA: write firmware to the spare slot */
#include "nvs_flash.h"
#include "esp_http_server.h"

static const char *TAG = "balance_bot";

/* --- Wi-Fi access point (connect to this, then browse to 192.168.4.1) --- */
#define WIFI_AP_SSID     "balance-bot"
#define WIFI_AP_PASS     "balance123"   /* must be >= 8 chars; "" = open network */
#define WIFI_AP_CHANNEL  1
#define WIFI_AP_MAX_CONN 4

/* --- I2C bus (MPU6050 lives here) --- */
#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_GPIO   GPIO_NUM_21
#define I2C_SCL_GPIO   GPIO_NUM_22
#define I2C_FREQ_HZ    400000

/* --- MPU6050 registers (datasheet "Register Map") --- */
#define MPU6050_ADDR        0x68    /* AD0 = GND; 0x69 if AD0 = VCC */
#define MPU_REG_SMPLRT_DIV  0x19
#define MPU_REG_CONFIG      0x1A    /* DLPF setting */
#define MPU_REG_GYRO_CFG    0x1B    /* gyro full-scale range */
#define MPU_REG_ACCEL_CFG   0x1C    /* accel full-scale range */
#define MPU_REG_ACCEL_XOUT  0x3B    /* first of 14 data bytes (accel/temp/gyro) */
#define MPU_REG_PWR_MGMT_1  0x6B
#define MPU_REG_WHO_AM_I    0x75    /* returns 0x68 on a genuine MPU6050 */

/* Sensitivity for the default ranges we configure below (datasheet sec 6.1-6.2):
 *   accel +/-2 g    -> 16384 LSB per g
 *   gyro  +/-250 dps -> 131.0 LSB per deg/s
 * Temperature: degC = raw/340 + 36.53. */
#define MPU_ACCEL_LSB_PER_G    16384.0f
#define MPU_GYRO_LSB_PER_DPS   131.0f

/* Per-read I2C timeout. A normal read is ~0.4 ms; this is ~10x that but still
 * under one 5 ms control tick, so a bus glitch costs ~1 missed tick instead of
 * stalling the control loop for 100 ms. */
#define MPU_I2C_TIMEOUT_MS     4

#define PWM_GPIO       GPIO_NUM_2

#define CONTROL_HZ     200                       /* control loop rate */
#define TIMER_RES_HZ   1000000                   /* 1 MHz -> 1 tick = 1 us */
#define ALARM_COUNT    (TIMER_RES_HZ / CONTROL_HZ)

/* --- LEDC (PWM) configuration --- */
#define PWM_MODE       LEDC_LOW_SPEED_MODE
#define PWM_TIMER      LEDC_TIMER_0
#define PWM_CHANNEL    LEDC_CHANNEL_0
#define PWM_RES_BITS   LEDC_TIMER_10_BIT         /* duty range 0..1023 */
#define PWM_DUTY_MAX   ((1 << 10) - 1)
#define PWM_FREQ_HZ    5000                      /* 5 kHz: flicker-free for an LED.
                                                  * For real motors we'll use ~20 kHz. */

/* Triangle-wave sweep: full fade up+down over this many control ticks. */
#define FADE_PERIOD_TICKS  (CONTROL_HZ * 2)      /* ~2 s up, then handled by mirror */

static TaskHandle_t s_control_task;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_mpu;       /* MPU6050 device on the I2C bus */
static bool s_mpu_ok;                       /* did the IMU init succeed? */

/* One IMU sample in physical units. */
typedef struct {
    bool  ok;              /* false if the last read failed */
    float ax, ay, az;      /* acceleration in g */
    float gx, gy, gz;      /* angular rate in deg/s */
    float temp_c;          /* die temperature in degC */
} imu_t;

/* Snapshot of loop state, produced by control_task and consumed by reporter_task. */
typedef struct {
    uint32_t ticks;        /* total control ticks so far */
    int64_t  dt_avg_us;    /* average loop period over the last window */
    int64_t  dt_min_us;    /* min/max loop period (jitter) over the window */
    int64_t  dt_max_us;
    float    duty;         /* last PWM level [0..1] */
    imu_t    imu;          /* latest IMU sample */
} telemetry_t;

/* 1-slot queue used as a mailbox: control_task overwrites it with the latest
 * snapshot; reporter_task blocks until a new one arrives. */
static QueueHandle_t s_telemetry_q;

/* Latest snapshot cached for the HTTP server, guarded by a mutex because the
 * web server task and reporter_task touch it from different contexts. */
static telemetry_t      s_latest;
static SemaphoreHandle_t s_latest_lock;

static httpd_handle_t   s_httpd;               /* server handle, for WS broadcast */
static volatile bool    s_stream_enabled = true;  /* toggle telemetry streaming */

/* ISR context: keep it minimal - just wake the control task for the next tick. */
static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    BaseType_t high_prio_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task, &high_prio_woken);
    return high_prio_woken == pdTRUE;
}

static void pwm_init(void)
{
    /* 1) The LEDC timer sets the PWM frequency and duty resolution. */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_RES_BITS,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    /* 2) The LEDC channel binds a GPIO to that timer and holds the duty value. */
    ledc_channel_config_t ch_cfg = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
}

/* Set PWM duty from a normalised level in [0.0, 1.0]. This is the interface the
 * controller will use later (level = commanded motor effort). */
static void pwm_set_level(float level)
{
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;
    uint32_t duty = (uint32_t)(level * PWM_DUTY_MAX + 0.5f);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

static void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_PORT,
        .sda_io_num        = I2C_SDA_GPIO,
        .scl_io_num        = I2C_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* helps if the module lacks pull-ups */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));
    ESP_LOGI(TAG, "I2C master on SDA=%d SCL=%d @ %d Hz",
             I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ);
}

/* Probe every 7-bit address and print which ones acknowledge. A device that is
 * present pulls the bus low to ACK its address, so i2c_master_probe() succeeds. */
static void i2c_scan(void)
{
    ESP_LOGI(TAG, "scanning I2C bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        esp_err_t err = i2c_master_probe(s_i2c_bus, addr, 50 /* ms */);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  found device at 0x%02X%s", addr,
                     (addr == 0x68 || addr == 0x69) ? "  <- looks like MPU6050" : "");
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "  no devices found - check wiring, power and pull-ups");
    } else {
        ESP_LOGI(TAG, "scan complete: %d device(s)", found);
    }
}

/* ===================== MPU6050 (6-axis IMU over I2C) ===================== */

/* Write one byte to an MPU register: send [reg, value] in a single transaction. */
static esp_err_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_mpu, buf, sizeof(buf), MPU_I2C_TIMEOUT_MS);
}

/* Read @p n bytes starting at @p reg: write the register pointer, then read.
 * i2c_master_transmit_receive does the write+repeated-start+read in one call. */
static esp_err_t mpu_read_regs(uint8_t reg, uint8_t *dst, size_t n)
{
    return i2c_master_transmit_receive(s_mpu, &reg, 1, dst, n, MPU_I2C_TIMEOUT_MS);
}

/* Attach the MPU6050 to the bus and wake it into a known configuration:
 * default ranges (+/-2 g, +/-250 dps), ~1 kHz sampling, DLPF ~44 Hz to tame
 * noise. Returns true on success and sets s_mpu_ok. */
static bool mpu6050_init(void)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_mpu) != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050: failed to add device at 0x%02X", MPU6050_ADDR);
        return false;
    }

    uint8_t who = 0;
    if (mpu_read_regs(MPU_REG_WHO_AM_I, &who, 1) != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050: no response (check wiring/power)");
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 WHO_AM_I = 0x%02X (expect 0x68)", who);

    /* PWR_MGMT_1 = 0 clears the SLEEP bit and selects the internal 8 MHz clock,
     * so the sensor starts converting. */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    mpu_write_reg(MPU_REG_SMPLRT_DIV, 0x00);   /* sample rate = 1 kHz / (1+0) */
    mpu_write_reg(MPU_REG_CONFIG,     0x03);   /* DLPF ~44 Hz accel / ~42 Hz gyro */
    mpu_write_reg(MPU_REG_GYRO_CFG,   0x00);   /* +/-250 dps */
    mpu_write_reg(MPU_REG_ACCEL_CFG,  0x00);   /* +/-2 g */

    s_mpu_ok = true;
    ESP_LOGI(TAG, "MPU6050 ready (+/-2 g, +/-250 dps, DLPF on)");
    return true;
}

/* Read the 14-byte accel/temp/gyro block and convert to physical units. Each
 * axis is a big-endian signed 16-bit value. Layout from MPU_REG_ACCEL_XOUT:
 *   [0..5] accel X,Y,Z   [6..7] temp   [8..13] gyro X,Y,Z */
static imu_t mpu6050_read(void)
{
    imu_t s = { 0 };
    if (!s_mpu_ok) return s;

    uint8_t b[14];
    if (mpu_read_regs(MPU_REG_ACCEL_XOUT, b, sizeof(b)) != ESP_OK) return s;

    int16_t ax = (int16_t)((b[0]  << 8) | b[1]);
    int16_t ay = (int16_t)((b[2]  << 8) | b[3]);
    int16_t az = (int16_t)((b[4]  << 8) | b[5]);
    int16_t t  = (int16_t)((b[6]  << 8) | b[7]);
    int16_t gx = (int16_t)((b[8]  << 8) | b[9]);
    int16_t gy = (int16_t)((b[10] << 8) | b[11]);
    int16_t gz = (int16_t)((b[12] << 8) | b[13]);

    s.ax = ax / MPU_ACCEL_LSB_PER_G;
    s.ay = ay / MPU_ACCEL_LSB_PER_G;
    s.az = az / MPU_ACCEL_LSB_PER_G;
    s.gx = gx / MPU_GYRO_LSB_PER_DPS;
    s.gy = gy / MPU_GYRO_LSB_PER_DPS;
    s.gz = gz / MPU_GYRO_LSB_PER_DPS;
    s.temp_c = t / 340.0f + 36.53f;
    s.ok = true;
    return s;
}

/* ===================== Wi-Fi (SoftAP) + HTTP server ===================== */

static void wifi_init_softap(void)
{
    /* Wi-Fi needs NVS (stores calibration data), a network interface and the
     * default event loop. */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    wifi_config_t ap = {
        .ap = {
            .ssid           = WIFI_AP_SSID,
            .ssid_len       = strlen(WIFI_AP_SSID),
            .channel        = WIFI_AP_CHANNEL,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };
    if (strlen(WIFI_AP_PASS) == 0) {
        ap.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        strncpy((char *)ap.ap.password, WIFI_AP_PASS, sizeof(ap.ap.password) - 1);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP '%s' started - connect, then open http://192.168.4.1/",
             WIFI_AP_SSID);
}

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

/* Serialize the latest snapshot to JSON. @p type tags the message so the client
 * can tell telemetry from command replies. Returns the string length. */
static int telemetry_to_json(char *buf, size_t len, const char *type)
{
    telemetry_t snap;
    xSemaphoreTake(s_latest_lock, portMAX_DELAY);
    snap = s_latest;
    xSemaphoreGive(s_latest_lock);

    return snprintf(buf, len,
        "{\"type\":\"%s\",\"rate\":%d,\"ticks\":%" PRIu32 ",\"dt_avg\":%lld,"
        "\"dt_min\":%lld,\"dt_max\":%lld,\"duty\":%.3f,"
        "\"imu_ok\":%d,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"temp\":%.1f}",
        type, CONTROL_HZ, snap.ticks, (long long)snap.dt_avg_us,
        (long long)snap.dt_min_us, (long long)snap.dt_max_us, snap.duty,
        snap.imu.ok ? 1 : 0,
        snap.imu.ax, snap.imu.ay, snap.imu.az,
        snap.imu.gx, snap.imu.gy, snap.imu.gz, snap.imu.temp_c);
}

/* Kept so `curl http://192.168.4.1/api/telemetry` still works for quick checks. */
static esp_err_t telemetry_get_handler(httpd_req_t *req)
{
    char json[384];
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
        ws_reply(req, "commands: help | stats | stream on | stream off | rollback");
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
        char json[384];
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

/* Broadcast a text frame to every connected WebSocket client. Called from
 * reporter_task on core 0 to stream telemetry. */
static void ws_broadcast(const char *text)
{
    if (!s_httpd) return;

    size_t max = CONFIG_LWIP_MAX_SOCKETS;
    int fds[CONFIG_LWIP_MAX_SOCKETS];
    if (httpd_get_client_list(s_httpd, &max, fds) != ESP_OK) return;

    for (size_t i = 0; i < max; i++) {
        if (httpd_ws_get_fd_info(s_httpd, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) {
            continue;
        }
        httpd_ws_frame_t frame = {
            .final = true, .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)text, .len = strlen(text),
        };
        httpd_ws_send_frame_async(s_httpd, fds[i], &frame);
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

static void start_webserver(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* OTA writes flash from inside the server task; give it a bigger stack. */
    cfg.stack_size = 8192;
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

static void start_control_timer(void)
{
    gptimer_handle_t timer = NULL;
    gptimer_config_t cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RES_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &timer));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_timer_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer));

    gptimer_alarm_config_t alarm = {
        .reload_count = 0,
        .alarm_count  = ALARM_COUNT,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm));
    ESP_ERROR_CHECK(gptimer_start(timer));
}

/* High-priority task woken by the timer ISR every 1/CONTROL_HZ seconds. */
static void control_task(void *arg)
{
    int64_t last_us = esp_timer_get_time();
    int64_t dt_min = INT64_MAX, dt_max = 0, dt_sum = 0;
    uint32_t ticks = 0;
    uint32_t phase = 0;            /* 0..2*FADE_PERIOD_TICKS-1 for up/down sweep */
    imu_t imu = { 0 };             /* latest IMU sample */

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t now_us = esp_timer_get_time();
        int64_t dt = now_us - last_us;
        last_us = now_us;

        /* Read the IMU every tick. The I2C transfer (~0.4 ms) blocks this task
         * only, and we watch dt_min/dt_max to confirm it doesn't hurt jitter.
         * (later: feed imu into the orientation estimate + PID -> "level") */
        imu = mpu6050_read();

        /* Triangle wave: ramp 0->1 over FADE_PERIOD_TICKS, then 1->0. */
        float level;
        if (phase < FADE_PERIOD_TICKS) {
            level = (float)phase / FADE_PERIOD_TICKS;
        } else {
            level = 1.0f - (float)(phase - FADE_PERIOD_TICKS) / FADE_PERIOD_TICKS;
        }
        if (++phase >= 2 * FADE_PERIOD_TICKS) phase = 0;

        pwm_set_level(level);

        /* Track timing quality. */
        if (dt < dt_min) dt_min = dt;
        if (dt > dt_max) dt_max = dt;
        dt_sum += dt;
        ticks++;

        /* Once per second, hand a snapshot to the reporter task and reset the
         * window. xQueueOverwrite never blocks, so the control loop stays lean -
         * the slow logging/networking happens on the other core. */
        if (ticks % CONTROL_HZ == 0) {
            telemetry_t snap = {
                .ticks     = ticks,
                .dt_avg_us = dt_sum / CONTROL_HZ,
                .dt_min_us = dt_min,
                .dt_max_us = dt_max,
                .duty      = level,
                .imu       = imu,
            };
            xQueueOverwrite(s_telemetry_q, &snap);
            dt_min = INT64_MAX; dt_max = 0; dt_sum = 0;
        }
    }
}

/* Runs on core 0. Blocks until the control task posts a snapshot, caches it for
 * the HTTP server, and also logs it to the serial console. */
static void reporter_task(void *arg)
{
    telemetry_t snap;
    for (;;) {
        if (xQueueReceive(s_telemetry_q, &snap, portMAX_DELAY) == pdTRUE) {
            /* Cache for the web server. */
            xSemaphoreTake(s_latest_lock, portMAX_DELAY);
            s_latest = snap;
            xSemaphoreGive(s_latest_lock);

            ESP_LOGI(TAG,
                     "[core %d] rate=%d Hz dt avg=%lld us (min=%lld max=%lld) duty=%.0f%% ticks=%" PRIu32,
                     xPortGetCoreID(), CONTROL_HZ,
                     (long long)snap.dt_avg_us, (long long)snap.dt_min_us,
                     (long long)snap.dt_max_us, snap.duty * 100.0f, snap.ticks);

            if (snap.imu.ok) {
                ESP_LOGI(TAG,
                         "         IMU a=[%+.2f %+.2f %+.2f]g g=[%+.1f %+.1f %+.1f]dps %.1fC",
                         snap.imu.ax, snap.imu.ay, snap.imu.az,
                         snap.imu.gx, snap.imu.gy, snap.imu.gz, snap.imu.temp_c);
            } else {
                ESP_LOGW(TAG, "         IMU read failed");
            }

            /* Stream to any connected WebSocket terminals. */
            if (s_stream_enabled) {
                char json[384];
                telemetry_to_json(json, sizeof(json), "telemetry");
                ws_broadcast(json);
            }
        }
    }
}

void app_main(void)
{
    printf("\n=== Self-Balancing Vehicle - Wi-Fi log + OTA ===\n");
    ESP_LOGI(TAG, "core 1 = control loop, core 0 = Wi-Fi + HTTP + reporter");

    /* Show which OTA slot we booted from - flips after each successful upload. */
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "running from partition '%s' (0x%06" PRIx32 ", %" PRIu32 " KB)",
             running->label, (uint32_t)running->address, running->size / 1024);

    /* NVS is required by the Wi-Fi driver. Erase + retry if the partition is
     * from an older layout. */
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    pwm_init();

    /* Bring up I2C and scan once at boot so we can confirm the IMU is wired. */
    i2c_init();
    i2c_scan();

    /* Attach + wake the MPU6050. If it's missing we keep running (the control
     * loop just reports imu_ok=false) so the rest of the firmware still works. */
    if (!mpu6050_init()) {
        ESP_LOGW(TAG, "continuing without IMU - check wiring at SDA=%d SCL=%d",
                 I2C_SDA_GPIO, I2C_SCL_GPIO);
    }

    /* Mailbox between the two tasks (length 1 -> always the latest snapshot). */
    s_telemetry_q = xQueueCreate(1, sizeof(telemetry_t));
    configASSERT(s_telemetry_q);
    s_latest_lock = xSemaphoreCreateMutex();
    configASSERT(s_latest_lock);

    /* Wi-Fi access point + web server (both live on core 0). */
    wifi_init_softap();
    start_webserver();

    /* Reporter on core 0 (PRO_CPU) - shares this core with Wi-Fi. */
    xTaskCreatePinnedToCore(reporter_task, "reporter", 4096, NULL, 4, NULL, 0);

    /* Control loop on core 1 (APP_CPU), high priority. Create it before the
     * timer so the ISR has a valid task handle to notify. */
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL,
                            configMAX_PRIORITIES - 2, &s_control_task, 1);

    start_control_timer();
}
