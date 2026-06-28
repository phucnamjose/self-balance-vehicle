/**
 * @file app_main.c
 * @brief Phase 4 - Open-loop motors (XY-160D) + wheel encoders (PCNT).
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
 * Motors (XY-160D): L EN=25 IN=26,27   R EN=33 IN=32,14.
 * Encoders (quadrature A/B): L A=18 B=19   R A=23 B=13 (PCNT, internal pull-ups).
 * LEDs/notify: Left=GPIO17, Right=GPIO16.
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
#include "driver/pulse_cnt.h"   /* PCNT: hardware quadrature encoder counting */
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

#define LED_L_GPIO     GPIO_NUM_17                /* Left LED  - notification blink */
#define LED_R_GPIO     GPIO_NUM_16                /* Right LED - notification blink */

#define CONTROL_HZ     200                       /* control loop rate */
#define TIMER_RES_HZ   1000000                   /* 1 MHz -> 1 tick = 1 us */
#define ALARM_COUNT    (TIMER_RES_HZ / CONTROL_HZ)

/* LEDC speed mode shared by the motor PWM channels (ESP32 low-speed group). */
#define PWM_MODE       LEDC_LOW_SPEED_MODE

/* --- XY-160D dual H-bridge motor driver (see docs/hardware/xy-160d-motor-driver.md) ---
 * Each motor: ENx = PWM speed, IN1/IN2 = direction. Left = ch 1, Right = ch 2. */
#define MOTOR_L_PWM        GPIO_NUM_25            /* -> ENA */
#define MOTOR_L_IN1        GPIO_NUM_26            /* -> IN1 */
#define MOTOR_L_IN2        GPIO_NUM_27            /* -> IN2 */
#define MOTOR_R_PWM        GPIO_NUM_33            /* -> ENB */
#define MOTOR_R_IN1        GPIO_NUM_32            /* -> IN3 */
#define MOTOR_R_IN2        GPIO_NUM_14            /* -> IN4 */

#define MOTOR_PWM_TIMER    LEDC_TIMER_1           /* dedicated LEDC timer for motor PWM */
#define MOTOR_L_CHANNEL    LEDC_CHANNEL_1
#define MOTOR_R_CHANNEL    LEDC_CHANNEL_2
#define MOTOR_PWM_RES_BITS LEDC_TIMER_10_BIT      /* duty 0..1023 */
#define MOTOR_PWM_DUTY_MAX ((1 << 10) - 1)
#define MOTOR_PWM_FREQ_HZ  10000                  /* XY-160D ceiling is 10 kHz; lower if it whines/heats */

/* --- Quadrature encoders (one per wheel) via the PCNT hardware counter ---
 * Pins are non-strapping and pull-up capable (free on WROOM & WROVER). Encoder
 * A/B feed a PCNT unit configured for 4x quadrature decoding. */
#define ENC_L_A            GPIO_NUM_18
#define ENC_L_B            GPIO_NUM_19
#define ENC_R_A            GPIO_NUM_23
#define ENC_R_B            GPIO_NUM_13
#define ENC_PCNT_HIGH      30000                  /* rollover limits: count auto-resets here */
#define ENC_PCNT_LOW       (-30000)               /* and the watch callback accumulates it */
#define ENC_GLITCH_NS      1000                   /* ignore pulses shorter than 1 us */

static TaskHandle_t s_control_task;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_mpu;       /* MPU6050 device on the I2C bus */
static bool s_mpu_ok;                       /* did the IMU init succeed? */

/* One motor's wiring. */
typedef struct {
    gpio_num_t     pwm_gpio;   /* ENx */
    gpio_num_t     in1, in2;   /* direction pins */
    ledc_channel_t channel;    /* LEDC channel driving ENx */
} motor_t;

static const motor_t s_motors[2] = {
    { MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_CHANNEL },   /* 0 = left  */
    { MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, MOTOR_R_CHANNEL },   /* 1 = right */
};

/* Open-loop command per motor, -1.0..+1.0 (sign = direction). Set from the web
 * terminal, applied by control_task each tick. */
static volatile float s_motor_cmd[2];

/* One encoder. PCNT counts in hardware (16-bit); on each rollover the watch
 * callback adds the limit to accum, giving a full 64-bit position. */
typedef struct {
    pcnt_unit_handle_t unit;
    volatile int64_t   accum;
} encoder_t;

static encoder_t s_enc[2];

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
    float    mL, mR;       /* commanded motor effort, -1..+1 */
    int64_t  posL, posR;   /* total encoder counts */
    int32_t  velL, velR;   /* encoder counts/sec over the last window */
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

/* ===================== Notification LED (non-blocking) ===================== */

/* A tiny task owns the Left/Right LEDs (GPIO17/GPIO16), driven together. Callers
 * post a blink pattern to a
 * 1-slot mailbox and return immediately - no delays in the caller's context.
 * A new request always preempts the one in progress. */
typedef struct {
    uint16_t on_ms;     /* on duration per blink */
    uint16_t off_ms;    /* off duration per blink */
    int16_t  count;     /* number of blinks; <0 = repeat forever; 0 = off */
} led_pattern_t;

static QueueHandle_t s_led_q;

static inline void led_set(bool on) {
    gpio_set_level(LED_L_GPIO, on ? 1 : 0);
    gpio_set_level(LED_R_GPIO, on ? 1 : 0);
}

/* Non-blocking. Examples:
 *   led_blink(60, 60, 3)     -> three quick blinks, then off
 *   led_blink(60, 1940, -1)  -> slow heartbeat forever
 *   led_blink(1, 0, -1)      -> solid on
 *   led_blink(0, 0, 0)       -> off                                            */
static void led_blink(uint16_t on_ms, uint16_t off_ms, int16_t count)
{
    led_pattern_t p = { on_ms, off_ms, count };
    if (s_led_q) xQueueOverwrite(s_led_q, &p);
}

static inline void led_off(void)       { led_blink(0, 0, 0); }
static inline void led_on(void)        { led_blink(1, 0, -1); }
static inline void led_heartbeat(void) { led_blink(60, 1940, -1); }

static void led_task(void *arg)
{
    led_pattern_t p = { 0, 0, 0 };
    int  remaining = 0;          /* blinks left; -1 = forever; 0 = idle */
    bool on = false;
    TickType_t next = 0;         /* tick when the current phase ends */

    for (;;) {
        TickType_t now  = xTaskGetTickCount();
        TickType_t wait = (remaining == 0) ? portMAX_DELAY
                                           : (next > now ? next - now : 0);

        led_pattern_t np;
        if (xQueueReceive(s_led_q, &np, wait) == pdTRUE) {
            p = np;
            if (p.count == 0) {                       /* off */
                remaining = 0; led_set(false);
            } else if (p.off_ms == 0 && p.count < 0) { /* solid on */
                remaining = 0; led_set(true);
            } else {                                   /* start blinking */
                remaining = (p.count < 0) ? -1 : p.count;
                on = true; led_set(true);
                next = xTaskGetTickCount() + pdMS_TO_TICKS(p.on_ms);
            }
            continue;
        }

        if (remaining == 0) continue;   /* idle: nothing to do */

        if (on) {                       /* end of an ON phase -> go OFF */
            on = false; led_set(false);
            next = xTaskGetTickCount() + pdMS_TO_TICKS(p.off_ms);
        } else {                        /* completed one full blink */
            if (remaining > 0 && --remaining == 0) {
                led_set(false);         /* finished the requested count */
                continue;
            }
            on = true; led_set(true);
            next = xTaskGetTickCount() + pdMS_TO_TICKS(p.on_ms);
        }
    }
}

static void led_init(void)
{
    gpio_config_t io = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL << LED_L_GPIO) | (1ULL << LED_R_GPIO)),
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    led_set(false);

    s_led_q = xQueueCreate(1, sizeof(led_pattern_t));
    configASSERT(s_led_q);
    xTaskCreatePinnedToCore(led_task, "led", 2048, NULL, 2, NULL, 0);
}

/* ===================== XY-160D motors ===================== */

/* Apply a signed command to one motor: sign -> direction pins, magnitude -> PWM
 * duty on ENx. cmd 0 sets both IN pins low (brake/stop). */
static void motor_set(int i, float cmd)
{
    if (cmd >  1.0f) cmd =  1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    const motor_t *m = &s_motors[i];

    if (cmd > 0.0f) {                 /* forward */
        gpio_set_level(m->in1, 1);
        gpio_set_level(m->in2, 0);
    } else if (cmd < 0.0f) {          /* reverse */
        gpio_set_level(m->in1, 0);
        gpio_set_level(m->in2, 1);
    } else {                          /* stop (brake) */
        gpio_set_level(m->in1, 0);
        gpio_set_level(m->in2, 0);
    }

    float mag = (cmd < 0.0f) ? -cmd : cmd;
    uint32_t duty = (uint32_t)(mag * MOTOR_PWM_DUTY_MAX + 0.5f);
    ledc_set_duty(PWM_MODE, m->channel, duty);
    ledc_update_duty(PWM_MODE, m->channel);
}

static void motor_init(void)
{
    /* Direction pins as outputs, start low (stopped). */
    gpio_config_t io = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << MOTOR_L_IN1) | (1ULL << MOTOR_L_IN2) |
                        (1ULL << MOTOR_R_IN1) | (1ULL << MOTOR_R_IN2),
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    /* One LEDC timer shared by both ENx channels (separate from the LED timer). */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = PWM_MODE,
        .timer_num       = MOTOR_PWM_TIMER,
        .duty_resolution = MOTOR_PWM_RES_BITS,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = s_motors[i].pwm_gpio,
            .speed_mode = PWM_MODE,
            .channel    = s_motors[i].channel,
            .timer_sel  = MOTOR_PWM_TIMER,
            .duty       = 0,
            .hpoint     = 0,
            .intr_type  = LEDC_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
        motor_set(i, 0.0f);    /* ensure stopped */
    }
    ESP_LOGI(TAG, "motors ready (XY-160D, PWM %d Hz) - L:EN=%d/IN=%d,%d  R:EN=%d/IN=%d,%d",
             MOTOR_PWM_FREQ_HZ, MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2,
             MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2);
}

/* ===================== Quadrature encoders (PCNT) ===================== */

/* Rollover callback (ISR): the count just hit +/-limit and auto-reset to 0, so
 * fold that limit into the 64-bit accumulator. */
static bool IRAM_ATTR enc_on_reach(pcnt_unit_handle_t unit,
                                   const pcnt_watch_event_data_t *edata, void *user)
{
    encoder_t *e = (encoder_t *)user;
    e->accum += edata->watch_point_value;
    return false;
}

/* Configure one PCNT unit for 4x quadrature decoding of an A/B pair. */
static void encoder_init_one(encoder_t *e, gpio_num_t a, gpio_num_t b)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = ENC_PCNT_HIGH,
        .low_limit  = ENC_PCNT_LOW,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &e->unit));

    pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = ENC_GLITCH_NS };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(e->unit, &fcfg));

    /* Channel A: count edges of A, direction gated by B's level. */
    pcnt_chan_config_t ca = { .edge_gpio_num = a, .level_gpio_num = b };
    pcnt_channel_handle_t cha;
    ESP_ERROR_CHECK(pcnt_new_channel(e->unit, &ca, &cha));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(cha,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(cha,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* Channel B: count edges of B, direction gated by A's level (gives 4x). */
    pcnt_chan_config_t cb = { .edge_gpio_num = b, .level_gpio_num = a };
    pcnt_channel_handle_t chb;
    ESP_ERROR_CHECK(pcnt_new_channel(e->unit, &cb, &chb));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chb,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chb,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* Watch the rollover limits so accum tracks full revolutions. */
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(e->unit, ENC_PCNT_HIGH));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(e->unit, ENC_PCNT_LOW));
    pcnt_event_callbacks_t cbs = { .on_reach = enc_on_reach };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(e->unit, &cbs, e));

    /* Encoder outputs are often open-collector; enable internal pull-ups. */
    gpio_set_pull_mode(a, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(b, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(pcnt_unit_enable(e->unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(e->unit));
    ESP_ERROR_CHECK(pcnt_unit_start(e->unit));
}

static void encoder_init(void)
{
    encoder_init_one(&s_enc[0], ENC_L_A, ENC_L_B);
    encoder_init_one(&s_enc[1], ENC_R_A, ENC_R_B);
    ESP_LOGI(TAG, "encoders ready (PCNT) - L:A=%d/B=%d  R:A=%d/B=%d",
             ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B);
}

/* Total position = accumulator + current hardware count. Retry if a rollover
 * lands between the two reads so we never mix an old accum with a new count. */
static int64_t encoder_count(int i)
{
    int64_t a0, a1;
    int cur = 0;
    do {
        a0 = s_enc[i].accum;
        pcnt_unit_get_count(s_enc[i].unit, &cur);
        a1 = s_enc[i].accum;
    } while (a0 != a1);
    return a1 + cur;
}

static void encoder_reset(int i)
{
    s_enc[i].accum = 0;
    pcnt_unit_clear_count(s_enc[i].unit);
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
        "\"dt_min\":%lld,\"dt_max\":%lld,\"mL\":%.2f,\"mR\":%.2f,"
        "\"posL\":%lld,\"posR\":%lld,\"velL\":%ld,\"velR\":%ld,"
        "\"imu_ok\":%d,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"temp\":%.1f}",
        type, CONTROL_HZ, snap.ticks, (long long)snap.dt_avg_us,
        (long long)snap.dt_min_us, (long long)snap.dt_max_us,
        snap.mL, snap.mR,
        (long long)snap.posL, (long long)snap.posR,
        (long)snap.velL, (long)snap.velR,
        snap.imu.ok ? 1 : 0,
        snap.imu.ax, snap.imu.ay, snap.imu.az,
        snap.imu.gx, snap.imu.gy, snap.imu.gz, snap.imu.temp_c);
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
            s_motor_cmd[0] = v;
        } else if (strcmp(which, "r") == 0 || strcmp(which, "right") == 0) {
            s_motor_cmd[1] = v;
        } else if (strcmp(which, "both") == 0 || strcmp(which, "b") == 0) {
            s_motor_cmd[0] = s_motor_cmd[1] = v;
        } else {
            ws_reply(req, "motor: pick l | r | both");
            return;
        }
        char msg[64];
        snprintf(msg, sizeof(msg), "motor %s = %d%%", which, pct);
        ws_reply(req, msg);
    } else if (strcmp(cmd, "stop") == 0) {
        s_motor_cmd[0] = s_motor_cmd[1] = 0.0f;
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

/* Hand a telemetry frame to the server task and return immediately. Non-blocking:
 * reporter_task is never held up by a slow Wi-Fi client. */
static void ws_broadcast(const char *text)
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

static void start_webserver(void)
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
    imu_t imu = { 0 };             /* latest IMU sample */
    int64_t pos_last[2] = { 0 };   /* encoder counts at the previous report */

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t now_us = esp_timer_get_time();
        int64_t dt = now_us - last_us;
        last_us = now_us;

        /* Read the IMU every tick. The I2C transfer (~0.4 ms) blocks this task
         * only, and we watch dt_min/dt_max to confirm it doesn't hurt jitter.
         * (later: feed imu into the orientation estimate + PID -> "level") */
        imu = mpu6050_read();

        /* Apply the latest open-loop motor commands (set from the web terminal).
         * (later: the PID output will drive these instead of a manual command.) */
        motor_set(0, s_motor_cmd[0]);
        motor_set(1, s_motor_cmd[1]);

        /* Track timing quality. */
        if (dt < dt_min) dt_min = dt;
        if (dt > dt_max) dt_max = dt;
        dt_sum += dt;
        ticks++;

        /* Once per second, hand a snapshot to the reporter task and reset the
         * window. xQueueOverwrite never blocks, so the control loop stays lean -
         * the slow logging/networking happens on the other core. */
        if (ticks % CONTROL_HZ == 0) {
            /* The window is exactly 1 s, so delta counts == counts/sec. */
            int64_t posL = encoder_count(0);
            int64_t posR = encoder_count(1);
            telemetry_t snap = {
                .ticks     = ticks,
                .dt_avg_us = dt_sum / CONTROL_HZ,
                .dt_min_us = dt_min,
                .dt_max_us = dt_max,
                .mL        = s_motor_cmd[0],
                .mR        = s_motor_cmd[1],
                .posL      = posL,
                .posR      = posR,
                .velL      = (int32_t)(posL - pos_last[0]),
                .velR      = (int32_t)(posR - pos_last[1]),
                .imu       = imu,
            };
            pos_last[0] = posL;
            pos_last[1] = posR;
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
                     "[core %d] rate=%d Hz dt avg=%lld us (min=%lld max=%lld) mot=[%.0f %.0f]%% ticks=%" PRIu32,
                     xPortGetCoreID(), CONTROL_HZ,
                     (long long)snap.dt_avg_us, (long long)snap.dt_min_us,
                     (long long)snap.dt_max_us, snap.mL * 100.0f, snap.mR * 100.0f, snap.ticks);

            ESP_LOGI(TAG,
                     "         enc posL=%lld posR=%lld  vel=[%ld %ld] cnt/s",
                     (long long)snap.posL, (long long)snap.posR,
                     (long)snap.velL, (long)snap.velR);

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
                char json[512];
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

    led_init();     /* Left/Right LEDs for notifications */
    motor_init();   /* motors start stopped */
    encoder_init(); /* PCNT quadrature counters, count from 0 */

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

    /* Default "alive" notification - replace/extend with your own led_blink()
     * calls (e.g. error codes, Wi-Fi client connected, OTA in progress). */
    led_heartbeat();
}
