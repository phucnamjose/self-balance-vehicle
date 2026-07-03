#include "motors.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "motors";

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

/* Spin direction for a positive command. Flip to -1 if a wheel turns backwards
 * relative to its encoder, so +cmd always gives +encoder counts (forward). */
#define MOTOR_L_SIGN       (-1)
#define MOTOR_R_SIGN       (+1)

#define MOTOR_PWM_TIMER    LEDC_TIMER_1           /* dedicated LEDC timer for motor PWM */
#define MOTOR_L_CHANNEL    LEDC_CHANNEL_1
#define MOTOR_R_CHANNEL    LEDC_CHANNEL_2
#define MOTOR_PWM_RES_BITS LEDC_TIMER_10_BIT      /* duty 0..1023 */
#define MOTOR_PWM_DUTY_MAX ((1 << 10) - 1)
#define MOTOR_PWM_FREQ_HZ  10000                  /* XY-160D ceiling is 10 kHz; lower if it whines/heats */

/* One motor's wiring. */
typedef struct {
    gpio_num_t     pwm_gpio;   /* ENx */
    gpio_num_t     in1, in2;   /* direction pins */
    ledc_channel_t channel;    /* LEDC channel driving ENx */
    int            sign;       /* +1 or -1: spin direction vs a positive command */
} motor_t;

static const motor_t s_motors[2] = {
    { MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_CHANNEL, MOTOR_L_SIGN },   /* 0 = left  */
    { MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, MOTOR_R_CHANNEL, MOTOR_R_SIGN },   /* 1 = right */
};

/* Open-loop command per motor, -1.0..+1.0 (sign = direction). Set from the web
 * terminal, applied by control_task each tick. */
static volatile float s_motor_cmd[2];

void motor_cmd_set(int i, float cmd) { s_motor_cmd[i] = cmd; }
float motor_cmd_get(int i)           { return s_motor_cmd[i]; }

void motor_set(int i, float cmd)
{
    if (cmd >  1.0f) cmd =  1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    const motor_t *m = &s_motors[i];
    cmd *= m->sign;

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

void motor_init(void)
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
