#include "led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define LED_L_GPIO     GPIO_NUM_17                /* Left LED  - notification blink */
#define LED_R_GPIO     GPIO_NUM_16                /* Right LED - notification blink */

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

void led_blink(uint16_t on_ms, uint16_t off_ms, int16_t count)
{
    led_pattern_t p = { on_ms, off_ms, count };
    if (s_led_q) xQueueOverwrite(s_led_q, &p);
}

void led_off(void)           { led_blink(0, 0, 0); }
void led_on(void)            { led_blink(1, 0, -1); }
void led_heartbeat(void)     { led_blink(60, 1940, -1); }   /* ~0.5 Hz */
void led_heartbeat_1hz(void) { led_blink(60, 940, -1); }    /* 1 Hz: balancing */

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

void led_init(void)
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
