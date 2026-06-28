#include "encoders.h"

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"   /* PCNT: hardware quadrature encoder counting */
#include "esp_log.h"

static const char *TAG = "encoders";

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

/* The right wheel is mounted mirror-image to the left, so its encoder counts
 * down when the robot drives forward. Flip its sign so +counts = forward for
 * both wheels (keeps position and the velocity deltas consistent downstream). */
#define ENC_L_SIGN         (+1)
#define ENC_R_SIGN         (-1)

/* One encoder. PCNT counts in hardware (16-bit); on each rollover the watch
 * callback adds the limit to accum, giving a full 64-bit position. The sign is
 * applied at read time to make +counts = forward regardless of mounting. */
typedef struct {
    pcnt_unit_handle_t unit;
    volatile int64_t   accum;
    int                sign;   /* +1 or -1: orientation relative to forward */
} encoder_t;

static encoder_t s_enc[2];

/* Rollover callback (ISR): the count just hit +/-limit and auto-reset to 0, so
 * fold that limit into the 64-bit accumulator. */
static bool IRAM_ATTR enc_on_reach(pcnt_unit_handle_t unit,
                                   const pcnt_watch_event_data_t *edata, void *user)
{
    encoder_t *e = (encoder_t *)user;
    e->accum += edata->watch_point_value;
    return false;
}

/* Configure one PCNT unit for 4x quadrature decoding of an A/B pair. @p sign is
 * +1 or -1 to align the counting direction with forward robot motion. */
static void encoder_init_one(encoder_t *e, gpio_num_t a, gpio_num_t b, int sign)
{
    e->sign = sign;

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

void encoder_init(void)
{
    encoder_init_one(&s_enc[0], ENC_L_A, ENC_L_B, ENC_L_SIGN);
    encoder_init_one(&s_enc[1], ENC_R_A, ENC_R_B, ENC_R_SIGN);
    ESP_LOGI(TAG, "encoders ready (PCNT) - L:A=%d/B=%d  R:A=%d/B=%d",
             ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B);
}

/* Total position = accumulator + current hardware count. Retry if a rollover
 * lands between the two reads so we never mix an old accum with a new count. */
int64_t encoder_count(int i)
{
    int64_t a0, a1;
    int cur = 0;
    do {
        a0 = s_enc[i].accum;
        pcnt_unit_get_count(s_enc[i].unit, &cur);
        a1 = s_enc[i].accum;
    } while (a0 != a1);
    return s_enc[i].sign * (a1 + cur);
}

void encoder_reset(int i)
{
    s_enc[i].accum = 0;
    pcnt_unit_clear_count(s_enc[i].unit);
}
