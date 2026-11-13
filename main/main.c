// Mini project "Safe" - ESP-IDF, ESP32-S3
//
// Wiring:
//   KY-040 encoder: CLK -> GPIO5, DT -> GPIO6, SW -> GPIO7, + -> 3V3, GND -> GND
//   Buzzer KY-006:  S -> GPIO2, "-" -> GND
//   Servo SG90:     signal -> GPIO1, power from external 5 V, common GND with the board
//
// Code entry (default 3141):
//   turn one way - digit goes 0..9 (wraps around);
//   change direction - digit confirmed, next one starts at 0;
//   after the last digit change direction again - code is checked.
//   Button: reset input (counts as an attempt) / lock an open safe.
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "SAFE";

#define ENC_A_PIN    5
#define ENC_B_PIN    6
#define ENC_SW_PIN   7
#define SERVO_PIN    1
#define BUZZER_PIN   2

#define LOOP_MS            10
#define COUNTS_PER_DETENT  4    // X4 decoding: one click = 4 counts

/* ===== Encoder (PCNT, X4) ===== */
static pcnt_unit_handle_t enc_unit;
static int enc_last = 0;
static int enc_rest = 0;

static void encoder_init(void)
{
    pcnt_unit_config_t ucfg = {
        .low_limit = -1000,
        .high_limit = 1000,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &enc_unit));

    pcnt_glitch_filter_config_t flt = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(enc_unit, &flt));

    pcnt_chan_config_t a_cfg = { .edge_gpio_num = ENC_A_PIN, .level_gpio_num = ENC_B_PIN };
    pcnt_chan_config_t b_cfg = { .edge_gpio_num = ENC_B_PIN, .level_gpio_num = ENC_A_PIN };
    pcnt_channel_handle_t ch_a = NULL, ch_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(enc_unit, &a_cfg, &ch_a));
    ESP_ERROR_CHECK(pcnt_new_channel(enc_unit, &b_cfg, &ch_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // accum_count requires the limits as watch points
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(enc_unit, ucfg.low_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(enc_unit, ucfg.high_limit));

    ESP_ERROR_CHECK(pcnt_unit_enable(enc_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(enc_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(enc_unit));
}

// Full clicks since the previous call (positive or negative)
static int encoder_take_steps(void)
{
    int now = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(enc_unit, &now));
    enc_rest += now - enc_last;
    enc_last = now;
    int steps = enc_rest / COUNTS_PER_DETENT;
    enc_rest -= steps * COUNTS_PER_DETENT;
    return steps;
}

/* ===== Button (polling + debounce) ===== */
static int btn_stable = 1, btn_prev = 1, btn_same = 0;

static void button_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << ENC_SW_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

// True once, at the moment of the press
static bool button_pressed(void)
{
    int raw = gpio_get_level(ENC_SW_PIN);
    btn_same = (raw == btn_prev) ? btn_same + 1 : 0;
    btn_prev = raw;
    if (btn_same >= 3 && raw != btn_stable) {     // stable for 30 ms
        btn_stable = raw;
        return raw == 0;
    }
    return false;
}

/* ===== Servo (LEDC timer 0, 50 Hz) ===== */
#define SERVO_CH   LEDC_CHANNEL_0

static void servo_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ledc_channel_config_t c = {
        .gpio_num = SERVO_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = SERVO_CH,
        .timer_sel = LEDC_TIMER_0, .duty = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

// Angle in tenths of a degree (0..1800): 0 deg = 500 us, 180 deg = 2500 us
static void servo_write_ddeg(int ddeg)
{
    uint32_t us = 500 + (uint32_t)ddeg * 2000 / 1800;
    uint32_t duty = us * 16384 / 20000;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CH, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CH));
    ESP_LOGI(TAG, "Servo -> %d deg", ddeg / 10);
}

/* ===== Buzzer (LEDC timer 1) ===== */
#define BUZZER_CH  LEDC_CHANNEL_1

static void buzzer_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_1, .freq_hz = 1000, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ledc_channel_config_t c = {
        .gpio_num = BUZZER_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = BUZZER_CH,
        .timer_sel = LEDC_TIMER_1, .duty = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

static void tone(uint16_t hz)   // 0 = silence
{
    if (hz) ESP_ERROR_CHECK(ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, hz));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CH, hz ? 512 : 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CH));
}

/* ===== Non-blocking melody player ===== */
typedef struct { uint16_t hz; uint16_t ms; } note_t;   // {0, 0} = end

static const note_t *mel_start = NULL, *mel = NULL;
static int  mel_left = 0;
static bool mel_loop = false;

static void play(const note_t *m, bool loop)
{
    mel_start = mel = m; mel_loop = loop; mel_left = m->ms;
    tone(m->hz);
}

static void player_update(void)   // call every LOOP_MS
{
    if (!mel) return;
    mel_left -= LOOP_MS;
    if (mel_left > 0) return;
    mel++;
    if (mel->ms == 0) {
        if (!mel_loop) { mel = NULL; tone(0); return; }
        mel = mel_start;
    }
    mel_left = mel->ms;
    tone(mel->hz);
}

static const note_t BEEP[] = { {1500, 60}, {0, 0} };

/* ===== Safe logic ===== */
#define CODE_LEN       4
#define MAX_ATTEMPTS   3
#define LOCKED_DDEG    0      // 0 deg
#define OPEN_DDEG      900    // 90 deg

static const uint8_t SECRET[CODE_LEN] = { 3, 1, 4, 1 };

static const note_t MELODY_OK[]    = { {523,120}, {659,120}, {784,120}, {1047,350}, {0,0} };
static const note_t MELODY_WRONG[] = { {330,250}, {0,80}, {262,400}, {0,0} };
static const note_t MELODY_ALARM[] = { {880,250}, {660,250}, {0,0} };   // looped

typedef enum { ST_INPUT, ST_OPEN, ST_BLOCKED } state_t;

static state_t state = ST_INPUT;
static int attempts = 0;
static uint8_t entered[CODE_LEN];
static int n_entered = 0;
static int digit = 0;         // digit being dialed
static int last_dir = 0;      // +1 / -1, 0 = not turned yet

static void digits_str(const uint8_t *d, char *out)
{
    for (int i = 0; i < CODE_LEN; i++) out[i] = (char)('0' + d[i]);
    out[CODE_LEN] = '\0';
}

static void input_start(void)
{
    n_entered = 0; digit = 0; last_dir = 0;
    ESP_LOGI(TAG, "Attempt %d of %d, enter the code", attempts + 1, MAX_ATTEMPTS);
}

static void wrong_attempt(const char *why)
{
    attempts++;
    if (attempts >= MAX_ATTEMPTS) {
        ESP_LOGE(TAG, "%s. No attempts left - ALARM, blocked until reboot", why);
        play(MELODY_ALARM, true);
        state = ST_BLOCKED;
    } else {
        ESP_LOGW(TAG, "%s. Attempts left: %d", why, MAX_ATTEMPTS - attempts);
        play(MELODY_WRONG, false);
        input_start();
    }
}

static void check_code(void)
{
    char got[CODE_LEN + 1], want[CODE_LEN + 1];
    digits_str(entered, got);
    digits_str(SECRET, want);
    ESP_LOGI(TAG, "Entered: %s, password: %s", got, want);

    if (memcmp(entered, SECRET, CODE_LEN) == 0) {
        ESP_LOGI(TAG, "Correct code - opening. Press the button to lock");
        servo_write_ddeg(OPEN_DDEG);
        play(MELODY_OK, false);
        state = ST_OPEN;
    } else {
        wrong_attempt("Wrong code");
    }
}

// One encoder click: dir = +1 or -1
static void on_step(int dir)
{
    if (state != ST_INPUT) return;

    if (last_dir != 0 && dir != last_dir) {          // direction changed = confirm digit
        entered[n_entered++] = (uint8_t)digit;
        ESP_LOGI(TAG, "Digit %d confirmed (%d/%d)", digit, n_entered, CODE_LEN);
        play(BEEP, false);
        last_dir = dir;
        if (n_entered == CODE_LEN) { check_code(); return; }
        digit = 0;
    } else {
        last_dir = dir;
        digit = (digit + 1) % 10;
        ESP_LOGI(TAG, "Digit: %d", digit);
    }
}

static void on_button(void)
{
    switch (state) {
    case ST_INPUT:
        wrong_attempt("Reset by button");
        break;
    case ST_OPEN:
        servo_write_ddeg(LOCKED_DDEG);
        attempts = 0;
        state = ST_INPUT;
        ESP_LOGI(TAG, "Locked");
        input_start();
        break;
    case ST_BLOCKED:
        break;
    }
}

void app_main(void)
{
    servo_init();
    buzzer_init();
    encoder_init();
    button_init();
    servo_write_ddeg(LOCKED_DDEG);

    input_start();

    TickType_t last = xTaskGetTickCount();
    while (1) {
        int steps = encoder_take_steps();
        for (; steps > 0; steps--) on_step(+1);
        for (; steps < 0; steps++) on_step(-1);

        if (button_pressed()) on_button();

        player_update();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(LOOP_MS));
    }
}
