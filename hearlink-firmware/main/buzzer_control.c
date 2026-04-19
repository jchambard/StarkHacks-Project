#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "config.h"
#include "buzzer_control.h"

static const char *TAG = "BUZZER";

static const int buzzer_gpios[NUM_BUZZERS] = BUZZER_PINS;

// Per-buzzer active pattern state
typedef struct {
    BuzzerPattern pattern;
    uint8_t       intensity;
    uint8_t       phase;        // current phase within pattern sequence
    TickType_t    phase_end;    // when to advance to next phase
} BuzzerState;

static BuzzerState bstate[NUM_BUZZERS];

// ── Helpers ──────────────────────────────────────────────────────────────────

static void set_duty(uint8_t idx, uint8_t intensity)
{
    if (intensity == 0) {
        // Force the pin to idle LOW — ledc_set_duty(0) alone can leave the
        // channel "running" and produce residual glitches.
        ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, 0);
        return;
    }
    if (intensity > 100) intensity = 100;
    // Piezo volume is proportional to AC swing, which peaks at 50% duty.
    // Map intensity 1..100 → duty 1..512 (≈0.1% .. 50%).
    uint32_t duty = ((uint32_t)intensity * 1023) / 200;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx);
}

// ── Public API ───────────────────────────────────────────────────────────────

esp_err_t buzzer_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = BUZZER_PWM_RES,
        .freq_hz         = BUZZER_PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(err));
        return err;
    }

    for (int i = 0; i < NUM_BUZZERS; i++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = buzzer_gpios[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)i,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0,
        };
        err = ledc_channel_config(&ch_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "LEDC ch%d config failed: %s", i, esp_err_to_name(err));
            return err;
        }
        memset(&bstate[i], 0, sizeof(BuzzerState));
    }

    ESP_LOGI(TAG, "Buzzer LEDC initialized (%dHz, 10-bit, %d channels)",
             BUZZER_PWM_FREQ, NUM_BUZZERS);
    return ESP_OK;
}

void buzzer_set(uint8_t buzzer_index, uint8_t intensity)
{
    if (buzzer_index >= NUM_BUZZERS) return;
    set_duty(buzzer_index, intensity);
    bstate[buzzer_index].pattern   = PATTERN_CONTINUOUS;
    bstate[buzzer_index].intensity = intensity;
}

// ── Pattern tick ─────────────────────────────────────────────────────────────
// Called each time through the task loop. Advances pattern state for each buzzer.
// Returns the ticks until the soonest next state change (for queue timeout).
static TickType_t tick_patterns(TickType_t now)
{
    TickType_t next_wake = portMAX_DELAY;

    for (int i = 0; i < NUM_BUZZERS; i++) {
        BuzzerState *s = &bstate[i];

        if (s->pattern == PATTERN_OFF || s->pattern == PATTERN_CONTINUOUS) {
            continue;  // no timer-driven changes
        }

        if ((TickType_t)(now - s->phase_end) < (TickType_t)(portMAX_DELAY / 2)) {
            // phase_end not yet reached
            TickType_t remaining = s->phase_end - now;
            if (remaining < next_wake) next_wake = remaining;
            continue;
        }

        // Advance pattern phase
        switch (s->pattern) {
        case PATTERN_PULSE_FAST:
            // on 200ms, off 200ms, repeat
            if (s->phase == 0) {
                set_duty(i, s->intensity);
                s->phase     = 1;
                s->phase_end = now + pdMS_TO_TICKS(200);
            } else {
                set_duty(i, 0);
                s->phase     = 0;
                s->phase_end = now + pdMS_TO_TICKS(200);
            }
            break;

        case PATTERN_PULSE_SLOW:
            // on 500ms, off 500ms, repeat
            if (s->phase == 0) {
                set_duty(i, s->intensity);
                s->phase     = 1;
                s->phase_end = now + pdMS_TO_TICKS(500);
            } else {
                set_duty(i, 0);
                s->phase     = 0;
                s->phase_end = now + pdMS_TO_TICKS(500);
            }
            break;

        case PATTERN_DOUBLE_TAP:
            // phase: 0=1st on, 1=gap, 2=2nd on, 3=off(done)
            switch (s->phase) {
            case 0:
                set_duty(i, s->intensity);
                s->phase     = 1;
                s->phase_end = now + pdMS_TO_TICKS(100);
                break;
            case 1:
                set_duty(i, 0);
                s->phase     = 2;
                s->phase_end = now + pdMS_TO_TICKS(100);
                break;
            case 2:
                set_duty(i, s->intensity);
                s->phase     = 3;
                s->phase_end = now + pdMS_TO_TICKS(100);
                break;
            default:
                set_duty(i, 0);
                s->pattern = PATTERN_OFF;
                break;
            }
            break;

        default:
            break;
        }

        if (s->pattern != PATTERN_OFF) {
            TickType_t remaining = s->phase_end - now;
            if (remaining < next_wake) next_wake = remaining;
        }
    }
    return next_wake;
}

// ── Buzzer task ──────────────────────────────────────────────────────────────

static void buzzer_control_task(void *arg)
{
    QueueHandle_t q = (QueueHandle_t)arg;
    ESP_LOGI(TAG, "Buzzer control task running");

    for (;;) {
        TickType_t now     = xTaskGetTickCount();
        TickType_t timeout = tick_patterns(now);

        BuzzerCommand cmd;
        if (xQueueReceive(q, &cmd, timeout) == pdTRUE) {
            // Apply command to each buzzer in mask
            for (int i = 0; i < NUM_BUZZERS; i++) {
                if (!(cmd.buzzer_mask & (1u << i))) {
                    // Not in mask: turn off
                    set_duty(i, 0);
                    bstate[i].pattern = PATTERN_OFF;
                    continue;
                }

                bstate[i].intensity = cmd.intensity;
                bstate[i].pattern   = cmd.pattern;
                bstate[i].phase     = 0;
                now = xTaskGetTickCount();

                switch (cmd.pattern) {
                case PATTERN_OFF:
                    set_duty(i, 0);
                    break;
                case PATTERN_CONTINUOUS:
                    set_duty(i, cmd.intensity);
                    break;
                case PATTERN_PULSE_FAST:
                    set_duty(i, cmd.intensity);
                    bstate[i].phase     = 1;
                    bstate[i].phase_end = now + pdMS_TO_TICKS(200);
                    break;
                case PATTERN_PULSE_SLOW:
                    set_duty(i, cmd.intensity);
                    bstate[i].phase     = 1;
                    bstate[i].phase_end = now + pdMS_TO_TICKS(500);
                    break;
                case PATTERN_DOUBLE_TAP:
                    set_duty(i, cmd.intensity);
                    bstate[i].phase     = 1;
                    bstate[i].phase_end = now + pdMS_TO_TICKS(100);
                    break;
                }
            }
        }
    }
}

void buzzer_control_start(QueueHandle_t buzzer_queue)
{
    xTaskCreatePinnedToCore(buzzer_control_task, "buzzer_ctrl",
                            STACK_SIZE_BUZZER, buzzer_queue,
                            PRIORITY_BUZZER, NULL, 0);
}
