#ifndef BUZZER_CONTROL_H
#define BUZZER_CONTROL_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    PATTERN_OFF        = 0,
    PATTERN_CONTINUOUS = 1,
    PATTERN_PULSE_FAST = 2,  // 200ms on, 200ms off
    PATTERN_PULSE_SLOW = 3,  // 500ms on, 500ms off
    PATTERN_DOUBLE_TAP = 4,  // 100ms on, 100ms off, 100ms on, then off
} BuzzerPattern;

typedef struct {
    uint8_t       buzzer_mask;  // Bit N = buzzer N (bit 0 = buzzer 0)
    BuzzerPattern pattern;
    uint8_t       intensity;    // 0–100 → maps to 0–1023 LEDC duty
} BuzzerCommand;

/**
 * Initialize LEDC timer and 8 output channels.
 * Timer: 400Hz, 10-bit resolution.
 * All buzzers start at 0 (silent).
 */
esp_err_t buzzer_init(void);

/**
 * Start buzzer control task on Core 0 at priority PRIORITY_BUZZER.
 * Task handles pattern timing for all 8 buzzers.
 * @param buzzer_queue Source queue for BuzzerCommand items
 */
void buzzer_control_start(QueueHandle_t buzzer_queue);

/**
 * Set a buzzer's LEDC duty directly (bypasses pattern logic).
 * Intended for test_mode use only.
 * @param buzzer_index 0–7
 * @param intensity    0–100
 */
void buzzer_set(uint8_t buzzer_index, uint8_t intensity);

#endif // BUZZER_CONTROL_H
