#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"

typedef enum {
    ICON_TRIANGLE_WARNING = 0,
    ICON_SPEECH_BUBBLE    = 1,
    ICON_MUSIC_NOTE       = 2,
    ICON_CAR              = 3,
    ICON_SIREN            = 4,
    ICON_PERSON           = 5,
    ICON_COUNT            = 6,
} IconType;

typedef struct {
    uint8_t  direction;  // 0–255 maps to 0–360° around the ring
    IconType icon;
    uint8_t  category;   // SoundCategory (for potential future style decisions)
    uint8_t  active;     // 1 = show this slot, 0 = clear all slots (CMD_CLEAR_ALL)
} DisplayCommand;

/**
 * Initialize SSD1306 OLED at I²C address OLED_ADDR (0x3C).
 * Draws an empty ring on first frame.
 * @param i2c_bus Shared I²C master bus handle
 */
esp_err_t display_init(i2c_master_bus_handle_t i2c_bus);

/**
 * Start display update task on Core 0 at priority PRIORITY_DISPLAY.
 * @param display_queue Source queue for DisplayCommand items
 */
void display_update_start(QueueHandle_t display_queue);

/** Clear all icon slots and push blank frame to OLED. */
void display_clear(void);

/** Cycle a test icon around the ring at 8 positions × 500ms each. */
void display_test_pattern(void);

#endif // DISPLAY_H
