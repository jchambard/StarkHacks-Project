#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "config.h"
#include "display.h"

static const char *TAG = "DISPLAY";

// ── SSD1306 constants ────────────────────────────────────────────────────────
#define SSD1306_CMD_PREFIX   0x00
#define SSD1306_DATA_PREFIX  0x40

// ── Icon bitmaps (12×12 px, column-major, 2 pages tall = 2 bytes per column) ─
// Each icon is 12 columns × 2 bytes = 24 bytes.
// Bit 0 of byte 0 = top-left pixel; bit 7 of byte 1 = bottom-left pixel.

static const uint8_t icon_bitmaps[ICON_COUNT][24] = {
    // ICON_TRIANGLE_WARNING (⚠ triangle with ! inside)
    [ICON_TRIANGLE_WARNING] = {
        0x00, 0x00,
        0x80, 0x00,
        0xC0, 0x01,
        0x60, 0x03,
        0x30, 0x07,
        0x30, 0x07,  // outline thicker centre columns
        0x60, 0x03,
        0xC0, 0x01,
        0x80, 0x00,
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00,
    },
    // ICON_SPEECH_BUBBLE (simple rounded rectangle with tail)
    [ICON_SPEECH_BUBBLE] = {
        0xF0, 0x00,
        0xFC, 0x00,
        0xFE, 0x00,
        0xFF, 0x00,
        0xFF, 0x00,
        0xFF, 0x00,
        0xFF, 0x00,
        0xFF, 0x00,
        0xFE, 0x00,
        0xFC, 0x01,
        0xF0, 0x03,
        0x00, 0x00,
    },
    // ICON_MUSIC_NOTE (single eighth note)
    [ICON_MUSIC_NOTE] = {
        0xFE, 0x00,
        0x02, 0x00,
        0x02, 0x00,
        0x02, 0x00,
        0x02, 0x00,
        0x02, 0x00,
        0x02, 0x00,
        0xFE, 0x01,
        0xFE, 0x01,
        0xFC, 0x01,
        0xF8, 0x00,
        0x00, 0x00,
    },
    // ICON_CAR (top-down rectangle with wheels)
    [ICON_CAR] = {
        0x00, 0x00,
        0xF0, 0x00,
        0xFE, 0x01,
        0xFF, 0x01,
        0xFF, 0x01,
        0xFF, 0x01,
        0xFF, 0x01,
        0xFF, 0x01,
        0xFE, 0x01,
        0xF0, 0x00,
        0x00, 0x00,
        0x00, 0x00,
    },
    // ICON_SIREN (star burst / flash shape)
    [ICON_SIREN] = {
        0x20, 0x00,
        0x70, 0x00,
        0xF8, 0x00,
        0xFE, 0x00,
        0xFF, 0x00,
        0xFF, 0x00,
        0xFE, 0x00,
        0xF8, 0x00,
        0x70, 0x00,
        0x20, 0x00,
        0x00, 0x00,
        0x00, 0x00,
    },
    // ICON_PERSON (stick figure silhouette)
    [ICON_PERSON] = {
        0x00, 0x00,
        0x78, 0x00,
        0x78, 0x00,
        0x30, 0x00,
        0xF8, 0x00,
        0xF8, 0x00,
        0x30, 0x00,
        0x30, 0x00,
        0x58, 0x00,
        0x48, 0x00,
        0x00, 0x00,
        0x00, 0x00,
    },
};

// ── Icon slot storage ────────────────────────────────────────────────────────

typedef struct {
    uint8_t  active;
    uint8_t  direction;
    IconType icon;
    uint32_t last_updated;  // tick count for LRU eviction
} IconSlot;

static IconSlot slots[MAX_ACTIVE_ICONS];
static uint32_t slot_tick = 0;

// ── Framebuffer & I²C handle ─────────────────────────────────────────────────

static uint8_t fb[OLED_WIDTH * OLED_HEIGHT / 8];  // 1024 bytes
static i2c_master_dev_handle_t oled_dev = NULL;

// ── SSD1306 low-level ────────────────────────────────────────────────────────

static esp_err_t oled_send_cmd(uint8_t cmd)
{
    uint8_t buf[2] = { SSD1306_CMD_PREFIX, cmd };
    return i2c_master_transmit(oled_dev, buf, 2, pdMS_TO_TICKS(20));
}

static esp_err_t oled_flush(void)
{
    // Set column/page address range covering the full display
    oled_send_cmd(0x21); oled_send_cmd(0); oled_send_cmd(127); // col 0–127
    oled_send_cmd(0x22); oled_send_cmd(0); oled_send_cmd(7);   // page 0–7

    // Transmit framebuffer in 64-byte chunks (prefix 0x40 + 63 data bytes each)
    uint8_t chunk[65];
    chunk[0] = SSD1306_DATA_PREFIX;
    for (int offset = 0; offset < 1024; offset += 64) {
        int len = (offset + 64 <= 1024) ? 64 : (1024 - offset);
        memcpy(&chunk[1], &fb[offset], len);
        esp_err_t err = i2c_master_transmit(oled_dev, chunk, len + 1, pdMS_TO_TICKS(50));
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

// ── Drawing primitives ───────────────────────────────────────────────────────

static void fb_set_pixel(int x, int y)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    fb[x + (y / 8) * OLED_WIDTH] |= (uint8_t)(1u << (y & 7));
}

// Midpoint circle algorithm
static void fb_draw_circle(int cx, int cy, int r)
{
    int x = r, y = 0, err = 0;
    while (x >= y) {
        fb_set_pixel(cx + x, cy + y); fb_set_pixel(cx + y, cy + x);
        fb_set_pixel(cx - y, cy + x); fb_set_pixel(cx - x, cy + y);
        fb_set_pixel(cx - x, cy - y); fb_set_pixel(cx - y, cy - x);
        fb_set_pixel(cx + y, cy - x); fb_set_pixel(cx + x, cy - y);
        y++;
        if (err <= 0) { err += 2 * y + 1; }
        else          { x--; err += 2 * (y - x) + 1; }
    }
}

// Draw 12×12 icon (column-major, 2 bytes per column) at pixel (px, py)
static void fb_draw_icon(int px, int py, IconType icon)
{
    if (icon >= ICON_COUNT) return;
    const uint8_t *bmp = icon_bitmaps[icon];
    for (int col = 0; col < ICON_SIZE; col++) {
        uint16_t col_data = (uint16_t)bmp[col * 2] | ((uint16_t)bmp[col * 2 + 1] << 8);
        for (int bit = 0; bit < ICON_SIZE; bit++) {
            if (col_data & (1u << bit)) {
                fb_set_pixel(px + col, py + bit);
            }
        }
    }
}

// Convert direction byte → (x, y) position on ring (top-left of ICON_SIZE box)
static void direction_to_xy(uint8_t direction, int *x, int *y)
{
    // Map 0–255 → 0–2π. 0 = east (right), clockwise.
    float angle = ((float)direction / 256.0f) * 2.0f * (float)M_PI;
    float fx    = RING_CENTER_X + (float)RING_RADIUS * cosf(angle) - ICON_SIZE / 2.0f;
    float fy    = RING_CENTER_Y + (float)RING_RADIUS * sinf(angle) - ICON_SIZE / 2.0f;
    *x = (int)fx;
    *y = (int)fy;
}

// ── Frame render ─────────────────────────────────────────────────────────────

static void render_frame(void)
{
    memset(fb, 0, sizeof(fb));
    fb_draw_circle(RING_CENTER_X, RING_CENTER_Y, RING_RADIUS);

    for (int i = 0; i < MAX_ACTIVE_ICONS; i++) {
        if (!slots[i].active) continue;
        int x, y;
        direction_to_xy(slots[i].direction, &x, &y);
        fb_draw_icon(x, y, slots[i].icon);
    }
    oled_flush();
}

// ── Slot management ──────────────────────────────────────────────────────────

static void update_slot(const DisplayCommand *cmd)
{
    slot_tick++;

    if (!cmd->active) {
        // CMD_CLEAR_ALL: zero all slots
        memset(slots, 0, sizeof(slots));
        return;
    }

    // Find existing slot with same direction or oldest slot for eviction
    int target = -1;
    uint32_t oldest_tick = UINT32_MAX;
    int oldest_idx = 0;

    for (int i = 0; i < MAX_ACTIVE_ICONS; i++) {
        if (slots[i].active && slots[i].direction == cmd->direction) {
            target = i;
            break;
        }
        if (slots[i].last_updated < oldest_tick) {
            oldest_tick = slots[i].last_updated;
            oldest_idx  = i;
        }
        if (!slots[i].active) {
            oldest_idx = i;
            oldest_tick = 0;
        }
    }

    if (target == -1) {
        target = oldest_idx;
    }

    slots[target].active       = 1;
    slots[target].direction    = cmd->direction;
    slots[target].icon         = cmd->icon;
    slots[target].last_updated = slot_tick;
}

// ── Public API ───────────────────────────────────────────────────────────────

esp_err_t display_init(i2c_master_bus_handle_t i2c_bus)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = OLED_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &oled_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SSD1306 to I²C bus: %s", esp_err_to_name(err));
        return err;
    }

    // SSD1306 init sequence
    const uint8_t init_cmds[] = {
        0xAE,        // display off
        0xD5, 0x80,  // clock divide ratio / oscillator frequency
        0xA8, 0x3F,  // multiplex ratio (64-1)
        0xD3, 0x00,  // display offset = 0
        0x40,        // display start line = 0
        0x8D, 0x14,  // charge pump on
        0x20, 0x00,  // horizontal addressing mode
        0xA1,        // segment remap (col 127 = SEG0)
        0xC8,        // COM output scan direction remapped
        0xDA, 0x12,  // COM pins hardware config
        0x81, 0xCF,  // contrast
        0xD9, 0xF1,  // pre-charge period
        0xDB, 0x40,  // VCOMH deselect level
        0xA4,        // output follows RAM
        0xA6,        // normal (non-inverted) display
        0xAF,        // display on
    };

    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        err = oled_send_cmd(init_cmds[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SSD1306 init cmd 0x%02X failed: %s",
                     init_cmds[i], esp_err_to_name(err));
            return err;
        }
    }

    memset(fb, 0, sizeof(fb));
    memset(slots, 0, sizeof(slots));
    render_frame();

    ESP_LOGI(TAG, "SSD1306 128x64 initialized");
    return ESP_OK;
}

void display_clear(void)
{
    memset(slots, 0, sizeof(slots));
    render_frame();
}

void display_test_pattern(void)
{
    for (int step = 0; step < 8; step++) {
        memset(slots, 0, sizeof(slots));
        slots[0].active    = 1;
        slots[0].direction = (uint8_t)(step * 32);  // 8 steps × 32 = 256 full circle
        slots[0].icon      = ICON_TRIANGLE_WARNING;
        render_frame();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    display_clear();
    ESP_LOGI(TAG, "Display test complete");
}

// ── Display task ─────────────────────────────────────────────────────────────

static void display_update_task(void *arg)
{
    QueueHandle_t q = (QueueHandle_t)arg;
    ESP_LOGI(TAG, "Display update task running");

    for (;;) {
        DisplayCommand cmd;
        if (xQueueReceive(q, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            update_slot(&cmd);
            render_frame();
        }
    }
}

void display_update_start(QueueHandle_t display_queue)
{
    xTaskCreatePinnedToCore(display_update_task, "display_upd",
                            STACK_SIZE_DISPLAY, display_queue,
                            PRIORITY_DISPLAY, NULL, 0);
}
