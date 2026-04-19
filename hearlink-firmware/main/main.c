#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "led_strip.h"
#include "driver/i2c_master.h"

#include "config.h"
#include "audio_capture.h"
#include "imu_driver.h"
#include "network.h"
#include "buzzer_control.h"
#include "display.h"
#include "calibration.h"
#include "test_mode.h"

static const char *TAG = "MAIN";

// ── Globals (referenced by other modules via extern) ──────────────────────────

QueueHandle_t     audio_queue    = NULL;
QueueHandle_t     buzzer_queue   = NULL;
QueueHandle_t     display_queue  = NULL;
SemaphoreHandle_t imu_mutex      = NULL;
TaskHandle_t      test_task_handle = NULL;

volatile int16_t latest_imu_yaw_rate = 0;

// ── Status LED ────────────────────────────────────────────────────────────────

static led_strip_handle_t led_strip = NULL;

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!led_strip) return;
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

// ── Entry point ───────────────────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "HearLink Firmware Starting...");

    // ── 1. NVS ───────────────────────────────────────────────────────────────
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ── 2. Status LED ─────────────────────────────────────────────────────────
    led_strip_config_t led_cfg = {
        .strip_gpio_num  = STATUS_LED_PIN,
        .max_leds        = STATUS_LED_COUNT,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model        = LED_MODEL_WS2812,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,  // 10 MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_cfg, &rmt_cfg, &led_strip));
    led_set(0, 8, 0);  // dim green = booting

    // ── 3. I²C bus (shared by IMU + OLED) ────────────────────────────────────
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port             = I2C_NUM_0,
        .sda_io_num           = I2C_SDA_PIN,
        .scl_io_num           = I2C_SCL_PIN,
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt    = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));

    // ── 4. Hardware init ──────────────────────────────────────────────────────
    ESP_ERROR_CHECK(audio_capture_init());
    esp_err_t imu_err = imu_init(i2c_bus);
    if (imu_err != ESP_OK) {
        ESP_LOGW(TAG, "IMU not found (%s) — check wiring on GPIO8/9", esp_err_to_name(imu_err));
    }
    ESP_ERROR_CHECK(buzzer_init());
    esp_err_t disp_err = display_init(i2c_bus);
    if (disp_err != ESP_OK) {
        ESP_LOGW(TAG, "OLED not found (%s) — check wiring on GPIO8/9", esp_err_to_name(disp_err));
    }

    // ── 5. Calibration (loads NVS → applies offsets to audio) ─────────────────
    ESP_ERROR_CHECK(calibration_init());

    // ── 6. Queues and mutex ───────────────────────────────────────────────────
    audio_queue   = xQueueCreate(10, sizeof(AudioBuffer));
    buzzer_queue  = xQueueCreate(5,  sizeof(BuzzerCommand));
    display_queue = xQueueCreate(5,  sizeof(DisplayCommand));
    imu_mutex     = xSemaphoreCreateMutex();
    configASSERT(audio_queue && buzzer_queue && display_queue && imu_mutex);

#if MIC_TEST_BOOT_MODE
    // Bring-up path: skip network/app and just cycle each mic one at a time.
    // Must run in its own task — AudioBuffer (~2.5KB) won't fit on app_main's
    // default ~3.5KB stack.
    ESP_LOGW(TAG, "MIC_TEST_BOOT_MODE=1 — entering single-mic bring-up loop");
    led_set(16, 8, 0);  // amber = test mode
    audio_capture_start(audio_queue);
    extern void mic_test_loop_task(void *arg);
    xTaskCreatePinnedToCore(mic_test_loop_task, "mic_test_loop",
                            STACK_SIZE_TEST, audio_queue,
                            PRIORITY_TEST, NULL, 0);
    for (;;) vTaskDelay(portMAX_DELAY);
#endif

    // ── 7. Network ────────────────────────────────────────────────────────────
    ESP_ERROR_CHECK(network_init());

    // ── 8. Tasks ──────────────────────────────────────────────────────────────
    // Core 1: audio-critical
    audio_capture_start(audio_queue);   // priority 5
    udp_tx_start(audio_queue);          // priority 4

    // Core 0: control + I/O
    udp_rx_start(buzzer_queue, display_queue);  // priority 3
    imu_poll_start();                           // priority 3
    buzzer_control_start(buzzer_queue);         // priority 2
    display_update_start(display_queue);        // priority 1
    test_mode_task_start(audio_queue);          // priority 1

    led_set(0, 32, 0);  // bright green = ready
    ESP_LOGI(TAG, "All tasks started. System ready.");

    // ── 9. Heartbeat loop ─────────────────────────────────────────────────────
    esp_log_level_set("AUDIO",   ESP_LOG_INFO);
    esp_log_level_set("IMU",     ESP_LOG_INFO);
    esp_log_level_set("NET",     ESP_LOG_INFO);
    esp_log_level_set("BUZZER",  ESP_LOG_WARN);
    esp_log_level_set("DISPLAY", ESP_LOG_INFO);
    esp_log_level_set("TEST",    ESP_LOG_INFO);
    esp_log_level_set("CAL",     ESP_LOG_INFO);

    bool led_on = true;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t dropped = audio_get_dropped_frames();
        if (dropped > 0) {
            ESP_LOGW(TAG, "Heartbeat: %lu frames dropped in last interval",
                     (unsigned long)dropped);
            led_set(32, 0, 0);  // red flash = overload
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        led_set(0, led_on ? 16 : 4, 0);  // blink green
        led_on = !led_on;
    }
}
