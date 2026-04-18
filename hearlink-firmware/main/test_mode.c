#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "config.h"
#include "test_mode.h"
#include "buzzer_control.h"
#include "display.h"
#include "audio_capture.h"

// Declared in main.c
extern volatile int16_t latest_imu_yaw_rate;
extern TaskHandle_t test_task_handle;

static const char *TAG = "TEST";

// CommandType values mirrored here to avoid circular include with network.h
#define CMD_TEST_BUZZERS 0x11
#define CMD_TEST_DISPLAY 0x12
#define CMD_TEST_MICS    0x13
#define CMD_TEST_IMU     0x14

// ── Individual tests ─────────────────────────────────────────────────────────

void test_mode_buzzers(void)
{
    ESP_LOGI(TAG, "Buzzer test starting (8 buzzers × 1s each)");
    for (int i = 0; i < NUM_BUZZERS; i++) {
        ESP_LOGI(TAG, "  buzzer %d ON", i);
        buzzer_set(i, 50);
        vTaskDelay(pdMS_TO_TICKS(1000));
        buzzer_set(i, 0);
    }
    ESP_LOGI(TAG, "Buzzer test complete");
}

void test_mode_display(void)
{
    ESP_LOGI(TAG, "Display test starting");
    display_test_pattern();  // rotates icon 8 × 45° with 500ms delay each
    ESP_LOGI(TAG, "Display test complete");
}

void test_mode_mics(QueueHandle_t audio_queue)
{
    ESP_LOGI(TAG, "Mic test starting (10 seconds)");
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(10000);

    while (xTaskGetTickCount() < end) {
        AudioBuffer buf;
        if (xQueuePeek(audio_queue, &buf, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        // Compute peak (faster than RMS on embedded)
        int32_t peak[NUM_CHANNELS] = {0};
        for (int s = 0; s < SAMPLES_PER_BUFFER; s++) {
            int32_t v;
            v = buf.ch0[s]; if (v < 0) v = -v; if (v > peak[0]) peak[0] = v;
            v = buf.ch1[s]; if (v < 0) v = -v; if (v > peak[1]) peak[1] = v;
            v = buf.ch2[s]; if (v < 0) v = -v; if (v > peak[2]) peak[2] = v;
            v = buf.ch3[s]; if (v < 0) v = -v; if (v > peak[3]) peak[3] = v;
        }
        ESP_LOGI(TAG, "Ch0: %7ld | Ch1: %7ld | Ch2: %7ld | Ch3: %7ld",
                 (long)peak[0], (long)peak[1], (long)peak[2], (long)peak[3]);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "Mic test complete");
}

void test_mode_imu(void)
{
    ESP_LOGI(TAG, "IMU test starting (10 seconds)");
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
    while (xTaskGetTickCount() < end) {
        ESP_LOGI(TAG, "Yaw rate: %d LSB", (int)latest_imu_yaw_rate);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "IMU test complete");
}

// ── Test dispatcher task ─────────────────────────────────────────────────────

static QueueHandle_t mic_queue_ref = NULL;

static void test_task(void *arg)
{
    ESP_LOGI(TAG, "Test task waiting for commands");
    for (;;) {
        uint32_t cmd_type = 0;
        xTaskNotifyWait(0, UINT32_MAX, &cmd_type, portMAX_DELAY);

        ESP_LOGI(TAG, "Running test 0x%02lX", (unsigned long)cmd_type);
        switch (cmd_type) {
        case CMD_TEST_BUZZERS: test_mode_buzzers();              break;
        case CMD_TEST_DISPLAY: test_mode_display();              break;
        case CMD_TEST_MICS:    test_mode_mics(mic_queue_ref);   break;
        case CMD_TEST_IMU:     test_mode_imu();                  break;
        default:
            ESP_LOGW(TAG, "Unknown test command 0x%02lX", (unsigned long)cmd_type);
            break;
        }
    }
}

void test_mode_task_start(QueueHandle_t audio_queue)
{
    mic_queue_ref = audio_queue;
    xTaskCreatePinnedToCore(test_task, "test_task",
                            STACK_SIZE_TEST, NULL,
                            PRIORITY_TEST, &test_task_handle, 0);
}
