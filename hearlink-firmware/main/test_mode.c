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
#define CMD_TEST_BUZZERS     0x11
#define CMD_TEST_DISPLAY     0x12
#define CMD_TEST_MICS        0x13
#define CMD_TEST_IMU         0x14
#define CMD_TEST_MICS_INDIV  0x15
#define CMD_TEST_BUZZER_SOLO 0x16

// ── Individual tests ─────────────────────────────────────────────────────────

void test_mode_buzzers(void)
{
    ESP_LOGI(TAG, "Buzzer test starting (%d buzzers × 1s each)", NUM_BUZZERS);
    for (int i = 0; i < NUM_BUZZERS; i++) {
        ESP_LOGI(TAG, "  buzzer %d ON", i);
        buzzer_set(i, 50);
        vTaskDelay(pdMS_TO_TICKS(1000));
        buzzer_set(i, 0);
    }
    ESP_LOGI(TAG, "Buzzer test complete");
}

void test_mode_buzzer_solo(uint8_t index)
{
    if (index >= NUM_BUZZERS) {
        ESP_LOGW(TAG, "Solo buzzer test: invalid index %d (max %d)",
                 index, NUM_BUZZERS - 1);
        return;
    }
    ESP_LOGI(TAG, "Solo buzzer test: buzzer %d ON at 100%% for 10s", index);
    buzzer_set(index, 100);
    vTaskDelay(pdMS_TO_TICKS(10000));
    buzzer_set(index, 0);
    ESP_LOGI(TAG, "Solo buzzer test complete");
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

    // Outrank UDP TX (priority 4) so we win the race for queue items
    vTaskPrioritySet(NULL, PRIORITY_UDP_TX + 2);

    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(10000);

    while (xTaskGetTickCount() < end) {
        AudioBuffer buf;
        if (xQueueReceive(audio_queue, &buf, pdMS_TO_TICKS(100)) != pdTRUE) {
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
        const int BAR_WIDTH = 20;
        char bars[NUM_CHANNELS][BAR_WIDTH + 1];
        int pct[NUM_CHANNELS];
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            pct[ch] = (int)(((int64_t)peak[ch] * 100) / INT32_MAX);
            int n = (int)(((int64_t)peak[ch] * BAR_WIDTH) / INT32_MAX);
            if (n > BAR_WIDTH) n = BAR_WIDTH;
            for (int i = 0; i < BAR_WIDTH; i++) bars[ch][i] = (i < n) ? '#' : ' ';
            bars[ch][BAR_WIDTH] = '\0';
        }
        ESP_LOGI(TAG, "Ch0 [%s] %3d%%", bars[0], pct[0]);
        ESP_LOGI(TAG, "Ch1 [%s] %3d%%", bars[1], pct[1]);
        ESP_LOGI(TAG, "Ch2 [%s] %3d%%", bars[2], pct[2]);
        ESP_LOGI(TAG, "Ch3 [%s] %3d%%", bars[3], pct[3]);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskPrioritySet(NULL, PRIORITY_TEST);
    ESP_LOGI(TAG, "Mic test complete");
}

void test_mode_mics_individual(QueueHandle_t audio_queue)
{
    static const char *slot_name[NUM_CHANNELS] = {
        "MIC 0  (Bus 0 / I2S0 LEFT slot,  SD=GPIO6)",
        "MIC 1  (Bus 0 / I2S0 RIGHT slot, SD=GPIO6)",
        "MIC 2  (Bus 1 / I2S1 LEFT slot,  SD=GPIO16)",
        "MIC 3  (Bus 1 / I2S1 RIGHT slot, SD=GPIO16)",
    };

    // Outrank UDP TX so frames reach us, not the network path
    vTaskPrioritySet(NULL, PRIORITY_UDP_TX + 2);

    for (int mic = 0; mic < NUM_CHANNELS; mic++) {
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "================================================================");
        ESP_LOGW(TAG, "  TESTING %s", slot_name[mic]);
        ESP_LOGW(TAG, "  Tap/speak at this mic NOW  (%d seconds)",
                 MIC_TEST_SECONDS_PER_MIC);
        ESP_LOGW(TAG, "================================================================");

        TickType_t end = xTaskGetTickCount() +
                         pdMS_TO_TICKS(MIC_TEST_SECONDS_PER_MIC * 1000);
        int32_t peak_max = 0;
        int log_div = 0;

        while (xTaskGetTickCount() < end) {
            AudioBuffer buf;
            if (xQueueReceive(audio_queue, &buf, pdMS_TO_TICKS(100)) != pdTRUE) {
                continue;
            }
            int32_t *ch = (mic == 0) ? buf.ch0 :
                          (mic == 1) ? buf.ch1 :
                          (mic == 2) ? buf.ch2 : buf.ch3;

            int32_t peak = 0;
            for (int s = 0; s < SAMPLES_PER_BUFFER; s++) {
                int32_t v = ch[s]; if (v < 0) v = -v;
                if (v > peak) peak = v;
            }
            if (peak > peak_max) peak_max = peak;

            const int BAR = 30;
            int n = (int)(((int64_t)peak * BAR) / INT32_MAX);
            if (n > BAR) n = BAR;
            int pct = (int)(((int64_t)peak * 100) / INT32_MAX);
            char bars[BAR + 1];
            for (int i = 0; i < BAR; i++) bars[i] = (i < n) ? '#' : '.';
            bars[BAR] = '\0';

            if ((log_div++ % 5) == 0) {  // ~throttle to ~5/sec
                ESP_LOGI(TAG, "Mic %d [%s] %3d%%  peak=%ld",
                         mic, bars, pct, (long)peak);
            }
        }

        int pct_max = (int)(((int64_t)peak_max * 100) / INT32_MAX);
        if (peak_max >= MIC_DETECT_MIN_PP / 2) {
            ESP_LOGW(TAG, "Mic %d RESULT: OK    (max peak=%ld, %d%%)",
                     mic, (long)peak_max, pct_max);
        } else {
            ESP_LOGE(TAG, "Mic %d RESULT: SILENT (max peak=%ld, %d%%) — check wiring/power",
                     mic, (long)peak_max, pct_max);
        }
    }

    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "Individual mic test complete.");

    vTaskPrioritySet(NULL, PRIORITY_TEST);
}

// Standalone looping task used by MIC_TEST_BOOT_MODE; runs on its own stack
// so the large AudioBuffer inside test_mode_mics_individual doesn't blow
// app_main's default stack.
void mic_test_loop_task(void *arg)
{
    QueueHandle_t audio_queue = (QueueHandle_t)arg;
    for (;;) {
        test_mode_mics_individual(audio_queue);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
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
        uint32_t notify_word = 0;
        xTaskNotifyWait(0, UINT32_MAX, &notify_word, portMAX_DELAY);

        uint8_t cmd  = (uint8_t)(notify_word & 0xFF);
        uint8_t arg0 = (uint8_t)((notify_word >> 8) & 0xFF);

        ESP_LOGI(TAG, "Running test 0x%02X (arg=%u)", cmd, arg0);
        switch (cmd) {
        case CMD_TEST_BUZZERS:      test_mode_buzzers();                        break;
        case CMD_TEST_DISPLAY:      test_mode_display();                        break;
        case CMD_TEST_MICS:         test_mode_mics(mic_queue_ref);             break;
        case CMD_TEST_MICS_INDIV:   test_mode_mics_individual(mic_queue_ref);  break;
        case CMD_TEST_IMU:          test_mode_imu();                            break;
        case CMD_TEST_BUZZER_SOLO:  test_mode_buzzer_solo(arg0);                break;
        default:
            ESP_LOGW(TAG, "Unknown test command 0x%02X", cmd);
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
