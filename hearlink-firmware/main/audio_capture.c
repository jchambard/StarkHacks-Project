#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "config.h"
#include "audio_capture.h"

static const char *TAG = "AUDIO";

// I²S channel handles (one per controller)
static i2s_chan_handle_t rx0 = NULL;  // I²S0: mics 0+1
static i2s_chan_handle_t rx1 = NULL;  // I²S1: mics 2+3

// Calibration ring buffer: per channel, size = SAMPLES_PER_BUFFER + MAX_CAL_OFFSET*2
#define RING_SIZE   (SAMPLES_PER_BUFFER + MAX_CAL_OFFSET * 2)

static int32_t ring[NUM_CHANNELS][RING_SIZE];
static int     ring_head[NUM_CHANNELS];  // write position

// Protected by a mutex so audio_set_calibration_offsets() is thread-safe
static SemaphoreHandle_t cal_mutex;
static int32_t cal_offsets[NUM_CHANNELS] = {0};

// Dropped frame counter (atomic-ish: only written by one task)
static volatile uint32_t dropped_frames = 0;

// ── Initialization ──────────────────────────────────────────────────────────

static esp_err_t init_i2s_controller(i2s_port_t port,
                                      int bclk, int lrclk, int sd,
                                      i2s_chan_handle_t *out_handle)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(port, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 4;
    chan_cfg.dma_frame_num = SAMPLES_PER_BUFFER;
    chan_cfg.auto_clear    = true;

    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, out_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel port %d failed: %s", port, esp_err_to_name(err));
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                     I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = bclk,
            .ws   = lrclk,
            .dout = I2S_GPIO_UNUSED,
            .din  = sd,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(*out_handle, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode port %d failed: %s",
                 port, esp_err_to_name(err));
    }
    return err;
}

esp_err_t audio_capture_init(void)
{
    cal_mutex = xSemaphoreCreateMutex();
    configASSERT(cal_mutex);

    memset(ring,      0, sizeof(ring));
    memset(ring_head, 0, sizeof(ring_head));

    esp_err_t err;
    err = init_i2s_controller(I2S_NUM_0,
                               I2S0_BCLK_PIN, I2S0_LRCLK_PIN, I2S0_SD_PIN,
                               &rx0);
    if (err != ESP_OK) return err;

    err = init_i2s_controller(I2S_NUM_1,
                               I2S1_BCLK_PIN, I2S1_LRCLK_PIN, I2S1_SD_PIN,
                               &rx1);
    return err;
}

// ── Calibration offset application ─────────────────────────────────────────

void audio_set_calibration_offsets(int32_t offsets[4])
{
    xSemaphoreTake(cal_mutex, portMAX_DELAY);
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        int32_t v = offsets[ch];
        if (v >  MAX_CAL_OFFSET) v =  MAX_CAL_OFFSET;
        if (v < -MAX_CAL_OFFSET) v = -MAX_CAL_OFFSET;
        cal_offsets[ch] = v;
    }
    xSemaphoreGive(cal_mutex);
}

uint32_t audio_get_dropped_frames(void)
{
    uint32_t v = dropped_frames;
    dropped_frames = 0;
    return v;
}

// ── Ring-buffer helper ───────────────────────────────────────────────────────
// Write one sample into channel's ring, read back sample that is 'offset'
// positions behind the write head (positive offset = delay; negative = advance).
static inline int32_t ring_push_read(int ch, int32_t sample, int32_t offset)
{
    ring[ch][ring_head[ch]] = sample;
    // Read position: offset positions behind current write head
    // Positive offset: read older data (delay)
    // Negative offset: read newer ring slot (advance, only valid if pre-filled)
    int read_pos = ((int)ring_head[ch] - (int)offset + RING_SIZE) % RING_SIZE;
    ring_head[ch] = (ring_head[ch] + 1) % RING_SIZE;
    return ring[ch][read_pos];
}

// ── Capture task ─────────────────────────────────────────────────────────────

// Raw DMA buffer for one 160-sample stereo read (2 channels × 160 × 4 bytes)
#define DMA_BUF_BYTES  (SAMPLES_PER_BUFFER * 2 * sizeof(int32_t))

static void audio_capture_task(void *arg)
{
    QueueHandle_t q = (QueueHandle_t)arg;

    // Interleaved stereo raw buffers from DMA
    static int32_t raw0[SAMPLES_PER_BUFFER * 2];  // I²S0: L0, R1, L0, R1, ...
    static int32_t raw1[SAMPLES_PER_BUFFER * 2];  // I²S1: L2, R3, L2, R3, ...

    AudioBuffer buf;
    size_t bytes_read;

    // Enable both I²S controllers back-to-back to minimise startup skew
    portDISABLE_INTERRUPTS();
    ESP_ERROR_CHECK(i2s_channel_enable(rx0));
    ESP_ERROR_CHECK(i2s_channel_enable(rx1));
    portENABLE_INTERRUPTS();

    ESP_LOGI(TAG, "Audio capture task running");

    for (;;) {
        // Block on I²S0 first (both DMA interrupts will fire nearly together)
        esp_err_t err0 = i2s_channel_read(rx0, raw0, DMA_BUF_BYTES,
                                           &bytes_read, pdMS_TO_TICKS(20));
        esp_err_t err1 = i2s_channel_read(rx1, raw1, DMA_BUF_BYTES,
                                           &bytes_read, pdMS_TO_TICKS(20));

        if (err0 != ESP_OK || err1 != ESP_OK) {
            ESP_LOGW(TAG, "I²S read timeout/error (0=%s 1=%s)",
                     esp_err_to_name(err0), esp_err_to_name(err1));
            continue;
        }

        // Snapshot calibration offsets under mutex
        int32_t offsets[NUM_CHANNELS];
        xSemaphoreTake(cal_mutex, portMAX_DELAY);
        memcpy(offsets, cal_offsets, sizeof(offsets));
        xSemaphoreGive(cal_mutex);

        // Deinterleave stereo and apply calibration ring buffer
        for (int i = 0; i < SAMPLES_PER_BUFFER; i++) {
            buf.ch0[i] = ring_push_read(0, raw0[i * 2 + 0], offsets[0]);  // I²S0 Left
            buf.ch1[i] = ring_push_read(1, raw0[i * 2 + 1], offsets[1]);  // I²S0 Right
            buf.ch2[i] = ring_push_read(2, raw1[i * 2 + 0], offsets[2]);  // I²S1 Left
            buf.ch3[i] = ring_push_read(3, raw1[i * 2 + 1], offsets[3]);  // I²S1 Right
        }

        // Non-blocking send; drop oldest if queue is full
        if (xQueueSend(q, &buf, 0) != pdTRUE) {
            AudioBuffer discard;
            xQueueReceive(q, &discard, 0);
            xQueueSend(q, &buf, 0);
            dropped_frames++;
        }
    }
}

void audio_capture_start(QueueHandle_t audio_queue)
{
    xTaskCreatePinnedToCore(audio_capture_task, "audio_cap",
                            STACK_SIZE_AUDIO, audio_queue,
                            PRIORITY_AUDIO_CAPTURE, NULL, 1);
}
