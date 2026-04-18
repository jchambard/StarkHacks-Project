#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "config.h"

// One audio frame: 4 channels × 160 samples (10ms @ 16kHz)
typedef struct {
    int32_t ch0[SAMPLES_PER_BUFFER];
    int32_t ch1[SAMPLES_PER_BUFFER];
    int32_t ch2[SAMPLES_PER_BUFFER];
    int32_t ch3[SAMPLES_PER_BUFFER];
} AudioBuffer;

/**
 * Initialize I²S0 (mics 0+1) and I²S1 (mics 2+3) in stereo RX mode.
 * Both controllers: 16kHz, 32-bit, MSB-justified.
 * Must be called before audio_capture_start().
 */
esp_err_t audio_capture_init(void);

/**
 * Launch the audio capture task on Core 1 at priority PRIORITY_AUDIO_CAPTURE.
 * Task deinterleaves stereo from both I²S buses into 4 mono channels and
 * posts AudioBuffer to audio_queue.
 * @param audio_queue Target queue (depth ≥ 10, item size = sizeof(AudioBuffer))
 */
void audio_capture_start(QueueHandle_t audio_queue);

/**
 * Update per-channel calibration offsets (sample delays).
 * Positive offset delays a channel; negative advances it.
 * Values are clamped to ±MAX_CAL_OFFSET at apply time.
 * Thread-safe: may be called from any task.
 * @param offsets Array of 4 int32_t offsets, one per channel
 */
void audio_set_calibration_offsets(int32_t offsets[4]);

/**
 * Return the cumulative number of frames dropped due to a full audio_queue.
 * Resets to 0 each time it is read.
 */
uint32_t audio_get_dropped_frames(void);

#endif // AUDIO_CAPTURE_H
