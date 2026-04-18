#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include "esp_err.h"

/**
 * Initialize NVS and load calibration offsets.
 * Defaults all offsets to 0 if no saved calibration exists.
 * Applies loaded offsets to the audio capture module.
 */
esp_err_t calibration_init(void);

/**
 * Get current calibration offsets.
 * @param offsets Output array of 4 int32_t values (one per channel)
 */
void calibration_get_offsets(int32_t offsets[4]);

/**
 * Set and save all calibration offsets to NVS.
 * @param offsets Array of 4 int32_t values; clamped to ±MAX_CAL_OFFSET
 */
esp_err_t calibration_set_offsets(const int32_t offsets[4]);

/**
 * Set offset for a single channel and save to NVS.
 * Used by CMD_SET_CALIBRATION (host sends one channel at a time).
 * @param ch     Channel index 0–3
 * @param offset Signed sample offset; clamped to ±MAX_CAL_OFFSET
 */
esp_err_t calibration_apply_single(int ch, int8_t offset);

/**
 * Log a message indicating calibration mode has started.
 * Host is expected to send an impulse and follow up with CMD_SET_CALIBRATION.
 */
void calibration_start_calibration(void);

#endif // CALIBRATION_H
