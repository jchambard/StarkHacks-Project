#ifndef TEST_MODE_H
#define TEST_MODE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * Start the dedicated test task on Core 0 at priority PRIORITY_TEST.
 * The task blocks on xTaskNotifyWait(); UDP RX task posts the CommandType
 * value (CMD_TEST_BUZZERS, CMD_TEST_DISPLAY, CMD_TEST_MICS, CMD_TEST_IMU)
 * via xTaskNotify(). One test runs at a time; the task handle must be stored
 * in the global test_task_handle for the RX task to reference.
 *
 * @param audio_queue  Passed to test_mode_mics for reading captured audio
 */
void test_mode_task_start(QueueHandle_t audio_queue);

// ── Individual test implementations (called from the test task) ───────────────

/** Cycle each buzzer 0–7 at 50% intensity for 1 second each. */
void test_mode_buzzers(void);

/** Rotate a warning icon around 8 ring positions, 500ms per position. */
void test_mode_display(void);

/** Print per-channel RMS to serial for 10 seconds. */
void test_mode_mics(QueueHandle_t audio_queue);

/** Print raw IMU yaw rate to serial every 100ms for 10 seconds. */
void test_mode_imu(void);

#endif // TEST_MODE_H
