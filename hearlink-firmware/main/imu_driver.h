#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// MPU-6050 I²C registers
#define MPU6050_ADDR        0x68
#define MPU6050_WHO_AM_I    0x75
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_GYRO_ZOUT_H 0x47

/**
 * Add MPU-6050 to the shared I²C bus and wake it from sleep.
 * Sets gyro range to ±250°/s (131 LSB/°/s sensitivity).
 * Verifies WHO_AM_I == 0x68; returns ESP_FAIL if mismatch.
 * @param i2c_bus Shared I²C master bus handle (created in main)
 */
esp_err_t imu_init(i2c_master_bus_handle_t i2c_bus);

/**
 * Read raw gyro Z-axis (yaw rate).
 * Reads GYRO_ZOUT_H and GYRO_ZOUT_L, combines big-endian → int16.
 * @return Raw LSB value; multiply by (1/131) for °/s
 */
int16_t imu_read_yaw_rate(void);

/**
 * Start IMU polling task on Core 0 at priority PRIORITY_IMU_POLL.
 * Polls at 100Hz, updates the global latest_imu_yaw_rate under imu_mutex.
 * Requires imu_init() to have succeeded.
 */
void imu_poll_start(void);

#endif // IMU_DRIVER_H
