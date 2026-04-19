#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "config.h"
#include "imu_driver.h"

// Declared in main.c; shared with network/udp_tx
extern volatile int16_t latest_imu_yaw_rate;
extern SemaphoreHandle_t imu_mutex;

static const char *TAG = "IMU";
static i2c_master_dev_handle_t imu_dev = NULL;
static bool imu_ready = false;

// ── Low-level register helpers ───────────────────────────────────────────────

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(imu_dev, buf, sizeof(buf), pdMS_TO_TICKS(10));
}

static esp_err_t mpu_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(imu_dev, &reg, 1,
                                       data, len, pdMS_TO_TICKS(10));
}

// ── Public API ───────────────────────────────────────────────────────────────

esp_err_t imu_init(i2c_master_bus_handle_t i2c_bus)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &imu_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MPU-6050 to I²C bus: %s", esp_err_to_name(err));
        return err;
    }

    // Verify WHO_AM_I
    uint8_t who_am_i = 0;
    err = mpu_read_regs(MPU6050_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(err));
        return err;
    }
    if (who_am_i != 0x68) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x68", who_am_i);
        return ESP_FAIL;
    }

    // Wake from sleep
    err = mpu_write_reg(MPU6050_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PWR_MGMT_1 write failed: %s", esp_err_to_name(err));
        return err;
    }

    // Set gyro range to ±250°/s (GYRO_CONFIG[4:3] = 00)
    err = mpu_write_reg(MPU6050_GYRO_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GYRO_CONFIG write failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MPU-6050 initialized (±250°/s, 131 LSB/°/s)");
    imu_ready = true;
    return ESP_OK;
}

int16_t imu_read_yaw_rate(void)
{
    uint8_t data[2] = {0};
    esp_err_t err = mpu_read_regs(MPU6050_GYRO_ZOUT_H, data, 2);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Gyro Z read failed: %s", esp_err_to_name(err));
        return 0;
    }
    // Big-endian: ZOUT_H at [0], ZOUT_L at [1]
    return (int16_t)((uint16_t)data[0] << 8 | data[1]);
}

// ── Poll task ────────────────────────────────────────────────────────────────

static void imu_poll_task(void *arg)
{
    ESP_LOGI(TAG, "IMU poll task running (100Hz)");
    for (;;) {
        if (imu_ready) {
            int16_t yaw = imu_read_yaw_rate();
            xSemaphoreTake(imu_mutex, portMAX_DELAY);
            latest_imu_yaw_rate = yaw;
            xSemaphoreGive(imu_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void imu_poll_start(void)
{
    xTaskCreatePinnedToCore(imu_poll_task, "imu_poll",
                            STACK_SIZE_IMU, NULL,
                            PRIORITY_IMU_POLL, NULL, 0);
}
