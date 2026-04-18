#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config.h"
#include "calibration.h"
#include "audio_capture.h"

static const char *TAG = "CAL";
static int32_t cal_offsets[NUM_CHANNELS] = {0};

esp_err_t calibration_init(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No calibration found in NVS, using defaults (0)");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    char key[16];
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        snprintf(key, sizeof(key), NVS_KEY_OFFSET_FMT, ch);
        int32_t val = 0;
        // NVS stores as i32 but the actual value fits int8; read as i32
        err = nvs_get_i32(nvs, key, &val);
        if (err == ESP_OK) {
            cal_offsets[ch] = val;
        } else {
            cal_offsets[ch] = 0;
        }
        ESP_LOGI(TAG, "ch%d offset = %ld", ch, (long)cal_offsets[ch]);
    }
    nvs_close(nvs);

    audio_set_calibration_offsets(cal_offsets);
    return ESP_OK;
}

void calibration_get_offsets(int32_t offsets[4])
{
    memcpy(offsets, cal_offsets, sizeof(cal_offsets));
}

static int32_t clamp_offset(int32_t v)
{
    if (v >  MAX_CAL_OFFSET) return  MAX_CAL_OFFSET;
    if (v < -MAX_CAL_OFFSET) return -MAX_CAL_OFFSET;
    return v;
}

static esp_err_t save_offsets(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open for write failed: %s", esp_err_to_name(err));
        return err;
    }

    char key[16];
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        snprintf(key, sizeof(key), NVS_KEY_OFFSET_FMT, ch);
        err = nvs_set_i32(nvs, key, cal_offsets[ch]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NVS set ch%d failed: %s", ch, esp_err_to_name(err));
            nvs_close(nvs);
            return err;
        }
    }
    err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t calibration_set_offsets(const int32_t offsets[4])
{
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        cal_offsets[ch] = clamp_offset(offsets[ch]);
    }
    audio_set_calibration_offsets(cal_offsets);
    return save_offsets();
}

esp_err_t calibration_apply_single(int ch, int8_t offset)
{
    if (ch < 0 || ch >= NUM_CHANNELS) {
        ESP_LOGW(TAG, "Invalid channel %d", ch);
        return ESP_ERR_INVALID_ARG;
    }
    cal_offsets[ch] = clamp_offset((int32_t)offset);
    ESP_LOGI(TAG, "ch%d offset set to %ld", ch, (long)cal_offsets[ch]);
    audio_set_calibration_offsets(cal_offsets);
    return save_offsets();
}

void calibration_start_calibration(void)
{
    ESP_LOGI(TAG, "Calibration mode started — send impulse then CMD_SET_CALIBRATION");
}
