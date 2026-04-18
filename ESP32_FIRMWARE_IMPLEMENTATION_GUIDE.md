# ESP32-S3 Firmware Implementation Guide for HearLink

## Overview

This guide provides complete specifications for implementing the ESP32-S3 firmware for the HearLink spatial audio awareness system. The firmware acts as a pure I/O node, handling:

1. 4-channel I²S audio capture (16kHz, 32-bit)
2. MPU-6050 IMU polling (yaw rate only)
3. UDP streaming to host (audio + IMU data)
4. UDP command reception from host
5. 8-channel buzzer control via LEDC
6. SSD1306 OLED display updates

All signal processing, beamforming, and classification occurs on the host (Python).

---

## Project Structure

```
hearlink-firmware/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   ├── main.c
│   ├── config.h
│   ├── audio_capture.c
│   ├── audio_capture.h
│   ├── imu_driver.c
│   ├── imu_driver.h
│   ├── network.c
│   ├── network.h
│   ├── buzzer_control.c
│   ├── buzzer_control.h
│   ├── display.c
│   ├── display.h
│   ├── calibration.c
│   ├── calibration.h
│   ├── test_mode.c
│   └── test_mode.h
└── test/
    ├── test_audio.c
    ├── test_imu.c
    ├── test_network.c
    └── test_integration.c
```

---

## Hardware Configuration

### GPIO Pin Map

| Function | GPIOs | Notes |
|----------|-------|-------|
| I²S0 BCLK | GPIO4 | Mics 0-1 bit clock |
| I²S0 LRCLK | GPIO5 | Mics 0-1 L/R clock |
| I²S0 SD | GPIO6 | Mics 0-1 data |
| I²S1 BCLK | GPIO7 | Mics 2-3 bit clock |
| I²S1 LRCLK | GPIO15 | Mics 2-3 L/R clock |
| I²S1 SD | GPIO16 | Mics 2-3 data |
| I²C SDA | GPIO8 | IMU + OLED data |
| I²C SCL | GPIO9 | IMU + OLED clock |
| PWM Buzzer 0 | GPIO1 | 0° (front-right) |
| PWM Buzzer 1 | GPIO2 | 45° |
| PWM Buzzer 2 | GPIO10 | 90° (back-right) |
| PWM Buzzer 3 | GPIO11 | 135° |
| PWM Buzzer 4 | GPIO12 | 180° (back-left) |
| PWM Buzzer 5 | GPIO13 | 225° |
| PWM Buzzer 6 | GPIO14 | 270° (front-left) |
| PWM Buzzer 7 | GPIO17 | 315° |
| Status LED | GPIO48 | Built-in RGB LED |

### I²C Devices

| Device | Address | Bus Speed |
|--------|---------|-----------|
| MPU-6050 | 0x68 | 400kHz |
| SSD1306 OLED | 0x3C | 400kHz |

---

## Network Configuration

### WiFi SoftAP

```c
#define WIFI_SSID      "HearLink"
#define WIFI_PASSWORD  "hearlink2025"
#define AP_IP          "192.168.4.1"
#define AP_GATEWAY     "192.168.4.1"
#define AP_NETMASK     "255.255.255.0"
```

### UDP Ports

```c
#define UDP_PORT_TX    5005  // ESP32 → Host (audio + IMU)
#define UDP_PORT_RX    5006  // Host → ESP32 (commands)
```

---

## Protocol Definitions

### Outbound Packet (ESP32 → Host)

```c
#define SAMPLES_PER_BUFFER 160  // 10ms @ 16kHz

typedef struct __attribute__((packed)) {
    uint32_t seq;                       // Packet sequence number
    uint32_t timestamp_us;              // Microsecond timestamp
    int16_t  imu_yaw_rate;              // Raw gyro Z-axis (LSB/°/s per datasheet)
    uint8_t  pad[2];                    // Padding for alignment
    int32_t  audio[4][SAMPLES_PER_BUFFER]; // 4 channels × 160 samples
} AudioIMUPacket;

// Total size: 8 + 2 + 2 + (4 * 160 * 4) = 2572 bytes per packet
// At 16kHz with 160 samples: 100 packets/sec → ~257 kB/s (~2.06 Mbps)
```

### Inbound Commands (Host → ESP32)

```c
typedef enum {
    CMD_DISPLAY_ICON = 0x01,
    CMD_FIRE_BUZZER  = 0x02,
    CMD_CLEAR_ALL    = 0x03,
    CMD_CALIBRATE    = 0x04,
    CMD_SET_CALIBRATION = 0x05,
    CMD_TEST_MODE    = 0x10,
    CMD_TEST_BUZZERS = 0x11,
    CMD_TEST_DISPLAY = 0x12,
    CMD_TEST_MICS    = 0x13,
    CMD_TEST_IMU     = 0x14,
} CommandType;

typedef enum {
    CATEGORY_DANGEROUS = 0,
    CATEGORY_SOCIAL    = 1,
    CATEGORY_AMBIENT   = 2,
} SoundCategory;

typedef struct __attribute__((packed)) {
    uint8_t type;       // CommandType
    uint8_t direction;  // 0-255 maps to 0-360°
    uint8_t category;   // SoundCategory
    uint8_t pattern;    // Pattern ID (buzzer or icon style)
} Command;

// For CMD_SET_CALIBRATION, use direction field as channel index,
// and pattern field as offset value (signed, reinterpret as int8_t)
```

---

## FreeRTOS Task Architecture

### Task Allocation (6 tasks on 2 cores)

#### Core 1 (Audio-critical)
- **Task: audio_capture_task** (Priority 5)
  - Reads I²S0 and I²S1 DMA buffers
  - Deinterleaves stereo to 4 mono channels
  - Posts 160-sample buffers to audio queue
  - Never blocks on full queue (drops oldest)

- **Task: udp_tx_task** (Priority 4)
  - Polls audio queue (blocking, timeout 20ms)
  - Combines audio + latest IMU reading
  - Sends AudioIMUPacket via UDP
  - Increments sequence number

#### Core 0 (Control & I/O)
- **Task: udp_rx_task** (Priority 3)
  - Blocking recvfrom on UDP port 5006
  - Parses Command struct
  - Dispatches to appropriate handler

- **Task: imu_poll_task** (Priority 3)
  - Polls MPU-6050 at 100Hz (10ms intervals)
  - Reads GYRO_ZOUT register
  - Stores latest yaw_rate in shared variable (atomic or mutex)

- **Task: buzzer_control_task** (Priority 2)
  - Receives commands from buzzer queue
  - Updates LEDC duty cycles
  - Handles buzzer patterns (continuous, pulse, etc.)

- **Task: display_update_task** (Priority 1)
  - Receives display commands from display queue
  - Renders icons on SSD1306 using u8g2
  - Updates at ~10Hz or on-demand

### Queues

```c
QueueHandle_t audio_queue;      // audio_capture → udp_tx (10 buffers deep)
QueueHandle_t buzzer_queue;     // udp_rx → buzzer_control (5 commands deep)
QueueHandle_t display_queue;    // udp_rx → display_update (5 commands deep)
```

### Shared Variables

```c
// Protected by mutex or atomic operations
volatile int16_t latest_imu_yaw_rate;
SemaphoreHandle_t imu_mutex;
```

---

## Module Specifications

### 1. config.h

**Purpose:** Central configuration header

```c
#ifndef CONFIG_H
#define CONFIG_H

// Audio
#define SAMPLE_RATE         16000
#define BITS_PER_SAMPLE     32
#define SAMPLES_PER_BUFFER  160
#define NUM_CHANNELS        4

// I²S GPIO
#define I2S0_BCLK_PIN       4
#define I2S0_LRCLK_PIN      5
#define I2S0_SD_PIN         6
#define I2S1_BCLK_PIN       7
#define I2S1_LRCLK_PIN      15
#define I2S1_SD_PIN         16

// I²C GPIO
#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9
#define I2C_FREQ_HZ         400000

// Buzzer GPIO (8 channels)
#define BUZZER_PINS { 1, 2, 10, 11, 12, 13, 14, 17 }
#define NUM_BUZZERS         8
#define BUZZER_PWM_FREQ     400  // Hz
#define BUZZER_PWM_RES      LEDC_TIMER_10_BIT

// Status LED
#define STATUS_LED_PIN      48

// WiFi
#define WIFI_SSID           "HearLink"
#define WIFI_PASSWORD       "hearlink2025"
#define AP_IP_ADDR          "192.168.4.1"

// UDP
#define UDP_PORT_TX         5005
#define UDP_PORT_RX         5006
#define UDP_BUFFER_SIZE     4096

// NVS namespace for calibration
#define NVS_NAMESPACE       "cal"
#define NVS_KEY_OFFSET_FMT  "offset%d"  // offset0, offset1, ...

// Task priorities
#define PRIORITY_AUDIO_CAPTURE  5
#define PRIORITY_UDP_TX         4
#define PRIORITY_UDP_RX         3
#define PRIORITY_IMU_POLL       3
#define PRIORITY_BUZZER         2
#define PRIORITY_DISPLAY        1

// Task stack sizes
#define STACK_SIZE_AUDIO        4096
#define STACK_SIZE_UDP_TX       4096
#define STACK_SIZE_UDP_RX       3072
#define STACK_SIZE_IMU          2048
#define STACK_SIZE_BUZZER       2048
#define STACK_SIZE_DISPLAY      3072

#endif // CONFIG_H
```

---

### 2. audio_capture.h / audio_capture.c

**Purpose:** I²S audio capture from 4 INMP441 mics

#### audio_capture.h

```c
#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Buffer structure for one audio frame (4 channels × 160 samples)
typedef struct {
    int32_t ch0[SAMPLES_PER_BUFFER];
    int32_t ch1[SAMPLES_PER_BUFFER];
    int32_t ch2[SAMPLES_PER_BUFFER];
    int32_t ch3[SAMPLES_PER_BUFFER];
} AudioBuffer;

/**
 * Initialize I²S0 and I²S1 in standard stereo mode
 * - I²S0: Mics 0 (left) and 1 (right)
 * - I²S1: Mics 2 (left) and 3 (right)
 * Both at 16kHz, 32-bit samples
 */
esp_err_t audio_capture_init(void);

/**
 * Start the audio capture task (Core 1, priority 5)
 * @param audio_queue FreeRTOS queue to post captured buffers
 */
void audio_capture_start(QueueHandle_t audio_queue);

/**
 * Apply calibration offsets to align channels
 * @param offsets Array of 4 sample offsets (can be negative)
 */
void audio_set_calibration_offsets(int32_t offsets[4]);

#endif // AUDIO_CAPTURE_H
```

#### audio_capture.c Implementation Requirements

1. **I²S Initialization:**
   - Use `i2s_std_gpio_config_t` and `i2s_std_config_t` from ESP-IDF 5.2+ driver
   - Configure both I²S0 and I²S1 in parallel stereo mode
   - DMA buffer: 160 samples per channel, ping-pong (2 buffers)
   - Enable auto-clear on underflow

2. **audio_capture_task:**
   - Read from both I²S channels using `i2s_channel_read()`
   - Deinterleave stereo (L/R) into 4 mono channels
   - Apply calibration offsets (circular buffer logic)
   - Post AudioBuffer to queue with `xQueueSend(..., 0)` (non-blocking)
   - On queue full: drop oldest using `xQueueReceive()` then retry send

3. **Calibration offset logic:**
   - Store offsets in static array `static int32_t cal_offsets[4] = {0}`
   - Implement circular buffer shifts when offset != 0
   - Example: if `cal_offsets[2] = -5`, buffer ch2 by 5 samples before output

---

### 3. imu_driver.h / imu_driver.c

**Purpose:** MPU-6050 gyroscope reading (yaw rate only)

#### imu_driver.h

```c
#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>
#include "driver/i2c_master.h"

#define MPU6050_ADDR        0x68
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_GYRO_ZOUT_H 0x47

/**
 * Initialize MPU-6050 via I²C
 * - Wake from sleep
 * - Set gyro range to ±250°/s (sensitivity 131 LSB/°/s)
 */
esp_err_t imu_init(i2c_master_bus_handle_t i2c_bus);

/**
 * Read raw gyro Z-axis value (yaw rate)
 * @return int16_t raw value (LSB, convert using 131 LSB/°/s)
 */
int16_t imu_read_yaw_rate(void);

/**
 * Start IMU polling task (Core 0, priority 3)
 * Polls at 100Hz, updates global latest_imu_yaw_rate
 */
void imu_poll_start(void);

#endif // IMU_DRIVER_H
```

#### imu_driver.c Implementation Requirements

1. **Initialization:**
   - Write 0x00 to PWR_MGMT_1 to wake device
   - Write 0x00 to GYRO_CONFIG for ±250°/s range
   - Verify WHO_AM_I register (0x75) returns 0x68

2. **imu_read_yaw_rate:**
   - Read 2 bytes from GYRO_ZOUT_H (0x47) and GYRO_ZOUT_L (0x48)
   - Combine into signed int16_t (big-endian)
   - Return raw value (host will handle scaling and bias)

3. **imu_poll_task:**
   - Every 10ms: call `imu_read_yaw_rate()`
   - Lock `imu_mutex`, update `latest_imu_yaw_rate`, unlock
   - Use `vTaskDelay(pdMS_TO_TICKS(10))`

---

### 4. network.h / network.c

**Purpose:** WiFi SoftAP and UDP socket management

#### network.h

```c
#ifndef NETWORK_H
#define NETWORK_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * Initialize WiFi in SoftAP mode with static IP
 */
esp_err_t network_init(void);

/**
 * Start UDP TX task (Core 1, priority 4)
 * Reads from audio_queue, combines with IMU, sends to host
 */
void udp_tx_start(QueueHandle_t audio_queue);

/**
 * Start UDP RX task (Core 0, priority 3)
 * Receives commands from host, dispatches to buzzer/display queues
 */
void udp_rx_start(QueueHandle_t buzzer_queue, QueueHandle_t display_queue);

/**
 * Get host IP address (last connected client)
 */
const char* network_get_host_ip(void);

#endif // NETWORK_H
```

#### network.c Implementation Requirements

1. **network_init:**
   - Initialize TCP/IP stack with `esp_netif_init()`
   - Create AP netif with `esp_netif_create_default_wifi_ap()`
   - Set static IP: 192.168.4.1 / 255.255.255.0
   - Configure WiFi: SSID "HearLink", password "hearlink2025"
   - Max 4 stations, channel 1, auth WPA2_PSK

2. **udp_tx_task:**
   - Create UDP socket bound to port 5005
   - Block on `xQueueReceive(audio_queue, ...)` with 20ms timeout
   - On receive: populate AudioIMUPacket struct
     - Lock imu_mutex, read `latest_imu_yaw_rate`, unlock
     - Increment sequence number
     - Set `timestamp_us = esp_timer_get_time()`
   - `sendto()` to host IP (stored from RX task's recvfrom)
   - On socket error: log and retry after delay

3. **udp_rx_task:**
   - Create UDP socket bound to port 5006
   - Blocking `recvfrom()` to receive Command struct
   - Store client IP as host IP (for TX task)
   - Dispatch based on `cmd.type`:
     - `CMD_DISPLAY_ICON`, `CMD_CLEAR_ALL` → display_queue
     - `CMD_FIRE_BUZZER` → buzzer_queue
     - `CMD_CALIBRATE`, `CMD_SET_CALIBRATION` → call calibration module
     - `CMD_TEST_*` → call test_mode functions
   - On malformed packet: log warning, continue

---

### 5. buzzer_control.h / buzzer_control.c

**Purpose:** LEDC PWM control of 8 buzzers

#### buzzer_control.h

```c
#ifndef BUZZER_CONTROL_H
#define BUZZER_CONTROL_H

#include <stdint.h>
#include "freertos/queue.h"

typedef enum {
    PATTERN_OFF        = 0,
    PATTERN_CONTINUOUS = 1,
    PATTERN_PULSE_FAST = 2,  // 200ms on, 200ms off
    PATTERN_PULSE_SLOW = 3,  // 500ms on, 500ms off
    PATTERN_DOUBLE_TAP = 4,  // 100ms on, 100ms off, 100ms on, then off
} BuzzerPattern;

typedef struct {
    uint8_t buzzer_mask;  // Bitmask of buzzers to activate (bit 0 = buzzer 0, etc.)
    BuzzerPattern pattern;
    uint8_t intensity;    // 0-100 (maps to LEDC duty cycle)
} BuzzerCommand;

/**
 * Initialize 8 LEDC channels for buzzers
 */
esp_err_t buzzer_init(void);

/**
 * Start buzzer control task (Core 0, priority 2)
 */
void buzzer_control_start(QueueHandle_t buzzer_queue);

/**
 * Set buzzer state immediately (for testing)
 */
void buzzer_set(uint8_t buzzer_index, uint8_t intensity);

#endif // BUZZER_CONTROL_H
```

#### buzzer_control.c Implementation Requirements

1. **buzzer_init:**
   - Configure LEDC timer: 400Hz, 10-bit resolution
   - Initialize 8 LEDC channels, one per buzzer GPIO
   - Set all duty cycles to 0 (off)

2. **buzzer_control_task:**
   - Poll `buzzer_queue` with blocking receive
   - On command: decode buzzer_mask
   - For each bit set in mask:
     - Set LEDC duty cycle based on intensity (0-100 → 0-1023)
     - Start pattern timer if needed (using FreeRTOS software timers)
   - Pattern handling:
     - `PATTERN_CONTINUOUS`: Set duty, leave on
     - `PATTERN_PULSE_*`: Toggle duty on/off at intervals
     - `PATTERN_DOUBLE_TAP`: Duty on → off → on → off sequence
     - `PATTERN_OFF`: Set duty to 0

3. **buzzer_set (test function):**
   - Directly set LEDC duty for given buzzer index
   - Used by test mode

---

### 6. display.h / display.c

**Purpose:** SSD1306 OLED control using u8g2

#### display.h

```c
#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "freertos/queue.h"

typedef enum {
    ICON_TRIANGLE_WARNING = 0,  // Dangerous
    ICON_SPEECH_BUBBLE    = 1,  // Social
    ICON_MUSIC_NOTE       = 2,  // Ambient
    ICON_CAR              = 3,
    ICON_SIREN            = 4,
    ICON_PERSON           = 5,
} IconType;

typedef struct {
    uint8_t direction;  // 0-255 (maps to angle on ring)
    IconType icon;
    uint8_t category;   // For color/blink decision
    uint8_t active;     // 1 = show, 0 = clear this slot
} DisplayCommand;

/**
 * Initialize SSD1306 OLED via I²C using u8g2
 */
esp_err_t display_init(i2c_master_bus_handle_t i2c_bus);

/**
 * Start display update task (Core 0, priority 1)
 */
void display_update_start(QueueHandle_t display_queue);

/**
 * Clear all icons from display
 */
void display_clear(void);

/**
 * Show test pattern (for TEST_DISPLAY mode)
 */
void display_test_pattern(void);

#endif // DISPLAY_H
```

#### display.c Implementation Requirements

1. **display_init:**
   - Initialize u8g2 library with SSD1306 128x64 I²C
   - Set font: u8g2_font_6x10_tr (or similar small font)
   - Draw initial ring outline (circle at center)

2. **display_update_task:**
   - Poll `display_queue` with 100ms timeout
   - On command: update internal icon list (max 3 active icons)
   - Redraw screen:
     - Clear buffer
     - Draw ring (circle)
     - For each active icon:
       - Convert direction (0-255) to angle (0-360°)
       - Calculate (x, y) position on ring
       - Draw icon bitmap at that position
     - Send buffer to OLED

3. **Icon bitmaps:**
   - Define small 8x8 or 12x12 bitmaps for each icon type
   - Store as byte arrays in PROGMEM

4. **display_test_pattern:**
   - Rotate a single icon around the ring at 45° steps
   - Called by test mode

---

### 7. calibration.h / calibration.c

**Purpose:** NVS storage and management of I²S channel offsets

#### calibration.h

```c
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

/**
 * Initialize NVS and load calibration offsets
 * If no calibration exists, offsets default to 0
 */
esp_err_t calibration_init(void);

/**
 * Get current calibration offsets
 * @param offsets Output array of 4 int32_t values
 */
void calibration_get_offsets(int32_t offsets[4]);

/**
 * Set and save calibration offsets to NVS
 * @param offsets Array of 4 int32_t values
 */
esp_err_t calibration_set_offsets(const int32_t offsets[4]);

/**
 * Enter calibration mode (future: trigger impulse capture)
 * For now, just a placeholder that logs a message
 */
void calibration_start_calibration(void);

#endif // CALIBRATION_H
```

#### calibration.c Implementation Requirements

1. **calibration_init:**
   - Open NVS namespace "cal"
   - For each channel 0-3:
     - Try to read key "offset0", "offset1", etc.
     - If not found, default to 0
   - Store offsets in static array
   - Call `audio_set_calibration_offsets()`

2. **calibration_set_offsets:**
   - Write each offset to NVS with key "offset%d"
   - Commit NVS
   - Update static array
   - Call `audio_set_calibration_offsets()`

3. **calibration_start_calibration:**
   - Log: "Calibration mode started - send impulse and SET_CALIBRATION command"
   - (Host will compute offsets via cross-correlation and send CMD_SET_CALIBRATION)

---

### 8. test_mode.h / test_mode.c

**Purpose:** Built-in hardware tests triggered by host commands

#### test_mode.h

```c
#ifndef TEST_MODE_H
#define TEST_MODE_H

/**
 * Test each buzzer in sequence (1 second each)
 */
void test_mode_buzzers(void);

/**
 * Rotate an icon around the display ring
 */
void test_mode_display(void);

/**
 * Stream each mic channel independently (print stats to serial)
 */
void test_mode_mics(void);

/**
 * Print raw IMU yaw rate to serial for 10 seconds
 */
void test_mode_imu(void);

#endif // TEST_MODE_H
```

#### test_mode.c Implementation Requirements

1. **test_mode_buzzers:**
   - For buzzer 0 to 7:
     - Call `buzzer_set(i, 50)` (50% intensity)
     - `vTaskDelay(1000 ms)`
     - Call `buzzer_set(i, 0)`
   - Log: "Buzzer test complete"

2. **test_mode_display:**
   - For angle 0° to 315° in 45° steps:
     - Display icon at that angle
     - `vTaskDelay(500 ms)`
   - Clear display
   - Log: "Display test complete"

3. **test_mode_mics:**
   - For 10 seconds:
     - Read audio buffers from queue
     - Compute RMS or peak level for each channel
     - Print to serial: "Ch0: 1234 | Ch1: 2345 | Ch2: 3456 | Ch3: 4567"
   - Log: "Mic test complete"

4. **test_mode_imu:**
   - For 10 seconds:
     - Read `latest_imu_yaw_rate` every 100ms
     - Print to serial: "Yaw rate: -123 LSB"
   - Log: "IMU test complete"

---

### 9. main.c

**Purpose:** Entry point, initialization, task creation

#### main.c Structure

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "config.h"
#include "audio_capture.h"
#include "imu_driver.h"
#include "network.h"
#include "buzzer_control.h"
#include "display.h"
#include "calibration.h"

static const char *TAG = "MAIN";

// Global queues and mutex
QueueHandle_t audio_queue = NULL;
QueueHandle_t buzzer_queue = NULL;
QueueHandle_t display_queue = NULL;
SemaphoreHandle_t imu_mutex = NULL;

volatile int16_t latest_imu_yaw_rate = 0;

void app_main(void) {
    ESP_LOGI(TAG, "HearLink Firmware Starting...");
    
    // 1. Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 2. Load calibration
    ESP_ERROR_CHECK(calibration_init());
    
    // 3. Initialize I²C bus (shared by IMU and OLED)
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));
    
    // 4. Initialize hardware modules
    ESP_ERROR_CHECK(audio_capture_init());
    ESP_ERROR_CHECK(imu_init(i2c_bus));
    ESP_ERROR_CHECK(buzzer_init());
    ESP_ERROR_CHECK(display_init(i2c_bus));
    ESP_ERROR_CHECK(network_init());
    
    // 5. Create queues
    audio_queue = xQueueCreate(10, sizeof(AudioBuffer));
    buzzer_queue = xQueueCreate(5, sizeof(BuzzerCommand));
    display_queue = xQueueCreate(5, sizeof(DisplayCommand));
    imu_mutex = xSemaphoreCreateMutex();
    
    // 6. Start tasks
    // Core 1 tasks
    audio_capture_start(audio_queue);  // Priority 5, Core 1
    udp_tx_start(audio_queue);         // Priority 4, Core 1
    
    // Core 0 tasks
    udp_rx_start(buzzer_queue, display_queue);  // Priority 3, Core 0
    imu_poll_start();                           // Priority 3, Core 0
    buzzer_control_start(buzzer_queue);         // Priority 2, Core 0
    display_update_start(display_queue);        // Priority 1, Core 0
    
    ESP_LOGI(TAG, "All tasks started. System ready.");
    
    // Blink status LED to indicate ready
    gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(STATUS_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(STATUS_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

## CMakeLists.txt Configuration

### Root CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hearlink-firmware)
```

### main/CMakeLists.txt

```cmake
idf_component_register(
    SRCS 
        "main.c"
        "audio_capture.c"
        "imu_driver.c"
        "network.c"
        "buzzer_control.c"
        "display.c"
        "calibration.c"
        "test_mode.c"
    INCLUDE_DIRS "."
    REQUIRES 
        nvs_flash 
        esp_wifi 
        esp_netif
        driver
        esp_timer
        lwip
)

# Add u8g2 component (assuming it's in components/ directory)
# Or use idf_component_get_property to reference external component
```

---

## sdkconfig.defaults

```ini
# WiFi Configuration
CONFIG_ESP_WIFI_SOFTAP_SUPPORT=y
CONFIG_LWIP_DHCPS_MAX_STATION_NUM=4

# I²S Configuration
CONFIG_I2S_ISR_IRAM_SAFE=y
CONFIG_I2S_SUPPRESS_DEPRECATE_WARN=y

# FreeRTOS
CONFIG_FREERTOS_HZ=1000
CONFIG_FREERTOS_UNICORE=n

# Memory
CONFIG_ESP_MAIN_TASK_STACK_SIZE=4096
CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE=3072

# Logging
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
CONFIG_LOG_MAXIMUM_LEVEL_VERBOSE=y

# I²C
CONFIG_I2C_ENABLE_DEBUG_LOG=n

# NVS
CONFIG_NVS_ENCRYPTION=n

# Task watchdog
CONFIG_ESP_TASK_WDT_TIMEOUT_S=10
```

---

## Hardware Tests (test/ directory)

### test_audio.c

**Purpose:** Verify I²S capture and channel alignment

```c
/**
 * Test: Capture 1 second of audio, compute RMS per channel
 * Expected: All channels should have similar RMS (within 3dB) for white noise
 */
void test_audio_capture_rms(void);

/**
 * Test: Send impulse (clap), check cross-correlation peaks
 * Expected: All channels detect impulse within ±10 samples
 */
void test_audio_synchronization(void);
```

### test_imu.c

**Purpose:** Verify IMU reading and polling

```c
/**
 * Test: Read IMU while stationary, compute variance
 * Expected: Variance < threshold (indicates stable reading)
 */
void test_imu_static_noise(void);

/**
 * Test: Rotate device 90° clockwise, integrate yaw
 * Expected: Integrated yaw ≈ 90° ± 5°
 */
void test_imu_rotation(void);
```

### test_network.c

**Purpose:** Verify WiFi and UDP communication

```c
/**
 * Test: Send 100 UDP packets, measure round-trip time
 * Expected: <20ms average RTT
 */
void test_udp_latency(void);

/**
 * Test: Saturate UDP TX with max rate packets
 * Expected: No dropped packets, stable throughput >2 Mbps
 */
void test_udp_throughput(void);
```

### test_integration.c

**Purpose:** End-to-end system test

```c
/**
 * Test: Full pipeline - capture audio, send UDP, receive command, fire buzzer
 * Expected: Buzzer fires within 50ms of command
 */
void test_full_pipeline(void);
```

---

## Build and Flash Instructions

### 1. Setup ESP-IDF Environment

```bash
# Clone ESP-IDF v5.2 or v5.3
git clone -b v5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
. ./export.sh
```

### 2. Clone u8g2 ESP-IDF Component

```bash
cd hearlink-firmware/components
git clone https://github.com/olikraus/u8g2.git
```

### 3. Build Project

```bash
cd hearlink-firmware
idf.py set-target esp32s3
idf.py menuconfig  # Optional: verify config
idf.py build
```

### 4. Flash and Monitor

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Testing Workflow

### 1. Hardware Tests (on device)

```bash
# Build with test configuration
idf.py -DCONFIG_ENABLE_TESTS=1 build flash

# Run tests via serial monitor
idf.py monitor
# In monitor, trigger tests with commands:
# test_audio
# test_imu
# test_network
# test_integration
```

### 2. Host-Triggered Tests

From Python host script:

```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
esp32_ip = "192.168.4.1"

# Test buzzers
sock.sendto(bytes([0x11, 0, 0, 0]), (esp32_ip, 5006))

# Test display
sock.sendto(bytes([0x12, 0, 0, 0]), (esp32_ip, 5006))

# Test mics
sock.sendto(bytes([0x13, 0, 0, 0]), (esp32_ip, 5006))

# Test IMU
sock.sendto(bytes([0x14, 0, 0, 0]), (esp32_ip, 5006))
```

---

## Debug and Troubleshooting

### Common Issues and Solutions

| Issue | Diagnostic | Solution |
|-------|------------|----------|
| No I²S data | Check DMA buffer reads | Verify GPIO routing, check INMP441 VDD |
| I²C timeout | `i2c_master_probe()` fails | Lower I²C speed to 100kHz, check pull-ups |
| UDP packet loss | Monitor drops in udp_tx_task | Increase audio_queue depth, check WiFi signal |
| IMU drift | Yaw integrates even when still | Implement bias calibration on host |
| Buzzer weak | Low intensity | Check transistor base resistor, verify GPIO current |
| Display blank | `display_init()` fails | Verify SSD1306 address (0x3C vs 0x3D) |

### Serial Log Levels

```c
// In main.c, set per-module log levels:
esp_log_level_set("AUDIO", ESP_LOG_INFO);
esp_log_level_set("IMU", ESP_LOG_DEBUG);
esp_log_level_set("NET", ESP_LOG_VERBOSE);
esp_log_level_set("BUZZER", ESP_LOG_WARN);
esp_log_level_set("DISPLAY", ESP_LOG_INFO);
```

---

## Performance Metrics (Expected)

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Audio latency (capture to UDP TX) | <20ms | Timestamp in packet vs capture time |
| UDP throughput | >2 Mbps | Packet seq numbers, no gaps |
| IMU sample rate | 100 Hz ±5% | Count samples over 10s window |
| Buzzer response time | <50ms | Command RX to GPIO high |
| Display update rate | 10 Hz | Frame counter in display task |
| CPU usage (Core 1) | <70% | `vTaskGetRunTimeStats()` |
| CPU usage (Core 0) | <50% | `vTaskGetRunTimeStats()` |

---

## Next Steps After Firmware Implementation

1. **Calibration Procedure:**
   - Generate impulse (clap or sine burst) near mics
   - Capture synchronized data on host
   - Compute cross-correlation, extract offsets
   - Send CMD_SET_CALIBRATION with offsets

2. **Host Integration:**
   - Python script to receive UDP packets
   - Beamforming pipeline (delay-and-sum)
   - YAMNet integration
   - Command dispatcher back to ESP32

3. **Field Testing:**
   - Test with real environmental sounds (traffic, sirens)
   - Validate buzzer directional cues with user trials
   - Measure end-to-end latency from sound to haptic feedback

---

## Code Style Guidelines

- **Naming:**
  - Functions: `module_verb_noun()` (e.g., `audio_capture_init()`)
  - Structs: `PascalCase` (e.g., `AudioBuffer`)
  - Enums: `UPPER_SNAKE_CASE` (e.g., `CMD_DISPLAY_ICON`)
  - Constants: `UPPER_SNAKE_CASE` (e.g., `SAMPLE_RATE`)

- **Error Handling:**
  - Always check `ESP_ERROR_CHECK()` for init functions
  - Log errors with `ESP_LOGE(TAG, ...)`
  - Return `ESP_OK` or `ESP_FAIL` from functions

- **Comments:**
  - Document all public functions with purpose, params, return
  - Use `// TODO:` for incomplete implementations
  - Add inline comments for non-obvious logic

---

## Conclusion

This implementation guide provides complete specifications for building the ESP32-S3 firmware for HearLink. Each module is designed to be independently testable and follows ESP-IDF best practices for FreeRTOS task management, hardware driver usage, and network communication.

Key implementation priorities:
1. Audio capture with proper I²S synchronization
2. Robust UDP communication with the host
3. Calibration system for channel alignment
4. Comprehensive test modes for hardware validation

The firmware acts as a pure I/O node, delegating all signal processing to the host while providing low-latency audio streaming and responsive feedback control.
