#ifndef CONFIG_H
#define CONFIG_H

// Audio
#define SAMPLE_RATE         16000
#define BITS_PER_SAMPLE     32
#define SAMPLES_PER_BUFFER  160
#define SAMPLES_PER_SUBPKT  80
#define NUM_CHANNELS        4
#define MAX_CAL_OFFSET      64   // max ±samples for calibration ring buffer

// Mic-presence detection (run once at startup).
// A working I²S mic produces ambient noise with non-trivial peak-to-peak;
// a disconnected mic floats and reads as ~constant or low-amplitude garbage.
// MEMS mics output saturated junk for ~tens of ms after BCLK starts, so we
// wait + drain before measuring.
#define MIC_WARMUP_MS              200
#define MIC_WARMUP_DRAIN_BUFFERS   8
#define MIC_DETECT_BUFFERS         8
#define MIC_DETECT_MIN_PP          100000   // 32-bit raw LSBs; tune from reported numbers

// Single-mic bring-up mode: set to 1 to skip the normal app on boot and loop
// through each mic one at a time with a big banner + live amplitude bars.
// Leave at 0 for normal operation.
#define MIC_TEST_BOOT_MODE         0
#define MIC_TEST_SECONDS_PER_MIC   10

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

// Buzzer GPIO (4 channels — 4 PCB control boards available)
// Order: buzzer 0=front, 1=right, 2=back, 3=left (90° spacing)
#define BUZZER_PINS         { 1, 2, 10, 11 }
#define NUM_BUZZERS         4
#define BUZZER_PWM_FREQ     200   // Hz
#define BUZZER_PWM_RES      LEDC_TIMER_10_BIT

// Status LED (WS2812 addressable on ESP32-S3-DevKitC-1)
#define STATUS_LED_PIN      48
#define STATUS_LED_COUNT    1

// WiFi
#define WIFI_SSID           "HearLink"
#define WIFI_PASSWORD       "hearlink2025"
#define AP_IP_ADDR          "192.168.4.1"
#define AP_NETMASK          "255.255.255.0"
#define AP_MAX_STA          4
#define AP_CHANNEL          1

// UDP
#define UDP_PORT_TX         5005  // ESP32 → Host
#define UDP_PORT_RX         5006  // Host → ESP32
#define UDP_BUFFER_SIZE     4096

// NVS namespace for calibration
#define NVS_NAMESPACE       "cal"
#define NVS_KEY_OFFSET_FMT  "offset%d"  // offset0 .. offset3

// Buzzer intensity by sound category
#define INTENSITY_DANGEROUS 100
#define INTENSITY_SOCIAL     70
#define INTENSITY_AMBIENT    40

// Task priorities
#define PRIORITY_AUDIO_CAPTURE  5
#define PRIORITY_UDP_TX         4
#define PRIORITY_UDP_RX         3
#define PRIORITY_IMU_POLL       3
#define PRIORITY_BUZZER         2
#define PRIORITY_DISPLAY        1
#define PRIORITY_TEST           1

// Task stack sizes (bytes)
#define STACK_SIZE_AUDIO        8192
#define STACK_SIZE_UDP_TX       8192
#define STACK_SIZE_UDP_RX       8192
#define STACK_SIZE_IMU          4096
#define STACK_SIZE_BUZZER       4096
#define STACK_SIZE_DISPLAY      4096
#define STACK_SIZE_TEST         8192

// Display
#define OLED_ADDR           0x3C
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define RING_CENTER_X       64
#define RING_CENTER_Y       32
#define RING_RADIUS         26
#define ICON_SIZE           12
#define MAX_ACTIVE_ICONS    3

#endif // CONFIG_H
