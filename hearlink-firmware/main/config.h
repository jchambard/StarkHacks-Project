#ifndef CONFIG_H
#define CONFIG_H

// Audio
#define SAMPLE_RATE         16000
#define BITS_PER_SAMPLE     32
#define SAMPLES_PER_BUFFER  160
#define SAMPLES_PER_SUBPKT  80
#define NUM_CHANNELS        4
#define MAX_CAL_OFFSET      64   // max ±samples for calibration ring buffer

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
#define BUZZER_PINS         { 1, 2, 10, 11, 12, 13, 14, 17 }
#define NUM_BUZZERS         8
#define BUZZER_PWM_FREQ     400   // Hz
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
#define STACK_SIZE_AUDIO        4096
#define STACK_SIZE_UDP_TX       4096
#define STACK_SIZE_UDP_RX       3072
#define STACK_SIZE_IMU          2048
#define STACK_SIZE_BUZZER       2048
#define STACK_SIZE_DISPLAY      3072
#define STACK_SIZE_TEST         3072

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
