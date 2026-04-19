#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "config.h"
#include "network.h"
#include "audio_capture.h"
#include "buzzer_control.h"
#include "display.h"
#include "calibration.h"

// Declared in main.c
extern volatile int16_t latest_imu_yaw_rate;
extern SemaphoreHandle_t imu_mutex;
extern TaskHandle_t test_task_handle;

static const char *TAG = "NET";

// ── Split-packet protocol ─────────────────────────────────────────────────────

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint32_t timestamp_us;
    int16_t  imu_yaw_rate;
    uint8_t  sub_seq;  // 0 = samples[0..79], 1 = samples[80..159]
    uint8_t  pad;
    int32_t  audio[NUM_CHANNELS][SAMPLES_PER_SUBPKT];
} AudioIMUSubPacket;

_Static_assert(sizeof(AudioIMUSubPacket) == 1292,
               "AudioIMUSubPacket size must be 1292 bytes");

// ── Inbound command types ─────────────────────────────────────────────────────

typedef enum {
    CMD_DISPLAY_ICON     = 0x01,
    CMD_FIRE_BUZZER      = 0x02,
    CMD_CLEAR_ALL        = 0x03,
    CMD_CALIBRATE        = 0x04,
    CMD_SET_CALIBRATION  = 0x05,
    CMD_TEST_MODE        = 0x10,
    CMD_TEST_BUZZERS     = 0x11,
    CMD_TEST_DISPLAY     = 0x12,
    CMD_TEST_MICS        = 0x13,
    CMD_TEST_IMU         = 0x14,
    CMD_TEST_MICS_INDIV  = 0x15,
    CMD_TEST_BUZZER_SOLO = 0x16,
} CommandType;

typedef enum {
    CATEGORY_DANGEROUS = 0,
    CATEGORY_SOCIAL    = 1,
    CATEGORY_AMBIENT   = 2,
} SoundCategory;

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t direction;
    uint8_t category;
    uint8_t pattern;
} Command;

// ── Shared host IP (set by RX, read by TX) ───────────────────────────────────

static char host_ip[INET_ADDRSTRLEN] = "";
static SemaphoreHandle_t host_ip_mutex = NULL;

const char *network_get_host_ip(void)
{
    return host_ip;
}

// ── WiFi init ─────────────────────────────────────────────────────────────────

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)event_data;
            ESP_LOGI(TAG, "STA connected — MAC %02X:%02X:%02X:%02X:%02X:%02X",
                     e->mac[0], e->mac[1], e->mac[2],
                     e->mac[3], e->mac[4], e->mac[5]);
        } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            ESP_LOGI(TAG, "STA disconnected");
        }
    }
}

esp_err_t network_init(void)
{
    host_ip_mutex = xSemaphoreCreateMutex();
    configASSERT(host_ip_mutex);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    // Static IP for AP interface
    esp_netif_ip_info_t ip_info;
    esp_netif_dhcps_stop(ap_netif);
    inet_pton(AF_INET, AP_IP_ADDR,  &ip_info.ip);
    inet_pton(AF_INET, AP_IP_ADDR,  &ip_info.gw);
    inet_pton(AF_INET, AP_NETMASK,  &ip_info.netmask);
    esp_netif_set_ip_info(ap_netif, &ip_info);
    esp_netif_dhcps_start(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .ap = {
            .ssid            = WIFI_SSID,
            .ssid_len        = sizeof(WIFI_SSID) - 1,
            .channel         = AP_CHANNEL,
            .password        = WIFI_PASSWORD,
            .max_connection  = AP_MAX_STA,
            .authmode        = WIFI_AUTH_WPA2_PSK,
        },
    };
    memcpy(wifi_cfg.ap.ssid,     WIFI_SSID,     sizeof(WIFI_SSID));
    memcpy(wifi_cfg.ap.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi SoftAP started — SSID=%s", WIFI_SSID);
    return ESP_OK;
}

// ── UDP TX task ───────────────────────────────────────────────────────────────

static void udp_tx_task(void *arg)
{
    QueueHandle_t q = (QueueHandle_t)arg;
    ESP_LOGI(TAG, "UDP TX task running (port %d)", UDP_PORT_TX);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "TX socket create failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(UDP_PORT_TX),
    };

    AudioBuffer buf;
    AudioIMUSubPacket pkt;
    uint32_t seq = 0;

    for (;;) {
        if (xQueueReceive(q, &buf, pdMS_TO_TICKS(20)) != pdTRUE) {
            continue;
        }

        // Retrieve host IP
        char cur_ip[INET_ADDRSTRLEN];
        xSemaphoreTake(host_ip_mutex, portMAX_DELAY);
        memcpy(cur_ip, host_ip, sizeof(cur_ip));
        xSemaphoreGive(host_ip_mutex);

        if (cur_ip[0] == '\0') {
            continue;  // no client yet
        }

        inet_pton(AF_INET, cur_ip, &dest.sin_addr);

        // Snapshot IMU
        int16_t yaw;
        xSemaphoreTake(imu_mutex, portMAX_DELAY);
        yaw = latest_imu_yaw_rate;
        xSemaphoreGive(imu_mutex);

        uint32_t ts = (uint32_t)esp_timer_get_time();

        // Send sub-packet 0 (samples 0..79) then sub-packet 1 (samples 80..159)
        for (int sub = 0; sub < 2; sub++) {
            int base = sub * SAMPLES_PER_SUBPKT;
            pkt.seq           = seq;
            pkt.timestamp_us  = ts;
            pkt.imu_yaw_rate  = yaw;
            pkt.sub_seq       = (uint8_t)sub;
            pkt.pad           = 0;

            // Copy 80 samples per channel
            memcpy(pkt.audio[0], &buf.ch0[base], SAMPLES_PER_SUBPKT * sizeof(int32_t));
            memcpy(pkt.audio[1], &buf.ch1[base], SAMPLES_PER_SUBPKT * sizeof(int32_t));
            memcpy(pkt.audio[2], &buf.ch2[base], SAMPLES_PER_SUBPKT * sizeof(int32_t));
            memcpy(pkt.audio[3], &buf.ch3[base], SAMPLES_PER_SUBPKT * sizeof(int32_t));

            int sent = sendto(sock, &pkt, sizeof(pkt), 0,
                              (struct sockaddr *)&dest, sizeof(dest));
            if (sent < 0) {
                ESP_LOGD(TAG, "sendto failed: errno %d", errno);
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
        seq++;
    }
}

void udp_tx_start(QueueHandle_t audio_queue)
{
    xTaskCreatePinnedToCore(udp_tx_task, "udp_tx",
                            STACK_SIZE_UDP_TX, audio_queue,
                            PRIORITY_UDP_TX, NULL, 1);
}

// ── Direction → buzzer mask conversion ───────────────────────────────────────

static uint8_t direction_to_buzzer_mask(uint8_t direction)
{
    // NUM_BUZZERS buzzers evenly spaced around the ring.
    // Map direction 0–255 → buzzer index 0..NUM_BUZZERS-1 (nearest)
    uint8_t idx = (uint8_t)(((uint16_t)direction * NUM_BUZZERS + 128) / 256) % NUM_BUZZERS;
    return (uint8_t)(1u << idx);
}

static uint8_t category_to_intensity(uint8_t category)
{
    switch ((SoundCategory)category) {
    case CATEGORY_DANGEROUS: return INTENSITY_DANGEROUS;
    case CATEGORY_SOCIAL:    return INTENSITY_SOCIAL;
    default:                 return INTENSITY_AMBIENT;
    }
}

// ── UDP RX task ───────────────────────────────────────────────────────────────

static void udp_rx_task(void *arg)
{
    // arg is a pointer to a struct containing both queues
    QueueHandle_t *queues   = (QueueHandle_t *)arg;
    QueueHandle_t bq        = queues[0];
    QueueHandle_t dq        = queues[1];

    ESP_LOGI(TAG, "UDP RX task running (port %d)", UDP_PORT_RX);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "RX socket create failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_PORT_RX),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "RX bind failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    uint8_t rx_buf[UDP_BUFFER_SIZE];
    struct sockaddr_in src;
    socklen_t src_len = sizeof(src);

    for (;;) {
        int n = recvfrom(sock, rx_buf, sizeof(rx_buf), 0,
                         (struct sockaddr *)&src, &src_len);
        if (n < 0) {
            ESP_LOGW(TAG, "recvfrom error: errno %d", errno);
            continue;
        }
        if (n < (int)sizeof(Command)) {
            ESP_LOGW(TAG, "Short packet (%d bytes), ignoring", n);
            continue;
        }

        // Store source IP for TX task
        char new_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &src.sin_addr, new_ip, sizeof(new_ip));
        xSemaphoreTake(host_ip_mutex, portMAX_DELAY);
        memcpy(host_ip, new_ip, sizeof(host_ip));
        xSemaphoreGive(host_ip_mutex);

        Command cmd;
        memcpy(&cmd, rx_buf, sizeof(cmd));

        switch ((CommandType)cmd.type) {
        case CMD_DISPLAY_ICON: {
            DisplayCommand dcmd = {
                .direction = cmd.direction,
                .icon      = (IconType)cmd.pattern,
                .category  = cmd.category,
                .active    = 1,
            };
            xQueueSend(dq, &dcmd, 0);
            break;
        }
        case CMD_CLEAR_ALL: {
            DisplayCommand dcmd = { .active = 0 };
            xQueueSend(dq, &dcmd, 0);
            break;
        }
        case CMD_FIRE_BUZZER: {
            BuzzerCommand bcmd = {
                .buzzer_mask = direction_to_buzzer_mask(cmd.direction),
                .intensity   = category_to_intensity(cmd.category),
                .pattern     = (BuzzerPattern)cmd.pattern,
            };
            xQueueSend(bq, &bcmd, 0);
            break;
        }
        case CMD_CALIBRATE:
            calibration_start_calibration();
            break;
        case CMD_SET_CALIBRATION:
            calibration_apply_single(cmd.direction, (int8_t)cmd.pattern);
            break;
        case CMD_TEST_BUZZERS:
        case CMD_TEST_DISPLAY:
        case CMD_TEST_MICS:
        case CMD_TEST_MICS_INDIV:
        case CMD_TEST_IMU:
        case CMD_TEST_MODE:
            if (test_task_handle) {
                xTaskNotify(test_task_handle, cmd.type, eSetValueWithOverwrite);
            }
            break;
        case CMD_TEST_BUZZER_SOLO:
            // Pack buzzer index (from direction byte) into bits 8-15 of the notify word
            if (test_task_handle) {
                uint32_t word = (uint32_t)cmd.type | ((uint32_t)cmd.direction << 8);
                xTaskNotify(test_task_handle, word, eSetValueWithOverwrite);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown command type: 0x%02X", cmd.type);
            break;
        }
    }
}

// Static queue pair for passing two queues to rx task
static QueueHandle_t rx_queues[2];

void udp_rx_start(QueueHandle_t buzzer_queue, QueueHandle_t display_queue)
{
    rx_queues[0] = buzzer_queue;
    rx_queues[1] = display_queue;
    xTaskCreatePinnedToCore(udp_rx_task, "udp_rx",
                            STACK_SIZE_UDP_RX, rx_queues,
                            PRIORITY_UDP_RX, NULL, 0);
}
