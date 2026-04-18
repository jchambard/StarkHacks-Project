#ifndef NETWORK_H
#define NETWORK_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * Initialize WiFi in SoftAP mode with static IP 192.168.4.1.
 * SSID: "HearLink", password: "hearlink2025", channel 1, WPA2-PSK, max 4 STA.
 * Must be called after nvs_flash_init().
 */
esp_err_t network_init(void);

/**
 * Start UDP TX task on Core 1 at priority PRIORITY_UDP_TX.
 * Reads AudioBuffer from audio_queue, emits two AudioIMUSubPackets per frame.
 * Drops frames silently until a client IP is known (set by RX task on first command).
 * @param audio_queue Source queue (populated by audio_capture_task)
 */
void udp_tx_start(QueueHandle_t audio_queue);

/**
 * Start UDP RX task on Core 0 at priority PRIORITY_UDP_RX.
 * Receives Command structs on UDP port 5006, dispatches to queues / handlers.
 * Stores the source IP for use by the TX task.
 * @param buzzer_queue  Destination for CMD_FIRE_BUZZER (BuzzerCommand)
 * @param display_queue Destination for CMD_DISPLAY_ICON / CMD_CLEAR_ALL (DisplayCommand)
 */
void udp_rx_start(QueueHandle_t buzzer_queue, QueueHandle_t display_queue);

/**
 * Return the last host IP address seen by the RX task ("" if none yet).
 */
const char *network_get_host_ip(void);

#endif // NETWORK_H
