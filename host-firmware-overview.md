# HearLink Firmware — Host Implementation Reference

This document describes everything the host software needs to know to communicate with the
HearLink ESP32-S3 firmware. It covers connectivity, inbound audio/IMU data, outbound commands,
and calibration.

---

## 1. Network Setup

The ESP32 runs a **WiFi SoftAP** — the host must join this network.

| Parameter | Value |
|---|---|
| SSID | `HearLink` |
| Password | `hearlink2025` |
| Security | WPA2-PSK |
| ESP32 IP (static) | `192.168.4.1` |
| DHCP | Enabled on AP — host gets assigned an IP dynamically |

The host IP is discovered automatically: the firmware learns it the moment any UDP packet
arrives from the host on the RX port. **The host must send at least one command packet to
register itself before the ESP32 starts streaming audio.**

---

## 2. UDP Ports

| Direction | Port | Description |
|---|---|---|
| ESP32 → Host | **5005** | Audio + IMU data stream |
| Host → ESP32 | **5006** | Commands (buzzer, display, calibration, test) |

Both are raw UDP on IPv4. Max buffer size is 4096 bytes.

---

## 3. Inbound Data: Audio + IMU Packets (port 5005)

### Packet Structure (`AudioIMUSubPacket`, 1292 bytes, packed)

```c
typedef struct __attribute__((packed)) {
    uint32_t seq;             // monotonically increasing, per AudioBuffer (160 samples)
    uint32_t timestamp_us;   // ESP32 uptime in microseconds (esp_timer_get_time())
    int16_t  imu_yaw_rate;   // MPU-6050 raw gyro Z output (±250°/s range, 131 LSB/°/s)
    uint8_t  sub_seq;        // 0 = first half, 1 = second half of this buffer
    uint8_t  pad;            // reserved, always 0
    int32_t  audio[4][80];   // 4 channels × 80 samples, int32_t (24-bit data in MSBs)
} AudioIMUSubPacket;         // total: 4+4+2+1+1 + 4*80*4 = 12 + 1280 = 1292 bytes
```

### Sub-packet protocol

Each 160-sample audio buffer is split into **two** UDP packets with the same `seq` number:
- `sub_seq = 0` → samples `[0..79]`
- `sub_seq = 1` → samples `[80..159]`

To reconstruct a full buffer: collect both sub-packets with the same `seq`, then concatenate
channel arrays. Either sub-packet can arrive first or be lost independently.

### Audio format

- **Sample rate:** 16,000 Hz
- **Bit depth:** 32-bit signed integers from I²S DMA, but MEMS mics output 24-bit data
  right-justified in the MSBs. Effective data is in the upper 24 bits — shift right by 8
  (or divide by 256) to get a signed 24-bit value.
- **Channels:** 4 mono channels, spatially arranged at 90° intervals:
  - `ch0` = Front (I²S0 Left)
  - `ch1` = Right (I²S0 Right)
  - `ch2` = Back (I²S1 Left)
  - `ch3` = Left (I²S1 Right)
- **Buffer rate:** 160 samples @ 16kHz = **10ms per buffer**, i.e. 100 buffers/sec,
  200 UDP packets/sec.

### IMU format

- **Sensor:** MPU-6050, gyroscope Z-axis only (yaw rate)
- **Range:** ±250°/s, **131 LSB per °/s**
- To convert to °/s: `yaw_rate_dps = imu_yaw_rate / 131.0`
- The value is snapshotted once per buffer (100Hz effective), shared across both sub-packets
  of the same `seq`.

### Calibration / timing offsets

The firmware applies per-channel sample delays (calibration offsets, ±64 samples max) to
time-align the 4 microphones before packing the audio. The host receives pre-aligned audio
and does **not** need to apply additional channel delays unless doing its own fine calibration.

---

## 4. Outbound Commands (port 5006)

All commands share a common 4-byte structure:

```c
typedef struct __attribute__((packed)) {
    uint8_t type;       // CommandType enum (see below)
    uint8_t direction;  // 0–255 maps to 0–360° (0=front, 64=right, 128=back, 192=left)
    uint8_t category;   // SoundCategory (0=DANGEROUS, 1=SOCIAL, 2=AMBIENT)
    uint8_t pattern;    // BuzzerPattern or IconType depending on command
} Command;
```

### Command types

| Hex | Name | Effect | `direction` | `category` | `pattern` |
|---|---|---|---|---|---|
| `0x01` | `CMD_DISPLAY_ICON` | Show icon on OLED at given direction | 0–255 → angle | sound category | `IconType` |
| `0x02` | `CMD_FIRE_BUZZER` | Vibrate buzzer(s) toward direction | 0–255 → angle | sets intensity | `BuzzerPattern` |
| `0x03` | `CMD_CLEAR_ALL` | Clear OLED display | — | — | — |
| `0x04` | `CMD_CALIBRATE` | Trigger automatic calibration routine | — | — | — |
| `0x05` | `CMD_SET_CALIBRATION` | Set one channel's sample offset | channel index (0–3) | — | signed offset as uint8_t |
| `0x10` | `CMD_TEST_MODE` | Enter/exit test mode | — | — | — |
| `0x11` | `CMD_TEST_BUZZERS` | Test all buzzers | — | — | — |
| `0x12` | `CMD_TEST_DISPLAY` | Test OLED display | — | — | — |
| `0x13` | `CMD_TEST_MICS` | Test all mics together | — | — | — |
| `0x14` | `CMD_TEST_IMU` | Test IMU | — | — | — |
| `0x15` | `CMD_TEST_MICS_INDIV` | Test mics individually | — | — | — |
| `0x16` | `CMD_TEST_BUZZER_SOLO` | Test one buzzer by index | buzzer index (0–3) | — | — |

### Direction encoding

`direction` is a uint8_t that linearly maps to compass angle:
- `0` = Front
- `64` = Right
- `128` = Back
- `192` = Left

The firmware maps this to the nearest of the 4 buzzer positions using:
`buzzer_index = round(direction × 4 / 256) % 4`

### Buzzer patterns (`pattern` field for `CMD_FIRE_BUZZER`)

| Value | Name | Behavior |
|---|---|---|
| `0` | `PATTERN_OFF` | Silence |
| `1` | `PATTERN_CONTINUOUS` | On continuously |
| `2` | `PATTERN_PULSE_FAST` | 200ms on / 200ms off, repeating |
| `3` | `PATTERN_PULSE_SLOW` | 500ms on / 500ms off, repeating |
| `4` | `PATTERN_DOUBLE_TAP` | Two 100ms pulses with 100ms gap, then off |

### Buzzer intensities by category

| Category | Value | Duty cycle (~) |
|---|---|---|
| `DANGEROUS` (0) | 100 | ~50% (max volume) |
| `SOCIAL` (1) | 70 | ~35% |
| `AMBIENT` (2) | 40 | ~20% |

Buzzer PWM frequency is **200 Hz** (piezo resonance range). Intensity maps linearly to
duty cycle capped at 50% (piezo AC swing peaks there).

---

## 5. Connection / Session Lifecycle

1. Host joins WiFi AP `HearLink`.
2. Host sends **any** 4-byte command to `192.168.4.1:5006`. This registers the host IP.
3. ESP32 begins streaming `AudioIMUSubPacket` datagrams to `host_ip:5005`.
4. If the host IP changes (reconnect), the firmware auto-updates to the new IP on the next
   received command.
5. There is **no handshake or keepalive** — if no audio is needed, simply stop sending
   commands and ignore the stream. The firmware streams unconditionally once a host IP is set.

---

## 6. Packet Loss Handling

- UDP is unreliable. Sub-packets may arrive out of order or be dropped.
- Each buffer has a 32-bit `seq` number. Gaps in `seq` indicate dropped buffers.
- Missing sub-packets (missing `sub_seq=0` or `sub_seq=1` for a given `seq`) should be
  treated as zeroed samples or interpolated — the firmware does not retransmit.
- The firmware itself drops audio buffers if the transmit queue fills (logged as
  "dropped frames"). This happens under heavy load — keep the host pipeline fast.

---

## 7. Calibration

Calibration offsets shift each channel's audio by ±64 samples in the ring buffer to
time-align the 4 mics (corrects for physical placement asymmetry).

- `CMD_CALIBRATE` triggers the on-device automatic calibration routine (records impulse,
  computes cross-correlation offsets, saves to NVS flash).
- `CMD_SET_CALIBRATION` manually sets a single channel's offset. `direction` = channel index
  (0–3), `pattern` = signed offset cast to uint8_t.
- Offsets persist across reboots via NVS.

---

## 8. Quick Reference: Byte Layout

**Sending a `CMD_FIRE_BUZZER` for a sound coming from the right at DANGEROUS intensity,
double-tap pattern:**
```
[0x02, 0x40, 0x00, 0x04]
  ^     ^     ^     ^
  type  dir   cat   pattern
  FIRE  90°  DANG  DOUBLE_TAP
```

**Sending a `CMD_DISPLAY_ICON` for a sound coming from behind:**
```
[0x01, 0x80, 0x01, <icon_type>]
  ^     ^     ^
  type  180°  SOCIAL
```
