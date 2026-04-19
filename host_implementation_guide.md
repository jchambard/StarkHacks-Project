# HearLink Host Computer Implementation Guide

**Target Platform:** macOS (latest)  
**Python Version:** 3.10+ (for best TensorFlow compatibility)  
**Environment:** venv  
**Processing Priority:** Functionality first, then optimize

---

## ⚠️ Critical Bugs Fixed in This Version

**The following bugs from the initial version have been corrected:**

1. **YAMNet API Fix**: `model.class_names()` doesn't exist → Use `model.class_map_path()` + CSV parsing
2. **TensorFlow on Apple Silicon**: Added `tensorflow-macos` + `tensorflow-metal` for ARM64 Macs
3. **Removed `asyncio` from requirements**: It's stdlib since Python 3.4, PyPI version can cause conflicts
4. **Deprecated API Fix**: Replaced `asyncio.get_event_loop()` → `asyncio.get_running_loop()`
5. **False Packet Loss Warnings**: Only check seq on `sub_seq=0` (both sub-packets share same seq)
6. **Audio Conversion Clarity**: Single-step formula `/ (256.0 * 2**23)` prevents misinterpretation
7. **Thread-Safety Note**: Corrected callback advice for potential future threading

**All core math verified correct**: struct layout, beamforming geometry, IMU scaling, direction encoding ✅

---

## Critical Implementation Notes for AI Agents

### Must-Know Architecture Decisions

1. **8 Mics in Hardware, 4-Channel Processing**
   - ESP32-S3 has 8× INMP441 mics @ 45° spacing (hardware)
   - But firmware streams **4 channels only** (90° spacing: Front/Right/Back/Left)
   - Beamformer processes 4-channel audio with 90° mic spacing
   - 8 buzzers provide finer haptic output than 4-channel audio input

2. **Dual I2S Architecture**
   - I2S0: Mics 0-3 (Front, Right at I2S0-L/R; Back, Left at I2S0-L/R)
   - I2S1: Mics 4-7 (redundant in current firmware - only 4 channels used)
   - Each I2S bus captures stereo (L/R channels)
   - **Host receives 4 independent channels in audio[4][80] array**

3. **Sub-Packet Protocol**
   - Each 160-sample buffer split into 2 UDP packets (sub_seq 0 and 1)
   - Same `seq` number for both halves
   - Must reassemble before processing
   - Either packet can arrive first or be lost

4. **Audio Format Conversion**
   - Received: `int32_t` from I2S (24-bit data left-aligned in MSBs)
   - Conversion: `float32 = int32 / (256.0 * 2^23)` → [-1.0, 1.0] range
   - Single-step formula: shift right 8 bits (÷256) and normalize signed 24-bit
   - YAMNet expects: 16kHz mono, normalized to [-1, 1]

5. **IMU Data Flow**
   - Raw gyro Z: `int16_t` in packet (±250°/s range)
   - Conversion: `yaw_rate_dps = raw / 131.0`
   - Integration: `yaw += yaw_rate * dt` (dt = 0.01s at 100Hz)
   - Bias calibration: average first 2 seconds, subtract from all subsequent readings

6. **Direction Encoding**
   - Collar frame: 0° = Front, 90° = Right, 180° = Back, 270° = Left
   - Head-relative: `θ_rel = θ_collar - current_yaw`
   - Command format: `direction_byte = int(angle_deg * 256 / 360)`

7. **Beamforming Fundamentals**
   - 4 mics at 90° spacing on ~0.15m radius ring
   - Delay for mic `i` at steering angle `θ`: `Δt = (r/c) * cos(θ - φ_i)`
   - Steer to 24 angles (0°, 15°, 30°, ..., 345°)
   - Fractional delays via interpolation or FFT shift

8. **YAMNet Usage**
   - Model URL: `https://tfhub.dev/google/yamnet/1`
   - Input: 16kHz mono waveform, any length (uses 0.96s internal windows)
   - Output: (num_frames, 521) scores array
   - Average scores across frames for overall classification

9. **Category Priority & Buzzer Behavior**
   - DANGEROUS (0) > SOCIAL (1) > AMBIENT (2)
   - Max 3 concurrent events displayed
   - Filter by per-category confidence thresholds
   - **Buzzer defaults: DANGEROUS only** (customizable via `audio_config.yaml`)
   - Visual icons always shown for all categories

10. **Network Protocol**
    - ESP32 IP: `192.168.4.1` (static on SoftAP)
    - Inbound: Port 5005 (audio/IMU)
    - Outbound: Port 5006 (commands)
    - Host must send ≥1 command to register IP before ESP32 streams

---

## Table of Contents

1. [Project Structure](#project-structure)
2. [Dependencies & Setup](#dependencies--setup)
3. [Module Architecture & Data Flow](#module-architecture--data-flow)
4. [Module Specifications](#module-specifications)
5. [Configuration Files](#configuration-files)
6. [Testing Guidelines](#testing-guidelines)
7. [Performance Targets](#performance-targets)

---

## Project Structure

```
hearlink/
├── firmware/              # ESP32-S3 firmware (existing)
├── host/                  # New host computer program
│   ├── config/
│   │   ├── yamnet_categories.yaml    # YAMNet class → category mapping
│   │   ├── audio_config.yaml         # Audio processing parameters
│   │   └── network_config.yaml       # WiFi/UDP settings
│   ├── src/
│   │   ├── __init__.py
│   │   ├── main.py                   # Entry point & main loop
│   │   ├── network/
│   │   │   ├── __init__.py
│   │   │   ├── udp_receiver.py       # Audio/IMU packet receiver
│   │   │   └── command_sender.py     # Command packet sender
│   │   ├── audio/
│   │   │   ├── __init__.py
│   │   │   ├── buffer_manager.py     # Circular buffer & packet reassembly
│   │   │   ├── beamformer.py         # Delay-and-sum beamforming
│   │   │   └── audio_utils.py        # Format conversion, windowing
│   │   ├── classification/
│   │   │   ├── __init__.py
│   │   │   ├── yamnet_classifier.py  # YAMNet inference wrapper
│   │   │   └── category_mapper.py    # Map YAMNet classes to user categories
│   │   ├── imu/
│   │   │   ├── __init__.py
│   │   │   ├── yaw_integrator.py     # Gyro integration & bias correction
│   │   │   └── calibration.py        # Startup bias calibration
│   │   ├── feedback/
│   │   │   ├── __init__.py
│   │   │   ├── event_router.py       # Decision logic for icons & buzzers
│   │   │   └── display_manager.py    # Track active icons & their lifetimes
│   │   └── utils/
│   │       ├── __init__.py
│   │       ├── logger.py             # Logging configuration
│   │       └── debug_viz.py          # Optional visualization tools
│   ├── tests/
│   │   ├── __init__.py
│   │   ├── test_packet_parsing.py
│   │   ├── test_beamformer.py
│   │   ├── test_yaw_integration.py
│   │   └── test_end_to_end.py
│   ├── requirements.txt
│   ├── setup.py
│   └── README.md
└── web/                   # Category configuration website (future)
```

---

## Dependencies & Setup

### requirements.txt

```txt
# Core audio processing
numpy>=1.24.0
scipy>=1.10.0
librosa>=0.10.0

# Machine learning
# For Apple Silicon Macs, use tensorflow-macos
tensorflow-macos>=2.13.0; platform_machine == 'arm64'
tensorflow-metal>=1.0.0; platform_machine == 'arm64'
# For Intel Macs and other platforms
tensorflow>=2.13.0; platform_machine != 'arm64'
tensorflow-hub>=0.14.0

# Configuration
pyyaml>=6.0

# Logging & debugging
coloredlogs>=15.0
matplotlib>=3.7.0  # For debug visualization

# Testing
pytest>=7.4.0
pytest-asyncio>=0.21.0
```

### Environment Setup Commands

```bash
cd hearlink/host
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

---

## Module Architecture & Data Flow

### Data Flow Diagram

```
ESP32-S3 (192.168.4.1)
    │
    │ UDP packets @ 200 Hz
    │ Port 5005
    ▼
┌─────────────────────────────────────────────────────────────┐
│ UDPReceiver                                                  │
│ - Receives AudioIMUSubPacket (1292 bytes)                    │
│ - Parses seq, sub_seq, timestamp, imu_yaw_rate, audio[4][80]│
└────────────┬────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────┐
│ BufferManager                                                │
│ - Reassembles sub-packets (sub_seq 0+1) → full buffers      │
│ - Maintains circular buffer of last ~2 seconds              │
│ - Detects packet loss via seq gaps                          │
│ - Converts int32 → float32 (divide by 256 for 24-bit)       │
└────────────┬────────────────────────────────────────────────┘
             │
             ├─────────────────┬──────────────────┐
             ▼                 ▼                  ▼
    ┌─────────────────┐  ┌──────────┐   ┌───────────────┐
    │ Beamformer      │  │ YawInt   │   │ AudioBuffer   │
    │                 │  │ egrator  │   │ for YAMNet    │
    │ - 4-mic ring    │  │          │   │               │
    │ - Delay & sum   │  │ - Bias   │   │ - Windowing   │
    │ - 24 beams      │  │ - Integr.│   │ - Overlap     │
    │   @ 15° steps   │  │ - Yaw(t) │   │               │
    └────────┬────────┘  └────┬─────┘   └───────┬───────┘
             │                │                  │
             │                │                  │
             ▼                │                  │
    ┌─────────────────┐      │                  │
    │ Direction Peak  │      │                  │
    │ Detection       │      │                  │
    │                 │      │                  │
    │ - Energy vs θ   │      │                  │
    │ - Smooth & peak │      │                  │
    │ - Output: θ[]   │      │                  │
    └────────┬────────┘      │                  │
             │                │                  │
             └────────┬───────┘                  │
                      ▼                          │
             ┌──────────────────┐                │
             │ Per-Direction    │◄───────────────┘
             │ Audio Extraction │
             │                  │
             │ - Beamformed     │
             │   mono @ 16kHz   │
             └────────┬─────────┘
                      │
                      ▼
             ┌──────────────────┐
             │ YAMNet Classifier│
             │                  │
             │ - 521 classes    │
             │ - Confidence     │
             └────────┬─────────┘
                      │
                      ▼
             ┌──────────────────┐
             │ CategoryMapper   │
             │                  │
             │ YAMNet class →   │
             │ D/S/A category   │
             └────────┬─────────┘
                      │
                      ▼
             ┌──────────────────┐
             │ EventRouter      │
             │                  │
             │ - Filter by conf │
             │ - Priority logic │
             │ - Icon selection │
             │ - Buzzer mapping │
             └────────┬─────────┘
                      │
                      ▼
             ┌──────────────────┐
             │ DisplayManager   │
             │                  │
             │ - Track active   │
             │   icons (1-3s)   │
             │ - Head-relative  │
             │   angle updates  │
             └────────┬─────────┘
                      │
                      ▼
             ┌──────────────────┐
             │ CommandSender    │
             │                  │
             │ - Format packets │
             │ - UDP to :5006   │
             └──────────────────┘
                      │
                      ▼
                  ESP32-S3
```

## Module Specifications

### 1. network/udp_receiver.py

**Purpose:** Receive and parse UDP packets from ESP32-S3

```python
import asyncio
import struct
import logging
from dataclasses import dataclass
from typing import Optional, Callable

@dataclass
class AudioIMUSubPacket:
    """Parsed representation of ESP32 UDP packet"""
    seq: int                    # uint32_t
    timestamp_us: int           # uint32_t
    imu_yaw_rate: int          # int16_t (raw gyro Z)
    sub_seq: int               # uint8_t (0 or 1)
    pad: int                   # uint8_t
    audio: list[list[int]]     # 4 channels × 80 samples (int32)

class UDPReceiver:
    """
    Receives AudioIMUSubPacket datagrams from ESP32-S3 on port 5005.
    
    Packet format (1292 bytes, packed):
    - seq: uint32_t (4 bytes)
    - timestamp_us: uint32_t (4 bytes)
    - imu_yaw_rate: int16_t (2 bytes)
    - sub_seq: uint8_t (1 byte)
    - pad: uint8_t (1 byte)
    - audio[4][80]: int32_t (1280 bytes)
    """
    
    PACKET_SIZE = 1292
    STRUCT_FORMAT = '<IIhBB' + ('i' * 320)  # little-endian
    
    def __init__(self, port: int = 5005, callback: Optional[Callable] = None):
        self.port = port
        self.callback = callback
        self.logger = logging.getLogger(__name__)
        
    async def start(self):
        """Start UDP receiver loop"""
        loop = asyncio.get_event_loop()
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: UDPProtocol(self._handle_packet),
            local_addr=('0.0.0.0', self.port)
        )
        self.logger.info(f"UDP receiver listening on port {self.port}")
        
    def _handle_packet(self, data: bytes, addr: tuple):
        """Parse incoming packet and invoke callback"""
        if len(data) != self.PACKET_SIZE:
            self.logger.warning(f"Invalid packet size: {len(data)} (expected {self.PACKET_SIZE})")
            return
            
        try:
            packet = self._parse_packet(data)
            if self.callback:
                self.callback(packet)
        except Exception as e:
            self.logger.error(f"Packet parsing error: {e}")
            
    def _parse_packet(self, data: bytes) -> AudioIMUSubPacket:
        """Unpack binary data into AudioIMUSubPacket"""
        values = struct.unpack(self.STRUCT_FORMAT, data)
        
        seq = values[0]
        timestamp_us = values[1]
        imu_yaw_rate = values[2]
        sub_seq = values[3]
        pad = values[4]
        
        # Reshape flat audio array into [4 channels][80 samples]
        audio_flat = values[5:]
        audio = [
            list(audio_flat[i*80:(i+1)*80])
            for i in range(4)
        ]
        
        return AudioIMUSubPacket(
            seq=seq,
            timestamp_us=timestamp_us,
            imu_yaw_rate=imu_yaw_rate,
            sub_seq=sub_seq,
            pad=pad,
            audio=audio
        )

class UDPProtocol(asyncio.DatagramProtocol):
    """Asyncio protocol for UDP packet reception"""
    
    def __init__(self, packet_handler: Callable):
        self.packet_handler = packet_handler
        
    def datagram_received(self, data: bytes, addr: tuple):
        self.packet_handler(data, addr)
```

**Key Implementation Notes:**
- Use `struct.unpack` with little-endian format (`<`)
- Audio is packed as 320 int32 values (4 channels × 80 samples)
- Callback is invoked from asyncio's event loop, so async operations are safe
- If moved to threaded receiver, use `loop.call_soon_threadsafe()` for cross-thread calls

**Testing:**
```python
# Test packet parsing with synthetic data
def test_parse_packet():
    receiver = UDPReceiver()
    # Create mock packet with known values
    test_data = struct.pack('<IIhBB' + 'i'*320, 
                            42, 1000000, 1000, 0, 0, *([100]*320))
    packet = receiver._parse_packet(test_data)
    assert packet.seq == 42
    assert packet.sub_seq == 0
    assert len(packet.audio) == 4
    assert len(packet.audio[0]) == 80
```

---

### 2. network/command_sender.py

**Purpose:** Send command packets to ESP32-S3

```python
import socket
import struct
import logging
from enum import IntEnum

class CommandType(IntEnum):
    """Command types matching firmware enums"""
    CMD_DISPLAY_ICON = 0x01
    CMD_FIRE_BUZZER = 0x02
    CMD_CLEAR_ALL = 0x03
    CMD_CALIBRATE = 0x04
    CMD_SET_CALIBRATION = 0x05
    CMD_TEST_MODE = 0x10
    # ... etc

class SoundCategory(IntEnum):
    DANGEROUS = 0
    SOCIAL = 1
    AMBIENT = 2

class BuzzerPattern(IntEnum):
    PATTERN_OFF = 0
    PATTERN_CONTINUOUS = 1
    PATTERN_PULSE_FAST = 2
    PATTERN_PULSE_SLOW = 3
    PATTERN_DOUBLE_TAP = 4

class CommandSender:
    """
    Sends 4-byte command packets to ESP32-S3 on port 5006.
    
    Packet format:
    - type: uint8_t
    - direction: uint8_t (0-255 maps to 0-360°)
    - category: uint8_t
    - pattern: uint8_t
    """
    
    STRUCT_FORMAT = 'BBBB'  # 4 unsigned bytes
    
    def __init__(self, esp32_ip: str = '192.168.4.1', port: int = 5006):
        self.esp32_addr = (esp32_ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.logger = logging.getLogger(__name__)
        
    def send_command(self, cmd_type: CommandType, direction: int = 0, 
                     category: int = 0, pattern: int = 0):
        """Send a 4-byte command packet"""
        packet = struct.pack(self.STRUCT_FORMAT, cmd_type, direction, category, pattern)
        self.sock.sendto(packet, self.esp32_addr)
        self.logger.debug(f"Sent command: type={cmd_type}, dir={direction}, "
                         f"cat={category}, pat={pattern}")
        
    def display_icon(self, angle_deg: float, category: SoundCategory, icon_type: int = 0):
        """Show an icon at the given angle (0-360°)"""
        direction_byte = int((angle_deg % 360) * 256 / 360)
        self.send_command(CommandType.CMD_DISPLAY_ICON, direction_byte, category, icon_type)
        
    def fire_buzzer(self, angle_deg: float, category: SoundCategory, 
                    pattern: BuzzerPattern = BuzzerPattern.PATTERN_CONTINUOUS):
        """Activate buzzer(s) toward the given angle"""
        direction_byte = int((angle_deg % 360) * 256 / 360)
        # Intensity is encoded in category value (per firmware)
        intensity_map = {
            SoundCategory.DANGEROUS: 100,
            SoundCategory.SOCIAL: 70,
            SoundCategory.AMBIENT: 40
        }
        intensity = intensity_map.get(category, 70)
        self.send_command(CommandType.CMD_FIRE_BUZZER, direction_byte, intensity, pattern)
        
    def clear_display(self):
        """Clear all icons from OLED"""
        self.send_command(CommandType.CMD_CLEAR_ALL)
        
    def trigger_calibration(self):
        """Trigger automatic mic calibration on ESP32"""
        self.send_command(CommandType.CMD_CALIBRATE)
```

**Key Implementation Notes:**
- Direction encoding: `angle_deg * 256 / 360` → uint8
- Category directly maps to intensity for buzzers
- Socket can be reused for all commands

**Testing:**
```python
def test_command_format():
    sender = CommandSender()
    # Fire buzzer at 90° (right), DANGEROUS category, DOUBLE_TAP pattern
    # Expected: [0x02, 0x40, 0x64, 0x04] (type=FIRE, dir=64, intensity=100, pattern=4)
    # Manual verification needed with ESP32
```

---

### 3. audio/buffer_manager.py

**Purpose:** Reassemble sub-packets and maintain audio buffer

```python
import numpy as np
import logging
from collections import defaultdict
from typing import Optional, Tuple

class BufferManager:
    """
    Reassembles AudioIMUSubPacket sub-packets into complete 160-sample buffers
    and maintains a circular buffer for recent audio history.
    """
    
    SAMPLES_PER_BUFFER = 160
    SAMPLES_PER_SUBPACKET = 80
    NUM_CHANNELS = 4
    SAMPLE_RATE = 16000
    BUFFER_DURATION_SEC = 2.0  # Keep last 2 seconds
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        
        # Sub-packet storage: {seq: {sub_seq: AudioIMUSubPacket}}
        self.pending_packets = defaultdict(dict)
        
        # Circular buffer: shape (num_channels, buffer_length)
        buffer_length = int(self.SAMPLE_RATE * self.BUFFER_DURATION_SEC)
        self.audio_buffer = np.zeros((self.NUM_CHANNELS, buffer_length), dtype=np.float32)
        self.buffer_write_pos = 0
        
        # IMU data storage
        self.yaw_rate_buffer = []  # List of (timestamp_us, yaw_rate_dps)
        
        # Packet loss tracking
        self.last_seq = None
        self.dropped_packets = 0
        
    def add_packet(self, packet) -> Optional[Tuple[np.ndarray, float]]:
        """
        Add a sub-packet. Returns (audio_buffer, yaw_rate) if a complete
        buffer is assembled, otherwise None.
        
        Returns:
            Tuple of (audio array [4, 160], yaw_rate in °/s) or None
        """
        seq = packet.seq
        sub_seq = packet.sub_seq
        
        # Track packet loss only on sub_seq=0 (both sub-packets share same seq)
        if sub_seq == 0:
            if self.last_seq is not None:
                expected = self.last_seq + 1
                if seq > expected:
                    lost = seq - expected
                    self.dropped_packets += lost
                    self.logger.warning(f"Packet loss detected: {lost} buffers dropped "
                                       f"(seq jump {self.last_seq} → {seq})")
            self.last_seq = seq
        
        # Store sub-packet
        self.pending_packets[seq][sub_seq] = packet
        
        # Check if we have both sub-packets for this seq
        if 0 in self.pending_packets[seq] and 1 in self.pending_packets[seq]:
            complete_buffer = self._assemble_buffer(seq)
            del self.pending_packets[seq]  # Free memory
            return complete_buffer
        
        return None
        
    def _assemble_buffer(self, seq: int) -> Tuple[np.ndarray, float]:
        """Combine sub_seq 0 and 1 into a full 160-sample buffer"""
        packet0 = self.pending_packets[seq][0]
        packet1 = self.pending_packets[seq][1]
        
        # Concatenate audio: [4 channels][80 samples] + [4 channels][80 samples]
        audio_int32 = np.zeros((self.NUM_CHANNELS, self.SAMPLES_PER_BUFFER), dtype=np.int32)
        for ch in range(self.NUM_CHANNELS):
            audio_int32[ch, 0:80] = packet0.audio[ch]
            audio_int32[ch, 80:160] = packet1.audio[ch]
            
        # Convert int32 to float32 normalized to [-1.0, 1.0]
        # INMP441 outputs 24-bit left-aligned in 32-bit I2S words (MSBs)
        # Divide by 256 to shift right 8 bits, then by 2^23 to normalize signed 24-bit range
        audio_float = audio_int32.astype(np.float32) / (256.0 * 2**23)
        
        # Convert IMU raw value to °/s
        imu_raw = packet0.imu_yaw_rate  # Same for both sub-packets
        yaw_rate_dps = imu_raw / 131.0
        
        # Add to circular buffer
        self._append_to_buffer(audio_float)
        
        # Store IMU data with timestamp
        timestamp = packet0.timestamp_us
        self.yaw_rate_buffer.append((timestamp, yaw_rate_dps))
        
        return audio_float, yaw_rate_dps
        
    def _append_to_buffer(self, audio: np.ndarray):
        """Append 160 samples to circular buffer"""
        write_end = self.buffer_write_pos + self.SAMPLES_PER_BUFFER
        
        if write_end <= self.audio_buffer.shape[1]:
            # No wrap
            self.audio_buffer[:, self.buffer_write_pos:write_end] = audio
        else:
            # Wrap around
            first_chunk = self.audio_buffer.shape[1] - self.buffer_write_pos
            self.audio_buffer[:, self.buffer_write_pos:] = audio[:, :first_chunk]
            self.audio_buffer[:, :self.SAMPLES_PER_BUFFER - first_chunk] = audio[:, first_chunk:]
            
        self.buffer_write_pos = write_end % self.audio_buffer.shape[1]
        
    def get_recent_audio(self, duration_sec: float) -> np.ndarray:
        """
        Extract the most recent `duration_sec` seconds of audio.
        
        Returns:
            Array of shape (num_channels, num_samples)
        """
        num_samples = int(duration_sec * self.SAMPLE_RATE)
        num_samples = min(num_samples, self.audio_buffer.shape[1])
        
        # Read backwards from write position
        read_start = (self.buffer_write_pos - num_samples) % self.audio_buffer.shape[1]
        
        if read_start < self.buffer_write_pos:
            # No wrap
            return self.audio_buffer[:, read_start:self.buffer_write_pos].copy()
        else:
            # Wrap around
            first_chunk = self.audio_buffer[:, read_start:]
            second_chunk = self.audio_buffer[:, :self.buffer_write_pos]
            return np.concatenate([first_chunk, second_chunk], axis=1)
```

**Key Implementation Notes:**
- Sub-packets with same `seq` are matched and concatenated
- Audio conversion: int32 → float32 → normalized [-1, 1]
- Circular buffer allows extracting recent audio for windowing
- IMU data stored with timestamps for interpolation

**Testing:**
```python
def test_buffer_reassembly():
    manager = BufferManager()
    
    # Create two sub-packets with same seq
    packet0 = AudioIMUSubPacket(seq=1, timestamp_us=1000, imu_yaw_rate=131, 
                                sub_seq=0, pad=0, audio=[[100]*80]*4)
    packet1 = AudioIMUSubPacket(seq=1, timestamp_us=1000, imu_yaw_rate=131, 
                                sub_seq=1, pad=0, audio=[[200]*80]*4)
    
    # First sub-packet returns None
    result = manager.add_packet(packet0)
    assert result is None
    
    # Second sub-packet returns complete buffer
    result = manager.add_packet(packet1)
    assert result is not None
    audio, yaw_rate = result
    assert audio.shape == (4, 160)
    assert abs(yaw_rate - 1.0) < 0.01  # 131 / 131.0 = 1.0 °/s
```

---

### 4. audio/beamformer.py

**Purpose:** Delay-and-sum beamforming for direction detection

```python
import numpy as np
from scipy.interpolate import interp1d
import logging

class DelayAndSumBeamformer:
    """
    4-microphone circular array delay-and-sum beamformer.
    
    Mic positions (assuming 0.15m radius collar):
    - Mic 0: Front (0°)
    - Mic 1: Right (90°)
    - Mic 2: Back (180°)
    - Mic 3: Left (270°)
    """
    
    SPEED_OF_SOUND = 343.0  # m/s at 20°C
    SAMPLE_RATE = 16000
    
    def __init__(self, radius_m: float = 0.15, num_beams: int = 24):
        """
        Args:
            radius_m: Radius of microphone ring in meters
            num_beams: Number of steering directions (evenly spaced 0-360°)
        """
        self.radius = radius_m
        self.num_beams = num_beams
        self.logger = logging.getLogger(__name__)
        
        # Steering angles (0°, 15°, 30°, ..., 345°)
        self.steering_angles = np.linspace(0, 360, num_beams, endpoint=False)
        
        # Microphone positions in radians (Front, Right, Back, Left)
        self.mic_angles_rad = np.array([0, 0.5*np.pi, np.pi, 1.5*np.pi])
        
        # Precompute delay tables
        self.delay_samples = self._compute_delay_table()
        
        self.logger.info(f"Beamformer initialized: {num_beams} beams, radius={radius_m}m")
        
    def _compute_delay_table(self) -> np.ndarray:
        """
        Compute delay in samples for each (beam, mic) pair.
        
        For a plane wave arriving from angle θ, the delay at mic i is:
            Δt_i = (r/c) * cos(θ - φ_i)
        where φ_i is the mic's angular position.
        
        Returns:
            Array of shape (num_beams, 4) with delays in samples
        """
        delays = np.zeros((self.num_beams, 4))
        
        for beam_idx, angle_deg in enumerate(self.steering_angles):
            angle_rad = np.deg2rad(angle_deg)
            
            for mic_idx, mic_angle in enumerate(self.mic_angles_rad):
                # Delay in seconds
                delay_sec = (self.radius / self.SPEED_OF_SOUND) * np.cos(angle_rad - mic_angle)
                # Convert to samples
                delays[beam_idx, mic_idx] = delay_sec * self.SAMPLE_RATE
                
        # Make delays relative to mic 0 (Front) for each beam
        delays -= delays[:, 0:1]
        
        return delays
        
    def steer_beam(self, audio: np.ndarray, beam_idx: int) -> np.ndarray:
        """
        Apply delay-and-sum for a specific steering direction.
        
        Args:
            audio: Array of shape (4, num_samples)
            beam_idx: Index of steering angle (0 to num_beams-1)
            
        Returns:
            Beamformed mono audio of shape (num_samples,)
        """
        num_samples = audio.shape[1]
        delays = self.delay_samples[beam_idx]
        
        # Apply fractional delays using linear interpolation
        delayed_channels = []
        for ch in range(4):
            delay_samples = delays[ch]
            
            if abs(delay_samples) < 0.01:  # No delay needed
                delayed_channels.append(audio[ch])
            else:
                # Create time-shifted version via interpolation
                original_times = np.arange(num_samples)
                shifted_times = original_times - delay_samples
                
                # Clamp to valid range
                shifted_times = np.clip(shifted_times, 0, num_samples - 1)
                
                # Interpolate
                interp_func = interp1d(original_times, audio[ch], 
                                      kind='linear', fill_value='extrapolate')
                delayed = interp_func(shifted_times)
                delayed_channels.append(delayed)
                
        # Sum all delayed channels
        beamformed = np.sum(delayed_channels, axis=0) / 4.0
        
        return beamformed
        
    def compute_all_beams(self, audio: np.ndarray) -> np.ndarray:
        """
        Compute beamformed output for all steering directions.
        
        Args:
            audio: Array of shape (4, num_samples)
            
        Returns:
            Array of shape (num_beams, num_samples)
        """
        beams = np.zeros((self.num_beams, audio.shape[1]))
        
        for beam_idx in range(self.num_beams):
            beams[beam_idx] = self.steer_beam(audio, beam_idx)
            
        return beams
        
    def detect_directions(self, audio: np.ndarray, top_k: int = 3) -> list:
        """
        Detect prominent sound directions from multi-channel audio.
        
        Args:
            audio: Array of shape (4, num_samples)
            top_k: Number of top directions to return
            
        Returns:
            List of (angle_deg, energy) tuples, sorted by energy
        """
        # Compute all beams
        beams = self.compute_all_beams(audio)
        
        # Compute energy per beam
        energies = np.sum(beams**2, axis=1)
        
        # Smooth energies (moving average across angles)
        window_size = 3
        kernel = np.ones(window_size) / window_size
        energies_smooth = np.convolve(energies, kernel, mode='same')
        
        # Find top-k peaks
        top_indices = np.argsort(energies_smooth)[-top_k:][::-1]
        
        # Return as (angle, energy) tuples
        detections = [
            (self.steering_angles[idx], energies_smooth[idx])
            for idx in top_indices
        ]
        
        # Filter out weak detections (below noise floor)
        noise_floor = np.median(energies_smooth)
        threshold = noise_floor * 2.0  # 2x median as threshold
        
        detections = [(angle, energy) for angle, energy in detections 
                      if energy > threshold]
        
        self.logger.debug(f"Detected {len(detections)} directions: {detections}")
        
        return detections
```

**Key Implementation Notes:**
- Delay calculation assumes plane wave from far field
- Fractional delays implemented with linear interpolation
- Energy smoothing reduces angular jitter
- Threshold based on median energy (noise floor)

**Testing:**
```python
def test_beamformer_synthetic():
    beamformer = DelayAndSumBeamformer(radius_m=0.15, num_beams=24)
    
    # Generate synthetic signal from 90° (right side)
    duration = 0.1  # 100ms
    sample_rate = 16000
    num_samples = int(duration * sample_rate)
    
    # Create impulse at different times for each mic
    audio = np.zeros((4, num_samples))
    
    # Calculate expected delays for 90° source
    source_angle = np.deg2rad(90)
    mic_angles = beamformer.mic_angles_rad
    
    for mic_idx, mic_angle in enumerate(mic_angles):
        delay_sec = (beamformer.radius / beamformer.SPEED_OF_SOUND) * \
                    np.cos(source_angle - mic_angle)
        delay_samples = int(delay_sec * sample_rate)
        
        # Place impulse at delayed position
        impulse_pos = 800 + delay_samples
        if 0 <= impulse_pos < num_samples:
            audio[mic_idx, impulse_pos] = 1.0
            
    # Detect directions
    detections = beamformer.detect_directions(audio, top_k=3)
    
    # Should detect peak near 90°
    best_angle, best_energy = detections[0]
    assert abs(best_angle - 90) < 20  # Within ±20° tolerance
```

---

### 5. classification/yamnet_classifier.py

**Purpose:** YAMNet sound classification

```python
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub
import logging

class YAMNetClassifier:
    """
    Wrapper for YAMNet sound classification model.
    
    YAMNet expects 16kHz mono audio and uses 0.96s windows internally.
    """
    
    MODEL_URL = 'https://tfhub.dev/google/yamnet/1'
    SAMPLE_RATE = 16000
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.logger.info("Loading YAMNet model from TensorFlow Hub...")
        
        # Load model
        self.model = hub.load(self.MODEL_URL)
        
        # Load class names from CSV
        # YAMNet provides class_map_path() which returns path to yamnet_class_map.csv
        class_map_path = self.model.class_map_path().numpy().decode('utf-8')
        
        self.class_names = []
        import csv
        with open(class_map_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.class_names.append(row['display_name'])
        
        self.logger.info(f"YAMNet loaded successfully ({len(self.class_names)} classes)")
        
    def classify(self, audio: np.ndarray, top_k: int = 5) -> list:
        """
        Classify audio and return top-k predictions.
        
        Args:
            audio: Mono audio array, 16kHz sample rate
            top_k: Number of top classes to return
            
        Returns:
            List of (class_name, confidence) tuples
        """
        # Ensure audio is 1D and float32
        if audio.ndim > 1:
            audio = audio.flatten()
        audio = audio.astype(np.float32)
        
        # YAMNet expects audio in [-1.0, 1.0] range (already normalized)
        
        # Run inference
        scores, embeddings, spectrogram = self.model(audio)
        
        # Scores shape: (num_frames, 521)
        # Average across frames for overall classification
        mean_scores = np.mean(scores.numpy(), axis=0)
        
        # Get top-k classes
        top_indices = np.argsort(mean_scores)[-top_k:][::-1]
        
        results = [
            (self.class_names[idx], float(mean_scores[idx]))
            for idx in top_indices
        ]
        
        self.logger.debug(f"YAMNet top-{top_k}: {results}")
        
        return results
        
    def classify_window(self, audio: np.ndarray, window_sec: float = 0.96, 
                       overlap: float = 0.5, top_k: int = 5) -> list:
        """
        Classify audio using sliding window with overlap.
        
        Args:
            audio: Mono audio array
            window_sec: Window size in seconds
            overlap: Overlap fraction (0.0 to 1.0)
            top_k: Number of top classes per window
            
        Returns:
            List of (timestamp, class_name, confidence) tuples
        """
        window_samples = int(window_sec * self.SAMPLE_RATE)
        hop_samples = int(window_samples * (1 - overlap))
        
        results = []
        
        for start in range(0, len(audio) - window_samples + 1, hop_samples):
            end = start + window_samples
            window = audio[start:end]
            
            # Classify this window
            predictions = self.classify(window, top_k=top_k)
            
            # Add timestamp
            timestamp_sec = start / self.SAMPLE_RATE
            for class_name, confidence in predictions:
                results.append((timestamp_sec, class_name, confidence))
                
        return results
```

**Key Implementation Notes:**
- YAMNet outputs scores per 0.96s frame; average across frames
- Model loaded once at startup (slow), then cached
- Audio must be normalized to [-1, 1] range
- Returns class names (strings) with confidence scores

**Testing:**
```python
def test_yamnet_basic():
    classifier = YAMNetClassifier()
    
    # Generate test signal (1 second of 440Hz sine wave - musical tone)
    duration = 1.0
    sample_rate = 16000
    t = np.linspace(0, duration, int(duration * sample_rate))
    audio = np.sin(2 * np.pi * 440 * t).astype(np.float32)
    
    # Classify
    results = classifier.classify(audio, top_k=5)
    
    # Should detect musical tones or similar
    print("Top predictions:", results)
    assert len(results) == 5
    for class_name, confidence in results:
        assert 0.0 <= confidence <= 1.0
```

---

### 6. classification/category_mapper.py

**Purpose:** Map YAMNet classes to user categories

```python
import yaml
import logging
from enum import IntEnum

class SoundCategory(IntEnum):
    DANGEROUS = 0
    SOCIAL = 1
    AMBIENT = 2

class CategoryMapper:
    """
    Maps YAMNet class names to user-defined categories (DANGEROUS, SOCIAL, AMBIENT).
    
    Configuration loaded from YAML file:
    
    ```yaml
    dangerous:
      - "Siren"
      - "Car horn"
      - "Emergency vehicle"
      - "Smoke detector, smoke alarm"
      - "Screaming"
      
    social:
      - "Speech"
      - "Conversation"
      - "Doorbell"
      - "Telephone"
      - "Child speech, kid speaking"
      
    ambient:
      - "Music"
      - "Musical instrument"
      - "Dog"
      - "Traffic noise"
      - "Rain"
    
    confidence_thresholds:
      dangerous: 0.3
      social: 0.4
      ambient: 0.5
    ```
    """
    
    def __init__(self, config_path: str):
        self.logger = logging.getLogger(__name__)
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            
        # Build mapping dict: {class_name: category}
        self.class_to_category = {}
        
        for class_name in config.get('dangerous', []):
            self.class_to_category[class_name.lower()] = SoundCategory.DANGEROUS
            
        for class_name in config.get('social', []):
            self.class_to_category[class_name.lower()] = SoundCategory.SOCIAL
            
        for class_name in config.get('ambient', []):
            self.class_to_category[class_name.lower()] = SoundCategory.AMBIENT
            
        # Confidence thresholds
        thresholds = config.get('confidence_thresholds', {})
        self.thresholds = {
            SoundCategory.DANGEROUS: thresholds.get('dangerous', 0.3),
            SoundCategory.SOCIAL: thresholds.get('social', 0.4),
            SoundCategory.AMBIENT: thresholds.get('ambient', 0.5)
        }
        
        self.logger.info(f"Category mapper loaded: {len(self.class_to_category)} classes mapped")
        
    def map_predictions(self, yamnet_results: list) -> list:
        """
        Map YAMNet predictions to categories and filter by confidence.
        
        Args:
            yamnet_results: List of (class_name, confidence) tuples from YAMNet
            
        Returns:
            List of (category, confidence, class_name) tuples that pass threshold
        """
        mapped = []
        
        for class_name, confidence in yamnet_results:
            # Look up category
            category = self.class_to_category.get(class_name.lower())
            
            if category is None:
                # Unknown class, skip
                continue
                
            # Check if confidence meets threshold
            threshold = self.thresholds[category]
            if confidence >= threshold:
                mapped.append((category, confidence, class_name))
                
        # Sort by confidence descending
        mapped.sort(key=lambda x: x[1], reverse=True)
        
        self.logger.debug(f"Mapped {len(mapped)} predictions to categories")
        
        return mapped
```

**Key Implementation Notes:**
- Case-insensitive class name matching
- Per-category confidence thresholds
- Returns only predictions that pass threshold
- YAML config allows easy user customization

**Testing:**
```python
def test_category_mapping():
    # Create test config
    test_config = """
dangerous:
  - "Siren"
  - "Car horn"
social:
  - "Speech"
ambient:
  - "Music"
confidence_thresholds:
  dangerous: 0.3
  social: 0.4
  ambient: 0.5
"""
    with open('/tmp/test_categories.yaml', 'w') as f:
        f.write(test_config)
        
    mapper = CategoryMapper('/tmp/test_categories.yaml')
    
    # Test predictions
    yamnet_results = [
        ("Siren", 0.85),
        ("Speech", 0.45),
        ("Music", 0.35),  # Below AMBIENT threshold (0.5)
    ]
    
    mapped = mapper.map_predictions(yamnet_results)
    
    assert len(mapped) == 2  # Music filtered out
    assert mapped[0][0] == SoundCategory.DANGEROUS
    assert mapped[1][0] == SoundCategory.SOCIAL
```

---

### 7. imu/yaw_integrator.py

**Purpose:** Integrate gyro data to track yaw angle

```python
import numpy as np
import logging
from collections import deque

class YawIntegrator:
    """
    Integrates gyroscope Z-axis (yaw rate) to compute cumulative yaw angle.
    
    Includes startup bias calibration and drift compensation.
    """
    
    CALIBRATION_DURATION_SEC = 2.0  # Hold still for 2 seconds at startup
    GYRO_SCALE = 131.0  # MPU-6050 ±250°/s range: 131 LSB per °/s
    
    def __init__(self, sample_rate: float = 100.0):
        """
        Args:
            sample_rate: Expected IMU sample rate in Hz
        """
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        self.logger = logging.getLogger(__name__)
        
        # State
        self.yaw_deg = 0.0
        self.bias_dps = 0.0
        self.calibrated = False
        
        # Calibration buffer
        self.calibration_samples = []
        self.calibration_target = int(self.CALIBRATION_DURATION_SEC * sample_rate)
        
    def add_sample(self, yaw_rate_raw: int, timestamp_us: int):
        """
        Add a new gyro sample.
        
        Args:
            yaw_rate_raw: Raw int16 gyro Z value from MPU-6050
            timestamp_us: ESP32 timestamp in microseconds
        """
        # Convert to °/s
        yaw_rate_dps = yaw_rate_raw / self.GYRO_SCALE
        
        # Calibration phase
        if not self.calibrated:
            self.calibration_samples.append(yaw_rate_dps)
            
            if len(self.calibration_samples) >= self.calibration_target:
                # Compute bias as mean
                self.bias_dps = np.mean(self.calibration_samples)
                self.calibrated = True
                self.logger.info(f"IMU calibrated: bias = {self.bias_dps:.3f} °/s "
                               f"(std = {np.std(self.calibration_samples):.3f})")
            return
            
        # Normal operation: integrate bias-corrected yaw rate
        yaw_rate_corrected = yaw_rate_dps - self.bias_dps
        self.yaw_deg += yaw_rate_corrected * self.dt
        
        # Wrap to [0, 360) range
        self.yaw_deg = self.yaw_deg % 360.0
        
    def get_yaw(self) -> float:
        """Get current yaw angle in degrees [0, 360)"""
        return self.yaw_deg
        
    def is_calibrated(self) -> bool:
        """Check if calibration is complete"""
        return self.calibrated
        
    def reset(self):
        """Reset yaw to zero (useful for re-zeroing heading)"""
        self.yaw_deg = 0.0
        self.logger.info("Yaw reset to 0°")
```

**Key Implementation Notes:**
- Bias calibration assumes user holds device still at startup
- Simple Euler integration with fixed dt
- Yaw wrapped to [0, 360°) for consistency
- No drift compensation (sufficient for 1-3 second windows)

**Testing:**
```python
def test_yaw_integration():
    integrator = YawIntegrator(sample_rate=100)
    
    # Calibration phase: simulate 2 seconds of stationary data
    for _ in range(200):
        integrator.add_sample(yaw_rate_raw=131, timestamp_us=0)  # 131 = 1°/s
        
    assert integrator.is_calibrated()
    assert abs(integrator.bias_dps - 1.0) < 0.1  # Bias should be ~1°/s
    
    # Simulate 90° right turn at 45°/s for 2 seconds
    for _ in range(200):
        integrator.add_sample(yaw_rate_raw=int(45 * 131), timestamp_us=0)
        
    # Yaw should be near 90° (44°/s corrected × 0.01s × 200 samples)
    assert abs(integrator.get_yaw() - 90) < 5
```

---

### 8. feedback/event_router.py

**Purpose:** Decision logic for icons and buzzers

```python
import logging
from dataclasses import dataclass
from typing import List, Optional
from enum import IntEnum

@dataclass
class SoundEvent:
    """Detected sound event with classification and direction"""
    angle_deg: float
    category: int  # SoundCategory
    class_name: str
    confidence: float
    timestamp: float

class EventRouter:
    """
    Routes detected sound events to appropriate feedback actions.
    
    Priority: DANGEROUS > SOCIAL > AMBIENT
    Max concurrent events: 3
    
    Buzzer behavior (customizable via user preferences):
    - DANGEROUS: Always fires buzzer (default)
    - SOCIAL: Optional, disabled by default
    - AMBIENT: Optional, disabled by default
    """
    
    MAX_CONCURRENT_EVENTS = 3
    
    ICON_TYPES = {
        # Map YAMNet class substrings to icon type IDs
        'siren': 1,
        'car': 2,
        'speech': 10,
        'doorbell': 11,
        'music': 20,
        'dog': 21,
        # ... expand as needed
    }
    
    BUZZER_PATTERNS = {
        0: 1,  # DANGEROUS: CONTINUOUS
        1: 4,  # SOCIAL: DOUBLE_TAP
        2: 2,  # AMBIENT: PULSE_FAST
    }
    
    def __init__(self, command_sender, buzzer_preferences: dict = None):
        self.command_sender = command_sender
        self.logger = logging.getLogger(__name__)
        
        # Default buzzer preferences: only DANGEROUS triggers haptic feedback
        self.buzzer_enabled = buzzer_preferences or {
            'dangerous': True,   # Always on by default
            'social': False,     # Off by default
            'ambient': False     # Off by default
        }
        
    def route_events(self, events: List[SoundEvent]):
        """
        Process detected events and send appropriate commands.
        
        Args:
            events: List of SoundEvent objects to process
        """
        # Sort by priority: category (asc) then confidence (desc)
        events_sorted = sorted(events, key=lambda e: (e.category, -e.confidence))
        
        # Take top N events
        active_events = events_sorted[:self.MAX_CONCURRENT_EVENTS]
        
        self.logger.debug(f"Routing {len(active_events)} events")
        
        for event in active_events:
            # Send display icon command (always show visual feedback)
            icon_type = self._select_icon_type(event.class_name)
            self.command_sender.display_icon(
                angle_deg=event.angle_deg,
                category=event.category,
                icon_type=icon_type
            )
            
            # Send buzzer command only if enabled for this category
            category_name = ['dangerous', 'social', 'ambient'][event.category]
            if self.buzzer_enabled.get(category_name, False):
                pattern = self.BUZZER_PATTERNS[event.category]
                self.command_sender.fire_buzzer(
                    angle_deg=event.angle_deg,
                    category=event.category,
                    pattern=pattern
                )
                self.logger.debug(f"Buzzer fired for {category_name} at {event.angle_deg}°")
            else:
                self.logger.debug(f"Buzzer skipped for {category_name} (disabled in preferences)")
                
    def _select_icon_type(self, class_name: str) -> int:
        """Select icon type based on YAMNet class name"""
        class_lower = class_name.lower()
        
        for keyword, icon_type in self.ICON_TYPES.items():
            if keyword in class_lower:
                return icon_type
                
        # Default icon
        return 0
```

**Key Implementation Notes:**
- Priority sorting ensures critical sounds get feedback first
- Icon type mapping can be extended with more class keywords
- Buzzer activation can be made user-configurable
- Active event limit prevents UI clutter

---

### 9. main.py - Main Application Loop

**Purpose:** Orchestrate all components

```python
import asyncio
import logging
import coloredlogs
import yaml
import numpy as np
from pathlib import Path

# Import all modules
from network.udp_receiver import UDPReceiver
from network.command_sender import CommandSender
from audio.buffer_manager import BufferManager
from audio.beamformer import DelayAndSumBeamformer
from classification.yamnet_classifier import YAMNetClassifier
from classification.category_mapper import CategoryMapper
from imu.yaw_integrator import YawIntegrator
from feedback.event_router import EventRouter, SoundEvent

class HearLinkHost:
    """Main application class for HearLink host computer"""
    
    def __init__(self, config_dir: Path):
        # Setup logging
        coloredlogs.install(level='INFO', 
                           fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)
        
        # Load configs
        with open(config_dir / 'audio_config.yaml') as f:
            self.audio_config = yaml.safe_load(f)
            
        # Initialize components
        self.logger.info("Initializing HearLink host...")
        
        self.buffer_manager = BufferManager()
        self.beamformer = DelayAndSumBeamformer(
            radius_m=self.audio_config['mic_array']['radius_m'],
            num_beams=self.audio_config['beamforming']['num_beams']
        )
        self.yamnet = YAMNetClassifier()
        self.category_mapper = CategoryMapper(config_dir / 'yamnet_categories.yaml')
        self.yaw_integrator = YawIntegrator(sample_rate=100)
        
        self.command_sender = CommandSender()
        
        # Load buzzer preferences from config
        buzzer_prefs = self.audio_config.get('buzzer_preferences', {
            'dangerous': True,
            'social': False,
            'ambient': False
        })
        self.event_router = EventRouter(self.command_sender, buzzer_preferences=buzzer_prefs)
        
        self.receiver = UDPReceiver(callback=self._handle_packet)
        
        # Processing state
        self.audio_window_samples = int(
            self.audio_config['processing']['window_duration_sec'] * 16000
        )
        self.process_interval_sec = self.audio_config['processing']['process_interval_sec']
        
        self.last_process_time = 0.0
        
        self.logger.info("Initialization complete")
        
    def _handle_packet(self, packet):
        """Callback for received UDP packets"""
        # Add to buffer manager
        result = self.buffer_manager.add_packet(packet)
        
        if result is not None:
            audio_buffer, yaw_rate = result
            
            # Update yaw integrator
            self.yaw_integrator.add_sample(packet.imu_yaw_rate, packet.timestamp_us)
            
    async def _processing_loop(self):
        """Main processing loop - runs periodically"""
        while True:
            await asyncio.sleep(self.process_interval_sec)
            
            # Skip if IMU not calibrated
            if not self.yaw_integrator.is_calibrated():
                self.logger.info("Waiting for IMU calibration...")
                continue
                
            # Get recent audio for processing
            audio = self.buffer_manager.get_recent_audio(
                duration_sec=self.audio_config['processing']['window_duration_sec']
            )
            
            if audio.shape[1] < self.audio_window_samples:
                continue  # Not enough data yet
                
            # Step 1: Beamforming and direction detection
            detections = self.beamformer.detect_directions(audio, top_k=3)
            
            if not detections:
                continue  # No sounds detected
                
            # Step 2: Per-direction classification
            events = []
            current_yaw = self.yaw_integrator.get_yaw()
            
            for angle_deg, energy in detections:
                # Get beamformed audio for this direction
                beam_idx = int(angle_deg / 15)  # Assuming 15° spacing
                beamformed_audio = self.beamformer.steer_beam(audio, beam_idx)
                
                # Classify
                yamnet_results = self.yamnet.classify(beamformed_audio, top_k=5)
                
                # Map to categories
                categorized = self.category_mapper.map_predictions(yamnet_results)
                
                if categorized:
                    # Take best category match
                    category, confidence, class_name = categorized[0]
                    
                    # Convert to head-relative direction
                    angle_head_relative = (angle_deg - current_yaw) % 360
                    
                    events.append(SoundEvent(
                        angle_deg=angle_head_relative,
                        category=category,
                        class_name=class_name,
                        confidence=confidence,
                        timestamp=asyncio.get_running_loop().time()
                    ))
                    
            # Step 3: Route events to feedback
            if events:
                self.event_router.route_events(events)
                
    async def run(self):
        """Start the application"""
        self.logger.info("Starting HearLink host application")
        
        # Send initial command to register with ESP32
        self.command_sender.send_command(0x10, 0, 0, 0)  # Test mode command
        
        # Start UDP receiver
        await self.receiver.start()
        
        # Start processing loop
        await self._processing_loop()

def main():
    """Entry point"""
    config_dir = Path(__file__).parent.parent / 'config'
    
    app = HearLinkHost(config_dir)
    
    try:
        asyncio.run(app.run())
    except KeyboardInterrupt:
        logging.info("Shutting down...")

if __name__ == '__main__':
    main()
```

---

## Testing Guidelines

### Critical Test Cases

**Packet Parsing** (`tests/test_packet_parsing.py`)
- Verify struct unpacking matches firmware format exactly
- Test sub-packet reassembly with seq/sub_seq
- Validate int32 → float32 conversion (÷256 normalization)

**Beamforming** (`tests/test_beamformer.py`)
- Synthetic impulse from known angles (0°, 90°, 180°, 270°)
- Verify delay calculations for 4-mic 90° spacing
- Test peak detection accuracy (±15° tolerance acceptable)

**YAMNet Integration** (`tests/test_yamnet.py`)
- Load model successfully from TensorFlow Hub
- Classify known test signals (440Hz tone, speech clip, siren)
- Verify output format (class_name, confidence pairs)

**IMU Integration** (`tests/test_yaw_integration.py`)
- Bias calibration convergence over 2 seconds
- Yaw integration accuracy for 90° simulated turn
- Angle wrapping to [0, 360°) range

**End-to-End** (`tests/test_end_to_end.py`)
- Connect to live ESP32 and receive packets
- Process real audio through full pipeline
- Verify commands sent back to ESP32

---

## Performance Targets

### Latency Budget

**Target: ~1-2 seconds from sound occurrence to feedback**

| Stage | Expected | Critical Notes |
|-------|----------|---------------|
| ESP32 capture + network | ~20-30ms | Local WiFi, minimal |
| Buffer assembly | ~10ms | Two sub-packets |
| Beamforming (24 beams) | ~50-100ms | NumPy vectorized ops |
| YAMNet (3 directions) | ~600-1200ms | **BOTTLENECK** - CPU inference |
| Command TX | ~5ms | Single UDP packet |
| **Total** | **~0.7-1.4s** | ✅ Within target |

### Optimization Notes

- **Parallel YAMNet**: Use `multiprocessing.Pool` for 3 simultaneous direction inferences
- **Selective classification**: Only run YAMNet on directions with energy > threshold
- **Beam reuse**: Cache beamformed outputs across overlapping windows
- **GPU acceleration**: Enable TensorFlow GPU if MacBook has compatible hardware (unlikely on M-series, but check)

---

## Configuration Files

### config/audio_config.yaml

```yaml
mic_array:
  radius_m: 0.15
  num_mics: 4
  
beamforming:
  num_beams: 24  # 15° angular resolution
  speed_of_sound_mps: 343.0
  
processing:
  window_duration_sec: 1.0
  process_interval_sec: 0.5  # 2 Hz processing rate
  
imu:
  sample_rate_hz: 100
  calibration_duration_sec: 2.0

# Buzzer haptic feedback preferences
buzzer_preferences:
  dangerous: true   # Always vibrate for dangerous sounds (sirens, horns, alarms)
  social: false     # Don't vibrate for social sounds by default (speech, doorbell)
  ambient: false    # Don't vibrate for ambient sounds (music, dogs, traffic)
```

### config/yamnet_categories.yaml

```yaml
dangerous:
  - "Siren"
  - "Civil defense siren"
  - "Emergency vehicle"
  - "Ambulance (siren)"
  - "Police car (siren)"
  - "Fire engine, fire truck (siren)"
  - "Car horn, honking"
  - "Smoke detector, smoke alarm"
  - "Fire alarm"
  - "Screaming"
  - "Gunshot, gunfire"
  
social:
  - "Speech"
  - "Conversation"
  - "Narration, monologue"
  - "Child speech, kid speaking"
  - "Doorbell"
  - "Door"
  - "Telephone"
  - "Telephone bell ringing"
  - "Ringtone"
  
ambient:
  - "Music"
  - "Musical instrument"
  - "Dog"
  - "Bark"
  - "Traffic noise, roadway noise"
  - "Motor vehicle (road)"
  - "Rain"
  - "Wind"
  - "Thunder"
  
confidence_thresholds:
  dangerous: 0.3
  social: 0.4
  ambient: 0.5
```

---

## Implementation Checklist for AI Agents

### ✅ Phase 1: Basic Connectivity
- [ ] Implement `UDPReceiver` with asyncio
- [ ] Parse `AudioIMUSubPacket` struct (1292 bytes)
- [ ] Implement `CommandSender` (4-byte packets)
- [ ] Test: Receive packets from ESP32, verify parsing

### ✅ Phase 2: Audio Pipeline
- [ ] Implement `BufferManager` (sub-packet reassembly, circular buffer)
- [ ] Implement `DelayAndSumBeamformer` (4-mic, 24 beams)
- [ ] Direction detection (beam energy, peak finding)
- [ ] Test: Synthetic signals from known directions

### ✅ Phase 3: Classification
- [ ] Load YAMNet from TensorFlow Hub
- [ ] Implement `CategoryMapper` (YAML config)
- [ ] Per-direction classification pipeline
- [ ] Test: Real audio samples (siren, speech, music)

### ✅ Phase 4: IMU & Feedback
- [ ] Implement `YawIntegrator` (bias calibration, integration)
- [ ] Implement `EventRouter` (priority, icons, buzzers)
- [ ] Head-relative direction updates
- [ ] Test: Simulated head turns

### ✅ Phase 5: Integration
- [ ] Implement `main.py` (async event loop)
- [ ] Connect all modules
- [ ] Configuration loading (YAML)
- [ ] Error handling and logging
- [ ] Test: End-to-end with live ESP32

---

## Quick Start Commands

```bash
# Setup environment
cd hearlink/host
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

# Run application
python src/main.py

# Run tests
pytest tests/ -v
```

---

**This guide provides complete technical specifications for AI agents to implement the HearLink host computer program. All modules include working code examples, exact data formats, and test cases.**
