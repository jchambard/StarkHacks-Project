# HearLink

Wearable spatial sound awareness system for deaf users and distracted listeners.

5-mic collar array → beamforming + YAMNet AI → OLED glasses display + haptic buzzers.

Built for StarkHacks (36-hour hackathon).

## Structure
```
hearlink/
├── firmware/        # Raspberry Pi Pico W (MicroPython)
│   ├── src/         # Main firmware
│   ├── lib/         # MicroPython drivers
│   └── config/      # WiFi/pin config (secrets.py is gitignored)
├── host/            # Mac host pipeline (Python)
│   ├── pipeline/    # Beamformer + direction detection
│   ├── classifier/  # YAMNet sound classification
│   ├── comms/       # UDP server + Pico command client
│   └── utils/       # Shared helpers
├── docs/            # Architecture + wiring diagrams
├── tests/           # Unit + integration tests
└── tools/           # Dev utilities (audio sim, mic test)
```

## Quick Start

### Host (Mac)
```bash
cd host
pip install -r requirements.txt
python main.py
```

### Firmware (Pico W)
1. Flash MicroPython to Pico W
2. Copy `firmware/config/settings.py` → `secrets.py` and fill in WiFi/IP
3. Upload `firmware/` to Pico W via `mpremote` or Thonny
