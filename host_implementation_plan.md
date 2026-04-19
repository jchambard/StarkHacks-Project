# HearLink Host — Implementation Plan

## Context

The ESP32-S3 firmware already streams 4-channel 16 kHz audio plus MPU-6050 yaw-rate over UDP (port 5005, 1292-byte `AudioIMUSubPacket`, 200 pps) and accepts 4-byte commands on port 5006. The host computer program is the missing piece: it must reassemble sub-packets, beamform the mic array to locate sound sources, classify each source with YAMNet, map classes to DANGEROUS/SOCIAL/AMBIENT categories, integrate head yaw, and dispatch icon/buzzer commands back to the firmware.

Deliverable: a Python 3.10+ package under `hearlink/host/` implementable by a lesser coding agent from this plan, phased so each step is verifiable in isolation.

### Hardware layout (confirmed with user)
- **Mics (4):** on the collar, 90° ring, ~0.15 m radius → beamformer outputs angle in **collar frame**.
- **Buzzers (4):** on the collar → commands use **collar-frame** angle (no yaw subtraction).
- **IMU (MPU-6050):** on the head with the HUD → `imu_yaw_rate` stream IS head yaw rate.
- **Display:** single head-mounted OLED HUD rendering a **heading-up** compass → commands use **head-relative** angle = `(collar_angle − head_yaw) mod 360`.
- **No collar OLED exists.** Ignore any prose in the guide that implies otherwise.

### Known bugs in the reference guide (do NOT copy-paste)
1. `CommandSender.fire_buzzer` in the guide passes `100/70/40` as the `category` byte. The firmware (`network.c:317-325`, `category_to_intensity`) expects `SoundCategory` (`0/1/2`) — it looks up intensity internally. **Send the enum value.**
2. The guide's prose mentions "8 buzzers"; firmware `config.h:44` defines `NUM_BUZZERS 4`. Treat as 4.
3. `UDPReceiver.start` uses `asyncio.get_event_loop()` — the guide's own fix list says to use `asyncio.get_running_loop()`. Use the latter.
4. `main.py` subtracts yaw for **both** display and buzzer events. Only the HUD icon gets head-relative; buzzers stay collar-frame.
5. `pending_packets` in `BufferManager` grows unbounded if a sub-packet is permanently lost. Add TTL-based eviction (drop entries older than current `seq − 5`).

---

## Module build order & dependencies

```
Phase 1: Scaffolding        (no deps)
   └─> Phase 2: Network      (deps: Phase 1)
   └─> Phase 3: Audio core   (deps: Phase 1; independent of Phase 2 for unit tests)
   └─> Phase 4: IMU          (deps: Phase 1)
   └─> Phase 5: Classification (deps: Phase 1; network-independent)
   └─> Phase 6: Feedback     (deps: Phases 2, 4)
   └─> Phase 7: Main loop    (deps: ALL prior)
   └─> Phase 8: Smoke test   (deps: Phase 7 + live ESP32)
```

Every phase has a **verification checkpoint**. Do not proceed until it passes.

---

## Phase 1 — Scaffolding (Low complexity / Low risk)

### Files to create
- `hearlink/host/requirements.txt` — copy from guide §Dependencies verbatim.
- `hearlink/host/setup.py` — minimal `setuptools.setup` with `packages=find_packages('src')`, `package_dir={'': 'src'}`.
- `hearlink/host/README.md` — quick-start (venv, install, run).
- `hearlink/host/.gitignore` — `venv/`, `__pycache__/`, `*.pyc`, `.pytest_cache/`.
- `hearlink/host/src/__init__.py` and a `__init__.py` in every subpackage listed in the tree.
- `hearlink/host/src/utils/logger.py` — `get_logger(name)` wrapping `coloredlogs.install` with a single call-site in `main.py`.
- `hearlink/host/config/audio_config.yaml` — from guide §Configuration, unchanged.
- `hearlink/host/config/yamnet_categories.yaml` — from guide §Configuration, unchanged.
- `hearlink/host/config/network_config.yaml` — new: `esp32_ip`, `rx_port: 5005`, `tx_port: 5006`.
- `hearlink/host/tests/__init__.py` — empty.
- `hearlink/host/tests/conftest.py` — `sys.path.insert` the `src/` dir so tests can `from network.udp_receiver import …`.

### Verification
- `cd hearlink/host && python3 -m venv venv && source venv/bin/activate && pip install -r requirements.txt` completes with no errors on macOS (arm64 → `tensorflow-macos` + `tensorflow-metal`).
- `pytest tests/ -v` runs (zero tests, zero errors).
- `python -c "import tensorflow; import tensorflow_hub; import numpy; import yaml"` succeeds.

---

## Phase 2 — Network layer (Low complexity / Low risk)

### 2a. `src/network/udp_receiver.py`
Implement per guide §1, with these fixes:
- Use `asyncio.get_running_loop()` (not `get_event_loop()`).
- `PACKET_SIZE = 1292`, `STRUCT_FORMAT = '<IIhBB' + ('i' * 320)`.
- Expose `async start()` that creates a `DatagramProtocol` and returns the `transport` so `main.py` can close it on shutdown.
- Callback signature: `callback(packet: AudioIMUSubPacket) -> None`. Invoked from event-loop thread.
- Log rate-limited warnings on short/malformed packets (first 3 occurrences, then once per 100).

### 2b. `src/network/command_sender.py`
Implement per guide §2, **with the category-byte bug fixed**:
- `fire_buzzer(angle_deg, category: SoundCategory, pattern: BuzzerPattern)` must send `category.value` (0/1/2) as the `category` byte. **Do not** send 100/70/40.
- Keep one `socket.socket(AF_INET, SOCK_DGRAM)` reused across calls.
- Add `register_with_esp32()` helper that sends `Command(type=CMD_TEST_MODE, 0, 0, 0)` — used by `main.py` at startup to bind the host IP in firmware.
- Direction encoding: `int((angle_deg % 360) * 256 / 360) & 0xFF`.

### Tests — `tests/test_packet_parsing.py`
- Round-trip: `struct.pack` known values → `_parse_packet` → assert seq, sub_seq, timestamp, yaw_rate, `audio[4][80]` shape and values match.
- Edge case: packet of wrong size returns `None`/logs warning, does not crash.
- Direction encoding: `int((90 % 360) * 256 / 360) == 64`, `180 → 128`, `270 → 192`, `0 → 0`.

### Verification
- `pytest tests/test_packet_parsing.py -v` passes.
- Manual socket loopback test (no ESP32 needed): script sends one synthetic 1292-byte packet to `127.0.0.1:5005`, receiver prints parsed contents.

---

## Phase 3 — Audio core (Medium complexity / Medium risk)

### 3a. `src/audio/buffer_manager.py`
Implement per guide §3, **with TTL eviction**:
- `add_packet(packet)` returns `Optional[Tuple[np.ndarray, float]]` only when both sub-packets for a `seq` arrived.
- **New:** at top of `add_packet`, evict any `pending_packets[k]` where `k < packet.seq − 5`. Log each eviction as "incomplete buffer dropped".
- Audio normalization: `audio_int32.astype(np.float32) / (256.0 * 2**23)` (single expression, matches guide).
- Circular buffer: shape `(4, 16000 * 2)` (2 s). Write wraps at `buffer_write_pos`.
- `get_recent_audio(duration_sec)` returns a contiguous copy.
- Keep `yaw_rate_buffer` bounded (e.g., `deque(maxlen=200)` for last 2 s of samples).

### 3b. `src/audio/audio_utils.py`
Small helpers (keep thin — don't over-engineer):
- `int32_to_float(audio_int32)` → one-line wrapper over `/ (256.0 * 2**23)`.
- `hann_window(n)` — `scipy.signal.windows.hann`.
Nothing else unless a later phase actually needs it.

### 3c. `src/audio/beamformer.py`
Implement per guide §4.
- Mic angles: `np.array([0, π/2, π, 3π/2])` (Front, Right, Back, Left).
- Delay table: `(r/c) * cos(θ − φ_i) * SAMPLE_RATE`, normalized to mic 0.
- `steer_beam`: linear interpolation via `scipy.interpolate.interp1d(kind='linear', fill_value='extrapolate')`. Vectorize over all mics but one beam at a time (guide code is fine).
- `detect_directions`: energy-per-beam → 3-tap moving-average smoothing → top-k by smoothed energy → filter below `2 × median`.
- **Return beamformed audio alongside angles** — modify the guide's signature to
  `detect_directions(audio, top_k) -> list[tuple[angle_deg, energy, beam_audio_1d]]`.
  This caches the beam output so Phase 7 can feed YAMNet without re-steering. Saves ~1.5× compute.

### Tests — `tests/test_buffer.py` and `tests/test_beamformer.py`
- Buffer reassembly: two sub-packets with same seq → complete buffer; assert shape `(4, 160)`, correct yaw conversion (raw 131 → 1.0 °/s).
- Buffer TTL: inject sub_seq=0 at seq=1, then sub_seq=0 at seq=10 — assert seq=1 is evicted and `pending_packets` does not contain it.
- Buffer circular: write > 2 s of packets, assert `get_recent_audio(1.0)` returns the last 16 000 samples per channel.
- Beamformer synthetic impulse from 0°, 90°, 180°, 270° — assert detected peak within ±15°.
- Noise-only input (white noise on all channels, same seed) → `detect_directions` returns empty list after threshold filter (energy isotropic).

### Verification
- All Phase 2 + Phase 3 tests pass.

---

## Phase 4 — IMU (Low complexity / Low risk)

### 4. `src/imu/yaw_integrator.py`
Implement per guide §7. Notes:
- `GYRO_SCALE = 131.0`.
- Bias calibration = `np.mean` of first 200 samples (2 s at 100 Hz). Log bias and std; warn if std > 5 °/s (user moved during cal).
- Euler integration with fixed `dt = 0.01`. Wrap to `[0, 360)` via `% 360`.
- Expose `is_calibrated()`, `get_yaw()`, `reset()`, and `progress()` (returns `samples_collected / target`) so `main.py` can show a startup countdown.

### `src/imu/calibration.py`
The guide lists this file but the logic lives entirely inside `YawIntegrator`. Skip this file or leave a one-line `from .yaw_integrator import YawIntegrator` re-export. Don't invent a second module.

### Tests — `tests/test_yaw_integration.py`
- Calibration: feed 200 samples of raw 131 → `is_calibrated() is True`, `bias_dps ≈ 1.0`.
- Integration: after cal, feed 200 samples of raw `int(45 * 131)` → yaw ≈ 88° (44 °/s bias-corrected × 2 s).
- Wrap: drive yaw past 360 → wraps to near 0.

### Verification
- `pytest tests/test_yaw_integration.py -v` passes.

---

## Phase 5 — Classification (Medium complexity / Medium-High risk)

Risk comes from the TF Hub model download and platform-specific TF wheel install, not the code itself.

### 5a. `src/classification/yamnet_classifier.py`
Implement per guide §5 with the `class_map_path()` + CSV fix already present. Additional notes:
- Load model in `__init__`. This is ~5 s cold. Log start/end.
- `classify(audio, top_k=5)`: assume audio is mono float32 in [-1, 1]. If stereo slips through, `.flatten()` + warn.
- `mean_scores = np.mean(scores.numpy(), axis=0)` — guide is correct.
- Return `list[tuple[str, float]]`.

### 5b. `src/classification/category_mapper.py`
Implement per guide §6. No changes; case-insensitive match, per-category threshold.

### Tests — `tests/test_category_mapper.py`
- Write YAML with known classes to a tmp path, load, call `map_predictions([("Siren", 0.85), ("Music", 0.35), …])`.
- Assert only classes above threshold survive, sorted by confidence desc.
- Case-insensitivity: `"siren"` and `"Siren"` both map.

### Tests — `tests/test_yamnet_integration.py` (**marked `@pytest.mark.slow`**)
- Skip by default via `pytest -m "not slow"`.
- When run: load classifier, classify 1 s of 440 Hz sine, assert top-1 confidence > 0 and is in the 521-class list.

### Verification
- Fast tests pass in < 2 s: `pytest -m "not slow"`.
- Slow test passes when explicitly run: `pytest -m slow`.

---

## Phase 6 — Feedback (Low-Medium complexity / Low-Medium risk)

### 6a. `src/feedback/event_router.py`
Implement per guide §8 with these adjustments:
- `SoundEvent` gets **two** angle fields:
  `angle_collar_deg: float` (for buzzer) and `angle_head_rel_deg: float` (for HUD icon).
  The caller in `main.py` fills both.
- `route_events(events)`:
  - `self.command_sender.display_icon(angle_head_rel_deg, category, icon_type)` — HUD uses head-relative.
  - `self.command_sender.fire_buzzer(angle_collar_deg, category, pattern)` — buzzer uses collar-frame.
- Buzzer preferences default: `{dangerous: True, social: False, ambient: False}`.

### 6b. `src/feedback/display_manager.py` — **v1 minimal**
Fire-and-forget; firmware's OLED renderer owns the icon lifetime. Implementation:
- A single method: `emit(event: SoundEvent)` that just delegates to `event_router.route_events([event])`.
- No active-icon tracking, no re-sending on head turn, no `CMD_CLEAR_ALL`. Document this decision in a short class docstring so the next iteration knows why it's so thin.

### Tests — `tests/test_event_router.py`
- Fake `command_sender` capturing calls.
- Two events, one DANGEROUS + one AMBIENT → both call `display_icon`, only DANGEROUS calls `fire_buzzer` (default prefs).
- Four events → only top 3 are routed (`MAX_CONCURRENT_EVENTS = 3`).
- Priority: AMBIENT (0.9) + DANGEROUS (0.4) → DANGEROUS routed first.

### Verification
- `pytest tests/test_event_router.py -v` passes.

---

## Phase 7 — Main orchestration (Medium complexity / High risk)

### 7. `src/main.py`
Implement per guide §9 with these corrections and additions:

```python
def _handle_packet(self, packet):
    complete = self.buffer_manager.add_packet(packet)
    # Gate yaw integration to sub_seq == 0 only — avoids double-counting
    # because both sub-packets share the same imu_yaw_rate value.
    if packet.sub_seq == 0:
        self.yaw_integrator.add_sample(packet.imu_yaw_rate, packet.timestamp_us)
```

**Correction 1:** Call `add_sample` on `sub_seq == 0` only (see above). The guide's approach of calling it after buffer assembly also avoids double-counting but misses samples when `sub_seq == 1` is lost.

**Correction 2:** The guide's processing loop calls `beamformer.detect_directions` then separately re-calls `beamformer.steer_beam` for each peak. With the Phase-3c signature change, `detect_directions` already returns the cached beam audio — use it directly.

**Correction 3:** Build the `SoundEvent` with both angles:
```python
current_yaw = self.yaw_integrator.get_yaw()
for angle_c, energy, beam_audio in detections:
    results = self.yamnet.classify(beam_audio, top_k=5)
    categorized = self.category_mapper.map_predictions(results)
    if not categorized:
        continue
    category, confidence, class_name = categorized[0]
    events.append(SoundEvent(
        angle_collar_deg   = angle_c,
        angle_head_rel_deg = (angle_c - current_yaw) % 360,
        category=category,
        class_name=class_name,
        confidence=confidence,
        timestamp=asyncio.get_running_loop().time(),
    ))
self.event_router.route_events(events)
```

**Correction 4:** Startup sequence:
1. `coloredlogs` install.
2. Construct all components (loads YAMNet — ~5 s cold start).
3. `command_sender.register_with_esp32()` — sends a test-mode packet so firmware learns our IP.
4. `await receiver.start()`.
5. Wait for IMU calibration, logging progress every second: `"IMU calibrating: 45% (90/200 samples)"`.
6. Enter `_processing_loop`.

**Correction 5:** Graceful shutdown — on `CancelledError` or `KeyboardInterrupt`, call `command_sender.clear_display()` and close the UDP transport. Use `asyncio.run(app.run())` inside a top-level `try/except KeyboardInterrupt`.

### Verification
- App starts, loads YAMNet, connects to ESP32, IMU calibrates, then logs "Ready" and starts emitting events when you make noise.
- No crashes after 60 s of idle operation.
- Ctrl-C exits cleanly.

---

## Phase 8 — Smoke test (Low complexity / Low risk, requires hardware)

### 8. `hearlink/host/scripts/smoke_test.py`
Standalone script (NOT in `src/`), run manually with ESP32 powered on and WiFi joined:

1. Open UDP socket on 5005, send a `CMD_TEST_MODE` command to register host IP.
2. Wait 2 s; assert ≥ 300 packets received (expect ~400 @ 200 pps).
3. Parse a sample of packets; assert seq is monotonic, audio values within `(-2**31, 2**31)`.
4. Send `CMD_DISPLAY_ICON` at 0°, 90°, 180°, 270° with 500 ms between (user visually checks HUD compass rotates).
5. Send `CMD_FIRE_BUZZER` at each direction, DANGEROUS category, DOUBLE_TAP (user feels each buzzer fire).
6. Send `CMD_CLEAR_ALL`.

Script prints `PASS`/`FAIL` for automatable steps and prompts user for visual/haptic confirmation where needed.

### Verification
- Script prints `SMOKE TEST: PASS` when run against live hardware.

---

## Testing strategy summary

| Layer | Kind | Fixture |
|---|---|---|
| `udp_receiver`, `command_sender` | unit | synthetic bytes via `struct.pack` |
| `buffer_manager` | unit | hand-crafted `AudioIMUSubPacket` dataclasses |
| `beamformer` | unit | synthetic impulses at known angles |
| `yaw_integrator` | unit | scalar sample streams |
| `category_mapper` | unit | tmp YAML file |
| `event_router` | unit | fake `CommandSender` recording calls |
| `yamnet_classifier` | integration (`@pytest.mark.slow`) | real TF Hub load |
| full pipeline | manual | `scripts/smoke_test.py` vs. live ESP32 |

`pytest.ini` must define `markers = slow: requires TF Hub download`. Default run: `pytest -m "not slow"`.

---

## Critical files & cross-references

| File | Purpose | Reference |
|---|---|---|
| `src/network/udp_receiver.py` | Parse 1292-byte packet | guide §1 |
| `src/network/command_sender.py` | Emit 4-byte commands | guide §2, `firmware/main/network.c:317-325` |
| `src/audio/buffer_manager.py` | Reassemble + circular buffer | guide §3 |
| `src/audio/beamformer.py` | Delay-and-sum, 24 beams | guide §4 |
| `src/classification/yamnet_classifier.py` | Inference | guide §5 |
| `src/classification/category_mapper.py` | YAML-driven mapping | guide §6 |
| `src/imu/yaw_integrator.py` | Bias calibration + integration | guide §7 |
| `src/feedback/event_router.py` | Priority + dispatch | guide §8 |
| `src/feedback/display_manager.py` | Thin fire-and-forget wrapper (v1) | — |
| `src/main.py` | Orchestration | guide §9 |
| `config/*.yaml` | User-facing tuning knobs | guide §Configuration |
| `hearlink-firmware/main/network.c` | Ground truth for command packet semantics | — |
| `hearlink-firmware/main/config.h` | `NUM_BUZZERS=4`, intensity values | — |

---

## Risk & complexity matrix

| Module | Complexity | Risk | Primary risk |
|---|---|---|---|
| Scaffolding | Low | Low | TF wheel install on arm64 |
| UDPReceiver | Low | Low | asyncio event-loop lifecycle |
| CommandSender | Low | Low | Category-byte bug (documented above) |
| BufferManager | Medium | Medium | Circular buffer + TTL correctness under packet loss |
| Beamformer | Medium | Medium | Delay sign convention, interpolation edge cases |
| YAMNetClassifier | Low | Medium | TF Hub cold start, macOS GPU fallback |
| CategoryMapper | Low | Low | — |
| YawIntegrator | Low | Low | — |
| EventRouter | Low | Low | — |
| DisplayManager v1 | Low | Low | Intentionally thin by design |
| main.py | Medium | High | End-to-end async timing; graceful shutdown |
| smoke_test.py | Low | Low | Requires hardware and user to hold still during cal |

---

## Assumptions

1. Developer targets macOS arm64 — `tensorflow-macos` + `tensorflow-metal` install path is primary.
2. Python 3.10 or 3.11 (TF compatibility window).
3. ESP32 firmware is the version in this repo (`hearlink-firmware/main/`). Any protocol change requires plan revision.
4. Mic array radius 0.15 m is approximate; adjustable in `audio_config.yaml`.
5. At startup the user holds their head still for 2 s while IMU bias calibrates. This is communicated in logs.
6. "Collar-front" ≈ "head-front" at the moment bias calibration completes. This is the v1 limitation — if the user is looking sideways when the system starts, the HUD compass reference will be off by that angle until they reset. Document in README.
7. The smoke test (Phase 8) is run manually by the developer against live hardware before any follow-on development.
