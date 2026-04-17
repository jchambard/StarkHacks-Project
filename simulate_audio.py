"""
Audio Simulator — generates synthetic test audio and sends it to the host
pipeline as if it were coming from the Pico W. Use this to test the full
pipeline before hardware arrives.

Usage:
    python tools/simulate_audio.py --ip 127.0.0.1 --port 5005 --sound siren --angle 45
"""
import argparse
import socket
import struct
import numpy as np
import time

SAMPLE_RATE = 16000
NUM_MICS = 5
MIC_SPACING = 0.075
SPEED_OF_SOUND = 343.0
SAMPLES_PER_PACKET = 512


def generate_tone(freq: float, duration_s: float, sr: int = SAMPLE_RATE) -> np.ndarray:
    t = np.linspace(0, duration_s, int(sr * duration_s), endpoint=False)
    return (np.sin(2 * np.pi * freq * t) * 0.5).astype(np.float32)


def apply_delays(mono: np.ndarray, angle_deg: float) -> np.ndarray:
    """Simulate mic array geometry — apply delays for a sound at angle_deg."""
    angle_rad = np.deg2rad(angle_deg)
    direction = np.array([np.cos(angle_rad), np.sin(angle_rad)])
    radius = (MIC_SPACING * NUM_MICS) / (2 * np.pi)
    mic_angles = np.linspace(0, 2 * np.pi, NUM_MICS, endpoint=False)
    mic_pos = radius * np.stack([np.cos(mic_angles), np.sin(mic_angles)], axis=1)

    channels = []
    for pos in mic_pos:
        delay_s = np.dot(pos, direction) / SPEED_OF_SOUND
        delay_samples = int(delay_s * SAMPLE_RATE)
        channels.append(np.roll(mono, delay_samples))
    return np.stack(channels, axis=0)  # (5, samples)


SOUND_FREQS = {
    "siren":   [700, 900],   # alternating
    "horn":    [440],
    "speech":  [200, 300, 400],
    "alarm":   [880],
    "ambient": [100],
}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip",    default="127.0.0.1")
    parser.add_argument("--port",  type=int, default=5005)
    parser.add_argument("--sound", default="siren", choices=SOUND_FREQS.keys())
    parser.add_argument("--angle", type=float, default=0.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Simulating '{args.sound}' at {args.angle}° → {args.ip}:{args.port}")

    seq = 0
    while True:
        freq = SOUND_FREQS[args.sound][seq % len(SOUND_FREQS[args.sound])]
        mono = generate_tone(freq, SAMPLES_PER_PACKET / SAMPLE_RATE)
        multi = apply_delays(mono, args.angle)  # (5, 512)

        # Interleave channels → int16 bytes
        interleaved = multi.T.reshape(-1)  # (512*5,)
        samples_int16 = (interleaved * 32767).astype(np.int16)
        packet = struct.pack(">I", seq) + samples_int16.tobytes()
        sock.sendto(packet, (args.ip, args.port))

        seq += 1
        time.sleep(SAMPLES_PER_PACKET / SAMPLE_RATE)

if __name__ == "__main__":
    main()
