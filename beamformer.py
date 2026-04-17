"""
Delay-and-Sum Beamformer

Steers a listening beam across 360 by applying fractional sample delays
to each mic channel and summing. All beam angles are computed from the
same simultaneous audio capture — no hardware rotation required.
"""
import numpy as np

SPEED_OF_SOUND = 343.0  # m/s


class Beamformer:
    def __init__(self, num_mics: int, mic_spacing: float, sample_rate: int, num_angles: int = 72):
        self.num_mics = num_mics
        self.mic_spacing = mic_spacing
        self.sample_rate = sample_rate
        self.num_angles = num_angles
        self.angles = np.linspace(0, 360, num_angles, endpoint=False)
        self.mic_positions = self._compute_mic_positions()
        self.delay_samples = self._precompute_delays()

    def _compute_mic_positions(self) -> np.ndarray:
        """Evenly space mics around a circle."""
        angles = np.linspace(0, 2 * np.pi, self.num_mics, endpoint=False)
        radius = (self.mic_spacing * self.num_mics) / (2 * np.pi)
        return np.stack([radius * np.cos(angles), radius * np.sin(angles)], axis=1)

    def _precompute_delays(self) -> np.ndarray:
        """Precompute delay (in samples) for each (angle, mic) pair."""
        delays = np.zeros((self.num_angles, self.num_mics))
        for i, angle_deg in enumerate(self.angles):
            direction = np.array([np.cos(np.deg2rad(angle_deg)), np.sin(np.deg2rad(angle_deg))])
            for j, pos in enumerate(self.mic_positions):
                delays[i, j] = (np.dot(pos, direction) / SPEED_OF_SOUND) * self.sample_rate
        return delays

    def apply_beam(self, audio: np.ndarray, angle_idx: int) -> np.ndarray:
        """Apply delay-and-sum for one angle. audio: (num_mics, num_samples)"""
        num_samples = audio.shape[1]
        output = np.zeros(num_samples)
        for mic_idx in range(self.num_mics):
            shift = int(np.round(self.delay_samples[angle_idx, mic_idx]))
            output += np.roll(audio[mic_idx], -shift)
        return output / self.num_mics

    def compute_energy_map(self, audio: np.ndarray) -> np.ndarray:
        """RMS energy for every beam angle. Returns shape (num_angles,)"""
        return np.array([
            np.sqrt(np.mean(self.apply_beam(audio, i) ** 2))
            for i in range(self.num_angles)
        ])
