"""
YAMNet Sound Classifier — wraps Google YAMNet via TF Hub.
Accepts 1D audio at 16kHz, returns top label + confidence + category tier.
"""
import numpy as np
import logging
from dataclasses import dataclass
from typing import Optional

log = logging.getLogger(__name__)

YAMNET_URL = "https://tfhub.dev/google/yamnet/1"
SAMPLE_RATE = 16000
WINDOW_SAMPLES = int(0.96 * SAMPLE_RATE)  # 15,360

DANGEROUS = {"Siren", "Car alarm", "Alarm", "Fire alarm", "Screaming", "Car horn", "Honking", "Emergency vehicle"}
SOCIAL    = {"Speech", "Conversation", "Doorbell", "Telephone bell ringing", "Narration, monologue"}


@dataclass
class ClassificationResult:
    label: str
    confidence: float
    category: str  # "danger" | "social" | "ambient"


class YAMNetClassifier:
    def __init__(self, confidence_threshold: float = 0.3):
        self.confidence_threshold = confidence_threshold
        self._model = None
        self._class_names = None

    def _load_model(self):
        if self._model is None:
            import tensorflow_hub as hub
            import tensorflow as tf
            log.info("Loading YAMNet from TF Hub...")
            self._model = hub.load(YAMNET_URL)
            self._class_names = [c.decode() for c in self._model.class_names.numpy()]
            log.info(f"YAMNet ready. {len(self._class_names)} classes.")

    def classify(self, audio: np.ndarray) -> Optional[ClassificationResult]:
        self._load_model()
        import tensorflow as tf
        audio = np.pad(audio, (0, max(0, WINDOW_SAMPLES - len(audio))))[:WINDOW_SAMPLES].astype(np.float32)
        scores, _, _ = self._model(audio)
        mean_scores = tf.reduce_mean(scores, axis=0).numpy()
        idx = int(np.argmax(mean_scores))
        conf = float(mean_scores[idx])
        if conf < self.confidence_threshold:
            return None
        label = self._class_names[idx]
        category = "danger" if label in DANGEROUS else ("social" if label in SOCIAL else "ambient")
        return ClassificationResult(label=label, confidence=conf, category=category)
