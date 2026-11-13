"""The modeled speaker must stay silent until native playback qualifies."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('lp_audio', Path(__file__).resolve().parents[1] / 'render_audio.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class AudioTests(unittest.TestCase):
    def test_silence_before_and_after_playback(self):
        frames = list(module.audio_frames([{'metrics': {'music_playing': v}} for v in [False, True, False]]))
        self.assertFalse(np.frombuffer(frames[0], dtype='<i2').any())
        self.assertTrue(np.frombuffer(frames[1], dtype='<i2').any())
        self.assertFalse(np.frombuffer(frames[2], dtype='<i2').any())
        self.assertTrue(all(len(f) == 4800 for f in frames))
