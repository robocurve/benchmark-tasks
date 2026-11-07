"""Detect lost rendering, without altering policy observations or task scores."""
import json
from pathlib import Path
import numpy as np

CAMERAS = ('top_cam', 'left_cam', 'right_cam')


class ImageMonitor:
    """The fixed overview always contains the table/fixture, even at rest.

    Three almost spatially uniform overview frames invalidate the environment.
    A stationary but textured image is valid; wrists may legitimately face a blank
    surface. This detects the observed grey-render failure, not every image fault.
    """
    def __init__(self):
        self.rows = []
        self.blank_streak = 0
        self.error = None

    def update(self, images):
        if self.error:
            raise RuntimeError(self.error)
        for camera in CAMERAS:
            rgb = images.get(camera)
            if not isinstance(rgb, np.ndarray) or rgb.shape != (176, 320, 3) or rgb.dtype != np.uint8:
                self.error = 'Invalid original camera format: ' + camera
                raise RuntimeError(self.error)
        spatial = float(images['top_cam'].astype(np.float32).std(axis=(0, 1)).mean())
        self.blank_streak = self.blank_streak + 1 if spatial < 2.0 else 0
        self.rows.append(dict(frame=len(self.rows), top_spatial_std=spatial,
                              blank_streak=self.blank_streak))
        if self.blank_streak >= 3:
            self.error = 'Original overview rendering is empty; invalidate trial, not model score'
            raise RuntimeError(self.error)

    def report(self):
        return dict(valid=self.error is None, error=self.error, frames=len(self.rows),
                    minimum_spatial_std=min((r['top_spatial_std'] for r in self.rows), default=None),
                    blank_threshold=2.0, invalid_after=3, rows=self.rows)


def install(output):
    """Check before an observation can reach a policy; never replace its images."""
    from yam_arena.inspect_embodiment import YamArenaEmbodiment
    original = YamArenaEmbodiment._observation
    monitor = ImageMonitor()
    path = Path(output) / 'image_health.json'

    def observe(self):
        observation = original(self)
        try:
            monitor.update(observation.images)
        finally:
            if monitor.error or len(monitor.rows) % 20 == 1:
                path.write_text(json.dumps(monitor.report(), indent=2) + '\n')
        return observation

    YamArenaEmbodiment._observation = observe

    def save():
        path.write_text(json.dumps(monitor.report(), indent=2) + '\n')
    return save
