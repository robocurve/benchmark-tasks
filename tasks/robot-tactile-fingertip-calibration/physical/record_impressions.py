#!/usr/bin/env python3
"""Record and score calibration impressions from the physical sensor camera."""

import argparse
from dataclasses import asdict, dataclass
import json
from math import hypot, isfinite, pi, sqrt
from pathlib import Path

import cv2
import numpy as np


PIXELS_PER_MM = 10
PAD_WIDTH_MM, PAD_HEIGHT_MM = 26, 30
OUTPUT_SIZE = (PAD_WIDTH_MM * PIXELS_PER_MM + 1, PAD_HEIGHT_MM * PIXELS_PER_MM + 1)
REGIONS_MM = ((-6, -5), (6, -5), (0, 6))
FIDUCIALS_MM = ((-12, -13), (12, -13), (12, 13), (-12, 13))
HOLD_SECONDS = 0.4
MAX_FRAME_GAP = 0.075
BASELINE_FRAMES = 30
TRIAL_SECONDS = 31.0


def to_pixels(point):
    return (
        (point[0] + PAD_WIDTH_MM / 2) * PIXELS_PER_MM,
        (point[1] + PAD_HEIGHT_MM / 2) * PIXELS_PER_MM,
    )


def rectify_matrix(calibration, frame_size):
    points = np.asarray(calibration['camera_pixels'], dtype=np.float32)
    if points.shape != (4, 2) or not np.isfinite(points).all():
        raise ValueError('calibration needs four finite camera pixel pairs')
    if np.any(points < 0) or np.any(points >= frame_size):
        raise ValueError('calibration points are outside the captured frame')
    if not cv2.isContourConvex(points) or abs(cv2.contourArea(points)) < 100.0:
        raise ValueError('calibration points must outline the pad in physical corner order')
    target = np.asarray([to_pixels(p) for p in FIDUCIALS_MM], dtype=np.float32)
    return cv2.getPerspectiveTransform(points, target)


@dataclass(frozen=True)
class Patch:
    x_mm: float
    y_mm: float
    diameter_mm: float

    def region(self):
        if 4.7 <= self.diameter_mm <= 6.5:
            for index, (x, y) in enumerate(REGIONS_MM, start=1):
                if hypot(self.x_mm - x, self.y_mm - y) <= 2.3:
                    return index
        return None


class PatchDetector:
    def __init__(self, unloaded, threshold):
        self.background = np.median(unloaded, axis=0).astype(np.uint8)
        self.threshold = threshold
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def measure(self, image):
        difference = np.max(cv2.absdiff(image, self.background), axis=2)
        mask = np.where(difference >= self.threshold, 255, 0).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [shape for shape in contours if cv2.contourArea(shape) >= PIXELS_PER_MM**2]
        if len(contours) != 1:
            return None
        (shape,) = contours
        x, y, width, height = cv2.boundingRect(shape)
        if x == 0 or y == 0 or x + width == mask.shape[1] or y + height == mask.shape[0]:
            return None
        area = cv2.contourArea(shape)
        perimeter = cv2.arcLength(shape, closed=True)
        if 4 * pi * area / perimeter**2 < 0.65:
            return None
        center = cv2.moments(shape)
        return Patch(
            center['m10'] / area / PIXELS_PER_MM - PAD_WIDTH_MM / 2,
            center['m01'] / area / PIXELS_PER_MM - PAD_HEIGHT_MM / 2,
            2 * sqrt(area / pi) / PIXELS_PER_MM,
        )


def save_image(path, image):
    if not cv2.imwrite(str(path), image):
        raise OSError(f'cannot write {path}')


class Recorder:
    def __init__(self, output):
        self.output = output
        self.accepted = {}
        self.region = None
        self.samples = []
        self.images = []

    def add(self, timestamp, patch, raw, rectified):
        region = patch.region() if patch else None
        gap = timestamp - self.samples[-1]['time_s'] if self.samples else 0.0
        if region != self.region or gap > MAX_FRAME_GAP or region is None:
            self.region = region
            self.samples.clear()
            self.images.clear()
        if region is None or region in self.accepted:
            return
        self.samples.append({'time_s': timestamp, **asdict(patch)})
        self.images.append(rectified)
        elapsed = timestamp - self.samples[0]['time_s']
        if elapsed + 1e-9 < HOLD_SECONDS:
            return

        name = f'region-{region}'
        save_image(self.output / f'{name}.png', raw)
        save_image(self.output / f'{name}-rectified.png', rectified)
        np.savez_compressed(self.output / f'{name}-hold.npz', bgr=np.stack(self.images),
                            time_s=[sample['time_s'] for sample in self.samples])
        self.accepted[region] = {'camera_image': f'{name}.png', 'hold_seconds': round(elapsed, 6),
                                 'samples': self.samples.copy()}
        print(f'Saved region {region} after {elapsed:.3f} s of continuous valid contact', flush=True)


def camera_frames(capture, *, live):
    previous = None
    while True:
        ok, image = capture.read()
        if not ok:
            if live:
                raise OSError('camera stopped delivering frames')
            return
        timestamp = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        if not isfinite(timestamp) or timestamp < 0 or (previous is not None and timestamp <= previous):
            raise ValueError('capture backend did not supply strictly increasing frame timestamps')
        previous = timestamp
        yield timestamp, image


def capture_trial(frames, params, output, *, live, diagnose=False):
    first_timestamp, first = next(frames)
    size = first.shape[1], first.shape[0]
    matrix = rectify_matrix(params, size)
    threshold = params['difference_threshold']
    if type(threshold) is not int or not 8 <= threshold <= 80:
        raise ValueError('difference_threshold must be an integer between 8 and 80')

    def rectify(image):
        if (image.shape[1], image.shape[0]) != size:
            raise ValueError('capture frame dimensions changed during the trial')
        return cv2.warpPerspective(image, matrix, OUTPUT_SIZE)

    baseline = [rectify(first)]
    started = first_timestamp
    while len(baseline) < BASELINE_FRAMES:
        started, frame = next(frames)
        baseline.append(rectify(frame))
    detector = PatchDetector(baseline, threshold)
    if any(detector.measure(image) for image in baseline):
        raise ValueError('baseline did not stay unloaded and stable')

    save_image(output / 'unloaded.png', detector.background)
    recorder = Recorder(output)
    measurements = []
    print('Armed. Start the robot trial now.', flush=True)
    end_reason = 'camera_stopped' if live else 'replay_finished'
    try:
        for timestamp, frame in frames:
            elapsed = timestamp - started
            if elapsed > TRIAL_SECONDS:
                end_reason = 'time_limit'
                break
            image = rectify(frame)
            patch = detector.measure(image)
            if diagnose and patch:
                measurements.append({'time_s': elapsed, **asdict(patch)})
            recorder.add(elapsed, patch, frame, image)
            if len(recorder.accepted) == len(REGIONS_MM):
                end_reason = 'all_regions_captured'
                break
    finally:
        report = {
            'source': 'V4L2 frame timestamps' if live else 'video frame timestamps',
            'end_reason': end_reason,
            'score': len(recorder.accepted) / len(REGIONS_MM),
            'settings': params,
            'regions': recorder.accepted,
        }
        if diagnose:
            report['detected_patches'] = measurements
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def open_capture(device, video):
    if video:
        capture = cv2.VideoCapture(str(video), cv2.CAP_FFMPEG)
    else:
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        capture.set(cv2.CAP_PROP_FPS, 30)
    if not capture.isOpened():
        capture.release()
        raise OSError(f'cannot open {video or device}')
    return capture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--device', help='Linux capture device, e.g. /dev/video0')
    source.add_argument('--video', type=Path, help='replay a timestamped optical-camera video')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--snapshot', type=Path, help='save an unloaded image for locating the fiducials')
    mode.add_argument('--calibration', type=Path, help='JSON camera pixels and fixed difference threshold')
    parser.add_argument('--output', type=Path, help='new evidence directory for a scored trial')
    parser.add_argument('--diagnose', action='store_true', help='log all detected patches for commissioning')
    args = parser.parse_args()
    if args.calibration and args.output is None:
        parser.error('--calibration requires --output')
    try:
        capture = open_capture(args.device, args.video)
        try:
            frames = camera_frames(capture, live=args.device is not None)
            if args.snapshot:
                _, frame = next(frames)
                save_image(args.snapshot, frame)
            else:
                params = json.loads(args.calibration.read_text())
                args.output.mkdir()
                report = capture_trial(frames, params, args.output, live=args.device is not None,
                                       diagnose=args.diagnose)
                print(f"Score {report['score']:.3f}. Evidence saved in {args.output}")
        finally:
            capture.release()
    except (OSError, StopIteration, KeyError, ValueError, cv2.error) as error:
        parser.exit(1, f'Capture failed: {error or "insufficient camera frames"}\n')


if __name__ == '__main__':
    main()
