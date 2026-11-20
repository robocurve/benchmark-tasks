"""Synthetic captures with measured geometry, actual marker codes and independently drawn ink."""
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

from dryrun_demo import dry_run
from score_photo import score_photo

ROOT = Path(__file__).resolve().parents[1]
A, B, C = (0, 0), (80, 0), (80, 60)
TRIANGLE = [A, B, C, A]


def capture(paths, *, perspective=False, noise=False):
    # Independent camera model: frontoparallel photo, 10 px/mm at the screen.
    def xy(points):
        return np.rint((np.asarray(points) - (-58, 111)) * (10, -10)).astype(np.int32)

    frame = np.full((1510, 2170), 95, np.uint8)
    cv2.rectangle(frame, xy((-30, -20)), xy((131.4, 94.6)), 192, -1)
    for path in paths:
        cv2.polylines(frame, [xy(path)], False, 60, 4, cv2.LINE_AA)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    centre = np.array([50.7, 37.3])
    for i, tag in enumerate(((-44, 92), (145, 92), (145, -16), (-44, -16))):
        x, y = xy(centre + (tag - centre) * 300 / 298)
        width = round(120 * 300 / 298)
        left, top = x - width // 2, y - width // 2
        frame[top-20:top+width+20, left-20:left+width+20] = 255
        frame[top:top+width, left:left+width] = cv2.aruco.generateImageMarker(dictionary, i, width)
    if perspective:
        before = np.float32([(0, 0), (2169, 0), (2169, 1509), (0, 1509)])
        after = np.float32([(148, 48), (2084, 111), (2139, 1406), (62, 1480)])
        frame = cv2.warpPerspective(frame, cv2.getPerspectiveTransform(before, after), (2170, 1510))
    if noise:
        blurred = cv2.GaussianBlur(frame, (5, 5), 1)
        rng = np.random.default_rng(43)
        frame = np.clip(blurred + rng.normal(0, 3, frame.shape), 0, 255).astype(np.uint8)
    return frame


@pytest.mark.parametrize(("paths", "expected"), [
    ([], 0.), ([[A, (60, 0)]], 0.), ([[A, B]], .25), ([[A, B, C]], .5),
    ([TRIANGLE], 1.), ([TRIANGLE, [(110, 75), (112, 76)]], .25)
])
def test_photo_bands(paths, expected):
    result, _ = score_photo(capture(paths), "pass")
    assert result["score"] == expected, result
    if len(paths) > 1:
        assert result["tier"] == 1. and result["stray_at_mm"]


@pytest.mark.parametrize(("step_mm", "score"), [(.8, 1.), (1.5, .5), (2., .5)])
def test_staircase_with_full_coverage(step_mm, score):
    path = [A, B, C]
    for x in np.arange(80., 0., -step_mm):
        next_x = max(0., x - step_mm)
        path.extend([(next_x, .75 * x), (next_x, .75 * next_x)])
    result, _ = score_photo(capture([path], perspective=True, noise=True), "pass")
    assert result["covered_mm"] == {"bottom": 80, "upright": 60, "slant": 100}
    assert not result["stray"] and result["score"] == score
    assert (result["step_at_mm"] is None) == (score == 1.)


def test_warp_blur_noise_cli(tmp_path):
    photo = tmp_path / "photo.jpg"
    cv2.imwrite(str(photo), capture([TRIANGLE], perspective=True, noise=True))
    out = tmp_path / "grade"
    result = subprocess.run([sys.executable, str(ROOT / "tools/score_photo.py"), str(photo),
                             "--video", "pass", "--out", str(out)], check=True, capture_output=True)
    report = json.loads(result.stdout)
    assert report == json.loads((out / "score.json").read_text())
    assert report["score"] == 1. and report["covered_mm"] == {"bottom": 80, "upright": 60, "slant": 100}
    assert cv2.imread(str(out / "annotated.png")).shape[2] == 3
    again, _ = score_photo(cv2.imread(str(photo), cv2.IMREAD_GRAYSCALE), "fail")
    assert again == dict(report, score=0., video_check="fail")


def test_gaps_do_not_create_credit_or_steps():
    result, _ = score_photo(capture([[A, (25, 0)], [(55, 0), B, C, A]]), "pass")
    assert result["score"] == 0. and result["covered_mm"]["bottom"] < 55


def test_missing_tag_requires_recapture():
    frame = capture([TRIANGLE])
    frame[:350, :350] = 255
    with pytest.raises(ValueError, match="marker"):
        score_photo(frame, "pass")


def test_rendered_scripted_dry_run():
    path = [A]
    run = dry_run(path_mm=path)
    assert run["score"] == 1.
    result, _ = score_photo(capture([path], perspective=True, noise=True), "pass")
    assert result["score"] == 1., result
    assert result["covered_mm"] == run["covered"]
