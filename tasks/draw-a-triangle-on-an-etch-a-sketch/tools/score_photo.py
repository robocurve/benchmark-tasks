"""Score one fixed-mount Etch A Sketch photo. Run from any directory with --help."""

import argparse
import json
import sys
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rdb_etch_triangle import grading, layout

# Coordinate pairs are (right, up) in mm from the start dot. All tags face the same way.
MARKERS = np.array([(-44, 92), (145, 92), (145, -16), (-44, -16)], dtype=float)
TAG_MM = 12
CAMERA_HEIGHT_MM = 300
TAG_HEIGHT_MM = 2
PPMM = 10
SCREEN_MM = 1000 * np.asarray(layout.SCREEN)
OPTICAL_CENTRE = SCREEN_MM / 2 - 1000 * np.asarray(layout.START_ON_SCREEN)
# Inset one mm to avoid the bezel; the stylus stops over three mm from every edge.
GRID_ORIGIN = OPTICAL_CENTRE + (-SCREEN_MM[0] / 2 + 1, SCREEN_MM[1] / 2 - 1)
GRID_SIZE = tuple(np.rint((SCREEN_MM - 2) * PPMM).astype(int) + 1)


def pixel(mm):
    return (np.asarray(mm) - GRID_ORIGIN) * (PPMM, -PPMM)


def rectify(gray):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    if ids is None or sorted(ids.ravel()) != list(range(4)):
        raise ValueError("Recapture with exactly one visible marker of each ID 0–3.")
    measured = np.stack(corners).reshape(-1, 4, 2)[np.argsort(ids.ravel())]
    if np.linalg.norm(measured - np.roll(measured, 1, axis=1), axis=2).min() < TAG_MM * 6:
        raise ValueError("Recapture at higher resolution (each black marker edge must span at least 72 px).")
    offsets = TAG_MM / 2 * np.array([(-1, 1), (1, 1), (1, -1), (-1, -1)])
    world = MARKERS[:, None, :] + offsets
    # Intersect the camera-to-tag rays with the slightly lower drawing plane.
    scale = CAMERA_HEIGHT_MM / (CAMERA_HEIGHT_MM - TAG_HEIGHT_MM)
    targets = pixel(OPTICAL_CENTRE + scale * (world - OPTICAL_CENTRE))
    measured = measured.reshape(-1, 2)
    targets = targets.reshape(-1, 2)
    transform, _ = cv2.findHomography(measured, targets, method=0)
    if transform is None:
        raise ValueError("Cannot register the markers. Check their layout and recapture.")
    fitted = cv2.perspectiveTransform(measured[None], transform)[0]
    error = float(np.linalg.norm(fitted - targets, axis=1).max() / PPMM)
    if not np.isfinite(error) or error > .5:
        raise ValueError("Marker fit exceeds 0.5 mm. Check placement, orientation and lens correction.")
    return cv2.warpPerspective(gray, transform, GRID_SIZE, flags=cv2.INTER_LINEAR), error


def skeletonize(mask):
    """Zhang–Suen thinning with simultaneous deletions within each half-iteration."""
    image = np.pad(mask.astype(np.uint8), 1)
    centre = image[1:-1, 1:-1]
    ring = (image[:-2, 1:-1], image[:-2, 2:], image[1:-1, 2:], image[2:, 2:],
            image[2:, 1:-1], image[2:, :-2], image[1:-1, :-2], image[:-2, :-2])
    n, _, e, _, s, _, w, _ = ring
    while True:
        changed = False
        for phase in (0, 1):
            neighbours = sum(ring)
            transitions = sum((p == 0) & (q == 1) for p, q in zip(ring, ring[1:] + ring[:1]))
            remove = (centre == 1) & (neighbours >= 2) & (neighbours <= 6) & (transitions == 1)
            if phase == 0:
                remove &= (n * e * s == 0) & (e * s * w == 0)
            else:
                remove &= (n * e * w == 0) & (n * s * w == 0)
            if remove.any():
                centre[remove] = 0
                changed = True
        if not changed:
            break
    # A small even-width dot can collapse completely during parallel deletion.
    count, labels, _, centroids = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for label in np.setdiff1d(np.arange(1, count), labels[centre != 0]):
        rows, cols = np.where(labels == label)
        x, y = centroids[label]
        closest = np.argmin((rows - y) ** 2 + (cols - x) ** 2)
        centre[rows[closest], cols[closest]] = 1
    return centre.astype(bool)


def turn_cosine(vertex, pair):
    a, b = np.asarray(pair) - vertex
    return (a @ b) / (np.linalg.norm(a) * np.linalg.norm(b))


def chains(skeleton):
    """Trace adjacent pixels. At crossings pair the straightest continuations first."""
    pixels = set(zip(*np.nonzero(skeleton)))
    links = {}
    for p in sorted(pixels):
        r, c = p
        neighbours = []
        for dr, dc in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):
            q = (r + dr, c + dc)
            if q not in pixels:
                continue
            # A diagonal shortcut across a connected right angle is a spurious branch.
            if dr and dc and ((r + dr, c) in pixels or (r, c + dc) in pixels):
                continue
            neighbours.append(q)
        if not neighbours:
            yield [p]
        for q in neighbours:
            links[p, q] = None
        while len(neighbours) >= 2:
            if len(neighbours) == 2:
                a, b = neighbours
            else:
                a, b = min(combinations(neighbours, 2), key=lambda pair: turn_cosine(p, pair))
            links[p, a], links[p, b] = b, a
            neighbours.remove(a)
            neighbours.remove(b)

    # Open trails first, then closed loops. Never interpolate across a gap.
    for p, q in sorted(links, key=lambda edge: links[edge] is not None):
        if (p, q) not in links:
            continue
        path = [p]
        while q is not None and (p, q) in links:
            path.append(q)
            del links[p, q]
            p, q = q, links.pop((q, p))
        yield path


def score_photo(gray, video):
    if video not in ("pass", "fail"):
        raise ValueError("Supply the trial video check as pass or fail.")
    screen, error = rectify(gray)
    clean = cv2.medianBlur(screen, 3)
    otsu, _ = cv2.threshold(clean, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    threshold = min(int(otsu), int(np.median(clean)) - 20)
    skeleton = skeletonize(clean <= threshold)
    grade = grading.TriangleGrade((0., 0.), 0.)
    for path in chains(skeleton):
        grade.anchor = None
        for point in GRID_ORIGIN + np.asarray(path)[:, ::-1] / (PPMM, -PPMM):
            grade.sample(point)
    result = grade.drawing_result()
    result.update(score=result["drawn_score"] if video == "pass" else 0., video_check=video,
                  threshold_gray=threshold, marker_max_error_mm=round(error, 3),
                  skeleton_pixels=int(skeleton.sum()))
    annotation = cv2.cvtColor(screen, cv2.COLOR_GRAY2BGR)
    for side, (a, b) in grading.SIDES.items():
        a, b = pixel(a), pixel(b)
        bins = grade.covered[side]
        for i, covered in enumerate(bins):
            segment = a + np.outer([i, i + 1], (b - a) / len(bins))
            cv2.line(annotation, *np.rint(segment).astype(int), (50, 155, 50) if covered else (0, 120, 255), 3)
    annotation[skeleton] = (200, 55, 0)
    for name in ("step_at_mm", "stray_at_mm"):
        if result[name] is not None:
            at = np.rint(pixel(result[name])).astype(int)
            cv2.circle(annotation, at, 12, (0, 0, 220), 2)
            cv2.putText(annotation, name.split("_")[0], at + (12, -6), 0, .6, (0, 0, 220), 2)
    cv2.putText(annotation, f"Score {result['score']:.2f}   video {video}", (16, 35), 0, 1, (0, 0, 0), 2)
    return result, annotation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("photo", type=Path)
    parser.add_argument("--video", required=True, choices=["pass", "fail"], help="trial motion and release check")
    parser.add_argument("--out", type=Path, default=Path("photo-grade"), help="directory for score.json and annotated.png")
    args = parser.parse_args()
    gray = cv2.imread(str(args.photo), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        parser.error("Cannot read photo. Supply a JPEG or PNG.")
    try:
        result, annotation = score_photo(gray, args.video)
    except ValueError as exc:
        parser.error(str(exc))
    args.out.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.out / "annotated.png"), annotation):
        parser.error("Cannot write annotated.png.")
    report = json.dumps(result, indent=2)
    (args.out / "score.json").write_text(report + "\n")
    print(report)


if __name__ == "__main__":
    main()
