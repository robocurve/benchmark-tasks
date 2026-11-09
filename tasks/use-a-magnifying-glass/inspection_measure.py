"""Measure real image enlargement and five-bar legibility.

This is the photo component of the task scorer. ``scoring.py`` adds the physical
conditions and ``score_hold.py`` verifies every state of the three-second hold.
No synthetic image or geometric projection supplies the measured enlargement.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from PIL import Image

try:
    from .measure import _components, _project
except ImportError:
    from measure import _components, _project


def find_fiducials(rgb, target):
    """Locate the printed colour square anywhere in the actual camera image.

    The original static-proof helper restricted detections to the central 60%.
    Final judging has no such framing condition: use the same colour/geometry
    checks over the full image and retain centrality only as a weak tie-breaker.
    """
    image = np.asarray(rgb, dtype=float)
    height, width = image.shape[:2]
    r, g, b = np.moveaxis(image, -1, 0)
    masks = {"red": (r > 1.45*g) & (r > 1.45*b),
             "green": (g > 1.35*r) & (g > 1.25*b),
             "blue": (b > 1.4*r) & (b > 1.15*g),
             "magenta": (r > 1.4*g) & (b > 1.4*g) & (r < 1.6*b) & (b < 1.6*r)}
    bright = image.max(axis=-1) > 45
    weights = image.max(axis=-1)-image.min(axis=-1)+1
    candidates = []
    for fiducial in target["fiducials"]:
        components = _components(masks[fiducial["name"]] & bright, weights)
        components.sort(key=lambda c: np.linalg.norm(c["xy"]-[width/2, height/2]))
        candidates.append(components[:6])
    if any(not choices for choices in candidates):
        missing = [fid["name"] for fid, choices in zip(target["fiducials"], candidates) if not choices]
        raise ValueError("Missing RGB fiducial(s): " + ", ".join(missing))
    world = np.array([[*f["xy_mm"], 1] for f in target["fiducials"]])
    choices = []
    for group in itertools.product(*candidates):
        pixels = np.array([c["xy"] for c in group])
        affine = np.linalg.lstsq(world, pixels, rcond=None)[0]
        singular = np.linalg.svd(affine[:2], compute_uv=False)
        if singular[-1] < .15 or singular[0]/singular[-1] > 5:
            continue
        residual = float(np.sqrt(np.mean(np.sum((world@affine-pixels)**2, axis=1))))
        span = np.median([np.linalg.norm(pixels[i]-pixels[j]) for i, j in itertools.combinations(range(4), 2)])
        if residual > max(1.2, .04*span):
            continue
        centre_distance = np.linalg.norm(pixels.mean(axis=0)-[width/2, height/2])/min(width, height)
        choices.append((residual/max(span, 1)+centre_distance*.08, group, affine, residual))
    if not choices:
        raise ValueError("RGB fiducials do not form a coherent target quadrilateral")
    choices.sort(key=lambda item: item[0])
    _, selected, affine, residual = choices[0]
    return selected, affine, {"affine_rms_error_px": residual,
                              "coherent_quartets": len(choices),
                              "selection": "Full-image coherent colour quadrilateral; centrality only breaks ties"}


def luminance(rgb):
    return np.asarray(rgb, dtype=float)[..., :3] @ np.array([.2126, .7152, .0722]) / 255


def native_reference_samples(gray, reference, affine):
    """Read existing pixels inside the central half of a reference patch."""
    center = np.asarray(reference["center_mm"])
    half = np.asarray(reference["size_mm"]) * .25
    corners = np.array([center+[x*half[0], y*half[1]] for x, y in itertools.product((-1, 1), repeat=2)])
    projected = _project(corners, affine)
    lo = np.ceil(projected.min(axis=0)).astype(int)
    hi = np.floor(projected.max(axis=0)).astype(int)
    if np.any(lo < 0) or hi[0] >= gray.shape[1] or hi[1] >= gray.shape[0]:
        raise ValueError("Reference patch falls outside the image")
    xx, yy = np.meshgrid(np.arange(lo[0], hi[0]+1), np.arange(lo[1], hi[1]+1))
    xy = np.stack((xx.ravel(), yy.ravel()), axis=-1)
    world = (xy-affine[2]) @ np.linalg.inv(affine[:2])
    inside = np.all(np.abs(world-center) <= half+1e-10, axis=-1)
    xy = xy[inside]
    if not len(xy):
        raise ValueError("No native pixel lies inside a reference patch's central half")
    return float(np.median(gray[xy[:, 1], xy[:, 0]])), len(xy)


def count_scanline(values, locations, black, white):
    """Count actual dark/bright runs, without resampling or fitting a grating.

    Midpoint separates runs. Each counted run must also contain a native pixel
    at least 10% of the reference range away from the midpoint, so almost-flat
    gray noise cannot establish readable black bars and white gaps.
    """
    values = np.asarray(values)
    dark = values < (black+white)/2
    starts = np.flatnonzero(np.r_[True, dark[1:] != dark[:-1]])
    ends = np.r_[starts[1:], len(values)]
    runs = []
    dynamic = white-black
    for start, end in zip(starts, ends):
        is_dark = bool(dark[start])
        strong = values[start:end] <= black+.4*dynamic if is_dark else values[start:end] >= black+.6*dynamic
        runs.append({"kind": "dark" if is_dark else "bright", "pixels": int(end-start),
                     "strong_pixels": int(strong.sum()),
                     "start_xy": list(map(int, locations[start])),
                     "end_xy": list(map(int, locations[end-1]))})
    # White margins outside the two end bars are not part of the five-bar code.
    if runs and runs[0]["kind"] == "bright":
        runs.pop(0)
    if runs and runs[-1]["kind"] == "bright":
        runs.pop()
    kinds = [run["kind"] for run in runs]
    expected = ["dark" if i % 2 == 0 else "bright" for i in range(9)]
    return {"readable": bool(kinds == expected and all(run["strong_pixels"] >= 1 for run in runs)),
            "dark_bars": kinds.count("dark"), "bright_gaps": kinds.count("bright"),
            "runs": runs}


def measure_group(gray, group, affine, black, white):
    center = np.asarray(group["center_mm"])
    bar_axis = 0 if group["orientation"] == "vertical" else 1
    stripe_axis = 1-bar_axis
    image_center = _project(center, affine)
    bar_vector, stripe_vector = affine[bar_axis], affine[stripe_axis]
    varying_axis = int(np.argmax(np.abs(bar_vector)))
    other_axis = 1-varying_axis
    extent = group["size_mm"][bar_axis]/2
    central_half_extent = group["size_mm"][stripe_axis]/4
    result = {"name": group["name"], "readable": False, "scanlines": [],
              "criterion": "Five dark runs and four bright gaps, each with a strong original pixel, along every chart-aligned native-pixel transect spaced at most one pixel apart across the central half of the bar length."}
    # Follow the printed bars, rather than requiring a horizontal image row to
    # fit a rotated, narrow patch. At each integer coordinate of the dominant
    # image axis read the nearest original pixel on that straight transect.
    # No interpolation, rotated image, averaging or invented pixel is used.
    # This is deliberately conservative at resolution limits: every transect
    # must resolve the complete pattern, including intermediate offsets.
    count = max(1, int(np.ceil(2*central_half_extent*np.linalg.norm(stripe_vector))))
    for offset in np.linspace(-central_half_extent, central_half_extent, count+1):
        origin = image_center+stripe_vector*offset
        endpoints = [origin[varying_axis]+bar_vector[varying_axis]*u for u in (-extent, extent)]
        values_at = np.arange(int(np.ceil(min(endpoints))), int(np.floor(max(endpoints)))+1)
        if not len(values_at):
            continue
        locations = np.empty((len(values_at), 2), dtype=int)
        locations[:, varying_axis] = values_at
        parameters = (values_at-origin[varying_axis])/bar_vector[varying_axis]
        locations[:, other_axis] = np.floor(origin[other_axis]+bar_vector[other_axis]*parameters+.5).astype(int)
        if (np.any(locations < 0) or np.any(locations[:, 0] >= gray.shape[1])
                or np.any(locations[:, 1] >= gray.shape[0])):
            result["error"] = "Bar group falls outside the image"
            return result
        line = count_scanline(gray[locations[:, 1], locations[:, 0]], locations, black, white)
        line.update(dominant_image_axis="xy"[varying_axis], offset_along_bar_mm=float(offset))
        result["scanlines"].append(line)
    result["native_scanline_count"] = len(result["scanlines"])
    result["readable"] = bool(result["scanlines"] and all(line["readable"] for line in result["scanlines"]))
    if not result["scanlines"]:
        result["error"] = "No native-pixel transect crosses the bar group"
    return result


def measure_image(rgb, target):
    rgb = np.asarray(rgb)[..., :3]
    result = {"image_size": [int(rgb.shape[1]), int(rgb.shape[0])], "valid": False}
    try:
        selected, affine, fit = find_fiducials(rgb, target)
        gray = luminance(rgb)
        result.update(fit)
        result["fiducials"] = {f["name"]: {"xy_px": c["xy"].tolist(), "mask_pixels": c["pixels"]}
                               for f, c in zip(target["fiducials"], selected)}
        result["mm_to_pixel_affine"] = affine.tolist()
        references = {}
        for ref in target["references"]:
            value, count = native_reference_samples(gray, ref, affine)
            references[ref["name"]] = {"luminance": value, "native_pixels": count}
        black, white = references["black"]["luminance"], references["white"]["luminance"]
        result["references"] = references
        result["reference_dynamic_range"] = white-black
        if white-black < .2:
            raise ValueError("Black/white reference range is below 0.20 of full luminance")
        result["bar_groups"] = [measure_group(gray, group, affine, black, white) for group in target["bar_groups"]]
        result["valid"] = True
    except (ValueError, np.linalg.LinAlgError) as exc:
        result["error"] = str(exc)
    return result


def edge_gains(before, after):
    pairs = (("red", "green"), ("blue", "magenta"), ("red", "blue"), ("green", "magenta"))
    edges = []
    for a, b in pairs:
        p0 = np.array(before["fiducials"][a]["xy_px"])-before["fiducials"][b]["xy_px"]
        p1 = np.array(after["fiducials"][a]["xy_px"])-after["fiducials"][b]["xy_px"]
        edges.append({"pair": [a, b], "baseline_pixels": float(np.linalg.norm(p0)),
                      "candidate_pixels": float(np.linalg.norm(p1)),
                      "gain": float(np.linalg.norm(p1)/np.linalg.norm(p0))})
    return {"edges": edges, "minimum_edge_gain": min(edge["gain"] for edge in edges)}


def find_registration_marker(gray, center, half_window, expected_size_px):
    """Find a printed black square relative to its own surrounding background.

    Central swatches are viewed through glass and cannot calibrate illumination
    of outer marks. Fit a local background plane instead, then require a compact
    dark component consistent with the square's known projected dimensions.
    """
    center = np.asarray(center)
    expected_size_px = np.asarray(expected_size_px, dtype=float)
    left = max(0, int(np.floor(center[0]-half_window)))
    right = min(gray.shape[1], int(np.ceil(center[0]+half_window))+1)
    top = max(0, int(np.floor(center[1]-half_window)))
    bottom = min(gray.shape[0], int(np.ceil(center[1]+half_window))+1)
    region = gray[top:bottom, left:right]
    yy, xx = np.indices(region.shape)
    dx, dy = xx+left-center[0], yy+top-center[1]
    design = np.stack((np.ones_like(dx), dx, dy), axis=-1)
    annulus = np.maximum(np.abs(dx), np.abs(dy)) >= half_window-1.25
    if annulus.sum() < 12:
        raise ValueError("Insufficient background around outer marker")
    keep = annulus.copy()
    # Robustly discard isolated dark contamination of the outer fitting ring.
    for _ in range(3):
        coefficients = np.linalg.lstsq(design[keep], region[keep], rcond=None)[0]
        background = design@coefficients
        residual = region-background
        mad = np.median(np.abs(residual[keep]-np.median(residual[keep])))
        keep = annulus & (np.abs(residual) <= max(.06, 4*1.4826*mad))
        if keep.sum() < 12:
            raise ValueError("Outer marker background is too occluded or irregular")
    if np.sqrt(np.mean((region[keep]-background[keep])**2)) > .06:
        raise ValueError("Outer marker background does not support a local illumination model")
    if background.min() < .15:
        raise ValueError("Outer marker region is too dark for reliable registration")
    contrast = background-region
    darkness = contrast/np.maximum(background, .15)
    strong_core = (contrast >= .15) & (darkness >= .45)
    # Antialiasing can leave a narrow dark core inside a wider visible square.
    # Use its connected moderate-contrast support for shape and centroid, while
    # still requiring at least two strong core pixels to establish real ink.
    mask = (contrast >= .08) & (darkness >= .20)
    remaining = set(map(tuple, np.argwhere(mask)))
    choices = []
    expected_area = float(np.prod(expected_size_px))
    while remaining:
        seed = remaining.pop()
        stack, points = [seed], [seed]
        while stack:
            y, x = stack.pop()
            for ddy, ddx in ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)):
                p = y+ddy, x+ddx
                if p in remaining:
                    remaining.remove(p); stack.append(p); points.append(p)
        indices = np.asarray(points)
        core_pixels = int(strong_core[indices[:, 0], indices[:, 1]].sum())
        if core_pixels < 2:
            continue
        low, high = indices.min(axis=0), indices.max(axis=0)
        size_xy = (high-low+1)[::-1]
        touches_border = bool(np.any(low == 0) or np.any(high == np.array(region.shape)-1))
        if touches_border or not max(2, .2*expected_area) <= len(points) <= 1.8*expected_area:
            continue
        if np.any(size_xy < np.maximum(1, .4*expected_size_px)) or np.any(size_xy > 1+1.8*expected_size_px):
            continue
        aspect = (size_xy[0]/size_xy[1])/(expected_size_px[0]/expected_size_px[1])
        if not .5 <= aspect <= 2.0 or len(points)/float(np.prod(size_xy)) < .4:
            continue
        weights = darkness[indices[:, 0], indices[:, 1]]
        xy = np.average(indices[:, ::-1], axis=0, weights=weights)+[left, top]
        choices.append({"xy": xy, "pixels": len(points), "strong_core_pixels": core_pixels,
                        "box_size_px": size_xy.tolist(),
                        "median_local_contrast": float(np.median(contrast[indices[:, 0], indices[:, 1]])),
                        "median_normalized_darkness": float(np.median(weights)),
                        "background_at_center": float(coefficients[0])})
    if not choices:
        raise ValueError("An outer black registration marker was not found")
    chosen = min(choices, key=lambda c: np.linalg.norm(c["xy"]-center))
    if np.linalg.norm(chosen["xy"]-center) > half_window:
        raise ValueError("Outer registration marker is outside its search window")
    return chosen


def registration_check(baseline_rgb, candidate_rgb, target, baseline_result):
    result = {"valid": False, "passes": False, "maximum_allowed_shift_px": 1.0, "markers": []}
    try:
        affine = np.asarray(baseline_result["mm_to_pixel_affine"])
        gray0, gray1 = luminance(baseline_rgb), luminance(candidate_rgb)
        for marker in target["registration_markers"]:
            expected = _project(np.asarray(marker["xy_mm"]), affine)
            expected_size = np.abs(affine[:2]).T@np.asarray(marker["size_mm"])
            a = find_registration_marker(gray0, expected, 5.0, expected_size)
            b = find_registration_marker(gray1, a["xy"], 4.0, expected_size)
            result["markers"].append({"name": marker["name"], "baseline_xy_px": a["xy"].tolist(),
                                      "candidate_xy_px": b["xy"].tolist(),
                                      "shift_px": float(np.linalg.norm(a["xy"]-b["xy"])),
                                      "baseline_mask_pixels": a["pixels"], "candidate_mask_pixels": b["pixels"],
                                      "baseline_strong_core_pixels": a["strong_core_pixels"],
                                      "candidate_strong_core_pixels": b["strong_core_pixels"],
                                      "expected_size_px": expected_size.tolist(),
                                      "baseline_local_contrast": a["median_local_contrast"],
                                      "candidate_local_contrast": b["median_local_contrast"],
                                      "baseline_box_size_px": a["box_size_px"],
                                      "candidate_box_size_px": b["box_size_px"]})
        result["maximum_shift_px"] = max(m["shift_px"] for m in result["markers"])
        result.update(valid=True, passes=result["maximum_shift_px"] <= 1.0,
                      detector="v2 local-background contrast with known-square size and shape bounds")
    except (ValueError, KeyError) as exc:
        result["error"] = str(exc)
    return result


def compare_arrays(baseline_raw, candidate_raw, baseline_policy, candidate_policy, target):
    if np.asarray(baseline_raw).shape != np.asarray(candidate_raw).shape:
        raise ValueError("Raw reference and candidate dimensions must match")
    if np.asarray(baseline_policy).shape[:2] != (224, 224) or np.asarray(candidate_policy).shape[:2] != (224, 224):
        raise ValueError("Supply actual 224 x 224 policy input images")
    result = {"schema": "magnifier-inspection-photo-v3", "photo_score": 0.0,
              "full_trial_score_certified": False, "valid": False,
              "limits": ["Photo result does not establish a physical grasp, pickup, clearance or a continuous three-second hold.",
                         "Use actual camera and policy images at unchanged intrinsics; no image zoom or crop is allowed.",
                         "Raw outer-marker registration checks the same camera/target pose within one pixel.",
                         "A local affine map locates the printed groups; strong nonlinear distortion can make that lookup inconclusive."]}
    for name, before, after in (("raw", baseline_raw, candidate_raw), ("policy", baseline_policy, candidate_policy)):
        a, b = measure_image(before, target), measure_image(after, target)
        result[name] = {"baseline": a, "candidate": b}
        if a["valid"] and b["valid"]:
            result[name].update(edge_gains(a, b))
    if not all(result[name][side]["valid"] for name in ("raw", "policy") for side in ("baseline", "candidate")):
        result["error"] = "At least one image lacks measurable fiducials or contrast references"
        return result
    result["registration"] = registration_check(baseline_raw, candidate_raw, target, result["raw"]["baseline"])
    prior = {g["name"]: g["readable"] for g in result["policy"]["baseline"]["bar_groups"]}
    current = {g["name"]: g["readable"] for g in result["policy"]["candidate"]["bar_groups"]}
    new_groups = [name for name in current if current[name] and not prior[name]]
    minimum_gain = min(result[name]["minimum_edge_gain"] for name in ("raw", "policy"))
    result.update(valid=True, newly_readable_groups=new_groups,
                  minimum_edge_gain_both_sizes=minimum_gain)
    photo_gate = result["registration"]["passes"] and bool(new_groups)
    if photo_gate and minimum_gain >= 1.25:
        result["photo_score"] = .5
    if photo_gate and minimum_gain >= 1.5 and all(current.values()):
        result["photo_score"] = 1.0
    result["photo_rule"] = {"partial_gain": 1.25, "full_gain": 1.5,
                            "partial_detail": "At least one previously unreadable group becomes readable.",
                            "full_detail": "Both groups readable, with at least one newly readable.",
                            "gain_measurement": "Minimum of all four square-edge ratios in raw and actual policy images.",
                            "bar_rule": "Every chart-aligned native-pixel transect, spaced at most one pixel apart across the central half of the bar length, contains five dark bars and four bright gaps with at least one strong original pixel each; dark <= B+0.4(W-B), bright >= B+0.6(W-B)."}
    return result


def compare_files(baseline_raw, candidate_raw, baseline_policy, candidate_policy, target_json):
    paths = [baseline_raw, candidate_raw, baseline_policy, candidate_policy]
    arrays = [np.asarray(Image.open(path).convert("RGB")) for path in paths]
    result = compare_arrays(*arrays, json.loads(Path(target_json).read_text()))
    result["source_sha256"] = {name: hashlib.sha256(Path(path).read_bytes()).hexdigest()
                                for name, path in zip(("baseline_raw", "candidate_raw", "baseline_policy", "candidate_policy"), paths)}
    return result


def self_test():
    """Synthetic algorithm checks only; no synthetic image is optical evidence."""
    import tempfile
    from PIL import ImageFilter
    try:
        from .inspection_target import generate
    except ImportError:
        from inspection_target import generate
    with tempfile.TemporaryDirectory() as temporary:
        target = generate(temporary, pixels_per_mm=20)
        texture = Image.open(Path(temporary)/"target.png")
        def card(scale):
            image = Image.new("RGB", (640, 360), (150, 150, 150))
            resized = texture.resize((round(120*scale), round(100*scale)), Image.Resampling.LANCZOS)
            image.paste(resized, ((640-resized.width)//2, (360-resized.height)//2))
            return np.asarray(image)
        baseline = card(1.95)
        enlarged = card(1.95*1.9)
        yy, xx = np.indices(baseline.shape[:2])
        lens_region = (xx-320)**2+(yy-180)**2 < 80**2
        candidate = baseline.copy()
        candidate[lens_region] = enlarged[lens_region]
        def policy(raw):
            return np.asarray(Image.fromarray(raw).resize((224, 224), Image.Resampling.BILINEAR))
        good = compare_arrays(baseline, candidate, policy(baseline), policy(candidate), target)
        assert good["photo_score"] == 1.0, good
        # The complete card remains visible at these offsets, but enlarged
        # coloured dots extend outside the old central-60% development crop.
        for translation in (-160, 160):
            before = np.roll(baseline, translation, axis=1)
            after = np.roll(candidate, translation, axis=1)
            off_centre = compare_arrays(before, after, policy(before), policy(after), target)
            assert off_centre["photo_score"] == 1.0, off_centre
        identity = compare_arrays(baseline, baseline, policy(baseline), policy(baseline), target)
        assert identity["photo_score"] == 0
        zoom = compare_arrays(baseline, enlarged, policy(baseline), policy(enlarged), target)
        assert zoom["photo_score"] == 0 and not zoom.get("registration", {}).get("passes", False)
        blurred = candidate.copy()
        blurred_all = np.asarray(Image.fromarray(candidate).filter(ImageFilter.GaussianBlur(6)))
        # Erase detail while keeping markers, reference swatches and outer registration intact.
        blurred[157:171, 298:342] = blurred_all[157:171, 298:342]
        blurred[176:217, 313:327] = blurred_all[176:217, 313:327]
        blur_result = compare_arrays(baseline, blurred, policy(baseline), policy(blurred), target)
        assert blur_result["photo_score"] == 0, blur_result
        moved = np.roll(candidate, 3, axis=1)
        movement = compare_arrays(baseline, moved, policy(baseline), policy(moved), target)
        assert movement["photo_score"] == 0 and not movement["registration"]["passes"]
        flat = np.full_like(baseline, 128)
        assert compare_arrays(baseline, flat, policy(baseline), policy(flat), target)["photo_score"] == 0
        # A missing or broadly occluded single marker must not be replaced by
        # an unrelated dark edge, while all inner optical details remain intact.
        missing_marker = candidate.copy()
        missing_marker[97:108, 412:423] = 242
        assert compare_arrays(baseline, missing_marker, policy(baseline), policy(missing_marker), target)["photo_score"] == 0
        occluded_marker = candidate.copy()
        occluded_marker[96:109, 411:424] = 12
        assert compare_arrays(baseline, occluded_marker, policy(baseline), policy(occluded_marker), target)["photo_score"] == 0
        # A true square remains localizable under a slow spatial illumination
        # gradient even when its pixels are brighter than shaded central ink.
        yy, xx = np.indices((41, 41))
        gradient = .45+.006*xx+.002*yy
        local_square = gradient.copy()
        local_square[19:22, 19:22] = .24
        located = find_registration_marker(local_square, [20, 20], 5, [3, 3])
        assert np.linalg.norm(located["xy"]-[20, 20]) < .1
        try:
            find_registration_marker(gradient, [20, 20], 5, [3, 3])
            raise AssertionError("Smooth illumination gradient was mistaken for a printed square")
        except ValueError:
            pass
        # A subpixel square has a two-pixel dark core and a wider antialiased
        # footprint. Its full support, not the thresholded core, defines shape.
        soft = np.full((41, 41), .9)
        soft[19:21, 19:22] = [[.52, .20, .56], [.52, .20, .56]]
        soft_location = find_registration_marker(soft, [20, 19.5], 5, [3, 3])
        assert soft_location["strong_core_pixels"] == 2
        assert np.linalg.norm(soft_location["xy"]-[20, 19.5]) < .1
        faint_shadow = np.full((41, 41), .9)
        faint_shadow[19:22, 19:22] = .6
        try:
            find_registration_marker(faint_shadow, [20, 20], 5, [3, 3])
            raise AssertionError("A moderate shadow without dark ink was accepted")
        except ValueError:
            pass
        return {"passed": True, "cases": 14, "synthetic_only": True,
                "positive_minimum_gain": good["minimum_edge_gain_both_sizes"],
                "positive_new_groups": good["newly_readable_groups"],
                "local_gradient_marker_error_px": float(np.linalg.norm(located["xy"]-[20, 20])),
                "interpretation": "Synthetic image-counting checks, not rendered optical validation."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--candidate", type=Path)
    parser.add_argument("--baseline-policy", type=Path)
    parser.add_argument("--candidate-policy", type=Path)
    parser.add_argument("--target-json", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        result = self_test()
    else:
        if not all((args.baseline, args.candidate, args.baseline_policy, args.candidate_policy, args.target_json)):
            parser.error("Supply all four actual-image paths and --target-json")
        result = compare_files(args.baseline, args.candidate, args.baseline_policy, args.candidate_policy, args.target_json)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))
