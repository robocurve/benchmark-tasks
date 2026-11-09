"""Measure actual RGB enlargement and line detail in a same-pose optical test.

No lens geometry or camera projection is used to supply an enlargement result.
Color fiducials locate a local affine map for sampling the printed line patches.
This conservative development metric is not a finished competition scorer.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from PIL import Image


def _bilinear(array, xy):
    xy = np.asarray(xy, dtype=float)
    x, y = xy[..., 0], xy[..., 1]
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    if np.any(x0 < 0) or np.any(y0 < 0) or np.any(x0+1 >= array.shape[1]) or np.any(y0+1 >= array.shape[0]):
        raise ValueError("Measured target extends beyond image bounds")
    fx, fy = x-x0, y-y0
    return ((1-fx)*(1-fy)*array[y0, x0] + fx*(1-fy)*array[y0, x0+1]
            + (1-fx)*fy*array[y0+1, x0] + fx*fy*array[y0+1, x0+1])


def _components(mask, weights):
    """Small dependency-free 8-connected component detector."""
    remaining = set(map(tuple, np.argwhere(mask)))
    result = []
    while remaining:
        seed = remaining.pop()
        stack, pixels = [seed], [seed]
        while stack:
            y, x = stack.pop()
            for dy, dx in ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)):
                neighbor = y+dy, x+dx
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
                    pixels.append(neighbor)
        indices = np.asarray(pixels)
        if len(pixels) > mask.size * .02:
            continue
        w = weights[indices[:, 0], indices[:, 1]]
        xy = np.average(indices[:, ::-1], axis=0, weights=w)
        result.append({"xy": xy, "pixels": len(pixels), "weight": float(w.sum())})
    return result


def find_fiducials(rgb, target):
    image = np.asarray(rgb, dtype=float)
    height, width = image.shape[:2]
    r, g, b = np.moveaxis(image, -1, 0)
    masks = {"red": (r > 1.45*g) & (r > 1.45*b),
             "green": (g > 1.35*r) & (g > 1.25*b),
             "blue": (b > 1.4*r) & (b > 1.15*g),
             "magenta": (r > 1.4*g) & (b > 1.4*g) & (r < 1.6*b) & (b < 1.6*r)}
    bright = image.max(axis=-1) > 45
    # The static proof centres its intended target; avoid unrelated robot colors.
    region = np.zeros((height, width), dtype=bool)
    region[round(height*.2):round(height*.8), round(width*.2):round(width*.8)] = True
    weights = (image.max(axis=-1)-image.min(axis=-1)) + 1
    candidates = []
    for fiducial in target["fiducials"]:
        components = _components(masks[fiducial["name"]] & bright & region, weights)
        components.sort(key=lambda c: np.linalg.norm(c["xy"]-[width/2, height/2]))
        candidates.append(components[:6])
    if any(not choices for choices in candidates):
        missing = [fid["name"] for fid, choices in zip(target["fiducials"], candidates) if not choices]
        raise ValueError("Missing RGB fiducial(s): " + ", ".join(missing))
    world = np.array([[*f["xy_mm"], 1] for f in target["fiducials"]])
    choices = []
    for group in itertools.product(*candidates):
        pixels = np.array([c["xy"] for c in group])
        affine, _, _, _ = np.linalg.lstsq(world, pixels, rcond=None)
        singular = np.linalg.svd(affine[:2], compute_uv=False)
        if singular[-1] < .15 or singular[0]/singular[-1] > 5:
            continue
        residual = float(np.sqrt(np.mean(np.sum((world@affine-pixels)**2, axis=1))))
        span = np.median([np.linalg.norm(pixels[i]-pixels[j]) for i, j in itertools.combinations(range(4), 2)])
        if residual > max(1.2, .04*span):
            continue
        center_distance = np.linalg.norm(pixels.mean(axis=0)-[width/2, height/2]) / min(width, height)
        choices.append((residual/max(span, 1)+center_distance*.08, group, affine, residual))
    if not choices:
        raise ValueError("RGB fiducials do not form a coherent target quadrilateral")
    choices.sort(key=lambda c: c[0])
    _, selected, affine, residual = choices[0]
    return selected, affine, {"affine_rms_error_px": residual,
                               "coherent_quartets": len(choices),
                               "selection": "Most central coherent colored fiducial quadrilateral"}


def _project(xy, affine):
    xy = np.asarray(xy)
    return np.concatenate((xy, np.ones((*xy.shape[:-1], 1))), axis=-1) @ affine


def measure_image(rgb, target):
    rgb = np.asarray(rgb)[..., :3]
    result = {"image_size": [int(rgb.shape[1]), int(rgb.shape[0])], "valid": False}
    try:
        selected, affine, fit = find_fiducials(rgb, target)
        result.update(fit)
        result["fiducials"] = {fid["name"]: {"xy_px": c["xy"].tolist(), "mask_pixels": c["pixels"]}
                               for fid, c in zip(target["fiducials"], selected)}
        result["mm_to_pixel_affine"] = affine.tolist()
        gray = (rgb.astype(float) @ np.array([.2126, .7152, .0722])) / 255
        references = {}
        for reference in target["references"]:
            center, size = np.array(reference["center_mm"]), np.array(reference["size_mm"])
            xx, yy = np.meshgrid(np.linspace(-.35, .35, 17)*size[0], np.linspace(-.2, .2, 3)*size[1])
            xy = np.stack([xx, yy], axis=-1)+center
            references[reference["name"]] = float(np.median(_bilinear(gray, _project(xy, affine))))
        dynamic = references["white"]-references["black"]
        result["reference_luminance"] = references
        result["reference_dynamic_range"] = dynamic
        patterns = []
        for patch in target["patches"]:
            center, size = np.array(patch["center_mm"]), np.array(patch["size_mm"])
            axis = 0 if patch["orientation"] == "vertical" else 1
            orthogonal = 1-axis
            length = size[axis]*.92
            position = np.linspace(-length/2, length/2, 181)
            offsets = np.linspace(-.32, .32, 9)*size[orthogonal]
            xy = np.tile(center, (len(offsets), len(position), 1))
            xy[:, :, axis] += position
            xy[:, :, orthogonal] += offsets[:, None]
            profile = _bilinear(gray, _project(xy, affine)).mean(axis=0)
            expected = 1/patch["pitch_mm"]
            best = None
            # Small optical distortion can change local pitch relative to the fiducial span.
            for frequency in np.linspace(expected*.9, expected*1.1, 31):
                design = np.stack((np.ones_like(position), position, np.sin(2*np.pi*frequency*position),
                                   np.cos(2*np.pi*frequency*position)), axis=-1)
                coef, _, _, _ = np.linalg.lstsq(design, profile, rcond=None)
                error = float(np.mean((profile-design@coef)**2))
                if best is None or error < best[0]:
                    best = (error, frequency, coef)
            error, frequency, coef = best
            variance = float(np.var(profile))
            r_squared = 1-error/max(variance, 1e-12)
            normalized_modulation = 2*float(np.hypot(coef[2], coef[3]))/max(dynamic, 1e-6)
            pixels_per_period = float(np.linalg.norm(affine[axis])/frequency)
            # Native pixel spacing, not the upsampled profile, gates resolution.
            resolved = (dynamic >= .2 and pixels_per_period >= 3.0
                        and normalized_modulation >= .25 and r_squared >= .45)
            patterns.append({"name": patch["name"], "pitch_mm": patch["pitch_mm"],
                             "orientation": patch["orientation"], "resolved": bool(resolved),
                             "fitted_frequency_per_mm": float(frequency),
                             "native_pixels_per_period": pixels_per_period,
                             "normalized_modulation": normalized_modulation,
                             "fit_r_squared": r_squared})
        result.update(valid=True, patterns=patterns)
    except ValueError as exc:
        result["error"] = str(exc)
    return result


def compare_arrays(baseline, candidate, target):
    before, after = measure_image(baseline, target), measure_image(candidate, target)
    result = {"baseline": before, "candidate": after, "valid": before["valid"] and after["valid"],
              "optical_benefit": False}
    if not result["valid"]:
        return result
    ratios = []
    names = [fid["name"] for fid in target["fiducials"]]
    for a, b in itertools.combinations(names, 2):
        span0 = np.linalg.norm(np.array(before["fiducials"][a]["xy_px"])-before["fiducials"][b]["xy_px"])
        span1 = np.linalg.norm(np.array(after["fiducials"][a]["xy_px"])-after["fiducials"][b]["xy_px"])
        ratios.append({"pair": [a, b], "baseline_span_px": float(span0), "candidate_span_px": float(span1),
                       "linear_ratio": float(span1/span0)})
    gain = float(np.median([pair["linear_ratio"] for pair in ratios]))
    prior = {p["name"]: p for p in before["patterns"]}
    newly_resolved = [p["name"] for p in after["patterns"] if p["resolved"] and not prior[p["name"]]["resolved"]]
    both_orientation_pitches = sorted({p["pitch_mm"] for p in after["patterns"]
                                      if f"vertical_{p['pitch_mm']:g}mm" in newly_resolved
                                      and f"horizontal_{p['pitch_mm']:g}mm" in newly_resolved})
    result.update(linear_magnification=gain, fiducial_pairs=ratios, newly_resolved_patterns=newly_resolved,
                  newly_resolved_in_both_orientations_mm=both_orientation_pitches,
                  optical_benefit=bool(gain >= 1.2 and both_orientation_pitches))
    return result


def compare_files(baseline, candidate, target_json, baseline_policy=None, candidate_policy=None):
    target = json.loads(Path(target_json).read_text())
    original = [Image.open(path).convert("RGB") for path in (baseline, candidate)]
    if original[0].size != original[1].size:
        raise ValueError("Baseline and candidate must use identical camera resolutions")
    if bool(baseline_policy) != bool(candidate_policy):
        raise ValueError("Supply both native policy PNGs, or neither")
    if baseline_policy:
        resized = [Image.open(path).convert("RGB") for path in (baseline_policy, candidate_policy)]
        if any(im.size != (224, 224) for im in resized):
            raise ValueError("Native policy inputs must each be 224 x 224")
        resize_description = "Native policy PNGs saved by the actual camera preprocessing pipeline."
    else:
        # PIL bilinear antialiases; it is not identical to policy cv2 INTER_LINEAR.
        resized = [im.resize((224, 224), Image.Resampling.BILINEAR) for im in original]
        resize_description = "PIL bilinear diagnostic proxy; actual policy PNGs were not supplied."
    results = {"schema": "magnifier-rgb-comparison-v1",
               "source_sha256": {"baseline": hashlib.sha256(Path(baseline).read_bytes()).hexdigest(),
                                  "candidate": hashlib.sha256(Path(candidate).read_bytes()).hexdigest()},
               "raw": compare_arrays(*[np.asarray(im) for im in original], target),
               "resized_224": compare_arrays(*[np.asarray(im) for im in resized], target),
               "resize": resize_description,
               "actual_policy_input_checked": bool(baseline_policy),
               "limits": ["A static same-pose image comparison is required; camera or target motion invalidates the optical claim.",
                          "Color fiducials and line bars are measured from RGB. Geometry only labels physical dimensions.",
                          "A local affine map approximates distortion; inspect saved images and any rejected fiducials.",
                          "Three native pixels per period plus modulation/fit thresholds conservatively reject aliasing.",
                          "This is a feasibility metric, not a validated task scorer."]}
    if baseline_policy:
        results["source_sha256"].update({"baseline_policy": hashlib.sha256(Path(baseline_policy).read_bytes()).hexdigest(),
                                         "candidate_policy": hashlib.sha256(Path(candidate_policy).read_bytes()).hexdigest()})
    results["small_optical_test_passes"] = bool(baseline_policy and results["raw"]["optical_benefit"] and results["resized_224"]["optical_benefit"])
    return results


def self_test():
    """Synthetic image algorithm checks only: deliberately not optics evidence."""
    import tempfile
    from target import generate
    with tempfile.TemporaryDirectory() as temp:
        target = generate(temp, pixels_per_mm=20)
        texture = Image.open(Path(temp)/"target.png")
        def simulated(scale):
            frame = Image.new("RGB", (640, 360), (100, 100, 100))
            card = texture.resize((round(120*scale), round(100*scale)), Image.Resampling.LANCZOS)
            frame.paste(card, ((640-card.width)//2, (360-card.height)//2))
            return np.asarray(frame)
        reference, enlarged = simulated(1.5), simulated(3)
        identity = compare_arrays(reference, reference, target)
        scaled = compare_arrays(reference, enlarged, target)
        assert identity["valid"] and abs(identity["linear_magnification"]-1) < 1e-12
        assert not identity["optical_benefit"]
        assert scaled["valid"] and abs(scaled["linear_magnification"]-2) < .06
        assert scaled["optical_benefit"], scaled
        from PIL import ImageFilter
        blurred = enlarged.copy()
        low_detail = np.asarray(Image.fromarray(enlarged).filter(ImageFilter.GaussianBlur(3.5)))
        # Keep all four markers intact: enlargement alone must not pass when
        # the central fine detail is blurred away.
        blurred[162:198, 296:344] = low_detail[162:198, 296:344]
        blurred_result = compare_arrays(reference, blurred, target)
        assert blurred_result["valid"] and blurred_result["linear_magnification"] > 1.9
        assert not blurred_result["optical_benefit"]
        missing = np.full_like(reference, 128)
        assert not compare_arrays(reference, missing, target)["valid"]
        return {"checks": 5, "passed": True, "evidence_type": "Synthetic image algorithm checks; not rendered optics validation",
                "synthetic_measured_ratio": scaled["linear_magnification"]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--candidate", type=Path)
    parser.add_argument("--target-json", type=Path)
    parser.add_argument("--baseline-policy", type=Path, help="Actual 224x224 policy PNG, not an offline proxy")
    parser.add_argument("--candidate-policy", type=Path, help="Actual 224x224 policy PNG, not an offline proxy")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        result = self_test()
    else:
        if not (args.baseline and args.candidate and args.target_json):
            parser.error("--baseline, --candidate and --target-json are required")
        result = compare_files(args.baseline, args.candidate, args.target_json, args.baseline_policy, args.candidate_policy)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
