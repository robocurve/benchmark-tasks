#!/usr/bin/env python3
"""Check a measured water-only commissioning series and its indexed records."""
import argparse
import csv
import hashlib
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

CASES = {
    "aligned": (50, 90), "low": (170, 90), "high": (-70, 90),
    "sealed": (170, 0), "onset": (50, 10), "mid": (50, 45), "full": (50, 80),
}
FIELDS = (
    "condition repeat start_fill_ml head_start_mm head_end_mm angle_deg "
    "clip_t0_s clip_t15_s clip_t30_s source0_g source15_g source30_g "
    "receiver0_g receiver15_g receiver30_g blank15_g record_file record_sha256"
).split()


def ensure(test, message):
    if not test:
        raise ValueError(message)


def number(fields, key):
    value = float(fields[key])
    ensure(math.isfinite(value), f"{key} must be finite")
    return value


SWEEP_FIELDS = "direction nominal_angle_deg actual_angle_deg head_mm clip_t0_s clip_t30_s receiver0_g receiver30_g record_file record_sha256".split()


def volume_bound(v, setup, seconds):
    """Worst absolute mass, density and timing contribution to a volume difference."""
    rho = float(setup["water_density_g_ml"])
    drho = float(setup["density_uncertainty_g_ml"])
    scale = float(setup["max_abs_check_weight_error_g"]) + 0.5*(
        float(setup["scale_resolution_g"]) + float(setup["max_repeat_spread_g"]))
    return (2*scale + abs(v)*drho)/(rho-drho) + abs(v)*2*float(setup["timing_uncertainty_s"])/seconds


def bounded(value, low, high, uncertainty, label):
    ensure(low-1e-8 <= value <= high+1e-8, label)
    ensure(low-1e-8 <= value-uncertainty and value+uncertainty <= high+1e-8,
           f"inconclusive at measurement uncertainty: {label}")


def model(head, angle, seconds):
    return .01*max(0, head)*min(1, max(0, (angle-10)/70))*seconds


def check_curve(value, uncertainty, head, angle, seconds, setup):
    prediction = model(head, angle, seconds)
    ensure(abs(value-prediction) <= max(1, 0.1*prediction)+1e-8, "response outside model band")
    dh = float(setup["survey_uncertainty_mm"])
    da = float(setup["angle_uncertainty_deg"])
    # Timing error is included in volume_bound. Do not count it twice here.
    low = model(head-dh, angle-da, seconds)
    high = model(head+dh, angle+da, seconds)
    lower_limit = high-max(1, .1*high)
    upper_limit = low+max(1, .1*low)
    ensure(value-uncertainty >= lower_limit-1e-8 and value+uncertainty <= upper_limit+1e-8,
           "inconclusive at measurement uncertainty: response/model agreement")


def add_record(row, csv_path, reserved, file_hashes, used_clips, t0, t2):
    rel = Path(row["record_file"])
    base = csv_path.resolve().parent
    path = (base / rel).resolve()
    ensure(not rel.is_absolute() and path.is_relative_to(base) and path.is_file()
           and path not in reserved and path.stat().st_size > 0,
           "raw acquisition must exist under the measurement directory")
    if path not in file_hashes:
        digest = hashlib.sha256()
        with path.open("rb") as record:
            for chunk in iter(lambda: record.read(1 << 20), b""):
                digest.update(chunk)
        file_hashes[path] = digest.hexdigest()
    digest = file_hashes[path]
    ensure(row["record_sha256"].lower() == digest, "raw acquisition checksum")
    ensure(not any(t0 < end and start < t2 for start, end in used_clips[digest]), "reused clip interval")
    used_clips[digest].append((t0, t2))


def sweep_observations(setup, sweep_path, reserved, file_hashes, used_clips):
    expected = [(d, a) for d, angles in (("up", range(0,91,5)), ("down", range(90,-1,-5)))
                for a in angles]
    rho = float(setup["water_density_g_ml"])
    measured = {}
    previous = {}
    with sweep_path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        ensure(reader.fieldnames == SWEEP_FIELDS, "use the supplied valve sweep header")
        for i, row in enumerate(reader):
            try:
                direction, angle = row["direction"], int(row["nominal_angle_deg"])
                ensure(i < len(expected) and expected[i] == (direction, angle),
                       "missing, repeated, or out-of-order sweep angle")
                actual = number(row, "actual_angle_deg")
                head = number(row, "head_mm")
                start, end = number(row, "clip_t0_s"), number(row, "clip_t30_s")
                ensure(abs(actual-angle) <= 1 and abs(head-50) <= 1, "wrong sweep setting")
                ensure(start >= 0 and abs((end-start)-30) <= .10000001, "sweep duration")
                volume = (number(row,"receiver30_g")-number(row,"receiver0_g"))/rho
                unc = volume_bound(volume, setup, end-start)
                if direction in previous:
                    last, last_unc = previous[direction]
                    reverse = last-volume if direction == "up" else volume-last
                    ensure(reverse <= unc+last_unc+1e-8, "nonmonotonic response beyond measurement uncertainty")
                check_curve(volume, unc, head, actual, end-start, setup)
                add_record(row, sweep_path, reserved, file_hashes, used_clips, start, end)
                measured[(direction, angle)] = volume
                previous[direction] = (volume,unc)
            except (KeyError, TypeError, ValueError, OSError) as exc:
                raise ValueError(f"valve sweep line {i+2}: {exc}") from exc
    ensure(len(measured) == len(expected), "need all 38 sweep observations")
    return {direction: [round(measured[direction, a],3) for d,a in expected if d == direction]
            for direction in ("up","down")}


def observations(setup_path, csv_path, sweep_path=None):
    setup = json.loads(setup_path.read_text())
    for key in ("fixture_id", "source_scale_id", "receiver_scale_id", "operator"):
        value = setup[key]
        ensure(isinstance(value, str) and value.strip().lower() not in
               {"", "todo", "pending", "example", "unmeasured", "placeholder"},
               f"enter measured setup metadata for {key}")
    ensure(setup["source_scale_id"] != setup["receiver_scale_id"], "use independent balances")
    rho = number(setup, "water_density_g_ml")
    ensure(0.99 <= rho <= 1.01, "water density outside expected working range")
    for key in ("scale_resolution_g", "max_abs_check_weight_error_g", "max_repeat_spread_g"):
        value = number(setup, key)
        ensure(0 <= value <= 0.1 and (key != "scale_resolution_g" or value > 0),
               f"{key} outside the commissioning tolerance")
    for key in ("survey_uncertainty_mm", "timing_uncertainty_s", "density_uncertainty_g_ml", "angle_uncertainty_deg"):
        value = number(setup, key)
        ensure(0 < value <= {"survey_uncertainty_mm": 1, "timing_uncertainty_s": 0.1,
                             "density_uncertainty_g_ml": 0.01, "angle_uncertainty_deg": 1}[key], f"invalid {key}")
    ensure(number(setup, "restriction_length_mm") > 0, "enter the final restriction length")
    ensure(number(setup, "finished_mid_port_diameter_mm") >= 0.5, "enter the finished rotor pilot diameter")
    file_hashes, used_clips = {}, defaultdict(list)
    reserved = {path.resolve() for path in (setup_path,csv_path,sweep_path) if path is not None}
    results = {}
    uncertainties = {}
    with csv_path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        ensure(reader.fieldnames == FIELDS, "use the supplied water CSV header")
        for line, row in enumerate(reader, 2):
            try:
                name, rep = row["condition"], int(row["repeat"])
                head, angle = CASES[name]
                ensure(rep in range(1, 6) and (name, rep) not in results, "duplicate/invalid repetition")
                v = {key: number(row, key) for key in FIELDS[2:-2]}
                fill_range = (490, 500) if rep % 2 else (325, 335)
                ensure(fill_range[0] <= v["start_fill_ml"] <= fill_range[1], "incorrect starting fill")
                h0, h1 = v["head_start_mm"], v["head_end_mm"]
                ensure(max(abs(h0-head), abs(h1-head), abs(h0-h1)) <= 1, "head or head drift")
                ensure(abs(v["angle_deg"]-angle) <= 1, "tap angle")
                t0, t1, t2 = (v[k] for k in ("clip_t0_s", "clip_t15_s", "clip_t30_s"))
                ensure(t0 >= 0 and all(abs(x-y) <= .10000001 for x, y in
                       ((t1-t0, 15), (t2-t1, 15), (t2-t0, 30))), "timing")
                ensure(abs(v["blank15_g"])/rho <= .30000001, "blank drift")
                source = [v[f"source{t}_g"] for t in (0, 15, 30)]
                receiver = [v[f"receiver{t}_g"] for t in (0, 15, 30)]
                ensure(min(source) > 0, "source masses must be positive")
                halves = [[(source[i]-source[i+1])/rho for i in (0, 1)],
                          [(receiver[i+1]-receiver[i])/rho for i in (0, 1)]]
                ensure(min(map(min, halves)) >= -.30000001, "apparent reverse delivery above drift allowance")
                totals = [sum(pair) for pair in halves]
                ensure(abs(totals[0]-totals[1]) <= 1.00000001, "supply/collection disagreement")
                uncertainty = [volume_bound(total, setup, t2-t0) for total in totals]
                # Both balances use the same frame boundaries. Shared timing error cancels in this difference.
                difference_error = sum(volume_bound(total, setup, math.inf) for total in totals)
                bounded(totals[0]-totals[1], -1, 1, difference_error, "supply/collection disagreement")
                for total, unc in zip(totals, uncertainty):
                    check_curve(total, unc, (h0+h1)/2, v["angle_deg"], t2-t0, setup)
                    if name in ("high", "sealed"):
                        bounded(total, -math.inf, 1, unc, "unexpected delivery")
                    if name == "low":
                        ensure(total-unc > 35, "insufficient or inconclusive low-outlet delivery")
                if name == "aligned":
                    for pair in halves:
                        for vol in pair:
                            bounded(vol, 6, 9, volume_bound(vol, setup, 15), "aligned 15 s delivery outside acceptance")
                uncertainties[name, rep] = uncertainty[1]
                add_record(row, csv_path, reserved, file_hashes, used_clips, t0, t2)
                results[name, rep] = totals[1]
            except (KeyError, ValueError, TypeError, OSError) as exc:
                raise ValueError(f"water CSV line {line}: {exc}") from exc
    ensure(len(results) == 5*len(CASES), "need five independent resets for all seven conditions")
    low = min(results["low", rep]-uncertainties["low", rep] for rep in range(1, 6))
    aligned = max(results["aligned", rep]+uncertainties["aligned", rep] for rep in range(1, 6))
    ensure(low - aligned >= 17-1e-8, "low/aligned separation below 17 mL")
    totals = {case: [round(results[case, rep], 3) for rep in range(1, 6)] for case in CASES}
    return {"numeric_and_file_checks": "pass", "collector_totals_ml": totals,
            "valve_sweep_ml": sweep_observations(setup, sweep_path, reserved, file_hashes, used_clips)
            if sweep_path else "unverified: sweep not supplied", "independent_record_review": "required"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("setup", type=Path)
    parser.add_argument("measurements", type=Path)
    parser.add_argument("--sweep", type=Path, help="38 wet-transfer observations in 5-degree steps")
    args = parser.parse_args()
    try:
        print(json.dumps(observations(args.setup, args.measurements, args.sweep), indent=2))
    except (ValueError, TypeError, KeyError, OSError) as error:
        print(f"Water series unqualified: {error}", file=sys.stderr)
        sys.exit(1)
