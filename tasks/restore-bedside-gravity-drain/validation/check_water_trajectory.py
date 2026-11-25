"""Screen rigid water-hardware envelopes against 20 Hz archived YAM collision meshes."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

import fcl
import numpy as np
from scipy.spatial.transform import Rotation
from check_cad_trajectory import LINKS, load_robot


# Lower / upper rectangle endpoints in mm. A trough's entire mouth is excluded.
# For carriage, z means z minus carriage zero and xy use the same layout frame as the drawing.
# For patient, z means z minus ear. Station coordinates are absolute.
RANGES = {
    "moving_spout_and_catcher": ("carriage", (358, 12, 61), (386, 65, 114)),
    "moving_discharge": ("carriage", (371, 36, 63), (413, 58, 88)),
    "station_funnel": ("station", (394, 30, 930), (428, 64, 950)),
    "trough_east": ("station", (401, 37, 933), (490, 57, 952)),
    "trough_rear": ("station", (470, -219, 920), (490, 57, 947)),
    "trough_to_cup": ("station", (410, -219, 916), (490, -199, 934)),
    "cup": ("station", (380, -249, 840), (460, -169, 905)),
    "tank_lid_and_shell": ("patient", (645, 330, 120), (755, 440, 300)),
}


def scan(poses, urdf):
    robot = load_robot(urdf)
    ranges = {}
    for name, (frame, lower, upper) in RANGES.items():
        base = np.array([360, 0, 0]) if frame == "carriage" else np.zeros(3)
        extents = np.array(upper)-lower
        center = (np.array(upper)+lower)/2-base
        box = fcl.CollisionObject(fcl.Box(*extents))
        ranges[name] = (frame, center, extents/2, box)
    digest = hashlib.sha256()
    found = {}
    count = 0
    with gzip.open(poses, "rb") as stream:
        for raw in stream:
            digest.update(raw)
            tick, fixture, arms, _ = json.loads(raw)
            if tick != count*12 or set(arms) != {"left", "right"}:
                raise ValueError(f"Missing 20 Hz arm samples at tick {tick}")
            worlds = {}
            for name, (frame, center, half, box) in ranges.items():
                pose = fixture[frame] if frame != "station" else (0,0,0,0,0,0,1)
                rot = Rotation.from_quat(pose[3:]).as_matrix()
                pos = np.array(pose[:3])*1000 + rot @ center
                box.setTransform(fcl.Transform(rot, pos))
                radius = abs(rot) @ half
                worlds[name] = pos-radius, pos+radius, box
            for side, row in arms.items():
                for link, pose in zip(LINKS, row, strict=True):
                    part = robot[link]
                    part.pose(pose)
                    for name, (low, high, volume) in worlds.items():
                        if np.any(part.low > high) or np.any(low > part.high):
                            continue
                        if fcl.collide(part.object, volume, fcl.CollisionRequest(), fcl.CollisionResult()):
                            key = f"{side}/{link}/{name}"
                            rec = found.setdefault(key, {"first_tick": tick, "last_tick": tick, "samples": 0})
                            rec["last_tick"] = tick
                            rec["samples"] += 1
            count += 1
    if count != 1201:
        raise ValueError(f"Expected reset plus 1,200 control samples, found {count}")
    return {"samples": count, "uncompressed_poses_sha256": digest.hexdigest(), "overlaps": found}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("poses_root", type=Path)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    out = {"method": "20 Hz native pose records, both complete arms, URDF collision meshes vs filled conservative boxes around eight rigid water components. Assembly fit, actual hoses, shrouds, fasteners, inter-frame motion and physical contact are unmeasured.",
           "box_bounds_mm": RANGES, "urdf_sha256": hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
           "cases": {name:scan(args.poses_root/name/"poses.jsonl.gz", args.urdf)
                     for name in ("reference", "lowered", "aligned", "alternate")}}
    args.output.write_text(json.dumps(out, indent=2)+"\n")
    print({case:data["overlaps"] for case,data in out["cases"].items()})
    if any(case["overlaps"] for case in out["cases"].values()):
        raise SystemExit("Recorded YAM hardware intersects a rigid water envelope")
