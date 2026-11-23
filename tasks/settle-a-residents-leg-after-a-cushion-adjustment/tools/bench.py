"""Grade a bench worksheet. Coordinates in mm. Threshold-ambiguous frames invalidate the record."""

import argparse, csv, json
from pathlib import Path
from task.scoring import score


def grade(path):
    rows = []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            if row["readable"] != "1":
                raise ValueError(f"Unscorable evidence at frame {row['frame']}")
            w = {
                k: float(row[k])
                for k in (
                    "heel_channel_n",
                    "heel_clear_mm",
                    "protected_clear_mm",
                    "sole_bed_gap_mm",
                    "sole_up",
                    "person_clear_mm",
                    "park_gap_mm",
                )
            }
            for k in ("lift_slack", "bed_touch", "both_jaws_on_limb"):
                if row[k] not in ("1", "0"):
                    raise ValueError("Expected 0/1 in " + k)
                w[k] = row[k] == "1"
            for k in (
                "heel_xyz_mm",
                "knee_xyz_mm",
                "sole_marks_mm",
                "stripe_compression_mm",
                "calf_rows",
            ):
                w[k] = json.loads(row[k])
            occupied = sorted(set(w["calf_rows"][0] + w["calf_rows"][1]))
            w["bearing_span_mm"] = (occupied[-1] - occupied[0]) * 43 if occupied else 0
            rows.append(dict(tick=int(row["frame"]) - 1, witness=w))
    return score(rows)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("worksheet", type=Path)
    print(*grade(p.parse_args().worksheet))
