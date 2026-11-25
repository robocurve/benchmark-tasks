"""Score brush pickup and image recovery attributable to brush-head wiping."""
import argparse
import json
import math
from pathlib import Path


def grade(grasp_achieved, recovery, *, verified_brush_wipe, direct_gripper_wipe):
    if not all(isinstance(flag, bool) for flag in (
        grasp_achieved, verified_brush_wipe, direct_gripper_wipe
    )):
        raise TypeError("pickup and wiping evidence must be boolean")
    if not math.isfinite(recovery):
        raise ValueError("image recovery must be finite")
    if not grasp_achieved:
        return 0.0
    if not verified_brush_wipe or direct_gripper_wipe:
        return 0.25
    if recovery >= 0.90:
        return 1.0
    if recovery >= 0.50:
        return 0.75
    if recovery >= 0.25:
        return 0.50
    return 0.25


def evaluate(trial, images, *, physical=False):
    if physical:
        # Assessor annotations from a continuous view of the brush and pane.
        verified_wipe = trial["verified_brush_wipe"]
        gripper_wipe = trial["direct_gripper_wipe"]
    else:
        # In both simulator runners, these are sliding brush-face contacts
        # measured against the pane. Only these contacts can remove dust.
        contact_steps = trial.get("contact_steps", 0)
        residual = trial.get("residual", 1.0)
        verified_wipe = (
            trial.get("contact_force_target") == "camera_window_body"
            and contact_steps > 0 and 0.0 <= residual < 1.0
        )
        gripper_wipe = False
    recovery = float(images["recovery_fraction_raw"])
    return {
        "score": grade(trial["grasp_achieved"], recovery,
                       verified_brush_wipe=verified_wipe, direct_gripper_wipe=gripper_wipe),
        "grasp_achieved": trial["grasp_achieved"],
        "first_grasp_step": trial.get("first_grasp_step"),
        "verified_brush_wipe": verified_wipe,
        "direct_gripper_wipe": trial["direct_gripper_wipe"] if physical else None,
        "wiping_evidence_source": "assessor_video" if physical else "brush_pane_contact_dose",
        "image_error_recovery": recovery,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--physical", action="store_true", help="Use assessor video annotations")
    args = parser.parse_args()
    result = evaluate(json.loads(args.trial.read_text()), json.loads(args.images.read_text()),
                      physical=args.physical)
    output = json.dumps(result, indent=2) + "\n"
    args.out.write_text(output)
    print(output, end="")
