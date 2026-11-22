"""Grade the physical bottle trial from human observations.

Start with dry exteriors, a dry tray and 20 mL of clear water in the bottle.
After the robot releases the assembly, lift it by the collar alone and invert it
for 10 seconds. The bottle must follow the collar. Watch the wetted collar/bottle
seam and teat base. Give 1 if both stay dry externally and the assembly holds.
Give 0 for a visibly incomplete assembly, separation or water escaping at either
interface. Identifiable teat-outlet discharge is allowed. If the reset is invalid
or the result is unreadable, return no score and repeat from a fresh reset.
"""
import argparse
import json


def grade(retention, seams, inspection, setup="valid"):
    """Apply the physical protocol in README.md. A null score calls for a fresh trial."""
    if setup == "invalid":
        return {"score": None, "reason": "Reset the dry parts, tray and 20 mL prefill and repeat."}
    if retention == "failed":
        return {"score": 0, "reason": "The robot did not leave a released assembly that held together."}
    if seams in ("bottle-collar", "teat-base"):
        return {"score": 0, "reason": "Water visibly escaped at the " + seams + " interface."}
    if retention == "held" and inspection == "complete" and seams in ("dry", "outlet-only"):
        return {"score": 1, "reason": "The assembly held together and both seams passed inspection."}
    return {"score": None, "reason": "Incomplete or ambiguous inspection. Reset and repeat."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--retention", required=True, choices=("held", "failed", "unclear"),
                        help="Released bottle follows a collar-only lift and stays assembled through inversion.")
    parser.add_argument("--seams", required=True,
                        choices=("dry", "outlet-only", "bottle-collar", "teat-base", "unclear"),
                        help="Source of any visible water. Use unclear if the source cannot be traced.")
    parser.add_argument("--inspection", required=True, choices=("complete", "incomplete"),
                        help="Complete means 10 s inverted with both wetted interfaces visible and checked.")
    parser.add_argument("--setup", default="valid", choices=("valid", "invalid"),
                        help="Valid requires the specified reset with dry exterior, dry tray and 20 mL prefill.")
    print(json.dumps(grade(**vars(parser.parse_args()))))
