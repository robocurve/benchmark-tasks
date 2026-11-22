"""Conservative physical-trial worksheet. Input angles in degrees and travel in mm."""
import argparse
import csv
import math
from pathlib import Path
from native.shaft_contract import LIMIT, DISENGAGE, PARK, PRELOAD


def evaluate(rows, angle_error, key_error, max_gap):
    """Errors bound combined calibration, reading and between-frame motion."""
    if not all(math.isfinite(x) and x>0 for x in (angle_error,key_error,max_gap)):
        raise ValueError("Specify measured positive error bounds and a maximum frame interval")
    prev=None
    unclear=False
    dwell_start=None
    possible_start=None
    for row in rows:
        t,a,r,k=(float(row[name]) for name in ("time_s","input_deg","output_deg","key_mm"))
        if not all(math.isfinite(x) for x in (t,a,r,k)):raise ValueError("Nonfinite measurement")
        flags=[]
        for name in ("observed","clear","intact","collision"):
            if str(row[name]) not in ("0","1"):raise ValueError(f"Expected 0 or 1 in {name}")
            flags.append(int(row[name]))
        observed,clear,intact,collision=flags
        if prev is not None and t<=prev:raise ValueError("Camera times must increase")
        unclear |= not observed or (prev is not None and t-prev>max_gap+1e-9)
        if prev is None:
            if observed and (abs(a)-angle_error>=3 or abs(r)-angle_error>=3 or abs(k)-key_error>=2):
                return "invalid reset"
            unclear |= abs(a)+angle_error>=3 or abs(r)+angle_error>=3 or abs(k)+key_error>=2
        prev=t
        twist=abs(PRELOAD+a-r)
        if observed and (collision or not intact or abs(r)-angle_error>=LIMIT or
                (k-key_error>=DISENGAGE and twist-2*angle_error>=LIMIT) or
                k-key_error>50 or k+key_error< -2):
            return "0"
        unclear |= abs(r)+angle_error>=LIMIT or (k+key_error>=DISENGAGE and twist+2*angle_error>=LIMIT)
        unclear |= k-key_error< -2 or k+key_error>50
        certain=observed and clear and k-key_error>=PARK and abs(r)+angle_error<3 and twist+2*angle_error<3
        if certain and dwell_start is None:dwell_start=t
        if not certain:dwell_start=None
        possible=observed and clear and k+key_error>=PARK and max(0,abs(r)-angle_error)<3 and max(0,twist-2*angle_error)<3
        if not possible:possible_start=None
        elif possible_start is None:possible_start=t
    if prev is None:raise ValueError("Empty worksheet")
    if unclear:return "inconclusive"
    if dwell_start is not None and prev-dwell_start>=.5:return "1"
    if possible_start is not None and prev-possible_start>=.5-max_gap:return "inconclusive"
    return "0"


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("worksheet",type=Path)
    for name in ("angle-error","key-error","max-gap"):p.add_argument("--"+name,type=float,required=True)
    a=p.parse_args()
    with a.worksheet.open(newline="") as f:print(evaluate(csv.DictReader(f),a.angle_error,a.key_error,a.max_gap))
