"""Summarise a COOKIE_DEBUG=1 run log: each cookie's position on the blade while it is carried, and the last line.
Created 11_15_26. Usage: python dev/trace.py <log> [every]"""
import re
import sys

every = int(sys.argv[2]) if len(sys.argv) > 2 else 3
lines = [ln for ln in open(sys.argv[1]) if ln.startswith("COOKIE")]
for ln in lines[::every]:
    t = float(re.search(r"t=\s*([\d.]+)", ln).group(1))
    pitch, roll = re.search(r"pitch\s+([-\d.]+)", ln).group(1), re.search(r"roll\s+([-\d.]+)", ln).group(1)
    out = []
    for m in re.finditer(r"(c\d) \[([^\]]*)\] onblade \[([^\]]*)\] tilt\s+([\d.]+) gap ([\d.]+) (\S+)", ln):
        c, pos, ob, tilt, gap, st = m.groups()
        x, y, z = (float(v) for v in ob.split(","))
        if 60 < x < 215 and abs(y) < 75 and -34 < z < -15:
            out.append(f"{c} pos[{pos}] onblade[{ob}] tilt{tilt} gap{gap} {st}")
    if out:
        print(f"t={t:5.1f} pitch {pitch} roll {roll} | " + " | ".join(out))
if lines:
    print(re.sub(r"tip .*? \| c1", "| c1", lines[-1]).strip())
