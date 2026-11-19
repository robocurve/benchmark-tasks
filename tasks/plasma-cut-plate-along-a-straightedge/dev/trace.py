"""Dev only: read a PLASMA_DEBUG=1 log and report the cut along the line (standoff, distance from the line, gap to the
straightedge, travel speed per 20 mm), the straightedge's drift, and when the offcut dropped.  python dev/trace.py <stdout.log>"""
import re
import sys

import numpy as np

PAT = re.compile(r"PLASMA t=\s*([\d.]+) arc (\d) x\s+(-?[\d.]+) lat\s+(-?[\d.]+) h\s+(-?[\d.]+) tilt\s+([\d.]+) gap\s+(-?[\d.]+) bear (\d) v\s+(-?[\d.]+) "
                 r"hands (\S+) L \[([-\d., ]+)\] R \[([-\d., ]+)\] torch \[([-\d., ]+)\] guide \[([-\d., ]+)\] gyaw\s+(-?[\d.]+) off \[([-\d., ]+)\] "
                 r"cut (\d+) sound (\d+) hot (\d+) unsafe (\d) rested (\d+) clear (\d) still ([\d.]+) score ([\d.]+)")
v3 = lambda s: np.array([float(v) for v in s.split(",")])  # noqa: E731
rows = []
for line in open(sys.argv[1], errors="replace"):
    m = PAT.search(line)
    if m:
        rows.append(dict(t=float(m[1]), arc=int(m[2]), x=float(m[3]), lat=float(m[4]), h=float(m[5]), tilt=float(m[6]), gap=float(m[7]), bear=int(m[8]),
                         v=float(m[9]), hands=m[10], L=v3(m[11]), R=v3(m[12]), torch=v3(m[13]), guide=v3(m[14]), gyaw=float(m[15]), off=v3(m[16]),
                         cut=int(m[17]), sound=int(m[18]), hot=int(m[19]), unsafe=int(m[20]), rested=m[21], clear=int(m[22]), still=float(m[23]), score=float(m[24])))
print(len(rows), "steps")
if not rows:
    sys.exit()
every = int(sys.argv[2]) if len(sys.argv) > 2 else 20
for r in rows[::every]:
    print(f"t {r['t']:6.2f} arc {r['arc']} x {r['x']:6.1f} lat {r['lat']:6.2f} h {r['h']:6.2f} tilt {r['tilt']:4.1f} gap {r['gap']:6.2f} b{r['bear']} v {r['v']:6.1f} {r['hands']:20s} "
          f"L {r['L']} R {r['R']} guide {r['guide']} yaw {r['gyaw']:5.2f} offz {r['off'][2]:6.1f} cut {r['cut']} sound {r['sound']} hot {r['hot']} rested {r['rested']} clear {r['clear']} score {r['score']}")
arc = [r for r in rows if r["arc"]]
if arc:
    print(f"arc on {len(arc)} steps, t {arc[0]['t']:.2f}-{arc[-1]['t']:.2f}; h {min(r['h'] for r in arc):.2f}..{max(r['h'] for r in arc):.2f}; "
          f"lat {min(r['lat'] for r in arc):.2f}..{max(r['lat'] for r in arc):.2f}; gap {min(r['gap'] for r in arc):.2f}..{max(r['gap'] for r in arc):.2f}; "
          f"tilt max {max(r['tilt'] for r in arc):.1f}; bearing {sum(r['bear'] for r in arc)}/{len(arc)}")
    for lo in range(160, 380, 20):
        sel = [r for r in arc if lo <= r["x"] < lo + 20]
        if sel:
            v = np.array([-r["v"] for r in sel])
            print(f"  x {lo}-{lo + 20}: n {len(sel):3d} v {v.mean():5.1f} [{v.min():5.1f},{v.max():5.1f}] mm/s h {np.mean([r['h'] for r in sel]):5.2f} lat {np.mean([r['lat'] for r in sel]):5.2f} "
                  f"gap {np.mean([r['gap'] for r in sel]):5.2f} tilt {np.mean([r['tilt'] for r in sel]):4.1f} guide y {np.mean([r['guide'][1] for r in sel]):6.2f} yaw {np.mean([r['gyaw'] for r in sel]):5.2f}")
z0 = rows[0]["off"][2]
drop = next((r for r in rows if r["off"][2] < z0 - 5), None)
print("offcut z start", z0, "max |dz| before any cut", max(abs(r["off"][2] - z0) for r in rows if r["cut"] == 0), "| with 1..39 cut:",
      max([abs(r["off"][2] - z0) for r in rows if 0 < r["cut"] < 40] or [0]), "| dropped at t", drop and drop["t"], "cut then", drop and drop["cut"], "| final", rows[-1]["off"], "score", rows[-1]["score"])
