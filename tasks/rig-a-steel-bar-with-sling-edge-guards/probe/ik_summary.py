"""Compact tables from ik_scan.py's JSON.   python ik_summary.py out/ik_X.json [...]"""

import json
import sys


def main(path):
    r = json.load(open(path))
    print(f"=== layout {r['layout']}  bar {r['bar']}  bowl bottom {r['hook_bowl_bottom_mm']}  throat {r['throat']}")
    print(f"    edges {r['edges']}")
    if "guards" in r:
        print("  guards: edge|family|arm  ok/n  clear(bar&hook>=20)/n  best: margin  bar_body bar_fing hook_body  tcp  approach  worst")
        for k, g in r["guards"].items():
            print(f"    {k:22s} {g['n_ok']:3d}/{g['n']:<3d} {g['n_clear']:3d}  "
                  + (f"m={g['margin_deg']:5.1f} bar={g.get('bar_body_mm', '-')!s:>6} fing={g.get('bar_fingers_mm', '-')!s:>6} "
                     f"hook={g.get('hook_body_mm', '-')!s:>6} tcp={g['tcp_mm']} a={g['approach']} {g['worst_joint']}"
                     if g["ok"] else f"NO SOLUTION (err {g['err_mm']} mm, rot {g['rerr_deg']} deg, {g['worst_joint']})"))
    if "hang_summary" in r:
        print("  hang paths:", r["hang_summary"])
        print("  hang waypoints ok:", r["hang_waypoint_ok"])
        for h in r["hang_best"][:5]:
            print(f"    {h['arm']:5s} alpha={h['alpha']} gamma={h['gamma']} sgn={h['sgn']} min_margin={h['min_margin_deg']} "
                  f"hook_body={h['min_hook_body_mm']} bar_body={h['min_bar_body_mm']} jump={h['max_jump_deg']}")
            for w in h["waypoints"]:
                print(f"       {w['tag']:15s} tcp={w['tcp_mm']} a={w['approach']} m={w['margin_deg']} hook_body={w.get('hook_body_mm')} "
                      f"hook_fing={w.get('hook_fingers_mm')} bar={w.get('bar_body_mm')} {w['worst_joint']} q={w['q_deg']}")
    if "steady_summary" in r:
        print("  steady:", r["steady_summary"])
        for arm, rows in r["steady_best"].items():
            for s in rows[:3]:
                print(f"    {arm:5s} {s['family']} v={s['v']} az={s['az']} m={s['margin_deg']} hook_body={s['hook_body_mm']} "
                      f"bar={s['bar_body_mm']} tcp={s['tcp_mm']} a={s['approach']} {s['worst_joint']}")
    if "pairs" in r:
        print("  pairs (hang arm + steady arm): arm-arm mm, steady-to-fitting mm")
        for p in r["pairs"][:6]:
            print(f"    hang {p['hang_arm']} (a={p['alpha']} g={p['gamma']} s={p['sgn']}, m={p['hang_min_margin_deg']}, hook={p['hang_min_hook_body_mm']})"
                  f" + steady {p['steady_arm']} {p['steady']['family']} v={p['steady']['v']} az={p['steady']['az']} "
                  f"m={p['steady']['margin_deg']} hook={p['steady']['hook_body_mm']} -> arm-arm {p['arm_arm_mm']}  steady-fit {p['steady_to_fitting_mm']}")
    print(f"  ({r.get('seconds')} s)")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        main(p)
