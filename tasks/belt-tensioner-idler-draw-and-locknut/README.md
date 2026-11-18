# Draw a belt-tensioner idler carriage to its calibrated tension mark and lock the clamp collar on its stud

Instruction: "Calibrate the idler carriage in its tension window using the setup gauge, clamp the stud collar down, return the gauge to its tray, and clear both grippers."

Files in this folder: `task.json`, `belt_tensioner_idler_draw_and_locknut.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_belt_tensioner_idler_draw_and_locknut.py`, `README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment, `station` scene.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_belt_tensioner_idler_draw_and_locknut.py` with `yam_arena` on `PYTHONPATH`: verifies initial slack state, clamp-seat and 18-24 mm draw bounds, all score tiers (0.0 / 0.4 / 0.7 / 1.0), negative shortcuts and unbolted-base push/tilt ablations, fail-closed missing-TCP cap, top-camera occlusion proxy clearance, scoring numeric parity, stateless scorer and demo purity, and IK reachability.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=belt_tensioner_idler_draw_and_locknut.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Calibrate the idler carriage in its tension window using the setup gauge, clamp the stud collar down, return the gauge to its tray, and clear both grippers." \
  --scorer task_score --max-steps 1040 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=belt_tensioner_idler_draw_and_locknut.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Calibrate the idler carriage in its tension window using the setup gauge, clamp the stud collar down, return the gauge to its tray, and clear both grippers." \
  --scorer task_score --max-steps 1040 --seed 0
```
`demo()` reads scene geometry constants only (never simulator state). Right gripper holds the unbolted 360 g sub-frame down on its press pad while left gripper transfers the 40 g acetal setup gauge from its tray into the pocket, pinches the X-pinch draw grip, and draws the carriage to the 21 mm calibrated mark against the 160 N/m (3.36 N) return spring. Right gripper re-indexes to the knurled collar wing-bar and drives two clockwise strokes (`-60 deg`, `+120 -> -50 deg`) so the helical thread advance seats the collar flush on the carriage plate, left returns the gauge to its tray, and both grippers park >= 45 mm clear for 3.0 s. Plan duration is 39.6 s; scripted rollout finishes in ~37.2 s inside a 1040-step (52 s) `run_limit`.

## Scoring
Judged at trial end from physical end-state geometry (verified on bench using a caliper, ruler, and angle gauge); succeeds once 1.0 holds for 3.0 s:
- `1.0`: carriage inside the 18 to 24 mm draw window; collar turned >= 180 deg and seated flush within 0.2 mm of the 1.0 mm stud gap; setup gauge returned within 75 mm of its tray and >= 60 mm from the frame pocket; sub-frame within 25 mm XY and 8 deg tilt; both grippers >= 45 mm clear of post, collar, and gauge.
- `0.7`: clamped inside window, gauge left in pocket/away from tray, or gripper within 45 mm.
- `0.4`: held in the 18 to 24 mm window without collar clamping.
- `0.0`: otherwise (noop, sub-frame shove without draw, unclamped spring recoil, over-draw past 24 mm, or > 8 deg tilt).
Possible scores: `0.0`, `0.4`, `0.7`, `1.0`.

## Runtime per trial
Runtime per trial: 1040 control steps at 20 Hz (52 s of simulated time, about 5 min wall clock on one A10 GPU); scripted demo completes in ~37.2 s once 1.0 holds for 3.0 s.

## Seeds
Seeds: 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Slide the idler carriage back to 0 mm slack, spin the clamp collar back off the carriage plate, return the acetal setup gauge to its left tray at (0.190, 0.250) m, dock the sub-frame front face against the datum block at (0.300, 0.000) m, and open both grippers.

## Mechanism and sources
| part | real item | model | precedent |
|---|---|---|---|
| unbolted linear actuator sub-frame + carriage | extruded anodised linear slide frame (`320 x 340 mm`, `360 g`) with belt-tensioner idler carriage (`95 g`, 42 mm pulley) | compound rigid frame + `draw` slide joint (`160 N/m`, `3.36 N` at `21 mm`), `22 mm` limit, rule-driven gauge stop (`K_STOP = 2600 N/m`), and `-X/+X` pocket walls (`22 mm` tall) (source: https://www.mcmaster.com/products/timing-belt-tensioners/) | industrial actuator tensioning |
| helical locknut clamp collar | knurled M8 threaded clamping collar (`40 g`) on stud (`y = -0.085`) | coupled `clamp` axial slide (`4.0 mm` total travel, `1.0 mm` per `180 deg` / `2.0 mm/rev`) + `spin` hinge with T-wing grip bar and `K_CLAMP = 5200 N/m` lock (source: https://www.mcmaster.com/products/locknuts/) | positive-lock fastener |
| stepped acetal setup gauge + tray | 6 mm acetal tensioning feeler gauge (`40 g`) in benchtop holder | compound block with 6 mm stem, 26 x 28 x 22 mm head, 28 x 36 mm overhang lip, and fixed bench tray (source: https://www.mcmaster.com/products/feeler-gauges/) | precision go/no-go spacer |

## Real-world build
Components: machined aluminium linear-slide sub-frame with flanged idler pulley and GT2 belt section, M8 threaded post and knurled clamping collar with wing grip, acetal 6 mm setup gauge and tray, and datum block (~$22 total BOM). Place the sub-frame at (0.300, 0.000) m, datum block at (0.480, 0.000) m, and gauge tray at (0.190, 0.250) m.

## Fidelity
Isaac Sim 6.1 rigid-body simulation (PhysX, 20 Hz control). Carriage return preload uses a 160 N/m spring (`3.36 N` at `21 mm`); collar thread advance couples a hinge (`spin`) to a 4 mm `clamp` slide joint (`1.0 mm` per `180 deg` turn). Rule-driven pocket gauge-stop (`K_STOP = 2600 N/m` at `21 mm`) and clamp lock (`K_CLAMP = 5200 N/m` once seated past `180 deg`) modulate joint drives via task rules. Elastomer belt creep and thread backlash are omitted.

## Licence
All shapes are procedural primitives authored in `belt_tensioner_idler_draw_and_locknut.py`; no external third-party mesh, texture, or font files are included (original own work).
