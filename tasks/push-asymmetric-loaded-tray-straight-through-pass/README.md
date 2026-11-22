# Push an asymmetrically loaded cafeteria tray straight through the counter pass-through

Push an asymmetrically loaded cafeteria tray along its offset center of friction (`y_cf ≈ +69 mm`), or the
unbalanced frictional yaw torque spins the tray crooked (`+26°..+41°`) and physically wedges it inside the 18-inch
counter pass-through aperture.

No existing benchmark task tests this ability:

- **#388** [FINAL, TK-8BVXP4] (*pass-parcels-through-a-shared-window*): "Importance (not met): The partner's
  automatic retreat removes the need for the controlled arm to negotiate the opening... Construct validity (not
  met): A robot can carry a cube below wall height around an end of the wall and receive credit without using the
  opening." Here the 1.20 m full-counter partition plus transom beam physically blocks bypassing, and the
  difficulty is 2D planar asymmetric-friction yaw control, not obstacle-free cube carry.
- **#286** [FINAL, TK-SG3296] (*Clear a bussing tray*): an item-sorting / miniaturization task ("Solid glass
  cylinders and 3 cm-thick, 5–8 cm-wide plate discs eliminate hollow-glass orientation and realistic rim-gripping
  skills."), not sliding a loaded full-scale tray through a pass-through.
- **#1554** [MERGED]: 1D longitudinal momentum dosing on passive rollers, not 2D planar asymmetric-CoM yaw
  compensation.

Instruction: "Push the loaded tray cleanly through the counter pass-through window onto the far side without
wedging or spilling either item."

Files in this folder: `task.json`, `push_asymmetric_loaded_tray_straight_through_pass.py` (scene, scorer,
scripted demo), `labkit.py` (multi-shape parts and step hooks), `arena_assets_fix.py` (library asset paths),
`test_push_asymmetric_loaded_tray_straight_through_pass.py` (offline tests), `README.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment,
`station` scene. Task file SHA-256 (first 12): see the freeze log.

## Scene
- `slide_mat` (fixed): 0.82 x 0.90 x 0.005 m polypropylene service mat covering the counter x in [0.00, 0.82];
  the tray's slide surface.
- `pass_frame` (fixed): 12 mm partition walls spanning y in [-0.600, +0.600] at x = 0.355, a single 18-inch
  (457.2 mm) clear aperture at y = 0.075, a 40 mm transom beam over the opening (clear above 0.44 m), a flush
  steel sill and bronze trim. No route around the window exists.
- `tray` (loose, 0.359 kg): rigid 302 x 410 x 20 mm fast-food tray (4 non-overlapping corner pads + floor +
  4 rim walls + corner ribs), sitting flat on the mat at (0.195, 0.075), long side along +Y.
- `heavy_dish` (loose, 1.45 kg): hollow 203 x 203 x 64 mm clear glass baking dish on the tray, centred 95 mm
  toward +Y of the tray midline.
- `tumbler` (loose, 0.073 kg): 76 mm dia x 130 mm clear plastic tumbler on the tray, centred 105 mm toward -Y.

The load sits asymmetric on purpose: tray mass is symmetric, so the loaded centre of friction is the load's --
`LOAD_CF = (1.45 x 0.095 - 0.073 x 0.105) / 1.882 ≈ +0.069 m` off the midline, i.e. `Y_CF ≈ 0.144` in world y.

## How it is modelled
| Mechanism | Physical body / joint / contact | Source for the numbers |
|---|---|---|
| Cafeteria tray | one loose rigid body, 4 corner pads + floor + rim walls + corner ribs, 302 x 410 x 20 mm, 0.359 kg | 12 x 16 in polypropylene fast-food tray (see Real-world build) |
| Glass baking dish | loose rigid shell, base + 4 walls + 2 grip ears, 203 x 203 x 64 mm, 1.45 kg | 8-inch square clear glass baking dish |
| Tumbler | loose rigid body, base disc + tapered body + rim ring, 76 mm dia x 130 mm, 0.073 kg | 12.6 oz clear plastic tumbler |
| Pass-through | fixed partition + 457.2 mm aperture + transom + sill | 18-inch manual pass-through counter window |
| Slide friction | PhysX average pair, mat and tray both mu = 0.44 | PP-on-PP dynamic coefficient, maker datasheet (Real-world build) |
| Asymmetric friction | rigid contact: the off-centre normal load makes the midline push torque unbalanced | centre-of-friction model |

## Scoring (see `score()`; text == code constants)
- 0.0 if the tray dumps past 10 deg, or an item spills (tilts past 15 deg or leaves the tray), or the tray
  skews past the 12 deg wedge limit, or it drifts more than 120 mm sideways, or its front edge passes the
  800 mm delivery limit.
- 0.5 if the tray is straight (yaw <= 8 deg) with both items seated upright and its front edge engaged in the
  window while its rear edge has not yet cleared the far face of the 12 mm frame wall (rear < frame + 6 mm).
- 1.0 if the rear edge is at least 6 mm past that far face, the front edge stays within 800 mm of the table
  origin, yaw <= 8 deg, both items upright and seated, everything at rest, both grippers >= 40 mm clear.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_push_asymmetric_loaded_tray_straight_through_pass.py`
with `yam_arena` on `PYTHONPATH`: start/holding-still score 0.0, partial window engagement 0.5, clean delivery
1.0, grippers inside the 40 mm clearance score 0.0, wedged yaw (>12 deg) 0.0, 9 deg skew 0.0, spilled items 0.0,
lateral bypass 0.0, front past the delivery limit 0.0; both demo plans are IK-reachable (< 2 mm, < 2 deg), the
demo reads no simulator state, the scoring text matches the constants, and both top cameras see every scored
object with >= 40 mm sight-line clearance at the demo's actual end pose (URDF forward kinematics of the final
parked joint angles, plus a positive-control blocker test).

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=push_asymmetric_loaded_tray_straight_through_pass.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Push the loaded tray cleanly through the counter pass-through window onto the far side without wedging or spilling either item." \
  --scorer task_score --max-steps 1200 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=push_asymmetric_loaded_tray_straight_through_pass.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Push the loaded tray cleanly through the counter pass-through window onto the far side without wedging or spilling either item." \
  --scorer task_score --max-steps 1200 --seed 0
```
`demo()` reads geometry constants only. The left arm positions a 35-deg forward-pitched, roll-0 half-open
two-finger jaw (`PUSH_FINGER = -0.020`) behind the tray's rear rim (`PX0 = 0.026`) at `TIP_Z_OFF = MAT_T + 0.008`
and pushes along the loaded centre-of-friction line `PUSH_Y = 0.144` across three stroke segments to `PX1 = 0.485`,
then backs off `-X` and swings outboard to park clear of both top cameras; the right arm stays parked off the
counter. No pick-and-release occurs, so the undo-replay probe finds nothing to undo. The 1.0 s success hold
completes at about 223 steps (11.1 s) of the 60 s run limit.

## Demo results
| Check | Result |
|---|---|
| Offline MuJoCo physics check (`mj_check`), scripted demo | 1.0 at 223 steps (~11.1 s); `verdict` PASS, `ik_worst_mm` 0.197, `end_occluded_by_arms` {}, undo skipped (no release), all 7 robustness cases 1.0 |
| GPU simulator (cloud, isaacsim 6.1.0.0), scripted demo | 1.0 at 216 steps, ended by success |
| GPU simulator (cloud), Dream-1 with the instruction above | 0.0 over the full 1,200-step run |
| Offline MuJoCo, holding still at ready pose | 0.0 |
| MuJoCo friction sweep, demo | 1.0 at friction x0.70 / x0.85 / x1.00 / x1.30 (yaw -1.1..-5.5 deg) |
| MuJoCo friction sweep, geometric-midline push (y = 0.075) | 0.0 at x0.70 / x1.00 / x1.30, wedged yaw +39.9..+41.6 deg |

## Real-world build
One-time parts cost for one trial set: $12.99 (`real_trial_cost_usd`) -- the per-unit sum of tray $3.02 +
dish $8.83 + tumbler $1.14, excluding the reusable counter structure and mat.

- 12 x 16 in black polypropylene fast-food tray, 302 x 410 x 20 mm, 0.359 kg, $3.02 each --
  https://www.webstaurantstore.com/cambro-1216ff110-12-x-16-black-customizable-fast-food-tray-case/2141216FFBK.html
- 8-inch square clear glass baking dish, 203 x 203 x 64 mm, 1.45 kg, $8.83 each --
  https://www.webstaurantstore.com/anchor-hocking-preferred-bakeware-8-clear-glass-square-baking-dish-81934l20-case/55081934.html
- 12.6 oz clear plastic tumbler, 76 mm dia x 130 mm H, 0.073 kg, $1.14 each --
  https://www.webstaurantstore.com/cambro-nt12152-newport-12-6-oz-clear-customizable-plastic-tumbler-case/214NT12CL.html
- Pass-through aperture reference: 18 in = 457.2 mm clear width drive-thru window --
  https://www.webstaurantstore.com/ready-access-65163301-bo-10-bronze-manual-open-self-close-drive-thru-window/71A65163301.html
- Friction citation: INEOS polypropylene engineering properties (0.44 dynamic, 0.76 static) --
  https://www.ineos.com/globalassets/ineos-group/businesses/ineos-olefins-and-polymers-usa/products/technical-information--patents/ineos-engineering-properties-of-pp.pdf

Importance citation: BLS OEWS May 2025 occupation 35-3023, Fast Food and Counter Workers -- 3,854,050 US jobs;
O*NET task summary https://www.onetonline.org/link/summary/35-3023.00.

Real-trial verification protocol:
- At the end of the trial, check the tray's rear rim is fully past the window frame's far face, its front rim
  still on the counter, the tray straight within a few degrees, and both items upright inside it.
- Review the run video: a tray dumped past 10 deg, an item tipped past 15 deg or off the tray, a skew past
  12 deg in the aperture, sideways drift past 120 mm, or a hand left within 40 mm scores 0.0.

## Runtime per trial
1,200 control steps max at 20 Hz (60 s simulated time); the scripted demo finishes once the 1.0 s success hold
completes (about 175 steps / 8.7 s).

## Seeds
Seed 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Set the empty tray flat on the slide mat at its start mark, centre the glass dish on its +Y mark and the
tumbler on its -Y mark, and send both arms to ready pose.

## Licence
No mesh or texture assets: every part is built from boxes and cylinders in
`push_asymmetric_loaded_tray_straight_through_pass.py`; original work, released under the repository's MIT
licence.
