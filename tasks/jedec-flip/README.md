# Flip 21 BGAs ball side up by sandwiching their JEDEC tray under an empty one, lip seated and pin-1 chamfers together

Bimanual YAM task for Isaac Lab-Arena (`yam_arena` embodiment, station scene). Category: Impact on AI timelines
(chip inspection).

## The task
Instruction: "Turn the BGA packages in the loaded tray over into the empty tray, ball side up, for ball inspection, and
put the old tray aside on the table."

In AI-accelerator package inspection, BGA packages are checked ball side up (missing, bridged or damaged balls,
coplanarity), so before ball-side inspection they must lie dead-bug. Packages are never handled one by one for this: an empty JEDEC tray is stacked on the loaded tray, the
two-tray stack is held together at both ends and turned over, and the old tray, now on top, is lifted off. The
packages then lie ball side up in the cells on the underside of the new tray.

What makes it hard for a robot:
- Registration: the empty tray must be put down on the loaded tray so that its walls drop over the loaded tray's
  stacking lip (0.5 mm clearance a side, 0.4 mm lead-in), pin-1 chamfers together. The trays start in a different
  place every trial; the cue is visual (flush tray edges, matching chamfer corners, the stacking lip).
- The roll: the stack is 322.6 mm long and its trays only rest on each other; it is held at both ends with the
  pads above and below each end and turned 180 deg about its long axis, both arms moving together. Inside the stack
  each 12 g package has 3 mm of vertical play and drops from one tray into the other during the roll. A cover left
  riding on the lip shifts during the roll and the packages end up on the cell walls.
- The lift-off: the old tray is lifted straight off the new one by its end walls without dragging the parts out of
  their cells, and set aside.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
Levels are reached in order (person check in brackets):
- 0.25: the empty tray stacked on the loaded tray, walls over the stacking lip, edges flush, chamfers together
  (no step at the edges you can feel; sim: within 1.0 mm and 0.4 deg of the stacked pose, seated within 0.6 mm).
- 0.5: the stack turned over, all 21 packages still between the trays.
- 1.0 (ends the trial): every package lies flat, ball side up, in a cell of the new tray (photo; sim: within 1.2 mm
  of a cell centre, resting on the cell floor, within 2 deg of flat in the tray); the old tray lies on the table clear of the new
  tray; both hands at least 3 cm from the new tray and the packages; everything still for 1 s.
- 0 at any time: a package outside both trays (fell out on the riser, bench or floor: a dropped BGA is scrapped).
Otherwise the highest level reached.

## Dream-1 evaluation

The Inspect Robots run used model `dreamlabs/dream-1-beta` and returned a task score of 0.0 at the 2,400-step limit. The scripted reference run scored 1.0.
From the task's `isaac/` directory in the Isaac Sim 6.1 and Isaac Lab-Arena environment, run:

```bash
instruction=$(python -c 'import json; print(json.load(open("../task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --seed 0 --max-steps 2400 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy jedec_flip_scripted)
python official_run.py -- run --embodiment yam_arena --policy jedec_flip_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 2400 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera
python action_check.py <out>/official/policy_log.json                       # executed vs commanded actions

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 2400

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438   # the default inspect-robots trial seed
python demo.py task.py <out>/<run> --stage demo --seed 7            # another seeded start
python demo.py task.py <out>/<run> --stage demo                     # named setup 0 (JFLIP_SETUP=k for setup k)
python demo.py task.py <out>/<run> --stage neg-no-cover    # loaded tray gripped at its ends and rolled alone (no cover tray)
python demo.py task.py <out>/<run> --stage neg-blind-cover # empty tray put down where the loaded tray nominally is (never looked at)
JFLIP_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run2> --stage replay --seed 7   # open-loop replay
python demo.py task.py <out>/<run> --stage idle,demo       # reset check (several trials in one process)
```
Unit checks (no simulator): `python tests/test_scoring.py`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> top.mp4 left.mp4 right.mp4`.

## Recorded media

- `model_run.mp4` shows the full Dream-1 trial (seed 0, 2,400 steps, final task score 0.0). The three views run at 20 Hz in order from left to right: overhead, left wrist, right wrist.
- `official_tiled_top_left_right.mp4`: the official Inspect Robots run of the scripted reference policy at the default
  seed, score 1.0 after 1,429 control steps (H.264, 960 x 176). Packages show bright nickel lids at the start and dark
  ball fields at the end.
- `setup_overhead_hires.png`: the start state at 1280 x 704 (default seed): riser, loaded tray with 21 lids up, empty
  tray, end handling slots and pin-1 chamfers.

Start states: every trial reset with a seed (every `inspect-robots run` passes one) moves the loaded tray on its
riser by up to 10 mm in x, 5 mm in y and 4 deg, and the empty tray on the table by up to 12 mm in x, 15 mm in y and
8 deg, redrawing until the empty tray is at least 8 mm (x or y) or 3 deg away from a straight home-to-home move onto
the loaded tray and the loaded tray's ends are at least 3 mm off its nominal place (so a cover put down on the
nominal place without looking never fits the 0.9 mm lip capture). The default inspect-robots seed (1415158438) gives: loaded tray +4.1 / -4.6 mm, +2.5 deg; empty tray
-7.5 / +2.8 mm, -1.2 deg (11.6 mm, 7.5 mm and 3.7 deg apart). Unseeded resets (direct `demo.py` runs without
`--seed`) use the named setups 0-4 of `src/layout.py` (`SETUPS`) in turn (`JFLIP_SETUP` picks the first; set, it also
forces named setups on seeded resets).

Named check setups (offsets from the nominal places; x / y in mm, yaw in deg):

| Setup | Loaded tray on the riser | Empty tray on the table |
|---|---|---|
| 0 | +6 / -3, -2.5 | -10 / +12, +6.0 |
| 1 | -8 / +4, +3.0 | +10 / -14, -7.0 |
| 2 | +4 / -3, +1.5 | +12 / +10, +4.0 |
| 3 | +9 / +2, +3.5 | -12 / -8, -5.0 |
| 4 | -5 / -4, -3.5 | +6 / +15, +7.5 |

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Trays: the outline is the JEDEC matrix-tray outline (12.7 x 5.35 in = 322.6 x 135.9 mm, 7.62 mm stack pitch, pin-1
  chamfer on one corner). The interior is representative, not a vendor drawing (vendors' pocket and stacking details
  differ and were not available to us): 3 x 7 pockets 36 mm square at 40 mm pitch (0.5 mm a side on the 35 mm
  package), pocket walls 3.4 mm, floor web 1.2 mm, cell walls 2.6 mm deep on the underside on the same grid, a
  1.4 mm stacking lip on the wall top (0.5 mm clearance to the upper tray's wall, 0.4 mm lead-in) with a matching
  rebate under the wall, and a 70 x 16.8 mm handling slot at each end. Each tray is one free rigid body (0.20 kg)
  of 43 convex pieces; carbon-loaded mPPE is modelled only by its mass and friction (0.4, estimate). The JEDEC
  registration number of the outline was not verified.
- Packages: 35 x 35 mm lidded flip-chip BGA with 1156 balls (an FFVB1156-class FPGA package), each one free rigid body
  of three slabs: ball field 33 x 33 x 0.45 mm (one slab, not 1156 balls), substrate 35 x 35 x 1.15 mm, lid
  31 x 31 x 1.8 mm; 3.4 mm and 12 g in total (approximate; datasheet heights vary by package revision); friction 0.35.
- Riser: kinematic 150 x 220 x 55 mm block (ESD-safe riser) under the loaded tray, so both tray ends overhang 51 mm
  and fingers can get under them.
- Contact offset 0.3 mm (the rib gaps are 0.42 mm), rest offset 0. Bodies never sleep.
- Nothing welds, snaps or teleports a tray or package; the scripted demo only sends the 14 joint / gripper commands
  and reads poses. The scorer only reads poses. On an environment reset (between trials only) the kit puts the trays
  and packages back to the start state.
- Grip forces (finger contact forces on the trays, logged every physics step): about 10 N per finger on the tray end
  walls and about 10 N per finger (2 mm squeeze against the 5000 N/m finger drive) in the end grips during the
  roll; peaks in the official run 11 to 13 N per finger, far under the YAM's 40 N.
- Scoring uses poses only; "outside both trays" is a package centre outside both tray volumes.

## Real rig and replica
- Make two identical trays from [the tray solid in millimetres](fixture/tray-mm.stl). The solid is the union of the simulated collision pieces. The handling slots, top stacking lip, lower rebate, corner chamfer and underside receiving cells are included. Use a full-size build plate or a two-sided machining fixture with registered datums. Keep the lip and receiving cells free of support or burrs. Weigh each tray against the nominal 200 g and distribute any trim mass symmetrically outside the pockets and gripping areas.
- Make 21 reusable inert coupons to [the package envelope](fixture/package-mm.stl). Centre a 33 × 33 × 0.45 mm bottom layer, 35 × 35 × 1.15 mm substrate and 31 × 31 × 1.80 mm lid. One stock recipe uses aluminium for the first two layers and a laminated lid of approximately 1.40 mm aluminium and 0.40 mm brass. Face to the finished heights after bonding. Weigh each coupon and trim to 12.0 ±0.3 g. Mark the two faces with contrasting dots.
- Inspect the 36 mm pocket openings and 35 mm substrates (nominal 0.5 mm side clearance), confirm both trays nest at the 7.62 mm pitch without binding, then load all 21 coupons and do one hand flip over a catch surface. The lower tray initially supports the ball-field face. After rotation each lid must be supported in its own underside cell while the upper tray lifts away and exposes the marked ball-field faces. Reject a tray that allows coupons to cross pockets or pulls them up with the cover. The fixture reproduces this inspection transfer with inert coupons.
- Riser: a 150 x 220 x 55 mm ESD-safe block (dissipative foam or acetal) centred under the loaded tray.
- Positions as in `src/layout.py` (station frame): riser centre x 33 cm, y 0; empty tray centre x 15 cm, y 0, both
  trays with their length across the robot (along y), pin-1 chamfers at the corner on the robot's left nearest the robot. Start offsets
  per trial from the seed (see Start states); marks on the bench every 5 mm and 2 deg reproduce them.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Trays, packages, riser geometry | generated in code by this task (`src/layout.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
