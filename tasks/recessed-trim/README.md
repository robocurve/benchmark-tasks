# Fit a downlight trim overhead: squeeze each torsion spring shut and thread it up through its bracket slot, flange flush

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and geometry, scorer, fast IK) and `tests/` their unit checks.

## The task
A 6 in. recessed-light housing is installed above a section of drywall ceiling; its BR30 lamp is already in the
socket. The housing has two torsion-spring brackets inside the can, on opposite sides, at whatever angle the housing
was installed. The trim (a 6 in. baffle trim with two torsion springs) rests flange-down on its carton on the bench
below. The robot lifts it, squeezes each spring's two wire legs shut, turns the trim so that each spring is under its
bracket, passes the squeezed legs up through the bracket slots, lets the springs go (the legs spread and hook above the
slots), pushes the trim up until the flange is flat on the ceiling, and lets go. The springs then hold the trim up.

Why it matters: fitting the trim is the last step of every recessed downlight an electrician installs (O*NET 47-2111
Electricians, about 760,000 US jobs; task "Install ... lighting fixtures"). Torsion-spring trims are a standard way of
mounting downlight trims in recessed housings.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
Every level needs the robot off the trim and its springs (no contact force).
- 1.0 (ends the trial): both springs captured (each spring's two legs pass up through its bracket's slot and their tips
  are above the bracket tab) and the flange flat on the ceiling: the gap between the flange's top face and the ceiling
  is at most 2 mm at 12 points round the flange (radius 92 mm); trim still for 1 s.
- 0.5: both springs captured, flange not flush.
- 0.25: one spring captured (the trim hangs from it).
- 0: no spring captured (trim on the carton, dropped, resting on the springs below the brackets, or held by the robot).
Real trial: look up past the flange into the can (both wire ends of each spring above its bracket, through the slot),
check that the trim stays up with the hands off, and try a 2 mm feeler (or a coin of that thickness) under the flange
round the rim.

## Dream-1 evaluation

Dream-1 scored 0.0 on seed 0 at the 2,000-step limit. Scripted demo: 1.0 in the official Inspect Robots run (default seed, 944 of 2,000 steps); 1.0 twice, identical, at the default seed; 1.0 at seed 7. Doing nothing: 0. Demo replayed open-loop at seed 3: 0. Trim not turned: 0.

From the task's `isaac/` directory in the Isaac Sim 6.1 and Isaac Lab-Arena environment, run:

```bash
instruction=$(python -c 'import json; print(json.load(open("../task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --seed 0 --max-steps 2000 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy recessed_trim_scripted)
python official_run.py -- run --embodiment yam_arena --policy recessed_trim_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 2000 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 2000

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo              # named setup 0; TRIM_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage demo --seed 7     # a seeded start state, as in inspect-robots runs
python demo.py task.py <out>/<run> --stage neg-no-turn       # trim not turned to the brackets: legs meet the tabs: 0
python demo.py task.py <out>/<run> --stage neg-clock-zero --seed 1   # turned to the left-right line, not to the brackets: 0
python demo.py task.py <out>/<run> --stage look --cam-size 1280x704  # setup view: right wrist camera looks up into the housing
python demo.py task.py <out>/<run> --stage hold              # both arms hold still for 2000 steps: 0
python demo.py task.py <out>/<run> --stage neg-light-pinch   # springs only partly squeezed: hooks cannot pass: 0
python demo.py task.py <out>/<run> --stage neg-no-push       # both springs in, flange never pushed up: 0.5
TRIM_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run> --stage replay --seed N   # open-loop replay
```
Unit checks (no simulator): `python tests/test_scoring.py`. Executed-vs-commanded check of an official run:
`python isaac/action_check.py <official log dir>`; stimulus checks of negatives: `python isaac/negcheck.py out.json <runs>`.

Start states: every trial reset with a seed (every `inspect-robots run` passes one) turns both brackets by phi, drawn
uniformly from 3.5 to 5 deg either side of the robot's left-right line, turns the trim on its carton within 3 deg of
that line and at least 4 deg away from phi, and moves the carton by up to 15 mm in x and 6 mm in y. The default
inspect-robots seed gives phi +4.1 deg, trim -2.8 deg. Unseeded resets (direct `demo.py` runs) use the named setups
(phi, trim) = (-4.5, 2), (4, -2), (5, 1), (-3.5, 2.5), (3.5, -2.5) deg in turn (`TRIM_SETUP` picks the first; set, it
also forces named setups on seeded resets). The ranges are set by the arms' reach: the side pinch on both springs that
keeps the grippers under the drywall is reachable only with both springs near the robot's left-right line.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Recorded media

`model_run.mp4` shows the full seed 0 Dream-1 trial (2,000 steps, final task score 0.0).

- `official_tiled_top_left_right.mp4`: the official Inspect Robots run of the scripted reference policy at the default
  seed (brackets +4.1 deg, trim -2.8 deg on its carton), score 1.0 after 944 control steps (H.264, 960 x 176).
- `setup_hires_overhead_lookup.png`: the two high-resolution setup views side by side (2560 x 704); suggested as the
  setup image.

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Ceiling height and extent: the ceiling's underside is 1.20 m above the floor (45 cm above the bench top) so that the
  YAM arms reach it; real ceilings are 2.4-2.7 m and the electrician works from a ladder, with the hands at about the
  same distance below the ceiling as here. The drywall covers one joist bay round the housing (34 cm wide between two
  2x6 joists, 44 cm long). Beyond the joists the mock-up has no drywall: when the YAM's hands work just under the
  ceiling, its forearms rise above the ceiling plane at the sides of the bay (a person's forearms stay below it). The
  hands themselves work in the same cramped space under the drywall as on a real job: the fingers come within 3 mm of
  the drywall's underside while the springs go in.
- Trim: one free rigid body (flange, baffle cone, spring clips and coils as convex pieces), 0.30 kg; each of the four
  spring legs is a free rigid body (1.6 mm wire, 70 mm, with a 3 mm outward tip hook) on a revolute joint at the coil
  (hard joint limits at -8 and 80 deg, outside the working range). The torsion spring is a torque computed every
  physics step from the measured leg angle, 0.11 N m/rad per leg about a free half-angle of 50 deg, plus light damping
  and a stiff stop (3 N m/rad) where the squeezed legs meet (-1 deg), applied to the leg and equal and opposite to the
  trim (joint drives on these joints do not act in this PhysX build; a hard joint stop pressed by a finger made the
  light leg jump out of the fingers). A real double torsion spring couples its two legs through the coil; here each
  leg is sprung against the trim, which gives the same squeeze force when both legs are pinched together. Legs 6 g
  each (wire plus a share of the coil, heavier than the wire alone, for solver stability).
- Brackets: two horizontal steel tabs (1.5 mm) on the can wall with 5 x 16 mm slots; the legs, the hooks and the tabs
  interact by contact only. Capture, the springs' pull toward the ceiling and the flange's seat are all contact and the
  computed spring torque; there are no welds, snaps or proximity rules. The bracket and spring dimensions are
  representative of torsion-spring trims (no manufacturer drawing was available; see below).
- Housing, ceiling, lamp and carton are kinematic bodies; the housing's turn (the bracket angle) is set at reset.
- Friction estimates: wire on steel 0.35, trim on drywall / steel 0.4, carton 0.5, YAM finger pads as in yam_arena.
- Spring forces: squeezing a spring shut needs about 12 N per finger 8 mm from the coil (10 N at the demo's pinch 10
  mm from the coil; YAM limit about 40 N). The measured robot contact force on the legs while holding (lift, carry,
  line-up, insert) was 16-25 N (median about 16 N). The per-physics-step contact log also shows a short spike of
  78-85 N at the instant the closing fingers first meet the legs (one or two 20 Hz control steps, then back below
  20 N): a contact-impact transient of the solver, not a held grip force.
  With both springs in, the springs press the flange onto the ceiling with about 1.6 times the trim's weight (4.7 N
  from the legs pressing on the slot ends at flush), plus friction.
- Bracket angle range: phi is 3.5-5 deg either side of the robot's left-right line and the trim starts within 3 deg of
  it, because the YAM reaches the side pinch on both springs only near that line. Squeezed spring tips must be within
  about 2 deg of the bracket angle to pass the 16 mm slots.
- Work light: a light source (no body) 25 cm below the ceiling, because the station's lights are above the ceiling and
  the trim's underside and the housing would otherwise be in deep shadow on every camera.
- The scorer reads body poses and the robot-contact force on the trim and legs (collider identity, never contact
  positions).

## Real rig and parts
- Housing: 6 in. IC airtight new-construction housing with torsion-spring brackets, for example Elco EL7ICA (or
  equivalent), installed in 1/2 in. drywall (6-3/4 in. cut-out) across one joist bay of a frame of 2x6 joists on 2x4 posts;
  BR30 LED lamp in the socket.
- Trim: 6 in. shallow baffle trim with torsion springs, for example Elco ELM305W (or equivalent; the springs come
  fitted).
- Part numbers: the catalogue titles of both Elco parts were found by web search; their spec sheets could not be opened
  from the build host, so the dimensions are not checked against them.
- Model dimensions used here (not checked against the manufacturers' drawings, which could not be opened from the build
  host): can 172 mm inside diameter; brackets' tabs 20 mm above the ceiling underside, slots 5 x 16 mm on a 158 mm
  circle; trim flange 194 mm OD, baffle 148 mm at the flange, 75 mm tall; springs 1.6 mm wire, legs 70 mm.
- Carton: the trim's own 150 x 150 x 120 mm carton on the bench under the housing.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Ceiling, housing, brackets, lamp, trim, springs, carton geometry | generated in code by this task (`src/layout.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
