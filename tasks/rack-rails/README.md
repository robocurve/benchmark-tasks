# Hang a telescoping server rail across a rack: latch one end, draw it out and find the same U's holes on the far post

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and rack geometry, latch rule and scorer, fast IK) and `tests/` their unit checks.

## The task
A 12U four-post open-frame rack stands on the bench with its side open toward the robot: the near-side rear post on the
robot's left, the near-side front post on its right, 840 mm apart (mounting depth), both with EIA-310 square-hole
flanges and printed U numbers. A 1U server (blue lid) is already installed on its rails across both posts. The new
server's sliding rail lies closed in a foam cradle in front of the rack. The robot lifts the rail with both hands,
carries it level into the rack's near side at the free U directly above the installed server, pegs and latches the
rear end on the rear post, then
draws the telescoping front member out about 40 mm to the front post, pegs it in the same U and latches it, and lets
go.

Why it matters: racking servers is the physical step of every AI cluster build-out; every 1U/2U server goes in on a
pair of rails hung like this, at the U the rack elevation plan gives.

What makes it hard:
- Same-U alignment across a 0.84 m span at 0.25 mm peg play: the closed rail (800 mm) is shorter than the post
  spacing (840 mm), so one end is pegged and latched first; then the telescoping member is drawn out about 40 mm
  against its slide drag and its pegs must find the other post's holes at the SAME U, 0.84 m away.
- Each end has two 9.0 mm square pegs in 9.5 mm square holes (0.25 mm play a side) at 31.75 mm, which matches only one
  U's top and bottom holes: an end can seat only on a U boundary, and an end one U off gives a sloped rail (score 0).
- Each end's spring catch needs a push of more than 10 N over its last 2.5 mm; a light touch does not latch it, and an
  end pushed part way springs back out of the catch zone.
- The target U changes per trial (U2, U3 or U4): the cue is the installed server and its rail directly below it
  (visible across the rack in the overhead camera), plus the U numbers and U boundary ticks printed on both posts'
  legs.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 0 if either end is latched or pegged (pegs in the flange's holes) at any U other than the target.
- 0.25: one end latched at the target U.
- 0.5: one end latched at the target U and the other end's pegs in the target U's holes but not latched; or both ends
  latched at the target U with a fingertip still within 10 cm of the rail.
- 1.0 (ends the trial): both ends latched at the target U, every fingertip 10 cm or more from the rail, rail still
  (< 2 mm/s) for 1 s.
- Otherwise 0 (rail in the cradle, in the air, pegs not in).
Real trial: each end's latch tab (green flush = latched, orange out = open) and a tug on the end; the U from the
printed numbers next to the pegs; hands from the video.

## Dream-1 evaluation

Dream-1 returned no usable action on seed 0. Inspect Robots completed 32 warm-up steps and produced no task score. The policy error was `PolicyError: q_r_chunk gripper leaves 0..1: [-0.123, 0.913]`. Scripted demo: 1.0 in the official Inspect Robots run (default seed, target U4, 853 of 2400 steps); 1.0 twice from setup 0 (U3); 1.0 at seed 21 (U2). Doing nothing: 0. Open-loop replay at 4 other starts: 0. Mounted one U high: 0.

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
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy rack_rails_scripted)
python official_run.py -- run --embodiment yam_arena --policy rack_rails_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 2400 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 2400

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo            # named setup 0 (target U3); RACK_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage demo --seed 7   # a seeded start state, as in inspect-robots runs
python demo.py task.py <out>/<run> --stage neg-wrong-u     # mounted and latched one U high (a gap above the server): 0
python demo.py task.py <out>/<run> --stage neg-front-no-push # rear latched, front pegs in but never pushed past its catch: 0.5
python demo.py task.py <out>/<run> --stage neg-rear-only   # rear latched, front never drawn out: 0.25
RACK_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run2> --stage replay --seed 7   # open-loop replay
```
Unit checks (no simulator): `python tests/test_scoring.py`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Recorded media

Only the camera warm-up was recorded. The submitted trial video shows the scripted reference.

- `official_tiled_top_left_right.mp4`: the official Inspect Robots run of the scripted reference policy at the default
  seed (target U4), score 1.0 after 853 control steps (H.264, 960 x 176).
- `setup_overhead_hires.png`: the start state of named setup 0 (target U3) from the overhead camera at 1280 x 704.

## Start states
Every trial reset with a seed (every `inspect-robots run` passes one) draws the target U uniformly from U2, U3, U4 (the
installed server sits one U lower), moves the rack by 4 to 8 mm in x (never within 4 mm of its nominal place) and up to 6 mm in y and the rail's cradle by up to 10 mm in
x, 12 mm in y and 3 deg in yaw. The default inspect-robots seed gives target U4. Unseeded resets (direct `demo.py`
runs) use the named setups 0-4 in turn (`src/layout.py` SETUPS; `RACK_SETUP` picks the first and, when set, also forces
named setups on seeded resets).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 240 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Rack: modelled on a StarTech.com 4POSTRACK12U (12U four-post open frame, 22-40 in adjustable mounting depth; set to
  840 mm). Its posts are modelled as square-hole EIA-310 flanges (9.5 mm holes, hole pattern 15.875 / 15.875 / 12.7 mm,
  465.1 mm hole column spacing) with an outer leg and a back plate; the frame, far-side posts and printed U marks are
  simplified. The manufacturer specifies square cage-nut holes and base anchoring holes. [StarTech datasheet](https://media.startech.com/cms/pdfs/4postrack12u_datasheet.pdf). The rack stands on its
  base frame on the bench (casters off) so that its lowest U are within the arms' reach (U1 lower boundary 12 mm above
  the bench); only U2-U4 are used as targets (the YAM pair's top-down reach ends near a 0.96 m tool height).
- Rail: the custom tool-less training rail built to the endplate and hook drawings, with dimensions outer member 14 x 40 x 620 mm, inner member 10 x 30 x 500 mm, closed length 800 mm bracket face to
  bracket face, 80 mm telescoping travel, end brackets 3 mm thick with two 9.0 mm square pegs 8 mm long at
  +-15.875 mm (nose tapered 1.5 mm a side over its last 3 mm: a rounded stab-in nose modelled as a frustum), 1.45 kg
  in total. The collision profiles are solid boxes. The fabricated outer member is hollow to admit the inner member.
- The two members are free rigid bodies on a prismatic joint (hard limits 0-80 mm; they do not collide with each
  other). Drives on such (maximal) joints do not act in this PhysX build, so the slide drag is a computed force:
  -clip(150 N s/m x relative speed, +-4 N), applied every physics step to both members (estimate; a real ball-bearing
  slide has a small static drag that this viscous-saturating model does not hold at rest).
- Latch (spring catch at each end, `scoring.Latch`, from the measured member pose every physics step): while the end's
  pegs are in its flange's holes (peg axis within 1.2 mm of the hole pair's axis, within 1.5 deg of square) and its
  bracket is within the last 2.5 mm of travel to the flange, a 10 N force pushes the member back out; when the bracket
  face reaches the flange (gap <= 0.4 mm) the catch is engaged, at whatever U the end is in; from then on a stiff
  spring (20 kN/m, up to 150 N) keeps the end from backing out. 10 N is the replica calibration target. The actual peaks are recorded during physical commissioning. The
  latch tabs (visual only) switch from orange to green when the catch engages. Nothing pulls a free end in, centres
  it or holds it sideways: the pegs in the square holes do that by contact.
- Rack, installed server (with its own near-side rail, latched) and cradle are kinematic rigid bodies (contact forces are reported). Friction (static = dynamic): rail
  on the gripper pads 0.5 and steel on steel 0.4 (combined by PhysX averaging); estimates.
- Contact offset 0.1 mm, rest offset 0 (below the 0.25 mm peg play a side). Rail bodies never sleep.
- Nothing welds, snaps or teleports the rail; the scripted demo only sends the 14 joint / gripper commands and reads
  poses. The scorer only reads. On an environment reset (between trials only) the kit puts the rail back in its
  cradle (closed; re-applied once more after the reset's settling steps) and moves the rack, server and cradle to
  the next start state.
- Grip: the reference demo commands its fingers 4 mm inside the rail's web faces against the 5000 N/m finger drive,
  about 20 N per finger (measured peaks 20-23 N in the demo and official runs), under the YAM's 40 N.

## Real rig
- Rack: StarTech.com 4POSTRACK12U (or any 12U four-post square-hole open frame rack), mounting depth set to 840 mm,
  standing on the bench with the casters removed; near side facing the robot.
- Practice rail: fabricate both latch ends, telescoping members and two-state flags from the dimensioned drawings and parts list in [fixture/assembly.md](fixture/assembly.md). Through-bolt the four rack feet to the bench as specified there. Qualify insertion force, retention, end gap and flags at both ends before real trials.
- An installed 1U server (any 1U chassis about 760 mm deep; the sim renders its lid blue for contrast) on its own rail kit at the
  trial's U - 1.
- Foam cradle: the rail kit's box insert, or two blocks of foam with an 18 mm slot, in front of the rack.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Rack, rails, installed server, cradle geometry | generated in code by this task (`src/layout.py`, `isaac/rackkit.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
