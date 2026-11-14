# Make a chainsaw safe to carry: engage the chain brake and fit the bar cover

A 10-inch cordless chainsaw lies on its side on the table, its bar pointing to the robot's right. The chain brake is off
and the bar is bare. A yellow plastic bar cover (a scabbard) rests on a holder beyond the bar tip. The instruction is:
**"Make the chainsaw safe to carry."**

To do it, the robot has to:
- know or read what makes a saw safe to carry (a warning printed on the housing says: engage the chain brake, fit the bar cover);
- swing the front hand guard forward, toward the bar tip, past its detent, until it snaps onto the brake stop (a label by the guard shows the direction);
- slide the 26.5 cm cover along the bar, from beyond the tip to the sprocket cover, without touching the chain;
- do both without pushing the 2.8 kg saw more than 3 cm or tipping it, and leave the arms clear.

## Why it matters

Carrying a chainsaw with its chain brake off and its bar uncovered is a standard way to be cut: a stumble, a swung saw or
a bumped trigger puts a moving or sharp chain against a leg. The Canadian Centre for Occupational Health and Safety's
page "Chainsaws - Handling, Transportation, and Storage" (https://ccohs.ca/oshanswers/safety_haz/chainsaws/handling.html,
read on 12 November 2026) says "Engage the chain brake when carrying the chain saw" and "Use a scabbard or bar guard to
cover the chain when carrying or transporting a chainsaw." Chainsaws are used by grounds maintenance workers, tree
trimmers, forestry workers, landscapers and many households. A robot that helps outdoors would have to leave a saw in
that state, and doing so takes two different manipulations: pushing a spring-detented lever past its click, and sliding a
loose cover along a bar without touching the chain. The order matters too: the brake goes on first, and the hands stay
off the bare chain. A gripper that touches the chain while the brake is off is a latched violation.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the saw (one articulation with a hinged hand guard), the scabbard, its holder, the brake's detent, `success()` and `score()` |
| `logic.py` | The scoring (pure Python and NumPy, no Isaac Sim): brake, cover coverage, arms clear, hold, latched violations |
| `geometry.py` | Every dimension, the layout (A or B), and the thresholds |
| `build_assets.py` | Writes the textures and `assets/chainsaw.usda`, `assets/chainsaw_b.usda` (layout B), `assets/scabbard.usda` (needs `pxr`, PIL and the DejaVu fonts) |
| `assets/` | The saw (one articulation: housing, bar, chain, handles, and the hand guard on a hinge), the scabbard, and their textures |
| `scenekit.py`, `demokit.py` | Copies of the accepted tasks' helpers (USD bodies and articulations, reset hooks; scripted-demo helpers), unchanged |
| `demo.py` | The scripted full-score demo and its ablation modes |
| `check_assets.py`, `check_plan.py`, `check_collisions.py` | Offline checks of the assets, of the plan's reach and of its clearances in world coordinates |
| `run.sh` | One command per trial: `./run.sh dream` or `./run.sh demo` (and the ablation modes) |
| `tests/` | Unit tests (scoring, geometry, task.json, demo plan, layout B), run without Isaac Sim |

`scenekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`). Build the assets once
(`python build_assets.py`; `assets/` is included).

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Make the chainsaw safe to carry." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/chainsaw-dream
```

`task_score` prints the trial's score: `score()` at the end of the episode (1.0 on success).

- **Layouts:** `TASK_VARIANT=A` (default) is the layout described here. `TASK_VARIANT=B` is its mirror image across the
  table's centre line (bar pointing to the robot's left, cover beyond the tip on that side, the right arm working the hand
  guard and the left arm the cover); set the variable before the command.
- **Seeds:** one fixed layout per variant and no randomisation (`object_jitter=0`), so the seed has no effect.
- **Runtime:** a full 2,400-step Dream-1 trial took about 14 minutes on the shared L40S GPU box (other simulations were
  running, load 10 to 25), plus about a minute to start Isaac Sim. A scripted demo takes about 2 to 3 minutes the same way;
  a successful trial ends earlier (the demo stops after 581 steps).

Dream-1 scored 0.0 over the full 2,400 steps (one run, `object_jitter=0`). Looking at the top camera every 300 steps: both
arms rose from the ready pose, the right arm came down over the end of the bar cover and stayed hovering there, and the left
arm stayed up near its base. The hand guard stayed back, the cover stayed on its holder and the saw did not move: no brake, no cover.

## Scripted demo

```bash
python demo.py task.py demo_out                       # full score: writes demo_*_cam.mp4, setup.png, result.json
python demo.py task.py abl_out --mode no_brake        # the other modes: no_scabbard, half_scabbard, touch_chain, settle
TASK_VARIANT=B python demo.py task.py demo_b_out      # the mirror-image layout
```

The plan is open loop: fixed waypoints from `geometry.py` (the layout is fixed), with no reading of the simulator's object
poses.

1. **Brake.** The left arm grips the hand guard's grip bar from above (the jaws straddle its 2.6 cm thickness) and swings
   the guard forward along its arc, past the detent, a little beyond the brake stop. The guard snaps onto the stop. The arm
   lets go and goes home.
2. **Cover.** The right arm comes down beside the cover (its jaws straddle the 5.2 cm width, 10 cm from the middle toward the closed end),
   grips it, and slides it 29 cm along the bar toward the housing in 6 s, until it stops 4 mm short of the sprocket cover. It lets go and goes home.
3. **Hold.** Both arms are clear for 2 s.

The demo scores 1.0 in 581 steps (29 s at 20 Hz): the brake is on after about 150 steps, the cover is fully on at about
step 480, and the 2 s hold ends the trial. The IK follows the plan to within 0.1 mm. `check_plan.py` replays every mode against
the IK (largest tool-point error 0.1 mm, smallest joint margin 0.107 rad). `check_collisions.py` puts both arms' collision hulls
in world coordinates (table top at z = 0.7475 m) against the table, the saw's pieces (as bounding boxes), the holder, the hand guard
and the cover, at every third control step: the smallest clearance in the demo is 10.0 mm (the right arm's pads against the bar-and-chain box at the end of the slide;
the left arm is at least 18.2 mm from the housing, and the arms are always more than 50 mm apart), 21.0 mm in `half_scabbard`;
the required minimum is 4 mm. The pads touching the guard's grip bar and the cover's walls are the grip itself and are left out
while the jaws close on them (the jaws stop at the part's width). The touch_chain ablation touches the chain on purpose and is not audited. The layout-B demo (mirror image) also scores 1.0 in 581 steps.

## Scoring

Judged from the video and, at the end, a photo and a tape measure.

| Score | When |
|---|---|
| 1.0 (success) | The chain brake is engaged (the hand guard swung 26 degrees or more toward the bar tip, past the detent), the cover covers at least 95 percent of the bar's 25 cm visible length, sitting on the bar's line, the saw is not moved more than 3 cm and not tipped more than 10 degrees, no gripper touched the bar or chain, and both arms are clear. Held 2 s, which ends the trial |
| 0.75 | The brake is engaged and the cover is on, but the 2 s have not passed or an arm is still in the way |
| 0.5 | The cover is on (at least 95 percent of the bar), the brake off |
| 0.25 | The brake is engaged and the cover is not fully on (off the bar, or on it less than 95 percent); or the cover is on the bar's line and at least half on, brake off |
| 0 | Anything else: nothing done, or a cover lying off the bar or less than half on, brake off |

Latched violations cap the score at 0.25 (they stay on for the rest of the trial, even if the saw is put back): the saw is
moved more than 3 cm from where it started or tipped more than 10 degrees (dropped, or knocked off its place), or a
gripper touches the bar or chain (its tool point inside the 2.8 cm margin round the bar, at the bar's height) while it
is not holding the cover. Without a cover on the bar that touch is a laceration hazard; the demo's cover grip is beside
the cover's walls, never inside the zone of the chain.

- **End-state:** the score is the state at the current step, never a best score. Releasing the brake (the guard brought
  back to rest) or pulling the cover off lowers it again.
- **In the real world** the checks are a tape measure and a photo: the guard's angle (a protractor or the printed ON mark), the cover's
  edge against the bar's tip and the sprocket cover, the saw's position against a pencil outline, and a look at the hands.
  The detent can be felt: it clicks over centre.

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left. Heights are above the
table top. Orientations: the saw lies on its flat motor side, bar pointing to the robot's right (-y).

- **Chainsaw** (a generic cordless 10 in saw; no real brand or model; its battery pack is printed with an invented name),
  2.8 kg with its battery, lying on rubber feet (friction about 0.9). About 53 cm overall from the bar tip to the battery.
  - Housing: orange moulded plastic, 14 x 20 x 8.5 cm (x 16 to 30, y 2 to 22), bar axis at (20, 2). Its top face has a printed warning:
    "Before carrying or transporting this saw: 1 Engage the chain brake (push the hand guard forward until it clicks) 2 Fit the
    bar cover. Never carry it with the chain uncovered."
  - Battery pack: black, 11.5 x 6.2 x 7.5 cm behind the housing (y 22 to 28.2), printed 18V 4.0Ah.
  - Sprocket cover: grey plastic, 9.5 x 8 x 1.5 cm on the housing's bar side (y 2 to 10, top at 10 cm).
  - Guide bar: steel, 25 cm visible (y 2 to -23) and 2.6 cm wide, 6 mm thick, lying 10.4 cm above the table (horizontal), with a 7.5 mm
    thick chain round its rim: outline 3.2 cm wide, with 3/8 in low-profile teeth (the chain's dark drive links and the cutters'
    tops are drawn along both edges). Printed along the bar: "SAW CHAIN 3/8 LP | GUIDE BAR 10 IN / 25 CM".
  - Front handle (black, round, 2.9 cm, at x 32.8) and rear handle (black, 4 x 4.7 cm) over the housing.
  - **Hand guard** (the chain brake lever): red plastic, 7 cm long, 1.2 cm thick and 8.5 cm tall, with a 2.6 cm thick grip bar at its free end, on a vertical hinge
    at (30.8, 7.2), resting back (0 degrees, brake off). It swings 35 degrees toward the bar tip. At about 17 degrees it
    snaps over a detent onto the brake stop (33 degrees). The brake counts as engaged from 26 degrees. A "CHAIN BRAKE ON / OFF" label with an arrow
    toward the bar tip is printed beside it.
- **Bar cover (scabbard):** yellow polypropylene channel, 26.5 x 5.2 x 2.55 cm outside (3 mm walls; inside 4.6 x 1.95 cm; closed at
  one end), 0.14 kg, friction about 0.5, printed "BAR COVER 10 IN / 25 CM, Fit before carrying or storing. Keep the chain covered."
  with hazard stripes at the closed end. It fits over the bar with 7 mm clearance each side and 6 mm above and below the
  chain. It lies on a holder block (grey, 4 x 10 x 9.1 cm, narrower than the cover so the grippers' pads clear it), centre at (20, -41.25), its
  open end facing the bar, 5 cm beyond the bar tip.
- **Arms** start at the ready pose with open grippers.
- **Reset:** guard back to 0 (brake off), cover on the holder, the saw flat on its side. The saw is only touched by the
  guard and the cover; it needs no resetting if it was not moved.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control. The saw is one floating articulation (2.8 kg: saw 2.74 kg and guard 0.06 kg) with colliders for the
  housing, battery, sprocket cover, bar plate, chain (two side rails and the nose) and handles. It lies free on the table, friction 0.9
  and 0.8, so it slides if pushed. The scabbard is a free rigid body (five box colliders: top, bottom, two walls, closed end).
- **The hand guard** is a real revolute hinge (-35.5 to 6.9 degrees), spring-driven toward its current target.
- **The brake and its detent are task code.** After every control step the monitor reads the guard's angle: past 17.2 degrees forward it
  drives the guard toward the brake stop (33 degrees), so it snaps over centre and stays; brought back past 6.9 degrees it drives it
  back to rest. The brake is engaged at 25.8 degrees or more. Nothing is cut or stopped: no engine, trigger or chain motion is simulated.
- **The cover's fit is physics** (the channel slides on the bar with real clearance); **its coverage check is task code**: the share of the
  bar's visible length inside the cover, from the cover's pose, and that it sits on the bar's line (within 1.2 cm sideways, 1 cm vertically, tilted
  under 8 degrees, turned under 6 degrees from pointing along it).
- **The chain** is a fixed part of the saw (no motion). Bar and chain are sized at their real outline.
- **Arms clear** means every gripper tool point is above 14 cm or outside the 30 x 90 cm work area round the saw and the cover.
- **Latched violations** are checked after every control step.
- **Scale:** the saw has a 25 cm (10 in) bar, not the 40 cm bar of a larger saw. A larger saw's cover would reach the arms' limit: the arms
  reach about 50 cm and the cover must start beyond the bar tip. The 10 in cordless saw is a real, common product (2.3 to 3 kg).

## Ablations

Measured with the scripted demo's modes (a run per mode), Isaac Sim, `object_jitter=0`:

| Mode | What the robot does | Predicted | Measured (steps) |
|---|---|---|---|
| `demo` | Brake, then cover, then hold | 1.0 | **1.0** (581) |
| `no_brake` | The cover only, the key safety step skipped | 0.5 | **0.5** (462, no violation) |
| `no_scabbard` | The brake only | 0.25 | **0.25** (378) |
| `half_scabbard` | The brake, then the cover pushed only 44 percent of the way (the grip moves 44 percent of its 29.6 cm slide, so about 8 cm, 32 percent, of the bar is covered) | 0.25 | **0.25** (633) |
| `touch_chain` | The right gripper is first lowered onto the bar and chain at 5 cm from the tip with the brake off, then the full demo | 0.25 (latched) | **0.25** (854, violation "hand on the chain") |
| `settle` | No motion for 10 s | 0 | **0.0** (280) |
| Dream-1 | Policy run, 2,400 steps | (expected 0) | **0.0** (2400) |
| `demo`, layout B | The mirror image, right arm on the guard, left on the cover | 1.0 | **1.0** (581) |
| `no_brake`, layout B | | 0.5 | **0.5** (462) |

Releasing the brake or pulling the cover off after the demo's end lowers the score again (unit-tested in `tests/test_logic.py`, which
holds the scorer to the same rules in both layouts).

## Reset between trials

Push the hand guard back to rest (brake off), put the cover on its holder with its open end toward the bar, and check that the
saw is still on its marked outline.

## Nearest board tasks

These were found by searching the board for chainsaw, saw, brake, scabbard, cover and guard, and read with `gh pr view`:
- #1674, "Add Stop a coasting flywheel with its brake lever, not by hand, before taking the nut off its hub task" (open): also
  engages a brake lever as a safety step, on a flywheel; there the lever stops a rotating hub before service and a hand is the hazard. Here the brake lever is a spring-detented guard
  that must be pushed past a click, and a second step, sliding a loose channel over a long bar, follows it.
- #586, "Set a relief block on the saw fence ahead of the blade, then butt the board to it" (closed): a saw's safety setup, but a workshop saw
  fence and a board; no brake and no cover.
- #1458, "Put three used lithium-ion camera batteries in a recycling collection box, first pushing a terminal cover onto each one" (open): fitting
  a small cover as a safety step; here the cover is a 26.5 cm channel sliding along a bar with 7 mm of clearance, with the hand-on-the-chain violation.
