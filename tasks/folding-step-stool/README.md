# Unfold a steel folding step stool the way its maker shows

A steel folding step stool, the **Vive Folding Step Stool, product number LVA2068BLK**, lies upside down on the table
with its two legs folded flat against the platform's underside. The instruction asks for both legs to be opened fully,
each until its own lock clicks, and the stool let go of. This is how Vive's own setup photos open it ("Step 1" to "Step
3": the stool upside down, each leg swung out until it locks). Each leg has its own spring lock, which catches only once
the leg is a little past square to the platform; short of that, the lock's spring pushes the leg back, so a leg can look
open and still be unlocked. With both hands off, each leg is pushed back towards folded, the way it is loaded when the
stool is stood on: a locked leg holds; an unlocked one folds.

## Changes in this revision
For the review of TK-TN4M9Z (2026-11-21: "The make and model of the stepstool should be specified"):
- **The stool is now a product on sale, not our own build:** the Vive Folding Step Stool, LVA2068BLK (UPC
  810041984739). The same design is sold as VonHaus 3500099. Our printed design, its STL files and its parts list are
  gone.
- **The sim has the Vive's listed size and mass:**
  - platform 15 x 10.5 in, 6.5 in off the floor;
  - 4 lb (1.8 kg);
  - steel U-frame legs with a crossbar at the foot.
- **The task follows Vive's own setup steps:** the stool starts upside down, and each leg is swung out until it locks.
- **Leaving out the last step:** Vive's last step, turning the stool over onto its feet, is left out. It needs a two-arm
  flip of a 1.8 kg stool, beyond these arms. Standing the stool on its feet under a 40 N load used to be the check that
  a leg is really locked. It's replaced by its equivalent on each leg: with the hands off, each leg is pushed back
  towards folded.
- **Unchanged:** the latch model (catches at 98 degrees, pushes back from 80 degrees on), the scoring tiers, and the
  rule that a leg that only looks open earns nothing.

## The stool
| Size | Value used | Source |
|---|---|---|
| product | Vive Folding Step Stool, LVA2068BLK, coated steel frame, textured non-slip mat, 250 lb capacity | Vive's product page (vivehealth.com/products/folding-step-stool, read 2026-11-21) |
| platform | 38.1 x 26.7 cm (15 x 10.5 in), 2.5 cm thick | 15 x 10.5 in: the page's dimensioned photo (its text says 15 x 10 in); thickness **estimate** |
| height | the platform's top 16.5 cm (6.5 in) off the floor, open | the page's FAQ ("6.5 in off the ground"; the photo: 6 in) |
| mass | 1.8 kg (4 lb): platform 1.2 kg, legs 0.3 kg each | 4 lb: the page; the split is an **estimate** |
| legs | steel U-frames, two square posts (1.9 cm) and a round crossbar (1.8 cm) at the foot, angled outwards ("wide angled base"), each with a lock released by "pushing in the locking latch on the inside of each leg" | the page and its photos; tube sizes **estimate** |
| hinge | 2 cm in from the platform's end, 0.8 cm under it | **estimate** |
| lock | catches at 98 degrees from folded; the leg's stop at 100 (10 degrees splayed); pushes the leg back 0.10 to 0.165 N m from 80 degrees on | **estimate**: to be measured on a stool |

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder (`fittingkit.py` beside the task file):

```bash
INSTR="The step stool lies upside down with its legs folded. Open each of its two legs fully, swinging it up and out until its lock clicks, then let go of the stool."
COMMON=(--embodiment yam_arena -E task=folding_step_stool.py -E object_jitter=0 -E control_hz=20 -E cam_width=320
        -E cam_height=176 --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames)

# Dream-1
inspect-robots run "${COMMON[@]}" --policy dream --log-dir ~/yam-eval/folding-step-stool-dream

# The scripted demo, which reaches full score
inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/folding-step-stool-demo

# A wrong way: each leg swung out only to 92 degrees, where it looks open but isn't locked
STOOL_SWING_PAST=-8 inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/folding-step-stool-short

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/folding-step-stool-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. With `STOOL_LOG=<file>` set, each lock catching and each check is logged;
with `STOOL_TRACE=<file>`, every 0.1 s the platform's pose, the legs' angles and both TCPs.

## Setup
Positions in cm on the table, from the midpoint between the arm bases: x forward, y to the robot's left.

| Object | Description | Position |
|---|---|---|
| platform | black steel, 38.1 x 26.7 x 2.5 cm, grey textured mat, 1.2 kg | upside down, mat face down, centred at (30, 0), long side along y |
| legs | two steel U-frames, one at each end: posts 1.9 cm square, 23.7 cm apart, a round grey crossbar 1.8 cm across at the foot, 12.5 cm from the hinge; red latch buttons on the posts' inside; 0.3 kg each | folded flat against the platform's underside, the crossbars 4.6 cm either side of the middle |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`.

## Scoring
From the latch model and the stool's pose.

| Score | When, at the end |
|---|---|
| 1.0 | both legs locked and holding under the check (below), held 1 s, which ends the trial (0.9 if time runs out first) |
| 0.5 | both legs locked, a hand still on the stool |
| 0.25 | one leg locked |
| 0 | anything else: a leg that only looks open earns nothing |

- **Locking:** a leg locks once it is opened 98 degrees from folded, and stays locked.
- **Short of the lock:** from 80 degrees on, the lock's spring pushes the leg back (0.10 N m at 80 degrees to 0.165 N m
  just short of 98), so a leg let go short of its lock falls back.
- **The check:** whenever both grippers are more than 2 cm from the stool and it lies still and level, upside down on
  the table, each leg is pushed back towards folded with 0.6 N m (about 5 N on its crossbar). A locked leg holds (the
  model holds it at its stop with 2 N m); an unlocked one folds.

**Scoring a real trial:** with the hands off, push each leg's crossbar towards folded with 5 N (a spring gauge, or a
500 g weight hung from it by a string over the platform's end). A locked leg doesn't move; an unlocked one folds.

## The latch model
`Locks` in `folding_step_stool.py` runs before every physics substep (240 Hz):
- **Angle:** it reads each leg's angle on its hinge from the legs' and the platform's poses. The legs are rigid bodies on
  PhysX revolute joints (`fittingkit.Hinge`, limited to 0 to 100 degrees).
- **Push-back:** short of the lock, it applies the lock spring's push-back.
- **Lock:** at 98 degrees it notes the lock, and from then on holds the leg open against its stop.
- **Check:** every 0.1 s it checks the grippers' collision hulls against the stool's parts, and applies the check
  while the hands are off and the stool lies still.

The lock is a rule, not simulated contacts, because the real latch is a small spring pin inside the leg's hinge.

## The scripted demo
It plays a fixed plan from the fixed layout with the arm's own inverse kinematics, and reads nothing from the simulator.
One leg at a time, each by the hand on its side:
1. The hand, pointing down, pinches the leg's crossbar where it lies folded. The fingers are aimed 2.5 mm into it, a
   medium pinch, with the pads across its middle.
2. It swings the leg up and out round its hinge, turning with the leg to 60 degrees and then holding that turn. It goes
   past square to 106 degrees, beyond the stop, so the lock clicks.
3. It lets go and moves off. Then the other hand does the other leg.

Earlier versions failed, and each failure changed the plan:
- Closed fully, the round crossbar couldn't turn in the fingers and slipped out at 13 degrees.
- Pinched lightly with the pads high on it, the leg didn't lift.
- Pointing down throughout, one hand lost its leg at 83 degrees.
- A hand turning all the way with the leg can't reach past 84 degrees (offline IK).

## Runtime and seeds
- **Official runs (2026-11-21, this version):**
  - the scripted demo: 1.0, both legs locked at 98 degrees and holding under the check;
  - the wrong way (`STOOL_SWING_PAST=-8`, each leg stopped at 92 degrees): 0, both legs folded under the check;
  - Dream-1: 0.0 over the full 1,200 steps.
- **Offline checks:** `python -m pytest -q test_folding_step_stool.py` (11 tests), which covers the angle, the locks,
  the check, the tiers, the Vive's sizes and the demo's reach.
- **Seeds:** one fixed layout (`-E object_jitter=0`).
