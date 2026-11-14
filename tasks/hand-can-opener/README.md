# Open a can with a hand-held rotary opener: one hand keeps the handles squeezed while the other turns the key

A sealed steel food can (the common "15 oz" 300 x 407 size) stands on the table in front of the arms. An ordinary
two-handle rotary can opener of the "swing" pattern lies flat beside it, key up, handles open. The robot must open
the can the way people do at home: pick up the opener, set its head on the front of the rim with the handles open,
squeeze the handles shut (this pierces the lid and clamps the rim between the cutting wheel and the toothed feed
wheel), and keep them squeezed with one hand while the other turns the wing key, half a turn per grasp, until the
lid is cut all the way round; then let the handles open, take the opener off and put it down on the table.

What the instruction does not say: the handles spring open as soon as they are let go. Open, the wheels leave the
rim, so turning the key cuts nothing. One gripper therefore has to hold the squeeze the whole time while the other
regrasps the key about fourteen times (one turn of the key turns the rim 0.16 turn past the head).

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the can, the lid, the opener, the clamp and the cut, `success()` and `score()` |
| `openerkit.py` | Rigid parts built from boxes and cylinders, with hinged links (springs, dampers), a releasable fixed joint and a switchable revolute joint with a drive, for yam_arena task files |
| `demo.py` | The scripted full-score demo (closed loop), run in the same embodiment with the same scorer, with video recording, plus the cheap-strategy variants |
| `task.json` | The submission text |
| `LICENSE` | MIT |

`openerkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment with the YAM SDK, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the can with the can opener: clamp it on the rim, turn the key until the lid is cut all the way round, then take the opener off and put it on the table." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/hand-can-opener
```

`task_score` prints the trial's score: 1.0 on success (held for 1 s), otherwise `score()` at the end of the episode.
`--policy yam_scripted` with the same arguments runs the task with both arms holding still (`demo()` returns an
empty plan): the do-nothing check.

## Scripted demo

The demo is closed loop, so it is not a `demo()` waypoint plan. `demo.py` runs it in the same `yam_arena`
embodiment, task file, start layout (`object_jitter=0`, 20 Hz, 320 x 176 cameras) and limits as a trial; it sends 14
joint commands per step, like a policy, and scores the last step with the same `task_score` scorer, which it prints.

```bash
python demo.py task.py demo_out                                # demo.mp4 (three cameras side by side), setup.png,
                                                               # actions.jsonl, result.json
DEMO_VARIANT=no_squeeze python demo.py task.py out --no-record   # head set on the rim, handles held open, key turned
DEMO_VARIANT=let_go python demo.py task.py out --no-record       # squeezed shut, then let go before the key is turned
DEMO_VARIANT=no_turn python demo.py task.py out --no-record      # squeezed and held, key never turned
DEMO_VARIANT=backwards python demo.py task.py out --no-record    # key turned anticlockwise
DEMO_VARIANT=half python demo.py task.py out --no-record         # stop after half a turn of the can
DEMO_VARIANT=knock python demo.py task.py out --no-record        # shove the can over
DEMO_VARIANT=nothing python demo.py task.py out --no-record      # both arms stay still
```

What the demo reads while it runs: the poses of the can, the lid and the opener and the opener's two joint angles
(how far its handles are open, which way its key's wings point), all visible to the cameras, and the arms' own
commands. It never reads the clamp, the cut or any force.

1. Left arm: grips both handles from above, 10 cm along them from the head end (11.6 cm from the hinge), squeezes
   them shut to carry the opener, lifts it, turns it upright in the air (handles horizontal, key toward the robot)
   and brings the head over the front of the rim.
2. It opens the handles to about 20 degrees (the spring opens them under its fingers), lowers the head onto the rim,
   and squeezes them shut again, lowering its wrist by half the closing so that its lower finger stays under the
   lower handle. The cutting wheel pierces the lid and the wheels clamp the rim. It keeps squeezing.
3. Right arm: approaches the key end-on (gripper pointing along the key's axis), rolls its wrist to line its fingers
   up with the wings, grips, and rolls the wrist half a turn clockwise. It opens fully (the fingers then clear the
   wings), rolls back half a turn and grips again. It repeats until the lid has dropped (from the lid's height) or the
   can has turned 372 degrees (from its pose: the stripe), about 14 half-turns.
4. The left arm lets the handles open (the wheels leave the rim), lifts the opener off, squeezes again, carries it
   back, lays it flat where it started and lets go. Both arms move clear.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- The cut: the rim is tracked in 2-degree steps (180). A step is cut only while the opener is clamped and the key is
  turned clockwise seen from its end, so that the rim really passes the cutting wheel (the smaller of the can's turn
  relative to the opener and the key's turn times 0.16 counts). Clamped: the head was set on the rim (the can's axis
  within 3 mm radially and in height and 6 mm along the rim of where it is with the head seated, opener tilted less
  than 8 degrees) with the handles at least 20 degrees open, then the handles were squeezed within 3 degrees of shut.
  Opened more than 6 degrees, the clamp lets go. When every step is cut, the lid comes loose and drops onto the
  contents, about 6 mm lower.
- 0 if the can was ever tipped more than 30 degrees.
- 1.0 (success, held 1 s): lid cut all the way round; opener not clamped, lying still on the table (lowest point
  within 1.5 cm of the table, its body nowhere above 8 cm, its bounding box at least 1 cm from the can's side); can
  upright within 10 degrees and within 5 cm of its start; every gripper point (palm, pad centre, pad tips) at least
  2 cm from the opener's body and from the can.
- 0.75: lid cut all the way round, the rest not (opener still on the can or held).
- 0.5: at least half of the rim cut.
- 0.25: the opener was clamped on the rim (the lid pierced).
- Otherwise 0.

### Cheap strategies (measured in Isaac Sim with the demo variants)

| Strategy (variant) | Score | What happens |
|---|---|---|
| Full demo | 1.0 (success, 1563 steps) | lid cut through after 14 half-turns of the key (can turned 357 degrees), opener laid back on the table |
| Do nothing (`nothing`, 3,000 steps) | 0 | nothing moves |
| Turn the key with the handles held open on the rim (`no_squeeze`) | 0 | never clamped; the can does not turn, the loose opener is pulled off the rim onto the table |
| Squeeze shut, then let go before turning (`let_go`) | 0.25 | clamped once (lid pierced); the handles spring open and the opener falls off the can |
| Squeeze and hold, never turn the key (`no_turn`) | 0.25 | lid pierced; the key creeps 14 degrees by itself while the opener is handled, 1 % of the rim cut |
| Turn the key anticlockwise (`backwards`) | 0.25 | the can turns the wrong way a full turn; 9 % of the rim is cut by small forward rebounds when the gripper lets go of the key, under the 50 % step |
| Stop after half a turn of the can (`half`) | 0.5 | 52 % of the rim cut |
| Shove the can at mid-height (`knock`) | 0 | the can tips over (89 degrees) |

Turning the key without holding the squeeze (`no_squeeze`, `let_go`) and squeezing without turning (`no_turn`)
score 0.25 or less. The official CLI with `--policy yam_scripted` (empty plan) also scores 0 (it stops after 61 steps).

## Runtime and seeds

The demo reaches 1.0 in 1,563 steps; run_limit is 3,000 steps. A 3,000-step trial takes about 15 minutes on an
RTX A6000 plus 1 to 2 minutes for Isaac Sim to start. The layout is fixed (`-E object_jitter=0`); there are no seeds
to choose. The demo is deterministic apart from small physics differences between machines.

## Fidelity

All parts are rigid bodies that collide as boxes and cylinders; the opener is drawn with meshes generated in code
(black grips, steel head plates, toothed wheels, a bow-tie wing key). The can's rolled seam is a ring of 36 boxes. The opener
is a free articulation: its body (head and lower handle), the upper handle on the jaw hinge (limits 0 to 25 degrees,
an opening spring of 2.9 N m/rad whose rest angle is 15 degrees past the open stop, so it holds the handles open and
needs 2.0 N m to hold them shut), and the wing key on a hinge with 0.15 N m s/rad damping (the effort of cutting:
about 0.4 N m at the demo's speed), a little joint friction and the inertia of a gear train. The wheels are drawn but
do not collide; the head's shoe rests on the seam. Metal cutting is not simulated: when the handles are squeezed
shut on the seated head, a revolute joint about the can's axis is switched on between can and opener (the clamp);
its drive turns the can relative to the opener by 0.16 of the key's turn (feed wheel 6 mm, rim 37.5 mm in radius).
The cut is tracked as above and a fixed joint holds the lid until the whole rim is cut. The handle spring force, the
key effort and the opener's mass are estimates, not measurements. While clamped, the stiff clamp joint and the
holding arm can lift the can up to about 2 mm off the table.

Real numbers used: the 300 x 407 can is 3 in (76 mm) across the seam and 4 7/16 in (113 mm) tall; a No. 300 can
holds about 15.5 oz (USDA Food Buying Guide), so about 0.47 kg with the can (the empty can's 50 g is an estimate). Swing-pattern openers such as the
Swing-A-Way 407 are 7 to 8.5 in long and listed at 0.6 to 0.7 lb shipping weight; 0.18 kg is used for the opener alone.

## Real-world build

A sealed 300 x 407 food can with a paper label marked with a vertical stripe, and a Swing-A-Way 407 (or any
swing-pattern opener with a light handle spring) lying flat beside it. Mark both positions on the table. Judge the cut
from the video (the can turns once under the head) and the loose lid at the end: cut all the way round means the lid
is free and can be lifted out; for a partial cut, measure the length of opened seam round the lid's edge with a tape
as a share of the full way round, or read the can's turn under the head from the stripe on its label. Check the
opener's and the can's final positions with a ruler.

## Third-party assets and licenses

The SDK is yam-isaac-lab-arena at 0059d02a1a91, the program's official dependency, installed by the user's
environment. Nothing from it is redistributed here. Licenses are given as each source states them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (https://github.com/i2rt-robotics/i2rt, SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, Copyright (c) I2RT Robotics (`LICENSE.i2rt`) | no |
| link2/link3 meshes split into black and white parts | station arm colours | SDK `yam_arena/assets/yam/dream/`, from the dreamlabs 1.0.23 wheel | "MIT, see ../LICENSE.i2rt" (SDK `dream/SOURCE`) | no |
| Station floor and table | room | SDK: boxes with flat colours generated in code; no mesh or texture file loaded | SDK README: "This repository has no license yet: most of it is ported from YAM-inspect-Isaac by Thomas Pierson, which has none." | no |
| Top-camera stand | room | SDK: shapes generated in code | as the row above | no |
| Lights | lighting | SDK scene code (Isaac Lab-Arena dome light, Isaac Lab sphere light) | as the row above | no |
| top_cam, left_cam, right_cam | policy RGB | SDK camera sensors at the calibrated poses | as the row above | no |
| Station fruit, bowl, plate and bottle (SDK object library) | not used: this task's own objects replace them | - | - | no |
| Can, lid, can opener; `task.py`, `openerkit.py`, `demo.py` | the task | this folder: built in code from boxes, cylinders and generated meshes; no third-party meshes or textures | the authors' own, MIT License in `LICENSE` | yes |

Isaac Lab-Arena release/0.3.1 (c7edf3a9d5d8276565208f72422f8f8a0806d6cf) ships the Apache License 2.0 in its
LICENSE.md. Isaac Sim and Isaac Lab are runtime dependencies under their own licenses. None of them is redistributed
here.
