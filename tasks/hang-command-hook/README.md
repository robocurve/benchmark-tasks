# Stick an adhesive wall hook on its marked spot, level, then press it firmly for 30 seconds

A Command adhesive hook stands in a slot in front of a piece of painted drywall with a cross marked on it. Its foam
strip is ready, liner off. The robot has to put the hook on the wall centred on the cross and level, then press it
firmly for 30 seconds and let go. The strip grabs as soon as it lies against the wall, so there is one chance: the hook
has to arrive square to the wall, already on the mark and level, because it can't be slid or turned afterwards. Then
the press has to be firm and long enough, which is what makes the bond hold its rated load. Nearly everyone has hung
one of these.

## The real step
3M's instructions for its Command hooks: press the hook "firmly for 30 seconds", then "Wait 1 hour for adhesive to
build" before hanging anything ([Command large hook sheet][sheet]; the medium hook's own listing gives the same hour,
[17001-6][uk]). 3M's guide adds that the harder you press, the stronger the bond ([how-to][howto]). Done by: almost
anyone, renters especially, and cleaners and office staff putting up hooks for others. What goes wrong: a hook stuck
off its mark or crooked can't be moved without pulling the strip off and using a new one, and a hook pressed too lightly
or too briefly lets go under its load. A photo of the wall shows where the hook is against the cross.

## The hook and the wall: sizes and sources
| Part | Value used | Source |
|---|---|---|
| hook | 3M Command Medium Utility Hook (17001), white plastic, 76 mm tall, 23 mm wide, 25 mm deep; holds 3 lb (1.36 kg) | 3M US listing ([17001][us]); 3M UK 17001-6 ([uk]) |
| hook's shape | a 4.5 mm back plate; an 8 x 5 mm arm 6 mm above its bottom edge, reaching out 19 mm, with a 12 mm upturned lip | **estimate** from product photos |
| strip | Command medium strip, foam, 64 x 19 x 1.6 mm, centred on the plate | **estimate** |
| press | "firmly for 30 seconds" | 3M sheet ([sheet]) |
| "firmly" | 10 N or more | **estimate**: no published figure; a firm thumb press is 10 to 30 N |
| wall | 1/2 in (12.7 mm) drywall, 30 x 25 cm, painted light blue, on a wooden foot | ordinary drywall offcut |
| cross | black marker, 40 mm strokes, 3 mm wide | drawn for the task |

[sheet]: https://multimedia.3m.com/mws/media/1487622O/17036mb-es-command-large-matte-black-double-hook-instructions.pdf&fn=17036MB-ES_34872390793_R1.pdf
[uk]: https://3m.co.uk/3M/en_GB/p/d/v000532605
[us]: https://3m.com/3M/en_US/p/d/v000277433
[howto]: https://command.3m.co.uk/3M/en_GB/command-gb/how-to/

## How the sim models it
- **The wall** is a fixed mechanism: a wooden foot, and the drywall panel on a very stiff spring (40,000 N/m) along its
  normal, so the panel's travel measures how hard it is being pushed, as a force gauge behind the wall would. A firm
  10 N press moves it 0.25 mm.
- **The hook** is one free body of four boxes (strip, plate, arm, lip). The strip's foam is tacky: friction 2.0.
- **The bond.** When three of the strip's four corners lie within 0.6 mm of the wall's face and the wall is pushed
  with at least 1 N, the strip bonds: a joint between the hook and the panel, set up with the hook unjoined, is
  switched on at their poses of that moment. From then on the hook can't slide or turn on the wall.
- **The press goes through the hook.** The panel reports its contacts to the task each step: the force on it in all,
  and the part that comes from the hook. A second of pressing counts only while the gauge reads at least 10 N and
  everything else together (fingers, wrist, the other arm) pushes on the panel with less than 0.5 N, so a hand
  pressing the wall beside a lightly tacked hook, or a finger resting on the wall while the hook is pressed, doesn't
  count. If the contacts can't be read, no second counts.

What we simplified:
- The bond is all or nothing: once made it holds whatever load, where a real strip that was pressed too little gives
  way under its rated weight. So the firm 30 s is scored directly, as 3M's instructions state it, rather than through a
  load test in the sim. A real trial checks the load (below).
- The strip bonds only when it lies flat; a real strip can grab at one edge first and be pressed down.
- The hook weighs 30 g, not about 8 g, because a very light body squeezed between finger pads makes the solver give.
- The liner is already off, and the hook stands in a slot to start with, instead of in a hand.
- The wall gives 0.25 mm under a firm press. All shapes are boxes.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left, heights above the table.

| Object | Description | Position |
|---|---|---|
| wall (fixed) | light blue painted 1/2 in drywall, 30 x 25 cm, standing on a 9 x 30 x 2 cm wooden foot with a brace behind, its face towards the robot | face at x 32, from y 0 to 30, from 2 to 27 cm up |
| cross | black marker cross on the wall, 40 mm strokes 3 mm wide | centre at (32, 12), 13 cm up |
| holder (fixed) | two tan blocks 12 x 40 x 5 mm with a 5.5 mm slot between them | slot at (20, 12) |
| hook (loose) | 3M Command Medium Utility Hook, standing in the slot on its plate's bottom edge, strip towards the wall, hook towards the robot | its strip's middle at (20.4, 12), 3.8 cm up |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`. In a real trial the wall is
clamped to the table and the hook stands in its slot with a fresh strip on it, liner just pulled off.

## Scoring
**Stuck:** the strip bonded to the wall, as above. **Pressed:** seconds with the gauge at 10 N or more and nothing but
the hook touching the wall, as above. The hook's place is its plate's middle (the strip's middle) on the
wall against the cross's centre; its tilt is the plate's turn from upright in the wall's plane. **Hands clear:** no
point along either gripper's finger pads, or of its fingertips, within 1 cm of the hook.

| Score | When |
|---|---|
| 1.0 | stuck within 5 mm of the cross and 3 degrees of level, pressed through the hook with at least 10 N for 30 s in all, both hands clear (this ends the trial) |
| 0.75 | stuck on the mark and level, but pressed firmly for less than 30 s, or a hand still on it |
| 0.5 | stuck within 15 mm of the cross and 8 degrees of level |
| 0.25 | stuck anywhere else on the wall |
| 0 | not stuck to the wall |

In a real trial:
- **Place:** a ruler from the cross's visible stroke ends to the plate's edges (the horizontal stroke shows 8.5 mm
  either side of a centred hook), and a small level or a phone level app on the plate's edge.
- **Press (a force gauge is required, not timing alone):** the panel stands with its back against a push-pull force
  gauge (or on a digital kitchen scale under its foot, read in newtons), logging at 10 Hz or more, and a 60 fps video
  is taken of the press, started together. A second counts only when the gauge reads at least 10 N (1.02 kgf) and the
  video shows nothing but the hook touching the wall; add up those seconds.
- **Load (the real check of the press):** after 1 hour, hang the rated 1.36 kg on the hook for 5 minutes; a full-credit
  hook holds it.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with a
Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hang_command_hook.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stick the hook on the wall, centred on the black cross and level. The strip grabs as soon as it touches, so line it up first. Then press the hook firmly against the wall for 30 seconds and let go." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/hang-command-hook-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hang_command_hook.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stick the hook on the wall, centred on the black cross and level. The strip grabs as soon as it touches, so line it up first. Then press the hook firmly against the wall for 30 seconds and let go." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/hang-command-hook-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/hang-command-hook-dream/*.json; do inspect-robots video "$log"; done
```

The task file needs nothing beyond the official simulator: it builds the wall (an articulation), the hook (a body of
several shapes) and the bond itself, by hooking `yam_arena` when it is imported. Environment variables, all optional:
- `HOOK_DEBUG=<file>`: one line per control step: whether the hook is stuck, the press force, the force on the panel
  from anything but the hook, the firm seconds so far, the strip's four corner gaps to the wall, the hook's offset from
  the cross and its tilt, and whether the hands are clear.
- `HOOK_DEMO=brief`: a wrong way for checking the score: pressed for 5 s instead of 30.
- `HOOK_DEMO=off`: a wrong way: stuck 12 mm to the right of the cross.
- `HOOK_DEMO=beside`: a wrong way: tacked on the mark with a light touch and let go, then the bare wall pressed with the
  closed fingertips 45 mm beside the hook, with 15 N for 34 s.
- `HOOK_VIDEO=<folder>`: record the top camera at its full 640 x 360 render to `<folder>/top_cam.mp4`, and its first
  frame to `setup.png`.

## The scripted demo
It plays a fixed plan from the fixed layout with the arm's own inverse kinematics, and reads nothing from the
simulator.
1. The left hand, pointing at the wall and tipped 20 degrees below level, pinches the hook's plate by its two side
   edges, its fingertips 2 mm behind the strip.
2. It lifts the hook out of its slot, carries it to 1 cm in front of the cross, square to the wall and level, and moves
   it straight in until the strip touches (it bonds there).
3. It presses with about 15 N for 34 s, eases off, opens and backs away.

The arm's joints are soft position servos (several mm of give per newton at these poses), so the press is aimed past
the wall by the arm's compliance times the press force, computed from the arm's Jacobian and joint gains.

## Runtime and seeds
- Official runs of this version (2026-11-20, task file SHA-256 `40fbb2fd...750323`):
  - the scripted demo 1.0, ending on success at step 1013 (50.7 s): the strip bonded at step 233 with three corners
    within 0.24 mm of the wall under 1.06 N, its middle 0.29 mm right of and 0.50 mm below the cross, 0.6 degrees off
    level; pressed at a mean 13.7 N, 35.05 s of it at 10 N or more, with 0.00 N on the panel from anything but the hook
    throughout; after the hand let go the hook stayed where it bonded;
  - a wrong way, `HOOK_DEMO=beside`: 0.75. The strip bonded on the mark at step 235 (0.25 mm from the cross, under
    1.67 N), then the fingertips pressed the bare wall beside it: the gauge read 10 N or more for 34.85 s (mean 12.9 N),
    all of it from the fingers (10.1 to 13.0 N), so none of it counted (0 s pressed);
  - a wrong way, `HOOK_DEMO=brief` (pressed 5 s): 0.75, stuck on the mark with 6.05 s pressed;
  - a wrong way, `HOOK_DEMO=off` (aimed 12 mm right of the cross): 0.5, stuck 11.0 mm off, 35.05 s pressed.
  - The brief and off runs used the file a few minutes before this one (SHA-256 `38548c1c...149bd9`), which differs
    only in the beside probe's plan (its light push was added after its first run, which pushed too lightly to bond).
  - In every run the panel read up to 6.2 N from something other than the hook during the first 11 steps only, as it
    settled on its spring with nothing near it, and 0.00 N from then on except where the fingers pressed it.
- Dream-1 (1,800 steps, the full run limit), run on the previous version (SHA-256 `b6bd9bb6...11aa13`, which counted a
  press anywhere on the panel): 0. It picked the hook out of its slot with the right hand by step 506 and carried it
  towards the cross, but let go at step 680 with the strip still 22 mm from the wall and the hook tipped 21 degrees; the
  hook fell on its side on the table and was not picked up again. The strip never bonded, so this version, which only
  narrows what counts as pressing, scores that trial the same.
- Runtime, measured 2026-11-20 on one L40S shared with other simulations: 15 to 19 minutes for the demo trial (1,013
  steps), with the videos.
- The scoring and reach checks run without the simulator: `python -m pytest -q test_hang_command_hook.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.

## Credits
The mechanism code in the task file (shapes, links, joints, and the hooks that build and reset a mechanism) is adapted
from robocurve/benchmark-tasks pull request #223 ("Open a panel held shut by a spring plunger",
`tasks/spring-plunger-panel/mechanism.py`), published under the MIT License; its notice is kept in the task file.
