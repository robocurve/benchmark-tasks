# Haul a heavy dock line across the gap with its messenger and drop the eye over the bollard

Mooring with a messenger line, on the table. The left half of the table is the boat's deck and the right half the
dock; a 6 cm blue tape strip along the middle (y -3 to +3 cm) is the water gap between them. A heavy white 12 mm
dock line (50 cm) is tied to a fixed white cleat block on the boat, far out from both arms, and lies straight towards
the far left corner. It ends in a red rigid square eye (60 x 60 x 12 mm, 36 mm opening). A thin yellow 6 mm
messenger (65 cm) is tied to the eye's left side bar and lies straight back along the deck towards the left arm.
A black bollard (20 mm round, 50 mm tall) stands on the dock. The robot must haul the eye across the gap with the
messenger, drop the eye over the bollard, and let go of the eye and the dock line.

Why two arms (reach-forced relay): the eye and the whole dock line lie at least 61 cm from both arm bases, beyond
either arm's reach (with the gripper pointing down an arm reaches about 50 cm). Only the messenger's free end lies
within reach, and only of the left (boat) arm. The bollard stands at y -15 cm, where the left arm cannot reach (it
reaches to about y -2 cm); the right (dock) arm cannot reach the boat side (y > +2 cm). So the boat arm must pass the
messenger across the gap, and the dock arm hauls the eye in and puts it on the bollard. The slack is fixed: the
cleat is 31 cm from the bollard and the eye's centre can get at most 53 cm from the cleat, so the line has about 22 cm
to spare at the bollard and is never pulled taut.

## Context

- Wikipedia, "Mooring" (en.wikipedia.org/wiki/Mooring_(watercraft)): "Heavy mooring lines are often passed from
  larger vessels to people on a mooring by smaller, weighted heaving lines." Its photo caption: "A dockworker places
  a mooring line on a bollard."
- Wikipedia, "Monkey's fist" (en.wikipedia.org/wiki/Monkey%27s_fist): "A lightweight feeder line would be tied to the
  bowline, then the weighted heaving line could be hurled between ship and dock. The other end of the lightweight line
  would be attached to a heavier-weight line, allowing it to be drawn to the target easily."
- O*NET 53-5011.00 Sailors and Marine Oilers, task: "Handle lines to moor vessels to wharfs, to tie up vessels to
  other vessels, or to rig towing lines."

Here the messenger is laid across the gap by the boat arm instead of being thrown.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the rope simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Moor the boat: drop the red eye of the heavy white dock line over the black bollard on the dock. The eye is out of reach, so haul it across the water gap with the thin yellow messenger line tied to it. Then let go of the eye and the dock line."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop (reads the live messenger and eye). The boat (left) arm pinches the messenger 3 cm from its free end,
carries it across the gap and lays the end on the water tape at x 30 cm, which already draws the eye about 12 cm in;
the left arm then moves clear. The dock (right) arm hauls hand over hand: each stroke it pinches the messenger where
it runs closest to a grab point near the gap (x 40, y -2 cm), draws it back low over the dock to x 14, y -10 cm and
lets go, until the eye has come across the gap (three strokes). It then pinches the eye's side bar nearest to it,
carries the eye over the bollard, lowers it to just above the bollard top, corrects on the live eye and lets go, so
the eye drops over the bollard; the arm moves clear. Success holds 1 s and ends the episode at step 628 (31 s);
task_score 1.0. `demo/` holds the videos, `setup.png` and `result.json`.

Shortcut and partial variants (test only, `HAUL_DEMO=<name> python cabledemo.py task.py out`):

| variant | what it does | score |
|---|---|---|
| `nothing` | no motion | 0 |
| `reach` | both arms stretch by hand for the eye and the dock line (no messenger), close and pull back | 0 (nothing reached) |
| `hold` | the full sequence, but the dock arm keeps holding the eye down over the bollard | 0 (not released) |
| `beside` | the full sequence, but the eye is set down 6 cm from the bollard | 0.5 (partial credit) |

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if either gripper's TCP (centre between the finger pads) is within 3.5 cm of the eye bars or the dock line.
  Real trials: 0 if the cleat or the bollard is moved.
- **1.0** if the eye is over the bollard: the bollard's axis passes through the eye's 36 mm opening (top view) and
  the eye's centre is at least 12 mm below the bollard top.
- **0.5** if the eye lies on the dock (centre past the tape's dock edge, y < -3 cm, less than 2 cm above the table)
  within 8 cm of the bollard axis.
- **0.25** if the eye lies on the dock farther from the bollard.
- **0** otherwise (the eye still on the boat side or in the gap, or held up in the air).
- `distance()`: eye centre to bollard axis (m, top view). `progress()` (analysis only): messenger taken, messenger
  across the gap, eye-to-bollard progress (0-1), eye on dock, eye near bollard, eye on bollard, released.
  `VARIATION`: seeded start offsets for the dock line (with the cleat and the messenger) and the bollard.
  `TWIN_EXTRA`: the eye loop, the messenger knot and the cleat tie for the printable kit.

## How this differs from similar tasks

Other tasks bring an out-of-reach item closer by pulling a rigid support or by reaching out with a tool. Here nothing
rigid is pulled: a heavy rope is hauled through a second, thinner rope tied to its end, and the job is a relay between
the arms. Only one arm can reach the messenger and only the other the bollard, so the messenger must be passed across
the gap and hauled in hand over hand (re-taking it several times) before the eye, a loop, is dropped over a post.

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). One trial takes about 5-10 min wall time on one GPU including the 1-2 min Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the ropes are never jittered (they always start straight), so every trial
  starts identically.
