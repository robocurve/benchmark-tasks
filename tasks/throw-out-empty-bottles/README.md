# Throw out the empty bottles, leave the full ones: they look the same

Four identical opaque white bottles, caps on, stand in a row. Two of them, drawn afresh before every trial, are full
of water; the other two are empty. They look the same; only lifting them tells. The robot must put the empty bottles
into the recycling bin and leave the full ones standing on the table.

**Instruction:** `Put the empty bottles into the recycling bin. Leave the full ones on the table.`

## Why this task

Clearing bottles for recycling means telling empty from full, and opaque bottles with their caps on give no visual
cue. A robot that sorts by appearance throws out full bottles or leaves empty ones. The distinction here is physical,
not visual: the policy must lift each bottle, notice its weight, and act on it, setting the full ones back down
(nothing more to do with them) and binning the empty ones. The instruction is partly a negation ("leave the full
ones"). Which bottles are full changes every trial, so the choice cannot be memorised and cannot be read from the
cameras.

## Design

| Object | Size | Mass | Notes |
|---|---|---|---|
| Bottles (4) | 0.5 l, 7 cm across, 15 cm tall, caps on, at x 32 cm, y 15, 5, -5 and -15 cm (spots 1 to 4) | full 0.55 kg, empty 40 g | Opaque white HDPE, identical. Two full of water, drawn afresh before every trial |
| Recycling bin | 24 x 16 cm inside, 4 cm walls, centred at x 18, y 0 cm, long side along y | | Green plastic, in a taped outline; holds all four bottles |

Positions are from the midpoint between the two arm bases (x forward, y left, z up).

## Scoring

Two whole-trial conditions, then the end state (no early end, so every bottle can be tried).

| Condition | Score |
|---|---|
| Any bottle, at any moment of the trial, tilted more than 30 degrees from upright or fell off the table (even if set back up later) | 0 |
| A full bottle in the bin at the end | 0 |
| Both empty bottles resting in the bin at the end | 1.0 |
| One empty bottle resting in the bin at the end | 0.5 |
| Otherwise | 0 |

"Resting in the bin": centre inside the walls, base on the bin floor, upright (30 degrees or less), still, and let go
(neither gripper touching it). "In the bin" for a full bottle: its centre inside the walls, below the top of the walls.

| Check | Real trial | Simulation (`score()`, after every control step) |
|---|---|---|
| Tipped over | Video: a bottle leans past 30 degrees at any moment | Any bottle's axis more than 30 degrees from vertical at any step; latched until the next reset |
| Fell off | Video: a bottle leaves the table top | Any bottle's base more than 5 cm below the table top at any step; latched |
| Base on the bin floor | Photo: no gap under the base | Lowest point of the base within 3 mm of the floor's top |
| Still | Video: not moving at the end | Under 1 cm/s |
| Let go | Photo/video: no finger on it | Both grippers' TCPs (the point between the fingertips) at least 9 cm from the bottle's axis, so it is not between the fingers (open pads 47 mm either side of the TCP; bottle radius 3.5 cm) |

Unintended strategies and what stops them:
- Throwing every bottle in the bin: a full bottle in the bin scores 0.
- Guessing by appearance or position: the full bottles are drawn at random and look identical to the empty ones.
- Knocking bottles over while testing them, or tipping one and standing it back up: 0 (whole-trial condition).
- Holding an empty bottle in the bin, or dropping it from above at the end: no credit (it must rest on the floor,
  still and let go).
- Doing nothing: 0.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the empty bottles into the recycling bin. Leave the full ones on the table." \
  --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/throw-out-empty-bottles-demo
```

Every bottle is gripped 10 cm up (above its centre of mass, so it hangs) and lifted 4 cm to feel its weight. A full
bottle is set straight back down where it stood; an empty one goes into the bin. The left arm tests its two bottles,
then moves away; only then does the right arm test its two, so the hands never share the bin. Between bottles the
open fingers pass 1 cm over the bottle tops. The plan follows the trial's draw (`FULL=0,2` fixes it: 0 is the
leftmost bottle), as an operator would after lifting each bottle. inspect-robots resets the policy (where the demo
makes its plan) before the embodiment, so each draw is made one reset ahead: every reset applies the pending draw and
draws the next trial's, and the demo plans for the pending one.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: bottles, bin, the draw and masses applied at every reset, `score()`, and `demo()` |
| `jointkit.py` | Rigid parts made of boxes (the 16-sided bottles), and per-step and reset hooks (hooks yam_arena without modifying it) |
| `test_throw_out_empty_bottles.py` | The task file loads, the bottles look identical, the bin holds four, the demo tests every bottle and is reachable for every draw, every reset draws afresh (`FULL` fixes it), tipping or a fall zeroes the trial even if undone, only bottles resting on the bin floor and let go count |
| `task.json` | Submission details |

## Runtime and seeds

A demo run takes about 5 minutes of wall time on an RTX workstation. Run limit: 1,200 steps (1 minute at 20 Hz).
Pass `-E object_jitter=0`. With `FULL` unset, every reset draws two bottles afresh from the operating system's random
source and sets the four bottles' masses (0.55 kg full, 0.04 kg empty, inertia scaled with the mass) through Isaac
Lab's rigid-body view (`set_masses_index`, `set_inertias_index`); the log prints each trial's draw and the masses read
back from the physics view. Set `FULL=0,2` to fix the draw for every trial.

## Tests

```bash
python -m pytest -q      # in this folder, in the yam_arena environment; no GPU needed
```

## Verified in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1), with the command above.

| Run | `task_score` | What the log and video show |
|---|---|---|
| Scripted demo, `FULL` unset (random draw: bottles 1 and 3 full), task.py sha256 e94eafc511b25250 | **1.0** | The left arm lifts bottle 1 (full, set back), bottle 2 (empty, into the bin), and moves away; the right arm lifts bottle 4 (empty, into the bin) and bottle 3 (full, set back). Masses read back from the physics engine: 0.55 / 0.04 / 0.55 / 0.04 kg. No bottle tilts past 30 degrees at any step. 838 steps |
| Scripted demo, bottles 2 and 4 full (`FULL=1,3`), same task.py | **1.0** | The left arm bins bottle 1 and sets bottle 2 back; the right arm sets bottle 4 back and bins bottle 3. 838 steps |
| Five resets with `FULL` unset (5 steps each), earlier task.py with the same draw code | | Draws per trial: bottles 3+4, 1+2, 1+3, 1+3, 3+4; the masses read back from the physics engine match each draw (0.55 / 0.04 kg) |
| Dream-1, `FULL` unset (drawn: bottles 1 and 4 full), same task.py | **0** | 1,200 steps (step limit). Both arms move about near the row and their bases, never grip a bottle; all four stay standing in place |
| `python -m pytest -q` | 12 passed | |

## Real-world build

- **Bottles:** four identical opaque white 0.5 l HDPE bottles with screw caps (7 cm across, 15 cm tall); fill two with
  water (0.55 kg each; check on a kitchen scale).
- **Draw for every trial:** four slips numbered 1 to 4 in a cup (spot 1 = y 15 cm, spot 4 = y -15 cm). Before each
  trial, out of the cameras' view, draw two slips: the full bottles stand at those spots, the empty ones at the other
  two. Put the slips back.
- **Bin:** a low green plastic recycling bin about 24 x 16 cm inside with 4 cm walls, in a taped outline.
- **Safety:** light, unbreakable objects.

## Still approximate or unverified

- Rigid bottles with a fixed mass per trial: no sloshing water.
- The bin is fixed in simulation.
- One Dream-1 run (1,200 steps, previous version of this task) scored 0: neither arm reached a bottle.
