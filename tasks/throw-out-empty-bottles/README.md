# Throw out the empty bottles, leave the full ones: they look the same

Four identical opaque white bottles, caps on, stand in a row. Two of them, drawn at random each trial, are full of
water; the other two are empty. They look the same; only lifting them tells. The robot must put the empty bottles
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
| Bottles (4) | 0.5 l, 7 cm across, 15 cm tall, caps on, at x 32 cm, y 15, 5, -5 and -15 cm | full 0.55 kg, empty 40 g | Opaque white HDPE, identical. Two full of water, drawn at random |
| Recycling bin | 24 x 16 cm inside, 4 cm walls, centred at x 18, y 0 cm, long side along y | | Green plastic, in a taped outline; holds all four bottles |

Positions are from the midpoint between the two arm bases (x forward, y left, z up).

## Scoring

Evaluated at the end of the episode (no early end, so every bottle can be tried).

| Condition | Score |
|---|---|
| A full bottle is in the bin, or any bottle tipped over (more than 30 degrees) or fell off the table | 0 |
| Both empty bottles standing in the bin | 1.0 |
| One empty bottle standing in the bin | 0.5 |
| Otherwise | 0 |

"In the bin": the bottle's base on the bin's floor and its centre inside the walls. In a real trial: a photo of the
bin and the row, and the video for anything tipped or dropped.

Unintended strategies and what stops them:
- Throwing every bottle in the bin: a full bottle in the bin scores 0.
- Guessing by appearance or position: the full bottles are drawn at random and look identical to the empty ones.
- Knocking bottles over while testing them: 0.
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
leftmost bottle), as an operator would after lifting each bottle.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: bottles, bin, the random draw, `score()`, and `demo()` |
| `jointkit.py` | Rigid parts made of boxes (the 16-sided bottles), and per-step and reset hooks (hooks yam_arena without modifying it) |
| `test_throw_out_empty_bottles.py` | The task file loads, the bottles look identical, the bin holds four, the demo tests every bottle and is reachable for every draw |
| `task.json` | Submission details |

## Runtime and seeds

A demo run takes about 5 minutes of wall time on an RTX workstation. Run limit: 1,200 steps (1 minute at 20 Hz).
Pass `-E object_jitter=0`. The full bottles are drawn when the task file loads, from the operating system's random
source; set `FULL` to fix them.

## Tests

```bash
python -m pytest -q      # in this folder, in the yam_arena environment; no GPU needed
```

## Verified in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1), with the command above.

| Run | `task_score` | What the log and video show |
|---|---|---|
| Scripted demo, bottles 1 and 3 full (`FULL=0,2`) | **1.0** | The left arm lifts bottle 1 (full, set back), bottle 2 (empty, into the bin), and moves away; the right arm lifts bottle 4 (empty, into the bin) and bottle 3 (full, set back). The episode runs to the end of the plan (838 steps) |
| Dream-1 (full bottles drawn at random) | **0** | 1,200 steps (step limit). Both arms move up near their bases and stay there; no bottle is touched |
| `python -m pytest -q` | 5 passed | |

## Real-world build

- **Bottles:** four identical opaque white 0.5 l HDPE bottles with screw caps (7 cm across, 15 cm tall); fill two with
  water.
- **Bin:** a low green plastic recycling bin about 24 x 16 cm inside with 4 cm walls, in a taped outline.
- **Safety:** light, unbreakable objects.

## Still approximate or unverified

- Rigid bottles with a fixed mass: no sloshing water.
- The bin is fixed in simulation.
- One Dream-1 run (1,200 steps) scored 0: neither arm reached a bottle.
