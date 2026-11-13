# Clear a desk for cleaning, leaving the one cup that is stuck down

Three identical white ceramic pen cups stand in a row on a desk, with an empty desk tray between them and the robot.
One of the cups, drawn at random each trial, is stuck to the desk. It looks exactly like the other two. The robot
must put the loose cups in the tray and leave the stuck one where it is, without tearing it off.

**Instruction:** `Clear the desk for cleaning: put everything that isn't stuck down into the tray.`

## Why this task

Office cleaners (janitors and cleaners, O*NET 37-2011, about 2 million US workers) clear desks before wiping them.
Some desk items are fixed down with adhesive pads or mounting tape, and nothing on them shows it. A robot that does
not notice keeps pulling, and tears the item off and damages the desk; a robot that skips items it has not tried
leaves the job undone.

The stuck cup cannot be found from an image: the three cups are identical, and which one is stuck changes every
trial. The only way to tell is to try to lift each cup gently and notice which one does not come. That makes it a
manipulation problem, not a recognition one: the policy has to control how hard it pulls, notice that a lift failed,
and move on. The instruction's negation ("everything that isn't stuck down") states the goal; it does not say which
item is stuck or how to find out.

## Design

| Object | Size | Mass | Notes |
|---|---|---|---|
| Pen cups (3) | 8 cm across, 10 cm tall, 4 mm walls, 6 mm floor | 0.28 kg each | White glazed ceramic, identical. At x 32 cm, y 14, 0 and -14 cm |
| Stuck cup | one of the three | | Picked by a die each trial (1-2 left, 3-4 middle, 5-6 right). Real trials: a steel disc in its base on a magnet under the desk top, 60 N pull-off. Simulation: a fixed joint to the world that breaks above 60 N or 3 N m |
| Desk tray | 12 x 33 cm inside, 4 cm walls 6 mm thick, 4 mm floor, centred at x 19, y 0 cm, long side along y | | Grey polypropylene, fixed in a taped outline with double-sided tape |

Positions are from the midpoint between the two arm bases (x forward, y left, z up). The desk is the station table.

## Scoring

Evaluated every control step; the score at the end of the episode counts.

| Condition | Score |
|---|---|
| The stuck cup was ever moved more than 3 mm from its spot (torn off), or any cup tipped past 45 degrees or fell off the desk | 0 |
| Both loose cups standing upright (under 15 degrees) on the tray's floor, centres inside the tray at least 1 cm from the walls (success, held 1 s) | 1.0 |
| One loose cup in the tray that way | 0.5 |
| Otherwise | 0 |

In a real trial: a photo of the tray, a ruler or the taped spot for the stuck cup (a cup off its magnet has been
torn off), and the video for anything tipped or dropped.

Unintended strategies and what stops them:
- Sweeping or dragging all three cups toward the tray: the stuck cup tears off (0); dragged cups also tip.
- Doing nothing, or leaving cups untried: at most 0.5 per loose cup moved.
- Memorising which cup to skip: the stuck cup is drawn at random each trial.
- Lifting hard and fast to "test" each cup: pulling the stuck cup above 60 N tears it off (0).

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clear the desk for cleaning: put everything that isn't stuck down into the tray." \
  --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/desk-clear-stuck-cup-demo
```

The demo is one fixed plan for every draw: it tries every cup the same way. Each cup is pinched lightly (finger
target 0.5 mm inside the cup's flats), lifted slowly, carried over the tray, lowered and released. A loose cup comes
with the hand; on the stuck cup the pads slip up its sides before the joint's 60 N limit, so it stays, and the hand
carries nothing to the tray.

1. **Right arm:** the right cup into the tray's right slot, then the middle cup into the middle slot.
2. **Left arm:** the left cup into the tray's left slot.

The episode ends on success as soon as both loose cups are in the tray, so with the middle cup stuck it ends before
the middle cup is tried. `GLUED=0`, `1` or `2` fixes the stuck cup (left, middle, right) instead of drawing it.

**Pulling-too-hard contrast (expected score 0):** the same plan with a firm pinch (8 mm inside the flats):

```bash
GLUED=0 DEMO_FORCE=1 inspect-robots run ...   # same arguments as above
```

`DESK_DEBUG=1` prints the stuck cup's position every 5 control steps.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: cups, tray, the random stuck cup, `score()` / `success()`, and `demo()` |
| `jointkit.py` | Rigid parts made of boxes, joints that can be switched on at reset (here: a breakable fixed joint to the world), and per-step and reset hooks (hooks yam_arena without modifying it) |
| `test_desk_clear_stuck_cup.py` | The task file loads, the cups are identical, the layout fits, the demo is the same for every draw and is reachable |
| `task.json` | Submission details |

## Runtime and seeds

A demo run takes about 4 minutes of wall time on an RTX workstation. Run limit: 1,200 steps (1 minute at 20 Hz).
Pass `-E object_jitter=0`. The stuck cup is drawn at each reset from the operating system's random source; set
`GLUED` to fix it.

## Tests

```bash
python -m pytest -q      # in this folder, in the yam_arena environment; no GPU needed
```

## Verified in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1), with the commands above.

| Run | `task_score` | What the log and video show |
|---|---|---|
| Scripted demo, left cup stuck (`GLUED=0`) | **1.0** | The left arm's light lift slips off the stuck left cup, which stays on its spot; the right and middle cups go into the tray. Success at step 498 |
| Scripted demo, right cup stuck (`GLUED=2`) | **1.0** | The right cup stays; the left and middle cups go into the tray. Success at step 499 |
| `GLUED=0 DEMO_FORCE=1` | **0** | The firm pinch tears the stuck cup off; all three cups end in the tray. The plan ends at step 607 |
| Dream-1 (middle cup stuck, drawn at random) | **0** | 1,200 steps (step limit). Both arms draw back beside their bases within the first seconds and stay there; no cup is touched |
| `python -m pytest -q` | 6 passed | |

## Real-world build

- **Cups:** three identical white glazed-ceramic pen cups, 8 cm across, 10 cm tall. Glue a thin steel disc into the
  base of one.
- **Stuck cup:** a magnet under the desk top (or a 6 mm hardboard desk overlay) at each cup spot, adjusted with
  spacers so a straight pull of 60 N (a luggage scale on a loop round the cup) lifts the cup off. Only the cup with
  the steel disc sticks, so moving that cup to the drawn spot sets the trial.
- **Tray:** a grey polypropylene desk tray about 12 x 33 cm inside, 4 cm tall, taped down in its outline.
- **Safety:** nothing is sharp or heavy; a cup that is torn off lands on the desk.

## Still approximate or unverified

- The cups are rigid 16-sided prisms; the adhesive is a breakable joint, not a modelled pad.
- The grip force comes from the finger target's depth into the cup and the gripper's drive stiffness; the light
  pinch's force was not measured, only that it slips below 60 N and lifts a 280 g cup.
- The tray is fixed in simulation.
- One Dream-1 run (1,200 steps, middle cup stuck) scored 0: neither arm reached a cup.
