# Close a swing-top bottle: swing the stopper on, then snap the lever over its centre while steadying the bottle

A 200 mL swing-top glass bottle of lemonade stands open on the table. Its porcelain stopper rests out beside the neck
on its wire bail, and its lever is flipped back over the top. The robot has to close it.

**Instruction:** "Close the swing-top bottle of lemonade."

A swing-top (flip-top) bottle is closed by an over-centre toggle, the same mechanism as a toggle clamp or a
clip-top preserving jar. Closing one takes two moves in a fixed order:

1. **Swing the stopper up and over onto the mouth.** It turns on its wire hinge and has to sit squarely on the mouth.
2. **Bring the lever back over the top and down the near side of the neck.** As it comes down it stretches the bail
   and pushes back, harder the further it goes, until it passes its centre. Then it snaps down by itself and clamps
   the stopper onto the mouth.

These things go wrong if they are done carelessly, and the scoring tells them apart:

- **Lever let go short of its centre:** it springs back open. The bottle is not sealed.
- **Lever shut first:** it closes on nothing. A stopper put on afterwards is not clamped, as on a real bottle.
- **Bottle not held:** the push needed to bring the lever through its centre (about 4 N sideways near the top of a
  0.45 kg bottle with a 56 mm base) pushes the bottle over or slides it away. It has to be steadied with the other
  hand. A bottle that tips 30 degrees has fallen or spilled, which scores 0.

## Why this task is different

**An over-centre toggle.** Accepted mechanism tasks on the board use latches, detents, bayonets and snap fits:
something that catches when pushed into place. An over-centre toggle works the other way. It resists more and more
and then flips, and the robot has to keep pushing through the peak force rather than stop at first resistance. Let
go one degree short and the bottle opens again. The same mechanism is in toggle clamps, buckles, cam levers,
clip-top jars and the lid clamps of pressure vessels.

**Order matters, and the instruction doesn't say what it is.** "Close the bottle" doesn't say the stopper must go
first or that the bottle must be held. A policy that shuts the lever first gets 0, and so does one that pushes
without steadying the bottle. Both are visible in the control runs below.

**Two arms because of the physics, not because the instruction asks.** One hand holds the bottle low on its body
while the other works the stopper and lever. A run with no holding hand (below) shows the unheld bottle being pushed
8.6 cm across the table, and the stopper never gets on.

**Turning a hinged part through an arc.** The stopper and lever each swing through more than 90 degrees about a
hinge on the neck. The robot has to carry them along that arc, pinched between its fingers, without knocking the
stopper off again when the lever passes over it.

## Where this comes up

- **Homes.** Swing-top bottles are common for homemade lemonade, kombucha, cordials, kefir, infused oils and home
  brew. Closing one is a daily kitchen task, and clip-top preserving jars close the same way.
- **Small food and drink producers.** Craft breweries, kombucha and cold-brew makers, cafés and farm shops often
  fill and close swing-top bottles by hand. Packaging and filling machine operators (O*NET 51-9111.00) and food
  batchmakers (O*NET 51-3092.00) close each one with a lever like this.
- **Labs and workshops.** Toggle clamps hold work on jigs, and over-centre latches close cases, lids and doors.
  Swing-top glass is used for samples and solvents. The push-through-the-peak skill is the same in all of them.

## Potential impact

- **Pushing through a peak force.** Many robot policies back off when resistance rises. Here backing off is exactly
  wrong: the lever has to be driven past its centre, against about 0.2 N m, with one hand.
- **Coordinating a holding hand with a working hand.** One hand has to brace the bottle against the force the other
  hand applies, low on the body so the two hands don't collide.
- **A fixed order that is not stated.** The stopper first, then the lever. A careless order can't be fixed later
  without reopening the lever.
- **Partial credit tells the failures apart.** The scores show whether a policy got the stopper on (0.5), sealed the
  bottle (1.0), or knocked it over (0).

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: the bottle and layout, the instruction, what happened during the trial, scoring |
| `bottlekit.py` | Builds the swing-top bottle (one articulation: the bottle, the stopper and the lever on hinges at the neck) and its over-centre bail (`bail_targets()`, run every control step) |
| `demo.py` | The scripted demo, and the control runs (`--mode stopper|lever|nohold|still`) |
| `test_scoring.py` | Offline tests of the bail's behaviour, the scoring rule and the bottle's geometry (no simulator) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Close the swing-top bottle of lemonade." \
  --scorer task_score --max-steps 1800
```

Run limit: 1,800 steps (90 s at 20 Hz). The demo reaches full score at step 665, so the limit allows more than double
that.

Second layout: set `BOT_LOT=B` in the environment (default `A`). The bottle moves 1 cm further out and 1 cm to the
right.

## Scripted demo

```bash
python demo.py task.py out_dir                  # layout A; BOT_LOT=B for layout B
python demo.py task.py out_dir --mode stopper   # control: stopper on, lever left open
python demo.py task.py out_dir --mode lever     # control: the lever shut with the stopper still off
python demo.py task.py out_dir --mode nohold    # control: both moves, without the left hand holding the bottle
python demo.py task.py out_dir --mode still     # control: the arms do not move
```

1. The left hand comes in from the arms' side, 45 degrees down, and holds the bottle low on its body (3.5 cm up).
2. The right hand comes straight down on the stopper, takes it between its fingers and carries it up and over onto
   the mouth. The stopper turns in its fingers about its hinge. Then it lets go.
3. The right hand takes the lever's paddle between its fingers and carries it back over the top and down the near
   side of the neck, through its centre, where it snaps shut. Then it lets go.
4. The left hand lets go of the bottle and both hands draw back.

Every target comes from the layout constants in `task.py` and the bottle's geometry in `bottlekit.py`. The only
feedback is the measured position of each hand, used to correct an arm's lag before letting go (by at most 1.5 cm).

## Scoring

Tools: the trial video and a protractor or phone level.

- **Sealed:** the lever has passed its centre (35 degrees short of lying down the neck) with the stopper on the mouth,
  and is still over its centre. The stopper is "on the mouth" when it is within 5 degrees of its seat.
- **Standing upright:** the bottle is tilted less than 10 degrees, with its base on the table (within 5 mm).
- **Let go:** both grippers' tool points are 3 cm or more from the lever's paddle.
- **Tipped:** at any time, the bottle tilts 30 degrees or more (fallen, or tipped far enough to spill).

| Outcome | Score |
|---|---|
| The bottle tipped at any time | **0** for the whole trial |
| Sealed, standing upright and let go, all together for 2 s | **1.0** |
| At the end, the stopper is on the mouth (sealed or not) and the bottle stands upright | **0.5** |
| Anything else (stopper still off, lever shut on nothing, bottle lying down or lifted) | **0** |

Every outcome falls into exactly one row. Holding the lever down by hand doesn't count, because the gripper has to
be 3 cm away, and a stopper set on after the lever is shut is never clamped. In code: `bail_targets()` in
`bottlekit.py`, `_watch()` and `score()` in `task.py`.

## Verification

| Run | Score | Notes |
|---|---|---|
| Scripted demo, layout A | 1.0 | Sealed; full score at step 665 (33 s); the bottle leaned at most 7 degrees while held |
| Scripted demo, layout B | 1.0 | Same script; full score at step 665 |
| Stopper on, lever left open (control) | 0.5 | Stopper on the mouth, not sealed |
| Lever shut first, stopper still off (control) | 0.0 | The lever closed on nothing |
| No holding hand (control) | 0.0 | The right hand pushed the unheld bottle 8.6 cm across the table; the stopper never got on |
| Arms still (control) | 0.0 | The bottle stays open |
| Dream-1, 1,800 steps | 0.0 | Both arms reached toward the bottle, but neither took the stopper or the lever; the bottle ended open and upright |
| `pytest test_scoring.py` | 16 passed | The bail (free, sprung back, snapping over, shut on nothing, stopper added too late, seal holds and breaks), score tiers, tilt and standing, geometry |

## Runtime and seeds

About 3 minutes per demo run and about 6 minutes per Dream-1 run on one L40S, including about a minute of start-up.
`object_jitter=0`. The layout is fixed per lot (A or B), and there is no randomness in the task.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table (x forward, away from the arms; y to the
left). Layout A, with layout B in brackets.

- **The bottle:** a 200 mL green-glass swing-top bottle of lemonade, standing upright with its base centred at
  x 28, y -2 [B: x 29, y -3]. 15 cm tall; body 5.6 cm across and 8.5 cm tall, then a shoulder up to a neck 2.5 cm
  across, and a lip 2.8 cm across at the mouth. 0.45 kg with the lemonade.
- **The swing-top:** a steel wire bail pivoting on a wire band round the neck, 2.5 cm below the mouth, with its two
  pivots on either side of the neck along x. A white porcelain stopper (2.3 cm across, 1.4 cm tall, red rubber
  washer, 18 g) hangs from the bail. A steel lever (12 g) has two wire arms 5 cm long and a thumb paddle 1.8 cm
  wide at its end.
- **The open state at the start:** the stopper rests out beside the neck on the right (-y) side, 95 degrees from its
  seat, against the bail's stop. The lever is flipped back over the top, lying out to the left (+y), 212 degrees from
  closed.
- **Closed:** the stopper on the mouth, and the lever lying down the right (-y) side of the neck, leaning out
  45 degrees.
- **Robot:** both arms start in the raised ready pose with their grippers open.

## Fidelity

- **Rigid parts and a joint-level bail.** The bottle, stopper and lever are rigid. A real bail is a wire loop linking
  the lever to the stopper. Here both hinge on the neck's pivot, and the bail's action is a rule applied to their
  joints every control step:
  - with the stopper on the mouth, the lever is sprung back open from 75 degrees short of closed (0.30 N m/rad, about
    0.21 N m at its centre);
  - past its centre (35 degrees short of closed) it is driven closed and the stopper is clamped down;
  - otherwise both swing freely.
- **No liquid simulation.** The lemonade is drawn, and its mass is in the bottle. "Tipped" (30 degrees) stands in for
  spilling.
- **The bottle's collision shape.** It is a stack of cylinders.

## Real-world build

- **The bottle:** a 200 mL swing-top bottle with a porcelain or plastic stopper, filled with water. These are sold
  for homemade lemonade, oils and cordials.
- **The open state:** set it as above before each trial: stopper resting out to the right, lever flipped back to the
  left.
- **Scoring:** a phone level or protractor for the bottle's tilt, and the video for whether the lever has snapped
  over (a real lever stays down by itself once over).

## Third-party assets

None. Every part is built from primitive shapes in `bottlekit.py`.
