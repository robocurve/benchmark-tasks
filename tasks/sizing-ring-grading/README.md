# Grade apricots by size with a sizing board

Five apricots of one colour sit in a picking tray: three 44 mm across, two 56 mm across. A closed box between
the arms is the small-grade bin; its lid is a sizing board with one 50 x 50 mm square opening, so a fruit that
passes through the opening drops into it and a fruit that does not stays sitting in the opening. An open box
in front of the right arm is the large-grade bin. The robot grades every fruit with the board.

Instruction: *Grade the fruit with the sizing board: fruit that passes through the opening belongs in the
small-grade bin, the rest in the large-grade bin.*

## What makes this task distinctive

The sizing board is the lid of the small-grade bin, so grading and sorting are one physical test: a fruit that
passes the 50 mm opening drops into the bin, and one that does not stays in the opening and has to be moved on to
the large-grade bin. The fruits share one colour and differ by 12 mm in diameter, so the opening, not the colour,
settles each grade. A fruit in the wrong bin costs 0.2.

Files: `task.py` (the task file: objects, `success`, `distance`, `score`, `demo`), `task.json`,
`test_task.py` (offline checks).

## Run and score a trial

Built against the YAM Isaac Lab-Arena embodiment at commit `bd62771`. From the root of the task repository (the folder that holds
`tasks/sizing-ring-grading/`), on the GPU machine with `inspect-robots` installed:

Dream-1 (the run limit is 3,600 steps, 3 minutes at 20 Hz):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/sizing-ring-grading/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Grade the fruit with the sizing board: fruit that passes through the opening belongs in the small-grade bin, the rest in the large-grade bin." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/sizing-dream
```

Scripted demo (41.2 s of motion including a 1.5 s final hold, 824 steps at 20 Hz; success ends it during the hold):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/sizing-ring-grading/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Grade the fruit with the sizing board: fruit that passes through the opening belongs in the small-grade bin, the rest in the large-grade bin." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/sizing-demo
```

The run prints `task_score` at the end: 1.0 when the trial ends in success, otherwise `score()` at the last
step (`SCORE_OVER = "end"`).

- **Runtime per trial:** wall clock on an RTX 5090, Isaac start-up included: 95 s for the demo, 366 s for
  the Dream-1 run. Simulated time is at most 180 s for Dream-1 and about 42 s for the demo.
- **Seeds:** run with `-E object_jitter=0`; the start layout is then fixed and the seed does not change it.
  Do not use the default jitter (0.03 m): neighbouring apricots are 65 mm apart centre to centre with radii
  summing to 50 mm, so a 3 cm shift each can start two of them overlapping.
- **Assets:** none. Every object is a `Box` or `Sphere` primitive; there is nothing to attribute.

## Offline tests

```bash
YAM_ARENA_PATH=~/yam-isaac-lab-arena python -m pytest -q tasks/sizing-ring-grading/test_task.py
```

`YAM_ARENA_PATH` points at a clone of the embodiment repo (not needed where `yam_arena` is installed).
They need only numpy and pytest and check: the file loads; both arms' demo plans solve IK at 20 Hz within
0.5 mm and 0.1 deg and inside the joint limits; the demo fits the run limit; the two TCPs stay over 18 cm
apart; the pad tips never come within 4 mm of the board's top; every partial-credit branch; the shortcuts
below; that a 56 mm fruit cannot pass the opening and the board closes the bin everywhere else; no start
overlaps; every fruit rests on the tray floor; every fruit fits the gripper with a centimetre to spare.

## Design numbers

- **Gripper:** the YAM pads open to **94 mm** (`FINGER_OPEN = -0.04695` m per finger; the embodiment's
  `test_constants.py` checks the 94 mm gap). With a 1 cm margin anything up to 84 mm is graspable; the largest
  fruit here is 56 mm. Open pads are 10 mm thick (outer faces 57 mm from the grip centre) and 40 mm wide.
- **Sizes:** small apricots 44 mm, large 56 mm (12 mm apart), opening 50 mm: 6 mm to spare for a small fruit,
  6 mm too big for a large one. A large fruit sits in the opening with its centre 12.6 mm above the board top,
  high enough for the pads (whose tips reach 13 mm below the grip point) to take it back out without touching
  the board.
- **Board height:** 95 mm of clearance under the board, enough for one small fruit to land on top of another
  without the upper one jamming in the opening (2 x 44 mm + 7 mm).
- **Demo:** the left arm takes each apricot from the tray and lets go of it with its bottom 10 mm above the
  opening. The small ones drop through; each large one stays in the opening, and the right arm lifts it out
  and sets it in the large-grade bin while the left arm waits beside the board.

## Design choices

- **Fruit:** apricots, not apples; at 44 and 56 mm they fit the gripper with a wide margin.
- **Board on the bin walls, not on separate posts:** the board is the bin's lid, flush on its walls. A board
  on posts over an open bin would leave gaps beside it, and a robot could then drop fruit into the
  small-grade bin past the board, judging size by eye. With the lid, the opening is the only way in.
- **Wrong-bin penalty (-0.2 per apricot in the wrong bin):** added for construct validity. With 0.2 per
  correct fruit alone, putting all five in the large-grade bin without testing any of them would score 0.4;
  with the penalty it scores 0. A large fruit cannot reach the small-grade bin at all, so the penalty only
  ever applies to a small fruit put in the large-grade bin.
- **A fixed board rather than a hand-held ring:** loose objects in the simulator must be single primitives,
  so a ring with a hole can only be built from fixed slabs.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 95 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 3,600-step run limit; 366 s wall clock.
- Offline tests: 32 passed.
