# Stack five balls into a pyramid in a square tray

Five identical 76 mm wooden balls sit in two ball racks, one ball per square hole. Between the arms is an empty
square display tray, 168 x 168 mm inside with a 25 mm rim. Four balls fit on its floor, two by two, and the fifth
fits only on top of them. The robot builds the pyramid.

Instruction: *Put four of the balls in the tray in a two-by-two square, then rest the fifth ball on top of them, in the middle.*

Files: `task.py` (the task file: objects, `success`, `distance`, `score`, `demo`), `task.json`, and
`test_task.py` (offline checks).

## Run and score a trial

Built against the YAM Isaac Lab-Arena embodiment at commit `bd62771`. Run these from the root of the task
repository (the folder that holds `tasks/ball-pyramid-tray/`), on the GPU machine with `inspect-robots` installed.

Dream-1 (the run limit is 3,600 steps, 3 minutes at 20 Hz):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/ball-pyramid-tray/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put four of the balls in the tray in a two-by-two square, then rest the fifth ball on top of them, in the middle." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/pyramid-dream
```

Scripted demo (64.5 s of motion, 1,290 steps at 20 Hz, with the last ball let go at 60.0 s; success ends the run
during the final hold):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/ball-pyramid-tray/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put four of the balls in the tray in a two-by-two square, then rest the fifth ball on top of them, in the middle." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/pyramid-demo
```

The run prints `task_score` at the end. It is 1.0 when the trial ends in success, and otherwise `score()` at the
last step (`SCORE_OVER = "end"`).

- **Runtime per trial:** wall clock on an RTX 5090, Isaac start-up included: 185 s for the demo, 353 s for the Dream-1 run. Simulated time is
  at most 180 s for Dream-1 and about 65 s for the demo, plus about 85 s of Isaac start-up.
- **Seeds:** run with `-E object_jitter=0`. The start layout is then fixed and the seed does not change it. Do not
  use the default jitter (0.03 m): a ball shifted 3 cm off its 50 mm hole starts out of its seat and rolls away.
- **Assets:** none. Every object is a `Box` or `Sphere` primitive, so there is nothing to attribute.

## Offline tests

```bash
YAM_ARENA_PATH=~/yam-isaac-lab-arena python -m pytest -q tasks/ball-pyramid-tray/test_task.py
```

`YAM_ARENA_PATH` points at a clone of the embodiment repo; it is not needed where `yam_arena` is installed. The
tests need only numpy and pytest. They check that:

- the file loads, both arms' plans solve IK at 20 Hz within 0.5 mm and 0.1 deg and inside the joint limits, the
  demo fits the run limit, it ends with a still hold, the TCPs stay 18 cm or more apart, only one arm is over the
  tray at a time, and lowering and pushing are slow;
- a box model of the pads and the closed fingers clears the balls already in the tray at every bottom release
  (by 10.3 mm or more), at every corner push (by 8.3 mm or more), and at the top release (by 5 mm or more), for a
  held ball anywhere within the grip offset plus the wrist error;
- each released ball lands between its corner and its cornered neighbours, the pad tips clear the rim, and the
  carrying heights clear the balls' tops;
- the geometry rules below hold: no fall-through, only four on the floor, and a rim that will not tip a ball;
- every scoring branch and shortcut listed under "Scoring" returns its value;
- no objects overlap at the start, every ball is seated in a hole, and every ball fits the gripper with a
  centimetre to spare.

## Geometry, done first

Symbols: ball diameter `d` = 76 mm (`r` = 38 mm); tray inner width `W`; rim height above the floor `w` = 25 mm.

**Where a ball can sit.** The rim is lower than a ball's equator, so a ball touches it at the rim's top inner
edge, not its face. The edge is `r - w` = 13 mm below the ball's centre, so the centre comes no closer to the rim's
inner face than `e = sqrt(r^2 - (r - w)^2)` = 35.7 mm. A ball pushed into a corner therefore has its centre
`c = W/2 - e` from the tray centre in x and in y. With `W` = 168 mm, `c` = 48.3 mm. Four cornered balls are
`2c` = 96.6 mm apart centre to centre, so the bottom layer can open by `s = 2c - d` = **20.6 mm** beyond touching.
(For a rim taller than `r`, this is the usual `W = 2d + s`.)

**Where the top ball rests.** Each bottom ball's centre is `sqrt(2)·c` from the tray centre horizontally, so a
top ball touching all four has its centre `h = sqrt(d^2 - 2c^2)` above theirs:

| bottom layer | spacing | `h` (top centre above bottom centres) |
|---|---|---|
| touching (s = 0) | 76 mm | d/sqrt(2) = 53.7 mm |
| in the corners (s = 20.6 mm) | 96.6 mm | 33.3 mm |
| falls through | s >= (sqrt(2) - 1) d = 31.5 mm | 0 |

The rim stops the layer at the corners. So in this tray the top ball always rests between 33.3 and 53.7 mm above
the bottom centres and can never drop between them: the largest possible spread, 20.6 mm, is 10.9 mm short of the
31.5 mm fall-through spread. Under its weight the bottom balls roll outward, because nothing resists rolling, until
the rim stops them in the corners, so the pyramid's resting height is 33.3 mm. Two more things follow from the
same numbers:

- **Only four fit on the floor.** Bottom centres can reach a square of side `2c` = 96.6 mm. Five points spread as
  far apart as possible in a square of side `L` are `L/sqrt(2)` = 68.3 mm apart, less than `d`. A fifth ball must go
  on top. Pushing the balls in one at a time cannot reach full score.
- **The rim holds.** The top ball pushes each bottom ball along its diagonal with a horizontal force of
  `(mg/4)·(sqrt(2)·c / h)` = 0.51 mg, or 0.36 mg on each rim. Tipping over a rim's edge needs that force times its
  13 mm lever to beat the 1.25 mg vertical load times its 35.7 mm lever. The margin is 9.5 times, and the test
  asks for 5. During an impact, the vertical part of the push grows with the horizontal part and still wins.

**The clearance rule against stability.** The first GPU round measured held objects sitting 12-20 mm off the
pads' centre and asked for at least 20 mm of side clearance. Here, a held ball can sit off the TCP only by
**(93.9 - 76) / 2 = 9 mm**, since the far pad stays at its open stop; add 5 mm of wrist error for 14 mm. That bound
is why the ball is 76 mm and not 60 mm: a 60 mm ball could sit 17 mm off. Measured along a corner's diagonal from
the tray centre (`u`), a ball dropped next to two cornered neighbours must land between `u = h` = 33.3 mm, where
it would touch them, and `u = sqrt(2)·c` = 68.3 mm, where it touches both rims. That leaves **35 mm of room**. The
demo lets go at `u` = 54.3 mm, so a ball 14 mm off either way lands at 40.3-68.3 mm. A larger `s` would give more
room but a lower, less stable top ball; at the 31.5 mm limit it falls through. 20.6 mm meets the clearance rule
(35 mm of room against 28 mm of possible error) and leaves 10.9 mm before fall-through, so both hold.

**Pads between neighbours.** Open pads reach 57 mm from the TCP and are 40 mm wide. Closing along x or y would
leave at most 1.6 mm between a pad and a neighbour, wherever the ball is let go. The bottom balls are therefore carried with the fingers
closing along the corner's diagonal (yaw +-45 deg). One pad then passes over the corner of the rim, which is why
the rim is low: the pad tips are 11 mm above it at the release. The other pad sits near the tray centre, 10.3 mm
from both cornered neighbours. The top ball is let go with the fingers closing along y (yaw 0), so its pads hang
in the gaps between the bottom balls' tops.

**Why the corner push.** A ball set down on a flat floor stays where it lands; it does not roll into a corner by
itself. If a ball stays 14 mm inside its corner, a later ball's pads and push overlap it by 3.7 mm. So after each
release the arm lifts clear, closes its fingers, lowers the closed fingers (20 x 40 mm) to the ball's equator on
its inner side, and pushes outward along the diagonal at under 2.5 cm/s. It ends 2 mm past where a cornered
ball would be touched, a gentle press through the soft wrist, and holds there for 1.5 s so the ball settles in the
corner (stopping 2 mm short left the ball free to roll along a rim).

**Loose balls at the start.** A sphere on a flat table never settles, and nothing resists rolling. Each ball
therefore starts seated on the four edges of a 50 mm square hole in a 12 mm board, centre 28.6 mm above the board
and bottom 2.6 mm clear of the table. This is the same seat that held a 56 mm sphere still in a sizing-board task
on the GPU. The pads grip 6 mm above the centre, with their tips 21.6 mm above the board.

## Scoring

- 0.15 per ball resting on the tray floor: centre 38 +- 4 mm above the floor and within 51.3 mm of the tray centre
  in x and y. At most four fit, so this part is worth at most 0.6.
- 0.4 when all four are in place and a fifth rests with its centre within 15 mm of the tray centre and 28.3-58.7
  mm above the bottom centres. That is the band of `h` above, plus or minus 5 mm.
- Success: everything above, held for 1 s.

Construct validity, all tested:

| Outcome | Score |
|---|---|
| start (all in the racks) | 0 |
| five balls in a row on the table | 0 |
| five balls in a row across the tray, on the rims | 0 |
| k balls on the floor, nothing on top | 0.15 k |
| four on the floor, the fifth left on the table or in its rack | 0.6 |
| four on the floor, the fifth lying in the gap between two balls at the rim (48 mm off centre) | 0.6 |
| four on the floor, the fifth 16 mm off centre | 0.6 |
| four on the floor, the fifth below the band (sunk into the layer) or above it (held over the layer) | 0.6 |
| three on the floor and one on top | 0.45 |
| top ball still rolling | 0.6 |
| a bottom ball still rolling | 0.45 |
| pyramid on four cornered balls, or on four touching balls | 1.0 |

## Demo

The left arm takes the three balls from its rack and the right arm the two from its rack. Each lifts a ball
straight up out of its hole, carries it 15 mm above the bottom balls' tops, and turns the grip to the corner's
diagonal. It lowers the ball at 3.4 cm/s and lets go 5 mm above the floor. It then lifts 3 cm before anything
else, rises clear of the balls' tops, closes its fingers, and pushes the ball into its corner. The arms take turns
in the tray: left near-left, right far-right, left far-left, right near-right. Then the left arm lowers the fifth
ball over the tray centre with the fingers along y and lets go 12 mm above its seat. It lifts 3 cm and leaves.
The plan ends 4.5 s after that last release, with both arms still for the final 2.5 s.

The top ball is let go 12 mm above its seat rather than at the 3-10 mm proven in the first GPU round. A ball held
up to 9 mm off the TCP still clears the two bottom balls it is shifted toward only from that height. With the
full 14 mm of slop it may touch them just before the release, which only pushes it toward the pocket. Wherever it is
let go, it lands within 14 mm of the centre in y and 5 mm in x. That is well inside the pocket the four centres
make (48.3 mm either way), so it rolls to the seat.

## Design choices

- **76 mm balls (3 in hardwood craft balls).** They fit the gripper with an 18 mm margin, bound the grip offset at
  9 mm, read clearly in the top camera, and are a stock size. Lacrosse balls (63.5 mm) or billiard balls (57 mm)
  would allow up to 15-18 mm of grip offset.
- **Low rim (25 mm).** The pads must pass over it at a corner. It still confines: see "The rim holds".
- **Top credit needs a complete layer.** A ball cannot rest on three balls in a square: the tray centre lies on
  the edge of their support triangle. The rule also stops a held ball over a partial layer from earning anything.
- **The band, not just a threshold.** A ball held above the layer (above 58.7 mm) earns nothing for the top. A
  ball held still inside the band cannot be told apart from one resting there by object state alone. A photo
  would show the gripper, but the scorer cannot, so success still needs the whole pyramid at rest for 1 s.
- **Racks, not loose balls on the table.** See "Loose balls at the start".

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 185 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 3,600-step run limit; 353 s wall clock.
- Offline tests: 49 passed.
