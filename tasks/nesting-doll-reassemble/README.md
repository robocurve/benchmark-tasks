# Put a five-piece wooden nesting doll set back together, each doll closed with its own top and its face lined up

A wooden nesting doll set has been taken apart on the table. The two largest dolls lie open: each bottom half stands
open end up and each top half stands on its rim. The third doll stands closed, holding the two smallest. The robot
must put the set back together and let go:

- the third doll inside the second;
- the second doll closed with its own top and standing inside the first;
- the first doll closed with its own top over it;
- each top turned so its painted face lines up with the apron painted on its bottom half.

Instruction given to the policy:

`Put the nesting dolls back together, each closed with its own top and its face lined up, the smaller ones inside the largest.`

A nesting doll set is a familiar toy and ornament, and putting one back together is a small everyday puzzle. Each top
only seats on its own bottom. A top fits over its bottom half's spigot as a 1.5 mm sliding fit, so it has to come down
square and centred. The other doll's top is either too big, sliding down over the whole bottom half, or too small,
sitting up on the spigot's rim. The order matters too:

- the second doll goes into the first before the first is closed;
- the second doll cannot be carried by its top, which only rests on it.

Lining up the face is a turn of the top about its own axis, read from the paint.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the dolls (lathed meshes), `score()`, `success()` and `distance()` |
| `demo.py` | Scripted demo through `inspect-robots`; writes `result.json`, videos and `setup.png` |
| `meshkit.py` | Lathed triangle-mesh bodies with signed-distance colliders, and their masses |
| `partkit.py` | Finger points for the release check (the scene itself uses meshkit bodies only) |
| `clearance.py` | Arm-to-arm clearance from the YAM collision meshes (logging only) |
| `task.json` | The submission details |

Offline tests: `tests/test_nesting_doll_reassemble.py` in the team repository (23 tests, no simulator).

## Run a trial

In the Isaac Lab-Arena environment, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put the nesting dolls back together, each closed with its own top and its face lined up, the smaller ones inside the largest." \
  --scorer task_score --store-frames --max-steps 2400 --log-dir runs/dream
```

Scripted demo and its wrong attempts:

```bash
python demo.py task.py demo_out                                         # full score
DEMO_VARIANT=skip_inner python demo.py task.py skip_inner --no-record   # the third doll left out
DEMO_VARIANT=unturned python demo.py task.py unturned --no-record       # the tops put on without turning them
DEMO_VARIANT=outside python demo.py task.py outside --no-record         # the second doll closed outside the first
DEMO_VARIANT=hold python demo.py task.py hold --no-record               # the last top never let go
```

The demo is open loop. It uses the fixed layout, the clock and the arms' own joint readings.

- **Grip:** every pinch closes along x, toward and away from the arms, and only to the piece's width less 4 mm. In the
  simulator one finger drives harder than the other, and full closes along y shoved the light halves 20 to 35 mm and
  tipped them.
- **Release over the bore:** the inner dolls are let go over the next bore, because the fingers cannot follow them in.
- **Clearance:** the tops are carried with 20 mm to spare, because they slip about 8 mm down in the pinch.

## Scoring

Everything is read at the end, from the video and a photo. Nothing scores while a gripper fingertip is within 20 mm of
any piece.

- **Inside:** a doll, or a bottom half, stands on the floor of the next bottom half. It is on that half's axis (within
  3 mm) and upright (within 8 degrees).
- **Closed:** a top sits on its own bottom half. Its rim is on the shoulder (within 2 mm), it is centred (2.5 mm) and
  square (5 degrees), and its face is within 15 degrees of the apron.

The levels:

- **1.0:** the third doll is inside the second, the second is closed and inside the first, and the first is closed and
  upright (5 degrees). No piece moves more than 1 mm and no fingertip comes near over the last 1 s (21 samples).
- **0.75:** the second doll, closed with the third in it, stands inside the first. This level also covers a fully
  assembled set that is not yet still.
- **0.5:** the second doll is closed with the third in it.
- **0.25:** the third doll is inside the second doll's bottom half.
- **0:** anything else. A top on the other doll's bottom does not count.
- **Fall:** if any piece falls off the table, the trial scores 0 for good. The record is cleared only when the scene is
  reset.

`task_score` is `score()` at the end of the run.

## Scores

- Scripted demo: 1.0 at step 1,422 (1.0 also in two earlier batches on the same scene).
- Wrong attempts:
  - `skip_inner` 0.0;
  - `unturned` 0.25;
  - `outside` 0.5;
  - `hold` 0.0.
- Dream-1 run (2,400 steps): 0.0. It moved its right arm among the pieces on the right for the two minutes and put
  nothing together.
- The two arms stay at least 92 mm apart throughout the demo.

## Runtime and seeds

2,400 control steps (2 minutes at 20 Hz) per trial; the scripted demo finishes in about 1,420. One fixed layout: run with `-E object_jitter=0`. There is no other
randomness.

## Fidelity

- **Shape:** each half is a rigid lathed mesh (48 sides) of a classic matryoshka outline: a flat foot, a full belly, a
  waist at the split, shoulders, a neck and a round head. Each has a signed-distance collider at about 0.5 mm.
- **Walls:** 2.5 mm, with 4 mm floors.
- **Spigot:** 2 mm thick and 8 mm tall.
- **Mass:** from linden at 0.45 g/cm3.
- **Fit:** a real set's halves are a snug friction fit; here each is a 1.5 mm sliding fit that seats by the top's own
  weight. The scorer reads a seated top from its rim on the shoulder, not from a press.
- **Third doll:** the third doll and the two inside it are one solid body.
- **Paint:** the painted face, shawl and apron are visual only.
- **Friction:** 0.5.

## Real-world build

- **Set:** a five-piece turned linden nesting doll set, the largest doll 150 mm tall and about 71 mm across (a common
  size, sold as 5.9 in by 2.8 in, about $20 to $30).
- **Fits to check:**
  - each top seats on its own bottom by hand without forcing;
  - the second doll, closed, drops into the first doll's bottom half;
  - the third doll, closed, drops into the second doll's.
- **Faces:** if the painting does not make it obvious, mark the middle of each face and each apron with a small dot
  of paint pen.
- **Layout:** stand the halves on printed outlines at the positions in `task.json`, the faces turned as marked.
- **Checks:**
  - a photo at the end from the camera side, to read each face against its apron with a 15 degree protractor card;
  - a photo from the side, to see that each top sits down on its shoulder: no gap under the rim.
  - To check an inner doll, open the outer doll after the trial: the score is read from the video first, then the photo
    of the opened set confirms what is inside.
- **Hazards:** none. The pieces are smooth varnished wood and nothing sheds on the grippers.

## Licence

The task is released under the MIT licence. It uses no library or third-party assets. Every shape is generated in
`task.py` with `meshkit.py`, which is part of this package.
