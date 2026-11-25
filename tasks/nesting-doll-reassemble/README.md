# Put a five-piece wooden nesting doll set back together, each doll closed with its own top and its face lined up

This task builds on an earlier submission, whose 48-hour revision hold lapsed without a revision. This version answers
that review: each top is now a snug friction fit that must be pressed home (see the table below).

A wooden nesting doll set has been taken apart on the table. The two largest dolls lie open: each bottom half stands
open end up and each top half stands on its rim. The third doll stands closed, holding the two smallest. The robot
must put the set back together and let go:

- the third doll inside the second;
- the second doll closed with its own top and standing inside the first;
- the first doll closed with its own top over it;
- each top turned so its painted face lines up with the apron painted on its bottom half;
- each top pressed home: a top let go on its spigot stops 4 mm proud on the snug fit.

Instruction given to the policy:

`Put the nesting dolls back together, each closed with its own top and its face lined up, the smaller ones inside the largest.`

A nesting doll set is a familiar toy and ornament, and putting one back together is a small everyday puzzle. Each top
only seats on its own bottom. A top goes over its bottom half's spigot with 1.5 mm to spare, so it has to come down
square and centred. The other doll's top is either too big, sliding down over the whole bottom half, or too small,
sitting up on the spigot's rim. The order matters too:

- the second doll goes into the first before the first is closed;
- the second doll is carried by its bottom half, not by its top.

**The snug fit.** On a real set the last few millimetres of a top's travel are a friction fit: let go, the top stays
up, and you press it down until it stops on the shoulder. Here the last 4 mm are that fit. A top let go on its spigot
stops 4 mm proud; it slides home only under an axial press of more than 4 N (first doll) or 3 N (second doll), against
friction. Its own weight (0.15 N and 0.08 N) moves it under 0.2 mm in a two-minute trial. So the robot has to do what a
person does: set the top on square, then press it down.

Lining up the face is a turn of the top about its own axis, read from the paint.

## What changed for the review

| Review point | Change |
|---|---|
| Construct validity: the tops seated under their own weight with 1.5 mm clearance, removing the seating force of the snug real fit | The last 4 mm of each top's travel is now a friction fit that needs an axial press of 4 N (first doll) or 3 N (second), over 25 times the top's weight. A top only let go on its spigot stands 4 mm proud and does not count as closed: the closed check now needs the rim within 1 mm of the shoulder (was 2 mm). |
| Use representative real doll halves | Sizes match a real, buyable set: the Spire 5-piece 15 cm set, largest doll 15 cm by 7.1 cm, unfinished linden. The force is an estimate, not a measurement; the real-world build below says how to measure it and set the simulation to match. |
| Feasibility: the demo used gravity-seated tops | The new demo sets each top on its spigot, lets go, then presses it home with the closed fingers on its head, aimed 25 mm past home. It scores 1.0 (see Scores). A new wrong attempt, `rest`, sets both tops on without pressing them and scores 0.25. |
| Update the fidelity note | Rewritten below. |

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the dolls (lathed meshes), the friction-fit rings, `score()`, `success()` and `distance()` |
| `demo.py` | Scripted demo through `inspect-robots`; writes `result.json`, videos and `setup.png` |
| `meshkit.py` | Lathed triangle-mesh bodies with signed-distance colliders, and their masses |
| `partkit.py` | The tops as parts with a slider (the hidden fit ring), and finger points for the release check |
| `clearance.py` | Arm-to-arm clearance from the YAM collision meshes (logging only) |
| `task.json` | The submission details |
| `tests/test_scoring.py` | Scorer and fit-geometry tests, no simulator (`python -m pytest tests -q`; 10 pass) |

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
DEMO_VARIANT=rest python demo.py task.py rest --no-record               # both tops set on, never pressed home
```

The demo uses the fixed layout, the clock and the arms' own joint readings. One step reads poses from the simulator:
before a top goes down, the hand is shifted until the top's rim is centred over its bottom half as that now stands,
because the top slips and swings 2 to 4 mm in the pinch, the bottom halves shift up to 6 mm when the inner dolls drop
in, and the skirt clears the spigot by only 1.5 mm. The press then goes to the top's head where it now stands.

- **Grip:** every pinch closes along x, toward and away from the arms, and only to the piece's width less 4 mm. In the
  simulator one finger drives harder than the other, and full closes along y shoved the light halves 20 to 35 mm and
  tipped them.
- **Release over the bore:** the inner dolls are let go over the next bore, because the fingers cannot follow them in.
- **Clearance:** the tops are carried with 20 mm to spare, because they slip about 8 mm down in the pinch.
- **Press:** after letting a top go on its spigot, the arm closes its fingers and comes down on the top's head, aimed
  25 mm past the seated height, so its position error pushes the top home over the fit; then it lifts away. Aimed 4 mm
  past (a first try), the arm stopped on the head about 7 mm short of its target and did not move the top.

## Scoring

Everything is read at the end, from the video and a photo. Nothing scores while a gripper fingertip is within 20 mm of
any piece.

- **Inside:** a doll, or a bottom half, stands on the floor of the next bottom half. It is on that half's axis (within
  3 mm) and upright (within 8 degrees).
- **Closed:** a top is pressed home on its own bottom half. Its rim is on the shoulder (within 1 mm; a top only let go
  on its spigot stands 4 mm proud), it is centred (2.5 mm) and square (5 degrees), and its face is within 15 degrees
  of the apron.

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

All from this version of the task (the friction fit), on one fixed layout.

- Scripted demo: 1.0 at step 1,818 of 2,400. Both tops pressed home: the fit rings travelled 3.98 mm (first doll) and
  4.03 mm (second) of their 4 mm.
- Wrong attempts:
  - `rest` (both tops set on and turned, never pressed) 0.25: each top stands 4 mm proud, so neither doll is closed;
  - `outside` 0.5;
  - `unturned` 0.25 (the third doll is in the second; the faces are off the aprons);
  - `hold` 0.0 (the first doll's top is held on its spigot and never let go);
  - `skip_inner` 0.0.
- Dream-1 run (2,400 steps): 0.0. Its arms stayed near their start pose for most of the two minutes;
  near the end the right arm reached over the second doll's bottom half. No piece moved.
- The two arms stay at least 94 mm apart throughout the demo.

## Runtime and seeds

2,400 control steps (2 minutes at 20 Hz) per trial; the scripted demo finishes at step 1,818. One fixed layout: run
with `-E object_jitter=0`. There is no other randomness.

## Fidelity

- **Shape:** each half is a rigid lathed mesh (48 sides) of a classic matryoshka outline: a flat foot, a full belly, a
  waist at the split, shoulders, a neck and a round head. Each has a signed-distance collider at about 0.5 mm.
- **Walls:** 2.5 mm, with 4 mm floors.
- **Spigot:** 2 mm thick and 8 mm tall.
- **Mass:** from linden at 0.45 g/cm3.
- **Fit:** the skirt goes over the spigot with 1.5 mm to spare, and the last 4 mm of travel are a friction fit. It is
  modelled by a hidden ring inside each top's skirt, on a prismatic joint along the doll's axis. The ring lands on
  the spigot's rim; the top slides down over it only against the joint's drive, a damping drive (10^5 N s/m) capped
  at 4 N (first doll) or 3 N (second), which acts as Coulomb friction. A top lifted clear of its own spigot has its
  ring put back at the rest stop, as the wood springs back, so every attempt needs the press.
- **Not modelled:** the fit's grip on pulling a top off (a real snug top can lift its bottom with it), and the twist
  resistance of a seated top. The demo never lifts a doll by its top and turns each top before setting it on.
- **Force:** 4 N and 3 N are estimates for a firm hand press, not measurements. The real-world build below measures
  them; set `FIT_FORCE` in `task.py` to the measured values.
- **Third doll:** the third doll and the two inside it are one solid body.
- **Paint:** the painted face, shawl and apron are visual only.
- **Friction:** 0.5.

## Real-world build

- **Set:** a five-piece turned linden nesting doll set, the largest doll 150 mm tall and 71 mm across. For example the
  unfinished Spire "Nesting Doll Set 5 pc - 15 cm" (largest doll 5.9 in by 2.8 in, about $17;
  https://letsplayspire.com/products/nesting-doll-set-5-pc-15-cm). Paint each top's face and each bottom's apron, or
  mark their middles with a paint pen.
- **Fit force (measure once per set):** stand a bottom half on a kitchen scale, set its top on square, and press the
  top down slowly by hand until it stops on the shoulder; note the scale's peak reading (grams times 0.0098 is
  newtons). Do it three times per doll and use the mean as `FIT_FORCE`. Also note how far proud the top stands when
  only set on (`FIT_TRAVEL`, 4 mm here). A set whose tops drop home by their own weight is too loose for this task:
  use another set.
- **Fits to check:**
  - each top presses home on its own bottom by hand;
  - the second doll, closed, drops into the first doll's bottom half;
  - the third doll, closed, drops into the second doll's.
- **Faces:** if the painting does not make it obvious, mark the middle of each face and each apron with a small dot
  of paint pen.
- **Layout:** stand the halves on printed outlines at the positions in `task.json`, the faces turned as marked.
- **Checks:**
  - a photo at the end from the camera side, to read each face against its apron with a 15 degree protractor card;
  - a photo from the side, to see that each top is pressed down on its shoulder: no gap under the rim (a 1 mm feeler
    card must not go in).
  - To check an inner doll, open the outer doll after the trial: the score is read from the video first, then the photo
    of the opened set confirms what is inside.
- **Hazards:** none. The pieces are smooth varnished wood and nothing sheds on the grippers.

## Licence

The task is released under the MIT licence. It uses no library or third-party assets. Every shape is generated in
`task.py` with `meshkit.py`, which is part of this package.
