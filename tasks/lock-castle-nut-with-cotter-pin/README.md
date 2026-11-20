# Lock a castle nut with a cotter pin: push it through the slot and the stud's cross-hole, then spread its prongs

A stud stands upright in a steel plate with a slotted (castle) nut on it, one slot lined up over the stud's
cross-hole. A new cotter pin lies in a groove on an aluminium block, and a precision flat-head screwdriver stands in a
rack. The robot must push the pin through the nut's slot and the stud's hole and spread its two prongs apart so the
pin can't come out.

Automotive service technicians (about 826,000 US jobs; BLS *Occupational Outlook Handbook*, 2025) do this whenever
they rebuild steering linkages (O*NET 49-3023.00, task: "Tear down, repair, and rebuild faulty
assemblies, such as power systems, steering systems, and linkages."). Service manuals call for a new pin each time,
bent once the nut is tightened to the next slot (Suzuki RA410 service manual: "bend new split pin"; Meritor bulletin:
tighten to the next slot, never back the nut off to line up the hole).

The hard part is physical. The pin (2.2 mm across) has to go in square through a 2.8 mm hole that is hidden under the
nut's slot. Its prongs are 0.75 mm thick and stand 0.7 mm apart: a parallel gripper can't take hold of one, nor
squeeze it hard enough to bend it; a thin blade has to go down between them and lever each one round past its yield
(about 6 N, 8 mm from its root). While the prongs are levered, the pin's head has to be held home, or the pin turns
and slides in the slot.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the released watch, `success()`, `score()`, `self_test()` and the scripted `demo()` (with its live aim at the gap) |
| `cotterkit.py` | The prongs' plastic bending: each prong's hinge keeps its bend once pushed past the prong's plastic moment |
| `scenekit.py` | USD bodies and fixed shapes for yam_arena task files (hooks yam_arena without changing it) |
| `touch.py` | Robot-to-body contacts from PhysX (helper shared with other tasks): whether the robot still touches the pin |
| `assets/fixture.usda` (16 KB) | Block, steel plate, 3/8-24 stud with its 2.8 mm cross-hole, 3/8-24 slotted hex nut |
| `assets/cotter.usda` (16 KB) | The cotter pin: eye and shank, two prongs on hinges at their roots |
| `assets/screwdriver.usda` (5 KB) | The precision screwdriver: thin straight tip, tapering blade, handle |

The three assets are built from simple solids (sizes as below). No third-party assets.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top.

| Object | Where | What |
|---|---|---|
| Fixture | stud at (30.0, 0.0) | A steel block 5 x 4 x 3 cm (x 26.5 to 31.5) carrying a steel plate 6 x 4 x 1 cm (x 26.0 to 32.0, top 4.0 cm up); an upright 3/8-24 steel stud (9.5 mm) through the plate |
| Castle nut | on the plate, round the stud | Zinc-plated 3/8-24 slotted hex nut: 9/16 in (14.3 mm) across flats, flats facing front and back, 8.3 mm thick, six slots 3.0 mm wide and 3.2 mm deep; one slot runs front to back over the stud's 2.8 mm cross-hole (the hole's centre 46.5 mm up) |
| Cotter pin | in the block's groove, prongs pointing at the robot | A 1/16 x 1 in low-carbon steel cotter pin (ASME B18.8.1): half-round prongs 0.75 x 1.5 mm, 25.4 mm from under the eye to the short prong's end, the other prong 1 mm longer, eye 5 mm across, about 1.2 g; its prongs opened 0.7 mm apart along their length |
| Block | x 24.6 to 30.6, centred on y -12.0 | Aluminium, 6 x 2 x 2 cm, a groove 3.0 mm wide and 1.0 mm deep along its top; the pin's eye and 8 mm of shank stick out past its far end (the eye's centre at (31.7, -12.0)) |
| Screwdriver | standing tip down in the rack | Precision flat-head, 120 mm long, 30 g: a steel blade 2.5 x 0.4 mm for its first 8 mm, widening evenly to 4 x 3 mm 40 mm up, then a red plastic handle 18 mm square and 80 mm long |
| Rack | centred at (22.0, 18.0) | Grey plastic block 3 x 3 cm, 2.8 cm tall, with a 4 mm square hole |

Sources: workers, BLS OOH, *Automotive Service Technicians and Mechanics*
(bls.gov/ooh/installation-maintenance-and-repair/automotive-service-technicians-and-mechanics.htm); task line, O*NET
49-3023.00 (onetonline.org/link/details/49-3023.00); practice, Meritor bulletin (static.nhtsa.gov/odi/tsbs/2015/
MC-10110472-9999.pdf) and Suzuki RA410 service manual; pin sizes and material (low-carbon steel 1005-1010), ASME
B18.8.1-1994 (table at fastenersuperstore.com/product_specs/pins-cotter); the prongs' plastic moment, 45 N mm, is our
own calculation for a half-round 0.75 x 1.5 mm prong at a 300 MPa yield.

## Run a trial

From this folder, in a shell with the yam_arena environment active:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lock the castle nut with the cotter pin: push it through the nut's slot and the stud's hole, then spread its prongs apart." \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cotter-dream
```

`task_score` prints the trial's score: 1.0 on success (held for 1 s), otherwise `score()` at the end of the episode.
A trial takes about 3 minutes on one RTX 4080 (scene start-up included). Seed 0; the scene has no random placement
(`-E object_jitter=0`).

Offline checks (no GPU): `python task.py` runs the scoring rules on 16 hand-built states and the plastic-bending rule
on 3, and replays the demo's plan through the arms' kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Lock the castle nut with the cotter pin: push it through the nut's slot and the stud's hole, then spread its prongs apart." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/cotter-demo
```

The right arm picks the pin by its eye where it sticks out of the block, carries it round to the nut's far side,
slides it along the slot through the stud's hole, lets go just short of home and pushes the eye home with its closed
fingertips; it keeps pressing it there while the left arm works. The left arm takes the screwdriver out of its rack
and lays it down to 25 degrees, tip forwards. It puts the blade down into the gap between the prongs by their roots,
levers the +y prong round about its root, and pushes it on round from beyond its tip; then it goes in again by the -y
prong and levers and pushes that one the other way. The demo reads where the pin sits and where the screwdriver's tip
is, and trims its aim at the gap while it holds still over it and goes down (the gap leaves 0.15 mm a side). The
prongs end 64 and 35 degrees out (99 degrees apart), the eye home; the right arm lets go, and the trial reaches 1.0
at about 72 s (1,450 steps). Four runs of the demo in a row all scored 1.0.

A wrong strategy, pushing the pin through and home and leaving its prongs straight, scores 0.25 (through, the prongs
0 degrees apart).

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`). Tools for a real trial: a phone camera held
straight above the nut, a protractor and a 1 mm feeler gauge.

- **Through:** the pin passes through the stud's cross-hole along the slot (its axis within 15 degrees of the slot's,
  within 1.5 mm of the hole's axis), the prongs' roots out past the stud.
- **Prong angle:** in the photo, the angle between the pin's axis and a line from a prong's root (where it leaves the
  nut) to its tip, counted outwards. The prongs are that far apart in all: the sum of the two. A prong bent inwards
  counts negative. In simulation: each prong's hinge angle.
- **Home:** the feeler gauge does not go in between the pin's eye and the nut. In simulation: the eye within 1 mm of
  the nut.
- **Released:** the robot does not touch the pin; the screwdriver's tip is more than 2 cm from the prongs' roots.

| Outcome | Score |
|---|---|
| The pin not through the cross-hole | 0 |
| Through, the prongs less than 45 degrees apart | 0.25 |
| At least 45 but less than 90 degrees apart, or one prong less than 30 degrees out | 0.5 |
| At least 90 degrees apart, each prong at least 30 degrees out, but the eye not home, the robot touching the pin, or the screwdriver's tip within 2 cm of the prongs' roots | 0.75 |
| At least 90 degrees apart, each prong at least 30 degrees out, the eye home, the robot off the pin and the screwdriver away, held for 1 s | 1.0 |

Spread 90 degrees apart, the prongs' tips stand well over 3 mm apart, wider than the slot and the hole, so the pin
can't be pulled back out; each prong at least 30 degrees out asks for both to be spread, not one.

## Reset

A spread cotter pin is not reused: take a new straight pin, open its prongs 0.7 mm apart with a 0.7 mm shim pressed
between them, and lay it in the block's groove as in the setup. Stand the screwdriver back in its rack.

## Real trials

Bending a prong takes about 6 N, 8 mm from its root, well within the arms' strength (about 40 N per finger). The
screwdriver is a precision one with a blunt 2.5 mm edge; nothing is hot, powered or under load. The fixture is clamped
to the table.
