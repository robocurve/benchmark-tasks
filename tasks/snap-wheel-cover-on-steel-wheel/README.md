# Fit a plastic wheel cover onto a car's steel wheel with its valve notch over the valve stem

A 13-inch steel wheel with its 155/80 R13 tyre stands on its hub as it does on a parked car: upright, axle 27.2 cm above the ground,
its outboard face towards the robot, under the wheel arch of the car's body side. The rubber valve stem is at the top of the rim. The
13-inch plastic wheel cover (hubcap) lies face up on the ground in front of the wheel, where it was put down when it came off, with its
valve notch pointing away from the robot and to its right. The instruction is only the goal: "Put the wheel cover on the wheel."

Doing it right takes three things a person does without thinking:

- **Bring the notch to the stem.** The cover has to be lifted, stood up to face the wheel and turned until its notch is over the stem.
  Pushed on anywhere else, the cover lands on the stem: it will not clip on at that side, and pushing harder crushes or tears the stem.
- **Push it on square and round the rim** until every clip has snapped in behind the bead seat's lip.
- **Let go.** Full credit needs both grippers clear of the cover.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the wheel and tyre, the car's body round the arch, the cover, the scorer (`score`, `success`) and the closed-loop scripted demo (`demo_script`) |
| `demokit.py` | Runs `demo_script`, unchanged, from the open-source kit in `tasks/heat-shrink-lug/` |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), unchanged, from the open-source kit in `tasks/heat-shrink-lug/` |
| `simready.py` | Loads the SimReady distractors, unchanged |
| `task.json` | The task's details for the board |

## Response to review

The review asked for the real installed context, a full-score demo in it, a clip scorer that requires each clip to be seated on the
wheel, and a check that a real cover can be seated within the robot's force. What changed:

**1. A tyre-equipped wheel, mounted as on the car.** The bare rim lying flat on the table is gone. The wheel now has its tyre (155/80 R13,
57.8 cm across, 15.5 cm wide, its footprint flattened where it stands), stands upright on a brake drum with four wheel nuts, and has its
axle 27.2 cm above the ground, the tyre's loaded radius, so the cover is fitted on a vertical face at the height it is on a car. Round it
is the car's body side: a vertical steel panel 2.5 cm inboard of the tyre's sidewall, 1.2 m long and up to 72 cm high, with the arch
opening (34.5 cm radius about the axle, 5.6 cm over the tyre), a rolled lip, the arch's plastic liner over the tyre and the sills below.
The body leaves the wheel's face open to the robot, as on a car, and blocks access from above and from behind, so the cover can only be
offered up face-on, the way a person kneeling at the wheel does it. Because a cover cannot rest on an upright wheel, it now starts on the
ground in front of the wheel, where it is put when it comes off, and has to be picked up, stood up and turned before it is pushed on.

What differs from a car on a driveway, and why:

- The ground is the surface the arms stand on (the table top). The arms are fixed to it, so a real car's ground cannot be lower than the
  arm bases; the wheel's height is real relative to the arms' ground, as if the robot stood on the driveway beside the wheel. For a real
  trial, an equivalent fixture reproduces this: a 13 x 4.5J steel wheel and its tyre bolted to a hub stand (or a scrap drum and hub) on the
  robot's bench with the axle 27.2 cm up, and a sheet of plywood cut to the arch's outline at the stated offset as the body side.
- The body is drawn only round the arch, and flat; the rest of the car is out of reach and out of view.
- The wheel is held still, as a parked car's wheel is by its brakes and its tyre's contact.
- The fixed top camera looks down at 45 degrees from behind the arms, so with the wheel at real height it sees the cover on the ground and
  the wheel's lower half; the face of the wheel is in the wrist cameras' view. The demo and model videos are the top camera's.

**2. A full-score demo in that setup.** `demo_script` (closed loop on the cover's observed pose, never the scorer's state) grips two spokes
through the slots, lifts the cover, turns it flat until its notch points away from the robot, stands it up to face the wheel (the notch
comes up to the top, over the stem), squares its face to the wheel and its notch to the stem from what it sees, carries it onto the axle
and pushes it on with both arms, lets go and backs off, then presses the rim in over each clip in mirrored pairs and moves away. It scored
1.0 in each of its final runs (below).

**3. The clip scorer now requires each clip to be seated on the wheel.** Before, a clip counted when the cover's lip beside it was near
the flange's height and the clip's slider had sprung back near its rest. Now every check is made in the wheel's frame, from each clip's
catching face as it actually is (its slider's travel included): the face must be hooked behind the bead seat's lip, at least 1.0 mm out
past the lip's inner face and behind the lip's back edge (within 0.5 mm, where it bears on it), and the cover's lip beside the clip must
be within 1.0 mm of the flange. A cover that is off-centre, cocked, held in front of the wheel or anywhere other than on it seats no
clips. Two related changes: the stem only counts as through the notch when its cap is inside the cover's depth (not merely lined up with
the notch from a distance), and the 1.0 is no longer latched: success needs every condition at once for 2 s, and the trial ends there.

**4. The force check.** The simulated retaining ring is eight separate clips of 1,500 N/m. Pushing the cover on in the demo took about
110 N at peak from both arms together (43 N left, 65 N right, read from the arms' drive position errors while the clips cammed over the lip, in a debugging run of the demo with physics on the CPU).
At that pose, the arms' joint effort limits (27 N m on the shoulder joints, the motors' peak torque) allow about 100 N (left) and 120 N
(right) of push along the wheel's axis; at the motors' continuous rating about a third of that. We could find no published figure for
the force needed to push a real cover on: wheel-cover patents discuss retention and mounting force without values, and fitting guides
say to push it on by hand round the rim. We have not measured a real cover, so we cannot confirm that a real one seats within the robot's
force. For a real trial, measure it first: lay the real wheel and tyre flat on a bathroom scale, zero it, and push the cover on by hand
pressing round the rim as the demo does; the highest reading while a clip snaps in is the force a press needs. If it is over about 100 N
per press (one arm's peak push) the real trial is not feasible with this robot, and pressing one clip at a time (as the demo's press
round does) is the way to keep each press within it.

Unchanged: the instruction, the cover's geometry, the eight clips and the stem.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena (`source ~/IsaacLab-Arena/yam.env`).
- Network access the first time the scene is built: `simready.py` downloads the SimReady packages into `~/.cache/simready`.

## Run and score a trial

From the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/snap-wheel-cover-on-steel-wheel/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put the wheel cover on the wheel." --scorer task_score --max-steps 2400 \
  --store-frames --log-dir ~/yam-eval/snap-wheel-cover-on-steel-wheel/dream
```

The scripted demo (`demo_script`, closed loop), from the task's folder:

```bash
cd tasks/snap-wheel-cover-on-steel-wheel
python demokit.py task.py ~/yam-eval/snap-wheel-cover-on-steel-wheel/demo          # final render: OUT/full/demo.mp4, setup.png, result.json
python demokit.py task.py OUT --fast --mode full,none,noturn,onearm                  # CPU physics, no cameras: the demo and the failure checks
```

`task_score` prints the score (demokit writes it to `OUT/<mode>/result.json`). Nothing is randomised, so every seed gives the same scene.
Runtime: the demo takes about 47 s of simulated time (about 950 steps) and ends on success; a 2,400-step trial is 2 minutes of simulated
time, which took several minutes of wall time on a shared GPU.

## The scene

- **Wheel** (`wheel`, a fixed mechanism): a 13 x 4.5J steel wheel, drawn as a revolved 3 mm steel section (flange, bead seats, drop
  centre, centre disc with four stud holes and four vent holes), on a brake drum with four wheel nuts, with a 155/80 R13 tyre. The rim
  collides through rings of 40 convex segments: the flange, where the cover's lip seats; the outboard bead seat's inner face, the lip the
  clips ride over (2.2 cm deep); the groove behind it, where the clips catch; and the well's shoulder. The tyre collides through 40 convex
  segments of its section.
- **Valve stem**: the wheel's one link, a rubber TR414-class snap-in stem at the top of the rim, on a hinge at its base about the rim's
  tangent, with a spring for the rubber's bending stiffness (0.5 N m/rad).
- **Car** (`car`, fixed): the body side round the arch, the arch's liner and the sills, as above.
- **Cover** (`cover`, a free mechanism, 0.45 kg): a dished ABS cover 36.0 cm across with a 3.35 cm valve notch and eight through slots
  between eight spokes, colliding through convex pieces (the notch and the slots are real gaps). Its eight retaining clips are links:
  each a sprung slider with a ramped head. Pushed onto the rim, the head's ramp meets the bead seat's lip and cams the clip in; further on
  the head clears the lip and springs out into the groove, where its flat face catches behind the lip with 0.5 mm of play.
- **Distractors**: a cleaning cloth and a pair of light-duty gloves, loose on the ground beside the work, ignored by the scorer.

## Scoring

Each control step (20 Hz), the scorer reads the cover's pose, its clips' joints, the stem's angle and the grippers' fingertip points, and
works in the wheel's frame:

- a clip is **seated** when its catching face is hooked behind the bead seat's lip (at least 1.0 mm out past the lip's inner face and
  behind its back edge, within 0.5 mm) and the cover's lip beside it is within 1.0 mm of the flange;
- the stem is **through the notch** when its cap is within 1.1 cm of the notch's centre line, between 12.5 and 18.0 cm from the cover's
  centre, and within the cover's depth (3.0 cm behind to 2.0 cm in front of its lip);
- **hands clear** means every fingertip point of both grippers (each finger's tip and 3 cm up it) is at least 3 cm from the cover.

| Outcome | Score |
|---|---|
| Stem through the notch, all eight clips seated, hands clear, all held 2 s | 1.0 (the trial ends) |
| Otherwise at the end, stem through the notch | 0.2 + 0.075 per seated clip (0.2 to 0.8) |
| Stem not through the notch | 0 |
| Stem bent more than 15 degrees from its rest at any time | 0 for the whole trial |

The bent-stem record is cleared only by the kit's reset hook, never by the scene looking right again.

In a real trial: the lip-to-flange gap is checked with a 1 mm feeler gauge beside each clip; a clip is hooked when the cover's edge
beside it does not come away from the flange when pulled outward by hand with light force (about 10 N on a luggage scale hooked through
the slot beside it); the stem's position in the notch is read with a ruler, and a bent stem with a protractor on a photo.

## The demo

Both arms take a spoke through the slots on either side of the cover, the left hand at the robot's left and the right hand on the spoke
nearest it on its own side (the right wrist's turning range rules out the spoke opposite the left hand's), and lift the cover. They
turn it flat until the notch points away from the robot, then stand it up, turning both grips about a line across the cover while it
rises to the axle's height, so it faces the wheel with the notch at the top. From the cover's observed pose they square its face to the
wheel and the notch to the stem, carry it onto the axle 7 cm out, then 3 cm out, and push it on with both arms. They let go, back off,
press the rim in beside each clip in mirrored pairs with closed fingers, and move away.

| Run | Score |
|---|---|
| Scripted demo, final run 1 | 1.0 (success held 2 s at step 942) |
| Scripted demo, final run 2 | 1.0 (success held 2 s at step 942) |
| Red team: do nothing (`--mode none`) | 0 (the cover stays on the ground) |
| Red team: stand it up and push it on without turning it (`--mode noturn`) | 0 (the notch is 67.5 degrees round from the stem; the cover lands on the stem, will not clip on and falls off) |
| Red team: one arm only, lifting the cover by one spoke, standing it up and pushing it on (`--mode onearm`) | 0 (the stem is never through the notch and no clip seats) |

In its Dream-1 run (2,400 steps) the model reached for the cover on the ground with its right gripper and pushed and turned it about
there, but never lifted it, so the stem was never through the notch and the run scored 0.

## Fidelity

Rigid PhysX bodies at 240 Hz. No SimReady wheel, tyre, car body or hubcap exists, so all are custom meshes; the wheel is held still, as
on a parked car. The ABS cover is rigid, where a real one flexes. Its steel retaining ring is modelled as eight separate sprung clips,
1,500 N/m each; pushing it on took about 110 N at peak from both arms. A real cover's push-on force is not known to us (see the force
check above). The stem is a sprung hinge. Friction is uniform; there is no click or sound, and no force sensing.

## Assets

The distractors are from NVIDIA's SimReady library,
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`cleaning_cloth_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_cloth_a01)
- [`ppe_gloves_lightDuty_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/ppe_gloves_lightDuty_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses set (cloth
50 g, gloves 60 g); any joints welded into one rigid body. `simready.py` downloads them into `~/.cache/simready` the first time the scene
is built. Nothing large is committed.

`mechkit.py` and `demokit.py` are the open-source kit shared by several tasks in this repository (copied unchanged from
`tasks/heat-shrink-lug/`).

## Importance and sources

Steel wheels with clip-on plastic covers are standard on base trims of small cars, and covers come off for every tyre change, rotation
and puncture repair, then go back on with the wheel on the car. Refitting one is a short, everyday job that needs two different skills in
order: an alignment the cover's own shape dictates (the notch must meet the stem, or pushing on breaks things), then a snap fit seated
all round, on a vertical face under the car's body. Fitting guides say the same:

- iFixit, "2002 Nissan Altima Hubcap Replacement": align the retention ring's divot with the tyre-valve cutout, align the hubcap's
  cutout with the tyre valve, then push in all the sides until it secures:
  https://www.ifixit.com/Guide/2002+Nissan+Altima+Hubcap+Replacement/172847
- Ring Automotive, 13" Vegas wheel trims (RWT1333): a current 13-inch high-impact ABS wheel trim held by an adjustable retaining ring, the
  kind of cover modelled here: https://ringautomotive.com/en/product/RWT1333
- Tesla, Model 3 service documentation, "Removing and Installing Wheel Covers": a manufacturer's procedure for a clip-on cover, which
  starts "Align the wheel cover with the valve stem", then presses it until secured, and warns to make sure it is fully secured before
  driving so it does not fall off:
  https://service.tesla.com/docs/Public/diy/model3/en_us/GUID-377C88DB-7FBE-4320-BC1A-0A01AF48005C.html
- The tyre's size and loaded radius: 155/80 R13 is 15.5 cm wide with a sidewall 80% of that (12.4 cm), on a 13-inch (33.0 cm) bead
  seat, 57.8 cm across; its static loaded radius, about 27 cm, sets the axle's height.
