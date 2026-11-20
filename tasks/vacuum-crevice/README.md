# Fit a cordless vacuum's crevice tool and vacuum the grit out of a narrow gap

A cordless vacuum's main body (a Dyson V8 Cyclone used as a handheld, no wand) stands on a wooden block. Its
quick-release crevice tool lies in a wooden cradle in front of the inlet. To the right, a gap 30 mm wide and 100 mm deep
runs between two boards, like the gap between a car seat and the centre console, and 32 g of 2-3 mm grit lies along
its floor. The robot must fit the crevice tool, switch the vacuum on and vacuum the grit out of the gap.

Getting crumbs and grit out of a gap the floor head and fingers can't reach is what a crevice tool is for, and most
households with a cordless vacuum do it (car seats, sofa cushions, the gap beside a cooker). The difficulty is physical.
The tool's collar has to go into the inlet square until its catch clicks; Dyson's manual shows tools pushed on until
they "click". The machine is heavy (about 1.7 kg) and long: with the tool on, its tip is 44 cm from the handle. Held
by its handle in a two-finger grip, it sags and turns. The gap is too deep for fingers and too narrow for the bare
inlet, so the tool has to go in tilted, reach the floor 10 cm down, and be drawn along both walls of the gap from end
to end without catching on the boards.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the power switch, the catch, suction, `success()`, `score()`, `self_test()` and the scripted `demo()` |
| `partkit.py` | Rigid parts built from boxes and cylinders, with spring-loaded sliders (the power button and the catch's tooth) |
| `soilkit.py` | The grit: small rigid spheres authored straight into USD, read back through one PhysX view every control step |
| `scenekit.py` | Fixed shapes for yam_arena task files (hooks yam_arena without changing it) |
| `tiltkit.py` | For the scripted demo only: tilted grips, joint-space moves and payload sag compensation on yam_arena's waypoints |

All shapes are built in code; there are no asset files and no third-party assets. The grit starts from a jittered
lattice and settles in the first second.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top.

| Object | Where | What |
|---|---|---|
| Vacuum | standing on the block, inlet pointing forward (+x); its inlet's front end at (49.6, -6.0), the handle's middle at (25.0, -6.0), 15.9 cm up | Dyson V8 Cyclone (model SV55) main body without wand or tool: 32 cm long, about 1.7 kg; inlet axis 9.2 cm above its base; red power button on top of the body (press once: on; again: off), where the earlier Dyson V8 had a trigger. Switched off |
| Block | centred (29.6, -6.0) | Wooden block, 28 x 14 x 9 cm (L x W x H) |
| Crevice tool | in the cradle, pointing forward along the inlet's axis (18.8 cm up), its collar's open end at (52.6, -6.0), 3.0 cm in front of the inlet; tip at x 75.6 | Dyson quick-release crevice tool (part 967612-01): collar 35 mm across, 55 mm long; blade 175 mm long, 17.2 x 24.7 mm, its thin side across (y); 23 cm long, about 40 g |
| Cradle | under the tool | Three wooden pieces: a post under the collar's front 14 mm (x 56.6-58.0) with a notch the collar sits in (17.0 cm up, side cheeks 39 mm apart); a post under the blade (x 67.6-70.6, notch 17.6 cm up, cheeks 21 mm apart); a stop 1 mm in front of the tip (x 75.7-76.7, 20 cm tall) |
| Gap | inner faces at y -21.0 and -24.0, from x 50.0 to 76.0 | Two plywood boards, 296 x 18 x 100 mm, standing on the table 30 mm apart, centred on x 63.0; closed at the far end by an 18 x 30 x 100 mm block (x 76.0-77.8); at the near end only a 6 x 30 x 6 mm strip (x 49.4-50.0). Clamped down |
| Grit | along the gap's floor, x 50.2-75.8 | 32 g of rounded quartz aquarium gravel, 2-3 mm (Olibetta Crystal Quartz 2-3 mm), spread evenly in a layer 2-3 grains deep |

Sources: the tool's reach and size, Dyson ("reaches up to 175 mm into tight, narrow spaces as small as 17.2 mm",
dyson.com support page for 967612-01; 17.3 mm width, Spares2Repair); its 35 mm collar, vhbw replacement listings
(ManoMano); tools clicking onto the inlet and the power button, Dyson's V8 Cyclone user manual
(p. 3, "Operating your machine": "The top button turns your vacuum on and off, while the lower button changes power
mode"; "the power button is red and closer to the post-motor filter"); the
body's 32 cm length, Smart Home Fox's V8 review ("32 cm plus attachment"); the other body sizes, scaled from the
manual's side drawing to that length; mass, the V8 handheld's 1.58 kg (Dyson specification, TechRadar) plus the 0.16 kg
the V8 Cyclone weighs more as a whole machine (2.70 kg, dyson.co.uk, against the V8's 2.54 kg, dyson.ie); quartz
density 2.65 g/cm3; the suction range, CleanControlling (suction inlets work 2-6 mm from the surface).

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the crevice tool to the vacuum, switch it on and vacuum the grit out of the gap." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/vacuum-crevice-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. A trial takes
about 6 minutes on one RTX 4080 (scene start-up included). Seed 0; the scene has no random placement
(`-E object_jitter=0`). `VAC_LOG=1` prints the vacuum's state every half second.

Offline checks (no GPU): `python task.py` runs the scoring rules on 16 hand-built states and the suction and fit
checks, and replays the demo's plan through the arms' kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Fit the crevice tool to the vacuum, switch it on and vacuum the grit out of the gap." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/vacuum-crevice-demo
```

The left arm leans over from its side and presses the power button with its closed fingertips; the vacuum starts.
The right arm comes in behind the handle from the side, grips it, lifts the machine 6 mm off the block and pushes it
forward onto the tool's collar until the catch's tooth drops into the collar's groove, then backs off 4 mm and lifts
the tool out of its cradle. It tilts the machine about 40 degrees nose-down over the middle of the gap, puts the tip down to
the floor, and draws it along the right-hand board to the near end and forward to the far end, then along the
left-hand board back and forth. 85 % of the grit is vacuumed at about 50 s and the trial ends with 1.0.

The demo plays yam_arena's waypoints through `tiltkit.py`: a tilted grip, joint-space moves where the arm changes
configuration, and an aim that allows for the arm's sag under the 1.8 kg load. The YAM's joints are soft PD drives
that compensate their own weight only, so the plan commands each joint past its target by the load's torque over the
joint's stiffness, and aims the hand 2 mm to the right of where the geometry puts it (measured in these runs).

A wrong strategy, switching the vacuum on and vacuuming without fitting the tool, scores 0: the bare inlet (36 mm)
can't get into the 30 mm gap, and the score needs the tool clicked on.

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`). Tools for a real trial: a kitchen scale (0.1 g) and
a cup.

- **Tool on:** at the end the crevice tool is pushed home in the inlet with its catch engaged: it does not come off when
  pulled by hand without pressing its release button. In simulation: the collar's open end within 2.5 mm of the
  socket's floor (the groove gives 2 mm of play), square to the inlet within 3 degrees, and the catch's tooth down.
- **Vacuumed:** the share of the grit that ended up in the vacuum's bin. On a real trial, empty the bin before the
  trial; afterwards empty it into the cup and weigh the grit, out of 32 g. Grit pushed out of the gap, or picked up
  any other way, does not count. In simulation: by mass (the grains are of one density, 2-3 mm), the grains the suction took.

| Outcome | Score |
|---|---|
| Tool not on at the end, or under 25 % vacuumed | 0 |
| Tool on, 25 % vacuumed or more | 0.25 |
| Tool on, 50 % or more | 0.5 |
| Tool on, 75 % or more | 0.75 |
| Tool on, 85 % or more, held for 1 s | 1.0 |

## Reset

Switch the vacuum off, press the tool's release button and pull the tool off, empty the bin into the gravel's jar, and
stand the vacuum and the tool back as in the setup. Spread 32 g of the gravel evenly along the gap's floor (a funnel or
a spoon), and put back any that ended up outside the gap. The gravel is reusable.

## Real trials

The vacuum's handheld body weighs about 1.7 kg, within the YAM's 2 kg payload; pushing the tool on takes a few
newtons, and the power button is pressed once. The grit is clean, dry, rounded aquarium gravel; the vacuum is battery
powered and its bin closed. Wear eye protection near the gap in case grit is thrown.

## Simulation notes

Rigid-body PhysX at 240 Hz. The vacuum is one rigid body built from boxes and cylinders with two sprung sliders: the
power button (2 mm travel; it counts as pressed past 1 mm) and the catch's tooth, which rides over a rib on the collar
and drops into its groove. The catch's spring and the button's force are not published; they are modelled as light
(0.6 N and 5 N at full travel). The vacuum's centre of mass is not published either: it is taken over the handle, as
the machine balances there. The grit is 1,411 rigid spheres, 2-3 mm across. Suction is not simulated as airflow: while
the vacuum is on, a grain whose surface comes within 4 mm of the opening (the tool's while it is fitted, else the bare
inlet's) is taken into the machine.
