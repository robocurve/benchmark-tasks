# Install two M3 heat-set inserts in a 3D-printed robot part

A 3D-printed PLA robot link plate lies on the table with a brass M3 heat-set insert sitting loose in the mouth of each of
its two pilot holes. A soldering iron with an M3 heat-set tip rests on its stand, and a steel 1-2-3 block stands beside the
plate. The robot must install both inserts the way CNC Kitchen's guide describes: melt each one most of the way in with
the iron, keeping the iron square to the plate and pressing lightly, then press the last part with a flat tool (the
block) while the plastic is still soft, so the insert ends flush and square. It then lays the hot iron back on its
stand.

A heat-set insert is a small brass sleeve, knurled outside and threaded inside (M3 here). Pressed into a hole in a 3D-printed plastic part with a soldering iron, it heats, melts the plastic round it and sinks in; the plastic sets again in its knurls and holds it, so the part has a metal thread that survives being screwed and unscrewed many times, where a thread cut in the plastic would strip. Open-source robot arms are built this way: the Instructables "Giant 3D Printed Robot Arm" joins most of its parts with heat-set inserts and machine screws, set with a soldering iron at 230 C; USC's SPRITE robot, the RBX1 arm and the PAROL6 desktop arm include the same step in their build guides. CNC Kitchen's guide ("Tips & Tricks for Heat-Set Inserts used in 3D printing") gives the method used here: iron 10-20 C above the print temperature (225 C for PLA), melt the insert to about 90 % of its depth, finish with a flat tool. The difficulty is physical: the iron must come down on a 4.4 mm insert and stay vertical while it sinks
it with light pressure (a tilted iron sets it crooked), stop short of flush (the tip's shoulder melts the plastic round
the hole if it goes past), the iron has to stay pressed on each insert for a few seconds before it gives, and the
block must reach the insert while it is still soft and come down flat on the plate, which is what leaves it flush.
All of it with a 225 C iron over a shared rig: its hot metal may touch only the inserts, the plate and its stand.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the heating rule, `score()`, `success()`, `self_test()` and the scripted two-arm `demo()` |
| `contactkit.py` | Reads the PhysX contact forces between the iron or the block and each insert or the plate |
| `touch.py` | Reads whether, and where, any robot link touches the iron (for "let go" and the hot-metal rule) |
| `scenekit.py` | USD bodies and fixed shapes for yam_arena task files (hooks yam_arena without changing it) |
| `tiltkit.py` | Any gripper orientation for the scripted demo (the iron is held tip-down from the side) |
| `assets/plate.usdc` | The plate (30 x 60 x 10 mm) with its two inserts, each on a prismatic joint, a fixed-base articulation, 277 KB |
| `assets/plate.stl` | The plate with its holes, for printing, 605 KB |
| `assets/iron.usdc` | The soldering iron with its heat-set tip, a free body, 26 KB |
| `assets/block.usdc` | The 1-2-3 block, a free body, 1 KB |
| `emulator/` | The no-heat emulator plate's firmware (`heatset_emulator.ino`, Arduino), the heating rule it shares with `task.py` (`rule.h`) and its test (`test_rule.cpp`) |

No third-party assets; `work/heatset/make_parts.py` builds all four from the dimensions below.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
work surface (the silicone mat's top). Same layout every trial. Both arms start in their ready pose, grippers open.

| Object | Where | What |
|---|---|---|
| Silicone mat | covering the work area: 32 x 40 cm, centred at (30.0, -4.0), long side along y | Silicone soldering mat, 1 mm thick, rated for continuous use at 500 C (the kind sold as a "soldering mat"); the plate, the iron's stand and the block all stand on it, and nothing flammable lies within the arms' reach |
| Plate | centred at (33.0, -3.0), lying flat, its 60 mm side along y, held still (taped to the table) | 3D-printed PLA plate, 30 x 60 x 10 mm (`assets/plate.stl`), with two blind pilot holes 4.0 mm across and 8 mm deep at (33.0, -4.5) and (33.0, -1.5) (the M3 pilot hole from Accu-Components' hole chart) |
| Inserts | one in each hole's mouth, set 0.8 mm in, upright | Brass M3 heat-set inserts, 5.7 mm long (McMaster-Carr 94459A140, M3 x 5.7 mm installed length), 4.4 mm across (Accu-Components, non-flanged M3) |
| Soldering iron | lying on its stand along x, tip towards the robot at (20.0, -17.0), its axis 4.0 cm above the mat; switched on at 225 C | Pen-style soldering iron with an M3 heat-set insert tip (a 5 mm shoulder over a pilot pin), about 175 mm long, 45 g: its bare metal (pin, shoulder and a 6 mm shaft) is the first 71 mm from the tip; its grip, 15 mm across, from 71 to 176 mm, stays cool |
| Iron stand | under the iron, on the mat | Steel, two supports, each a bar with a post either side: one under the shaft 3.5 cm from the tip, one under the grip 16 cm from the tip |
| Station and cord | the station off the table, on a shelf behind the right arm's base; the cord overhead | Temperature-controlled soldering station (for example a Hakko FX-888D) set to 225 C, its mains through the emergency stop's contactor (below). The iron's 1.2 m heat-resistant silicone cord leaves the iron's rear and runs up to a spring cable balancer hung 70 cm above the stand (as welding torch cables are hung), then to the station: it never lies on the work surface or crosses below the tip. Not in the simulation |
| 1-2-3 block | standing on its 25.4 x 50.8 mm end at (25.0, 10.0), its 50.8 mm side along x | Hardened steel 1-2-3 block, 25.4 x 50.8 x 76.2 mm, 0.77 kg |

## Real-trial heat safety

The default real trial uses the real, hot iron (the skill is the heating). The robot's rig, the work surface and the
people round it are protected as follows.

- **Work surface:** the silicone mat above (rated 500 C) under everything the iron can reach; the iron's stand is
  steel; nothing flammable (paper, cloth, solvent) within the arms' reach.
- **Grippers:** all four finger pads wear heat-resistant sleeves rated to 260 C or more (high-temperature silicone or
  aramid), and both arms' wrists, camera mounts and forearms aluminised fibreglass sleeves. Each sleeve carries an irreversible 60 C
  temperature-indicating label, read after every trial; after each trial an IR thermometer also reads each pad under
  its sleeve: 45 C or less.
- **Where the hand holds the iron:** by its grip only, 95 mm from the tip; the grip starts 71 mm from the tip and stays
  cool, so the pads' near edges are about 11 mm of plastic grip away from the bare metal, which points down, away from
  the hand.
- **Power:** one emergency-stop button beside the operator cuts both the robots' power and the soldering station's
  mains (through a contactor or a switched strip on the same circuit).
- **Cord:** hung from the spring balancer above the stand, running straight up from the iron to it and on to the
  station, never along an arm or across the mat.
- **People and fire:** an operator stands at the emergency stop for the whole trial, outside the arms' reach, and the
  robot is never left running unattended with the iron on; a CO2 or dry-powder extinguisher and a burn first-aid kit
  stand at the bench. The run is stopped at once if the iron is dropped, leaves the mat or touches anything but the
  inserts, the plate and its stand.
- **After the session:** the station is switched off and the iron cools on its stand until its shaft reads under
  50 C (IR thermometer; about 10 minutes) before anyone moves it.

**Where a hot iron isn't allowed: the no-heat emulator plate.** As some accepted tasks run their real trials on an
inert stand-in that follows the simulation's own rule (an emptied lighter, a vented gas canister with a lamp), this
task can be run with no heat at all. The iron is unplugged and cold; a thin flexible wire runs along its cord to its
tip, and another to the steel block. The plate is printed with each hole bored through into a base under it, in which
the insert (a real brass insert on a 3 mm steel pin) slides in a printed bush, held by the pin of a 5 V spring-locked
solenoid that releases only when energised (so power off means locked); an O-ring on the pin gives it about 0.3 N of
friction, and its travel ends flush. An Arduino reads, through 5 V and pull-ups, whether the tip or the block touches
each insert, runs exactly this task's heating rule (`emulator/rule.h`, the same constants as `task.py`, tested by
`emulator/test_rule.cpp`) and releases an insert's brake, and lights a red LED by its hole, only while the rule says it
is soft; it logs every insert's temperature every 0.5 s. The base stands in a pocket of a raised work board so that
the plate's top is at the same height as in the setup. Everything else, including the scoring, is unchanged; the
safety list above then reduces to the emergency stop and the operator.

## Scoring

Check each insert after the trial with a straightedge and a 0.2 mm feeler gauge, a 30 mm M3 screw and a small
square, and a photo of the plate; read the sleeves' temperature labels; watch the trial video (a camera beside the
plate, at its height).

- **0** if the iron's hot metal (its first 71 mm from the tip) touches any part of the robot (a sleeve's label has
  turned, or the video shows it) or the work surface (the mat or the table), at any time.

- Each insert scores **0.5** when it is **flush** (a straightedge across it rests on the plate and a 0.2 mm feeler
  gauge does not pass under it, and it is not more than 0.2 mm below the surface), **square** (a 30 mm M3 screw
  threaded into it stands within 3 degrees of perpendicular: 1.6 mm at its head against a small square), and the PLA
  round it carries **no melt mark** from the iron's shoulder.
- **0.25** if it is at least half way in (2.45 mm or more below where it was set) but not flush, not square, or marked.
- **0** if it is less than half way in, or sunk more than 0.2 mm below the surface.
- The two inserts' scores are added; full credit (1.0) also needs the iron lying back on its stand, let go, for 1 s;
  until then the score is at most 0.75.
- In the simulation depths are read from where each insert is (its joint's measured position), the robot touching
  the hot metal from contact points less than 71 mm from the tip, and the work surface from the hot metal coming
  within 1 mm of it.

**How the inserts melt in (and in the simulation).** A cold insert doesn't move. It heats only while the iron's tip is
pressed on it, and the PLA round it melts only once it passes about 160 C (PLA melts at 145-160 C): from room
temperature that takes a few seconds of contact, so a brief touch does nothing. Once the tip lifts it cools; the
cold steel block's face draws heat out faster still, so the block has to arrive within a few seconds and press
without pausing. In the simulation each insert has a temperature that rises towards the tip's 225 C with a 2 s time
constant while the tip touches it and falls towards the room's 22 C with a 15 s time constant once it lifts (4 s under
the block); above 160 C a push of at least 1 N from the iron or 0.3 N from the block moves it at up to 2 mm/s, from
where it is. From cold, 2.3 s of contact brings it to 160 C; after the iron leaves it stays above 160 C about 5 s on
its own, about 1.1 s under the block. These time constants are estimates (no published figures); the block can only
push it until its face rests on the plate, and the insert keeps the tilt the iron drove it in with.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This 3D-printed robot part needs metal threads for the M3 screws that will hold it. Install the two brass heat-set inserts sitting loose in its holes: melt each one most of the way in with the hot soldering iron, keeping the iron straight, then press it flush with the steel block while the plastic is still soft. Lay the iron back on its stand when you are done." \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/heatset-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Seed 0; the
scene has no random placement (`-E object_jitter=0`). On an RTX 3060 the scripted demo (58 s of simulated time) takes
about 6 minutes including start-up; a full 3,600-step trial about 15 minutes. Tested with yam-arena 0.1.0,
inspect-robots 1.0.0, Isaac Lab 17.0.2 on Isaac Sim 6.1.0.

Offline checks (no GPU): `python task.py` runs the heating rule and the scoring on hand-made pushes (28 cases, among
them a brief touch followed by the block, repeated short touches, the block arriving too late, a joint target held
back from a stuck insert, and the hot metal on the work surface) and replays both arms' plans through their
kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=2 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "..." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/heatset-demo
```

The right arm picks the iron off its stand by the grip, lifts it clear and turns it tip-down; for each insert it comes
down onto it, presses and holds 3 s while the insert heats, sinks it to 80 % of its depth at 1 mm/s, lifts and moves
off to the right. The left arm, holding the block by its top and waiting beside the insert, follows the iron in about
2 s after the tip has left the insert: at the first insert it presses the block down flat on the plate and lifts it
away; at the second it sets the block down on the insert and lets go, so the block's own weight (7.5 N) presses the
insert in until the block sits flat on the plate, and leaves it standing there while the insert sets. Finally the
iron is turned level and laid back on its stand.
`HEATSET_LOG=1` prints each insert's measured depth, joint target, temperature and tilt, the contact forces and the
hot-metal check every 0.5 s.

## Reset

Use a fresh printed plate with two fresh inserts set in its holes for each trial (or reprint the plate). Lift the
block off the plate if it was left there, let it cool to room temperature (5 minutes; it is the heat sink that sets the inserts) and wipe it, or swap in a second block.
Check the sleeves' temperature labels and replace any that have turned. The iron stays hot on its stand, its cord on
the balancer; lay it back on its stand by its grip only if a trial left it elsewhere.
