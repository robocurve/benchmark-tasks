# Hot-swap a server's failed power supply without disturbing the live one

A two-bay power-supply cage is fixed to the table with its bays facing the arms, like the back of a server in its
rack. The left bay holds a failed supply (amber light); the right bay holds the supply still powering the server
(green light). A spare lies on the table in front of the cage, to the left. All three are full-size CRPS-format
supplies (185 x 73.5 x 39.5 mm, 0.9 kg), the hot-swap format used across data-center servers. The robot must take
the failed supply out and seat the spare in its bay without disturbing the live one.

Each supply is held in its bay by a release latch modelled on a CRPS latch. An orange grip handle is fixed to the
supply's face; beside it, on its left, stands a separate green release tab on a spring-loaded slider. The tab drives a
hook on the supply's side whose square catch stands 5 mm out into a notch in the bay wall, so pulling the handle does
nothing until the tab is pushed 5 mm toward the handle (as on a real CRPS supply), which draws the hook in. Behind the
catch the hook is ramped: a supply pushed into a bay rides the ramp in on the mouth's edge, and the spring snaps the
hook out into its notch when the supply reaches home.

At the back of each bay is the supply's edge connector, modelled on a card-edge power connector: a steel blade from
the back stop with a ridge on each side, entering a slot in the supply's back where two spring-loaded contacts take it.
Over the last 8 mm of insertion the contacts climb the ridge's steep front face (spread 2 mm against 5 N/mm each), slide
over it and drop down its back face 1 mm before home. That takes a push of about 35 N to mate and about 11 N to unmate,
so a supply has to be pushed firmly and fully home for its hook to catch, and the failed one has to be pulled out
against the connector's hold.

What the task tests:

- **Releasing while pulling.** The tab has to be held against the handle while the same hand pulls; pulling the handle
  alone does nothing.
- **Moving a long, heavy part into a close bay.** The spare is 185 mm long with 2 mm of clearance a side (30-degree
  lead-ins at the mouth): turned by 1.2 degrees it jams. It starts off to the side and has to be slid across, squared
  up and pushed in straight.
- **Seating it fully,** through the connector's mating force, so the latch catches.
- **Not disturbing the live supply,** whose handle and power-cord plug the failed supply passes when it is moved out of
  the way.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `psukit.py` | Rigid parts built from boxes, with spring-loaded sliders or fixed in place, for yam_arena task files |
| `demo.py` | The scripted full-score demo |
| `test_scoring.py` | Offline checks: every scoring outcome, and the layout's geometry (no GPU) |
| `fabrication/` | Real-world build: OpenSCAD sources for every printed part (dimensions from `task.py`), `BUILD.md` (parts list, print settings, assembly, force calibration) and `check_params.py` / `test_fabrication.py` (check the sources against `task.py`) |

`psukit.py` hooks `yam_arena.scene` and the embodiment's reset without modifying yam_arena, so the task runs with the
standard `yam_arena` embodiment and `task_score` scorer. It follows the part kit used by other tasks in this
repository, and adds fixed parts and boxes turned about the vertical.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/psu-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1800              # writes demo_*_cam.mp4, setup.png, result.json
DEMO_DEBUG=1 python demo.py task.py debug_out --no-record      # also prints each supply's offset, sliders, tilt, yaw
python -m pytest test_scoring.py fabrication/test_fabrication.py   # offline checks
```

1. The right arm closes its fingers across the failed supply's handle and tab together, one pad on the fixed handle bar
   and the other on the tab, which pushes the tab to its stop against the handle (releasing the latch), as a
   technician's thumb and fingers do. Holding that, it pulls the supply 26 cm straight out, over the connector's hold,
   and lets go; then it grips the supply's body and slides it 15 cm right, in front of the live supply and clear of the
   bay's line.
2. The left arm grips the spare's body at its centre and slides it right and a little forward until it lines up with
   the empty bay, its back face 2 cm short of the mouth, in steps of at most 3 cm with small corrections of position
   and turn, and lets go.
3. The left arm closes its fingers, turns the gripper to point forward (20 degrees below level) and pushes the spare
   by the front of its handle bar, on its centreline, into the bay and home: the lead-ins steer it in, the hook rides
   in on its ramp, the connector's contacts climb over the blade's ridge, and the hook snaps into its notch. Pushing
   along the fingers, the force goes through the arm rather than bending its compliant wrist; the push is commanded
   6 cm past home so the arm presses hard enough, and the back stop takes the rest.

The demo runs open loop: it reads no simulator state. The arms are compliant and lag under the supplies' friction,
so where a supply ends up after a move and the small corrections in step 2 were measured once with
`DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record` (which runs the same moves closed loop on the
simulator's poses and prints the numbers) and are fixed in `CAL` in `demo.py`. The layout is fixed and the
simulation deterministic. The demo reaches 1.0 (success) at step 1,131 (57 s).

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). Positions are
measured from where a supply sits latched home in the left bay.

- 0 if the live (green) supply was at any point moved more than 5 mm.
- 1.0 once all of these have held together for 1 s (20 control steps in a row): the spare is home in the left bay
  (within 1.5 mm along it and 2 mm across it, level within 3 degrees) with its hook out in the notch (the release tab
  within 1 mm of its rest position), the failed supply is clear of the bay (its back face at least 5 mm in front of
  the mouth, or its centre more than 7.85 cm to the side), and the spare is still (it moved less than 1 mm over the
  preceding 1 s). `score()` gives 1.0 from the step this has held for 1 s, and `success()` returns true only from
  that same step (never while the score is below 1.0); the runtime ends the episode once `success()` has stayed true
  for its own 1 s, so an episode can only end with success at 1.0.
- 0.75 (also while the spare is home but has not yet held for 1 s): after the failed supply came out, the spare was
  pushed in with its back face 10 cm inside the mouth.
- 0.5: the failed supply was pulled all the way out of its bay (back face 5 mm in front of the mouth).
- 0.25: the failed supply was pulled at least 2 cm out (which needs its latch released).
- Otherwise 0.

## Runtime and seeds

About 1.5 minutes per trial for the full 1,800 steps on an L40S, plus about 1 minute for Isaac Sim to start. The
layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

Rigid bodies built from boxes; the supplies are full CRPS size and weight. The handle is fixed to the supply's body,
and the release tab is a separate part (60 g with the latch arm and hook) on a prismatic joint across the supply with a
spring drive (1000 N/m, 5 N at its 5 mm stop), pushed toward the handle as on a real CRPS supply. The handle and tab
stand 27.5 mm proud of the face and are 48 mm tall, larger than on a real supply, so a gripper's fingers fit on them.

The edge connector's mating resistance is representative of a CRPS card-edge power connector, whose maximum mating
force is specified at 40 N (TE Connectivity High Density Card Edge Power Connectors, which provide the CRPS power-supply
connection; product data). In simulation two contacts, each on a slider with a 5000 N/m spring, ride over a ridge with a
50-degree front and 40-degree back face: about 35 N at the peak (24 N of that from the springs and ramp, the rest
friction at 0.2), and about 11 N to pull a supply out (specified minimum unmating forces for such connectors are around
10 N). The newer, larger M-CRPS connectors are specified at higher forces (up to about 150 N); this task models the CRPS
format. On a real supply the gold-finger blade is on the supply and the sprung contacts are in the server; here they are
swapped, which gives the same forces. The connector's resistance is a function of position only (no wiping friction
along the fingers beyond the ridge).

Bays have 2 mm of clearance a side (a little more than a real chassis) and a 15 mm divider. The cage is fixed, as a
server is bolted in its rack. Power cords: the live supply's cord plug (IEC C13) is modelled, seated in its AC inlet and
part of that supply, so knocking it moves the live supply; the cord beyond the plug is not modelled (taken as routed
away along the rack). The failed supply's cord is taken as already unplugged (the first step of a real swap, normally
done by the technician before touching the latch); the spare has none. Nothing is powered. Friction: 0.25 for the
supplies, 0.15 for the steel cage. Light angular damping is added to the parts. No contact force cap is enforced in
simulation.

## Real-world build

`fabrication/BUILD.md` covers the whole build, with OpenSCAD sources for every printed part in `fabrication/`:

- **Cage:** two bays, with lead-ins, notches, back stops and the connector blades. It prints in two halves for a
  220 mm bed and is screwed to a board clamped to the table.
- **Supplies:** printed shells weighted with steel flat bar to 0.9 kg. Each has a fixed handle, a release tab that
  slides on a 1 N/mm spring and drives the latch carriage and ramped hook, and two contact carriers on 5 N/mm
  springs in the connector slot.
- **Dummy plug and inlet**, with stickers for the status lights.

`BUILD.md` also lists generic hardware (springs by rate and size, M3 screws and heat-set inserts), print settings,
assembly steps and the layout on the board. Its calibration checks use a kitchen scale and a luggage scale: the tab
releases at about 5 N over 5 mm, the connector mates at about 35 N and unmates at about 11 N, and each supply weighs
0.9 kg. Every dimension is in `fabrication/params.scad`, and `python fabrication/check_params.py` checks those values
against `task.py`. `BUILD.md` also describes using real unpowered CRPS supplies with only the cage printed. Nothing
in the setup is powered or connected to mains. The live supply's plug is a printed dummy, optionally with a short
cut cord stub whose conductors are not connected, so there is no electrical hazard.

## Assets

All geometry is defined in `task.py` from boxes; no external assets.
