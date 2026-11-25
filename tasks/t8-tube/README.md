# Take a dead T8 tube out of a strip light under a shelf: the quarter turn frees it, so it must already be held

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (bench layout and lampholder constants, scorer, fast IK) and `tests/` their unit checks.

## The task
A 2-ft one-lamp T8 strip light is screwed under a wall shelf above the bench. Its lamp, a dead F17T8 fluorescent tube
(25.4 mm across, 604 mm between its end caps, G13 bi-pin ends, blackened ends), is locked in the fixture's two
rotor-lock ("turn-type") G13 lampholders. An open lamp-recycling carton for 2-ft lamps lies on the bench in front of
the shelf. The instruction is only "Take the dead tube out of the light under the shelf and put it in the recycling
box."

How it is done, and why it is hard:
1. The lamp comes out only by turning it a quarter turn about its own axis (both rotors turn with its pins) against
   the lampholders' detent: about 0.2 N m per lampholder to turn it out of the locked click, then drag. Two grippers
   pinch the 25 mm glass tube and turn it together about its axis; the YAM wrist cannot go round 90 deg in one grip,
   so it is turned in strokes, letting go and going round between them (the rotor drag keeps it where it was left).
2. In an overhead fixture the lampholder slots open downward. The instant the pin pairs line up with the slots, nothing
   holds the lamp up: it must already be held when the last part of the turn is made (the reference demo holds it
   with both hands).
3. It can then only leave straight down: the pins slide down the 22 mm slots and out under the lampholders.
4. The 0.6 m glass lamp is carried, turned to lie along the carton, and laid in it without a fall.
Cues the robot can see: the lamp in its lampholders, the blue printed lamp code on the glass that turns with it (how
far it has turned), the slot in each lampholder, the lamp's height under them once it is free, the carton.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- Vetoes (latch 0 for the whole trial): the lamp fell (out of the lampholders, no robot link touching it, and moving
  faster than 0.8 m/s, i.e. a free fall of more than about 3 cm: a dropped glass lamp counts as broken); a lampholder
  broke (loaded past its strength, 60 N per lampholder, for 0.1 s: e.g. the lamp wrenched down hard without turning
  it).
- Full credit (1.0) requires both end caps supported by the carton floor, inside the carton, with the lamp free of the fixture and grippers for four continuous seconds.
- Partial credit (0.5) requires both end caps supported by the tabletop, the whole lamp clear of the carton footprint, and the same four-second released hold. The episode can continue after partial credit.
- A support-height tolerance of 4 mm accommodates collision offsets. At each 20 Hz observation the tube's centre, end-cap and rotational speed proxy must stay below 0.02 m/s. Motion, renewed lamp contact, or a change of supporting surface restarts the hold.
- All other unfinished, held or unstable placements score zero. A latched drop or holder break scores zero regardless of the final placement.

For a physical trial, record the whole removal and four seconds of the released tube at rest. Check both ends against the support and the carton boundary.

## Dream-1 evaluation

On Brev, Dream-1 scored 0.0 in 1,500 of 1,500 control steps at evaluation seed 0. The Inspect Robots log contains 1,500 actions of 14 values each. Run the following from `isaac/` inside the configured YAM environment.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu \
    -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction 'Take the dead tube out of the light under the shelf and put it in the recycling box.' --scorer task_score --seed 0 --max-steps 1500 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy t8_tube_scripted)
python official_run.py -- run --embodiment yam_arena --policy t8_tube_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "Take the dead tube out of the light under the shelf and put it in the recycling box." \
    --scorer task_score --max-steps 1500 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 1500

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438
python demo.py task.py <out>/<run> --stage neg-let-go-open --seed 1415158438  # turned open, then let go: falls: 0
python demo.py task.py <out>/<run> --stage neg-pull-down --seed 1415158438    # pinched and pulled straight down: 0
python demo.py task.py <out>/<run> --stage outside --seed 1415158438       # stable tabletop placement
python demo.py task.py <out>/<run> --stage neg-drop-high --seed 1415158438    # let go 10 cm above the carton: 0
python demo.py task.py <out>/<run> --stage replay --seed 7     # TUBE_REPLAY=<trace.jsonl> [TUBE_MIRROR=1]: open loop
python demo.py task.py <out>/<run> --stage look --cam-size 1280x704           # setup view from the left wrist camera
```
Unit checks (no simulator): `python -m pytest tests`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> top.mp4 left.mp4 right.mp4`.

## Submission media

- `model_run.mp4` shows the full Dream-1 trial. The views are overhead, left wrist and right wrist at 20 Hz.
- `scripted_run.mp4` shows the independently rerun official scripted policy, which scored 1.0 in 789 steps.
- `setup_overhead_hires.png` — Overhead view of the reference setup.

Start states: every trial reset with a seed (every `inspect-robots run` passes one) places the fixture with its shelf
360-435 mm forward, +-40 mm sideways and 270-330 mm above the bench (lamp axis), turned 4-8 deg either way (the shelf
is not square to the robot); the carton is placed 195-235 mm forward, +-40 mm sideways, turned up to 6 deg. When
either hand of the default start's pinch (15 cm either side of the lamp's centre), or of its mirror image (arms
swapped), would still find a start's lamp inside its capture window (up to 45 mm nearer, up to 15 mm farther and
within 14 mm across, plus a 10 mm margin along and 16 mm across), that start's fixture is redrawn (about 97 % of raw
draws, so a start's lamp ends up well lower, higher, nearer or farther than the default one), so that neither a
replayed default trajectory nor its mirror image finds the lamp inside its capture window (replays at the nearest
starts: 0). Unseeded resets (direct `demo.py` runs) use three named setups in turn.

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176.
- Lamp: F17T8 at real size: 25.4 mm glass tube, 604 mm between the end-cap faces, aluminium end caps, G13 pins
  (2.41 mm, 12.7 mm apart, drawn only), 0.14 kg (estimate), friction 0.8. Lamp dimensions are typical F17T8 data; no
  datasheet or drawing could be read while building the task.
- Fixture: a 2-ft one-lamp strip light (channel 60 x 35 mm) under a 90 x 28 cm shelf on a wall rail, all static
  colliders on one fixed mount (the mount's stiff slides only place it for each start state). Two rotor-lock G13
  lampholders (34 x 16 mm, slot 3.2 mm wide, 22 mm from the rotor centre to the bottom edge; estimated dimensions of
  the common turn-type tombstone) with 1 mm between each lampholder face and the lamp's end-cap face; the end caps
  collide with the faces. The lamp axis is 55 mm below the channel (about 2.2 in), so the gripper fits above it.
- Lampholder model (no pin or rotor geometry is collided; drives on a joint chain from the fixture to the lamp, switched
  each physics step): locked, the lamp is held at its seat by stiff drives whose force caps are the lampholders'
  strength, 60 N per lampholder (estimate for a snap-in plastic tombstone; held at the cap for 0.1 s = broken, veto).
  Turning about its own axis: a detent snaps it back to locked within 12 deg with up to 0.2 N m per lampholder, and
  past that the contacts drag at 0.08 N m per lampholder (estimates); rotor stops at 90 deg. Either direction opens.
  Within 2.6 deg of 90 deg (the slot clearance) the pins are in line with the slots, which open downward: nothing
  holds the lamp up; while an end's pins are in its slot the lamp cannot move across the slot, turn or swing sideways,
  and it slides down freely; once both pin pairs are below the slots the lamp is a free body. Approximation: while one
  end is out of its slot and the other is not, the whole lamp is still kept from moving across the slots.
- Drop = broken: the lamp counts as fallen if it is out of the lampholders, no robot link touches it and it moves faster
  than 0.8 m/s (a free fall of about 3 cm). Glass breakage itself is not simulated.
- Carton: open-top corrugated carton (inner 105 x 650 x 70 mm, 4 mm walls, 0.3 kg, estimated), a free body on the
  bench that the hands can shove.
- On the revised seed-0 full-score run the drive-based squeeze estimate peaked at 20.87 N per finger. PhysX reported maximum single-link contacts of 24.8 N on the tube, 10.6 N on the carton and 8.1 N on the fixture. The maximum summed robot contact on the tube was 85.1 N across the hands. Lampholder lateral load peaked at 8.33 N and the carton shifted 5.9 mm.
- Robot-on-part contact (for "untouched" and the drop monitor) is read from PhysX contact forces between the robot's
  links and the lamp, carton and fixture (collider identity, not contact positions).
- Nothing welds, snaps or teleports anything during a trial; the scripted demo only sends the 14 joint / gripper
  commands and reads state. On an environment reset (between trials only) the kit places the parts for the next start.
- The real practice scored: rotor-lock lampholders release the lamp after a quarter turn and it comes out downward;
  used fluorescent lamps contain mercury and are collected unbroken for recycling (US universal-waste rule,
  40 CFR 273), so a dropped lamp fails.

## Real rig
Safety: a dropped fluorescent tube shatters and releases mercury, so real trials never use a glass fluorescent lamp
and the fixture is never powered.
- Tube for the unpowered rig: RAB T8-6-24P-840-DIR. RAB specifies a 2 ft T8 with a G13 base, a plastic polycarbonate body, and a weight of 0.2 lb (about 91 g). Weigh the actual tube and check both ends against the chosen rotors before the first trial. Mark one end cap with blue tape for the rotation cue. [RAB product specifications](https://www.rablighting.com/product/T8-6-24P-840-DIR)
- Fixture: a 2-ft one-lamp T8 strip light with rotor-lock (turn-type) G13 lampholders, for example from the Lithonia
  ST8 strip series in its 2-ft one-lamp version (about USD 30-60); any 2-ft strip with turn-type tombstones works.
  Leave it unwired (no mains connection).
- Mounting: screw the strip to the underside of a 19 mm shelf board (about 90 x 28 cm) with its channel flush with the
  shelf's front edge; hang the shelf on two wall brackets on a wall rail behind the bench so that the tube's axis is
  270-330 mm above the bench and 360-435 mm in front of the line between the arm bases, turned 4-8 deg either way
  (draw the start from `layout.setup_spec`; mark bracket positions every 5 mm).
- Carton: an open-top corrugated carton about 11 x 65 x 7 cm inside (a 2-ft lamp-recycling carton or any box that size),
  on start marks on the bench.
- Reset (about 1 minute): put the tube back in the lampholders with its pins in the slots, push it up and turn it 90
  deg until both rotors click (blue tape facing the robot); move the shelf and the carton to the next start marks.
- Real-trial scoring checklist (eyes and video only):
  1. Did the tube fall at any time with no gripper on it (a visible free fall of more than a few cm)? Yes: 0.
  2. Did a lampholder crack, come off or get pulled out of the channel, or was the tube wrenched out without being
     turned? Yes: 0.
  3. At the end, is the tube out of the fixture and lying inside the carton (both end caps inside, below the rim),
     still, with both grippers clear of it for 4 s? Yes: 1.0.
  4. Is the whole tube clear of the carton, both ends resting on the tabletop, still and untouched for 4 s? Yes: 0.5. All remaining outcomes score 0.

## Known sim/real differences
- The real trial uses an unpowered shatterproof LED tube; the simulation models a glass F17T8 and
  does not light or break it: a fall of more than about 3 cm is scored as broken.
- In the simulation the pins and rotors are not collided: a joint model holds the tube at its seat, gives the rotor
  detent (0.2 N m click, 0.08 N m drag per lampholder, estimates) and frees it down the slots at 90 deg. Real
  lampholders vary in turning torque and in which directions they turn.
- The simulated lampholder breaks at a sustained 60 N (estimate); in a real trial "broken" is a visible crack or a
  lampholder pulled off.
- Simulated dimensions (lamp 604 mm, 25.4 mm; lampholder 34 x 16 mm, slot 22 mm; channel 60 x 35 mm, lamp axis
  55 mm below it; carton) are typical values, not measured on the named products.
- The fixture's start pose is redrawn in simulation to keep replayed trajectories from pinching the lamp; a real rig
  uses the same drawn poses via the shelf bracket marks.
- The revised full-score trial stayed below the program's 40 N per-finger guideline on every logged finger contact. The motor spring estimate and the simulated contact peaks measure different quantities. The physical tube, pads and holder detent have not been tested together. A real trial should record per-finger force during breakout and placement.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm URDF and meshes | [I2RT Robotics model source](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) | MIT. The installed runtime records the model commit in `yam_arena/assets/yam/SOURCE_COMMIT`. |
| Station, cameras and YAM embodiment | [YAM Arena runtime used on Brev](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730#license) | Installed dependency. Its README reports that the integration code has no declared license. |
| Isaac Sim 6.1 and Isaac Lab-Arena | NVIDIA and [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) | NVIDIA Isaac Sim license and Apache-2.0 for the Arena package, respectively. |
| Shelf, strip fixture, lampholders, lamp and carton geometry | generated in code by this task (`src/layout.py`, `isaac/tubekit.py`); simplified convex pieces at real sizes (estimated where no drawing was available), no manufacturer CAD | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
