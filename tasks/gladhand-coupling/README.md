# Couple a tractor's air line to a trailer's gladhand

A grey steel plate (the trailer's nose) stands across the far side of the table with two fixed air-brake gladhands on
it: the red emergency gladhand on the left and the blue service gladhand on the right, seal faces toward the robot,
bodies pointing up. One red air hose runs from a brass fitting on a post near the left arm to a red gladhand, which
starts locked, seal face down, on a dummy coupler in front of the left arm.

The robot has to turn the hose's gladhand off its dummy coupler, carry it to the red trailer gladhand, press the two
rubber seals together with the bodies at 90 degrees, turn it a quarter turn until it stops with its body pointing
down, and let go. The lock is not scripted: the gladhand's lug has to slide under the other gladhand's ramped flange
while the seals are squeezed, and a gladhand turned with the seals apart, or stopped short, is not locked.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), from
this folder. `gladkit.py` and `scoring.py` must stay next to `task.py`.

```bash
INSTRUCTION="Connect the tractor's air line to the trailer, then let go."

# Dream-1 (needs a Dream API key in ~/.dreamlabs/credentials)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/gladhand-dream
for log in ~/yam-eval/gladhand-dream/*.json; do inspect-robots video "$log"; done

# The scripted demo, which reaches full score (1.0 after 866 steps)
python demo.py task.py demo_out                    # 320x176 cameras
python demo.py task.py demo_out --cam-width 1280   # 1280x720, for a sharper video

# Check the demo's plan against the arm's joint limits without starting the simulator
python demo.py task.py - --dry
```

Both print `task_score` at the end. The demo writes one MP4 per camera, `setup.png`, `final.png` and `result.json` to
its output folder, and logs where the gladhand is against the dummy coupler and both trailer gladhands (face gap,
angle, offset, tilt); `DEMO_DEBUG=1` adds each arm's tracking error and joint torques.

The demo is closed-loop, so it has its own runner instead of the `yam_scripted` policy (which plays fixed waypoints).
It drives the same `yam_arena` embodiment with the same joint-position actions at 20 Hz. It looks at where the
gladhand is (it is in plain view of the cameras) before each grasp and while it lines up and turns; it never moves
objects or edits the simulation.

## Scoring
Scored at the end of the trial. `scoring.py` holds every threshold and needs no simulator.

The hose's gladhand is **locked** to a trailer gladhand when all three hold:
- the two connector plates are no more than 2 mm further apart than when fully locked (seals compressed);
- it has turned to within 10 degrees of the lock stop;
- it stays on when pulled 60 N along its hose for 1 s. In simulation: the locking lug is fully under the flange.

| Outcome | Score |
|---|---|
| Gladhand still on its dummy coupler (seal centre within 5 cm of the dummy's) | 0 |
| Off its dummy coupler, not locked to a trailer gladhand | 0.2 |
| Locked to the red trailer gladhand | 0.8 |
| Locked to the red trailer gladhand, and both grippers at least 2 cm clear of every gladhand and the hose for the last 1 s | 1.0 |
| Locked to the blue trailer gladhand | 0 |
| Gladhand or hose more than 2 cm below the table top at any time (off the table, or on the floor) | 0 |
| Hose's fixed end moved more than 5 mm at any time (pulled off its fitting) | 0 |

On a real bench: a photo against the lock stop gives the angle, a feeler gauge between the plates gives the gap, and
a spring scale along the hose gives the 60 N pull.

### Wrong strategies, run in the simulator
`DEMO_VARIANT=<name> python demo.py task.py out` plays these; each scores what the table above says it should.

| `DEMO_VARIANT` | What the arm does | Score |
|---|---|---|
| (none) | the full demo | 1.0 |
| `hold` | locks it but keeps hold of the gladhand | 0.8 |
| `short` | stops the turn 28 degrees short of the stop and lets go | 0.2 |
| `seals_apart` | turns the gladhand with the seals 3 cm apart; the lug misses the flange and it drops to the table | 0.2 |
| `off_only` | turns it off the dummy coupler and sets it down on the table | 0.2 |
| `crossed` | locks it onto the blue trailer gladhand | 0 |
| `none` | does nothing | 0 |

`python -m pytest -q test_scoring.py test_task.py` checks the scoring cases (including 15 degrees short, 3 mm apart,
resting against the coupler without the lug engaged, on the floor, hose pulled off) and that the layout and gladhand
dimensions in the code match the setup text. `test_scoring.py` needs only Python; `test_task.py` imports the task
file, so run it in the simulator's environment.

## Runtime and seeds
- The demo takes 866 control steps (43 s of simulated time): about 8 s to grip and turn the gladhand off the dummy
  coupler, 15 s to carry it to the trailer gladhand, 18 s to line up, press, turn and re-grip, and under 2 s to let
  go. The trial ends with success as soon as the grippers have been clear for 1 s. The run limit is 2,400 steps.
- On an L40S the demo run takes about 5 minutes including Isaac Sim's start.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start. Repeated runs give the same result.

## Fidelity
- **Physics:** Isaac Lab-Arena (Isaac Sim, PhysX), rigid bodies; control at 20 Hz.
- **Gladhands:** real size (150 mm long, 50 mm seal face, 30 mm thick at the grip, 0.45 kg). Each is built from
  convex collision pieces: head, handle, a locking lug at the neck, and a connector plate at the tip whose flange has
  a ramped underside, a detent and a stop. The lug slides under the opposite flange by contact alone; nothing is
  scripted and no joint is created when the parts meet.
- **Connector plate profile:** our own parametric model of the general gladhand form (quarter-circle flange, ramp,
  detent, stop). It was not measured from an SAE J318 part and no CAD model was used; the standard is paywalled and we
  found no openly licensed model. The flange ramps down 4.4 mm over 64 degrees to the detent, then rises 0.2 mm to the
  seat.
- **Seals:** each seal is a rubber disc on a short sliding joint with a 67 N/mm spring and 3 mm of travel. Two seals
  meet in series, and locking compresses them 2.9 mm in total, about 97 N. A 300 N/mm spring was tried on paper first:
  it would need about 20 N·m to turn in, against the arm's 7 N·m wrist limit.
- **Torque to turn into lock:** the demo's wrist-roll motor peaks at about 1.5 N·m during the final turn (sampled
  once a second), with the arm also pushing along the turn.
- **Hose:** 1.0 m, 19 mm across, 0.25 kg, as a chain of 20 rigid capsules joined by three-axis joints with bending
  stiffness 0.6 N·m/rad and twisting stiffness 0.5 N·m/rad per joint, and a 1 rad bend limit per joint (a 5 cm
  minimum bend radius). A real 3/8 in rubber air hose is somewhat stiffer.
- **Friction:** seals and hose 1.0 static, 0.8 dynamic; lugs, plates and bodies 0.4 static, 0.3 dynamic.
- **Not modelled:** air flow and pressure (coupling is judged from geometry), seal wear, and the coiled part of a real
  tractor line; only the end the driver handles is here.
- **Pull test:** the 60 N pull is not run in simulation; the scorer checks that the lug is fully under the flange.

## Files
- `task.py`: the task file (layout, objects, scoring hooks).
- `scoring.py`: the scoring rules, pure Python.
- `gladkit.py`: the gladhand and hose models and their state, as an add-on to `yam_arena` task files. It hooks the
  scene builder and the embodiment's step and reset when imported, without modifying `yam_arena`.
- `demo.py`: the scripted demo and its wrong-strategy variants.
- `test_scoring.py`, `test_task.py`: offline checks.

## Third-party assets
None. Every object is generated in code from boxes, cylinders and convex hulls; there are no meshes, textures or CAD
files from other sources. The scene uses the program's YAM station replica as installed.
