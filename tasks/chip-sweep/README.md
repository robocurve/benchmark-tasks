# Clear undercut machining-fixture T-slots with a rake

Use a T-slot rake to draw five chip surrogates through three undercut fixture channels
into a green collection bay. Return the tool and open both grippers. The 20 mm T-foot
fits inside each 32 mm chamber, while its 6 mm neck passes through a 14 mm top opening.
The foot must enter through an open end, then travel beneath the overhanging ledges.

The scripted physics demonstration scored **1.0** in **2,032 steps** (101.6 simulated
seconds), terminated with **success**, and took **204.82 seconds wall time** on the L40S.
All three complete 101.65-second H.264 camera videos were downloaded. The top sequence,
final collection bay, channel interaction and tool return were visually inspected.
All five chips are collected, the rake is parked, and both grippers are open and clear.
Ten offline scorer/geometry checks pass.

## Reproduce

In the configured YAM Arena RTX environment, from this folder:

```bash
uv pip install -e . --no-deps
python -m unittest discover -s . -p 'test_*.py'
inspect-robots run --embodiment yam_chip_sweep --policy chip_sweep_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E device=cpu -E top_cam=demo \
  -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction 'Use the T-slot rake to clear all five metal chips from the three undercut channels into the green collection bay. Return the rake to its supports and open both grippers.' \
  --scorer task_score --max-steps 2400 --seed 0 --store-frames \
  --log-dir /path/outside/repository/tslot-demo
```

Use `$SIM_ROOT/cluster/run-arena.sh` inside the configured L40S allocation. Render the
terminal JSON with `inspect-robots video <run.json>`. For Dream-1 use the same embodiment,
scene, seed and limit with `--policy dream -P model=dreamlabs/dream-1-beta`.
The historical package/entry-point names `chip_sweep` are retained by this task.

## Fixed setup

World x points forward, y left and z up. Tabletop z=0.7475 m. `task.py` and `build_assets.py`
are the complete specification; the generator uses usd-core to regenerate self-contained
USD assets. Lengths below are mm unless stated otherwise.

- A fixed 350 × 190 × 12 floor is centered at (0.35,0.13) m. Its 200 mm long undercut
  channels span world x=0.325 to 0.525 m, centered at y=0.075, 0.130 and 0.185 m.
  Each lower chamber is 32 wide, its top slot 14 wide; ledge undersides are 24 above
  the table and tops 34 above. Both ends are open. Side rails, front lip, fasteners
  and visual reference marks complete the fixture.
- The green collection bay covers x=(0.185,0.305), y=(0.045,0.215) m on the same floor,
  whose upper surface is z=0.7595 m. There is no floor step at the channel exits.
- Five rigid blunt chips are 12 × 8 × 6, each 6 g. Initial xy positions are
  (0.405,0.075), (0.455,0.075), (0.435,0.130), (0.405,0.185), (0.475,0.185) m,
  all centered at z=0.7625 m; yaw is 0, 0.15, −0.2, 0.2 and −0.15 rad.
- The 80 g rigid rake has a wooden handle, raised 25 × 20 × 50 grip, 6 × 6 neck,
  and a steel 12 × 20 × 3 T-foot. The foot bottom is 20 below the root. The raised
  grip keeps robot fingers above the fixture. Tool root starts at
  (0.24,0.27,0.7955) m, on fixed supports at x=0.18 and 0.34, y=0.27 with heights
  56 and 28. Its local-bounds reference center is offset (−11.25,0,27.5) from its root.

Reset restores tool, chips and both arms, clears contact/failure history and settles.
Seed 0 and zero jitter define this fixed layout. Nothing is consumed. The scene depicts
an unpowered workholding fixture and blunt training chips, with no cutting process.

## Numeric scoring

Use the final score. Each chip earns 0.16 only after measured rake–chip contact or a measured chip-contact chain from a rake-contacted chip and when
its full world AABB lies inside x=(0.185,0.305), y=(0.045,0.215) m, bottom within 3 mm of
z=0.7595 m, linear speed below 10 mm/s and angular speed below 0.5 rad/s.

Add 0.20 only when all five qualify, the rake is returned and resting, both grippers are
at least 75% open and both TCPs are farther than 90 mm from its reference center.
Parking means reference xy within 25 mm of (0.22875,0.27) m, bottom within 6 mm of
z=0.7755 m, vertical extent under 105 mm and rest. Holding 1.0 for one second succeeds.

Direct robot–chip contact permanently invalidates the trial, including actual
`LeftArm`/`RightArm` actor paths. So does a chip/tool bottom below tabletop minus 20 mm,
nonfinite position, center outside x=(−0.1,0.95) m or |y|>0.62 m. Invalid trials score
zero. Unqualified chips earn zero; no intermediate score maximum is retained.

## Physics, demonstration and review status

PhysX runs at 240 Hz, control at 20 Hz. Loose objects use 32/4 solver iterations,
0.5 mm contact offset, zero rest offset and approximate station 1.2/1.0 friction.
Tool and chips are rigid, with no compliance, fracture, adhesion or attachments. No
object poses are written during trials. Masses and material looks are uncalibrated.

The demonstration reads privileged object poses and robot FK and sends joint targets
only. Bounded integral pose correction and measured alignment holds keep the T-foot and
neck aligned during entry and travel. All nominal pickup, hover and working poses were
checked with FK below 0.1 mm residual. The model uses ordinary images/joint state through
the same contact-enabled embodiment. Contact reports verify tool-driven contact provenance for each chip, including chip clusters. A contact edge between chips propagates provenance through the measured episode contact graph; isolated chips do not qualify.

Tested framework pins: YAM `0059d02a1a91bc6834c1d0846022891df0c78730`, Arena
`c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; Python 3.12.12, Isaac Sim 6.1.0.0,
Isaac Lab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0, NumPy 2.3.1 and Torch 2.11.0+cu128.
Runtime hashes and media review are recorded in `validation.json`.

## Dream-1 evaluation

`dreamlabs/dream-1-beta` completed with a final score of **0** in **2,400 steps**,
terminating with `max_steps`. Wall time including initialization and inference
was **267.89 seconds**. The run used seed 0, zero jitter, 20 Hz control, 640 × 360
saved camera frames, the `demo` top-camera configuration and the same runtime source
hashes and physics as the expert demonstration. The model received ordinary camera images
and joint state. All three complete camera videos were downloaded and visually reviewed.

The reviewed sequence shows grasp attempts near the rake and fixture, but the rake remains on its supports and the five chips stay in their starting channels. No chips are cleared into the collection bay. Final history has no rake-chip contact or chip-contact provenance. The top sequence, final frame and both wrist views were inspected.

`validation.json` records the completed result, runtime revision and video hashes.
The Dream video, full-score demo and setup image are linked through the media ID in `task.json`.
