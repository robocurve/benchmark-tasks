# Hang a coat rack on the wall by its two keyhole slots

A section of wall stands across the far side of the table with two screws in it, 30 cm apart at a marked
height, their pan heads standing 7.5 mm proud. A dark oak coat rack with three cast hooks stands upright on
the table in front of the wall, hooks toward the robot. Two keyhole slots are routed into its back, 30 cm
apart, to match the screws. The instruction is "Hang the coat rack on the wall."

Keyhole hanging is how coat racks, floating shelves, key holders and towel rails go up, and everyone who has
hung one knows the fumbling: the slots are behind the piece, so both have to find their screw at once,
unseen. The piece goes over the two heads, flat to the wall so the heads enter the slots, and down until the
screw shanks sit in the slots' throats and it hangs. That is the ability tested: a blind two-point
alignment with a push-then-drop motion, the piece held level throughout.

`task.py` sets out the scene, the scoring and the scripted demonstration. `geometry.py` gives the rack's
parts, including the convex pieces that leave the keyhole slots open, and the screws' shapes. `runtime.py` is
the embodiment: the standard `yam_arena` one, with the rack built as one rigid body from those parts, the
screws given their round shanks and domed heads, and the grippers' positions passed to the scorer for the
release check. `run.py` runs a trial. No installed package is modified.

## Runtime and installation

Linux with an NVIDIA RTX GPU. Install the YAM adapter for Isaac Lab-Arena at the pinned commit, from the
benchmark repository root:

```bash
TASK_DIR="$(pwd)/tasks/coat-rack"
ADAPTER_DIR="$HOME/yam-arena-coat-rack"
export ARENA_DIR="$HOME/IsaacLab-Arena-coat-rack"
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git "$ADAPTER_DIR"
git -C "$ADAPTER_DIR" checkout 0059d02a1a91bc6834c1d0846022891df0c78730
git clone --branch release/0.3.1 https://github.com/isaac-sim/IsaacLab-Arena.git "$ARENA_DIR"
git -C "$ARENA_DIR" checkout -B release/0.3.1 c7edf3a9d5d8276565208f72422f8f8a0806d6cf
"$ADAPTER_DIR/setup.sh"          # uses the Arena checkout above and installs everything
source "$ARENA_DIR/yam.env"
cd "$TASK_DIR"
```

The reference environment: Python 3.12, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Isaac Lab-Arena 0.3.0
(release/0.3.1 at c7edf3a9d5), `yam-arena` 0.1.0, `inspect-robots` 1.0.0, `inspect-robots-dream` 0.1.0
and `dreamlabs-client` 1.1.3. The task uses the adapter unmodified and only primitive and procedural
geometry, so it needs no asset downloads beyond the robot and station that the adapter installs. Dream-1
needs a Dream Labs API key in `~/.dreamlabs/credentials` (see the adapter's README).

## Run and score

```bash
python run.py demo      # scripted demonstration (yam_scripted, plays task.py's demo())
python run.py dream     # Dream-1 (dreamlabs/dream-1-beta)
```

Both read the instruction from `task.json` and run with seed 0, no object jitter, 20 Hz control, 240 Hz
physics and 320 x 176 policy camera frames, scored by `task_score`. The limit is 3,200 actions (160 s of
simulation); a full success ends the trial early. Logs go to
`$YAM_LOG_ROOT/coat-rack/<mode>-<timestamp>` (default `~/yam-eval/...`), or `--log-dir DIR`. Add
`--headless false` to watch in the Isaac Sim window. Render the cameras of a trial with:

```bash
inspect-robots video /path/to/adhoc_LOG_ID.json
```

The log's JSON holds the score. `measurements.jsonl` records, every 5 control steps, where each screw is
in its keyhole's frame, whether the rack hangs on it, the rack's gap to the wall, height error and tilt,
the grippers' positions and the running score. `source_manifest.json` holds the hashes of the Python files
that ran.

## Results

Seed 0, the settings above, on one RTX GPU shared with other trials (which lengthens the wall times).
`demo_result.json` and `dream_result.json` hold each trial's final measurements and object poses, and the
hashes of the source files that ran. They match this folder.

| Policy | Score | Actions | Simulated time | Wall time | End |
| --- | --- | --- | --- | --- | --- |
| Scripted demo (`yam_scripted`) | 1.0 | 373 | 18.7 s | 45 s | success: hanging on both keyholes, 0.03 mm off the wall, 0.0 mm off the marked height, grippers 86 mm clear |
| Dream-1 (`dreamlabs/dream-1-beta`) | 0.0 | 3,200 | 160 s | 297 s | step limit: the grippers came within 30 mm of the rack but it never moved |

In the demo both arms work together: each closes on the upturned tip of an outer hook, with its fingers
closing along X so the tip is held between the pads (the pads cannot go between the rack and the wall).
They lift the rack until its keyhole entries are level with the screw heads, push it to the wall so the
heads enter the entries, lower it until the heads reach the top of the channels, let go and withdraw. Under
the 0.7 kg load the arms droop about 7 mm and the rack pitches a few degrees in the grip, so the plan
commands the lift 15 mm high and the push 13 mm past the wall, which the arms' compliance takes up: the
rack's top edge meets the wall first and the push flattens it. The dev runs that found those numbers are
not part of the submission.

## The scene

World frame, metres: X forward from the arm bases, Y to the robot's left, Z up. Two 6-DoF YAM arms with
parallel grippers stand at (0, +0.305, 0.750) and (0, -0.305, 0.750), starting at joint angles
[0, 1, 1, -0.6, 0, 0] rad with their grippers open. The station's oak-coloured table top is at Z = 0.7475.

| Object | What | Where |
| --- | --- | --- |
| Wall section | fixed; 18 mm plywood painted off-white, 600 mm wide and 200 mm tall, standing upright across the table | its face at X = 0.380, from Y = -0.300 to +0.300, from the table top to Z = 0.9475 |
| 2 screws | fixed; No. 8 zinc-plated pan-head screws driven into the wall with their heads standing 7.5 mm proud: 4 mm shank, 8 mm head domed to 5 mm at its top, 2.5 mm thick | centre lines at Y = +-0.150, Z = 0.8475 (100 mm above the table) |
| 2 pencil marks | 20 mm ticks where the rack's top edge goes (drawn in the sim as 1 mm thick dark strips) | Y = +-0.230, Z = 0.8605 |
| Coat rack | loose, 0.70 kg; a 400 x 80 x 25 mm dark-stained oak rail with three cast aluminium hooks on its face (30 x 40 mm base plate, a 50 mm arm standing out, 16 mm wide, turned up 45 mm at its end) at Y = 0 and +-0.160, and two keyhole slots routed into its back at Y = +-0.150: a 14 mm square entry pocket 8 mm deep with its centre 30 mm below the top edge, a 12 mm channel running up from it behind a 2.5 mm lip, the lip leaving a 6 mm throat; the entry's bottom edge and the lips' bottom edges are eased 2 mm | standing upright on its bottom edge, hooks toward the robot, its back 50 mm from the wall, centred on Y = 0; its hooks' tips at X = 0.260 |

Hung, the rack's top edge is at Z = 0.8605 (113 mm above the table) and its bottom edge 33 mm above it.
Nothing else is on the table. The station's camera stand, top camera, wrist cameras and lights are those of
the pinned adapter (`station.py`, `scene.py`). The uploaded setup image is the top camera's view after the
reset.

Building it for real: screw a 600 x 200 mm piece of 18 mm plywood to a batten so it stands upright at the
back of the table, drive two No. 8 pan-head screws 300 mm apart at a marked height, leaving the heads 7.5 mm
proud (an 8 mm head; a dab of paint on the heads helps the cameras); take any 400 mm oak coat rail with
three cast hooks and keyhole slots in its back (a keyhole router bit cuts them: 14 mm pocket, 6 mm throat,
12 mm travel), or rout two slots into a plain rail; ease the entries' bottom edges and the lips with a
countersink. Pencil a tick at each end of where the rack's top edge will be, and mark the rack's standing
place on the table. The wall section is short so that the robot's wrist clears its top; nothing about the
hanging depends on the wall above the rack.

## Scoring

The final state is scored, so a reviewer needs only a photo from in front, a spirit level and a tug.

- **Hanging on a keyhole:** the screw's shank is in the slot's throat with the throat's top resting on it
  (the rack's weight is on the screw). In the sim: the shank's centre within 1.5 mm of the throat's centre
  line and within 1.5 mm of the throat's top, behind the lip. A head sitting in the entry pocket only, or
  a shank part way up the throat with the rack still held, does not count: let go, it would fall.
- **Hung:** hanging on both keyholes, with the rack's lowest point at least 10 mm above the table, every
  back corner within 4 mm of the wall, its top edge within 5 mm of the pencilled height, and its long edge
  within 1 degree of level.
- **Released:** both grippers' centre points (between the finger pads) are at least 30 mm from the rack.

| Outcome | Score |
| --- | --- |
| Hung, both grippers released, the rack still, for 2 s | 1.0 (ends the trial) |
| Hung at the end of the run (a gripper still touching); the grippers are opened after the run and the rack must stay hanging | 1.0 |
| Hanging on one keyhole only (the other end dropped or never caught) | 0.5 |
| Anything else: on the table, held in the air, leaning on the wall, heads in the entries only | 0 |

No shortcut scores: the rack cannot rest on the heads (the pockets are deeper than the heads stand proud),
it cannot hang on the heads in the entries (it falls when let go), and a rack propped on the table against
the wall is under the marked height and on the table. The score at the end of the trial is reported
(`SCORE_OVER = "end"`). In simulation "still" means the rack moving under 1 cm/s and turning under
0.5 rad/s.

## Fidelity

PhysX at 240 Hz, YAM control at 20 Hz, rigid bodies at real size and mass. The rack is one rigid body with
a convex collider per part (the rail's front, the pieces of its back that leave the keyhole slots open, the
hooks' plates, arms and tips); the screws are fixed 12-sided shanks with domed heads; the wall is a fixed
box. Friction is estimated, not measured: rack 0.45 static / 0.40 sliding, screws 0.35 / 0.30, finger pads
0.5 (PhysX averages the two bodies at each contact). Simplified: the keyhole entry is a 14 mm square pocket
where a routed one is round; the rack's density is uniform, which puts its centre of mass about 7 mm
further back than the real rack's (both stand upright on the 25 mm edge); the pencil marks are 1 mm
strips. The screw's play under the head (2.5 mm) and the shank's play in the throat (1 mm each side) are
exact where real ones vary by a few tenths of a millimetre.

## Reset

Nothing is consumed. Stand the rack back on its mark, upright, hooks toward the robot. The simulator's
reset restores every pose and settles for 0.5 s.

## CPU checks

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests
```

They check the rack's shape and mass, that the back pieces leave exactly the keyhole slots open and that the
head passes the entry but not the throat, the starting layout, every scoring outcome and the states that
must not count (heads in the entries only, lowered part way, one keyhole, off the wall, turned, tilted, on
the table), and that the scripted plan is reachable, keeps the arms apart, off the wall and clear of the
rack while gripping its hooks, and ends inside the limit.

## Assets and licenses

- YAM robot meshes and wrist mounts: I2RT geometry bundled by the pinned adapter, MIT License
  (`yam_arena/assets/yam/LICENSE.i2rt`).
- Station room, table, camera stand, lights and cameras: the pinned adapter and Isaac Lab-Arena
  (Apache-2.0); keep their notices.
- Wall, screws, marks and rack: primitive and procedural geometry in this folder; no third-party meshes or
  textures.
