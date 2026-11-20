# Draw a right-angled triangle on an Etch A Sketch, turning both knobs together for the slanted side

Draw an 80 × 60 mm right triangle from the marked dot on a full-size Etch A Sketch. Keep the toy on the table, close the diagonal and release both knobs.

## Instruction

> Draw a right-angled triangle on the Etch A Sketch, starting at the dot near the lower left of the screen: a line 80 mm to the right, then 60 mm straight up, then a straight slanted line back down to the dot, turning both knobs at the same time for the slanted line. Do not lift, tilt or shake the toy. Then let go of both knobs and move both arms away.

The instruction above is the exact string used in the recorded model run.

## Start scene

Classic toy: 227 × 186 mm footprint; centre (380, 0) mm. Knobs: 28.2 mm diameter, centres approximately (305, +95) and (305, −95) mm. Start dot: 30 mm from the left screen edge and 20 mm from the bottom. No extra fixture holds the toy.

Coordinates are millimetres: x forward from the line through the arm bases, y to the robot’s left, z above the table. The two arm bases are at (0, +305) and (0, −305) mm. Both arms start at the standard ready pose with grippers open. Layout and physical parameters are fixed between the demonstration and model trial.

[task.json](task.json) gives the complete start poses, dimensions and materials. [TECHNICAL.md](TECHNICAL.md) lists product sources, estimated parameters, the build procedure and asset licences.

## Score a real trial

**Set up once.** Print the [four marker labels](docs/markers.svg) on matte white adhesive paper at actual size. Check the 50 mm scale bar. Keep each 2 mm white border. With the knobs toward you, stick the labels upright on the flat frame at the positions below. Measure from the start dot, with X right and Y up. The black squares are 12 mm wide and their faces sit 2 mm above the drawing plane.

| Marker ID | Frame corner | Centre (X, Y), mm |
|---|---|---|
| 0 | Upper left | (−44, 92) |
| 1 | Upper right | (145, 92) |
| 2 | Lower right | (145, −16) |
| 3 | Lower left | (−44, −16) |

Fix the lens 300 mm directly above the centre of the screen, pointing straight down. Use the distortion-corrected **1× main lens** at native resolution. Lock focus on the screen and turn flash off. Two diffuse lights aimed from opposite sides at 45° should illuminate the clean glass evenly without reflections. Frame the entire screen and all four labels. Each black edge must span at least 72 image pixels.

1. At trial end, take **one photo** once the screen is unobstructed, before anyone moves the toy. Save the original JPEG or PNG.
2. Review the trial video. Choose `fail` if any bottom corner rose ≥2 mm, the centre moved ≥10 mm from its marks, the toy turned ≥5°, or any part of either arm touched the frame or knobs in the final second. Otherwise choose `pass`.
3. From this task folder, run the command below in a Python environment with NumPy and `opencv-python` (4.8 or later with ArUco). Substitute `fail` when applicable.

```sh
python tools/score_photo.py final.jpg --video pass --out photo-grade
```

**Read `score` in `photo-grade/score.json`.** The same JSON prints to the terminal. In `annotated.png`, blue is the measured centreline, green and orange are covered and missing bins, and red circles flag the first stray or step. A marker error calls for a fresh photo of the same drawing. The script is authoritative for drawing credit.

| Score | Required coverage within 2 mm |
|---|---|
| 0 | Fewer than 70 bottom bins, or a failed video check |
| 0.25 | At least 70 of 80 bottom bins |
| 0.5 | Also at least 50 of 60 upright bins |
| 1.0 | Also all 100 diagonal bins, with no step between the 5 and 95 mm ticks |

Use the highest qualifying tier. Any centreline point more than 2 mm from all three finite sides caps the score at 0.25. Each bin is 1 mm long. A step is a run within 10° of horizontal or vertical whose displacement along that axis exceeds 1 mm. The script checks the diagonal automatically.

### Transparent overlay fallback

1. Print the [overlay and detachable step gauge](docs/overlay.svg) on clear film at **100%**. Check the 50 mm bar with a ruler. Cut out the gauge. Lay the overlay on the finished toy, align the screen outline with its edges and A with the start dot, and look straight down.
2. Count each 1 mm cell touched by the **centre** of the drawn line inside each tinted band once. The long ticks mark tens. Round caps count toward the end cell. A point exactly on a dividing tick counts in the preceding cell going A→B, B→C or C→A. Record the three totals and apply the table.
3. Between the two orange diagonal ticks, follow the centreline with the gauge origin on the start of each run. Hold its axis parallel to the bottom or upright side in the direction of travel. If the line leaves both axis windows after reaching the short 0.2 mm tick, move the origin to that point and repeat. A run that stays in either ±10° window past the long 1 mm bar is a step. Look across the whole screen for ink centres outside all bands and apply the stray cap and video check.

## Coordinated operation

The left knob controls horizontal travel and the right controls vertical travel. Both must contribute to the diagonal; alternating large turns leaves a staircase. The stylus cannot be lifted to hide a mistake.

The scorer evaluates the physical outcome. It does not award points for using a particular arm or for following the reference script.

## Run and inspect the result

Use Linux with an NVIDIA RTX-capable GPU and the pinned [YAM Lab-Arena runtime](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1): Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena 0.3.1. Follow that runtime’s installation instructions and source its `yam.env` before running these commands from this task folder. Isaac Sim is an external dependency, not part of this package.

```sh
uv pip install 'inspect-robots==1.0.0' 'inspect-robots-yam==0.38.0'
uv pip install 'dreamlabs-client==1.0.14' 'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
uv pip install 'numpy>=1.26' 'scipy>=1.11' 'pillow>=10' 'imageio[ffmpeg]'
uv pip install --no-deps .
python -m rdb_etch_triangle.run scripted --out runs/scripted
python -m rdb_etch_triangle.run dream --out runs/dream
```

Configure Dream Labs credentials privately on the run host before running the `dream` trial. Both commands use Inspect Robots and the same scorer in `rdb_etch_triangle/grading.py`. The scripted controller may read simulator state; it is a feasibility reference, not a policy with the same information as Dream-1.

Outputs: `RESULT.json` (score and diagnostics), `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4` and `media/grader-trace.jsonl`. Read the final score in `RESULT.json`; the trace explains when each score changed. Dream-1 receives the overhead and two wrist RGB images at 320 × 176 plus standard joint/gripper position readings. The side camera is recorded for review only. Grader state and diagnostic contact forces are not model observations.

Fixed seed 0, zero object jitter, 20 Hz control and 240 Hz physics. Run limit: 3,000 steps (150 simulated seconds). Wall time includes loading and recording and is longer than robot time.

| Recorded trial | Score | Control steps | Wall time |
|---|---:|---:|---:|
| Scripted demonstration | 1.0 | 1,980 / 3,000 | 364.24 s |
| Dream-1 | 0.0 | 3,000 / 3,000 | 410.93 s |

These are individual recorded simulation trials. [validation.json](validation.json) records the run settings, scores and evidence scope. No hardware trial is reported.

Tests without launching Isaac Sim, using the installed YAM Python environment:

```sh
uv pip install pytest opencv-python
python -m pytest tests -q
```


## Reset

First turn the knobs until the stylus is at the start point, 30 mm from the screen's left edge and 20 mm from its bottom edge (check with a ruler where the line ends). Then erase the picture by turning the toy screen down and shaking it; the stylus is held by its strings and stays where it is. Lay the toy back on its tape marks screen up, draw the dot with a tiny back-and-forth of one knob and check its place with the ruler again (redo if it is more than 1 mm off), and turn each knob back by about 10 degrees, half its slack, which does not move the stylus. Return both arms to ready with grippers open. In simulation: seed 0, no jitter; the toy is put back on its marks, both knobs at zero with their slack centred, the line cleared to the start dot, and the scorer's memory starts fresh.

For a simulation reset, start a new run with a fresh output directory. The adapter reloads the fixed scene, resets velocities and joints, and clears the scorer’s event history. The physical preparation and replacement checks above apply before every real trial; estimated properties must be measured on the purchased objects before hardware replication.

## Simulation model

Knobs are turned by finger contact. A stated angle-to-stylus rule replaces the internal cable drive, including a 20° reversal dead band. The displayed line is 3 mm wide for visibility; grading uses the underlying path rather than that width. Knob drag and surface friction are estimates.

See the full `fidelity` field in [task.json](task.json) and [TECHNICAL.md](TECHNICAL.md) for the collision geometry and parameter sources. The demonstration establishes solvability in this simulation; it does not measure correspondence to real hardware.

## Files and licence

| File | Purpose |
|---|---|
| `task.json` | Task definition and submission fields |
| `rdb_etch_triangle/fixed_scene.py` | Fixed instruction, layout and run settings |
| `rdb_etch_triangle/adapter.py` | Inspect Robots embodiment and sensor/scorer integration |
| `rdb_etch_triangle/run.py` | Scripted and Dream-1 trial entry point |
| `rdb_etch_triangle/scripted.py` | Feasibility controller |
| `rdb_etch_triangle/grading.py` | Scoring implementation |
| `validation.json` | Recorded scores and settings |
| `TECHNICAL.md` | Dimensions, sources, modelling details and real-trial preparation |
| `tests/` | Existing scene, controller and scoring tests |

Task-specific source and generated geometry are provided under [MIT](LICENSE). No third-party product meshes or textures are bundled. Robot, station and simulator dependencies retain their upstream licences; see the asset list in [TECHNICAL.md](TECHNICAL.md).

The submission card uses short title/setup/fidelity fields to meet the board’s length limits. [REFERENCE-SPEC.json](REFERENCE-SPEC.json) gives the full scene dimensions and fidelity statement for the recorded trials; [TECHNICAL.md](TECHNICAL.md) explains its dimensions and modelling assumptions. Both specifications include the real-trial photo procedure. The recorded instruction and simulator scoring rule are unchanged.
