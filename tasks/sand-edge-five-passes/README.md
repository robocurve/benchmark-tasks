# Sand a workpiece's edge: five full passes, block flat

Finishing a part in a fab lab or a woodshop: a laser-cut or sawn plywood part is clamped in a bench vise and its edge
is sanded smooth with a sanding block, stroke after stroke along the whole edge. The block has to stay flat on a 3 mm
edge for the whole stroke, and pressed only as hard as the vise's grip allows: tilted, it rounds the edge over, and
pushed too hard, the part slips in the jaws. It is sustained contact with a controlled normal force, counted in
strokes, which pick-and-place data doesn't have.

| File | What it is |
|---|---|
| `sand_edge_five_passes.py` | The `yam_arena` task file: scene, the pass counter (`on_step`), scoring (`success`, `score`) and the scripted `demo()` |
| `simready.py` | `SimReady`: an NVIDIA SimReady prop in a task file, pinned to one dataset revision, its jointed bodies merged into one, and named parts moved (`shift`, used to close the vise's sliding jaw onto the workpiece) |
| `bonds.py` | `Bonded`: a box held by a breakable PhysX joint (the workpiece in the jaws), and the `on_step`, `on_reset` and `on_spawn` hooks |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`*.usdc`), their textures, and `make_looks.py`, the Blender script that models and exports them |
| `test_sand_edge_five_passes.py` | Offline checks: the file loads, the layout, the pass counter, every scoring outcome and its memory, the looks fit their boxes, the demo's strokes and reach (no GPU) |
| `task.json` | The task's details for the board |

## The scene

Positions are centres in metres from the midpoint between the arm bases (x forward, y to the robot's left), heights
above the table top.

| Part | What | Physics |
|---|---|---|
| vise | NVIDIA SimReady `equipment_bench_vise_a01` (CC-BY 4.0, revision `9d4556bfe808`), scaled to 0.6: a blue 3-inch bench vise, 8.5 cm jaws, their tops 10.8 cm above the table, its handle away from the robot | fixed (kinematic), its five bodies merged into one, colliding as signed distance fields. Its sliding jaw, screw and handle are moved 3.6 cm (in the file, 2.2 cm at this scale) toward the fixed jaw, so the jaws stand 6 mm apart |
| workpiece | 3 mm birch plywood, 20 x 7 cm, 30 g, standing in the jaws along y, from y = -0.31 to -0.11 at x = 0.34, its top edge 3 cm above the jaws (13.8 cm above the table) | one box, 1.5 mm of play to each jaw, held by a PhysX fixed joint that breaks under 30 N or 2 N m (`Bonded`): pressed or levered too hard, it slips in the jaws and drops onto the vise's screw 1 cm below. Friction 0.6 static, 0.5 dynamic against anything (sandpaper on plywood) |
| sanding block | orange foam, 10 x 7 x 3 cm, 60 g, grey sandpaper on its underside, lying flat at (0.25, -0.35), its long side along y | one free box, the bench's rubber-like friction (1.2) |
| speed square | NVIDIA SimReady `measurement_ruler_speedsquare_a02` scaled to 0.6 (17 cm), at (0.24, 0.24) | fixed, decoration on the left |
| safety glasses | NVIDIA SimReady `ppe_eyeProtection_safetyGlasses_a01`, at (0.42, 0.14) | fixed, its bodies merged, decoration on the left |

The SimReady packages are downloaded at the pinned revision the first time the scene is built, into
`~/.cache/simready`. The workpiece's and the block's meshes are modelled in Blender (`looks/make_looks.py`, Blender
4.2 or later: `blender -b -P looks/make_looks.py`), fit inside their boxes (a test checks it), and their textures are
drawn in the script.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sand_edge_five_passes.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sand the top edge of the plywood piece in the blue vise with the orange sanding block: five full passes along the whole edge, keeping the block flat on the edge." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/sand-edge-five-passes-dream

# Scripted demo (right arm), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sand_edge_five_passes.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sand the top edge of the plywood piece in the blue vise with the orange sanding block: five full passes along the whole edge, keeping the block flat on the edge." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/sand-edge-five-passes-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
- The layout keeps the arms apart: the vise, the workpiece and the block are on the right arm's side, and the left
  arm holds still at its start pose on its own side for the whole demo.
- The demo (right arm only): a straight-down grip with the wrist turned a quarter turn, so the fingers close across
  the block's 7 cm, 2.5 cm deep, with a partial grip (about 27 N per pad instead of the full 50 N: the light block
  chatters against the edge in a full grip). It lifts the block over the vise, lowers it flat onto the edge's right end, and sweeps
  five strokes along y, each turning 1 cm past an end of the edge, its TCP sent 6 mm below where the face meets the
  edge so the arm leans on it. In the simulation the arm presses with about 3.5 N (measured with the workpiece on a
  stiff spring in a probe), the block stays within 2.3 degrees of flat, and the workpiece moves under 0.1 mm. The
  arms never come closer than 39 cm, and no part moves faster than 0.22 m/s. The fifth pass counts at about step
  400 and the demo ends on success at step 421 (21 s at 20 Hz). A trial takes about 2 minutes on an L40S with the
  window open (`-E headless=false`), including startup.
- Pressing hard scores 0: the same demo with the TCP sent 5 cm below the edge instead of 6 mm (`LEAN = 0.05` in
  the task file) rocks the block 40 degrees in the grip, so it digs into the edge (0 from about 8.4 s), and then pushes the workpiece
  down 2.6 cm in the jaws.
- Videos: `inspect-robots video ~/yam-eval/sand-edge-five-passes-demo/<log>.json` writes one MP4 per camera next to
  the frames.
- Offline checks: `python -m pytest -q test_sand_edge_five_passes.py`.

## Scoring

All positions are centres, in metres, in the station frame (x forward, y to the robot's left), and heights are above
the table top. The block's sanded face is its underside (the sandpaper). The scorer follows the edge's top along its
middle line and the block's face from the two parts' centres and turns, every control step (20 Hz).

- **On the edge:** some point of the edge's top lies under the sanded face (within its 7 x 10 cm) and within 1 mm of
  it, above or below.
- **Flat:** the face's normal within 10 degrees of straight down.
- **A pass:** while on the edge and flat at every step, the block's centre goes from within 1 cm of one end of the
  edge (or past it) to within 1 cm of the other end (or past it). Lifted, tilted 10 degrees or more, or off the edge
  for one step, the stroke under way is lost. Strokes in both directions count.
- **Slipped:** the workpiece's centre or either end of its top edge 5 mm or more from where it started. Pressed or
  levered too hard, the workpiece slips in the jaws.
- **Dug in:** some point of the edge's top within 1 mm of the block, on any face, with the face's normal 25 degrees
  or more from straight down, for 0.5 s without a break (10 steps).
- **Score:** 0.2 per pass, up to 1.0 with five (success, held 1 s). 0 for the rest of the trial once the workpiece
  has slipped or the block has dug in, and 0 if the state is not finite. The passes and the failures are remembered
  for the whole trial.

In a real trial, a tally of strokes from the video, a protractor on a photo of the block and a ruler against the
vise's jaw give the same numbers.
