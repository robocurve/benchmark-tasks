# Clamp two stacked boards with a one-hand bar clamp

Clamping a glue-up, the way it is done at a carpentry bench or in a maker space. Two pine boards lie stacked face to
face on two spacers, so the middle of the stack overhangs a 3 cm gap. A red one-hand bar clamp stands next to them.
One hand holds the clamp by its handle and slides its fixed jaw under the overhang, then lifts it against the bottom
board. The other hand pushes the sliding jaw down its bar onto the top board and squeezes. The sliding jaw rides a
one-way ratchet, so it stays where it is pushed and the squeeze stays in the clamp. Let go, and the clamp must hang on
the boards by its jaws, still pressing them with at least 20 N. The stack must not shift: the boards' edges stay
aligned within 3 mm.

| File | What it is |
|---|---|
| `bar_clamp_boards.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `bonds.py` | `Compound`: one rigid body made of several box colliders (the clamp's bar, fixed jaw and handle). `Ratchet`: a box on a prismatic joint held at a stop (an upper limit and a stiff spring) that is moved at run time (the sliding jaw), or left at its stop as a plain spring (the fixed jaw's pad). The `on_step`, `on_reset` and `on_spawn` hooks |
| `fingers.py` | Where the grippers' fingers are: points on their collision meshes, from the robot's `tip_left` and `tip_right` bodies |
| `simready.py` | NVIDIA SimReady props, downloaded at a pinned revision (the mallet and the speed square) |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`*.usdc`) and their textures, and `make_looks.py`, the Blender script that models and exports them |
| `test_bar_clamp_boards.py` | Offline checks: the file loads, the layout, every scoring outcome, the ratchet's travel, the finger check, the demo's order, layout and reach (no GPU) |
| `task.json` | The task's details for the board |

## The scene

The SimReady collection has no bar clamp (`tool_clamp_c01` is a screw C-clamp, `tools_clamp_large_a01` and
`tools_clamp_extraLarge_a01` have no root USD) and its `misc_boards_wooden_a01` is a 58 cm board, so the clamp, the
boards, the spacers and the mat are `yam_arena` boxes dressed in meshes modelled in Blender. The rubber mallet and the
speed square are SimReady props, fixed, for the workbench.

| Part | Collider | Physics |
|---|---|---|
| mat | 40 x 46 x 0.3 cm box, fixed | |
| spacers | two 4 x 10 x 3 cm boxes, fixed, under the boards' ends | 11 cm clear between them under the stack |
| boards | two 20 x 10 x 1.8 cm boxes, 180 g each, free | stacked on the spacers |
| clamp | one rigid body, 130 g, free: the bar (2 x 0.8 x 9 cm), the fixed jaw (10.6 x 2.4 x 1.7 cm) and the handle's grip and foot (10 x 3 x 3 cm in all) (`bonds.Compound`) | the jaws and the foot slide at friction 0.3, the grip holds at 1.0 |
| fixed jaw's pad | one 2 x 5 x 0.5 cm box, 20 g, on a prismatic joint along the clamp's up axis, its top 3 mm over the jaw's own face (`bonds.Ratchet` left at its stop) | a spring of 40 N per mm and 50 N s/m, 3 mm of travel: see below |
| sliding jaw | one 11.1 x 3.2 x 2 cm box (with its collar round the bar), 30 g, on a prismatic joint along the bar (`bonds.Ratchet`) | the ratchet and the trigger: see below |
| mallet, speed square | SimReady `handTool_hammer_rubberMallet_a01` and `measurement_ruler_speedsquare_a02`, fixed | |

**The clamping force.** The fixed jaw's black rubber pad is a spring: it gives way along the clamp's up axis when the
stack is pressed on it. It stands for the bar's flex and the pads' rubber together. It can only be pressed in by
something pushing on it, so its compression times 40 N per mm is the force the jaws press the stack with (the stack
stands still, so the sliding jaw presses it with the same force). The scorer reads it every step from the poses of the
pad and the clamp. 3 mm of travel is 120 N, after which the jaw's own face takes the load.

**The ratchet.** The sliding jaw's joint has an upper limit (its stop) and a stiff spring (10,000 N/m) that holds the
jaw at the stop. Every control step the task file measures how far the jaw has been pushed along the bar, and once it
is 0.1 mm past its stop, the stop and the spring's target move down to it. So the jaw can be pushed toward the fixed
jaw (about 1 N a click) or the bar pulled up through it, but it never slides back, and the clamp can hang from the
boards by its sliding jaw. Every reset puts the stop back at the start, 6 cm above the fixed jaw.

**The trigger.** A one-hand clamp is closed by squeezing its trigger. Here, while a finger is within 4 mm of the
sliding jaw and the jaw is on the top board (gap 0.5 mm or less, the clamp round the stack), the stop goes on down
below the jaw each control step: 0.3 mm a step while the clamp's bar is still rising to bring the pad up against the
bottom board, then 0.05 mm a step once the pad is pressed in, which is 2 N more each step, up to 2.5 mm of the pad's
travel (100 N). The limit drives the jaw on against the stack, the pad gives way, and the ratchet keeps it there. The arms cannot press a spring of 40 N per mm by themselves: in the runs they pushed 5 to 10 N for 1 to 2 cm of
position error. So the force a trial reaches depends on the trigger being pressed long enough, and it stays in the
clamp when the fingers leave. A clamp that is merely hung on the boards, or whose
trigger is let go too soon, is not pressing the stack, and the pad's compression says so.

Each box keeps its collider, and `looks.py` draws a mesh in its place:

| Box | Mesh |
|---|---|
| `clamp` | the steel bar with its end stop, the red fixed jaw with its black pad, the red handle housing with the black ribbed grip |
| `sliding_jaw` | the red jaw with its collar round the bar, its black pad and a black thumb pad at its tip |
| `board_bottom`, `board_top` | pine boards with their grain |
| `spacer_near`, `spacer_far` | walnut blocks |
| `mat` | dark grey rubber with a 1 cm grid |

The scored parts' meshes fit inside their boxes (a test checks it), so their centres, which the scorer reads, are their
boxes' centres. To change the meshes, edit `looks/make_looks.py` and run it in Blender 4.2 or later:
`blender -b -P looks/make_looks.py`.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 1,200 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=bar_clamp_boards.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Slide the red bar clamp's lower jaw under the two stacked pine boards, push its sliding jaw down and squeeze until both jaws hold the boards firmly, then let go so the clamp hangs on the boards by itself." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/bar-clamp-boards-dream

# Scripted demo (both arms), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=bar_clamp_boards.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Slide the red bar clamp's lower jaw under the two stacked pine boards, push its sliding jaw down and squeeze until both jaws hold the boards firmly, then let go so the clamp hangs on the boards by itself." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/bar-clamp-boards-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed. Keep it
  at 0: jitter would move the sliding jaw off its joint.
- The SimReady props are downloaded once into `~/.cache/simready` at the pinned revision `9d4556bfe808` (CC-BY 4.0).
- The layout keeps the arms apart: the stack lies on the right arm's side, the clamp's handle on the left arm's. The
  left arm grips the handle 9 cm behind the bar and the right arm pushes the sliding jaw 8.6 cm in front of it, 17 cm
  apart, and the arms' closest links stay 12.9 cm apart.
- The demo: the left arm grips the handle from above (fingers across it), slides the standing clamp 10 cm along the mat
  so its fixed jaw goes under the stack's overhang, and lifts it until the fixed jaw is near the bottom board. The right
  arm, closed fingers, pushes the sliding jaw 2.4 cm down its bar onto the top board and keeps its fingers on it: the
  trigger. The ratchet takes up the slack (the bar rises, the pad meets the bottom board) and then builds the force, 2 N
  a step. The left arm lets go while the trigger is pressed, and the right arm goes up. The clamp hangs on the boards
  and presses them with 71 N. The demo reaches 1.0 and ends on success at step 327 (16.4 s at 20 Hz). A trial
  takes about 2 minutes on an L40S, including startup.
- A shortcut scores what the text says: the same demo with the right arm taking its fingers off the sliding jaw after 1
  s instead of 4.6 s (`PRESS_HOLD = 1.0`, `scripts/task-ablate.sh bar-clamp-boards notrigger PRESS_HOLD=1.0`) leaves
  the clamp round the stack without the 20 N and scores 0.25.
- Dream-1: 0.0 (ran to the step limit without finishing). (Its one run on the earlier version, which scored on gaps and not on force,
  scored 0.0 and does not apply to this scorer.)
- Videos: `inspect-robots video ~/yam-eval/bar-clamp-boards-demo/<log>.json` writes one MP4 per camera next to the
  frames.
- Offline checks: `python -m pytest -q test_bar_clamp_boards.py`.

## Scoring

All positions are in metres in the station frame (x forward, y to the robot's left, from the midpoint of the arm
bases). The pads' points are worked out from the pads' and the sliding jaw's centres and turns, the boards' faces from
theirs, so everything follows the parts as they move.

- **Clamping force:** the fixed jaw's pad compression times 40 N per mm (see the scene), read every step. The pad can
  only be pressed in by the stack, so a pad that touches nothing, or only rests on the stack, reads 0 N.
- **Gaps:** each jaw's gap is the least distance, along the board's normal, from its pad (the pad's two ends and its
  centre, a 5 cm pad) to the board face it grips: the fixed jaw's pad to the bottom board's underside, the sliding
  jaw to the top board's top. A clamp that tilts touches with one end of a pad, as a feeler gauge finds it.
- **Round the stack:** both pad centres 1 cm or more inside the boards' outline, the fixed jaw under the stack and the
  sliding jaw over it (neither more than 3 mm into its board).
- **Gripping:** round the stack, both pads on their boards (gaps 0.5 mm or less) and a clamping force of 20 N or more.
  A gap alone never counts as gripping: it says where the jaws are, not whether they press.
- **Let go:** no point of any gripper finger within 1 cm of the clamp or its sliding jaw (the fingers are points on
  the robot's finger collision meshes, `fingers.py`).
- **Still:** the clamp and both boards each moved less than 0.5 mm and turned less than 0.01 rad in the last 0.5 s.
  (The simulator's velocity is not used: a stack pressed with tens of newtons reports about 1 cm/s while it stays put.)
- **0** if a board or the clamp has fallen off the table (centre more than 2 cm below the top) or the state is not
  finite.
- **1.0 (success, held 1 s):** gripping with 20 N or more, the pads' centre within 3.3 cm of the boards' centre along
  their length (the middle third), the two boards' matching corners within 3 mm of each other (the stack did not shift
  apart), let go, the clamp hanging on the boards by its jaws (its lowest point 1 mm or more above the mat) and still.
- **0.5:** gripping with 20 N or more, but off the middle third, the boards misaligned by 3 mm or more, still held,
  resting on the mat, or still moving.
- **0.25:** round the stack, but pressing with less than 20 N or a pad more than 0.5 mm off its board (the sliding jaw
  not pushed home, the fixed jaw not up against the bottom board, or the trigger not pressed long enough).
- **0** otherwise, including doing nothing or closing the clamp beside the stack.

**Why 20 N.** It is about 15 times the clamp's weight and almost 6 times the weight of the two boards (3.5 N), so a
clamp that only hangs or rests on the stack cannot reach it. With the 0.3 to 0.5 friction of pine on pine it keeps the
top board from sliding against a sideways push of 6 to 10 N, a few times what a nudge while the glue is spread takes. A
real one-hand clamp of this size gives 100 N and more, so 20 N is a floor that a properly squeezed clamp passes with
ease.

**How to check it in a real trial.**

1. **Force:** put a thin flat force sensor (a load cell or a force-sensing film, 0 to 200 N, 3 mm thick or less) between
   the sliding jaw's pad and the top board before the run, and a shim of the same thickness in the runs without it.
   One second after the grippers are clear, it must read 20 N or more, and keep reading it for 1 s. The clamp presses
   the stack with the same force on both pads, so one sensor is enough. A calibrated pressure-indicating film between
   the pad and the board gives the same reading as a colour.
2. **No sensor:** a spring scale hooked on the top board's short edge and pulled along the boards' length must not move
   the top board below 6 N (the friction of 20 N at 0.3), and a clamp that merely rests on the stack lets the
   board go at a much lower pull. A card slid under the handle shows the clamp hangs off the bench.
3. **The rest:** a feeler gauge (0.5 mm) under each pad, a tape measure for where the clamp sits along the boards, a
   square across their edges for the alignment, a dial gauge or a mark and a stop watch for "still" (under 0.5 mm in
   0.5 s) and the video for "let go".
