# Clamp two boards edge to edge with a one-hand bar clamp

Clamping a glue joint, as carpenters do every day: two pine boards lie flat on the bench, edge to edge, to be glued
into a panel. A red one-hand bar clamp lies flat in front of them, its jaws open. The robot takes the clamp by its
pistol grip and trigger, slides it so its jaws go round the boards' outer edges, and pumps the trigger: each squeeze
drives the bar a few millimetres through the handle, the brake holds it there, and once the jaws meet the boards the
next squeezes press them together. Let go, the clamp must hold the joint shut with at least 20 N on its own, the
boards still aligned.

| File | What it is |
|---|---|
| `bar_clamp_boards.py` | The `yam_arena` task file: scene, the clamp's mechanism (brake and drive plate), scoring (`success`, `score`) and the scripted `demo()` |
| `bonds.py` | The clamp's bodies and joints: `Compound` (one rigid body of several box colliders, drawn as its coloured parts), `Ratchet` (a box on a prismatic joint held at a stop that can be moved at run time: the fixed jaw's pad spring), `Slide` (a compound on such a joint: the bar and its jaw), `Hinge` (a box on a revolute joint with a torsion spring: the trigger) and `one_way` (a joint that only pushes: the trigger's drive on the bar). The `on_step`, `on_reset` and `on_spawn` hooks |
| `fingers.py` | Where the grippers' fingers are: points on their collision meshes, from the robot's `tip_left` and `tip_right` bodies |
| `simready.py` | NVIDIA SimReady props, downloaded at a pinned revision (the mallet and the speed square) |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The mat's and the boards' meshes (`*.usdc`) and textures, and `make_looks.py`, the Blender script that models and exports them |
| `test_bar_clamp_boards.py` | Offline checks: the layout, the mechanism's numbers, every scoring outcome, the 1 s hold, the finger check, the demo's geometry and reach (no GPU) |
| `task.json` | The task's details for the board |

## Who does this, and why it matters (category: Impact on jobs)

Carpenters (O\*NET 47-2031.00, about 890,000 US workers) "assemble and fasten materials ... using hand tools and wood
screws, nails, dowel pins, or glue" (O\*NET task statement). Every glued joint is clamped until the glue sets, and
edge-joining boards into a wider panel (a shelf, a tread, a countertop, a cabinet side) is one of the most common glue
joints in finish carpentry and on site. A one-hand bar clamp is the carpenter's standard clamp for it, because the
other hand stays free to hold the work: the clamp is operated by squeezing its trigger, stroke by stroke, and the force
it holds depends on how far it was squeezed.

## The clamp, and how it is modelled

The clamp is modelled on a 6 in one-hand mini bar clamp, the IRWIN QUICK-GRIP mini bar clamp: 6 in (152 mm) maximum
opening, 2-7/16 in (62 mm) throat, rated 140 lb (623 N), steel bar, resin body, non-marring pads. Its mechanism
(as in the QUICK-GRIP patents): the bar slides through the handle; the trigger pivots in the handle in front of the
pistol grip; squeezed, it tilts a drive plate that grips the bar and pushes it, so the moving jaw on the bar's far end
closes on the fixed jaw on the handle; on the way back the drive plate slips along the bar, and a brake plate keeps
the bar from sliding back open (a release lever frees it).

| Part | Model | Physics |
|---|---|---|
| handle | one rigid body, 0.12 kg: the housing round the bar (59 mm along the bar), the fixed jaw on its front (62 mm throat), the pistol grip (88 x 25 x 22 mm, rubber) pointing at the robot | housing and jaw slide on the mat at friction 0.3, the grip holds at 1.0 |
| fixed jaw's pad | a 40 x 20 mm black pad, 5 mm thick, 10 g, on a prismatic joint along the bar, its face 3 mm proud of the jaw (`Ratchet` left at its stop) | a spring of 40 N per mm, 3 mm of travel (120 N), then the jaw's own face takes the load: the force gauge (below) |
| trigger | a 74 x 12 x 16 mm red lever, 15 g, on a revolute joint in the housing beside the bar (`Hinge`), turning 0 to 0.40 rad (23 degrees) toward the grip, where its tip comes within 2 mm of the grip | a torsion spring: 0.32 N m held open (8 N at 40 mm from the pivot), 0.46 N m fully squeezed |
| bar and moving jaw | one rigid body, 0.17 kg (`Slide`): the steel bar, 16 x 4.8 x 231 mm, and the moving jaw (62 mm throat, black pad) on its far end, on a prismatic joint through the housing | the brake (below) |
| drive plate | not a body: a one-way joint (`one_way`) between the trigger, 9 mm from its pivot, and the bar | below |

**The trigger drives the bar.** The trigger's drive arm, 9 mm from its pivot, pushes the bar along itself through a
joint that only pushes: the arm's point cannot pass a stop on the bar, and is free in every other way. Squeezing the
trigger by an angle *a* drives the bar 9 mm x sin *a* toward the fixed jaw, 3.5 mm for a full stroke. The force goes
through the joint both ways: the bar's resistance (the work, once the jaws touch it) comes back to the trigger as a
torque, so the finger must push harder as the clamp tightens. A finger in the middle of the trigger, 36 to 50 mm from
the pivot, has a lever of 4 to 5.5 to 1: the robot's 50 N finger can press the jaws with up to about 150 N, and a
hand's 150 N squeeze gives the clamp's rated 623 N. A finger that only rests on the trigger, or presses it less than
its spring's 8 N, does not move it at all.

**The drive plate slips back, the brake holds.** Every control step (20 Hz) the task file moves two stops, as the
plates' grip does: the drive's stop follows the trigger's arm back along the bar when the trigger springs open (so the
next squeeze pushes from there), and the brake's stop (the bar's prismatic joint's upper limit, with a soft 0.3 N/mm
spring) follows the bar toward the fixed jaw, never back. Both move in steps of 0.1 mm, finer than a real plate's
grip needs. Nothing else moves the bar: it moves only when the trigger is squeezed or when something pushes it shut,
as on the real clamp (the bar of a QUICK-GRIP slides freely shut by hand).

**The clamping force** is read from the fixed jaw's pad: its compression along the bar times 40 N per mm. The pad can
only be pressed in by something pushing on its face, so a pad that touches nothing, or only rests on a board, reads
0 N. The spring stands for the jaws' and bar's flex and the pads' rubber together. 20 N is half a millimetre of it.

The SimReady collection has no bar clamp (`tool_clamp_c01` is a screw C-clamp, `tools_clamp_large_a01` and
`tools_clamp_extraLarge_a01` have no root USD), so the clamp is built from boxes and drawn as its coloured collision
boxes. The boards are `yam_arena` boxes dressed in pine meshes made in Blender (`looks/make_looks.py`, run with
`blender -b -P looks/make_looks.py`); the meshes fit inside their boxes (a test checks it), so their centres, which the
scorer reads, are their boxes' centres. The rubber mallet and the speed square are SimReady props, fixed, for the
workbench.

| Object | Size, material | Where (centre, m, station frame) |
|---|---|---|
| mat | 46 x 40 x 0.3 cm, dark grey rubber, fixed | (0.34, -0.02) |
| right board | 20 x 6 x 1.8 cm pine, 108 g, free, long side along x | (0.429, -0.090) |
| left board | the same, its long edge 2 mm from the right board's | (0.429, -0.028) |
| clamp | lying flat, its bar along y, its jaw tips 2.2 cm in front of the boards' near ends (x 0.307 and 0.329), the fixed jaw's pad face at y -0.125 (5 mm off the right board), the moving jaw's face at y 0.015 (13 mm off the left board), opened 14 cm; grip and trigger toward the robot | handle centre (0.2185, -0.1575), grip centre (0.174, -0.175) |
| mallet, speed square | SimReady `handTool_hammer_rubberMallet_a01` and `measurement_ruler_speedsquare_a02`, fixed | (0.585, 0.33), (0.58, -0.40) |

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 1,200 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=bar_clamp_boards.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clamp the two pine boards together edge to edge with the red bar clamp, then let go of the clamp." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/bar-clamp-boards-dream

# Scripted demo (the right arm works the clamp, the left arm films it), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=bar_clamp_boards.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clamp the two pine boards together edge to edge with the red bar clamp, then let go of the clamp." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/bar-clamp-boards-demo
```

- The run prints `task_score` at the end, and a `[bar_clamp]` line every 0.5 s with the trigger's angle, the bar's
  travel, the clamping force, the jaws' and the joint's gaps. With `-E object_jitter=0` the layout is fixed and no seed
  is needed. Keep it at 0: jitter would move the clamp's parts off their joints.
- The SimReady props are downloaded once into `~/.cache/simready` at the pinned revision `9d4556bfe808` (CC-BY 4.0).
- The demo (open loop: fixed TCP waypoints, nothing read from the simulation). The right arm holds the clamp as a
  hand does: the fingers go down on either side of the grip and trigger, 106 mm in front of the fixed jaw; the back
  finger rests on the grip's back like a palm and stays there, and the front finger works the trigger 50 mm from its
  pivot like an index finger (the TCP moves half the fingers' opening, so only the front finger moves). The first
  squeeze holds the clamp while the arm slides it 8 cm forward, its jaws round the boards' near end. Then it pumps the
  trigger twice more: open to 90 mm (the trigger springs fully back, the drive plate slips back along the bar), close
  to 40 mm (the front finger squeezes the trigger to its stop, 3.5 mm of bar). The bar takes up the slack, pushes the
  left board onto the right one and both onto the fixed jaw's pad, and the last stroke presses the pad in. It lets go
  and lifts away. The left arm touches nothing: it holds its wrist camera 28 cm above the panel's near end for a close
  view of both jaws, the bar and the joint (its fingers 9 cm above the mat, over the left board). On Isaac: **1.0 at
  step 250** (12.5 s). The squeeze that held the clamp for the slide and the first stroke took up the slack; on the
  second stroke the jaws closed on the panel and the force rose from 2 N to 89 N as the trigger went from 0 to 23
  degrees; after the release the clamp held 87 N, the joint closed, the boards' ends 0.4 mm apart, and every condition
  held for 1 s.
- The demo video (`demo.mp4` on the board) shows the top camera beside the left wrist camera's close view of the
  panel's near end (brightened): the moving jaw, the bar and the fixed jaw closing on the boards and the joint between
  them shutting. Under both runs the scorer's readout for that step: the trigger's angle, how far the bar has moved,
  the joint's gap, the clamping force and the scored conditions. The strokes are then replayed at half speed. The
  trigger itself sits between the right fingers, hidden from both cameras: the fingers' squeezes are seen, the
  trigger's angle is read out.
- The check that the trigger is needed: `BAR_CLAMP_DEMO=by_hand` (an environment variable) runs the same start and
  slide, then the right arm lets go and the left arm pushes the moving jaw shut with its closed fingers, 25 mm past
  where the jaws meet the panel, without touching the trigger. Run on Isaac (420 steps): the jaws closed on the panel
  and the joint shut, but the clamp and the boards slid along the mat ahead of the fingers, and the pad read at most
  1.3 N (1.0 N once the arm had backed off): score 0.5, closed but under 20 N. Pushing the jaw shut builds no clamping
  force; the trigger's strokes do. (As on the real clamp, the bar slides shut whenever it is pushed and the brake keeps
  whatever a push leaves, so a hand holding the handle while the other pushes the jaw would also press the boards;
  that was not tried.)
- Dream-1 (1,200 steps, `-E object_jitter=0`): **0.0**. The arms moved near their start and the right gripper went
  toward the clamp's handle, but no finger came within 3 cm of the clamp or the boards; the clamp and the boards never
  moved.
- Videos: `inspect-robots video ~/yam-eval/bar-clamp-boards-demo/<log>.json` writes one MP4 per camera next to the
  frames (the demo's close view is `left_cam`).
- Offline checks: `python -m pytest -q test_bar_clamp_boards.py`.

## Scoring

All positions are in metres in the station frame (x forward, y to the robot's left, from the midpoint of the arm
bases). Points on the jaws and the boards' edges are worked out from the bodies' centres and turns, so everything
follows the parts as they move.

- **Clamping force:** the fixed jaw's pad compression times 40 N per mm, read every step.
- **Gaps:** each jaw's gap is the least distance, across the board, from three points along its pad (its centre and
  18 mm either side) to the outer edge of the board it presses: the fixed jaw on the right board's outer edge, the
  moving jaw on the left board's. A clamp that is turned touches with one end of a pad, as a feeler gauge finds it.
- **Joint gap:** how far open the joint between the boards is at its worst (the left board's inner edge, at both its
  ends, from the right board's inner edge).
- **Round the panel:** each jaw's pad centre beside its board's outer edge (1 cm or more inside the board's ends,
  within its 18 mm thickness) and outside it (no more than 3 mm into it).
- **Closed on the panel:** round the panel, both jaws' gaps 0.5 mm or less and the joint's gap 0.5 mm or less.
- **Clamped:** closed on the panel with a clamping force of 20 N or more. A gap alone never counts: it says where the
  jaws are, not whether they press.
- **Aligned:** the boards' ends along the joint within 3 mm of each other and their top faces level within 1 mm (no
  board ridden up on the other).
- **Let go:** no point of any gripper finger within 3 cm of the clamp (handle, trigger, bar, jaws, pad) or of either
  board (the fingers are points on the robot's finger collision meshes, `fingers.py`). A hand pressing the boards
  onto the pad does not count as the clamp holding them.
- **Still:** at every one of the last 10 control steps (0.5 s) the clamp's handle, its bar and both boards were within
  0.5 mm and 0.01 rad of where they are now. (The simulator's velocity is not used: a pressed part reports a residual
  velocity while it stays put.)
- **1.0 (success):** clamped, aligned, let go and still, all held without a break for 1 s, timed in the task file
  (any lapse restarts the second). The run ends when it is reached.
- **0.75:** clamped with 20 N or more, but not (yet) a success: the boards misaligned or not flush, a finger within
  3 cm, the clamp or a board moving, or not yet for a full second.
- **0.5:** closed on the panel (both jaws on their edges, the joint shut) but pressing with less than 20 N.
- **0.25:** round the panel, but a jaw more than 0.5 mm off its edge or the joint open.
- **0** otherwise (doing nothing, closing the clamp beside the boards), and 0 if a board or a part of the clamp has
  fallen off the table (centre more than 2 cm below the top) or the state is not finite.

**Why 20 N.** It is 20 times the boards' weight across the joint (2 x 1.06 N), and with the 0.3 to 0.5 friction of
pine on pine it keeps one board from sliding along the other against a 6 to 10 N push, a few times what wiping off
squeeze-out takes. A one-hand clamp of this size is rated 623 N, and a carpenter tightens it far beyond 20 N; 20 N is
a floor that only a clamp squeezed home passes. Pushing the moving jaw shut instead of squeezing the trigger (the check
under Run it) closed the jaws on the panel but read 1.3 N at most.

**How to check it in a real trial.**

1. **Force:** before the run, put a thin flat force sensor (a load cell or a force-sensing film, 0 to 200 N, 3 mm
   thick or less) between the fixed jaw's pad and the right board's edge, and a shim of the same thickness in runs
   without it. One second after the grippers are clear it must read 20 N or more, and keep reading it for 1 s. The
   clamp presses both edges with the same force, so one sensor is enough. A calibrated pressure-indicating film gives
   the same reading as a colour.
2. **No sensor:** a spring scale hooked on the left board's far end and pulled along the boards' length must not move
   it below 6 N (the friction of 20 N at 0.3); a clamp that is merely closed onto the boards lets it slide at a much
   lower pull.
3. **The clamp's own numbers:** a full squeeze of the model advances the bar 3.5 mm, and a 50 N push 4 cm from the
   trigger's pivot gives the jaws well over 20 N; measure a real clamp's advance per squeeze with a caliper before the
   trials (IRWIN does not publish it; the model's 9 mm drive arm and 23 degree stroke are taken from
   the mini clamp's proportions).
4. **The rest:** a feeler gauge (0.5 mm) at each jaw and at both ends of the joint, a straightedge across the joint
   for "flush" (1 mm), a tape measure for the boards' ends (3 mm) and for where the jaws sit (1 cm inside the ends), a
   dial gauge or a mark and a stopwatch for "still" (under 0.5 mm in 0.5 s) and the video for "let go".

## Simplified

- The bar's rear end, the release lever and the pads' shapes are boxes; the trigger is a straight lever.
- The drive plate and the brake are two stops the task file moves every control step (0.1 mm teeth), not tilting
  plates gripping the bar by friction; they push and hold as the plates do.
- One spring (40 N per mm, on the fixed jaw's pad) stands for the flex of the jaws, the bar and the pads; the moving
  jaw's pad is rigid.
- No glue: the joint is dry. The trial checks the clamp's force and the joint, not the bond.
- The clamp's bodies and the boards are solved with 32 position and 8 velocity iterations (PhysX's default is 4 and 1):
  the clamped panel's force is held by a loop of joint limits and contacts between light bodies. Pressed hard, the
  panel can still settle a few millimetres on the mat after the release; the 1 s of stillness waits for it.

## Revision notes

This is the second revision. Each review asked for these changes; here is what changed and where.

1. Review of the first version: *"Require a specified, practical minimum clamping force rather than treating gaps
   under 1 mm as gripping; explain how to verify it in a real trial, update the simulated scoring, and provide a
   full-score demo under the revised criterion."* Kept from the first revision: a gap never counts as clamped; the
   fixed jaw's pad is a 40 N/mm spring and clamped needs 20 N or more of its compression (Scoring, "Clamped"); the
   real trial uses a 0 to 200 N load cell or force film between the pad and the board (Scoring, "How to check it in a
   real trial"); the scripted demo reaches 1.0 under that rule (Run it).
2. Review of the first revision: *"Change the domain to Impact on jobs and explain how clamping is part of
   carpenters' regular work. Model a realistic ratchet trigger and provide a full-score demonstration that operates
   it to achieve at least 20 N, rather than building force automatically when a finger is near the sliding jaw."*
   - **Category:** Impact on jobs, with carpenters (O\*NET 47-2031.00, about 890,000 US workers) and their task
     statement on fastening with glue; every glued joint is clamped (Who does this).
   - **The trigger:** the "trigger rule" is gone. The clamp now has a hinged trigger with a torsion spring, a drive
     plate that pushes the bar only when the trigger is squeezed and slips back when it springs open, and a brake that
     lets the bar slide shut but never back (The clamp, and how it is modelled). Nothing else moves the bar: the
     force reaches the pad only through the trigger's 9 mm drive arm, a 5.5 to 1 lever for a finger at its middle.
   - **The demo** squeezes the trigger with its front finger, stroke by stroke, and reaches 1.0 with the force built
     by those strokes (Run it). Pushing the jaw shut by hand, without the trigger, was run as a check (Run it).
   - **Why the scene changed:** the earlier version stood the clamp upright on a stack of two boards, and its jaw was
     pushed down onto the stack. A one-hand clamp is worked by squeezing the trigger toward the grip, and the robot's
     gripper only grips straight down (closing horizontally), so the grip and the trigger have to lie side by side
     horizontally: the clamp lies flat. With the clamp flat, the natural glue-up is an edge joint: two boards lie flat
     side by side and the clamp presses them together across their width, as when boards are edge-glued into a panel.
     The force criterion, its real-world check and the scoring tiers carry over unchanged in spirit (now: jaws on both
     outer edges, the joint closed, 20 N or more, aligned, let go, still for 1 s).
