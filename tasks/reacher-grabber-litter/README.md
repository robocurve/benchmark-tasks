# Pick up litter with a reacher-grabber

Groundskeepers and janitors pick litter off the ground with a trigger-grip pick-up tool (a "reacher-grabber"): the
tool is pointed at a piece, the trigger is pulled to close the jaws on it, and the trigger is held while the tool is
lifted and swung over the bin, where letting it go drops the piece in. This task gives a two-arm robot such a tool, a
real 24-inch (61 cm, 0.22 kg) aluminium litter picker, and three pieces of litter lying at the far edge of the arms'
reach: a crushed drinks can, an empty 500 mL PET bottle and a paper cup. The robot has to put all three in a bin
(an open plastic tote) using the tool.

A parallel gripper cannot hold the handle and pull the trigger at once (closing it on the handle does nothing to the
trigger, and opening it to let the trigger go would drop the tool), so one gripper holds the handle and points the tool
while the other works the trigger.

What the task tests:

- **Steering a long tool from its handle.** The jaws are 53 cm from the hand; a few millimetres of error at the handle
  is centimetres at the jaws, and the 13 cm jaw opening leaves 2.5 to 3.5 cm either side of each piece.
- **Coordinating two hands on one tool.** The trigger hand has to slide its part 28 mm back along the tool while the
  handle hand keeps the tool still, and both have to move together, the trigger held, while the tool is swung.
- **Holding on until the right moment.** Easing the trigger while the tool is lifted or swung drops the piece; it is
  let go only over the bin.
- **Not cheating with the grippers.** Litter touched by a gripper scores 0.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, the trigger-to-jaw drive, `success()` and `score()` |
| `litterkit.py` | Rigid parts built from boxes and cylinders, with spring-loaded sliders or fixed in place, for yam_arena task files |
| `demo.py` | The scripted full-score demo |
| `test_scoring.py` | Offline checks: every scoring outcome, the gripper-touch check, the trigger coupling, the layout (no GPU) |

`litterkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. It follows the part kit used by other tasks in
this repository and adds fixed parts, a starting pitch, a set centre of mass, slider position targets set while the
simulation runs, and the grippers' poses (for the touch check).

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/litter-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 2400               # writes demo_*_cam.mp4, setup.png, result.json
DEMO_DEBUG=1 python demo.py task.py debug_out --no-record       # also prints the tool's pose, trigger, jaws, litter
python -m pytest test_scoring.py                                # offline checks
```

1. Both grippers come down on the tool where it rests: the left across the handle housing, behind the grip, leaning
   back 45 degrees so the two wrists stay apart; the right straight down across the trigger, its fingers either side
   of the shaft. Both close.
2. The arms lift the tool. For each piece (can, bottle, cup) they swing the open jaws over it with the jaws 25 cm up,
   lower them around it (the bottle is taken across its body, base end just in front of the jaw head), and the right
   gripper slides the trigger 28 mm back to its stop, closing the jaws. (For the cup, far to the left, the swing
   toward it is made with the trigger held, which keeps the trigger hand within reach; it is let go on the way down.)
3. With the trigger held, they lift the piece, raise the jaws to 25 cm (above the bin's 17.8 cm rim), swing them over
   the bin and let the trigger go; the piece drops in.
4. The tool is put back down on the table and both grippers let go (the run ends on success, held 1 s, before
   this).

The left gripper's hold on the housing turns the tool with it, so the handle wrist sets the tool's pitch; the trigger
pivots between the right gripper's fingers. The arms are compliant: under the tool's weight the tool droops and the
trigger hand lags, so each move's command is offset by a constant measured once with
`DEMO_CALIBRATE=1 DEMO_DEBUG=1 python demo.py task.py cal_out --no-record --max-steps 5000` (which corrects each move
closed loop on the simulator's tool pose and prints the offsets) and fixed in `CAL` in `demo.py`. The layout is fixed
and the simulation deterministic; the demo itself runs open loop and reads no simulator state.
For the cup, at the edge of the arms' reach, `CAL` keeps the whole sequence of corrections the calibration stepped
through and replays it as recorded (`DEMO_CALIBRATE_ONLY=cup` recalibrates just those moves). Coming down onto the
cup, the jaws tip it over; it is picked up lying on its side. The demo reaches 1.0 (success) at step 1,659 (83 s).

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). A piece is in the
bin when its centre is inside the bin's inner walls and below its rim.

- 0 if a gripper touched a piece of litter at any point (any part of a piece within 3 mm of a gripper's fingers, the
  space between them or the gripper body just above them), or if the tool fell off the table (its handle more than 3 cm
  below the table top, or beyond the table's edges and less than 5 cm above the table top).
- Success: all three pieces in the bin and still (moving less than 1 mm over the last 1 s). It scores 1.0 once success
  has held for 1 s (20 control steps in a row), when the episode ends.
- Otherwise 0.3 for each piece in the bin plus 0.1 for each piece that was lifted clear of the table (its lowest point
  5 cm or more above the table top) at some point but is not in the bin: 0.9 with all three in before success has
  held for 1 s, otherwise at most 0.7 (two in, one lifted).

## Runtime and seeds

About 2 minutes per trial for the full 2,400 steps on an L40S, plus about 1 minute for Isaac Sim to start. The
layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

Rigid bodies built from boxes and cylinders. The pick-up tool has the length (61 cm), mass (0.22 kg), shaft
(19 mm aluminium tube) and jaw opening (13 cm) of a 24-inch trigger-grip litter picker; its handle housing, grip,
trigger and jaws are simplified blocks. Its centre of mass is set 13 cm along the tool (the plastic handle outweighs
the thin tube). The trigger is a 30 g block on a prismatic joint along the tool (28 mm stroke) with a return spring of
120 N/m (3.4 N at its stop), lighter than a real trigger because the compliant YAM arms lag about 5 mm per newton. A
real picker closes its jaws through a rod or cable; the simulator has no gear or cable joints, so after every control
step the code sets each jaw's spring drive to close in proportion to how far the trigger is pulled (fully pulled: shut
to a 6 mm gap). The jaws are 15 g plates on prismatic joints across the tool (real jaws pivot); a jaw stopped by a
piece presses on it at 300 N/m times the remaining closure (about 9 N on the bottle). The litter is rigid: the crushed
can is a 7 x 3.5 cm cylinder, the bottle three cylinders with a 2.4 cm flat strip along its underside (a dent, so it
does not roll), the cup four stacked cylinders, solid. The bin is a 16-quart storage tote with straight 3 mm walls (a
real one tapers slightly), fixed to the table. The gripper-touch check uses the gripper's pose and finger opening with
box-shaped finger and body zones, not contact sensing. Friction: 1.0 for the tool (rubber-coated grip and jaws), 0.5
for the litter and the bin. Light angular damping is added to the parts. No contact force cap is enforced in
simulation.

## Real-world build

Use a 24-inch (61 cm) trigger-grip litter picker with 13 cm jaws, a 16-quart storage tote (no lid) taped to the
table at its outline, a 355 mL aluminium can stamped flat end-on (about 7 cm across, 3.5 cm tall), an empty 500 mL PET
bottle with its cap on, and a 12 oz paper hot cup. Mark each starting position on the table.

## Assets

All geometry is defined in `task.py` from boxes and cylinders; no external assets.
