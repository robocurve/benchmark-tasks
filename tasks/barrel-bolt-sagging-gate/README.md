# Bolt a sagging garden gate shut: lift its free end to line the barrel bolt up, slide it home, drop the handle in

The latch end of a shut, boarded cedar garden gate stands across the table, its face toward the robot, 10 mm from its
gate post on the robot's right. A 150 mm black barrel bolt is screwed to the gate's lock rail, and its steel keeper is
screwed to the post. The gate's hinges have worn, so its free end has dropped 7 mm: the 12 mm bolt now sits 7 mm below
the keeper's 18 mm opening, and slid across it only butts against the keeper's lower strap. The robot must bolt the gate
shut.

Garden, yard, side-passage and paddock gates are bolted like this every day, and most older gates sag. Bolting one
takes two hands working together:

- **Bearing the gate's weight to line the bolt up.** The gate's free end has to be lifted 4 to 10 mm (the keeper's play
  is 3 mm either way) and held there. Held 0.29 m from its free edge it takes
  about 11 N. Lift too little or too much and the bolt
  stops against the keeper.
- **Sliding the bolt home while the gate is held.** The other hand slides the bolt 45 mm across into the keeper,
  until it stops. If the gate is let down before the bolt is in, it has to be lifted again.
- **Locking it.** The bolt's handle sticks out of a slot in the barrel. Only at the end of the bolt's travel is there a
  notch in the slot's lower lip that the handle can be turned down into. Turned down, the handle stops the bolt sliding
  back.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `gatekit.py` | A fixed mechanism built from boxes and cylinders, with links on a chain of hinges and slides, for yam_arena task files |
| `demo.py` | The scripted demo |
| `test_scoring.py` | Offline checks: every scoring outcome, and the mechanism's geometry (the sag blocks the bolt, the keeper's play, the notch) (no GPU) |

`gatekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer. It follows the part kit used by other tasks in this
repository, and adds links joined to each other (gate on its hinge, bolt sliding on the gate, handle turning on the
bolt), fixed to the world, with self-collisions on so that the bolt meets the keeper and the handle meets its slot.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/gate-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1200              # writes demo_*_cam.mp4, setup.png, result.json
DEMO_DEBUG=1 python demo.py task.py debug_out --no-record      # also prints the bolt's height, travel, handle, joints
DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record    # measures CAL["lift"] (see below)
python -m pytest test_scoring.py                               # offline checks
```

1. The left arm comes down over the top edge of the gate's infill boards, 0.29 m along the gate from its free edge,
   and pinches it, its fingers closing across the boards.
2. It lifts the gate's free end until the bolt is level with the keeper, and holds it there.
3. The right arm comes round in front of the post and down onto the bolt's knob from above, pinches it and slides the
   bolt across into the keeper until it stops (commanded 15 mm past the stop, since the arm lags).
4. The left arm lowers the gate onto the bolt, which now carries it, and lets go.
5. The right arm lets go of the knob: at the end of its slot the handle drops into the notch. The closed fingertips
   then press it down.

The demo runs open loop: it reads no simulator state while it runs; every target comes from the fixed layout in
`task.py`. The arms are compliant, and the left one sags under the gate's weight, so how far its hand has to be
commanded up to bring the bolt level with the keeper was measured once with `DEMO_CALIBRATE=1`. That run raises the
hand in 1 mm steps, reads the bolt's height and prints the result (21 mm). The handle's droop onto its slot's lip
(12.9 degrees) was read from the debug output. Both are fixed in `CAL` in `demo.py`. The layout is fixed and the
simulation deterministic. The demo reaches 1.0 (success) at step 485 (24 s): the left hand has let go of the gate, which
rests on the bolt, and the handle dropped into its notch by itself when the right hand let go of the knob.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). Measured with a ruler
and a photo: the height of the bolt's end against the centre of the keeper's opening, and how far the bolt's end is
past the keeper's mouth (the face it enters by). The bolt stops 27 mm past the mouth.

- 1.0 (success, which must hold for 1 s): the bolt's end is at least 25 mm past the keeper's mouth and the handle is
  turned down at least 70 degrees (into its notch), for 20 control steps (1 s) in a row with the bolt still (its end
  moving less than 1 mm, the handle less than 5 degrees). `score()` reports 1.0 only once this has held.
- 0.75: the bolt's end was at some point at least 25 mm past the keeper's mouth (home, handle not turned down).
- 0.5: it was at some point at least 5 mm past the keeper's mouth.
- 0.25: the bolt was at some point lined up with the keeper: its end within 3 mm (up or down) of the opening's centre,
  which takes lifting the gate's free end 4 to 10 mm.
- Otherwise 0.

Nothing in the scene can be damaged or knocked over (the post and keeper are fixed, the gate is on its hinges), so
there is no zero for damage. Holding the gate up at the end does not matter: a bolt home with its handle down counts
whether or not the gate is still held.

## Runtime and seeds

Expect about 3 minutes for a full 1,200-step trial on an L40S, including about 1 minute for Isaac Sim to start. The
layout is fixed (`-E object_jitter=0`), so there are no seeds to choose.

## Fidelity

The parts are rigid boxes and cylinders at real size. Only the part of the gate at bolt height near its latch end is
modelled: a 360 mm long, 140 mm tall section (45 x 70 mm stile and lock rail, 20 mm infill boards above the rail), on a
hinge about the gate's bottom hinge, 900 mm from its free edge and off the table. The gate above and below this band and
toward its hinges is left out, so the section weighs 0.72 kg (western red cedar, with the bolt's backplate and barrel)
and needs about 11 N to lift where the demo lifts it (7 N at its free edge). A whole gate would need more. The worn
hinges' play is a stop: the gate rests by its weight 7 mm low at the bolt and can be lifted up to 16 mm. The bolt
slides and the handle turns on frictionless joints with light damping; the handle falls by its weight when the notch
lets it. Friction is 0.6 for the wood and steel and 0.3 for the bolt. No contact force cap is enforced in simulation.

## Real-world build

Fit a 150 mm barrel bolt with a turn-down handle to a timber gate whose free end has dropped about 7 mm on its hinges,
and its keeper to the post at the height the bolt would have if the gate hung true; or hinge a 900 mm length of 45 x 70
mm timber to a post so that its free end rests 7 mm low on a stop, with the bolt on it. Start with the bolt drawn back
and the handle up. The trial is safe. Measure the bolt's height against the keeper and its travel past the keeper's
mouth with a ruler.

## Assets

All geometry is defined in `task.py` from boxes and cylinders; there are no external assets.
