# Load a butane canister into a cassette burner: notch up, pushed home against the seal, lever locked

A portable butane cassette burner (28.2 x 35 cm, 8 cm tall, 1.6 kg, unfixed on rubber feet) stands on the table with its
canister compartment along its right side. A full 250 g cassette canister (6.6 cm across, 19 cm long, 0.36 kg) lies on
the table to the right of it, with the notch on its rim turned 35 degrees off the top. Cooks at buffet, omelet and
catering stations (O*NET 35-2014.00) and servers at hot-pot and Korean barbecue tables (35-3031.00) change these
canisters during service. The robot must load the canister so that the burner is ready to light.

The compartment holds the canister on two rails. At the back is the gas inlet: a socket with a round opening and a guide
slot at its top. The canister's notch is modelled as a lug on its mounting cup, and the lug passes the socket only
within about 13 degrees of straight up; otherwise it stops against the socket face before the nozzle reaches the seal.
Inside the socket a spring plunger stands in for the seal and magnet coupling (1.5 N/mm, 4.5 mm travel). A canister let
go there is pushed back out of the seal. At the front, the locking lever is a bar that slides 4 cm across behind the
canister's base. It can only cross behind a canister that is pushed home, and it then holds the canister against the
plunger.

What the task tests:

- **Turning the canister in the hand.** The notch starts 35 degrees off. The canister has to be gripped and turned so
  that its notch faces up as it goes in.
- **Holding it home against a spring while the other hand works the lever.** Pushed home, the canister springs back
  out unless something holds it there.
- **Not knocking the burner.** The burner is loose on the table.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `burnerkit.py` | Rigid parts built from boxes and cylinders, with spring-loaded sliders, for yam_arena task files |
| `demo.py` | The scripted demo |
| `test_scoring.py` | Offline checks: every scoring outcome, and the layout's and the inlet's geometry (no GPU) |

`burnerkit.py` hooks `yam_arena.scene` and the embodiment's reset without modifying yam_arena, so the task runs with
the standard `yam_arena` embodiment and `task_score` scorer. It follows the part kit used by other tasks in this
repository. It adds parts laid on their side (a starting roll about their own axis) and a set centre of mass, so the
round canister does not roll on its own.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/burner-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1600              # writes demo_*_cam.mp4, setup.png, result.json
DEMO_DEBUG=1 python demo.py task.py debug_out --no-record      # also prints the canister's pose, notch, lever, plunger
python -m pytest test_scoring.py                               # offline checks
```

1. The right arm comes down along a hand axis tilted 38 degrees about the canister's axis, grips the canister across its
   middle and turns its hand upright as it lifts. The notch then faces straight up.
2. It carries the canister over the compartment, lays it on the rails 2.5 cm short of the inlet and pushes it in. The
   lug passes the guide slot and the nozzle presses the plunger. The arm keeps pushing.
3. The left arm comes down beside the lever's handle with its fingers closed and pushes the handle across. The bar
   slides behind the canister's base.
4. As soon as the bar is across, the right arm lets go, and the plunger sets the canister back against the bar. Then
   the left arm lets go.

The demo runs open loop: it reads no simulator state. The arms are compliant. The grip tilt that leaves the notch
straight up, and how far the carried canister hangs off the commanded hand position, were measured once with
`DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record`. That run makes the same moves and prints the measured
pose. The numbers are fixed in `CAL` in `demo.py`. The layout is fixed and the simulation deterministic. The demo
reaches 1.0 (success) at step 574 (29 s), after the right arm has let go.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). Positions are
measured along the compartment from the burner's centre line, in the burner's frame. The canister counts as on the
rails when its axis is within 10 mm of the compartment's line, within 6 mm of its height and within 10 degrees of its
direction.

- 0 if the burner was at any point moved more than 3 cm or tipped more than 10 degrees, or if the canister fell off the
  table.
- 1.0 (success, which must hold for 1 s): the canister is home (its centre 8.5 to 14 mm past the centre line, on the
  rails), its notch is within 15 degrees of straight up, the lever is at least 3.5 of its 4 cm across, and the
  canister is still (moving less than 1 mm), all for 1 s (20 steps in a row).
- 0.75: it was at some point home with its notch up.
- 0.5: its nozzle was at some point in the inlet (centre at least 5 mm past the line). The lug lets it get that far
  only through the guide slot.
- 0.25: it was at some point on the rails, its centre between 60 mm before and 16 mm past the line.
- Otherwise 0.

## Runtime and seeds

Expect about 1.5 minutes for the full 1,600 steps on an L40S, plus about 1 minute for Isaac Sim to start. The layout
is fixed (`-E object_jitter=0`), so there are no seeds to choose.

## Fidelity

The parts are rigid boxes and cylinders at real size and mass. The canister's notch is a 1 x 1 cm lug on its mounting
cup that has to pass a slot in the inlet. The inlet's seal and magnet coupling are a spring plunger. The locking lever
turns downward on real burners; here it is a bar that slides across, with damping and no spring, so it stays where it
is put. The compartment cover is left off. The canister's contents are rigid. Friction is 1.0 for the burner's feet
and 0.35 for the steel canister. Light angular damping is added to the parts. No contact force cap is enforced in
simulation.

## Real-world build

Use any portable cassette burner with a notch-guided inlet and a locking lever, and a standard 250 g butane cassette
canister. For a trial without gas, use an empty canister and keep the burner switched off. The burner stands loose on
a marked outline.

## Assets

All geometry is defined in `task.py` from boxes and cylinders; there are no external assets.
