# Advance a sheath over a guidewire on a training phantom without losing the wire

A Seldinger training phantom lies on the table: a skin-coloured block with a 9 mm bore whose mouth (the port) has a
dark grey collar. A black 6 mm training wire runs 8 cm into the bore and 40 cm out of it along one straight line; a
white band on the wire sits just outside the collar. A free blue sheath (16 cm sleeve) lies around the wire, its nose
26 cm from the port and the wire's tail 2 cm inside its rear hub. A valve pad in the hub rests on the wire, so the
sheath drags the wire with it unless the wire is held. The robot must slide the sheath along the wire until it seats
against the collar, then draw the wire out through the sheath, and let go.

The tested ability is keeping a line still while a sleeve that grips it travels past the point where it is held, and
passing that hold from one arm to the other. Pushing the sheath with the wire free pushes the wire into the phantom
(the band disappears into the port). In the Seldinger technique, loss of the guidewire into the vessel is "a
significant and generally preventable complication" (Wikipedia, Seldinger technique). One arm must
hold the wire while the other pushes. At first the tail is hidden inside the sheath, so the wire can only be held in
front of the sheath, near the port; the sheath then closes on that hand. Once the tail shows behind the sheath, the
hold must move to the tail and the other arm takes the sheath for the last part. The left arm cannot reach the tail
and the right arm cannot reach the port, so the swap is forced by reach. Withdrawing the wire also needs two arms: the
valve drags the sheath back out of the port unless the sheath is held.

The instruction states only the goal ("On this training phantom, slide the blue sheath along the wire until it seats
against the port, then draw the wire out through the sheath. Let go of everything at the end."). It does not say
that the band must stay at the port or that the wire must be held: that is what the task tests. The rule is in the
scoring below. The phantom has no patient; the wire is a scaled training cord, so trials are safe.
The raised grip collar 2 cm behind the sheath's nose is a test-rig feature for the robot gripper: real sheaths are
pushed at the hub, so the collar gives the fingers something to push on and is not part of the clinical technique.

## How this differs from similar tasks

- Sliding a ring or block along a fixed, tensioned wire: there the wire is anchored at both ends and only the slider
  has to be placed. Here the wire is free; the sheath grips it, so the robot must keep the wire still by hand while
  moving the sheath over it, and the sleeve passes over the place where the wire was held, which forces a hold
  handover from one arm to the other.
- Pushing or pulling a cable through a fixed conduit: there the cable moves and the tube stays. Here the line must
  not move at the port while the sleeve moves, and the sleeve itself is the part being placed.
- Sliding a rigid carriage onto a fixed rail: the rail cannot move. Here the "rail" is a loose wire that moves with the
  sleeve unless held.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
    --instruction "On this training phantom, slide the blue sheath along the wire until it seats against the port, then draw the wire out through the sheath. Let go of everything at the end."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1600
```

Closed-loop (reads the live wire and sheath): the left arm pinches the wire 5 cm outside the port while the right arm
pinches the sheath 7 cm behind its nose and slides it 13 cm along the wire (band moves at most 2 mm). The right arm
lets go and takes the wire's tail, now out of the sheath's rear; the left arm lets go of the wire, sets its closed
fingers behind the sheath's grip collar and pushes the sheath until it seats against the collar. The left fingers stay
behind the grip collar so the sheath cannot be dragged back while the right arm draws the wire out (two pulls, with
a re-pinch behind the rear). Both arms let go; success holds 1 s and ends the episode (975 steps);
task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

Test variants (`SHEATH_DEMO=<mode> python cabledemo.py task.py out`):

| `SHEATH_DEMO=` | what it does | result |
|---|---|---|
| `nothing` | waits | 0 (sheath not advanced) |
| `pushfree` | one arm pushes the sheath as far as it reaches, wire never held | 0 (the wire is dragged into the phantom; band moves 108 mm) |
| `onearm` | one arm only: push 4 cm, let go, pinch the wire behind the sheath, pull it back, repeat | 0 (each unheld push drags the wire in, and pulling the wire back drags the sheath back with it: band moves 66 mm, sheath ends 6 cm behind its start) |
| `pullfirst` | draws the wire 6 cm out of the port first, then the correct sequence | 0 (band moves 59 mm before the sheath is seated; the rest of the run seats the sheath and draws the wire out, which alone would score 1.0) |
| `hold` | the whole correct demo, but the left fingers stay on the sheath's grip collar | 0 (not released) |
| `tier03` | stops after the first 13 cm push, lets go | 0.3 |
| `tier06` | seats the sheath, never withdraws the wire, lets go | 0.6 |

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if, at any control step before the sheath was first seated, the white band's centre was more than 1.5 cm
  from where it started (either direction, including lifted). This is latched in module state for the whole trial:
  putting the wire back does not clear it; only the reset that starts the next trial does. After the sheath has been
  seated once, the wire may move (it has to, to come out).
- **0** unless both grippers have let go at the end: each gripper's TCP at least 3.5 cm from every point of the wire,
  from the sheath's centre line and from its valve pad.
- **0** if the sheath lies more than 1 cm off the port line (either end) or more than 2 cm above the table, or its
  nose advanced less than 12 cm from where it started (26 cm from the port).
- Otherwise **0.3**: nose advanced 12 cm or more but not seated; **0.6**: seated (nose within 5 mm of the collar
  face), wire tip not yet 3 cm out past the sheath's rear; **1.0**: seated and the wire's tip at least 3 cm out past the
  sheath's rear. `success()` also needs the sheath and the wire at rest.
- In a real trial, moving the phantom scores 0. Measure with a ruler: band position against the collar, nose gap to
  the collar, tip distance behind the sheath's rear.
- `distance()`: nose gap to the collar plus the tip's shortfall behind the rear (m); 1.0 once the band rule fired.

## Runtime and seeds

- Run limit 1,600 steps (80 s at 20 Hz). One trial takes about 4-6 minutes wall time on one GPU including the
  1-2 minute Isaac Sim start-up.
- The layout is fixed with `object_jitter=0`; the wire is never jittered (it always starts straight in the bore), so
  every trial starts identically.
