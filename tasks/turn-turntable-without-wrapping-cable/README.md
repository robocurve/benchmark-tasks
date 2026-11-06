# Turn a camera turntable without wrapping its cable

A camera turntable (grey disc, 120 mm diameter) turns on a fixed vertical axle in front of the right arm. A red
arrow on its top points away from the robot; a green mark on the table sits at the disc's robot-side edge, so the
arrow must turn half a turn. The camera cable leaves a connector on the far rim tangentially to the left, passes a
fixed grey post and runs through a fixed cable clip; beyond the clip it lies loose. Turning the disc clockwise winds
the cable round the turntable; turning it anticlockwise unwinds it, but the cable then sweeps into the post and hooks
round its far side. The robot has to work out the winding: turn anticlockwise, and get the cable over the post to
the robot's side of it before the sweep reaches the post. The reference solution turns the disc about 60 degrees
first (the connector swings towards the clip and the cable goes slack), holds the disc there with the right arm
while the left arm lifts the slack cable over the post and lays it down on the robot's side, then finishes the turn
and lets go.

The reference solution splits the work by side (arm bases at y +/-30 cm): the knob's path stays on the right arm's
side (y -11 to -3 cm) and the post and clip on the left arm's side (y 7 and 18 cm); while the left arm moves the cable,
the right arm holds the turntable still. Holding matters: in the `reroute_only` test (disc not held) lifting the cable
over the post dragged the disc 68 deg. Whether one arm alone could do the task (no reach check was run) is not tested.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (cable simulation and scripted-demo runner). `task.py` adds,
  on top of the kit: the turntable's axle (a joint free only in yaw and +/-3 mm of height, so its weight rests on the
  table and table friction acts as bearing drag, plus light damping), the knob's own vertical pin on the disc, and a vertical
  pin (swivelling strain relief, -200..+20 deg) between the disc rim and the cable's first segment.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Turn the camera turntable half a turn so its red arrow points at the green mark, then let go. Its cable must not end up wound round the turntable or wrapped round the grey post."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed loop (reads the live disc, knob and cable state): the right arm pinches the knob (wrist camera looking across
the disc) and swings it anticlockwise round the measured axle in 12 deg moves, to about 60 deg (58 deg logged); holding the disc
there, the left arm takes the now slack cable 3 cm past the post (wrist camera looking back along it at the post and
disc), lifts it over the post, lays it down well clear on the robot's side (pulling some spare cable through the
clip) and lets go; the right arm then finishes the half turn (arrow about 7 deg from the mark, inside the 10 deg limit by about 3 deg) and
lets go. Success holds
for 1 s and ends the episode at step 667 (~33 s); task_score 1.0. `demo/` holds its videos, `setup.png` and
`result.json`.

Unintended strategies (each scored 0 in its logged run), selected with `TT_DEMO=<mode>`:

| `TT_DEMO=` | what it does | result |
|---|---|---|
| `nothing` | waits | 0 (arrow 180 deg off) |
| `ccw_no_reroute` | turns anticlockwise half a turn without moving the cable | 0 (arrow 6 deg from the mark, but the cable is dragged round the post's far side: post enclosed, winding 1, and swept 261 deg round the turntable axle) |
| `cw` | turns clockwise half a turn (the other way) | 0 (arrow 3 deg from the mark, but the cable is wound 251 deg round the turntable and encloses the post) |
| `reroute_only` | lifts the cable over the post to the robot's side without holding or turning the disc | 0 (the cable drags the disc 68 deg: arrow 112 deg off; post enclosed, winding -1) |
| `hold` | the correct solution, but the right arm keeps pinching the knob at the end | 0 (gripper not let go) |

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s with the disc and knob at rest)

Measured in the top view on the cable path from the connector to the clip (the cable centre line up to the point
nearest the clip, then the clip centre):

- **0** if physics is NaN, either gripper is within 3 cm of the cable, the disc or the knob (not let go), the cable
  no longer passes through the clip, the cable sweeps 180 deg or more round the turntable axle (signed angle sum
  seen from the axle: about +76 deg at the start, about +251 deg after the `cw` clockwise half turn, about -97 deg after
  the reference solution's anticlockwise one), or the post is enclosed by the loop "cable from connector to clip, then straight back to
  the connector" (winding number about the post axis not 0: the cable is hooked round the post's far side).
- Otherwise **1.0** if the arrow is within 10 deg of the green mark, **0.5** if within 30 deg, else 0.
- `distance()`: arrow error in half turns, +1 if wound, +|loops| for the post. `progress()`: stage diagnostic.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). The scripted demo ran at about 3 steps/s on one GPU (667 steps in about
  4 min including start-up); at that rate a trial that uses all 2,400 steps takes roughly 15 min.
- Layout fixed with `object_jitter=0`; the cable is never jittered (it always starts straight). Always run with
  `object_jitter=0`.
