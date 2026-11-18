# Replace a plug-in control relay: seat it in its DIN-rail socket and snap the hold-down clip over it

A control-panel back plate lies on the bench with a DIN rail and a 14-pin plug-in relay socket. The failed relay has
been pulled; the socket's hold-down clip stands open on its stop. The new relay stands upright in a foam parts tray
beside the plate. The robot is told to fit the new relay in the empty socket and close its hold-down clip.

Replacing a failed control relay is routine work for electricians and maintenance workers. The relay's blades have
to go into the socket straight and all the way home, and the hold-down clip closes only over a relay that is fully
seated. The task tests a straight, guided insertion against contact friction followed by closing a hinged retainer
over the part, in that order.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the plate, socket, contacts and clip, the relay and its tray, `success()` and `score()` |
| `partkit.py` | Rigid parts built from boxes, with sliders and hinges, for yam_arena task files |
| `demo.py` | The scripted full-score demo and its negative variants |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the new relay in the empty socket and latch its retaining clip." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir dream_out
```

## Scripted demo

```bash
python demo.py task.py demo_out                       # full score: writes demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=short python demo.py task.py short       # relay pushed only 3 mm in and left: 0.25
DEMO_VARIANT=noclip python demo.py task.py noclip     # relay seated, clip left open: 0.5
```

The right arm takes the relay by its body, carries it over the socket and pushes it straight down until it sits on
the socket. It lets go and, with closed fingers, pushes the clip forward along its arc until it drops onto the relay,
then presses it until it snaps under its catch. Full score is reached in 565 steps. The demo reads only the layout
and its own joints.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- 0 if the back plate is pushed 5 mm or the relay is dropped. This is permanent.
- 1.0 (success, which must hold for 1 s): the relay seated flat on the socket (within 1 mm, 1.5 mm of centre,
  3 degrees square), the clip snapped under its catch, both grippers 30 mm or more away, nothing moving.
- 0.75: seated and clipped, but a gripper is near or something still moves.
- 0.5: seated, but the clip is not latched, both grippers 30 mm or more away.
- 0.25: the blades went 2 mm into the socket at some point.
- Otherwise 0.

## Runtime and seeds

About 1 minute per trial for the full 1,200 steps, plus Isaac Sim start-up. The layout is fixed (`object_jitter=0`);
there are no seeds to choose.

## Fidelity

Rigid bodies at true size. The 14 blades are one 20 x 14 x 6.4 mm block in a 22 x 16 mm cavity; the contact springs
are a plate in the cavity that holds until the blades push it with more than 3 N, then slides, and stays where it
is pushed. The
clip's catch is task code: pressed 1.5 degrees past flat it holds there. The back plate is loose and heavy.

## Real-world build

- A 4-pole 24 V plug-in relay (21.5 x 28 x 36 mm, 14 blades) and its matching 14-pin DIN-rail socket with hold-down
  clip, from the same maker.
- A 35 mm DIN rail screwed to a 200 x 160 mm steel back plate, with two bench stops for the plate.
- A foam parts tray with a pocket for the relay's blades.

Check the seat with a feeler: the relay's body must sit on the socket. Check the clip by lifting the relay gently.
