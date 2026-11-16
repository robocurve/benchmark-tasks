# Tilt a window's lower sash back into its frame, holding both tilt latches in until it latches

A small white vinyl single-hung tilt window stands on the table facing the arms, fixed in place as a window is fixed in
its wall: frame 47 cm wide and 65 cm tall, fixed upper sash. Its lower sash (38 x 30 x 3.5 cm, double glazed, 1.8 kg)
has been tilted in for cleaning: it hangs on its two pivot bars in the balance shoes at the bottom of the jamb tracks
and lies flat toward the arms, interior face down, its top rail resting on a foam-padded block. The robot must put it
back up in the frame so that it latches.

Two spring-loaded tilt latches sit on top of the sash's top rail, one near each end. Each is a blue thumb button and a
steel bolt in one part that slides along the rail: at rest the bolt's square nose stands 11 mm out of the sash's side,
into the slot in the jamb liner, and holds the sash upright. Sliding the button 12 mm toward the middle (against a
4.8 N spring) draws the bolt inside the sash.

What the task tests:

- **Lifting a hinged 1.8 kg sash evenly with two hands** about its pivot bars, through 90 degrees, without lifting it
  out of the shoes.
- **Holding both latches in at once while moving the sash.** Tilted back with either bolt out, the sash stops about
  10 degrees short of upright with the bolt against the jamb. The same grips that carry the sash have to slide the two
  buttons in, and keep them in, as it goes into the frame.
- **Letting go at the right moment:** only once the sash is upright do the bolts line up with the slots (1.4 cm wide)
  and snap in.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `sashkit.py` | Rigid parts built from boxes and cylinders, with spring-loaded sliders, fixed in place, or starting tilted, for yam_arena task files |
| `demo.py` | The scripted full-score demo |
| `test_scoring.py` | Offline checks: every scoring outcome, and the window's geometry (no GPU) |

`sashkit.py` hooks `yam_arena.scene` and the embodiment's reset without modifying yam_arena, so the task runs with the
standard `yam_arena` embodiment and `task_score` scorer. It follows the part kit used by other tasks in this
repository, and adds fixed parts and a starting tilt (pitch) for a part.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sash-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1200              # writes demo_*_cam.mp4, setup.png, result.json
DEMO_DEBUG=1 python demo.py task.py debug_out --no-record      # also prints the sash's tilt, pivot offset, latches
python -m pytest test_scoring.py                               # offline checks
```

1. Each arm comes in along its grip's axis to one latch button and closes on it (fingers either side of the button).
2. The two arms lift the sash together about its pivot bars, in 3-degree steps along the arc, from lying flat to
   15 degrees short of upright. The grips turn with the sash where the arms can reach (pitched 60 degrees toward the
   window at the start, 45 degrees near the top).
3. Both slide their buttons in toward the middle, past the 12 mm stops (the arms are compliant and lag), drawing the
   bolts into the sash.
4. With the buttons held in, they push the sash on until it closes against the upper sash (commanded 6 degrees past
   upright, since the arms lag).
5. They let go: the springs snap the bolts out into the jamb slots. The arms back away.

The demo runs open loop: it reads no simulator state, only the published layout in `task.py`. The sash turns about
its fixed pivot bars, so where each button is at every step follows from the layout. The demo reaches 1.0 (success)
at step 334 (17 s).

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). Tilt is how far the
lower sash leans in from upright (0 upright, 90 lying flat toward the arms).

- 0 if the sash ever came off its pivots (its pivot axis more than 1 cm from where the shoes hold it).
- Success: the sash upright (tilt between -1 and 1.5 degrees; it closes against the upper sash) with both latch bolts
  out in their slots (each latch within 1 mm of its rest position), and still (its tilt changing less than 0.3 degrees
  over the last 1 s). It scores 1.0 once success has held for 1 s (20 control steps in a row), when the episode ends;
  until then, 0.75.
- Otherwise the furthest it got: 0.75 if it was brought within 3 degrees of upright (only possible with both bolts
  drawn in); 0.5 within 20 degrees; 0.25 within 60 degrees (top rail lifted about 14 cm); else 0.

## Runtime and seeds

A trial is 1,200 steps (1 minute of simulated time). On an L40S a full Dream-1 trial took about 5 minutes, plus about 1 minute for Isaac Sim to start; the scripted demo ends at step 334. The layout is
fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

Rigid bodies built from boxes and cylinders, at full size and weight: the lower sash is 38 x 30 x 3.5 cm and 1.8 kg
(vinyl profile and 1.8 cm insulated glass), the size of the lower sash of a small (about 18 x 26 in) single-hung
window. Each tilt latch is one 40 g part (button and bolt) on a prismatic joint along the top rail with a spring drive
(400 N/m, 12 mm travel). The buttons stand 3 cm above the rail, taller than real ones, so gripper fingers fit on them.
The bolts have square noses, as on latches that must be held in to tilt the sash either way (some latches have ramped
noses that let a sash be pushed shut). The pivot bars rest in fixed shoes, as real balance shoes lock in the track when
the sash tilts; the spring balances that carry a raised sash are left out, since the sash is only tilted. The upper
sash is fixed, as in a single-hung window. The sash's bottom rail stands 8 mm clear of the sill (a real one is bevelled
and seals on a weatherstrip). Friction: 0.4 for the vinyl. Light angular damping is added to the parts. No contact
force cap is enforced in simulation.

## Real-world build

A small single-hung tilt window (vinyl, about 18 x 26 in) screwed to a board clamped to the table with its sill on the
board, or a frame of the dimensions in `task.py` in plywood and PLA with a 1.8 kg sash. A foam-padded wooden block
(7 x 24 x 5.7 cm) on the table under the tilted sash's top rail.

## Assets

All geometry is defined in `task.py` from boxes and cylinders; no external assets.
