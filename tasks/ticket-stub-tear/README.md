# Tear an event ticket's stub off along its perforation, put the stub in the stub box and leave the ticket in its stand

A patron's ticket stands on edge in the slot of an usher's ticket stand on the counter: a 2 x 8 in card event ticket
with a perforated 2 in stub at its right-hand end. Beside the stand is the stub box. The robot is told to tear the
stub off the ticket and put it in the stub box.

Tearing the stub at the door is the familiar moment of going to a show, a game or a cinema. The card parts only
along its perforation, and only when one part is held while the other is pulled hard enough; the ticket must stay
whole, in its stand, for the patron. The task tests a two-handed tear (hold one part, pull the other) through a part
that holds until a threshold force, followed by placing the torn-off piece.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the ticket stand, the ticket and its stub joined by a breakable joint, the stub box, `success()` and `score()` |
| `partkit.py` | Rigid parts built from boxes and cylinders, with a breakable weld between parts, for yam_arena task files |
| `demo.py` | The scripted full-score demo and its negative variants |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Tear the stub off this ticket and put it in the stub box." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir dream_out
```

## Scripted demo

```bash
python demo.py task.py demo_out                     # full score: writes demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=notear python demo.py task.py notear   # pinches both parts and lets go: 0
DEMO_VARIANT=drop python demo.py task.py drop       # tears the stub off and drops it on the counter: 0.25
```

The left arm pinches the ticket by its top edge and holds it down in its stand; the right arm pinches the stub and
pulls it straight up, so the perforation tears. The right arm carries the stub over the stub box and lets it fall in,
then both arms let go and move away, the ticket left in its stand. The demo reads only the layout and its own joints.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- 1.0 (success, which must hold for 1 s): the stub torn off along the perforation and lying in the stub box, the
  ticket still standing in its stand (lower edge in the slot, leaning under 25 degrees), both grippers 30 mm or more
  from both parts, nothing moving.
- 0.75: all that, but a gripper is near or something still moves.
- 0.5: torn with the stub in the box, but the ticket out of its stand.
- 0.25: the stub was torn off at some point.
- Otherwise 0.

## Runtime and seeds

About 1 minute per trial for the full 1,200 steps, plus Isaac Sim start-up. The layout is fixed (`object_jitter=0`);
there are no seeds to choose.

## Fidelity

Rigid bodies at true size; the card is modelled 0.8 mm thick (the real one is 0.3 mm). The ticket and its stub are two rigid cards joined along the perforation by a fixed joint that the physics engine breaks for good when it carries more than 8 N or 0.04 N m, about what a perforated card ticket takes to tear; the paper's bending is not modelled.

## Real-world build

- 2 x 8 in card event tickets with a perforated 2 in stub (stock ticket printing on 10 pt card), one per trial.
- A wooden block, 40 x 240 x 20 mm, with a 3 mm saw kerf 8 mm deep along its length, glued to the counter.
- An open box about 90 x 90 x 50 mm for stubs.

Check the result by eye: the stub in the box and the ticket standing whole in its slot.
