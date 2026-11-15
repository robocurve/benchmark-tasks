# Take a wheelchair's armrest off for a sliding-board transfer

A wheelchair's left side frame stands on the table with its outer side toward the arms and its front toward the right
arm: two square steel arm sockets fixed to the frame, the blue seat and back beyond them. A removable desk-length
armrest (1.2 kg) stands in the sockets on two square steel posts 20 cm apart, each 10 cm deep in its socket. Before a
sliding-board transfer to a bed, a nursing assistant or home care aide takes this armrest off so the patient can slide
across. The robot must take it off and lay it down on the table on its own side of the chair.

The armrest is locked in place, as removable armrests are. A steel catch on the front post stands 5 mm out through a
window in the front socket's wall; the catch and a red release tab, standing out of the front post toward the arms, are
one spring-loaded part that slides along the armrest. Pushing the tab 7 mm toward the back of the chair draws the catch
in. Lifting does nothing until the tab is held back.

What the task tests:

- **Releasing while lifting.** The tab has to be held back by the same hand that lifts that end; let go too early and
  the catch catches on the window's edge.
- **Lifting a two-post part square.** Each post has 1.5 mm of clearance in 10 cm of socket: tipped by about 1.7 degrees
  it binds. Both ends have to come up together, 10 cm, with the hands holding a 1.2 kg part well above the table.
- **Laying it down rather than dropping it.** The armrest is 33 cm tall and stands on two 2.2 cm feet; it has to be
  turned onto its side and set on the table, clear of the chair's side.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `chairkit.py` | Rigid parts built from boxes, with spring-loaded sliders or fixed in place, for yam_arena task files |
| `demo.py` | The scripted full-score demo |
| `test_scoring.py` | Offline checks: every scoring outcome, and the layout's geometry (no GPU) |

`chairkit.py` hooks `yam_arena.scene` and the embodiment's reset without modifying yam_arena, so the task runs with
the standard `yam_arena` embodiment and `task_score` scorer. It follows the part kit used by other tasks in this
repository, and adds fixed parts.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/armrest-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1200              # writes demo_*_cam.mp4, setup.png, result.json
DEMO_DEBUG=1 python demo.py task.py debug_out --no-record      # also prints the armrest's pose, lock, tilt, lowest point
python -m pytest test_scoring.py                               # offline checks
```

1. Both arms reach in level from the arms' side with the fingers closing up and down: the right arm's fingers close on
   the red tab, the left arm's on the rear half of the arm pad.
2. The right arm pushes the tab toward the back of the chair, past its 7 mm stop (the arm is compliant), which draws
   the catch in.
3. Both arms lift together, straight up, commanded 16 cm (the posts are 10 cm deep, and the arms sag a few
   centimetres under the load). The wrists are turned 2 degrees against the pitch the armrest's weight gives them, so
   it comes up square.
4. They carry it 5 cm toward the arms, then, in 24 small steps, tip it over onto its side (top away from the arms,
   posts toward them) while carrying it on toward the arms and lowering it, keeping it 4 cm off the table until the
   last steps and ending with its side just over the table. They let go; it settles onto its side, and the hands lift
   away.

The demo runs open loop: it reads no simulator state, only the published layout in `task.py`. The armrest is in both
grippers from the moment it leaves its sockets, so where it goes is where the arms put it; the layout is fixed and the
simulation deterministic. The demo reaches 1.0 (success) at step 401 (20 s).

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). The armrest's centre
is the point on its centreline 16.5 cm up the posts from their feet.

- 0 if the armrest was ever let fall: moving down faster than 0.8 m/s with its lowest point more than 5 cm above the
  table (a free drop of about 8 cm or more). Tipping over with its feet on the table does not count.
- 1.0 if both posts are out of their sockets, its centre is at least 10 cm out from the sockets' line toward the arms,
  and it lies on the table on its side (tipped at least 75 degrees from upright, its lowest point within 3 mm of the
  table). This is success once it is also at rest (moving slower than 1 cm/s and turning slower than 0.05 rad/s, about 3 degrees/s),
  and the episode ends when that has held for 1 s.
- Otherwise the furthest it got: 0.75 if both posts were out with its centre at least 10 cm out from the sockets'
  line; 0.5 if both posts were out of their sockets (each foot 5 mm above its socket's mouth, or 3 cm to the side of
  the socket's axis); 0.25 if it was lifted 2 cm (which needs the lock released: held, it lifts only 2 mm before the
  catch meets the window's top edge); else 0.

## Runtime and seeds

About 1 minute per trial for the full 1,200 steps on an L40S, plus about 1 minute for Isaac Sim to start. The layout
is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

Rigid bodies built from boxes, at full size and weight: the armrest is a desk-length removable arm (26 cm pad, posts
20 cm apart, 33 cm tall, 1.2 kg). The posts are square, 22 mm (real posts are usually 22 mm round tubes), in square
sockets 25 mm inside, so 1.5 mm of clearance a side (a real fit is a little closer). The lock is one 50 g part on a
prismatic joint along the armrest with a spring drive (400 N/m, 7 mm travel): on a real armrest a spring lever or
plunger engages a hole in the socket; here the tab, 35 x 24 x 12 mm so gripper fingers fit on it, slides a square
catch out of a window in the socket wall. Only the chair's left side frame is modelled, from the socket floors up,
fixed to the table as a braked chair: the sockets' floors are at the table (on a real chair they are about 45 cm
above the floor), and the wheels, footrests and right side are left out; the seat and back are rigid blocks.
Friction: 0.3 for the steel posts and sockets. Light angular damping is added to the armrest. No contact force cap
is enforced in simulation.

## Real-world build

Use a removable desk-length armrest and its two sockets from a standard wheelchair, with the sockets bolted to a board
clamped to the table at the spacing in `task.py`, or a 3D-printed or aluminium copy of the same size weighted to
1.2 kg, with a spring catch (about 0.4 N/mm, 7 mm travel) through a window in the front socket. The chair's seat and
back can be a board and foam blocks.

## Assets

All geometry is defined in `task.py` from boxes; no external assets.
