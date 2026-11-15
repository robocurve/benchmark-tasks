# Hot-swap a server's failed power supply without disturbing the live one

A two-bay power-supply cage is fixed to the table with its bays facing the arms, like the back of a server in its
rack. The left bay holds a failed supply (amber light); the right bay holds the supply still powering the server
(green light). A spare lies on the table in front of the cage, to the left. All three are full-size CRPS-format
supplies (185 x 73.5 x 39.5 mm, 0.9 kg), the hot-swap format used across data-center servers. The robot must take
the failed supply out and seat the spare in its bay without disturbing the live one.

Each supply is held in its bay by a release latch, modelled on a CRPS latch. An orange grip handle stands on the
supply's face; the handle and a hook on the supply's side are one spring-loaded part that slides across the supply.
At rest the hook's square catch stands 5 mm out into a notch in the bay wall, so pulling the handle does nothing until
the handle is pushed 5 mm sideways, which draws the hook in. Behind the catch the hook is ramped: a supply pushed into
a bay rides the ramp in on the wall's edge, and the spring snaps the hook out into its notch when the supply reaches
home. At the back of the bay the supply's edge connector is modelled as a spring plunger over the last 1.4 mm of
insertion (4.2 N), so a supply has to be pushed fully home for its hook to catch.

What the task tests:

- **Releasing while pulling.** The handle has to be held aside by the same grip that pulls; pulling it straight does
  nothing.
- **Moving a long, heavy part into a close bay.** The spare is 185 mm long with 2 mm of clearance a side (30-degree
  lead-ins at the mouth): turned by 1.2 degrees it jams. It starts off to the side and has to be slid across, squared
  up and pushed in straight.
- **Seating it fully,** against the connector, so the latch catches.
- **Not disturbing the live supply,** whose handle the failed supply passes when it is moved out of the way.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `psukit.py` | Rigid parts built from boxes, with spring-loaded sliders or fixed in place, for yam_arena task files |
| `demo.py` | The scripted full-score demo |
| `test_scoring.py` | Offline checks: every scoring outcome, and the layout's geometry (no GPU) |

`psukit.py` hooks `yam_arena.scene` and the embodiment's reset without modifying yam_arena, so the task runs with the
standard `yam_arena` embodiment and `task_score` scorer. It follows the part kit used by other tasks in this
repository, and adds fixed parts and boxes turned about the vertical.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/psu-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1800              # writes demo_*_cam.mp4, setup.png, result.json
DEMO_DEBUG=1 python demo.py task.py debug_out --no-record      # also prints each supply's offset, sliders, tilt, yaw
python -m pytest test_scoring.py                               # offline checks
```

1. The right arm closes on the failed supply's handle, pushes it aside to its stop (releasing the latch), pulls the
   supply 26 cm straight out and lets go; then it grips the supply's body and slides it 15 cm right, in front of the
   live supply and clear of the bay's line.
2. The left arm grips the spare's body at its centre and slides it right until it lines up with the empty bay, then
   pushes it straight in until its back half is in the bay, in steps of at most 3 cm with small corrections of
   position and turn.
3. The left arm grips the spare's handle and pushes it home and on 20 mm; the hook rides in on its ramp and snaps
   into its notch. It lets go.

The demo runs open loop: it reads no simulator state. The arms are compliant and lag under the supplies' friction,
so where a supply ends up after a move and the small corrections in step 2 were measured once with
`DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record` (which runs the same moves closed loop on the
simulator's poses and prints the numbers) and are fixed in `CAL` in `demo.py`. The layout is fixed and the
simulation deterministic. The demo reaches 1.0 (success) at step 1,314 (66 s).

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). Positions are
measured from where a supply sits latched home in the left bay.

- 0 if the live (green) supply was at any point moved more than 5 mm.
- 1.0 (success, which must hold for 1 s): the spare is home in the left bay (within 1.5 mm along it and 2 mm across
  it, level within 3 degrees) with its hook out in the notch (the handle within 1 mm of its rest position), the
  failed supply is clear of the bay (its back face at least 5 mm in front of the mouth, or its centre more than
  7.85 cm to the side), and the spare is still (moving less than 1 mm over the last second).
- 0.75: after the failed supply came out, the spare was pushed in with its back face 10 cm inside the mouth.
- 0.5: the failed supply was pulled all the way out of its bay (back face 5 mm in front of the mouth).
- 0.25: the failed supply was pulled at least 2 cm out (which needs its latch released).
- Otherwise 0.

## Runtime and seeds

About 1.5 minutes per trial for the full 1,800 steps on an L40S, plus about 1 minute for Isaac Sim to start. The
layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

Rigid bodies built from boxes; the supplies are full CRPS size and weight. The latch is one part (60 g) on a
prismatic joint across the supply with a spring drive (300 N/m, 5 mm travel); on a real CRPS supply the release is
a tab pushed toward a fixed handle, here the handle and tab are one part, so the handle itself is pushed aside. The
handle stands 27.5 mm proud of the face and is 48 mm tall so a gripper's fingers fit on it. The edge connector is a
spring plunger over the last 1.4 mm (4.2 N), lighter than a real connector's mating force, which is more than the
YAM arms can push. Bays have 2 mm of clearance a side (a little more than a real chassis) and a 15 mm divider. The
cage is fixed, as a server is bolted in its rack; there are no power cords (the failed supply's cord is taken to be
unplugged first). Friction: 0.25 for the supplies, 0.15 for the steel cage. Light angular damping is added to the
parts. No contact force cap is enforced in simulation.

## Real-world build

Use three CRPS supplies, or 3D-printed PLA shells of the same size weighted to 0.9 kg, with the handle, latch and
hook as in `task.py` (a compression spring of about 0.3 N/mm, 5 mm travel, behind the latch). The cage is bent sheet
steel or 3D-printed PLA with the dimensions in `task.py`, screwed to a board clamped to the table, with a spring
plunger (about 3 N/mm, 1.5 mm travel) at the back of each bay. The lights can be coloured stickers.

## Assets

All geometry is defined in `task.py` from boxes; no external assets.
