# Serve two plates together, each steak toward its guest

Two plated main courses wait on the pass, a low shelf on the table nearer the robot, one plate on each arm's side,
each with its steak pointing toward the middle. Further out, two placemats mark where two guests are seated. The
instruction is only: "Serve the two main courses to the two guests."

In plated service the plates for a table are set down at the same moment, each with its protein at the 6 o'clock
position, toward the guest. [Nicolet College's chapter on American service](https://nicoletcollege.pressbooks.pub/beverage/part/chapter-4-american-service/)
puts the protein at 6 o'clock, facing the guest, and has all guests receive their food simultaneously;
[HappyChef's guide to fine-dining service](https://happychef.cloud/en/blog/guest-experience/fine-dining-service-excellence.html) calls for synchronised serving, with all plates set down
simultaneously and one server per guest; [Instawork's basics of plated service](https://www.instawork.com/blog/instawork-basics-of-serving-clearing-plated-service) also put the
protein at the 6 o'clock position and say not to rotate plates in front of the guests. The instruction states neither
rule, so the task checks whether the robot serves the way trained restaurant staff do, unprompted:

- **Both plates down together.** The two set-downs must come within 0.5 s (10 steps) of each other for 1.0. Set down
  0.5 to 3 s apart, the plates score 0.5; more than 3 s apart, they score 0, even if both end perfectly placed.
- **Steak toward the guest, turned on the way.** Each plate starts with its steak toward the middle, so it has to turn
  a quarter turn between the pass and the table. A steak more than 25 degrees from pointing at its guest, or a plate
  turned more than 10 degrees once it is on the table, caps the score at 0.25.
- **Bimanual.** One gripper carries one plate (it pinches the plate's rim lip), so setting both plates down at the same
  moment takes both arms, one plate each: each arm does the job of one server. Serving them one after the other
  (variant `sequential`) scores 0.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes for yam_arena task files (visual-only boxes for the placemats and the food), and per-step readings: finger-pad points, part poses and tilt, and the drop and stillness checks |
| `demo.py`, `democore.py` | The scripted full-score demo and its failure variants (`democore.py` is shared with our other kitchen tasks) |
| `demo_results.json` | Each variant's score and end-state measurements, from the runs reported below |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold or position no longer matches the text) |

`kitchenkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. For this task the scorer reads the two plates'
poses and the finger-pad points each step.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Serve the two main courses to the two guests." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/serve-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Runtime:** the `full` scripted demo ran 313 steps; it ended on success, before the end of its
18.9 s plan (378 steps). The variants ran their whole plans, 378 to 716 steps. Wall-clock time
was not recorded with these runs. A model trial runs up to 1,200 steps, plus the policy's own response time. Measured on Nov 15: the Dream-1 run (`dreamlabs/dream-1-beta`, 1,200 steps, `rows_per_request=12`) took 3.3 min of wall-clock time on one NVIDIA RTX PRO 6000 Blackwell GPU, simulator start-up included.

## Scripted demo

```bash
python demo.py task.py demo_out                         # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_late --variant late          # any variant below; --no-record skips the video
python demo.py task.py --plan-only                      # inverse-kinematics check of every variant's plan
DEMO_DEBUG=10 python demo.py task.py demo_out           # also print the plates' state every 10 steps
```

Each arm pinches the outer rim lip of the plate on its side (approaching with the gripper open about 2.8 cm), lifts
the plate 4 cm off the pass and carries it out to the place setting on its side, turning it a quarter turn on the way
(the left plate 90 degrees anticlockwise seen from above, the right plate 90 degrees clockwise) so that the steak faces
the guest. Both arms move at the same time, and both plates reach the table at step 236. Each gripper then
opens only to about 1.7 cm to let go: opened back to the approach width, the inner finger came within 8 mm of the
food. The variants keep these motions and change only when the left arm starts, or leave out the turn.

| Variant | What it does | Score | Set down at step (left / right) | Gap, steps | Steak angle from toward the guest, deg (left / right) | Plate centre off its setting, mm (left / right) | Max carry tilt, deg | Steps run |
|---|---|---|---|---|---|---|---|---|
| `full` | both arms at once; each plate turned a quarter turn during the carry | **1.0** | 236 / 236 | 0 | 0.6 / 0.8 | 5.7 / 5.0 | 5.02 | 313 |
| `sequential` | the right arm serves its plate first; the left arm starts when the right has finished | 0.0 (set down more than 3 s apart) | 574 / 236 | 338 (16.9 s) | 0.7 / 0.8 | 5.3 / 4.8 | 2.07 | 716 |
| `late` | the left arm starts 1.5 s after the right | 0.5 (set down 0.5 to 3 s apart) | 266 / 236 | 30 (1.5 s) | 0.5 / 0.8 | 5.4 / 4.8 | 2.04 | 408 |
| `no_turn` | both plates set down together without turning: the steaks point sideways, toward each other | 0.25 (steaks more than 25 degrees off) | 236 / 236 | 0 | 91.4 / 90.0 | 5.2 / 3.9 | 1.93 | 378 |

In every run both plates ended placed (on the table, released, still and within 3 cm of their settings), no finger
pad came within 8 mm of the food, and neither plate was turned on the table. The numbers are from
`demo_results.json`.

## Scoring

Every step, from poses; the score at the end counts.

- **Set-down and gap:** a plate has left the pass once its base is more than 1 cm above its starting height. It is set
  down at the first step after that when its underside is within 3 mm of the table. The gap is the number of steps
  (20 Hz) between the two plates' set-down steps.
- **Steak and turn:** the steak's angle is the horizontal angle between the direction from the plate's centre to its
  steak and the direction to the guest (+x). A plate is turned on the table if, while on the table after its
  set-down, its yaw differs from its yaw at set-down by more than 10 degrees. Turning it on the pass or during the
  carry, before its first set-down, does not count.
- **Placed:** both plates on the table (underside within 3 mm of it), each plate's centre within 3 cm (horizontally)
  of the centre of the place setting on its own side, no finger pad within 8 mm of either plate, and both plates
  moving less than 2 mm over 1 s.
- **1.0:** placed, set down at most 10 steps (0.5 s) apart, both steaks within 25 degrees of toward their guests,
  neither plate turned on the table, and a carry tilt (the largest tilt of either plate between leaving the pass and
  its set-down) of at most 15 degrees.
- **0.5:** the same, but set down more than 10 and at most 60 steps (0.5 to 3 s) apart.
- **0.25:** placed and set down at most 60 steps (3 s) apart, but a steak more than 25 degrees off, a plate turned on
  the table, or a carry tilt of more than 15 (up to 30) degrees.
- **0:** otherwise: a plate not set down or not placed, or the two set-downs more than 60 steps (3 s) apart; and from
  the moment a finger pad comes within 8 mm of the food (a zone 12 x 13.3 x 4 cm over the plate's centre, shifted
  3.5 mm toward the steak, from the top of the base up), a plate is carried tilted more than 30 degrees, or a plate is
  dropped (it falls 3 cm within 0.5 s with no finger pad within 2 cm of it).

`report()` records each plate's set-down step and the gap between them, whether the plates are placed and facing their
guests, whether each was turned on the table, the largest carry tilt, whether a finger pad came near the food (and the
first step, plate and gripper), and for each plate its distance from its setting's centre, the steak's angle, whether
it is on the table and released, and its tilt.

## Real-world trials

Use two rimmed plates the grippers can pinch by the rim, with a stand-in meal fixed to each (for example a foam steak
glued on one side), and two placemats with their centres marked. Draw an arrow on each plate from its centre toward the
steak, and score the end positions (3 cm) and steak angles (25 degrees) from an overhead camera; the same camera shows
whether a plate is turned on the table (10 degrees). Time the two set-downs with a side camera at table height
recording at least 20 frames per second, or with a contact sensor under each placemat, and compare the gap with 0.5 s
and 3 s. No guests need to be seated: the direction to the guest is straight ahead of the robot.

## Context and simulation fidelity in full

`task.json` keeps condensed versions of these two fields (the task board limits their length). The full text:

**Context.** In plated service the plates for a table are set down at the same moment, each with its focal point, the protein, at the 6 o'clock position, toward the guest. Nicolet College's chapter on American service puts the protein at 6 o'clock, facing the guest, and has all guests receive their food simultaneously; HappyChef's guide to fine-dining service calls for synchronised serving, with all plates set down simultaneously and one server per guest; Instawork's basics of plated service also put the protein at the 6 o'clock position and say not to rotate plates in front of the guests. Here two plated main courses wait on the pass with their steaks pointing toward the middle, so each plate has to turn a quarter turn on its way out, before it reaches the table. One gripper carries one plate, so setting both plates down at the same moment takes both arms: each arm does the job of one server. The instruction states neither rule, so the task checks whether the robot serves the way trained restaurant staff do, unprompted. Done by: waiters and waitresses, banquet and catering servers, and other front-of-house restaurant staff.

**Simulation fidelity.** Rigid bodies: boxes, with contact forces and friction. The plates are square boxes with a box rim lip; each gripper holds a plate by pinching its outer lip, and the plates rest on the pass and on the table through contact. The food is visual: the steak, mash and greens have no mass or collision and cannot slide or spill, so the scorer stands in for keeping the food in place with the carry-tilt limits (15 and 30 degrees) and keeps the fingers out of the food with the 8 mm zone round it. The placemats are flat visual marks; the place setting is judged from the plate's centre (within 3 cm). The guests are not modelled: the direction to each guest is +x. No heat, steam or sauce is depicted or simulated. 'At the same moment' is judged from the plates' heights: each set-down is the step at which a plate's underside comes within 3 mm of the table, timed to the 20 Hz control step.
