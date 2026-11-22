# Clear from the right, serve from the left

A guest sits facing the robot across the table. The guest's place setting (the cover, a placemat) holds the finished
starter plate; the plated main course waits on the pass, a low shelf nearer the robot, and a low-rimmed bus tray stands
on the robot's left. The instruction gives the house standard: "Clear the guest's starter plate onto the bus tray and serve the main course. House standard: clear from the guest's right, serve food from the guest's left." Guides for servers say the same:
[Restaurantware's fine-dining etiquette tips for servers](https://www.restaurantware.com/blogs/restaurant-management/5-fine-dining-etiquette-tips-for-servers) say to serve from the left and clear from the right;
[WebstaurantStore's guide to fine-dining etiquette for servers](https://www.webstaurantstore.com/article/90/fine-dining-etiquette-for-servers.html) says always to clear from the guest's right,
while the serving side varies by house; [Tonno restaurant's server-etiquette training](https://tonnorestaurant.com/wp-content/uploads/2022/10/Training_ServerEtiquette.pdf) has pre-plated courses
served from the left and cleared from the right. The sides are the guest's, so the task checks whether the robot maps
them onto its own frame, and whether it keeps a rule the instruction leaves out:

- **The guest's sides, not the robot's.** The guest's right is the robot's left (+y). The starter must leave the cover
  toward the robot's left and the main must arrive from the robot's right: each plate's path must cross the circle
  10 cm round the cover's centre at least 5 cm to that side. Taking the starter straight toward the robot, or bringing
  the main straight in from the robot's side, gets that side wrong and scores 0.5.
- **Clear before serving (unstated).** The main may not go down before the starter has gone: setting it on the table
  within 8 cm of the cover's centre before the starter's centre has been 20 cm from it, or leaving it resting on top
  of the starter (variant `stack`), scores 0.
- **Bimanual by convenience.** With an arm on each side, the natural way is for the left arm to clear toward the guest's
  right while the right arm brings the main in from the guest's left, as two servers would; the scripted demo runs both
  arms at once. One arm could also do both in turn; no one-arm run was recorded.
- **Hands off the food and the guest.** No finger pad may come within 8 mm of the main course's food, and no plate may
  pass over the guest's lap.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes for yam_arena task files (visual-only boxes for the placemat, the guest and the food), and per-step readings: part poses, finger-pad points, and the drop and stillness checks |
| `demo.py`, `democore.py` | The scripted full-score demo and its failure variants (`democore.py` is shared with our other kitchen tasks) |
| `demo_results.json` | Each variant's score and end-state measurements, from the runs reported below |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold or position no longer matches the text) |

`kitchenkit.py` hooks the embodiment's step and reset without modifying yam_arena, so the task runs with the standard
`yam_arena` embodiment and `task_score` scorer. For this task the scorer reads the two plates' poses and the
finger-pad points each step.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clear the guest's starter plate onto the bus tray and serve the main course. House standard: clear from the guest's right, serve food from the guest's left." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/clear-serve-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Runtime:** the `full` scripted demo ran 319 steps; it ended on success, before the end of its
19.8 s plan (396 steps). The variants ran their whole plans: 396 steps for `clear_front` and
`serve_front`, 370 for `stack` (18.5 s). Wall-clock time was not recorded with these runs. A model trial runs up to
1,200 steps, plus the policy's own response time. Measured on Nov 15: the Dream-1 run (`dreamlabs/dream-1-beta`, 1,200 steps, `rows_per_request=12`) took 3.3 min of wall-clock time on one NVIDIA RTX PRO 6000 Blackwell GPU, simulator start-up included.

## Scripted demo

```bash
python demo.py task.py demo_out                         # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_stack --variant stack        # any variant below; --no-record skips the video
python demo.py task.py --plan-only                      # inverse-kinematics check of every variant's plan
DEMO_DEBUG=10 python demo.py task.py demo_out           # also print the plates' state every 10 steps
```

The left arm pinches the starter plate's left (+y) rim lip, approaching with the gripper open about 2.8 cm, lifts the
plate 5 cm, carries it 22 cm sideways off the cover toward the guest's right (the robot's left), then over the bus tray,
sets it down and lets go. Meanwhile the right arm pinches the main course plate's right (-y) rim lip, lifts it 5 cm off
the pass and carries it to a point 22 cm to the guest's left (the robot's right) of the cover. It waits there until
9.5 s into its plan (the starter is due 22 cm off the cover at 8.2 s), then slides the plate in, sets it down on the
cover with the steak toward the guest and lets go. Each gripper opens only to about 1.7 cm to let go: opened back to
the approach width, the inner finger came within 8 mm of the food. The variants change the direction of one move, or
leave out the clear.

The two crossing columns give where each plate's path crosses the circle 10 cm round the cover's centre, as the
sideways offset from the cover's centre (positive toward the robot's left, the guest's right). The starter needs
+5 cm or more; the main needs −5 cm or less.

| Variant | What it does | Score | Starter leaves the circle at, cm | Main enters the circle at, cm | Starter on the tray | Main down on the cover | Steak angle from toward the guest, deg | Main set down before the starter had gone | Steps run |
|---|---|---|---|---|---|---|---|---|---|
| `full` | the left arm takes the starter sideways off the cover toward the guest's right and puts it on the tray; the right arm brings the main round to the guest's left and, once the starter has gone, slides it in and sets it on the cover | **1.0** | +10.00 | −9.98 | yes | yes | 0.7 | no | 319 |
| `clear_front` | as `full`, but the starter is taken 16 cm straight toward the robot off the cover before it goes to the tray | 0.5 (starter cleared on the wrong side) | +0.19 | −9.97 | yes | yes | 0.5 | no | 396 |
| `serve_front` | as `full`, but the main is brought to a point 16 cm in front of the cover on the robot's side and slid straight in | 0.5 (main served from the wrong side) | +10.00 | −0.38 | yes | yes | 0.2 | no | 396 |
| `stack` | the left arm stays at home; the right arm brings the main in from the guest's left and sets it down on top of the starter, which was never cleared | 0.0 (the main was left on the starter before it was cleared; neither plate ends where it belongs) | none (the starter was never taken off the cover) | −9.99 | no | no | 0.5 | yes | 370 |

In the `stack` run the main was left resting on the starter, which counts as setting it down before the starter had
gone. In every run no plate passed over the guest's lap, no finger pad came within 8 mm of the food, and both plates
ended released and still. The numbers are from `demo_results.json`.

## Scoring

Every step, from poses; the score at the end counts.

- **Path and side:** a plate's path is its centre's horizontal position at every step from the first step its
  underside is more than 1 cm above its starting height. The side is read where the path crosses the circle 10 cm
  round the cover's centre, interpolated between steps: the starter is cleared from the guest's right if its path
  first leaves the circle at least 5 cm to the robot's left (+y) of the cover's centre; the main is served from the
  guest's left if its path last enters the circle at least 5 cm to the robot's right (−y). Both mean within 60 degrees
  of that side. A plate that is never lifted has no side.
- **Placed:** the starter rests on the bus tray (its centre within 7 cm of the tray's centre along x and along y, its
  underside within 1.2 cm of the table) and the main is down on the cover (its underside within 4 mm of the table, its
  centre within 3 cm of the cover's centre), no finger pad within 8 mm of either plate, and both plates moving less
  than 2 mm over 1 s.
- **1.0:** placed, both sides right, and the steak within 25 degrees of pointing at the guest (+x), seen from above.
- **0.5:** placed, one side wrong. **0.25:** placed, both sides wrong.
- **Steak:** a steak more than 25 degrees off takes 0.25 from any of these (1.0 becomes 0.75; not below 0).
- **0:** otherwise, and from the moment the main is down on the table (underside within 4 mm) within 8 cm of the
  cover's centre before the starter's centre has been 20 cm from it, the main is left resting on the starter (their
  centres within 8 cm horizontally, the main's underside above the starter's and at most 2.5 cm above it, no finger
  pad within 2 cm of the main), either plate's centre passes beyond x 50 cm (over the guest's lap; the guest's block
  starts at x 53), a finger pad comes within 8 mm of the main course's food (a zone 13 x 13 x 4 cm over the plate's
  centre, from the top of its base up), or a plate is dropped (it falls 3 cm within 0.5 s with no finger pad within
  2 cm of it).

`report()` records where each plate's path crossed the circle (`exit_side_y`, `entry_side_y`) and whether each side
is right, whether the starter is on the tray and the main down on the cover, the steak's angle, whether the main went
down (or was left on the starter) before the starter had gone (`set_before_cleared`), whether a plate passed over the
lap, whether a finger pad came near the food (and the first step and gripper), and whether both plates are released
and still. It does not list the drop check.

## Real-world trials

Use two square rimmed plates about 20 cm across that the grippers can pinch by the rim lip, a stand-in meal fixed to
the main plate (for example a foam steak glued on one side, with an arrow drawn from the plate's centre toward it), a
placemat with its centre marked, a 4 cm block or shelf for the pass and a low-rimmed tray. Do not seat a person at the
cover: a box stands in for the guest, and a strip of tape 50 cm in front of the midpoint between the arm bases marks
the lap line. Draw a 10 cm circle round the cover's centre with tick marks 5 cm to each side of its centre line, put a
marker on each plate's centre, and score from an overhead camera: where each plate's path crosses the circle, whether
the main went down within 8 cm of the cover's centre before the starter's centre had been 20 cm from it or was left on
top of the starter, the end positions (the main within 3 cm of the cover's centre; the starter's centre within the
tray's central 14 x 14 cm) and the steak's angle (25 degrees). No hot food is needed.

## Context and simulation fidelity in full

`task.json` keeps condensed versions of these two fields (the task board limits their length). The full text:

**Context.** The instruction gives the house standard of table service. Restaurantware's fine-dining etiquette tips for servers say to serve from the left and clear from the right; WebstaurantStore's guide to fine-dining etiquette for servers says always to clear from the guest's right, while the serving side varies by house; the server-etiquette training of Tonno restaurant has pre-plated courses served from the left and cleared from the right. The guest sits facing the robot, so the guest's right is the robot's left: the robot has to map the guest's sides onto its own frame. One rule is not stated: the new plate is never set down before the used one has gone; setting the main course down on the cover, or leaving it on top of the starter, before the starter has gone scores 0. With an arm on each side, the natural way is for the left arm to clear the starter toward the guest's right while the right arm brings the main course in from the guest's left, the two working at the same time, as two servers would. One arm could also do both in turn, so two arms are the natural way, not a necessity. Done by: waiters and waitresses, bussers and server assistants, banquet and catering servers, and other front-of-house restaurant staff.

**Simulation fidelity.** Rigid bodies built from boxes: each plate is a base and four rim lips; the pass and the bus tray are fixed. Contacts and friction are simulated (plates 0.6 static and 0.5 dynamic, pass and tray 0.5 and 0.4). The food is visual and fixed to its plate, so it cannot slide or spill; the scorer stands in for it with a zone over the main plate's centre that no finger pad may come within 8 mm of. No heat, steam or sauce is depicted or simulated. The guest is a visual block that does not move or react; the scorer stands in for the guest's lap with the line x 50 cm that no plate's centre may cross. A real server stands at the guest's side; here the arms face the guest across the table, so the scorer stands in for the server's side with the plate's path: the side on which it crosses a 10 cm circle round the cover, at least 5 cm to that side.
