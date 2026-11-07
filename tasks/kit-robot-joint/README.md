# Kit the parts of a robot joint into a shaped tray

Three parts of a small robot joint stand in a row: a servo motor, a bracket block and a bearing block. Each has its own
pocket in a kit tray, 4 mm wider than the part on every side. The servo motor's pocket runs side to side, so the motor
has to be turned a quarter turn to go in, and the small bearing block would also fit either of the other pockets, so the
robot has to match each part to its own.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=kit_robot_joint.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put each part of the robot joint into its matching pocket in the kit tray, standing the way it stands now." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/kit-robot-joint-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=kit_robot_joint.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put each part of the robot joint into its matching pocket in the kit tray, standing the way it stands now." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/kit-robot-joint-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/kit-robot-joint-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all three parts have been in their pockets and still
for 1 s; otherwise it scores the last step.

## Scoring
A part is **in its pocket** when it stands upright on the pocket floor (its 4 cm height within 10 degrees of
vertical, either way up; the bracket, a 4 cm cube, on any face), its lowest corner within 4 mm of the plate's top,
with all four bottom corners inside its own pocket's inner edges (up to 0.5 mm into a wall it is pressed against
still counts). The motor standing on its end would fit its
pocket, but it is the wrong way round for the kit, so it doesn't count.

| Outcome | Score |
|---|---|
| All three parts in their own pockets | 1.0 |
| Two | 0.67 |
| One | 0.33 |
| None | 0 |

A part in another part's pocket earns nothing for itself, and it usually blocks the part that belongs there: the
bearing block in the motor's pocket, with the bracket in its own, scores 0.33.

On a real table, open the grippers and take a top-down photo.

## Design notes
- **Matching is the point.** The motor fits only its own pocket, and only turned a quarter turn; the bracket fits only
  its own; the bearing block fits all three. Putting the bearing block in the motor's or the bracket's pocket scores
  nothing for it and leaves the other part nowhere to go.
- **The turn is checked in the sim, not just on paper.** The demo carried across without turning the motor scores
  0.67: the motor comes to rest on its pocket's walls, and the other two parts count.
- **Low walls, tall parts.** The pocket walls stand 8 mm and the parts 4 cm, so a gripper holding a part near its top
  stays clear of the walls once the part is down.
- **The demo lets go 4 mm above the pocket floor,** just inside the walls. Set down on the floor while still gripped,
  the parts were pushed 3 to 6 mm sideways (traced). Released higher, the demo reaches 1.0 with every aim moved 2 mm
  either way in x or y.

## Runtime and seeds
- The scripted demo (three moves with the left arm, the motor turned on the way) ends with success at 36.4 s of
  simulated time (727 steps).
- Dream-1 scores 0 in a full 3,600-step trial.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- `test_kit_robot_joint.py` checks which part fits which pocket, the scoring (each part, the turned motor, a wrong
  pocket, the corner margin, a tilted, lifted, upside-down or on-end part, a moving part), the demo's places and its
  reach (`python -m pytest -q test_kit_robot_joint.py`, no GPU needed).
