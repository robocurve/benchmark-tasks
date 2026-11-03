# Set a paint tin on the overhanging end of a plank

A plank lies on a ledge with one end sticking 16 cm out past the edge. The robot is asked to put a paint tin on that
far end without letting anything fall. The tin alone tips the plank. The safe order, which the instruction doesn't
state: first weigh down the plank's other end with the steel block on the table, then put the tin on. A tip scores 0
for the whole trial, even if the robot rebuilds the stack afterwards.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=counterweight_overhang.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the paint tin on the far end of the plank, the end that sticks out past the ledge, with its centre 2.5 to 8 cm from that end. Leave the plank where it is, and don't let anything fall." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/counterweight-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=counterweight_overhang.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the paint tin on the far end of the plank, the end that sticks out past the ledge, with its centre 2.5 to 8 cm from that end. Leave the plank where it is, and don't let anything fall." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/counterweight-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/counterweight-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the tin has stood on the far end, with the plank
balanced, for 1 s; otherwise it scores the last step.

**Keep `-E object_jitter=0`.** The plank starts 3 mm from the stop, so jitter would push it into the stop. The fall
flag also clears only when every loose object is back at its start, which is how each trial begins.

## Scoring
A fall is any of these: the plank tilting more than 3 degrees (a tipped plank lands at 18 degrees); the tin leaning
past 45 degrees; the tin or the block moving down faster than 1 m/s (a free fall of about 5 cm).

| Outcome | Score |
|---|---|
| Tin upright on the far end (its base centre 2.5 to 8 cm from the end, within 3 cm of the middle line, within 2 mm of the plank's top), plank level, in place and standing on its own, everything still, for 1 s | 1.0 |
| Nothing has fallen, plank level and in place, block on the plank far enough back to hold the tin anywhere in the zone (about 8.5 cm or more behind the edge), tin not on the far end yet | 0.5 |
| Anything fell during the trial, even if put right afterwards | 0 |
| Tin on the far end with no block behind it, the plank propped level by a gripper | 0 |
| Block too near the edge, plank moved or turned on the ledge, or nothing done | 0 |

"Standing on its own" is computed: the combined centre of mass of the plank and everything resting on it must lie at
least 1 cm behind the ledge edge. The sim can't tell a gripped object from a released one. Without this check, a
gripper propping the plank, or still holding the tin, would pass as balanced.

On a real table: watch the video for any tip or drop. Then open the gripper and measure with a tape the tin's base
centre from the plank's far end, and the block's centre from the ledge edge.

## The numbers
Masses: plank 0.4 kg, tin 0.6 kg, block 1 kg. Distances are along the plank, from the ledge edge.

| Load on the plank | Combined centre of mass |
|---|---|
| Nothing | 4.0 cm behind the edge: it holds itself up |
| Tin alone, anywhere in the target zone | 3.2 to 6.5 cm past the edge: it tips |
| Tin anywhere in the zone, block 18 cm behind the edge (the demo) | 5.8 to 7.4 cm behind the edge |

## Design notes
- **The instruction doesn't mention the tipping, on purpose.** A worker staging material is told where it goes, not
  how to keep the plank down. Seeing the tip coming and weighing the plank down first is what this task measures.
  "Don't let anything fall" warns that falls count, without saying what would fall.
- **The plank must stay where it is.** Moving more of it onto the ledge would remove the overhang, and with it the
  problem. The stop sits 3 mm behind the plank, so it can't be slid straight back. Turned about 16 degrees and pulled
  toward the arms, though, its end passes beside the stop and the tin alone balances (found in review). So 0.5 and 1.0
  need the plank's centre within 5 mm of its start and its length within 2 degrees of its start direction, and the
  instruction says to leave the plank where it is.
- **Lifting the tin off a tipped plank lets the plank level again by itself.** That is why a fall counts for the
  whole trial, not just while the plank is down.
- **The fall flag clears only at the exact start layout.** The task-file format has no reset hook, so the flag clears
  when every loose object is within 1 mm (x, y) and 2 mm (z) of where a reset puts it, and level. A tin or block put
  back by a gripper doesn't land that close. One case it can't tell from a reset: an empty plank pressed down and let
  go, with the tin and block never touched, settles back to its start. Nothing loaded fell, so it isn't counted.
- **No arm can hold the plank's other end down while the other loads.** Only the left arm reaches the plank's
  supported end, and the plank (10 cm wide) is too wide for the open fingers (9.4 cm apart) to grip. In the sim the
  right arm reaches the far end, low down, so it could prop it from below or hold the tin there. The balance check
  gives that no credit, because nothing on the plank holds it down. A tin or block counts as resting on the plank
  only within 2 mm of its top, so one held just above it doesn't count.

## Runtime and seeds
- Under 2 minutes per demo trial (23 s of sim time) and about 6 minutes per Dream-1 trial (the full 2,400 steps) on
  an L40S, measured 2026-11-03, after Isaac Sim's first start (about 6 minutes, once per machine).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- Dream's service once refused this instruction before the trial started ("unsafe instruction in this scene, refused:
  Placing a heavy object on the overhanging end will tip the plank"), and accepted it on the next three tries
  (2026-11-03). If a trial errors that way, run it again.

## Files
- `counterweight_overhang.py`: the task file (objects, success, score, demo).
- `test_counterweight_overhang.py`: scoring and demo-reach checks without the simulator
  (`python -m pytest -q test_counterweight_overhang.py`, needs `yam_arena`).
