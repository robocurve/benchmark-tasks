# Sort laundry by colour, but send hand-wash-tagged items to the mesh bag

A bimanual YAM task in Isaac Lab Arena. Six folded garments lie in a row: three white, three coloured. Two of
them, one white and one coloured, carry a small red hand-wash tag on top. The robot has to sort them into three
containers: whites into the white basket, colours into the blue basket, and any tagged item into the grey mesh
bag whatever its colour. The tags must stay on, and the robot must let go. Sorting by colour alone, the obvious
rule, scores 0: the task measures whether a small marker named in the instruction overrides the salient feature.

The whole task is one task file, [`sort_tagged_laundry.py`](sort_tagged_laundry.py), for the `yam_arena`
embodiment from [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (Isaac Sim 6, Isaac Lab
3, Isaac Lab Arena 0.3). The scene is built from boxes only, so nothing is downloaded.

## Run a trial

From this folder, in the Arena environment:

```bash
INSTRUCTION="Sort this laundry into the three containers: white items in the white basket, coloured items in the blue basket, but any item with a red hand-wash tag goes in the grey mesh bag, whatever its colour. Keep the tags on the items, and let go of each item once it is in."

# Dream-1 (needs DREAM_API_KEY in ~/.dreamlabs/credentials)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sort_tagged_laundry.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir runs/dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sort_tagged_laundry.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir runs/demo

# One MP4 per camera, in runs/<policy>/frames/<run>/
inspect-robots video runs/dream/*.json
```

The run prints `task_score` at the end and keeps it in the log.

- **Run limit:** 2,400 control steps (2 minutes at 20 Hz).
- **Seeds:** none. `-E object_jitter=0` starts every object exactly where the task file puts it, the layout in
  the screenshot. The scripted demo is open-loop and expects that layout.
- **Runtime per trial:** the scripted demo's trial ends at full score after about 1,000 steps (50 s simulated) and took
  about 3.4 minutes of wall time on an L40S. The Dream-1 trial ran the full 2,400 steps in about 7.5 minutes.
  Add a few minutes for Isaac Sim's first start on a fresh machine.

## Scene

Positions are in cm, from the midpoint between the two arm bases: x forward, y left, heights above the table
top. The arm bases are at y = +30.5 (left) and y = -30.5 (right).

| Object | Type and material | Size | Position (centre) | Mass | |
|---|---|---|---|---|---|
| White basket | open box, white plastic: 1 cm floor, four 1 cm walls 5 cm tall | 16 x 16 cm inside, 18 x 18 cm outside, rim 4 cm above the floor | x 30, y +20 | | fixed |
| Mesh bag | open box, grey (rigid stand-in for a mesh wash bag), same build | same | x 30, y 0 | | fixed |
| Blue basket | open box, blue plastic, same build | same | x 30, y -20 | | fixed |
| Green shirt | box, green cotton, folded flat | 8 (x) x 5 (y) x 2.5 cm | x 14, y -20 | 60 g | → blue basket |
| Green sweater | box, green wool, folded flat, **tagged** | 8 x 5 x 2.5 cm | x 14, y -12 | 60 g | → mesh bag |
| White towel | box, white cotton, folded flat | 8 x 5 x 2.5 cm | x 14, y -4 | 60 g | → white basket |
| Navy shirt | box, navy cotton, folded flat | 8 x 5 x 2.5 cm | x 14, y +4 | 60 g | → blue basket |
| White blouse | box, white silk, folded flat, **tagged** | 8 x 5 x 2.5 cm | x 14, y +12 | 60 g | → mesh bag |
| White shirt | box, white cotton, folded flat | 8 x 5 x 2.5 cm | x 14, y +20 | 60 g | → white basket |
| Hand-wash tags (2) | box, red card | 2.5 x 2.5 x 0.4 cm | lying on the sweater and the blouse, centred at x 16.5 | 3 g each | stay on |

Garments lie with their long side along x, square to the table edges. Whites and colours alternate along the
row, and each side of the row holds both, so position does not tell where a garment goes.

## Scoring

Scored by `score()` and `success()` in the task file, every control step, from the object poses. Extents are
computed from each box's pose and true size (the simulator's own bounding boxes are padded).

- **Hard 0:** any garment or tag ends with its lowest point more than 5 cm below the table top (fell off the
  table); or a tag is not on its own garment: its centre is not over the garment's upper face (checked in the
  garment's own frame, so a garment flipped over still counts with the tag on its new upper face), or its lowest
  point is more than 1 cm above that face.
- **Otherwise:** `0.9 x max(0, C/6 - W/3)`
  - `C`: garments (of 6) wholly inside their correct container.
  - `W`: garments wholly inside a wrong container.
  - Wholly inside: within the inner walls (2 mm tolerance for a garment resting against a wall), no part above
    the rim, and its lowest point within 5 mm of the floor or on another garment in the container.
- **1.0:** all six correct, both tags on, everything still (under 1 cm/s and 0.5 rad/s), and every fingertip of
  both grippers at least 3 cm from every garment and tag, held for 1 s in a row. This ends the trial.

Examples: doing nothing scores 0. Sorting by colour alone (the tagged blouse in the white basket and the tagged
sweater in the blue basket) scores 0 (4/6 - 2/3). Putting everything in one container scores 0. One garment
correct with nothing else moved scores 0.15. Five correct and the sixth in a wrong container scores 0.45. All
correct but a tag knocked off scores 0. All correct but still holding a garment scores 0.9.

To score a real trial: look at the containers with a photo or by eye. A garment is in a container if no part of
it is on or over a wall or above the rim. A tag is on its garment if it lies on the garment's upper face.
"Still" and "let go" mean the grippers are open and at least 3 cm from every garment for a second.

## Scripted demo

`demo()` plays an open-loop plan (`yam_scripted` policy) in three rounds, with only one arm in the middle of the
table at a time. Round 1: the right arm takes the navy shirt to the blue basket while the left arm takes the
white shirt to the white basket. Round 2: the left arm takes the tagged white blouse to the mesh bag's left half
while the right arm takes the green shirt to the blue basket. Round 3: the left arm reaches across for the white
towel and takes it to the white basket (the right arm waits parked), then parks while the right arm takes the
tagged green sweater to the mesh bag's right half. Each grasp closes across the garment's 5 cm width at its
centre, away from the tag, with the fingers opened only 8 mm wider than the garment on each side; it closes
slowly to a bounded squeeze, lifts, sets the garment down on the container floor, opens and rises. Two garments
sit side by side in each container. The pick targets include a small per-arm reach correction measured in
simulation. The plan reaches full score after about 1,000 control steps (50 s).

## Fidelity notes

- Garments are rigid blocks standing in for folded clothes, scaled to about a third of real size (a folded
  shirt is about 25 x 35 cm) so that the 9.4 cm gripper can hold one across its 5 cm width. Cloth isn't available
  in a task file.
- The hand-wash tag is a separate loose card lying on the garment, not sewn on: a task file can't join two
  boxes, and each box has one colour. The scorer treats a tag that comes off as a hard 0, as a lost handling tag
  would be in a laundry.
- The mesh bag is a rigid open box. The containers are fixed in place, like baskets clipped to a sorting table.
- Friction is the simulator default for every object; it can't be set per material in a task file.
- Grasp, contact and release are approximate. The release check uses the fingertip positions from the arms'
  joint states by forward kinematics (below).

## Dependencies

- yam-isaac-lab-arena at commit `0059d02a1a91bc6834c1d0846022891df0c78730` (2026-11-01), with Isaac Sim 6.1,
  Isaac Lab 3.0 and Isaac Lab Arena 0.3.
- `success()` checks the release with the arms' fingertip positions. The task-file hooks receive only object
  states, so it reads the joint positions of the `yam_arena` embodiment that calls it (found up the call stack)
  and runs `yam_arena.urdf.forward_kinematics`. This depends on the embodiment's private state at the commit
  above; if the state can't be found, scoring raises an error instead of guessing.
