# Lay stickers on a lumber pile directly above the beams below

Lumber is air-dried in piles, with thin sticks (stickers) between each course of boards. The stickers must sit directly
above the beams underneath, so the pile's weight passes straight down instead of bending the boards. The robot lays two
stickers across the first course of a small pile, each above one beam, then lays the next board on the stickers. It is
scored by how far each sticker's ends are from its beam's centreline, and whether the board sits on the stickers.

## Where the step comes from
USDA Forest Products Laboratory, *Air Drying of Lumber* (General Technical Report FPL-GTR-117, 1999), section
"Sticker Alignment": "Stickers should be in good vertical alignment... If the stickers are not aligned, the superimposed
weight will not be transmitted downward from one sticker, through the lumber, and directly down to the next sticker by
direct compression, but will act on the span of the boards to cause warp... sticker alignment depends on the care and
skill of the lumber stacker." Done by: lumber stackers in sawmills and lumber yards.

**What we simplified:** one fixed first course on two beams, one board for the next course, boards about a tenth of
real length and 6 cm wide so the gripper can hold them, and stickers 16 cm apart where 40-60 cm is common.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sticker_lumber.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the two red stickers across the tan boards, one directly above each of the two brown beams underneath, then lay the yellow board on the stickers directly above one of the tan boards." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/sticker-lumber-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sticker_lumber.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the two red stickers across the tan boards, one directly above each of the two brown beams underneath, then lay the yellow board on the stickers directly above one of the tan boards." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/sticker-lumber-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/sticker-lumber-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once everything has been in place and still for 1 s;
otherwise it scores the last step.

## Scoring
- A piece lies **flat** on a surface when its lowest corner is within 4 mm of it and it leans at most 10 degrees: a
  board on a broad face, either way up; a sticker (square in section) on any long face, its length level.
- A sticker lies **across the course** when it is flat on top of the tan boards and bears at least 1.5 cm on each.
- A sticker is **aligned** with a beam when both of its ends are within 5 mm of the beam's centreline, measured along
  the boards. Both ends, so a sticker centred on the beam but laid at an angle doesn't count.

| Part | Condition | Points |
|---|---|---|
| Each beam (2) | a sticker across the course, both ends within 5 mm of the beam's centreline | 0.25 |
| | the best sticker over it within 15 mm | 0.125 |
| | otherwise (two stickers over one beam count once) | 0 |
| Yellow board | each sticker across the course within 15 mm of a different beam, and the board flat on top of them, centre within 1 cm of a tan board's centre (left to right) and of x 26 cm, pointing the same way as the tan boards within 5 degrees | 0.5 |

The score is the sum: 0, 0.125, 0.25, 0.375, 0.5, 0.75, 0.875 or 1.0. A yellow board laid straight on the tan boards,
or on stickers that are along a board, between the beams or both over one beam, earns nothing; so do stickers left on
the table. On a real table, open the grippers, then measure from each sticker's ends to a centreline mark
on each beam, and take a top-down photo.

## Design notes
- **Why both ends.** Real stickers carry the load only where they cross the beam's line; a sticker crossing it at an
  angle leaves most of its length over the board's span, which is what the manual warns causes warp.
- **Why the board counts only on stickers over both beams.** Laying the next board straight on the course, or on
  stickers that are along a board or between the beams, builds the pile the manual warns will warp; it earns nothing.
  The board's height (2 cm above the course) and each sticker's distance from its beam tell these apart.
- **How it differs from other wood-stacking tasks.** BEHAVIOR's `stacking_wood` asks only that six logs end up touching
  each other. Here the skill is placing spacers to a measured line over supports the robot can only see at their ends.
- **Either sticker over either beam, the board over either tan board:** the stacker's choice doesn't change the result.
- **The demo's 2 degree aim.** In the official sim the gripper stops a little short of a quarter turn, and closing the
  fingers turns the sticker with it. Without a correction the sticker ends landed 3-4.7 mm off the line (limit 5);
  aiming 2 degrees further brought them to 1.4 and 2.0 mm (2026-11-03, measured in an instrumented copy of the task
  file with the same waypoints).

## Runtime and seeds
- The scripted demo (three moves with the left arm) reaches 1.0 after about 33 s of simulated time; a full Dream-1 trial runs the 2-minute limit and took 6 minutes on an L40S, Isaac Sim start included (2026-11-03).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- `test_sticker_lumber.py` checks the scoring (every outcome above, each limit, a turned, rolled, upside-down or moving
  piece) and the demo's reach (`python -m pytest -q test_sticker_lumber.py`, no GPU needed).
