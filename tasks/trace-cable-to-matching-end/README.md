# Find which plugs belong to two devices when the cables run hidden under a cover

Five identical black cables leave five fixed device boxes standing in a row at the far side of the table (yellow,
red, green, blue, orange from left to right, seen from the robot). Each runs straight towards the robot into a grey
cable cover. Under the cover, out of sight, each cable bends round a guide post, runs diagonally across under the
cover, crossing other cables, bends round a second post and comes out of the cover's robot side straight towards the
robot. Their identical black plugs lie in a row in front of the cover, every one away from its own box: the plug
lying in front of the red box and the plug lying in front of the blue box belong to other boxes; the red box's plug
lies at the far right and the blue box's at the far left. Two plug holders stand near the robot, one with blue walls
(left) and one with red walls (right).

The robot must put the red box's plug in the red holder and the blue box's plug in the blue holder, and leave the
other three plugs where they lie. The routes cannot be seen, so looking cannot answer which plug is which. The
robot has to find out by interacting: take hold of a cable where it leaves its box and pull it sideways along the table.
That draws cable out from under the cover, the cable slides round its guide posts, and its own plug slides
5-6 cm towards the cover while the others stay put. Then it carries that plug to its holder.

Both arms are needed, by reach (for a gripper pointing down at the table; a steeply tilted grip reaches further). Only the left arm reaches the red box's cable at the far side and the blue box's plug
at the far left; only the right arm reaches the blue box's cable and the red box's plug at the far right. So each
device needs one arm to tug its cable and the other to take its plug.

## How this differs from similar tasks

Tasks where crossed cables can be followed by eye test visual tracing. Here the crossing is hidden, so the ability
tested is finding a cable's far end by acting on it and watching the result (a tug test), then placing two plugs
with two arms. Tasks that pull out or reroute a cable through clips or around obstacles do not hide which cable is
which.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (the cable simulation, the code that lays
  the cables in their starting route at every reset, and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
    --instruction "Put the plug of the red box's cable in the red holder and the plug of the blue box's cable in the blue holder. Leave the other plugs where they are."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1600
```

Closed loop, reading the live cable and plug poses: the right arm hooks the blue box's cable 10 cm in front of
the box between almost closed fingers (it can slide through them) and pulls it 9 cm sideways, notes which plug slid,
moves back and lets go; the left arm does the
same with the red box's cable. Then the left arm lays the plug that slid on the first tug in the blue holder and
the right arm the plug that slid on the second tug in the red holder. Result on the final code: the tugs move the blue box's
plug 6.0 cm and the red box's 5.3 cm, every other plug 1 mm or less; both plugs are then laid flat in their own
holders; `task_score` 1.0, success after 766 steps (38 s).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if at any control step the yellow, green or orange box's plug is more than 5 cm (3D) from where it lay at
  the start, the red box's plug lies in the blue holder or the blue box's plug in the red holder, or a cable has
  been pulled out of its box (its end at the box moved more than 1 cm). These are
  tracked over the whole episode in module state that is cleared only at the start of each episode, so they cannot
  be undone later.
- A plug **lies in a holder** when it is between that holder's side walls, on the table, inside the slot or sticking
  out of its open far end by at most 1 cm. It is **released** when both gripper TCPs (between the finger pads) are at
  least 3 cm from its centre.
- **1.0**: at the end both plugs lie released in their own holders (red box's in red, blue box's in blue).
- **0.5**: at the end exactly one of the two lies released in its own holder.
- **0** otherwise, including doing nothing (the two plugs start about 12 and 13 cm from their holders) and dragging a plug along
  the table up to a holder without getting it into the slot.
- `distance()`: sum of the two plugs' distances to their holders' centres (m), 2.0 after a violation.

What a guess earns: the plug in front of each named box, the plug at the mirrored position and the plugs nearest
each holder all belong to other boxes, so those position-based picks score 0 (the first two were run, the third
moves another box's plug more than 5 cm). A blind pick of one of the five plugs for a holder is right 1 time in 5
and then earns at most 0.5; a wrong pick moves another box's plug more than 5 cm or puts a plug in the wrong
holder, which scores 0 whatever else is done. The layout is fixed, so a policy that has learnt it (blue's plug is the
leftmost, red's the rightmost) can skip the looking, but it still has to draw cable out at each named box first: a
plug carried to its holder against a taut cable pulls the cable out of its box (`onearm`, 0). Tugging a wrong box's
cable hard enough to slide its plug more than 5 cm also scores 0, so the tug has to be chosen or gentle.

Reach (offline IK, 13 grip yaws, gripper height 1 cm): the left arm reaches the red box's cable (x 43, y +12), the blue
box's plug (17, +30) and both holders, but not the blue box's cable (43, -12) or the red box's plug (17, -28); the
right arm is the mirror image. Under the cover (x 25-40) a reach at y -12 / +12 only exists around x 30, where the
2.5 cm gap under the plate blocks a gripper. So no single arm can complete either plug, and neither can reach all four
places.

## Test variants (`TRACE_DEMO=<mode> python cabledemo.py task.py out`), all scored by the same `score()`

| Variant | What it does | Score |
|---|---|---|
| `correct` (demo) | both tugs, each plug in its own holder, released | 1.0 |
| `tier05` | both tugs, only the blue box's plug placed and released | 0.5 |
| `nothing` | waits | 0 |
| `hold` | both plugs placed, but the grippers stay closed on them | 0 |
| `onearm` | left arm only, no tugs, knowing the answer: lifts the blue box's plug to the blue holder; with no cable drawn out at the box the drag pulls the cable out of its box | 0 |
| `front` | no tracing: the plug lying in front of each named box (orange's, green's) | 0 |
| `mirror` | the plugs at the mirrored positions (green's, orange's) | 0 |
| `random` | two other plugs (yellow's, green's) | 0 |
| `swap` | both tugs, then each plug in the other holder | 0 |
| `push` | both tugs, then the blue box's plug set down on the table just outside the blue holder (4.5 cm from the slot centre), released | 0 |

Result files: `demo/result.json`; each variant's `result.json` and its logged positions are kept as
`scratch/sc_<variant>.json` and `.log`.

## Runtime and seeds

- Run limit 1,600 steps (80 s at 20 Hz). One trial takes about 3-6 min of wall time on one GPU, including the
  1-2 min Isaac Sim start-up.
- One fixed layout (`object_jitter=0`). At every reset the cables are laid in the same route and left to settle for
  2 s before the first observation; repeated runs start from the same state.
- The cables cannot stretch. Carrying a plug to its holder needs cable drawn out at its box first (the tug does
  this). Without it, a real taut cable stops the plug short of its holder (0). The simulated arm can drag harder than
  a real one, so each simulated fixing lets go above 300 N rather than letting the chain stretch; a cable pulled out
  of its box scores 0, so the outcome is the same as on a real station. The guide posts are smooth (friction 0.15).

## Sources

1. O*NET-SOC 49-9051.00, Electrical Power-Line Installers and Repairers, task statement
   URL: https://www.onetonline.org/link/summary/49-9051.00
   Quote: "Test conductors, according to electrical diagrams and specifications, to identify corresponding conductors and to prevent incorrect connections."
   Supports: identifying which conductor at one end corresponds to which at the other is a recognized occupational task.
