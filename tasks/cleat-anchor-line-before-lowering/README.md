# Make the anchor line's bitter end fast, then lower the anchor over the side

A small-boat anchoring mock-up for two arms. A wooden deck (fixed, 6 cm high) covers the near part of the table;
its far edge is the boat's side, and a blue seabed sheet lies on the table beyond it. A 55 cm white anchor line lies
straight across the deck. Its inboard (bitter) end carries a red toggle, its outboard end a black anchor block. A
grey slotted pocket on the deck, the bitter-end cleat, holds the line once the toggle is dropped in and the line is
led out through the slot. The robot is asked only to anchor the boat "so that neither the anchor nor its line can be
lost overboard". It must work out that the bitter end has to be made fast before the anchor goes
over, then lower the anchor (not drop it) onto the seabed and let go of everything.

Why two arms: the toggle and cleat are on the left (y +21 cm), out of the right arm's reach, and the anchor starts at
y -36 cm, out of the left arm's reach (offline IK: right arm 34-54 mm error at the cleat, left arm 63 mm at the
anchor). The left arm makes the line fast while the right arm holds the anchor inboard. Then the right arm pays
it out over the side.

## Context

- eOceanic, sailing tip "Preventing the anchor and chain from 'slipping' away" (eoceanic.com): "The bitter end of the rode must be attached to the vessel".
- Boat-Ed, "Anchoring Your Boat" (boat-ed.com): "slowly lower the anchor over the bow to the bottom." (The same page ties the line off at a bow cleat after lowering; the bitter-end order rule here comes from the eOceanic tip.)
- O*NET 53-5011.00 Sailors and Marine Oilers, task: "Handle lines to moor vessels to wharfs, to tie up vessels to other vessels, or to rig towing lines."

The cleat in this mock-up is a toggle-in-slot pocket, not a real cleat or hitch.

## How this differs from similar tasks

Other board tasks test order rules on rigid items or switches (lock out before work, ground end first, support
before release) or test cleating alone (wrapping a rope on a cleat). In this task the ordered steps are both rope
handling: first the line's end is secured in a fitting so the line cannot run out, then the anchor is lowered under
control while its line pays out over an edge. The scoring checks both the order and the physical results: the line
is led through the slot, the anchor is lowered rather than dropped, it rests on the seabed, and all grippers are
released. The instruction never says which step comes first.

## Requirements

yam-isaac-lab-arena with Isaac Lab-Arena 0.3.1 installed; `cablekit.py` and `cabledemo.py` are in this folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Anchor the boat: get the black anchor over the side onto the blue seabed patch, so that neither the anchor nor its line can be lost overboard. Then let go of everything."
```

## Scripted demo

`python cabledemo.py task.py demo_out` (closed loop). The left arm picks up the toggle, measures where it sits in
the gripper, and drops it into the pocket with the line through the slot. Meanwhile the right arm lifts the anchor
clear of the deck while it is still inboard. After the left arm lets go, the right arm carries the anchor over the
side and lowers it in steps of at most 2 cm until it is about 2 mm above the seabed, steering the anchor's measured
centre to the target spot on the sheet, then opens and rises straight up before moving away. The full score is
reached in about 18 s (about 360-375 steps on three machines).

Shortcut variants (test only, `ANCHOR_CHEAT=<name> python cabledemo.py task.py out`) all score exactly 0:

| variant | what it does | why 0 |
|---|---|---|
| `none` | waits | toggle not in the cleat |
| `hold` | does everything, keeps holding the anchor | gripper within 3 cm at the end |
| `first` | lowers the anchor first, then makes the line fast | anchor passed the edge before the line was fast |
| `hand` | holds the toggle in the pocket by hand while the anchor goes over | not made fast (gripper on the toggle) when the anchor passed the edge |
| `hand_line` | lets go of the toggle but holds the line just outside the slot while the anchor goes over | not made fast (gripper on the inboard half of the line) when the anchor passed the edge |
| `skip` | never makes the line fast | toggle not in the cleat |
| `drop` | makes the line fast, then lets the anchor fall from carry height | anchor moved down faster than 0.5 m/s |

Partial-credit variants: `tier04` makes the line fast and leaves the anchor on the deck (0.4); `tier07` does the whole
task but sets the anchor on the table beside the seabed sheet (0.7).

## Scoring

Tiers by end state (zero gates passed, toggle in the cleat): 0.4 anchor still on the deck; 0.7 anchor past the deck edge, not on the seabed sheet; 1.0 anchor resting on the sheet.

`score()` (end of episode) gives 0 in any of these cases:
- The anchor centre passed the deck edge while the toggle was not in the cleat, or while a gripper was within
  3.5 cm of the toggle or 3 cm of the inboard half of the line. This is checked at the first step the anchor crosses the edge and latched.
- The anchor ever moved down faster than 0.5 m/s, i.e. it was let fall instead of lowered (latched).
- At the end, a gripper TCP is within 3 cm of the line, the toggle or the anchor.
- At the end, the toggle is not in the pocket (inside the walls, below the 20 mm rim), or the line does not leave
  through the 14 mm slot.

Otherwise the score is 0.4 with the anchor still on the deck, 0.7 with the anchor past the edge but not resting on
the seabed sheet, and 1.0 with the anchor resting on the sheet (centre over it, at most 6 mm above resting).
`success()` = score 1.0 with the line at rest. In a real trial, judge the order and the drop from video, and the end
state by eye and with a ruler.

Runtime per trial: 1,200 steps (60 s at 20 Hz), a few minutes of sim. Seeds: the layout is fixed with
object_jitter=0, and the line is not jittered.
