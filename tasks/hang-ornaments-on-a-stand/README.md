# Hang two glass baubles by their wire hooks on a wooden ornament stand

A wooden ornament stand (a thick post with four thin dowel branches) stands on the table between two ornament boxes.
Each box holds one glass Christmas bauble, cap up, nested in a card insert, with a wire ornament hook already fitted
to its cap. The robot has to take each bauble out of its box and hang it on a branch of the stand by its hook.

**Instruction:** "Hang both glass baubles on the ornament stand's branches."

An ornament hook is an open S-shaped wire: a straight shank up from the cap that curls over into a crook. Nothing
latches. A bauble stays up only because the crook sits over a branch and its weight pulls it down into the bend. To
hang one, the robot has to:

- pick a 60 mm glass ball up by its 16 mm metal cap, the only part that can be held, without squeezing it out of
  the fingers or letting the open hook catch on a finger;
- carry it to the stand with the hook upright. A pinch on the cap is a free pivot, so the ball swings in the fingers,
  and the swing has to die out before the hook can be placed;
- bring the crook down over a 5 mm dowel so that the branch passes into the bend, between the shank and the hook's
  free end. The crook's inside is 18 mm wide, so there are a few millimetres either side, and the target is a thin
  branch sticking out 10 cm from a post;
- let go and withdraw without dragging the hook back off. The bauble then has to hang free and come to rest;
- never drop or knock the glass. A dropped or flung bauble counts as broken and scores 0 for the whole trial.

## Why this task is different

**The hook is on the object, not on the fixture.** In most hanging tasks the fixture carries the hook, peg or slot,
and the robot slides a rigid object onto it: a mug's handle onto a rack's hook, a glass's foot into a rack's slot, a
bottle's mouth over a peg, a cable laid in a hook's cradle. Here it is the other way round. The fixture is a bare
round dowel with no guide, slot or lead-in, and the robot carries a thin open hook on the object and has to thread it
over the dowel. Nothing on the stand helps to centre it.

**A dangling, swinging load.** The bauble hangs below the grip point, so it swings on the cap. The robot cannot place
the hook by placing the hand. It has to let the swing settle and place the hook through the object it is holding.

**Nothing locks it in place.** There is no snap, latch or detent. The bauble hangs only because the crook is the right
way round over the branch. If it is a few millimetres off, the hook rests on top of the branch or slips off when it
is let go, and the simulated bauble does exactly that.

**Fragile and with a single grip point.** Glass can only be held by its cap, and any drop or hard knock fails the
whole trial. Pressing harder on the cap to steady it makes things worse: the cap pops out of the fingers.

**Two objects, two sides.** One bauble is on the left of the stand and one on the right, so a policy has to repeat
the skill, not replay one trajectory.

The same hook-over-a-rod skill appears in many everyday tasks: hanging clothes hangers on a rail, S-hooks and the
utensils on them on a kitchen rail, tools on a pegboard rail, price tags and merchandise on display arms, keys and
bags on hooks, and lab equipment on drying racks. Ornaments add the fragile, dangling load and the tiny open hook.

## Where this comes up

- **Homes.** Decorating a tree is one of the most widespread seasonal household chores: in December, many households
  hang dozens of fragile ornaments, one by one, each by its wire hook. It is a familiar test of careful handling for
  a home robot.
- **Retail visual merchandising.** Shops, garden centres and department stores dress display trees and ornament
  stands every season and restock them as ornaments sell. Merchandise displayers and window trimmers (O*NET
  27-1026.00) and retail stockers (O*NET 53-7065.00) do this by hand, and it is slow because each ornament has to be
  hooked over a branch individually.
- **Hotels, offices, hospitals and events.** Lobbies, wards and venues are decorated and taken down every year,
  usually by facilities staff.
- **The same skill elsewhere.** Hanging garments on hangers on a rail, utensils and tools on hooks and rails, and
  hooked items on display arms all depend on bringing a hook over a rod and letting go so that it holds.

## Potential impact

- **Placing through a held, swinging object.** The robot has to control the position of a hook that is 7 cm from
  its fingers and swinging on a pivot, not just the position of its hand. Policies that only move the gripper to a
  target fail here.
- **Millimetre placement on a thin, unguided feature.** An 18 mm opening has to be centred over a 5 mm dowel at the
  end of a 10 cm cantilever, with no chamfer or lead-in.
- **Grip force matters.** The cap is small and smooth. Too little force and it slips; too much and it squeezes out of
  the fingers. The demo shows that a moderate pinch works and a full squeeze does not.
- **Gentle handling as a hard rule.** A single dropped or flung bauble zeroes the trial, as breaking real glass would.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: the stand, boxes and baubles, the hanging check, scoring and the instruction |
| `ornkit.py` | Builds the glass baubles (ball, eight-sided crown cap and wire hook as one rigid body) and the fixed round rods and boxes |
| `demo.py` | The scripted demo |
| `test_scoring.py` | Offline tests of the scoring rule and the hanging geometry (no simulator) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang both glass baubles on the ornament stand's branches." \
  --scorer task_score --max-steps 2400
```

Run limit: 2,400 steps (2 minutes at 20 Hz). The demo finishes in 890 steps, so the limit leaves more than double
that for a policy that places the hooks more slowly or has to try again.

Second layout: set `ORN_LOT=B` in the environment (default `A`). The stand and both boxes are moved by 1 to 1.5 cm.

## Scripted demo

```bash
python demo.py task.py out_dir                  # layout A; ORN_LOT=B for layout B
python demo.py task.py out_dir --mode still     # control: the arms do not move
```

For each bauble in turn, first the left hand with the red one, then the right hand with the blue one:

1. The hand comes straight down over the bauble, fingers open, closing across the hook's plane so that neither
   finger lands on the crook. It pinches the cap moderately (to about a 10 mm finger command on the 16 mm cap)
   and lifts the bauble 7 cm out of the insert.
2. It carries the bauble over its branch, 2.5 cm in from the branch's tip, comes down to 2 cm above where the hook
   will rest and waits 1 s for the swing to die out.
3. It checks where its fingers actually are and corrects for any shortfall: an arm holding a load out at reach
   stops up to 1.3 cm short of where it was sent. It does this twice.
4. It lowers the crook over the branch until the branch is inside the bend, opens its fingers and rises 5 cm.

Every target pose comes from the layout constants in `task.py`. The only feedback is the measured position of the
hand in step 3; object states are read only for the result file.

## Scoring

Tools: the trial video and a ruler.

**Hanging on a branch** means all of these hold:

- **On a branch.** The branch passes through the bauble's crook, between the shank and the hook's free end, under
  the top of the bend: within 4.5 mm of the crook's centre across the hook and within 6 mm along it. Any of the
  four branches counts.
- **Let go.** Both grippers are more than 4 cm from the bauble's cap.
- **Still.** The bauble is moving slower than 1 cm/s.

**Broken:** at any time during the trial, a bauble moving faster than 1 m/s (it has been flung or dropped from
height), or a bauble whose lowest point comes within 3 mm of the table outside its box (it has been dropped).

| Outcome | Score |
|---|---|
| Any bauble broken at any time | **0** for the whole trial |
| Both baubles hanging on branches, let go and still, for 2 s together | **1.0** |
| At least one bauble hanging on a branch, let go and still, at the end | **0.5** |
| Anything else (baubles left in their boxes, held in the hand, or resting on the stand without hanging) | **0** |

Every outcome falls into exactly one row. A bauble held in the hand, or resting on the base or on top of a branch,
earns nothing. A bauble can't be wedged into place either, because the hanging check needs the branch inside the
hook's bend. In code: `on_branch()`, `_watch()` and `score()` in `task.py`.

## Verification

| Run | Score | Notes |
|---|---|---|
| Scripted demo, layout A | 1.0 | Red bauble on the left branch from step 460, blue on the right; full score at step 890 (45 s) |
| Scripted demo, layout B | 1.0 | Same script; red hanging from step 430, full score at step 890 |
| Arms still (control) | 0.0 | Both baubles stay in their boxes |
| Dream-1, 2,400 steps | 0.0 | The right gripper went to the blue bauble's box and hovered over it; the left arm wandered over the table. No bauble was lifted |
| `pytest test_scoring.py` | 9 passed | Score tiers, the broken-glass gate, which positions count as hanging (on, beside, below, off the end, hook turned along the branch), the start layout, the hook fitting the branch |

Failure modes that came up while building the demo, all of which the scorer handles:

- **Hook resting on top of the branch.** When the crook came down 1 cm off, the hook landed on top of the branch and
  the bauble stood propped up. It scores 0 because the branch is not inside the bend.
- **Bauble squeezed out of a hard pinch.** It drops back into its box. That is not a drop, because it lands inside
  the box.
- **Bauble falling after release.** It falls when the hook is let go beside the branch. That counts as broken,
  score 0.

## Runtime and seeds

About 3 minutes per demo run and about 7 minutes per Dream-1 run on one L40S, including about a minute of start-up.
`object_jitter=0`. The layout is fixed per lot (A or B), and there is no randomness in the task.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table (x forward, away from the arms; y to the
left). Layout A is given first, with layout B in brackets.

- **Ornament stand:** wooden, at x 33, y 0 [B: x 34, y 1]. Base 14 x 14 x 1.2 cm, post 2.5 cm across and 20 cm
  tall. Four 5 mm round dowel branches, each 10 cm long from the post's surface with a 9 mm gold bead at its tip:
  left (+y) and right (-y) at 13 cm above the table, back (+x) and front (-x) at 16.5 cm.
- **Ornament boxes:** two card boxes with walls 2.5 cm tall and a 6.2 x 6.2 cm cell, centred at (25, 20) on the
  left and (25, -20) on the right [B: (24, 21.5) and (26, -19)]. Each has a card insert with a 4 cm square opening
  that the ball nests in.
- **Baubles:** two 60 mm glass balls, red in the left box and blue in the right, 25 g each, cap up. Each has an
  eight-sided gold crown cap 16 mm across, standing about 2 cm proud of the glass, and a wire ornament hook (1.8 mm wire). The hook's
  straight shank rises 2.6 cm above the cap and curls over into a crook 2 cm across, toward the stand (+x), with
  its free end turned 1 cm back down.
- **Robot:** both arms start in the raised ready pose with their grippers open.

## Fidelity

- **The bauble is one rigid body.** The ball, cap and hook are welded into a single rigid object. In a real
  ornament the hook is a separate wire loosely looped through the cap and can pivot and bend; here it is fixed
  upright, which makes the hook easier to aim than a real one.
- **The glass does not break in the simulation.** Breakage is a speed threshold (1 m/s, about the speed of a drop
  from 5 cm) plus a drop check (the ball's lowest point within 3 mm of the table outside its box).
- **Damping stands in for friction.** The swing of a hanging bauble is damped (linear 1.0, angular 3.0) in place of
  the friction between hook and branch and the air drag on a real one. A real bauble settles in a second or two too.
- **Centre of mass.** It is 3 mm below the ball's centre, standing in for a glass ball's thicker bottom.
- **Fixed scenery.** The stand and boxes are fixed; the stand does not wobble.

## Real-world build

- **The stand:** a wooden ornament or jewellery display stand with dowel arms (or a 25 mm dowel post on a square
  base with four 5 mm dowels pushed into holes, small beads glued to their tips) at the heights above.
- **The baubles:** two 60 mm baubles with standard wire ornament hooks, bent to the shape above. Use **shatterproof**
  60 mm baubles for safe trials. A drop still scores 0 by the rule above, so the shatterproof bauble does not change
  the score; it only keeps the work area safe.
- **The boxes:** two small card boxes with a card insert cut with a 4 cm square hole, as in a retail ornament box.
- **Scoring:** filmed from above and from the side. The "on a branch" check is visible from the side, and the
  speed thresholds can be read from the video by frame-stepping.

## Third-party assets

None. Every object is built from primitive shapes in `ornkit.py` and `task.py`.
