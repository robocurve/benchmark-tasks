# Fit locating pins to a fixture and load a bracket onto them so it is located

An aluminium fixture plate has two 12.4 mm locating holes 100.0 mm apart. In two racks hang three loose steel
locating pins: two round, one diamond (relieved). A 150 x 60 x 8 mm aluminium bracket lies on rails, with two
16.4 mm holes 101.0 mm apart. The robot fits the pins, pushing each one home, and loads the bracket onto them so
that it sits flat on the rest pads, located. Which two pins go in, which hole each goes in, and which way the
diamond faces are the robot's to work out: the instruction names none of them.

## Why it is hard

1. **The bracket's holes are 1.0 mm farther apart than the fixture's** (a normal error for a welded, fabricated bracket), against
   0.2 mm of clearance round each pin head. Over two round pins the bracket overlaps their heads by 0.3 mm at
   best and cannot go down. Over a round pin and the diamond with its lands square to the line between the holes
   it can: the round pin fixes its position, and the diamond's 8 mm relief takes up the 1.0 mm along the line.
   A diamond turned a quarter turn overlaps by 0.28 mm: a real one binds, but in the simulator the contact give
   lets the bracket down (GPU run below), so the scorer judges it by the diamond's angle. (Computed,
   `fitgeom.py`, from the actual hole polygons and head shapes: tests `Geometry`.)
2. **The window is small.** With the pair in, the bracket goes down only with its hole over the round pin within
   +-0.16 mm, square to the line within +-0.18 degrees, and the diamond within about 45 degrees of square. The scorer asks for 0.3 mm and 0.3 degrees (the window plus the simulator's contact give).
3. **A pin cannot be turned once it is home.** It is locked there (a rule, see Simplifications), so the diamond
   must be turned before it is pushed home. The scorer accepts a diamond within 15 degrees of square; the bracket
   still goes down with it turned further (in the simulator, even a quarter turn).
4. **The grips.** A pin is held by its 16 mm head with 8 mm commanded between the pads, the bracket by its 60 mm
   width with 52 mm. The bracket is 185 g, a pin 34 to 44 g.

## Scoring

| Condition | Rule |
|---|---|
| Pin home | flange within 0.5 mm of the plate, axis within 3 degrees of vertical, head up, axis within 1 mm of a hole's centre |
| Located set | one round pin home in one hole, the diamond home in the other, its two lands within 15 degrees of square to the line between the holes |
| Seated | underside within 0.3 mm of the rest pads' top (25.0 mm), level and square to the line within 0.3 degrees, its hole over the round pin within 0.3 mm of the pin's axis |
| Clear and still | last 1 s: both grippers' finger zones 25 mm or more from the bracket and every pin; the bracket and its hole centres move under 0.3 mm, turn under 0.3 degrees |

| Outcome | Score |
|---|---|
| Located set home, bracket seated, grippers clear, still | 1.0 |
| Located set home, bracket not seated | 0.5 |
| Any one pin home, no located set (two round pins, diamond turned wrong, one pin; a bracket seated over them earns nothing more) | 0.25 |
| Anything else, or a pin or the bracket off the table | 0 |

Located pose: with the round pin in hole B the bracket's centre is 0.5 mm to the left of the fixture's centre
(toward A), with it in A, 0.5 mm to the right. Success: 1.0 held for 1 s. Doing nothing scores 0.

## Results in simulation

L40S, Isaac Lab-Arena, `-E object_jitter=0 -E control_hz=20`, one trial each, run limit 1,200 steps (60 s). Run
2026-11-14.

| Run | What it does | Score |
|---|---|---|
| Nothing (`DEMO_HOLD_ONLY=1`) | both arms hold still | 0.0 |
| Scripted demo | both arms, the phases below | 1.0 (held from step 1030) |
| Left arm alone (`DEMO_IDLE_ARM=right`) | diamond into A, then the bracket onto it | 0.25 |
| Right arm alone (`DEMO_IDLE_ARM=left`) | a round pin into B | 0.25 |
| Dream-1 | the task's instruction, full run limit | 0.0 |
| `DEMO_VARIANT=two_round` | both round pins in (B, A), the bracket lowered over them | 0.25 (bracket stays up 4.0 mm, tilted 4.1 degrees) |
| `DEMO_VARIANT=diamond_turned` | diamond into A without its quarter turn | 0.25 (bracket goes down: contact give) |
| `DEMO_VARIANT=bracket_only_one_pin` | a round pin in B, the diamond left in its rack, the bracket lowered | 0.25 |
| `DEMO_VARIANT=smoke` | right arm only: a round pin into B, pushed home (the lock fires) | 0.25 |

Either arm can in principle do the whole job; the task does not claim two arms are needed.

## Demo

Open loop: timed waypoints from the known start layout, nothing read from the simulator. Both grippers point
straight down. The left arm sets the diamond in A, turning its hand a quarter turn on the way so the lands end up
square to the line between the holes; the right arm waits over its rack, then sets a round pin in B; the left arm
holding the bracket lowers it once the right arm has moved aside. The arms take turns at the fixture (the two
holes are only 100 mm apart).

| Time (s) | Left gripper | Right gripper |
|---|---|---|
| 0 to 7.5 | down onto the diamond's head (across its two lands), lift | down onto a round pin's head, lift |
| 7.5 to 22.5 | carry it over A turning a quarter turn, lower to the hole's mouth, push the shank 10 mm in while held upright, let go, close, press the head home, up | wait over the rack |
| 22.5 to 41.0 | over the bracket, grip its long sides, lift, wait | carry over B, lower, push 10 mm in, let go, press home, up, aside |
| 41.0 to 52.5 | carry the bracket over the pins, lower to 1 mm over the heads, then slowly onto the pads, let go, up, aside | aside |

The demo ends about 53 s in; success starts counting once the bracket has been still and both hands clear for 1 s.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
  -E control_hz=20 --instruction "<the instruction>" --scorer task_score --max-steps 1200
```

For Dream-1 use `--policy dream`. `DEMO_IDLE_ARM` (left or right) drops one arm's plan, `DEMO_VARIANT` runs a
counterfactual, `TASK_TRACE=trace.jsonl` writes one line per control step.

## Simplifications

- Real slip fits are about 0.02 mm. The physics engine cannot resolve that, so every fit has 0.4 mm of play (12.4
  mm holes for 12.0 mm shanks, 16.4 mm holes for 16.0 mm heads) and the spacing error is 1.0 mm, so two round pins overlap by more than the simulator's contact give (0.2 mm). The ratio of
  clearance to error is not the real one.
- Round holes are 16-sided rings of turned boxes whose flats are tangent to the true circle; each has a 45 degree
  lead-in (2 mm deep in the plate, 1.5 mm at the bracket's underside), and the pin heads and shank tips have
  stepped 0.5 mm chamfers. The diamond's head is two 2 mm lands on the 16 mm circle and a relieved 11 x 8 mm body
  between them, all boxes.
- A pin pushed home is locked by a rule: from the step its flange is within 0.5 mm of the plate (upright, axis on a
  hole) its pose is rewritten each control step to the hole's centre, with the turn it had. It stands for a ball
  detent or press fit; real locating pins are often just a slip fit and stay put by their weight.
- Rigid bodies, friction 0.4 for every part, steel and aluminium at their real densities (the bracket is 185 g).
- The bracket's top face is flat: it is gripped by its long sides, 5 mm of them.

## Real build and reset

A fixture plate with two reamed 12.4 mm holes, 19 mm deep, 100.0 mm apart; four 5 mm rest pads; two round and one
diamond locating pins (12 mm shank, 22 mm flange, 16 mm head; the diamond relieved to 8 mm across flats) with a
ball detent; a 150 x 60 x 8 mm aluminium bracket with two 16.4 mm holes at 101.0 mm. A feeler gauge and a dial
indicator on the bracket, a photo from above for the diamond.

Reset: pins out of the plate (release the detent) and back in the racks, the diamond with its lands along y; the
bracket on its rails. Check both holes are empty and clean.

## Files

| File | What it is |
|---|---|
| `task.json`, `task.py` | The task text; the parts, the pin lock, `success()`, `score()`, `demo()` and its variants |
| `layout.py`, `fitgeom.py`, `scoring.py` | Every dimension and shape; the fit geometry in plain NumPy; the scorer |
| `mechkit.py`, `rigstate.py` | Build the parts as compound rigid bodies; read both grippers each step |
| `tests/` | Offline tests: `python3 -m unittest discover -s tests` |

Third-party assets: none.
