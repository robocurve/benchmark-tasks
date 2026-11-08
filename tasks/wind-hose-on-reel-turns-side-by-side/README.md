# Crank a hose reel with one hand while the other lays the turns side by side

A hand-crank garden hose reel stands on the right half of the table. Its drum turns on a horizontal axle running
left-right; a crank handle sits on the drum's right end, right of the robot's midline. A green 10 mm hose, its inner
end clamped to the drum core, lies on the table in a U and ends in a black nozzle. The robot must wind the hose on
neatly and let go; a one-way pawl on the axle then holds the drum.

Turning the crank winds the hose in under the drum. Where each turn lands across the drum depends on where the hose
is held as it goes on: left alone, it winds on as a loose open spiral (about 4 cm between turns) that runs to the far
flange; held in one place, the second turn climbs onto the first. A neat reel needs each turn laid against the one
before, which means one hand turns the crank while the other walks the hose across the drum, about one hose width
per turn, at the same time. Hose reel manuals ask for exactly this ("Distribute the hose evenly over the surface of
the drum assembly"), and a reel maker blames tangled, overlapping hose on a guide that is out of line.

The instruction states only the goal ("neatly"); what counts as neat is stated under Scoring.

## Objects

| object | type | size | material / colour | where (cm; x forward from the arm bases' midpoint, y left) |
|---|---|---|---|---|
| reel drum | cylinder on a revolute axle (y axis), 150 g, light drag, one-way pawl | 6 cm dia x 14 cm | grey PVC pipe | axle x 34, 7.5 cm above the table; drum y 7-21 |
| flanges (2) | discs fixed to the drum | 12 cm dia x 6 mm | red PLA or plywood | y 6.35-6.95 and 21.05-21.65 |
| uprights (2) | fixed posts carrying the axle | 16 x 6 x 71 mm | dark grey | x 34, y 5.8 and 22.2 |
| crank arm | bar fixed to the drum's right end, through the axle | 14 x 6 x 118 mm | dark grey | y 4.7 |
| counterweight | block on the crank arm opposite the handle, 20 g | 22 x 6 x 22 mm | dark grey | 5 cm from the axle; balances the handle so the crank does not swing round when let go |
| crank handle | ball on a pin, on an 8.4 cm stem from the crank arm (handle + stem 20 g) | 24 mm dia ball | black | 5 cm from the axle, y -5; starts straight up |
| hose | cable, inner end clamped to the drum core at its bottom | 10 mm dia, 60 cm, 60 g | green PVC | clamp at y 7.8; 16 cm straight toward the robot, 10 cm radius U to the left, back out to the nozzle |
| nozzle | rigid end on the hose | 40 x 16 x 16 mm, 20 g | black | starts at x 31.9, y 27.8 |

## Why two arms (simultaneous)

The crank handle circles 5 cm right of the midline (y -5 cm), past the left arm's reach (left arm y > -2 cm); every
place the hose can be guided onto the drum (y 7.3-20.5 cm) is past the right arm's reach (right arm y < +2 cm).
Offline IK with the gripper straight down (`scratch/ik_final.py`, output in `scratch/ik_final.out`): the right arm
reaches all 24 points of the handle circle (x 29-39, 3.1-13.1 cm above the table, wrist turned 90 deg); the left arm
reaches the guide line x 24, y 7.8-12, fingertips just above the table. Guiding only works while
the drum turns: with the crank turned and the hose not held, it winds on at 4.3 cm per turn (score 0); held still it
piles up. So the two hands work at the same time, one turning, one walking the hose across.

## How this differs

- Our "Bring a spring cord reel's work light home into its dock": the reel's spring winds the cord by itself; the
  robot only carries the light home under spring pull and must never let go. Nothing about how the cord lies.
- Our "Pay a multicore off a free-turning drum to its cradle": the reverse act on a vertical drum. The arm pulls
  cable off and must stop the drum from overrunning; the drum is never driven.
- Our "Hold a reel's spring-loaded cord at a length mark while the other hand sets the reel lock": a spring reel
  held at a length while a lock is set; no winding.
- Board "Take up patch-cord slack on spools, then plug in" (our pending task): one turn of cord carried round two
  fixed posts, then a plug. The posts never turn; there is no lay pitch.
- Here the robot drives the drum itself through a crank (one arm moving the handle round a circle, two turns) while
  the other arm sets where each turn lands. The tested ability is two-hand coordination on one line: the lay pitch
  comes from matching the guide's motion to the drum's turning. The end state (a single layer of side-by-side turns)
  is something none of the above produces.

## Context and sources (quoted from the pages)

- O*NET 37-3011.00 Landscaping and Groundskeeping Workers, task: "Water lawns, trees, or plants, using portable
  sprinkler systems, hoses, or watering cans." (onetonline.org/link/summary/37-3011.00)
- Yardworks hose reel, model 059-0819-6, instruction sheet (Canadian Tire; media-www.canadiantire.ca/manual/product/
  0590819/0590819p-en-fr-7dc297df-8b22-4fa1-aae3-9e575d368e9a.pdf): "Distribute the hose evenly over the surface of
  the drum assembly." The same sheet lists a "Crank Handle".
- Hoselink manual hose reel support page (hoselink.com.au/pages/support-product-support-manual-hose-reel): "Use the
  crank handle to wind the hose into the box evenly." and, on a hose reel cart: "It sounds like the hose guide is out
  of alignment causing the hose to tangle and overlap."

## Paper fields

- Skill tags: winding, routing/seating, constraint-keeping.
- Industry: landscaping and groundskeeping (also fire, wash-down and workshop hose reels).
- Two-arm need: simultaneous (and reach-forced: handle right-only, hose guide left-only).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (hose simulation, the U-shaped start layout,
  scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Wind the green hose onto the reel neatly, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop on the live handle and hose. The right arm pinches the ball handle (wrist camera looking left at the
drum) and walks it round the axle in 10 deg steps from its live position. The left arm puts its fingers round the hose
on the table 10 cm in front of the drum, partly closed (an 18 mm slot round the 10 mm hose, so the hose slides through),
and moves left in step with the drum, 10.5 mm per turn. The crank stops once 41 cm is on the drum (about 1.7 turns). The
right arm lets go of the handle and the left arm opens; both lift clear and the pawl holds the drum.

Demo result: score 1.0, success at step 533 of 1,200 (42 cm wound when the crank stopped, about 1.7 turns, one layer,
turns 10.6-13.2 mm apart; the end state is checked after release, see release sag below). Recording: `demo/` (top
camera and both wrist cameras).

## Scoring (`score()` at the end; `success()` must hold 1 s)

Score 0 if, over the trial, the drum turns more than 0.3 rad in all (either way, however slowly, summed every physics
step) while no closed right gripper is on the handle (turning the drum by its flange or the hose is not using the
crank), or, at the end, a gripper is within 3 cm of the hose, nozzle or handle, or the wound hose is not neat. On the handle
= the right gripper closed (finger joint under 0.030 m, open is 0.047) with its tool point within 2.5 cm of the handle
centre. On the
drum = segment centres within 1.5 cm of the first-layer radius, between the flanges, counted from the clamped end.
Wound = the hose on the drum at the end, counted as the most that was on the drum while a hand was on or within 3 cm
of the hose, nozzle or handle, but never more than 6 cm above what is still on the drum at the end (the pawl stops
unwinding, so only the release sag can take hose off). In end-state terms: the hose on the drum at the end must be
within 6 cm of the length that was wound. Neat (checked on the
end state) = one layer (no wound part more than 4 mm above the first layer) and every turn 8-17 mm across the drum
from the turn before it, always the same way (the hose is 10 mm wide, so touching turns are about 11 mm apart), and no
hose bunched loose between the flanges.

| end state | score |
|---|---|
| neat, 27-33 cm wound, released | 0.3 |
| neat, 34-36 cm wound, released | 0.6 |
| neat, 37 cm or more wound (about 1.5 turns), released | 1.0 |
| anything else | 0 |

Real trial: measure the turn spacing with a ruler across the drum, check by eye/photo that no turn sits on another,
mark the hose at 27, 34 and 37 cm from its clamped end and see which mark is on the drum.

Release sag (measured): once the guide opens there is no hose tension, and the last 4-6 cm of the wound end sags off
the underside of the drum (tier runs: 32 cm at the crank stop -> 28 on the drum at the end; 36 -> 30). Scoring
therefore takes the most hose on the drum while a hand worked it and allows up to 6 cm of sag: stopping at 40 cm
scores 1.0 (scripted check), as does the demo's 42 cm. The end state must still be neat.
There is room to wind well past that (60 cm hose, flanges 14 cm apart).

Spin latch margin: passing runs (demo, tier and stop checks) total 0.04 rad of drum turn without the handle held
during the wind and 0.16 rad by the end (release settling and sag), against the 0.3 rad limit. The pawl stops
back-turn, so re-gripping the handle mid-wind adds only the drum's forward coast (light drag, 150 g), well under that.

Narrow 0.3 band: neatness needs at least one turn laid beside the previous one, so stopping at about 30 cm (exactly one
turn) scores 0, and the 0.3 band (27-33 cm) is therefore narrow in practice: a stop a little over 30 cm up to 33 cm. The
0.6 band (34-36 cm) and 1.0 (37 cm or more) are wider.

## Shortcuts tried (scripted, same layout; each scored by `score()`)

| shortcut | what happens | score |
|---|---|---|
| do nothing | 4 cm on the drum | 0 |
| crank only, hose not guided (one arm) | 42 cm wound as an open spiral, turns 41-71 mm apart | 0 |
| crank while holding the hose in one place | second turn climbs onto the first (13.4 mm lift), turns 0.6-2.8 mm apart | 0 |
| wind neatly but keep holding the handle | 42 cm neat, a gripper still on the handle | 0 |
| turn the drum by its flange rim with the left hand, fast (no crank, pushing) | 0.3 rad of drum turn without the handle by step 37 (latched); hose not laid | 0 |
| turn the drum by its flange rim with the left hand, slowly (about 0.15 rad/s) | latched at 0.3 rad by step 42; hose not laid | 0 |
| tier check: neat, stop at 32 cm | 32 cm at stop, 28 cm on the drum at the end, neat | 0.3 |
| tier check: neat, stop at 36 cm | 36 cm at stop, 30 cm on the drum at the end, neat | 0.6 |
| robustness check: neat, stop at 40 cm | 40 cm at stop | 1.0 |

## Runtime and seeds

1,200 steps (60 s at 20 Hz) per trial. One fixed layout (`object_jitter=0`); nothing is randomised.
