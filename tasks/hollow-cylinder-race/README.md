# Find the thin-walled tube among three sealed look-alike cylinders with a two-lane roll race

Three grey cylinders lie in cradles in front of a wooden two-lane ramp. They have the same outer size, the same mass,
the same capped ends and the same paint; only a coloured tape band at each end (red, green, blue) tells them apart.
One is a thin-walled brass tube, one a thick-walled aluminium tube and one a solid glass-epoxy rod. The task is to
put the thin-walled tube in the red bin. Its mass sits nearest the rim, so it has the largest moment of inertia and
rolls down a slope the slowest; a race on the ramp shows it. The ramp has two lanes, so one race ranks only two
cylinders: the robot has to run a second race (the first race's loser against the third cylinder), keep both starts
fair through the shared start gate, and then bin the right one.

**Instruction:** `The three grey cylinders with red, green and blue end bands look the same and weigh the same. One is a thin-walled tube, one a thick-walled tube and one a solid rod. Put the thin-walled tube in the red bin and leave the other two out of it. The orange button opens and closes the start gate at the top of the two-lane ramp.`

## Why this task

Inspectors in receiving and quality control (O*NET 51-9061, inspectors, testers, sorters) sort round parts whose
inside cannot be seen: capped and painted spacers, rollers and bar offcuts made to the same size and weight. A
thin-walled tube passed on in place of a thick-walled tube or a solid rod buckles under a clamp or bearing load, so
binning the wrong one is the failure that matters, and the score is 0 when any other cylinder is in the bin. The
parts here are sealed and equal in mass, so looking and weighing do not separate them. How fast each one rolls does:
with the same outer radius and mass, a rolling cylinder accelerates at g sin(theta) / (1 + k), where its moment of
inertia is k m r^2, and k grows as the mass moves outward (0.50 solid, 0.65 thick-walled, 0.88 thin-walled).

What the task tests is planning and running a fair comparison with a physical constraint: two lanes, three parts.
One race never identifies the slowest of three, so a policy that races once and bins the loser scores 0 even when it
happens to be right. It has to run a second race with the right pair (or time solo runs in the same lane), start the
cylinders together through the shared gate, square each one in its lane, and keep track of which coloured band went
where. The manipulation is bimanual: the left arm loads the ramp top, the right arm recovers cylinders from the catch
pit, and they hand over through a middle cradle that both can reach.

## Real-world build: materials, and why the three really look and weigh the same

All three are 50.0 mm across and 60 mm long overall: a 56 mm body with a 2 mm ABS cap (50 mm disc, 4.1 g) glued on
each end, so no bore is visible. Each part is then sprayed with the same grey epoxy paint over body and caps, and gets
a 10 mm band of coloured vinyl tape at each end (red, green or blue). The bodies are faced to length so the three
weigh the same on a 0.1 g scale.

| Part | Body (56 mm long) | Body mass | With caps | k = I / (m r^2) |
|---|---|---|---|---|
| Thin-walled tube | C360 brass tube (8.50 g/cm3), OD 50.0 mm, ID 44.2 mm, wall 2.9 mm | 203 g | 211 g | 0.88 |
| Thick-walled tube | 6061 aluminium tube (2.70 g/cm3), OD 50.0 mm, bore 28.1 mm | 203 g | 211 g | 0.65 |
| Solid rod | G10 glass-epoxy rod (1.85 g/cm3), 50.0 mm diameter | 203 g | 212 g | 0.50 |

- k for a tube body is (1 + (r_in / r_out)^2) / 2; the caps (k = 0.5, 8 g together) are included in the last column.
- Same outer size and finish: one paint, the same caps and tape, so they look the same in every camera.
- Same mass to within 1 g, which neither a hand nor a scale separates.
- None of the three is magnetic (brass, aluminium, glass-epoxy), so a magnet does not separate them either. Tapping
  would sound different on the real parts; the robot has no microphone, and in a real trial no person taps them.
- The simulation gives all three the same mass (211 g) and the same collision shape (a 50 mm cylinder whose two tape
  bands are its rims), and sets each inertia tensor directly from the table above.

## Design

Positions in cm from the midpoint between the two arm bases, on the table top (x forward, y to the robot's left,
z up). The arm bases are at y +-30.5.

| Object | Where | Size, material, mass | In the simulation |
|---|---|---|---|
| Track base board | x 23.5 to 46.5, y -20 to +26.2 | 23 cm wide, 1.5 cm thick plywood; low edge rails (1.2 cm, PTFE-faced) along both sides | one 20 kg rigid assembly resting on the table (clamped in the real build) |
| Ramp | foot at y +2, top at y +25 | plank at 15 degrees, top surface 7.7 cm above the table; end board at the top | part of the track |
| Lanes | centred at x 30 and x 40 | painted, 10 cm apart, a 4 mm white line between them; no walls, so a cylinder rolls freely | part of the track |
| Start gate | across both lanes at y +16 | orange bar 23 x 0.6 x 2.2 cm, PTFE-faced; opened, it drops 3 cm into a slot below the surface | part of the track (sliding joint) |
| Gate button | (18, +30) | dark housing 7 x 7 x 3 cm, orange cap 3.6 cm square, 8 mm travel; a press of 4 mm or more opens a closed gate or closes an open one | 5 kg assembly |
| Finish line | y -16 | black stripe 6 mm wide across the board, flush | part of the track |
| Catch pit | y -29 to -20 | 3 mm blue foam floor 12 mm below the board, open across both lanes; far wall 5 cm tall, faced with 6 mm foam | part of the track |
| Cradle rack | x 15 to 23, y -18 to +18 | dark 4 mm base; four cradles of two rails (7 cm long along x, 8 mm tall, 3.8 cm apart) at y +12, +2, -8, -18 | 5 kg assembly |
| Cylinders | red at (19, +12), green (19, +2), blue (19, -8), axes along x; the cradle at y -18 is empty | see the materials table; 211 g each | loose |
| Red bin | centred (18, -30) | inside 10 x 10 cm, walls 3 cm tall, 4 mm thick | 20 kg assembly (clamped in the real build) |

- **Reach.** The left arm reaches the ramp top, the button and the cradles at y +12, +2 and -8; the right arm reaches
  the catch pit, the bin and the cradles at y +2, -8 and -18. The cradle at y +2 is the hand-over spot.
- **A timed run.** Cylinders resting behind the closed gate (centre within 2 cm of a lane's centre line, axis within
  3 degrees of square to the lane, not moving, no gripper within 6 cm) are the starters when the button opens the
  gate. Each one's time runs from that moment to its centre crossing the finish line, interpolated between control
  steps, within 4 s. One starter is a solo run, two (one per lane) are a race.
- **Times.** From rest behind the gate, the drop to the foot of the ramp is 4.8 cm and the finish line is 18 cm past
  it. The run-to-run noise and the gaps between the three kinds are measured in `media/checks.md`.
- **Hidden variable.** Which band colour is which kind is a random permutation drawn from the trial seed (all six
  orders occur). The three bodies keep fixed inertias set at spawn; each reset deals them to the red, green and blue
  cradles by the seed and shows the matching tape colour. Nothing about their mass properties changes at runtime.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

| Condition | Score |
|---|---|
| A cylinder fell off the table at any point, or at the end any cylinder other than the thin-walled tube is in the red bin (the wrong part rejected, the thin tube passed on) | 0 |
| The timed runs do not show the thin-walled tube slower than both other cylinders (including doing nothing, and any number of races that leave one comparison open) | 0 |
| The runs show the thin-walled tube slower than both others, and no other cylinder is in the bin | 0.5 |
| That, and the thin-walled tube lies alone in the bin, released, all three cylinders at rest (success, held 1 s) | 1.0 |

- **Timed run.** When the button opens the closed gate, the cylinders resting behind it start: centre within 2 cm
  of a lane's centre line and within 8 cm uphill of the gate, axis within 3 degrees of square to the lane, speed
  under 5 cm/s, and no gripper's tool centre within 6 cm. Two starters must be in different lanes. Each starter's
  time is from the gate opening to its centre crossing the finish line (interpolated between control steps), and
  counts only within 4 s.
- **What a run shows.** In a race, A is slower than B when A crosses more than 20 ms after B. In solo runs, A is
  slower than B when A's best solo time is more than 20 ms longer than B's. Chains count: A slower than B and B
  slower than C shows A slower than C. The 20 ms threshold is set from the run-to-run noise of identical cylinders
  (`media/checks.md`: largest gap between two identical cylinders 8.4 ms in 30 races, sd 3.6 ms; smallest gap between two kinds 32.9 ms).
- **In the bin.** The cylinder's centre is inside the bin's inner walls and less than 6 cm above the table, and no
  gripper's tool centre is within 6 cm of it.

In a real trial, a phone filming the finish line at 240 frames per second times the races (4 ms per frame); the bin
and the off-table rule are judged from the top-camera video.

Unintended strategies and what stops them:
- Binning a cylinder without racing (a 1 in 3 guess): 0, since the runs do not show it.
- One race and binning the loser: 0. One race compares two of the three, so it never shows one slower than both.
- Pushing cylinders down the ramp by hand, or releasing them one after the other: the gate opening is the only
  start, and a cylinder still held does not start.
- Putting all three in the bin, or the wrong one with the right one: 0.
- Throwing a cylinder: 0 if it leaves the table.

## Scripted demo

```bash
python demo.py task.py demo_out --script script.txt --seeds 0          # in the yam_arena environment, one RTX GPU
```

`demo.py` resets the scene with each seed (which draws the hidden kinds), plays the generator `script()` from
`script.txt` at 20 Hz and writes the three camera videos, `setup.png` and `result.json` (score, steps, and the
scorer's record of the runs) per seed. The policy reads only what the cameras show: the cylinders' poses (to grasp
them, to square them, and to see which one crosses the finish line second) and its own joint angles.

1. **Left arm**: red into the near lane, green into the far lane, each released 4 mm uphill of the closed gate,
   squared to the lane (the grip is turned to cancel any twist the cylinder took in the hand). If one catches on an
   opening finger and stays uphill, the closed fingertips push it gently down to the gate.
2. **Left arm** presses the button: the gate drops and the two race. It watches which one crosses the line second
   (the loser), then presses again to close the gate.
3. **Right arm** picks the loser out of the catch pit by its end caps and sets it in the middle cradle; **left arm**
   takes it from there into the near lane and puts blue into the far lane.
4. Second race. The loser of race 2 is slower than both others: either it lost both races, or it is blue, which lost
   to race 1's loser, itself slower than race 1's winner.
5. **Right arm** picks that cylinder out of the pit and puts it in the red bin, then both arms stay clear.

If a cylinder comes to rest more than 2 degrees or 12 mm off at the gate, the demo places it again; if a race
is not clear (finish times, read off the frames, less than 25 ms apart), it runs that race again, up to three
times; without two clear races it bins nothing.

Seed 0 (red solid, green thin-walled, blue thick-walled): race 1 red 702.6 ms, green 785.6 ms; race 2 blue
736.2 ms, green 787.0 ms; green binned; `task_score` 1.0 at step 2,565 (128 s). The other five orders of the three
kinds are in `media/checks.md`.

## Checks

See `media/checks.md`.

## Fidelity

Rolling, contacts and the catch pit are PhysX rigid-body physics (240 Hz). The cylinders are one 50 mm cylinder
collider each, running on their two tape bands; each one's mass (211 g) and inertia tensor are set directly from the
materials table, and each reset deals the three bodies to the colours by the seed (the tape bands are visual overlays). PhysX has no rolling resistance,
so the simulated times are those of an ideal rolling cylinder; real cylinders on plywood lose a few percent, the same
for all three. The edge rails and the gate are faced with a low-friction (PTFE, 0.05) strip; the rest is wood
(friction 0.9) and the cylinders' paint (1.0). The start gate is a bar on a stiff sliding joint that the button
toggles in code (in the real build, a solenoid or a hand lever under the ramp). The track (20 kg), rack, button and
bin are heavy rigid assemblies resting on the table; in the real build they are clamped. The lanes are painted, not
walled: a cylinder squared within 3 degrees drifts at most 2 cm sideways over the run and never touches the edge
rails, and a skewed one that rubbed a wall would lose time unfairly, which is why a skewed start does not count.
