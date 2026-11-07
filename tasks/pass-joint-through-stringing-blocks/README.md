# Pass a line's rigid joint past a stringing block too narrow for it, keeping tension

A table-top model of a stringing run. A yellow 6 mm line comes out of an orange tensioner (a spring that pulls it
back) and runs through three open-top stringing blocks standing in a row. Spliced into the line just right of the
middle block is a grey rigid joint, 2 cm thick. The two outer blocks have 3 cm throats; the middle block's throat is
only 1.2 cm, so the joint cannot pass through it. Left of the blocks the line's end lies on the table with a black
toggle bar across its end, and a blue cleat stands beyond the left block: an open-top pocket with a slot in its near
wall, so a toggle dropped in holds the line.

The robot must get the joint to the left of the narrow block and make the line fast in the cleat with a toggle, with
the line running in all three blocks at the end. Pulling the line drives the joint into the narrow block, where
it jams. The way through is the one used on a stringing run: lift the line out of the narrow block, carry the joint
over it while the line is drawn in from the other side, make the line fast, then reseat the line in the block. The
tensioner pulls the whole time, so the line must not be let run back.

## Objects (fixed unless stated)

| object | type, size, material | position (cm from the midpoint between the arm bases; x forward, y left) |
|---|---|---|
| three block stands | grey printed PLA boxes, 3 cm along the line, 5 cm tall; 5.4 cm wide (outer), 3.6 cm (middle) | x 32; y -26, -10, +5 |
| roller pads | black PLA (smooth, not turning), fill each throat from 5.0 to 5.65 cm up | in each block |
| block walls | grey PLA, 1.2 cm thick, 3 cm long; tops 8.5 cm up (outer), 7.5 cm (middle) | either side of each throat |
| throats | open-top slots: 3 cm wide (outer blocks), 1.2 cm wide (middle block); line axis 6 cm up | x 32 |
| tensioner | orange PLA tunnel, 25 cm long, 1.4 cm square bore at 6 cm up | x 32, y -59.5 to -34.5 |
| tensioner spring | line end on a spring carriage, 0.25 N on its home stop, about 0.40 N with 15 cm drawn out, 20 cm travel | inside the tunnel |
| line | yellow 6 mm braided cord, 70 cm, 50 g | from the tensioner (start at y -57.5) straight through all three throats |
| joint | grey rigid sleeve 3.5 x 2 x 2 cm, 10 g, glued on the line | centred on the line at y -18.5 |
| toggle | black bar 5 x 1 x 2 cm, 15 g (steel-cored), glued across the line 1 cm from its end | 69 cm from the line's tensioner end; the line's last 5 cm past the left block droop, toggle about x 32, y +9, 1.6 cm up |
| cleat | blue PLA open-top pocket: inside 3.5 cm along the line x 8.8 cm across, 6 mm walls, floor 4.9 cm up, rim 7.5 cm up; its near wall has a 2.4 cm slot down to 5.65 cm for the line (the 5 cm bar cannot pass it) | pocket centre x 25.8, y +23.6; near wall's inner face 16.5 cm from the left block's exit, on a line turned 20 deg from the run towards the robot |

## Task

Instruction: "Get the grey joint on the yellow line to the left of the narrow middle block and make the line fast
with a toggle in the blue cleat, leaving the line running in all three blocks. Then let go."

The instruction states the goal only. How to get past the narrow block, and that the line must keep its tension and
stay in the outer blocks, are part of what is tested and are checked in the scoring.

## Scoring (`score()`, at the end; `success()` must hold 1 s)

Score 0 if, at any time: the line ran back towards the tensioner by more than 5 cm from the most it had been drawn
out; the joint was butted end-on against the narrow block (overlapping it across the line, below its wall tops, its
end within 4 mm of a block face); or the line left either outer block's throat (above the wall tops or outside the
throat). Also 0 if at the end a gripper centre is within 3 cm of the line, joint or toggle, if the toggle is not in the
cleat (bar centre inside the pocket, below its rim, and the line down in the near wall's slot), or if the joint is not past the narrow block (wholly between the narrow block and the left block,
at least 5 mm clear of the narrow block, its centre within 2 cm of the run).

| end state | score |
|---|---|
| joint past, toggle in the cleat, released, but the line not down in the narrow throat (e.g. lying on its wall top or beside it) | 0.5 |
| as 0.5, and the line down in all three throats (within 1 mm of the throat width, at most 6 mm above the axis height, over each block's length) | 1.0 |

## Why two arms (offline IK, gripper straight down, error in mm; 9000 = no solution)

| point | left arm | right arm |
|---|---|---|
| narrow block (32, -10), 6.5 cm up | none | reached |
| narrow block, 10.5 cm up (joint carried over) | none | reached |
| joint at its start (32, -18.5), 6.2 and 10.5 cm up | none | 0.0 |
| joint's carry end (32, -2.5), 10.5 cm up | - | 0.0 |
| joint held near the left block (32, +0.3 and -2), 7.2 cm up | 0.0 | 0.0 |
| toggle near its start (32, +6.5), 4 cm up | 0.0 | 0.0 |
| pocket (25.2, +24.1), 6.5 and 9.5 cm up | 0.0 | none |
| over the pocket (26.5, +22; 27, +21), 6.5-10 cm up | 0.0 | none |
| line 3 cm short of the cleat (27, +16) | - | none |

Only the right arm reaches the joint and the narrow block; only the left arm reaches the cleat. Both are needed at
the same time: if the right arm carries the joint over while nothing holds the toggle side, or the left arm draws the
line in while nothing lifts the joint, the tensioner pulls the joint back against the narrow block. The left arm
can reach the line between the narrow and left blocks (32, -8.5: 2.5 mm error; 32, -2: 0.0), but a grip there is
between the two blocks, so it cannot draw the joint past without lifting the line out of the left block or letting it run back.
Final check (left arm, 32 cm forward): y -8.5 reached only at 6.5 cm up (9 cm: none); y -6 up to 10.5 cm; y -2 and +2 up to 12 cm. A single grip at y -2, 12 cm up could tent the line high enough at the joint (about 9 cm) but moves to the left block's entry as the joint passes, where lifting the line breaks the outer-throat rule; any regrasp lets the line run back, and the toggle still needs a free hand.

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1000
```

Closed loop on the live joint and toggle. Right arm pinches the joint across (wrist camera towards the narrow block),
left arm pinches the toggle by its ends. The right arm lifts the joint 3.5 cm, which brings the line up out of
the narrow throat, and carries it over the block to y -2.5 cm while the left arm draws the line in towards the cleat.
The right arm lowers the joint (the line drops back into the narrow throat) and holds it; the left arm lifts the
toggle just over the cleat's rim, holds it over the pocket and lets it drop in, the line falling into the slot; the
right arm lets go of the joint. The line's pull holds the toggle in the pocket and the joint between the blocks.

Shortcut and partial variants (test only, `STRING_CHEAT=<name>`), all evidence in `scratch/sc_*.json`:

| variant | what it does | why it scores what it does | score |
|---|---|---|---|
| `none` | no motion | joint not past, nothing made fast | 0 |
| `pull` | left arm alone draws the line towards the cleat by the toggle | joint drawn end-on into the narrow block (butted, latched) | 0 |
| `onearm` | right arm alone carries the joint over, sets it down and lets go; the left arm only holds the toggle where it lies | the line runs back and the joint is pulled back into the narrow block (both latched) | 0 |
| `letgo` | the joint is carried over, then the toggle is let go short of the cleat, then the joint | the line runs back and the joint butts on the narrow block (latched) | 0 |
| `hold` | the full method, but the right arm never lets go of the joint | not released | 0 |
| `tier05` | the full method, but the joint is carried and set down 1.5 cm off the run | the line comes down on the narrow block's wall top, not into its throat | 0.5 |

## How this differs

- Nearest board tasks: "Tension a messenger wire and hang its ring at the mark" (move an end block into a notch) and
  "Clamp a cable straight in a fixture" (pull a cable taut through U-seats). Neither has a part on the line that
  cannot pass a fixture, and in neither must the line come out of one guide and go back in while held under load.
- Our nearest tasks: open-snatch-block-and-seat-rope (reeve a slack rope round a sheave under a sprung side plate),
  pull-cable-to-length-mark (draw a cable out to a mark), tend-lock-lines-as-boat-rises (take in lines as a boat
  rises and lay a toggle in a cleat). Here the cleat only holds the result; the act is passing a rigid joint past a
  block too narrow for it on a line that is pulled back all the time, by taking the line out of that block only and
  putting it back after.

## Context and sources

- 29 CFR 1926.964(b)(11) (eCFR, www.ecfr.gov, Title 29 Part 1926 section 1926.964): "While a power-driven device is
  pulling the conductor or pulling line and the conductor or pulling line is in motion, the employer shall ensure
  that employees are not directly under overhead operations or on the crossarm, except as necessary for the employees
  to guide the stringing sock or board over or through the stringing sheave." Hardware on the line is guided over or
  through the block.
- Same section, note to (b)(10): "Examples of unsafe conditions include: employees in locations prohibited by
  paragraph (b)(11) of this section, conductor and pulling line hang-ups, and slipping of the conductor grip."
- Pengo-Miller swivels and connectors sheet (cdn.shopify.com/s/files/1/0645/0534/8345/files/Pengo.pdf): "Short,
  bullet shape accommodates passage though blocks." (typo in original) and "Swivel selection should be based on
  pulling line diameter, grip size, opening dimensions, pin sizes, working load, and sheave groove diameter." A part
  on the line has to fit the block it passes; here one does not.
- O*NET 49-9051.00 Electrical Power-Line Installers and Repairers, task: "String wire conductors and cables between
  poles, towers, trenches, pylons, and buildings, setting lines in place and using winches to adjust tension."

Paper fields: skill tags routing/seating, extraction, constraint-keeping, sequencing; industry: power-line
construction; two-arm need: reach-forced and simultaneous.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (line simulation, scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1000 \
    --instruction "Get the grey joint on the yellow line past the narrow middle block, to rest between it and the left block, and make the line fast with a toggle in the blue cleat, leaving the line running in all three blocks. Then let go."
```

Runtime: the demo succeeds at step 441 (22 s of sim time); a 1,000-step trial takes a few minutes on one GPU. Seeds: the
layout is fixed (`object_jitter=0`); the line spawns straight and its loose end settles onto the table the same way
every reset.
