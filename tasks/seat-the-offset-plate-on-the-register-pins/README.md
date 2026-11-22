# Seat the punched offset plate on the press register pins

Put the plate on the pins.

```bash
TASK_DEMO=demo   # 1.0: lower the plate onto the bed and let go
TASK_DEMO=short  # 0.5: stop short of the bed
TASK_DEMO=idle   # 0.0: do nothing
TASK_DEMO=naive  # 0.0: arm stays aside
```

## Setup
A short proof-press bed with two register pins, bearer rings and a lead-edge clamp bar sits in the middle of the table, unpowered. A punched A5-scale plate (about 150 × 110 mm) with a printed image and four red register marks already hovers 25 mm above the bed, holes over both pins, so the plate is a large, clearly visible rectangle in the table middle. A second, yellow, unpinned plate lies on a wooden board beside the bed. All props are rigid and start on fixed marks. The bed is bench-scale, not a full 40 in press.

## Scoring
As seen in the overhead video, the plate stays on both register pins. At the end, after 1 s at rest and with the gripper clear of the plate: 1.0 once the plate is flat on the bed (within 3 mm) and has been let go; 0.5 if the plate has been pushed at least 8 mm from the start but is still short of the bed, or if it is on the bed but has not been let go or is not yet still; 0 for the start pose itself (25 mm up, already on the pins) and for any path that never pushes. Possible scores: 0, 0.5, 1.0.

## Design notes
- **Why it matters.** Press operators hang a punched plate on the register pins of the plate-cylinder clamps before the clamps are tightened (PrintWiki, Plate Clamp; Printing Industries of America guidance). A plate that is not on the pins will be out of register or can be damaged as the clamps close.
- **Where the cue is.** The teal A5 plate with red register marks sits over the dark bed in the table middle; the yellow spare is beside it.
- **Distinct from.** Upstream #3470 transfers wood type from a composing stick to a galley (typography, not hanging a plate on register pins).
- **Honest limits.** The plate is a Z-slider already on both pins (PhysX contactOffset is 4 mm, so a free plate on real pin diameters is not used). The bed is an A5-scale proof-press segment, not a full press. No ink is applied and the cylinder does not turn.

## Reset
Raise the punched plate 25 mm above the bed, still on both register pins. The spare yellow plate stays on the board.

## Real trials
A small proof-press bed or a pin-register base, two plates and two pins, well under the about $100 bench-trial cost. The trial is cold and unpowered.

## Fidelity
See task.json. The bed, pins and plates are rigid bodies at real sizes and masses; the plate is a Z-slider already on both pins.

## Scores
- Scripted demo (yam_arena, 320 × 176, seed 0, Brev L40S, v1): 1.0, success at step 178 of 500.
- TASK_DEMO=short: 0.5 (stopped at 6.9 mm).
- TASK_DEMO=idle: 0.0.
- TASK_DEMO=naive: 0.0 (arm stays aside).
- Dream-1 (dreamlabs/dream-1-beta-flash, seed 1, 320 × 176, 22 Nov 2026 on a Brev L40S): 0.5 in 500 steps, not refused (the plate was pushed to 15.2 mm, short of the bed; 20 requests, 20 attempts, term=max_steps).
