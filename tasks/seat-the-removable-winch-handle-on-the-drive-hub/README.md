# Seat the removable winch handle on the drive hub

Put the handle on the winch.

```bash
TASK_DEMO=demo   # 1.0: seat the handle on the drive hub and let go
TASK_DEMO=short  # 0.5: partial engagement on the shaft
TASK_DEMO=idle   # 0.0: leave the handle on its outline
TASK_DEMO=naive  # 0.0: arm stays aside
```

## Setup
A compact Dutton-Lainson DLB1500A-class trailer winch (zinc TUFFPLATE body, dark 2 in strap on the drum, side plates, gear on the brake side, fairlead and hook, slotted base) sits in the middle of the table, unpowered, with its square drive pointing toward the left arm. The official 9-3/8 in removable handle, with a black molded grip, starts on a marked outline beside the hub, parallel to the shaft but not on its axis. The handle starts beside the shaft, not already on it. All props are rigid and start on fixed marks.

## Scoring
As seen in the overhead video, the handle stays on the drive shaft. At the end, after 1 s at rest and with the gripper 3 cm clear of the handle: 1.0 once the hub is on the shaft to the pin depth, aligned, and has been let go; 0.5 if the hub has at least 10 mm of engagement but is short of the pin depth, or if it is fully seated but has not been let go or is not yet still; 0 if the handle is still on its outline, misses the shaft, or is only pressed along a pre-aligned start. Possible scores: 0, 0.5, 1.0.

## Design notes
- **Why it matters.** Boat-trailer and utility winches (Dutton-Lainson, Fulton and similar) use a removable hand crank that has to be seated on a square drive before the drum can be wound or paid out. A handle left off, or only hooked on the shaft, will slip when a load comes on. Sourced from the Dutton-Lainson 6454 handle for the DLB1500A (drawing DD_DLB1500A, TUFFPLATE sku 14964).
- **Where the cue is.** The zinc winch, dark drum, shaft stub and the black-grip handle on its outline sit in the table middle in the top camera.
- **Distinct from.** #2827 (closed) counter-holds a trailer-winch crank already in the drum and pays out strap. #2889 (open) retracts a JOST landing leg and stows the crank. Those are wind, pawl or stow tasks, not seating a square drive onto an empty hub.
- **Honest limits / flagged trade-offs.** Tight square-on-square is not representable at a 4 mm PhysX contactOffset, so the shaft is scored geometrically on a looser 14 mm peg in a 40 mm hub with a 12 mm axial seat window. The colliding envelope is compact versus the real 7-7/32 in × 5-1/2 in unit so the grip stays in reach. The spring pin is scored as a depth, not a simulated detent. The drum is not wound. The trial is unpowered.

## Reset
Handle starts beside the shaft on its marked outline, not on it. The drum is not wound.

## Real trials
A small trailer winch and its removable handle, well under the about $100 bench-trial cost. The trial is unpowered; the drum is not wound.

## Fidelity
See task.json. The loose 40 mm hub and the 12 mm axial seat window, and the compact envelope versus the real unit, are flagged REACH / PhysX trade-offs, not hidden shrinks.

## Scores
- Scripted demo (yam_arena, 320 × 176, seed 0, Brev L40S, v1g): 1.0, success at step 265 of 700.
- TASK_DEMO=short: 0.5 at 320 steps (partly on the shaft, stayed on after release).
- TASK_DEMO=idle: 0.0 at 260 steps (handle left on the outline).
- TASK_DEMO=naive: 0.0 at 129 steps (arm stays aside).
- Dream-1 (dreamlabs/dream-1-beta-flash, seed 1, 320 × 176, 22 Nov 2026 on a Brev L40S): 0.0 in 700 steps, not refused (the left arm nudged the handle on the bench, never onto the shaft; 28 requests, 28 attempts, term=max_steps).
