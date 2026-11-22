# Make the open manhole safe to walk across

Make the open manhole safe to walk across.

```bash
TASK_DEMO=demo   # 1.0: lift the iron cover by its eye, carry it over the ring, lower it in, let go
TASK_DEMO=held   # 0.5: seat the iron cover but never let go
TASK_DEMO=naive  # 0.0: carry the small plastic lid over the opening and let go; it drops down the shaft
TASK_DEMO=short  # 0.0: release the iron cover 50 mm short of the ring
TASK_DEMO=undo   # 0.0: seat and release the iron cover, then lift it back out
TASK_DEMO=idle   # 0.0: do nothing
```

## Setup
A 220 mm square concrete-look slab, 20 mm thick, lies in the table middle. It has a round recessed ring: well 120 mm across with an 8 mm dark iron ledge, leaving a 104 mm opening whose bottom is the table (dark). The seat is 10 mm below the slab top. Two loose rigid lids lie flat on the table beside it. The iron cover is 108 mm across and 10 mm thick (dark grey, 0.35 kg) with a 40 x 30 x 16 mm yellow lifting eye on top. It starts at 155 mm from the ring centre on the +x side. The plastic lid is 88 mm across and 8 mm thick (blue, 0.12 kg) with a white 40 x 30 x 16 mm eye. It starts beside it, 135 mm along -y. A red stripe marks the slab edge. All props are rigid and start on fixed marks.

## Scoring
As seen in the overhead video, the iron cover sits in the ring, flush with the slab. At the end, after 1 s at rest and with the gripper clear of the cover: 1.0 once the iron cover is in the ring (within 10 mm of its centre, within 8 mm of seat height, tilt under 10 degrees) and has been let go; 0.5 if it is in the ring but still held or not yet still; 0 if it is not in the ring (on the slab or table, or lifted out again), if the plastic lid has dropped into the opening at any time (latched, even if the iron cover is seated afterwards), and for idle. Possible scores: 0, 0.5, 1.0. The score is recomputed from the final state, so a cover lifted out again scores 0.

## Design notes
- **Why it matters.** An open manhole is a hole in a walking surface. OSHA 29 CFR 1926.501(b)(4) requires holes to be covered or guarded and 1926.502(i) requires a cover to carry at least twice the load on it and to be secured against displacement. The instruction does not name the lid. A lid that is too small does not span the opening and drops down the shaft, where it can strike a worker below and cannot be fetched without confined-space entry. That is why the dropped lid latches 0.
- **The decision.** Two loose lids lie beside the open ring. Only the 108 mm iron cover is larger than the 104 mm opening and rests on the 8 mm ledge. The 88 mm plastic lid is smaller than the opening.
- **Genuine placement.** The cover has to be lifted by its eye, carried, centred over the ring within 10 mm and lowered 10 mm into the recess, then released and left still for 1 s. Holding it at the seat, hovering over the ring, leaving it on the slab and lifting it out again all score below 1.0.
- **Distinct from.** Upstream #1780 (floor-opening cover, 160 mm tabletop opening with pins) and #739 (inspection cover on a locating dowel) are closed. This task adds a loose-lid choice with a latched failure and a flush recessed ring, not pins or a dowel.
- **Honest limits.** Bench model, about 1:5.6 of a 610 mm manhole, so it tests the placement, not the load or the lifting hook. The ring has about 6 mm clearance a side (PhysX contactOffset is 4 mm). The lifting eye is a raised tab so a straight-down pinch can lift the lid.

## Reset
Put both lids back flat on their start marks. Take the plastic lid out of the opening if it dropped in. The slab does not move.

## Real trials
A 220 mm board with a round 104 mm opening and a recessed ledge ring, a 108 mm disc with a handle, and a smaller 88 mm disc. No lifting equipment needed.

## Fidelity
See task.json.

## Scores
See task.json.
