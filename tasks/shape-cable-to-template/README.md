# Shape a cable to a printed S-curve template

A paper template lies flat on the table in front of the robot, with a 20 mm wide blue band printed on it in a
wavy S (a quarter turn, a half turn right and a half turn left, all of 6 cm radius, then a 3 cm straight run to an
orange 30 mm end circle; 50 cm along its centreline). A black 6 mm cable, exactly 50 cm long, leaves a small
device box at the start of the band and lies straight across the table towards the robot's right. The robot must
lay the cable along the band, starting at the box, so that it lies inside the band along its whole length with
its free end on the orange circle, and let go. Unlike a raceway or channel, nothing holds the cable in shape: there
are no walls, clips or tape, so every few centimetres must be put down in the right place and left where friction
alone keeps it, without disturbing what is already laid. Because the cable is exactly as long as the band, any
shortcut through a turn leaves the end short of the circle. The natural solution is bimanual: one arm keeps the
loose cable clear (and pins the part already laid while the end is moved) while the other lays the next few centimetres.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Lay the black cable along the blue S-shaped band on the paper template, starting at the box, so that it lies inside the band along its whole length with its free end on the orange circle. Then let go of the cable."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed-loop (reads the live cable state). The right arm first takes the cable 3 cm from its free end and slides
the end off to the right of the template, so the loose cable lies flat and slack. The left arm then lays the first
21.5 cm of the band from the box: for band point u (every 3 cm) it grasps the cable at arclength u from the box
(the cable is exactly as long as the band, so feeding no extra length keeps the end on target), slides it just
above the table to the band point, sets it down and lets go; the grip yaw is chosen so the fingers close square
across the cable (a skewed finger lands on the neighbouring laid cable and drags it). A closed-loop touch-up
re-places the worst point of any bad section. Next, the left arm pins the last laid point with its closed fingertips
while the right arm moves the free end to just beyond the band's last turn; the left arm then moves clear and the
right arm lays the rest the same way, ending with the end on the orange circle, plus a touch-up. Both arms end
clear of the cable. Success (all 8 sections, released, at rest for 1 s) ends the episode at step 1477 (~74 s);
task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

Shortcuts (test only, `SHAPE_CHEAT=<name> python cabledemo.py task.py out --no-record`), each must score exactly 0:
- `nothing`: no motion; the cable stays straight. Score 0.0 (100 steps).
- `drag`: the right arm takes the free end and drags it low across the template to the end circle and lets go,
  leaving the cable as a rough chord from the box. Score 0.0 (240 steps; no section is laid).
- `half`: the demo's first half (4 sections laid perfectly), then stops with the free end lying off the template.
  Score 0.0 (728 steps): the end is not on the circle.
- `hold`: the full demo, then the right arm re-grips the cable 2 cm from its end and keeps holding it (all 8
  sections in the band). Score 0.0 (1582 steps): the cable is not released.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- The band centreline (50.1 cm) is split into 8 equal sections of 6.27 cm from the box, each checked at 8
  evenly spaced points (tick marks on a real template).
- A check point is covered if some part of the cable lying flat on the sheet (centre at most 4 mm above its resting
  height) is within 1 cm of it, i.e. inside the 20 mm band there.
- A section is laid if all 8 of its points are covered and the matching stretch of cable (the same 6.27 cm measured
  along the cable from the box, plus 1 cm before it) lies entirely inside the band. Section 8 also needs the free
  end on the orange circle.
- **Score 0** unless the cable is released (both gripper centres at least 3 cm from every part of it) and its free
  end lies on the circle (end tip within 15 mm of the centre, resting). Otherwise score = number of consecutive laid
  sections counted from the box / 8 (a section after the first bad one does not count).
- Success (1.0) = all 8 sections laid, released, cable at rest, held for 1 s.
- Doing nothing, dragging the end across, leaving the end off the circle, or holding the cable in shape all score 0.
  A straight cable from the box covers at most the first section (at about 70 degrees), and its end then lies far from
  the circle; a straight cable ending on the circle covers nothing. The end-circle gate rules out partial credit
  from a cable dragged into a rough curve.
- `distance()`: band length (m) from the first bad section on.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). One trial takes ~5–25 min wall time on one GPU including the ~1–2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight), so every trial
  starts identically.
