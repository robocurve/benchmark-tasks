# Clamp a cable straight in a fixture

A 50 cm black 6 mm cable leaves a fixed fixture block on the left through a grommet (held at the exit, free to swing
sideways) and lies loose on the table, swung 17 degrees off the fixture line. The line runs straight from the cable
exit (towards the robot's right) and is marked by two pairs of blue tick marks; on it, 24 cm from the exit, sits a
fixed grey U-seat (two blocks with a 34 mm gap). The robot must pull the cable onto the line, straight and taut
through the U, seat the free orange clamp block (30 x 30 x 20 mm) in the U on top of the cable, and let go of both.
Test, crimp and potting fixtures need the cable held straight while it is clamped: clamping a slack or wavy cable is
exactly the workmanship error these fixtures exist to prevent, so a block dropped on a slack cable scores 0. It is
naturally bimanual: one arm keeps the cable taut on the line while the other places the clamp block.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
    --instruction "Pull the black cable straight and taut along the fixture line marked by the blue ticks, so it runs through the grey U-seat, then place the orange clamp block in the U on top of the cable. Let go of both the block and the cable."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1600
```

Closed-loop (reads the live cable state), both arms: the left arm takes the cable 19 cm from the exit and the right
arm takes the plug; together they lift the cable 6 cm, carry it nearly taut over the fixture line (clear of the U
walls) and lower it, so it drops into the U. The left arm puts its grasp point exactly 19 cm (+4 mm) along the line,
which pulls the exit-side part straight; the right arm then pulls along the line until the measured deviation is
under 2 mm. The left arm lets go, picks the clamp block, lowers it into the U onto the cable and lets go; the right
arm, which kept the cable taut the whole time, then lets go of the plug and both move clear. Final deviation 1.2 mm;
the cable stays straight after release (it does not spring back). Success holds 1 s and ends the episode at step
477 (~24 s); task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

Shortcut variants (test only, `CLAMP_CHEAT=<name> python cabledemo.py task.py out`), all score exactly 0:
`none` (do nothing), `slack` (lay the cable through the U with 3 cm of slack, never pull it taut, then seat the
block on it and let go: block seated on the cable but deviation 52 mm), `loose` (as slack with 3 cm more slack near
the exit: the cable bows out of the U and the block lands on the table), `noblock` (pull the cable straight, 1.4 mm,
and let go without placing the block), `hold` (pull straight and seat the block, but keep gripping the block:
deviation 1.2 mm, would be 1.0 if released).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** unless both grippers have let go: each gripper centre at least 3 cm from every part of the cable, its plug
  and the clamp block.
- **0** unless the clamp block is seated: its centre lies inside the U (between the walls, within their 4 cm length),
  the cable passes under its footprint, and its bottom is at most 6 mm above the top of the cable under it.
- **0** if the cable's largest sideways deviation from the fixture line, from the cable exit to the far tick mark
  (39 cm along), exceeds 15 mm, or if the cable does not reach past the far mark.
- Otherwise **0.5 + 0.5 x straightness**: deviation 5 mm or less = 1.0, falling linearly to 0.5 at 15 mm.
- `success()` additionally requires the block and the cable to be at rest. `distance()`: the deviation once the
  block is seated, else 1 m.

## Runtime and seeds

- Run limit 1,600 steps (80 s at 20 Hz). One trial takes ~5-10 min wall time on one GPU including the ~1-2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight at the same
  angle), so every trial starts identically.
