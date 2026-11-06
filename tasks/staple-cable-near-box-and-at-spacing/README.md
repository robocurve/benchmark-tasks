# Staple a cable near the box and at a maximum spacing

A white 6 mm cable leaves a fixed blue outlet box on the left (held at the box face, free to swing sideways) and lies
loose on the table, swung 15 degrees off a dark-stained pine stud strip that runs straight from the box towards the robot's right.
A black end mark crosses the stud 55 cm from the box. Four free orange staple blocks (36 x 15 x 20 mm, with a 12 mm
slot underneath) stand on the table, two near each arm. The robot must lay the cable straight along the stud and fix
it with the staples, each set over the cable so the cable runs through its slot, following a securing rule: one
staple within 6 cm of the box, one within 6 cm of the end mark, and no more than 20 cm between neighbouring staples.
There are no marks for the staples: the robot has to judge the distances itself. Four staples are the fewest that
can satisfy the rule (three can reach at most 6 + 2 x 20 = 46 cm, short of 49 cm), so there is little slack.

This is a scaled-down version of how electricians secure nonmetallic-sheathed (NM) cable to studs: NEC 334.30 (as
adopted in Baltimore City's 2024 building code) requires staples within 12 in. (300 mm) of every box and at intervals
not exceeding 4.5 ft (1.4 m). The 6 cm and 20 cm here are tabletop stand-ins, not a proportional scaling. The task
needs both arms: laying the cable straight takes one arm near its middle and one at its end, and neither arm can
reach the whole 55 cm run (the left arm places the two staples nearest the box, the right arm the two far ones).

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
    --instruction "Lay the white cable straight along the wooden stud and fix it with the orange staple blocks, each set over the cable so the cable runs through its slot: one staple within 6 cm of the box, one within 6 cm of the black end mark, and no more than 20 cm between neighbouring staples. Then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1600
```

Closed loop (reads the live cable and staple states), both arms. The left arm takes the cable ~24 cm from the box,
the right arm ~4 cm from its end; together they lift it, carry it over the stud and lay it down, and the right arm
pulls along the stud until the cable is straight (measured sideways offset under 4 mm), then keeps holding the end.
The left arm lets go and sets staples at 3.5 cm and 19.5 cm from the box, each centred on where the cable actually
is (re-measured just before set-down). The right arm then lets go of the cable end and sets staples at 35.5 cm and
51.5 cm (gaps 16 cm; the last one 3.5 cm before the end mark). Staples are held by their top bar with the fingers
closing along the cable, so the wrist camera looks across the stud at the set-down point. Both arms park clear.
Success holds 1 s and ends the episode at step 854 (~43 s); task_score 1.0. `demo/` holds its videos, `setup.png`
and `result.json`.

Shortcut variants (test only, `STAPLE_CHEAT=<name> python cabledemo.py task.py out`):
`none` (do nothing: cable 15 cm off the stud) **0**; `where` (staple the cable where it lies, without laying it
on the stud: staples land on the table or tilt on the cable) **0**; `far` (lay the cable, four staples evenly at 9,
24, 39 and 54 cm, all straddling, but the first is 9 cm from the box) **0**; `beside` (lay the cable, staples at the
right distances but 4 cm to the side, on the table next to the stud) **0**; `pile` (lay the cable, two staples on it
within 6 cm of the box and nothing further) **0**; `hold` (all four placed, but the right arm keeps holding the last
staple) **0**. `two` (lay the cable, only the first two staples, at 3.5 and 19.5 cm) scores the honest partial
**0.25**: the rules hold for the first 19.5 cm of the run.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** unless both grippers have let go: each gripper TCP at least 3.5 cm from every part of the cable and from
  every staple block.
- **0** if the cable centreline (sampled every 2 mm) is more than 2 cm sideways from the stud centre line anywhere
  between the box face and the end mark. (Real trial: also 0 if the box or stud was moved.)
- A staple **counts** only if it straddles the cable: its bottom is within 3 mm of the stud top (both legs on the
  stud, not on the table or on the cable), it is tilted less than 10 degrees, its slot runs within 25 degrees of the
  stud line, and the cable centreline passes through its slot. Its position is its centre, measured along the stud
  from the box face.
- Chain: start at the counted staple farthest from the box that is still within 6 cm of it (none: score 0), then
  step to the next counted staple along the stud while the gap is at most 20 cm.
- **1.0** if the chain reaches a staple within 6 cm of the end mark (49-61 cm from the box). Otherwise **0.75 / 0.5 /
  0.25** if the last staple reached is at least 45 / 30 / 15 cm from the box, and **0** below that (a staple only at
  the box earns nothing).
- `success()` additionally requires the staples and the cable to be at rest. `distance()`: cable offset beyond 2 cm
  plus how far the chain falls short of the end mark. `progress()` reports stage values (cable on stud, staples
  straddling, box staple, reach fraction, end staple, released).

## Runtime and seeds

One trial is 1,600 steps at 20 Hz (80 s of robot time; ~5-8 min wall time on an L40S). The layout is fixed with
`object_jitter=0`; the cable is never jittered. `VARIATION` in `task.py` lists seeded ranges a later runner may
apply (cable yaw, staple start offsets); the closed-loop demo re-measures the cable and each staple before acting.
