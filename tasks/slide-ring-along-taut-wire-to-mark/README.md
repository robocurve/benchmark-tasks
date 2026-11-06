# Tension a messenger wire and hang its ring at the mark

A 5 mm black messenger wire runs straight across the table, 30 cm in front of the robot and 9 cm above the table. On
the right it runs into a dark grey anchor post, through a channel, to a spring tensioner inside. On the left it ends
in a black ferrule block that sits in the near notch of a grey two-notch bracket. In the near notch the wire is slack
and sags about 28 mm with the ring on it. Moving the ferrule 8 cm out to the far notch draws wire out of the post
against the spring, and the wire then sags only 4-8 mm. An orange square hanger ring with a grab plate on top hangs
on the wire, and a red tape mark on the table under the wire shows where the ring must end. The robot must leave the
wire tensioned (ferrule in the far notch), the ring hanging right above the mark, and let go of both.

The tested ability is ordering and coordination, not the notch. Drawing the wire tight drags an unheld ring with it:
in the sim, tensioning alone carried the ring about 7 cm toward the bracket, and a ring set on the mark first was
carried about 8 cm off it (score 0). So the ring has to be held while the wire is tensioned, or placed after. The
task needs both arms by reach (offline IK, gripper down, wire height): the right arm misses the far notch (y +20) by 60-80 mm
and the near notch by about 19 mm, so only the left arm can tension; the left arm misses the mark (y -13) by 25-27 mm at the
best of 25 wrist yaws (it reaches y -4 to within 0.1 mm), so only the right arm can place the ring at the mark. The left
margin is small: it is about the 25 mm width of the 0.5 band.

## How this differs from similar tasks

- Clamp a cable straight in a fixture (TK-Z56KQ2): pull a cable taut, then set a clamp block on top. The block does not ride
  the line and tensioning cannot move it, so any order works. Here the slider is threaded on the line and the pull moves it.
- Slide a carriage onto a rail until it clicks (TK-GBSVDR) and hand-guide a carriage to marks (TK-X7XSCF): rigid carriage
  on a rigid guide. Here the guide is a flexible, spring-tensioned wire that changes shape and drags the slider.
- Our peg-a-taut-row-line-at-marks and TK-4TQGWR (pin a cable at marks): pegs and bags sit beside or on the line and
  stay put; the line must be held taut while they are placed. Here the line is tensioned once and latched in a notch.
- Differentiator: tension-then-place with an ordering trap (an unheld ring is dragged ~7-8 cm off the mark), a mark
  with no stop, and a two-notch ferrule seat. Closest in spirit is TK-Z56KQ2 (tension then place an accessory): medium
  overlap on the abstract objective, distinct in the coupling.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Tension the black messenger wire by moving its end block into the far notch of the grey bracket, and leave the orange hanger ring hanging on the wire right above the red mark. Let go of both."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed-loop: the right arm pinches the ring's grab plate while the left arm pinches the ferrule. The left arm lifts
the ferrule over the near slit wall and draws it out until the measured ferrule is past the far slit wall (it lags
the fingers under the spring's pull), then lowers it into the far notch and lets go. The held ring stays put while
wire runs through it. The right arm then lets go, so the ring rides up with the now-taut wire, re-pinches it and
slides it along the wire to the mark, correcting on the measured ring centre. It opens wide, backs straight up, and
both arms move clear. Final run: sag 5 mm, ring 0.1 mm from the mark. Success holds 1 s and ends the episode at step
452 (about 23 s), task_score 1.0. Two repeats also scored 1.0 (steps 490 and 452). `demo/` holds the videos,
`setup.png` and `result.json`.

Shortcut and failure variants (test only, `RING_DEMO=<name> python cabledemo.py task.py out`) all score exactly 0:
`nothing` (wire left slack, sag 28 mm); `slack` (ring slid to the mark, 8 mm off, wire never tensioned: sag 24 mm);
`hold` (full demo, but the right arm keeps pinching the ring at the mark: not let go); `holdferrule` (the left arm
keeps pinching the seated ferrule); `holdwire` (the left arm lets go of the ferrule but keeps pinching the wire beside
the bracket); `tension` (wire tensioned,
ring never touched: tensioning drags it to y +3.4, 16 cm from the mark); `ringfirst` (ring set on the mark and
released, then the wire tensioned with the ring unheld: the ring is dragged about 8 cm off the mark). Partial-credit variant `tier05`: the full demo with the ring left 18 mm
from the mark scores 0.5.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if the ferrule is not resting in the far notch (centre between that notch's slit wall and end wall, on
  the bracket floor), if the wire's centre line anywhere between the post and the bracket is more than 10 mm below
  the straight line between them, if the ring is off the wire or tilted over 45 deg, or if either gripper is within 3 cm of the ferrule,
  the ring or the wire (not let go). A real trial also scores 0 if the post or the bracket is moved.
- Otherwise, by the ring centre's distance along the wire from the red mark: **1.0** within 10 mm, **0.5** within
  25 mm, **0** farther.
- `success()` also needs the ring at rest. `distance()` is the ring's distance to the mark plus any excess sag,
  plus 8 cm while the ferrule is not in the far notch. `progress()` reports the stages
  `tensioned / ring_near / ring_at_mark / released`.

Real-world measurement: a tape from the mark to the ring centre, and a straight edge across the two supports for the
sag (or a side photo against a grid).

## Runtime and seeds

The scripted demo takes about 23 s (452 control steps at 20 Hz). Run limit: 1,200 steps (60 s at 20 Hz). The layout is fixed with
`object_jitter=0`. The cable is never jittered, and the four ring bars are one welded body, so don't jitter them
independently.
