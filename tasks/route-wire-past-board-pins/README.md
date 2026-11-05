# Route a wire past formboard pins along the drawn path, then plug it in

A wire-harness formboard lies in front of the robot: a white layout sheet with a field of 12 steel routing pins
(3 rows x 4 columns, 8 cm pitch) and a blue route line printed between them. An orange 7 mm wire leaves a fixed start
fixture at the front-left of the board and lies straight along the board's front edge towards the right arm, with a
connector plug on its free end. The blue line runs from the start fixture along the front edge, turns into the pin
field through the gap between the first and second pin columns and climbs through it in a two-step staircase to a
black mating connector behind the field. The robot must lead the wire through the pin field so that it passes **every pin on the same side as the
blue line**, push the plug into the connector, and let go. The rule is topological, so it is checked pin by pin from
a top photo: slack and exact wire position do not matter, only which side of each pin the wire is on. The obvious
shorter routes are wrong: along the front to the right-most gap and straight back to the connector passes three pins
on the wrong side; in at the drawn gap and straight back to the last row before stepping right passes one pin on the
wrong side. Both score 0 even with the plug fully inserted.

The task does not require both arms; a left-arm hold is optional but helps. In the scripted demo the right arm picks up the plug,
carries it to the entry gap and leads it through the gap centres of the staircase (the trailing wire wraps the
inside-corner pins); the left arm then holds the routed wire down at the entry while the right arm squares the plug
up and pushes it into the connector, so the insertion pull cannot drag the routed part off its pins.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Route the orange wire on the harness board along the blue printed line: the wire must pass every steel pin on the same side as the blue line. Then push the wire's plug into the black connector at the end of the line and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Test-only variants (environment variable `PINS_DEMO`): `nothing` (no motion), `shortest` (front edge to the
right-most gap, straight back and in), `onejog` (drawn entry, straight back to the last row gap, one step right, in),
`hold` (drawn route and inserted, but the plug is never let go). All score 0.

## Scoring

`score()` (end of episode) closes the wire (start fixture -> plug) with the printed line (connector mouth -> start)
and computes the winding number of that loop around each pin: 0 means the wire passes that pin on the same side as
the line.

- Score 0 if: any pin has a non-zero winding number (wire on the other side of it) or the wire lies over a pin; any
  part of the wire is more than 25 mm above the board; a gripper is within 3.5 cm of the plug centre; or the plug
  centre is more than 4 cm from the connector mouth.
- Otherwise: plug tip at least 12 mm inside the cavity (centre within 6 mm of its axis, pointing within 30 deg of it)
  1.0; plug tip inside the cavity (0-12 mm) 0.5; plug lying within 4 cm in front of the mouth 0.25.

`progress()` reports stage diagnostics (fraction of pins on the drawn side, wire on the board, plug near / seated,
released); it does not change the score.

## Runtime and seeds

The scripted demo reaches 1.0 in about 510 steps (25 s of the 2,400-step limit); a trial takes a few minutes of wall time on one GPU. The layout is fixed with `object_jitter=0`; the wire is not jittered. `VARIATION` in `task.py` lists
seeded ranges for a later runner (wire spawn heading +-3 deg).
