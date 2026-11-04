# Sort cables onto matching coloured hooks

A grey cable rack stands in front of the robot with three short coloured hooks, 18 cm apart: red on the left,
green in the middle, blue on the right. Three loose black cables, 28 cm long, lie on the table, each with a bare end
and a small coloured plug (red, green or blue) on the other end. The plugs lie left to right as blue, red, green, so no
plug is in its hook's place or nearest to its own hook. The robot must hang each cable on the hook that matches its plug's colour by
draping it over the peg so that it hangs down on both sides, then let go. It has to read each cable's identity from
the small coloured plug on its end, pick the cable by that plug, and drape it so it stays on the hook. Both arms work at
once: the left arm hangs the red cable while the right arm hangs the green one on the middle hook; the blue cable
lies on the left, out of reach of the right arm that serves the blue hook, so the left arm passes its plug across
the midline (sets it down there) and the right arm hangs it.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 \
    --instruction "Sort the cables onto the rack: hang each cable on the hook that has the same colour as the plug on its end, draping it over the hook so that it hangs down on both sides of the hook. Then let go of the cables."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1800
```

Closed-loop (reads the live plug positions before each grasp): the left arm takes the red plug while the right arm
takes the green one; each lifts its cable by the plug, comes to its hook from the side where the rest of its cable
lies, carries the plug over the peg (so the hanging cable catches on it) and lowers it on the far side, then lets go
and backs away. The left arm then picks up the blue plug and sets it down on the midline; the right arm takes it and
drapes the blue cable over the blue hook the same way. Success holds for 1 s and ends the episode at step 969
(~48 s); task_score 1.0. `demo/` holds its videos, `setup.png` and `result.json`.

Cheat / failure variants (test only, `SORT_DEMO=<name> python cabledemo.py task.py out`), all score exactly 0:
`nothing` (do nothing: no cable hung), `order` (ignores colour and hangs the cables in their left-to-right table
order: blue on the red hook, red on the green hook, green on the blue hook: all wrong), `nearest_one` (hangs only
the right-most cable on the right-most hook, which is also the hook nearest to its plug: green on blue, wrong),
`hold` (the full correct demo, but the right arm keeps holding the blue plug at the end: two cables correctly
hung, 0 because a gripper still holds a cable).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** unless both grippers have let go: each gripper's centre at least 3 cm from every cable and plug (real trial:
  no gripper touching a cable).
- A cable is **hung** on a hook when part of it rests on top of that hook's peg (between the lip and the board, at
  most 2 cm above the peg top) and the cable continues down on both sides of the peg (to the left and to the right
  of it, one on each side of the resting part) to at least 4 cm below the peg top. The rest of the cable may lie on
  the table.
- Per cable: hung only on the hook of its plug's colour **+1**; hung on a hook of another colour, or on two hooks at
  once, **-1**; not hung **0**. Score = max(0, sum / 3): 1.0 (all correct, success), 0.67, 0.33 or 0. A wrong hook
  cancels a correct one, so hanging cables regardless of colour (all on one hook, in their table order, or each on its nearest hook) gives 0.
- `success()` additionally requires all cables to be at rest. `distance()`: number of cables not on their own hook,
  +1 while a gripper still holds a cable.

## Runtime and seeds

- Run limit 1,800 steps (1.5 min at 20 Hz). One trial takes ~5-10 min wall time on one GPU including the ~1-2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cables are never jittered (they always start straight), so every
  trial starts identically.
