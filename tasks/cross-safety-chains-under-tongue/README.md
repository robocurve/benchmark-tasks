# Cross the trailer safety chains under the tongue

A fixed trailer tongue points at the robot, its coupler sitting on a hitch ball. Two dark steel safety chains hang from eyes
on either side of the tongue and lie on the table. In front of the coupler, a tow plate carries two pins. Each chain's
yellow end link must go over the pin on the **opposite** side, so the chains cross under the coupler, and neither
chain is left touching the table. Each end link starts where only the arm on its own side can reach it, so each arm carries
its own chain across the midline, one after the other. In the demo the second chain ends up lying over the first at the crossing (the score does not care which is on top).

Reach check (offline IK, gripper down, z 1.8 and 6 cm): the left arm cannot reach any point of the right chain, from its eye (x 38, y -5.6) to its end link (x 24, y -20), and the right arm none of the left chain; both arms reach both pins. So each arm must fetch its own chain, and neither arm alone can finish.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Hook the trailer's two safety chains onto the tow plate: each chain's yellow end link goes over the pin on the opposite side, so the chains cross under the coupler block. Leave neither chain touching the table. Then let go of both chains."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop on the live end-link poses. The left arm pinches the left chain's end link by a side bar, lifts it, turns
it square and carries it under the coupler to the right pin, corrects the link centre over the pin and lets it drop
over the pin onto the plate, then backs off. The right arm then does the mirror image onto the left pin, so its chain
lies over the first at the crossing. Each wrist camera faces forward, towards the pins and the coupler.
`demo/` holds the videos, `setup.png` and `result.json`.

Shortcut variants (test only, `CHAINS_CHEAT=<name> python cabledemo.py task.py out`):

| variant | what it does | score |
|---|---|---|
| `none` | does nothing | 0 |
| `hold` | does the task, but the right gripper keeps hold of its end link | 0 |
| `straight` | hooks each chain straight ahead onto the pin on its own side | 0 |
| `short` | carries each chain across but aims the end link 3.5 cm past the pin; the link catches on the coupler and drops beside the plate, on no pin | 0 |
| `one` | hooks only the left chain, crossed (a partial-credit check) | 0.3 |

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if a gripper centre is within 5 cm of a chain or end link; if an end link is on the pin on its own chain's
  side; or if any part of a chain is above the coupler underside (10 cm), which means it was laid over the tongue (the chain top is measured, radius included).
- **0.3** exactly one end link on the opposite pin (the pin's axis passes through its 20 mm opening), the other chain not hooked.
- **1.0** both end links on the opposite pins, the chains cross in top view only inside the coupler footprint and below
  its underside, and every chain point is at least 1 cm above the table.
- Anything else, including doing nothing, is 0. `distance()` is the summed distance of the end links to their pins.

## How this differs from similar tasks

Other tasks cross two leads by plugging them into swapped, labelled ports, hook one cable to a ring, or set one
cable's sag between hooks. Here there are no plugs or labels: two hanging chains anchored under a fixed bar must be
crossed under it onto opposite pins at the midline, which is a reach split that needs both arms, and both chains must
end up off the table.

## Runtime and seeds

Run limit 1,200 steps (60 s at 20 Hz). One trial takes about 3-5 min of wall time on one GPU, including the
Isaac Sim start-up. Layout is fixed with `object_jitter=0`; the chains always settle the same way from their straight spawn.
