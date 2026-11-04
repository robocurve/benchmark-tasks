# Holster an EV charger and hang its cable off the parking bay

Tabletop mock of an EV charging point. A white charger post stands at the back of the table between the arms. Its
thick black charging cable (12 mm, 50 cm) leaves the bottom of the post's front-left corner and lies straight along
the front of the post and right across a parking bay taped on the table to the right of the post; the large black
plug body on its end lies just beyond the bay. On the post's front face are a holster cradle for the plug (right) and
a cable hook (left). The robot must put the plug in the holster and hang the cable over the hook so that no part of
the cable or plug lies in the bay, then let go: a charging cable left lying in a parking bay is a trip hazard and
blocks the next vehicle. It is naturally bimanual: the left arm lifts the cable near the post over the hook (the hook is too
high and too far left for the right arm to lift a cable over its lip), the right arm brings the plug from beyond the
bay (out of the left arm's reach) to the holster, which lifts the rest of the cable out of the bay.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Put the charger plug in its holster and hang the cable over the hook on the charger post, so that no part of the cable or plug lies in the taped parking bay, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed-loop (reads the live cable and plug state). Left arm: approaches in front of the post (clear of the holster),
takes the cable 22 cm from the boot (inside the bay) with the fingers ~32 mm open, carries it round in front of the
post, over the hook's lip and lays it on the peg, lets go and moves clear. Right arm: takes the plug near its cable
end, lifts it, turns it to point at the post while carrying it to the holster (this lifts the rest of the cable out of
the bay; it now hangs in a loop from the hook), corrects for where the plug actually sits in its fingers, lowers it
into the slot against the post face and lets go. Success holds for 1 s and ends the episode at step 465 (~23 s); task_score 1.0. `demo/` holds its videos, `setup.png` and
`result.json`.

Unintended strategies (test only, `EVHOLSTER_CHEAT=<mode> python cabledemo.py task.py out`), all score exactly 0:

| `EVHOLSTER_CHEAT=` | what it does | result |
|---|---|---|
| `none` | waits | 0 (cable across the bay, plug not holstered) |
| `holster` | right arm only: plug carried to the holster and released, cable not hung on the hook | 0 (the unhooked cable still lies across the bay; its weight also pulls the plug back out over the lip) |
| `drag` | right arm drags the plug and cable towards the robot and round to the front of the post, leaves them on the table | 0 (cable and plug fully out of the bay, but neither holstered nor hooked) |
| `hold` | full demo, but the right arm keeps holding the plug in the holster | 0 (gripper still at the plug; cable is on the hook and out of the bay) |

Partial credit for genuine progress: cable on the hook and out of the bay with the plug left elsewhere = 0.4; plug in
the holster with the cable out of the bay but not on the hook = 0.6.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if any part of the cable or plug lies over the area inside the bay's outer tape edge (seen from above; in
  sim: cable centreline within the rectangle grown by the cable radius, plug long axis grown by half its width), or
  if a gripper is still at the cable or plug (gripper centre within 4 cm of the cable centreline or 6 cm of the plug
  centre).
- Otherwise **+0.6** if the plug lies in the holster (plug centre between the slot's side walls and between its
  front lip and the post face, 3 mm tolerance, resting on the slot floor) and **+0.4** if the cable rests on the hook (some part of the cable lies
  on the peg's top between its lip and the post face). Holster + hook = 1.0.
- A plug left anywhere else (on the table, hanging) earns nothing for the plug term; a cable dragged out of the bay
  but not on the hook earns nothing for the hook term. Holstering the plug with cable still in the bay is 0.
- `success()` additionally requires the cable to be at rest. `distance()`: plug-to-holstered-position distance (m),
  + 0.1 if the cable is not on the hook.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz). One trial takes ~5-10 min wall time on one GPU including the ~1-2 min
  Isaac Sim start-up.
- Layout is fixed with `object_jitter=0`; the cable is never jittered (it always starts straight), so always run
  with `object_jitter=0`.
