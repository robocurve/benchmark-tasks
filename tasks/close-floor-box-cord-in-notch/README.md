# Close a floor box without pinching the cord

An office floor box (a recessed power box) is mocked up as an open white well, 14 x 12 cm inside and 4.5 cm deep,
fixed to the table in front of the robot. Inside it a power socket faces up, as in real floor boxes, so the plug goes
in from above. The top of the box's far wall has a cord notch (15 mm wide, 25 mm deep) 2 cm left of the wall's
centre. A free grey cover plate (16 x 14 cm, 6 mm thick, with a small lift handle on top) rests on the rim and closes
the box. A 6 mm black power cord comes out of a fixed grey service box on the right, beyond the well, and lies
straight on the table towards the robot. Its free end carries a right-angle plug (30 x 20 x 36 mm tall, cord
leaving one side), which stands on the table. The robot has to:

1. open the box;
2. plug the cord in;
3. lead the cord out through the notch;
4. close the cover flat;
5. let go.

The cord's natural path from the socket towards the service box runs over the far wall's rim to the right of the
notch. A cover closed over it rocks on the cord and leaves it pinched, which is the real-world failure: a crushed
cord and a cover that isn't flush. Leaving the cover off doesn't count either. The natural solution is bimanual. One
arm lifts the cover and holds it out of the way. The other plugs in, leads the cord into the notch and pushes the
plug home. The first arm then puts the cover back.

Instruction: `Plug the black power cord into the socket inside the floor box: lift the cover off, plug in, lead the cord out through the notch in the box's far wall, then put the cover back flat on the box and let go.`

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` (the simulated cable) is in this folder next to `task.py`; `cabledemo.py` runs the scripted demo.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Plug the black power cord into the socket inside the floor box: lift the cover off, plug in, lead the cord out through the notch in the box's far wall, then put the cover back flat on the box and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The demo is closed loop on the object states:

1. The left arm pinches the cover's handle, lifts the cover straight up and holds it out of the way on the left.
2. Meanwhile, the right arm picks the plug up by its top and waits beside the box.
3. Once the box is open, the right arm centres the plug over the socket opening (correcting for where the plug
   actually hangs in its grip), lowers it to the bottom of the socket and lets go.
4. The right arm then picks the cord up just outside the box. It sets the cord down on the table just beyond the
   notch, choosing the spot from the measured length of cord so the stretch from the socket runs straight through
   the notch. It presses the plug home with closed fingers and moves away.
5. The left arm brings the cover back over the box (correcting for how the cover sits in its grip), sets it down on
   the rim, opens and moves away.

The right wrist camera looks down into the box at the socket during insertion. The demo reaches `task_score` 1.0 in
694 steps (35 s). Videos are in `demo/`.

Partial-credit variants (`FLOORBOX_DEMO=...`), one per tier:

- `tier04` (0.4): plug in, cord left crossing the far rim, cover set down on the table beside the box.
- `tier06` (0.6): plug in, cord led out through the notch, cover set down on the table beside the box.

Shortcut variants (`FLOORBOX_DEMO=...`), each scoring exactly 0:

- `nothing`: do nothing (cover stays closed, cord outside).
- `hold`: do everything right, but keep pinching the cover's handle at the end.
- `lazy`: plug in, then close the cover without leading the cord anywhere. The cord crosses the far rim and the cover
  rocks on it.
- `pinch`: plug in, lead the cord over the far wall's rim right of the notch, close the cover on it.

## Scoring (`score()` at the end of the trial; `success()` = score 1.0 with cover and cord at rest, held 1 s)

**Score 0** if any of these holds at the end:

- a gripper is within 3.5 cm of the cord, the plug, the cover or its handle (not released);
- the plug sits less than 15 mm deep in the socket (its bottom less than 15 mm below the socket rim);
- the cover is over the box (any part of it above the box's outer outline) but not closed flat with the cord in the
  notch: any corner of its underside more than 3 mm above the rim (a cord under an edge lifts it 6 mm or more), or its
  centre more than 1 cm (horizontally) from the box centre. This is the pinched-cord failure.

Otherwise the end state falls into exactly one tier. Each tier needs everything in the tier below it:

| Score | End state |
|---|---|
| 0.4 | Plug seated, cord NOT through the notch, cover set aside with no part of it over the box. |
| 0.6 | Plug seated, cord led out through the notch, cover set aside with no part of it over the box. |
| 1.0 | Plug seated, cord led out through the notch, cover closed flat and centred on the box. |

A box left open is unfinished but harmless, so it earns partial credit. A cover closed on the cord crushes it, so it
scores 0. A cover can only lie flat when the cord leaves through the notch, so there is no "closed but not routed"
tier.

Real-world measurement: put a mark on the plug body 15 mm above its bottom (at or below the socket rim = seated). Try
a 3 mm feeler gauge under each cover edge (it must not fit). Check by eye that the cord leaves through the notch, that
a set-aside cover has no part over the box, and that the grippers are open and away.

**Changed in this revision:** the old ladder (0.6 for a seated plug, +0.4 for the notch, cover closed flat in every
case) could only score 0 or 1.0, because a closed cover without the cord in the notch always rocks on the cord. The
partial tiers now cover a seated plug with the box left open (0.4), plus the cord routed through the notch (0.6). The
zero gates for doing nothing, holding on and closing the cover on the cord are unchanged.

## Runtime and seeds

Run limit 1,200 steps (60 s at 20 Hz); the demo needs 694. A trial takes a few minutes wall-clock on one GPU
(first start of Isaac Sim adds 1–2 min). The layout is fixed (`object_jitter=0`; the cord is never jittered), so
every seed starts from the same state.
