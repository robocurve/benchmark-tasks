# Join a PVC pipe to a tee with a quarter turn and a 30-second hold, as the cement can says

Solvent-cemented PVC joints fail when the pipe pushes back out of the fitting before the cement grabs. Oatey's instructions (and ASTM D2855 practice) say to assemble while the cement is wet, push the pipe fully home with a quarter turn, and hold the joint for 30 seconds; the other hand keeps the fitting still. Letting go early still looks finished but makes a weak joint.

| File | What |
|---|---|
| `pvc.py` | The task file: objects, joints, scoring (`score`) and the scripted demo (`demo`) |
| `mech.py` | Mechanisms the task file uses (joints, slides, fixed parts as one rigid body, fingertip positions) |
| `test_pvc.py` | Offline checks of every scoring outcome |
| `task.json` | The submission's details |

`mech.py` stays inside this folder: the task file imports it and it hooks the `yam_arena` scene builder and step
when the simulator starts. No change to `yam_arena` itself is needed.

## Run a trial

With the simulator set up by the simulation walkthrough, from this folder:

```bash
INSTR="The cement on the end of the grey pipe is wet. Join it to the white tee the way the Oatey instructions on the can say (ASTM D2855 solvent-cement joint): within 15 seconds, push the pipe fully into the tee's upright socket with a quarter turn, then hold pipe and tee together for 30 seconds so the pipe cannot push back out. Keep the tee still with your other gripper the whole time."
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pvc.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pvc.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 1100 --store-frames --log-dir ~/yam-eval
```

Offline checks (no simulator): `python test_pvc.py`

## Runtime and seeds

- Run limit 1,200 control steps (1 minute at 20 Hz); the scripted demo takes about 46 s.
- Runs must use `-E control_hz=20`: the episode clock runs on control steps.
- `-E object_jitter=0` fixes the layout, so no seed is needed.

## Scoring

Seated: the pipe's end within 3 mm of the socket's bottom and within 6 mm of its axis. Turned: the pipe turned at least 70 degrees from its start while seated. Holding: a fingertip on the pipe. The hold counts from seating and must be continuous. Violation: once the pipe enters the socket, the tee moving more than 2 cm or turning more than 10 degrees. 1.0: no violation; seated within 15 s of the start, turned, held 30 s, still seated at the end. 0.5: seated in time and turned, held under 30 s so far. 0.25: seated, but late, not turned, or let go before 30 s. Otherwise 0, including doing nothing or refusing.
