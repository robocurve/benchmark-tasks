# Connect shore power with the whole cord on the dock

A boat (white hull block) is moored in the water (blue sheet) beside a raised dock (6 cm tall). Between them, along
the dock's front face, floats a low grey fender board. The boat's thick yellow shore-power cord (9 mm, 52 cm) leaves
an inlet on the hull at deck height and lies, dry, along the fender board, plug on the board. The robot must plug the
cord into the socket of a grey pedestal on the dock **and** leave the whole cord lying on the dock: a cord left
hanging off the dock edge slips into the water and, live, can cause electric-shock drowning, so plugging in with the
cord still hanging off the dock scores 0, while getting the cord onto the dock without plugging in earns half credit.
Only the first 12 cm of cord from the inlet (up to a black band) may cross the gap between hull and dock. The cord
never touches the water. The geometry makes the task bimanual: the hull-side part of the cord lies in front of the
right arm (out of the left arm's reach), the plug and pedestal in front of the left arm. The natural solution is for
the right arm to lift the cord onto the deck and hold it there while the left arm brings the plug over the deck and
pushes it into the socket.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Connect the moored boat to shore power. Lift the yellow cord off the grey fender board and lay the whole cord on the dock, clear of the water and the fender board, then push its plug fully into the grey pedestal's socket and let go."
```

After every reset the scene is stepped for 3 s with the arms at rest so each trial starts with the cord settled on
the fender board (it spawns straight at deck height and drops 2.5 cm onto the board).

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Closed loop on the live cord state: the right arm takes the cord 20 cm from the inlet, lifts it over the dock edge
and holds it on the deck 10 cm behind the edge; the left arm pinches the plug off the fender board, carries it over the
deck, lines it up with the socket (sideways, height and heading, closed loop on the plug) and pushes it ~20 mm
in; both let go. Success at step 467 (~23 s); task_score 1.0. `demo/` holds the videos, `setup.png` and `result.json`.

Test-only variants (env var `SHORE_DEMO`), all run in sim with the final `task.py`:
- `nothing`: do nothing → 0 (cord on the fender board).
- `plug_only`: left arm carries the plug to the socket, cord left hanging off the dock → 0 (the hanging cord is too
  short for the plug to go in; 22 cm of cord stays off the dock).
- `dangle`: full demo (plug seated 23 mm), then the right arm puts its part of the cord back over the dock edge and
  lets go → 0 (16 cm of cord off the dock).
- `hold`: everything done but both arms keep holding → 0.
- `dock_only`: cord laid on the dock, plug left on the deck in front of the socket → 0.5 (safe partial).

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if a gripper still holds/touches the cord or plug (a TCP within 3 cm of the cord or 4.5 cm of the plug
  centre).
- **0** if any cord point more than 12 cm (arclength) from the inlet, or the plug centre, is outside the dock top in
  top view or more than 1 cm below the deck (hanging over the edge, on the fender board or hull, over the water).
- Otherwise **0.5** (cord safely on the dock) **+ 0.5** if the plug tip is at least 15 mm inside the socket, lined
  up with it (centre within 8 mm of the socket axis, pointing in).
- `distance()`: cord length off the dock + remaining insertion depth.

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). A trial takes ~3–5 min wall time including Isaac Sim start-up.
- Layout fixed with `object_jitter=0`; the cord is not jittered and settles the same way every trial.
