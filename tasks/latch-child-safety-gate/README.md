# Close and lock a pressure-mounted child safety gate across a hallway

Everyday household task: swing a walk-through child gate shut and engage both maker-required locks. Built for the Robocurve YAM / Isaac Lab-Arena stack.

**Status:** both videos recorded and uploaded as media `M-JJ8CZJ`.

- RGB demo: **1.0** at step 650 (32.5 s of the 80 s limit); the trace regrades to 1.0 with the current `scorer.py`.
- Dream-1: **0.0**. The run ended at step 224 when Dream-1 sent a right-gripper command of −0.054, outside the 0–1 range the Inspect Robots Dream adapter accepts. A repeat run stopped the same way at step 416 (−0.063).

## Why this task

Pressure-mounted walk-through gates are ordinary parenting hardware. The [Regalo Easy Step latch procedure](https://manuals.plus/regalo/1164-ew-easy-step-extra-wide-safety-gate-manual) requires the door fully closed, the top lock lever pushed down so the spring latch re-engages, and the base lock down on the child side. An unlatched door is not a secured gate.

Board screening on 20 November 2026 (4,168 unique IDs) found **no** baby / child / stair / safety-gate neighbour. Closest household latches (child-proof drawer, infant car-seat handle) are different mechanisms.

Category is **Societal relatability**: closing and locking a child gate is a recognizable household routine that parents repeat many times a day, not a niche trade skill. A household robot that uses a gated doorway has to leave the gate secured the same way, or a toddler can push it open.

## Geometry and physics

Generic reconstruction of the Easy Step class (not manufacturer CAD):

- Opening width 762 mm; door 406 mm; height 762 mm; ~51 mm bar spacing; 19 mm tube.
- Rectangular 22 × 90 × 25 mm top handle.
- Native revolute door hinge, revolute top lever, prismatic spring latch, revolute base lock.
- Door starts open **35°**; lever raised; base lock horizontal.
- Finger caps: 75 mm aperture, 40 N per finger.
- Physics 240 Hz, control 20 Hz; friction 0.65/0.55 scene, 1.2/1.0 pads.

## Score ladder

- **0.25** door within **7°** of closed.
- **0.5** plus top lever down (within 8°) and ≥4 mm latch/catch overlap.
- **1.0** plus base lock down (within 12°), hands ≥5 mm clear, stable for 10 frames at 20 Hz.
- Workspace escape, moved fixed scenery or damage → permanent 0.
- Once the door has been within 7° of closed with both hands clear, opening it to more than **12°** from fully closed (5° beyond the closed band) → permanent 0 for that trial, even if it is closed again (`Scorer.update` in `scorer.py`, `spec.REOPEN_LIMIT`).

Physical judging: overhead + latch-side video and a millimetre scale at the catch.

## External robot asset

The external **YAM robot asset is MIT licensed** (I2RT YAM via the pinned YAM Isaac Lab-Arena adapter). It is not redistributed here. See `DEPENDENCIES.json` and retain upstream `LICENSE.i2rt`.

## Run

```bash
"$ISAAC_PYTHON" -m unittest discover -s . -p 'test_*.py'
bash run_inspect.sh --policy hold --steps 200 --check-reset --output runs/hold
bash run_inspect.sh --policy demo --steps 200 --output runs/smoke
bash run_inspect.sh --policy demo --steps 1400 --check-reset --output runs/demo
python scorer.py runs/demo/evidence/episode-00/trace.jsonl
"$ISAAC_PYTHON" audit_run.py runs/demo --require-full
```

Dream (only after a full-score demo; shared GPU lock; ledger required):

```bash
bash run_inspect.sh --policy dream --steps 1600 --output runs/dream \
  --budget-ledger /path/to/persistent/dream-budget.jsonl
```

Seed 0, object jitter 0. Demo observations are three RGB cameras only; Dream also receives standard joint/gripper readings. No object poses or scorer state reach the policy.
