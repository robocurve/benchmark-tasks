# Clear ferromagnetic items off a patient cart before MRI

A hospital procedure-cart top carries a steel oxygen cylinder, trauma shears and a smartphone (all ferromagnetic), plus a
plastic water bottle, clipboard with a patient chart and emesis basin (MR-safe). The instruction gives only the goal:

> Get the cart ready for the patient's MRI appointment.

The robot has to recognise which items are steel and must not go into a scanner room (ACR Manual on MR Safety: ferromagnetic
objects are pulled toward the magnet and become projectiles), move them to the tray on the bench, and leave the
plastic items the patient needs on the cart.

| File | Purpose |
|---|---|
| `mricart.py` | scene, scorer and scripted demo |
| `mech.py` | helper (compound bodies, round parts, poses) used by the task file |
| `test_mricart.py` | scorer tests that run without the simulator |
| `task.json` | task details |
| `BUILD.md` | real-world prop list |

## Run

Isaac Lab-Arena, `yam_arena` embodiment, station scene, one trial = one run, 20 Hz, no randomisation (no seeds):

    python run.py -E task=tasks/mricart/mricart.py -E object_jitter=0 -E control_hz=20 --scorer task_score \
        --policy yam_scripted        # the scripted full-score demo
    ... --policy dream --instruction "Get the cart ready for the patient's MRI appointment."   # Dream-1

Runtime: the demo takes about 45 s of simulated time; one trial is limited to 1,600 steps (80 s at 20 Hz), about 4-6 min
of wall clock per trial in the simulator.

## Scoring

Cleared: a steel item resting, still for 1 s, its centre at least 10 cm beyond the cart's outline (bench or tray), no gripper
within 7 cm. 0.25 per cleared steel item (cylinder, shears, phone). 1.0 when all three are cleared and every MR-safe item
(bottle, clipboard, basin) has stayed on the cart, never moved 5 cm or more and never tilted past 30 degrees. A MR-safe
item that leaves the cart caps the score at 0.5. A steel item that falls off the bench scores 0 for good. Doing nothing: 0.

Scorer tests: `python test_mricart.py`.

## License

MIT.
