# Find and replace the faulty part in a circuit

Swap testing with known-good spares is the standard repair method for electronics. Here more than one installed part may be bad and the number is not given, so a single swap that leaves the bulb dark proves nothing; the robot has to plan swaps that separate the possibilities.

## Run a trial

    uv venv && uv pip install -e .            # Python 3.12
    python run_trial.py --seed 0              # scripted reference demo; prints the score
    python run_trial.py --seed 0 --policy guess
    inspect-robots run --task robosci/faulty_circuit -T difficulty=hard --policy robosci_oracle \
        --embodiment robosci_mujoco --epochs 2 --no-prompt

On a headless Linux server set `MUJOCO_GL=egl` (GPU) or `MUJOCO_GL=osmesa` first.

`run_trial.py` prints the rubric stage (0-4), the score (stage / 4) and the scorer's record of
the trial. The model under test sees only the three camera images (top, left wrist, right wrist),
its joint positions and the instruction; the scorer reads simulator state.

- Control: 10 Hz in this simulation (a 20 Hz policy's chunks are played every second row).
- Runtime: one trial is at most 600 s of simulated time; the scripted demo takes about 377 s,
  which runs in a few minutes on one CPU core.
- Seeds: the hidden variables are drawn from the trial seed; the screenshot and demo use seed 0.

## Reference results (20 seeds, hard version)

| Policy | Mean score | Science success | Stage-4 runs |
|---|---|---|---|
| oracle | 1.00 | 1.00 | 20/20 |
| naive | 0.54 | 0.15 | 0/20 |
| guess | 0.25 | 0.00 | 0/20 |
| noop | 0.00 | 0.00 | 0/20 |

`oracle`: scripted reference procedure. `naive`: the obvious procedure that the hard version's trap defeats. `guess`: acts without experimenting. `noop`: does nothing.
