# Find the sealed box with a loose ball

Inferring what is inside a closed container from how it moves is common in inspection and logistics. One box holds an off-centre fixed weight that also keeps a rocker tipped, so a test in one direction cannot tell it from the loose ball; the robot has to test both directions.

## Run a trial

    uv venv && uv pip install -e .            # Python 3.12
    python run_trial.py --seed 0              # scripted reference demo; prints the score
    python run_trial.py --seed 0 --policy guess
    inspect-robots run --task robosci/sealed_box_ball -T difficulty=hard --policy robosci_oracle \
        --embodiment robosci_mujoco --epochs 2 --no-prompt

On a headless Linux server set `MUJOCO_GL=egl` (GPU) or `MUJOCO_GL=osmesa` first.

`run_trial.py` prints the rubric stage (0-4), the score (stage / 4) and the scorer's record of
the trial. The model under test sees only the three camera images (top, left wrist, right wrist),
its joint positions and the instruction; the scorer reads simulator state.

- Control: 10 Hz in this simulation (a 20 Hz policy's chunks are played every second row).
- Runtime: one trial is at most 260 s of simulated time; the scripted demo takes about 148 s,
  which runs in a few minutes on one CPU core.
- Seeds: the hidden variables are drawn from the trial seed; the screenshot and demo use seed 0.

## Reference results (20 seeds, hard version)

| Policy | Mean score | Science success | Stage-4 runs |
|---|---|---|---|
| oracle | 1.00 | 1.00 | 20/20 |
| naive | 0.50 | 0.00 | 0/20 |
| guess | 0.25 | 0.00 | 0/20 |
| noop | 0.00 | 0.00 | 0/20 |

`oracle`: scripted reference procedure. `naive`: the obvious procedure that the hard version's trap defeats. `guess`: acts without experimenting. `noop`: does nothing.
