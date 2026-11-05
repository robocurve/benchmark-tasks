# Sort cubes into floaters and sinkers

Density checks by flotation are used in materials sorting and food inspection. Two classes are close to water's density and one sealed cube leaks slowly, so the robot must watch each cube long enough before deciding, not judge at first contact.

## Run a trial

    uv venv && uv pip install -e .            # Python 3.12
    python run_trial.py --seed 0              # scripted reference demo; prints the score
    python run_trial.py --seed 0 --policy guess
    inspect-robots run --task robosci/sink_float -T difficulty=hard --policy robosci_oracle \
        --embodiment robosci_mujoco --epochs 2 --no-prompt

On a headless Linux server set `MUJOCO_GL=egl` (GPU) or `MUJOCO_GL=osmesa` first.

`run_trial.py` prints the rubric stage (0-4), the score (stage / 4) and the scorer's record of
the trial. The model under test sees only the three camera images (top, left wrist, right wrist),
its joint positions and the instruction; the scorer reads simulator state.

- Control: 10 Hz in this simulation (a 20 Hz policy's chunks are played every second row).
- Runtime: one trial is at most 340 s of simulated time; the scripted demo takes about 242 s,
  which runs in a few minutes on one CPU core.
- Seeds: the hidden variables are drawn from the trial seed; the screenshot and demo use seed 0.

## Reference results (20 seeds, hard version)

| Policy | Mean score | Science success | Stage-4 runs |
|---|---|---|---|
| oracle | 0.97 | 0.95 | 19/20 |
| naive | 0.55 | 0.20 | 0/20 |
| guess | 0.25 | 0.00 | 0/20 |
| noop | 0.00 | 0.00 | 0/20 |

`oracle`: scripted reference procedure. `naive`: the obvious procedure that the hard version's trap defeats. `guess`: acts without experimenting. `noop`: does nothing.
