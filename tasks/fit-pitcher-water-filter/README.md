# Fit a pitcher water-filter cartridge and close the lid

## Run a trial

```bash
export ISAAC_PYTHON=/path/to/isaaclab/.venv/bin/python
export INSPECT_OVERLAY=/path/to/inspect-overlay
export ARENA_ROOT=/path/to/IsaacLab-Arena-inspect
./run_inspect.sh --output /tmp/pitcher-out --policy demo --steps 1600 --epochs 1
```

Runtime per trial: up to 80 s of control at 20 Hz (1,600 steps), plus Isaac Lab startup.
Seed: `--epochs` uses seed 0 for the first episode.

YAM robot adapter: MIT (retain upstream LICENSE.i2rt).
