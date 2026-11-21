# Clip a dog leash bolt-snap onto a collar D-ring

## Run a trial

```bash
export ISAAC_PYTHON=/path/to/isaaclab/.venv/bin/python
export INSPECT_OVERLAY=/path/to/inspect-overlay
export ARENA_ROOT=/path/to/IsaacLab-Arena-inspect
./run_inspect.sh --output /tmp/leash-out --policy demo --steps 1600 --epochs 1
```

Runtime per trial: up to 80 s of control at 20 Hz (1,600 steps), plus Isaac Lab startup.

YAM robot adapter: MIT (retain upstream LICENSE.i2rt).
