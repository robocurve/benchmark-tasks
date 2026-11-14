# Seat a Mini-ITX motherboard on four test-bench standoffs

The task seats a full-size 170 mm Mini-ITX board on four locating standoffs,
aligns its rear ports with the test-bench opening, and releases the board.
It measures seating for fastening; it does not include screw installation or
electrical testing. The rigid pins do not reproduce commercial push-pin
detents, and the protective edge clamps are coupled to the board in the model.

## Results

- Dream-1: **0.0**, seed 0, 2,500-step limit. The board stayed on its ESD mat.
- Scripted Isaac Lab demo v10: **1.0**, seed 0. The board seated on all four
  standoffs, remained still, and both grippers cleared it.

## Alignment and level criteria

Full score requires all four locating pins to pass through their matching
mounting apertures. At each hole, the pin centre must be within 2 mm on both
horizontal axes. The board underside must be within 2 mm of the standoff tops;
board tilt and yaw must each be at most 3 degrees. The rear I/O housing must
enter the test-bench opening, the board must remain still for 1 second, and
both grippers must open and clear the board by at least 60 mm. The scorer
applies the seating, orientation, port-opening, stillness, and release checks
in that order, with partial credit described in `task.json`.

## Run and score

The maximum is 2,500 control steps at 20 Hz. With the YAM Isaac Lab Arena
environment active, run either policy from this folder:

```bash
./run.sh dream
OUT=~/yam-eval/mini-itx-bench-mount/demo ./run.sh demo
```

Set `YAM_ARENA` to a local checkout of `yam-isaac-lab-arena`, then run the
scorer tests and sampled CPU collision/reach check:

```bash
export YAM_ARENA=/path/to/yam-isaac-lab-arena
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$YAM_ARENA:$PWD" python3 test_scoring.py
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$YAM_ARENA:$PWD" python3 check_local.py \
  --yam-arena "$YAM_ARENA" \
  --modes demo settle no_board --report local_checks.json
```

The local check samples waypoint endpoints and midpoints; it does not replace
an Isaac Lab run. To render a completed evaluation, pass its JSON log and an
output path to `render_top.py`; it writes only the top-camera MP4.

Reset the board flat on the ESD mat and verify the four holes and locating pins
are unobstructed before each trial.
