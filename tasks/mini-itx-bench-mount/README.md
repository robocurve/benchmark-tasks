# Seat a Mini-ITX motherboard in a computer assembly fixture

Team assemblers position and mount motherboards during regular computer
assembly. This task seats a full-size 170 mm Mini-ITX board on four locating
standoffs in an assembly fixture, aligns its rear ports with the opening, and
releases it. The four C/F/H/J hole positions and 3.96 mm round hole diameter
come from [the Mini-ITX addendum, Figure 3](https://content.instructables.com/ORIG/FJD/TETT/GU59XBKK/FJDTETTGU59XBKK.pdf).
The scene's +X edge is the rear I/O edge: C and F are the rear-side mounting
holes, with F recessed 22.86 mm farther into the board than C.
The 2.4 mm round fixture pins leave 0.78 mm nominal radial clearance. The
simulator approximates the round apertures with thin collision strips and
couples the protective edge clamps to the board. Screws, pin detents, and
electrical testing are outside this placement trial.

## Results

- The 2026-11-15 Isaac Lab scripted demo with the corrected hole-to-I/O orientation scored **1.0** in 299 control steps. Its final maximum hole-to-pin offset was 0.763 mm, below the 0.770 mm full-score limit.
- Dream-1 on the corrected scene scored **0.0** at the 2,500-step limit.
- Earlier videos and scores do not validate this revision.

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
