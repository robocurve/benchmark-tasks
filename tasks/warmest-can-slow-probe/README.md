# Find the warmed can in a chilled delivery with a slow, jittery thermometer

A receiving check: three identical sealed cans from one refrigerated delivery, one of which has warmed up. The only
instrument is a contact thermometer plate with a display. Its reading takes about 20 s to settle on a new can and
jitters by about a degree between updates, so a quick look at each can ranks them by the order they were checked.
The robot has to stand each can on the plate, let go, wait for the display to stop drifting, keep reading, and repeat
until the warmest can is clear, then put it on the red reject mat.

## Files

- `task.py`: scene, thermometer, scoring (yam_arena task file). Every control step it works out which can stands on
  the plate untouched, moves the sensor toward that can's hidden temperature (or toward room temperature), and every
  0.5 s shows a new reading on the display and logs it. `score()` and `success()` use only that log of displayed
  readings and the final state.
- `robokit.py`: parts built from several boxes and cylinders (the cans, the display box). It hooks `yam_arena.scene`
  and the embodiment from the task's import; no change to yam_arena is needed.
- `seg7.py`: the 7-segment display, built from boxes that are shown or hidden.
- `demo.py` and `script.txt`: the scripted demo (see below).
- `model_check.py`: the thermometer and scoring rules without the simulator, used for the statistics in
  `checks.md` (error rate of the proof, blind and quick procedures, run time).

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed:

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 6000 --store-frames --log-dir logs

The score is `task_score` in the log: 1.0 on success, otherwise `score()` from `task.py` at the end of the episode.
Run one trial per process: the reading log is kept in the task module and cleared at each reset.

Scripted demo (writes one MP4 per camera, `setup.png` and `result.json` with the score and the reading log):

    python demo.py task.py demo_out --max-steps 6000

The demo uses only what the cameras show: the cans' positions, its own joints and clock, and the number on the
display, decoded from which LED segments are lit. It never reads the hidden temperatures, the sensor model or the
scorer's log. For each can it carries the can onto the plate, parks the arm clear, reads the display every 0.5 s
until its own readings stop drifting (the same public rule the scorer uses), reads 10 s more, and puts the can back.
It then runs the same significance test as the scorer on its own readings; if no can is proven warmest it measures
the leader and the closest rival again, and finally carries the warmest can to the mat (the left arm sets it on the
plate, the right arm takes it from there: neither arm reaches across the table with a full can).

Baselines (`checks.md`): `DEMO_MODE=guess` (a random can to the mat, no measuring), `DEMO_MODE=quick` (one reading
2 s after setting each can down), `DEMO_SCRIPT=script_noop.txt`, `script_random.txt` (random joint targets) and
`script_reach.txt` (random reaches and grasps over the table); `BASELINE_SEED` seeds them.

## Runtime and seeds

- Run limit 6,000 control steps (5 min at 20 Hz). The demo reaches success at step 4,027 (201 s) at seed 0, and at
  steps 3,779 to 4,011 at the other seeds checked: three measurements of about 25 s each, seven carries, and the
  final carry by way of the plate.
- Wall time: 15 to 40 minutes per trial on one TITAN RTX at 320 x 176 per camera, depending on load.
- The hidden temperatures are drawn from `WARMCUP_SEED` (default 0; the screenshot and demo use 0). One fixed
  starting layout: run with `-E object_jitter=0`.
