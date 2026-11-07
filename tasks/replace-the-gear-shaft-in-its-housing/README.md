# Replace the shaft that carries a gear inside a closed housing

A closed housing has a bore through its front and rear walls. Inside it a gear hangs on the old (dark) shaft, which
is flush with both faces of the housing. The robot has to fit the new (orange) shaft in its place. The new shaft is
longer than the housing, a grooved trough in front of the housing lines it up with the bore, and a rail behind the
housing catches whatever comes out of the rear. A slim drift lies on the table too.

The way that works is to push the old shaft out with the new one, end to end, so the gear always rests on one of
them. If the bore is ever left empty (the old shaft driven out with the drift and the drift taken out, or the new
shaft pushed in and drawn back out), the gear drops about 10 mm to the cavity floor. Its flat face then blocks the
new shaft, and nothing can reach in to lift it: the housing is closed apart from the two bore ends and an 8 mm slit
in the roof, which shows the gear to the top camera. The slit is narrower than the 10 mm drift; a fingertip can reach
the gear's top through it but cannot get under the gear. The instruction states only the goal.

The arm cannot feed the new shaft from behind: the bulkhead stands 135 mm behind the rear face, room for the 102 mm
old shaft to come out but not for the 170 mm new one to be lined up, and the space is also beyond the arm's reach at
bore height (the scripted IK misses it by 24-54 mm).

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena, run through Inspect Robots), on the `station` scene. It uses `parts.py` (in this folder) to build the
housing, the gear and the rods from several coloured boxes and cylinders. `near_misses.py` lists near-successes and
wrong outcomes with the score each must get; it runs without a simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Replace the dark shaft with the new orange one."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames

# Scripted demo (reaches 1.0 after about 21 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames
```

The run prints `task_score`: 1.0 on success, otherwise 0.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1-2 minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Submissions run with `-E object_jitter=0`, so every object starts exactly where `task.py`
  puts it.

## Scoring details

`success()` reads the poses of the two shafts and the gear, and a release check:

- **New shaft through:** its far end at or past 2 mm short of the rear face (x 45.0 cm), its near end at or in front
  of the front face (x 35.0 cm), its centre within 4 mm of the bore line sideways and 4 mm in height (checked separately), and its axis within
  3 degrees of it.
- **Old shaft out:** its world bounding box does not overlap the housing's outline (35.0-45.2 cm front to back,
  76 mm wide, 80 mm tall).
- **Gear in line:** its centre within 2 mm of its starting height, and its front-to-back axis within 8 degrees of
  level. A sideways swing on the shaft (about the bore line) is allowed; the gear hangs from the shaft and swings
  about 8 degrees in the demo.
- **Released:** after every control step an `on_step` callback reads both arms' `gripper`, `tip_left` and `tip_right`
  link poses, places their URDF collision hulls (`yam_arena.analysis`), and measures the distance from points on
  both shafts' surfaces (every 5 mm along, 16 round) to each hull. The plane distance used is a lower bound on the
  true distance, so the check errs toward "touching". Any value under 5 mm counts as touching. Offline
  (`near_misses.py`, no simulator) the check is skipped and counts as released.
- **Still:** under 1 cm/s for both shafts and the gear, and under 0.5 rad/s for the gear. For the shafts the spin
  limit is 2 rad/s about any axis, because the simulated cylinders are faceted and rock in place on the rail; in
  practice that rocking is about their own axes.

Success must hold for 3 s (60 steps, `SUCCESS_HOLD_S`), so the demo's arm is clear of the housing in the last frame; `score()` is 1.0 when `success()` holds, otherwise 0. A 1e-6 margin applies
to every limit, the 5 mm release distance included. No record is kept over the trial. A gear that has fallen to the cavity floor (10 mm) cannot be brought back into
line from outside the housing, so an end-state check of its height is enough. The round rollers lead a shaft
under the gear only while the gear sits less than 3 mm low (the shaft's top must meet a roller below its centre).
A gear resting 4.7 mm low on the drift puts the flat face above the rollers in the shaft's path, so by geometry a
14 mm shaft pushed in behind the drift stops at it; that case was not traced. If it did ride under and lift the gear
back into line, it would be the dummy-shaft method, and scoring it 1.0 would be right.

## Physics, as measured on the GPU

- At rest the gear hangs on the old shaft by its two rollers and does not move.
- **End-to-end push (the demo):** the gear slides back with the shafts until it meets the rear wall, then the new
  shaft passes under it; the gear's centre ends 0.6 mm below its start. The old shaft comes out onto the rail.
- **Drift first:** pushing the old shaft out with the 10 mm drift lets the gear drop 4.7 mm onto the drift; drawing
  the drift out drops it the full 10 mm onto the cavity floor. A rod pushed in again then stops against the gear's face,
  4 mm inside the cavity, and cannot be pushed further.

Things the build needed, each found in a trace: the new shaft first caught on the rear bore's lip when its end
drooped, hence the guide strips and the rear bore 1 mm deeper and 21 mm wide; it caught on the gear's flat bore top,
hence the gear hanging on round rollers that lead a shaft under it; and it was pushed sideways out of a shallow
groove, hence the 4 mm deep trough groove.

## Scripted demo

`demo()` uses fixed waypoints only, worked out from the starting layout, with the gripper pointing straight down and
turned only about the vertical. The left arm grips the new shaft at its middle, carries it across, and lets it down
into the trough 10 mm short of the old shaft. It then turns the closed gripper a quarter turn, comes down behind the
shaft's near end, and pushes it 130 mm along the groove in one stroke (commanded 16 mm further, because the soft arm
lags against friction), holds, backs off and moves well clear of the housing.

Failure replays (same command with the scripted policy and `SHAFT_DEMO=...`), each run as a GPU trace on the final
code; every one scored 0:
- `part`: the push stopped after 45 mm. The old shaft is still in both walls.
- `hang`: the push stopped early. The new shaft's end and the old shaft's front end are 24 mm inside the rear face.
- `hold`: the full push, with the fingers left pressed on the shaft's end. Shafts and gear end where the demo leaves
  them (old shaft out, gear in line), but the release check fails, so the score stays 0.
- `fingers`: the closed fingers push on the old shaft's flush front end. They cannot enter the bore and the old
  shaft does not move.
- `drift`: the old shaft driven out onto the rail with the drift (the gear drops 4.7 mm onto the drift), then the
  drift drawn back out (the gear drops the full 10 mm to the cavity floor). In this replay the scripted grip then
  failed to lift the drift clear of the trough, so the final push drove the drift back in; it stopped against the
  dropped gear's face 4 mm inside the cavity. In an earlier trace of the same replay (with a 2 mm trough groove) the
  drift was laid aside and the new shaft was pushed instead; it stopped at the same place.

Not run: drawing the new shaft back out after a part push. With the shaft lying in the trough the 4 mm groove lips
keep the finger pads above its equator, so the scripted grip slips; the drift replay shows what an empty bore does.
Not run: feeding the new shaft in from behind, which the scripted IK cannot reach (see above).
