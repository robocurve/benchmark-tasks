# Play a slow 4/4 rock-ballad groove with two sticks on an electronic drum kit

> **Status: both trials have been run in Isaac Sim, headless.** The scripted demonstration scores 1.0 (305 of 1,200
> steps; repeated once with an identical trace), and 11 controls have been run natively. Dream-1
> (`dreamlabs/dream-1-beta`) scores 0.0: it used all 1,200 steps, gripped both sticks, then opened its grippers and
> dropped them, and no hit was counted. `validation.json` records both runs and the controls.

The task instruction is: "Play this groove on the drum kit with the stick in each hand, at 75 beats per minute,
counting 1 & 2 & 3 & 4 &. Bars 1 to 3: hit the hi-hat on every count, loud on the numbers and soft on the ands, and the
snare loud on 2 and 4 together with the hi-hat. Bar 4: the same up to 3 &, then only the tom, on 4 and on 4 &. End with
the hi-hat and snare together on 1 of the next bar. Let each stroke come straight off the pad; at the end hold both
sticks still, clear of the kit."

The fixed scene holds three pads of an Alesis Nitro Max electronic drum kit at their real sizes, set for open-handed
playing: the 10 in hi-hat pad in front of the left arm, the 10 in mesh snare and an 8 in mesh tom in front of the right
arm, each on the kit's own clamp and L-arm on a rack tube along the table's far side. Each gripper holds a Vic Firth
American Classic 5A stick by friction alone, its finger pads on the stick's upper and lower sides. There is no bass
drum: the YAM has no foot, and the task is the hands' part of the groove. TECHNICAL.md has the physics, the scoring
and every number's source.

## Run and score

Runtime: Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0, inspect-robots-yam 0.38.0 and
yam-arena 0.1.0 on Linux with an NVIDIA RTX GPU (an L40S or A10, for example; A100 and H100 have no RT cores). Install
the [official YAM runtime](https://github.com/kairosiann/yam-isaac-lab-arena), then this task into its environment,
from this folder:

```sh
git clone https://github.com/kairosiann/yam-isaac-lab-arena && (cd yam-isaac-lab-arena && ./setup.sh)
source ~/IsaacLab-Arena/yam.env
# The Dream-1 trial also needs the Dream client in the same environment, if setup.sh did not install it:
uv pip install 'dreamlabs-client==1.0.14' 'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
uv pip install --no-deps .
python -m rdb_drum_groove.run scripted --out out/scripted     # the scripted demonstration, headless (add --live to watch it)
python -m rdb_drum_groove.run dream --out out/dream           # one Dream-1 trial
RDB_DRUM_DEMO=air python -m rdb_drum_groove.run scripted --max-steps 400 --out out/control-air   # a control (TECHNICAL.md)
python tools/soundtrack.py out/scripted                       # the combined video with sound: out/scripted/demo.mp4
```

Configure the Dream Labs client separately (`~/.dreamlabs/credentials`); no credentials are included. Both commands
use the same fixed layout, seed 0, zero object jitter, 20 Hz control (240 Hz physics) and the scorer
`drum_groove_score`. The limit is 1,200 actions at 20 Hz (60 simulated seconds), about four times what the scripted
demonstration needs (305 steps). The scene is fixed, so there are no other seeds. `run.py scripted` exits 0 only
if the demonstration scores 1.0. `--live` shows the Isaac Sim window and does not change the trial.

Runtime per trial, measured on one NVIDIA A10 (24 GB), headless: 272 s from start-up to the result for the
scripted trial (305 steps), with the GPU shared with other Isaac Sim runs. The Dream-1 trial, which used all 1,200
steps and waited on 49 requests to the Dream-1 service, took 714 s on another A10, with the GPU shared with one other
Isaac Sim run. Allow about 4 more minutes for the first start of Isaac Sim on a new machine.

Each command writes, in its `--out` folder: `RESULT.json` (read `score`; also the steps, wall time, the reason the trial
ended, file hashes, the physics settings, the judged hits and diagnostics), `media/hits.mid` (every hit as a MIDI
note-on, as the drum module would send it), `media/events.json` (each hit's stick, any gripper or arm striking a pad,
any stick resting on a pad, any dropped stick, and when the end became clean), `media/score-vs-hits.png`,
`media/scored-hits.csv` and `media/score-vs-hits.json` (see "Score against hits"), `media/setup.png` (the first
overhead frame), `media/top_cam.mp4` (the overhead camera, 640 x 360), `media/side_cam.mp4` (a camera in front of the
kit for the video, 1280 x 704, never given to a policy), `media/end_top_cam.png`, `media/end_side_cam.png` and
`media/grader-trace.jsonl` (the hits, strikes and rests counted every step, and the judged score whenever it was
judged), plus Inspect Robots' own log. The Dream-1 trial also writes `dream-input-rgb.zip`, `dream-action-trace.jsonl`
and `DREAM_USAGE.json`. A trial that fails still writes `RESULT.json` with the error and how far it got. Videos are
recorded at 20 frames a second (every step): a stroke lasts only a few frames.

Dream-1 receives 320 x 176 overhead and two wrist RGB images, plus robot joint and gripper positions; nothing else (no
hits, no sound, no side camera). The scripted reference may read simulator state (when each stick last touched a pad);
it commands the robot and moves nothing in the scene itself.

Videos: `tools/soundtrack.py` makes each run's combined video (overhead and side together, 20 fps) with a drum track
synthesized from `media/hits.mid`: every logged hit, including wrong pads and extra hits, at its exact time and
loudness, with no backing music. The video carries the caption "Sound added offline from the logged hits; not
simulated; the robot heard nothing". The drum sounds are synthesized by this package's own code (MIT; no samples). At
20 fps the picture can trail the sound by up to 50 ms. The task card's screenshot is `setup.png` in the recorded run's
folder: the first frame of `demo.mp4`, overhead and side together (`ffmpeg -i demo.mp4 -frames:v 1 setup.png`).

## The start pose

The arms start at a playing pose (`fixed_scene.START_Q`, radians, joints 1 to 6), not the station's ready pose: an
up-and-down grip cannot be set at the ready pose, where the finger pads face sideways.

| Arm | Joints 1 to 6 (rad) |
|---|---|
| left | -0.32717, 0.93901, 0.21786, 0.72071, 0.96495, 1.56246 |
| right | 0.25913, 1.56991, 0.27403, 1.29533, -1.10679, -1.66646 |

Each gripper points sideways toward the robot's centre line, its finger pads on the stick's upper and lower sides at
the tape mark 135 mm from the butt, the stick at right angles to the gripper, its tip 80 mm over the hi-hat's bow (left)
or the snare's head (right). The grippers are closed (commanded fully closed: about 35 N on each finger pad).

## Score against hits

Score against hits: for each run, a plot of every logged hit against the written groove, with the same numbers in scored-hits.csv.
The plot (`media/score-vs-hits.png`, 2400 x 900, English, DejaVu Sans) has a row per pad (Hi-hat,
Snare, Tom, Other), the fitted beat on its x axis with bars 1 to 5, the written notes as hollow circles (loud, medium,
soft), each hit as a filled marker at its real time (circle left stick, square right stick, diamond stick not
recorded; green on time within 25 ms, orange off the grid from 25 to 50 ms with its signed error, red wrong pad, purple
extra, grey count-in) with its MIDI velocity, and each pair joined by a line, green when together (two sticks within
40 ms). The title line gives the run, its score, the fitted tempo, the RMS timing error, the largest pair spread, the
pairs together, the extra or wrong hits and the notes played.

| Run | Score | Plot |
|---|---|---|
| `scripted-1` (demonstration) | 1.0 | `out/scripted-1/media/score-vs-hits.png` |
| `scripted-2` (the same code again) | 1.0 | `out/scripted-2/media/score-vs-hits.png` |
| `control-flat` | 0.9 | `out/control-flat/media/score-vs-hits.png` |
| `control-tempo82` | 0.75 | `out/control-tempo82/media/score-vs-hits.png` |
| `control-no_fill` | 0.75 | `out/control-no_fill/media/score-vs-hits.png` |
| `control-flam_bar3` | 0.5 | `out/control-flam_bar3/media/score-vs-hits.png` |
| `control-gripper` | 0.5 | `out/control-gripper/media/score-vs-hits.png` |
| `control-one_arm` | 0.25 | `out/control-one_arm/media/score-vs-hits.png` |
| `control-air` | 0.0 | `out/control-air/media/score-vs-hits.png` |
| `control-press` | 0.0 | `out/control-press/media/score-vs-hits.png` |
| `control-rest` | 0.75 | `out/control-rest/media/score-vs-hits.png` |
| `control-flail` | 0.0 | `out/control-flail/media/score-vs-hits.png` |
| `control-no_stop` | 0.0 | `out/control-no_stop/media/score-vs-hits.png` |
| `dream` (Dream-1) | 0.0 | `out/dream/media/score-vs-hits.png` |

For a real trial, the drum module's MIDI recording is scored the same way: `python rdb_drum_groove/judge_midi.py
<file.mid> --events events.json --plot --source real`.

## Real trials

For a real trial, fill in events.json from the trial video: for each of the 8 pairs, whether both sticks came down; the
start time of any stick resting on a pad for 0.5 s or more; any gripper, arm or dropped-stick strike; whether the end
was clean. Then run python judge_midi.py <file.mid> --events events.json --plot. (Here:
`python rdb_drum_groove/judge_midi.py <file.mid> --events events.json --plot --source real`; the keys are in the
module's docstring.) Record the module's note numbers (a `--note-map` JSON if they differ from General MIDI: 42 hi-hat,
38 snare, 40 snare rim, 43 tom) and its CUR setting with the trial.

The tom pad has one zone, so on the real kit a stick landing on its rim still sends a tom note; the simulation
therefore counts a stick on the tom's rim as a tom hit. Only the snare's rim, a separate zone with its own note, counts
as a wrong hit. The hi-hat pad is treated as one zone too.

## Recorded run (scripted demonstration)

`out/scripted-1/` (and `validation.json`): score 1.0 in 305 of 1,200 steps, ended by the scorer once the end had been clean for 1 s (`success`), 272 s from start-up to the result. The left stick played the 31 hi-hat notes and the right stick the 8 snare notes and the tom's two: all 41 notes on a fitted 75.00 bpm grid, RMS timing error 3.17 ms (largest 5.2 ms), all 8 pairs together (largest spread 8.3 ms), the hi-hat's numbers at velocity 90.5 and its ands at 51.7 on average, the snare at 89.5; no extra or wrong hits, no gripper or arm on a pad, no rests; both sticks held clear (the end). Over the trial the left stick turned 5.3 degrees in the finger pads' plane and slid 4.9 mm along its axis, the right stick -5.7 degrees and 0.6 mm; both stayed held (about 34 N on each finger pad). `out/scripted-1/demo.mp4` is the combined overhead and side video with the drum track (`media/sync.json`: both streams start at 0, cross-correlation offset 0.0 ms). `out/scripted-2/` is the same code run again: score 1.0, scorer trace, `hits.mid` and `events.json` byte-identical.

## Dream-1 run

`out/dream/` (and `validation.json`, `runs.dream`): `dreamlabs/dream-1-beta` through `inspect-robots-dream` (commit
b5e936a2) with `dreamlabs-client` 1.0.14, on the final code (its hashes equal the recorded run's) with the same scene,
instruction, scorer, cameras and 20 Hz control as the demonstration. Score 0.0 at the 1,200-step limit, 49 action
requests (cap 120). Until its first reply arrived (step 33) the runtime held every joint, the grippers included, at its
measured position, so the sticks sat loosely in the fingers (under 1 N); nothing touched a pad. Its first reply closed
both grippers on the sticks (27 to 33 N); from about step 49 it opened both grippers wide, and both sticks fell onto
their pads. The right gripper took the right stick again twice and touched the snare with it 21 times, none faster
than 0.19 m/s, so none was a hit (a hit needs 0.5 m/s); the left gripper held the left stick again only for a moment
(near 13.8 s), and the left stick ended on the table. No hit was counted,
no gripper or arm touched a pad, and no tempo could be fitted: 0 of 41 notes. `out/dream/model_run.mp4` is the combined
overhead and side video with the drum track made by `tools/soundtrack.py` (silent here: no hits), and
`out/dream/media/score-vs-hits.png` its plot; the images Dream-1 received are in `dream-input-rgb.zip` and its replies
in `dream-action-trace.jsonl`.

## Controls

| Run | Variant | Expected | Native score | Steps | What the scorer saw |
|---|---|---|---|---|---|
| `control-flat` | flat (must run) | 0.9 | 0.9 | 400 | 41/41 notes, 8/8 pairs, E 0, 75.0 bpm, RMS 2 ms, accent rule fails |
| `control-tempo82` | tempo82 | 0.75 | 0.75 | 400 | 41/41 notes, 8/8 pairs, E 0, 82.0 bpm, RMS 15 ms |
| `control-no_fill` | no_fill | 0.75 | 0.75 | 400 | 37/41 notes, 7/8 pairs, E 0, 75.0 bpm, RMS 3 ms |
| `control-flam_bar3` | flam_bar3 | 0.5 | 0.5 | 400 | 41/41 notes, 6/8 pairs, E 0, 74.8 bpm, RMS 16 ms |
| `control-gripper` | gripper | 0.5 | 0.5 | 420 | 41/41 notes, 8/8 pairs, E 1, 1 gripper strike, 75.0 bpm, RMS 3 ms |
| `control-one_arm` | one_arm (must run) | 0.25 | 0.25 | 400 | 31/41 notes, 0/8 pairs, E 0, 75.0 bpm, RMS 2 ms, accent rule fails |
| `control-air` | air (must run) | 0.0 | 0.0 | 400 | 0/41 notes, 0/8 pairs, E 0, no tempo |
| `control-press` | press (must run) | 0.0 | 0.0 | 400 | 0/41 notes, 0/8 pairs, E 0, 2 rests, no tempo |
| `control-rest` | rest | 0.75 | 0.75 | 400 | 41/41 notes, 8/8 pairs, E 0, 8 rests, 75.0 bpm, RMS 3 ms |
| `control-flail` | flail (must run) | 0.0 | 0.0 | 400 | 29/41 notes, 1/8 pairs, E 90, 83.5 bpm, RMS 28 ms, accent rule fails |
| `control-no_stop` | no_stop | 0.0 | 0.0 | 480 | 41/41 notes, 8/8 pairs, E 10, 75.0 bpm, RMS 3 ms |

Each control's `RESULT.json`, plot, MIDI file, events and combined video with sound are in its folder; all ran natively, under a step cap below the 1,200-step run limit (`run.py --max-steps`): each outcome is settled once the demonstration has stopped.

## Files

The run folders under `out/` are not in every copy of this package; `validation.json` records each run's numbers
and file hashes, and the commands above regenerate them.

- `rdb_drum_groove/`: `layout.py` (every dimension and rule, with its source), `fixed_scene.py` (frames, start pose,
  instruction), `kit.usda` (the scene, from `tools/build_assets.py`), `adapter.py` (the scene in the station, the
  240 Hz trigger, strikes, rests, the end condition), `hits.py` (the trigger), `midi.py` (MIDI files),
  `judge_midi.py` (the scorer, for real and simulated trials), `plot.py` (the score-vs-hits plot), `grading.py` (the
  Inspect Robots scorer), `poses.py` and `scripted.py` (the demonstration and its controls), `recording.py`,
  `run.py`, `dream_audit.py`.
- `tools/`: `build_assets.py`, `reach_scan.py` (the layout's reach check), `soundtrack.py` (sound for the videos),
  `make_validation.py`.
- `tests/`: see TECHNICAL.md, "Tests and dry runs".

## Third-party assets and licenses

Bundled here, and the task authors' own work under the MIT license in `LICENSE`: the code; the scene file `kit.usda`,
whose pads, sticks, rack and hardware are convex hulls, capsules and boxes generated by `tools/build_assets.py` at the
real parts' published sizes; and the drum sounds in the videos, synthesized by `tools/soundtrack.py` (no samples or
recordings). No manufacturer CAD, meshes, textures, images or sounds are used; the product names (Alesis Nitro Max,
Vic Firth American Classic 5A) identify the parts whose sizes were used.

Third-party assets used at run time, none of them bundled here:

| Asset | Used for | Source | License |
|---|---|---|---|
| YAM arm model: URDF, link and gripper meshes (STL), collision meshes and the USD converted from them | both robot arms | `yam_arena/assets/yam/` in the official YAM runtime, [kairosiann/yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) at commit bd62771e, taken from I2RT's [i2rt](https://github.com/i2rt-robotics/i2rt) repository at commit 120c3c81 | MIT (I2RT Robotics; `LICENSE.i2rt` in that folder) |
| Black and white cover parts of the arms' links 2 and 3 | the arms' appearance | I2RT's link 2 and 3 meshes split into two parts, from the dreamlabs 1.0.23 package (`yam_arena/assets/yam/dream/`) | MIT (I2RT Robotics, as above) |
| Station: table, top-camera stand and RealSense D435 camera body (boxes), dome light | the room around the task | `yam_arena`'s calibrated station (`station.py`, `scene.py`) in the same runtime, built from primitive shapes and colours in code | the runtime's README states it has no license yet; nothing from it is copied into this folder |
| Overhead camera (RealSense D435 lens) and the two wrist cameras (RealSense D405 lens) | the policy's images and the overhead video | pinhole cameras defined in code by `yam_arena`'s tabletop rig (`rig.py`, `embodiment.py`); no camera meshes | as for the station |
| Side camera | the side video only, never a policy input | defined in this task's `recording.py` | MIT (this task) |
| Ground plane (`default_ground_plane.usda`) | the floor under the table | NVIDIA's Isaac Lab asset server, loaded at run time by Isaac Lab's `GroundPlaneCfg` | NVIDIA content, used under the Isaac Sim 6.1 license |
| Isaac Sim 6.1 | simulator and renderer | NVIDIA | NVIDIA proprietary software license |
| Isaac Lab 17.0.2 and Isaac Lab-Arena 0.3.1 | simulation framework | NVIDIA ([IsaacLab](https://github.com/isaac-sim/IsaacLab), [IsaacLab-Arena](https://github.com/isaac-sim/IsaacLab-Arena)) | BSD-3-Clause; Apache 2.0 |
| Inspect Robots 1.0.0, inspect-robots-yam 0.38.0, inspect-robots-dream 0.1.0 | trial runner, YAM embodiment, Dream-1 policy client | PyPI and [inspect-robots-dream](https://github.com/kairosiann/inspect-robots-dream) | MIT |
| DejaVu Sans font | text in the score-against-hits plots and the video captions | bundled with matplotlib | DejaVu Fonts license (a free license derived from the Bitstream Vera Fonts license) |
