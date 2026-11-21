# Tune a ukulele's G string to pitch with a clip-on tuner

> **Status: both trials have been run in Isaac Sim.** The scripted demonstration scores 1.0 (675 of 2,400 steps;
> repeated once with an identical scorer trace). Dream-1 (`dreamlabs/dream-1-beta`) scores 0.0: it used all 2,400
> steps, closed both grippers and never touched the ukulele. `validation.json` records both runs and the control.

The task instruction is: "The ukulele's G string has gone flat. Tune it to G4 with the clip-on tuner. Then let go of
the tuning button, pluck the G string once more to check it, and move both hands away from the ukulele." The fixed
scene holds a full-size Kala KA-15S soprano ukulele standing on its side as it is held to play, soundboard facing the
robot and headstock to its right, in two foam blocks with a strap; its G string, the top string, is 100 cents flat
(F#4), and a Snark ST-2 clip-on tuner sits on the headstock tip. The G string's tuning button sticks straight up from
the headstock's upper edge; it is the one of the two buttons there nearer the nut, beside the C string's.

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
python tools/dryrun_demo.py                                   # kinematic check of reach, clearances and step count, no simulator
python -m rdb_ukulele_tuning.run scripted --out out/scripted  # the scripted demonstration (add --live to watch it)
python -m rdb_ukulele_tuning.run dream --out out/dream        # one Dream-1 trial
```

Configure the Dream Labs client separately (`~/.dreamlabs/credentials`); no credentials are included. Both commands
use the same fixed layout, seed 0, zero object jitter, 20 Hz control (240 Hz physics) and the scorer
`ukulele_tuning_score`. The limit is 2,400 actions at 20 Hz (120 simulated seconds). The scene is fixed, so there are
no other seeds. `run.py scripted` exits 0 only if the demonstration scores 1.0. `--live` shows the Isaac Sim window
and does not change the trial.

Runtime per trial, measured for the scripted trial on one NVIDIA A10 (24 GB), headless, with the GPU shared with two
other Isaac Sim processes: 489.5 s from start-up to the result (675 steps). The Dream-1 trial, which used all 2,400
steps and waited on 73 requests to the Dream-1 service, took 725.9 s on the same machine, with the GPU shared with one
other Isaac Sim process for part of it. Allow about 4 more minutes for the first start of Isaac Sim on a new machine.

Each command writes, in its `--out` folder: `RESULT.json` (read `score`; also the steps, wall time, the reason the
trial ended, file hashes, the scorer's final state and diagnostics), `media/setup.png` (the first overhead frame),
`media/top_cam.mp4` (the overhead camera, 640 x 360), `media/side_cam.mp4` (a review camera, 1280 x 704, never given
to a policy), `media/end_top_cam.png`, `media/end_side_cam.png` and `media/grader-trace.jsonl` (the scorer's state,
the pluck count and what the tuner showed, every step), plus Inspect Robots' own log. The Dream-1 trial also writes
`dream-input-rgb.zip`, `dream-action-trace.jsonl` and `DREAM_USAGE.json`. A trial that fails still writes
`RESULT.json` with the error and how far it got. Videos are recorded at 10 frames a second (every second step).

Dream-1 receives 320 x 176 overhead and two wrist RGB images, plus robot joint and gripper positions; nothing else
(no pitch, no tuner state, no side camera). The scripted reference may read simulator state (button angles, computed
pitches, the string's displacement, the pluck count); it commands the robot and moves nothing in the scene itself.
Uploaded videos should combine the complete synchronized overhead and side recordings of one trial, as the recorded
run's `demo.mp4` does:
`ffmpeg -i media/top_cam.mp4 -i media/side_cam.mp4 -filter_complex "[0:v]scale=1280:704:flags=neighbor[t];[t][1:v]hstack=inputs=2" -c:v libx264 -pix_fmt yuv420p -movflags +faststart demo.mp4`.

## Recorded run (scripted demonstration)

`out/scripted-1/` (and `validation.json`): score 1.0 in 675 steps, ended by the scorer once 1.0 had held for 1 s
(`success`). The left hand plucked the G string three times, each counted on the first try (the string pushed 5.2 to
5.3 mm): the tuner read F#, centred and green (G at -100 cents); the right hand, pinching the G button (the one nearer
the nut) from above, turned it 455 degrees by rolling its wrist, letting go and regripping three times, and corrected
it once from the button's measured angle; the second pluck read G at +0.04 cents; the right hand let go (the button
moved 0.26 degrees) and the third pluck, the final check, read G at +0.05 cents. Both hands then went back to the
ready pose (84.3 and 149.4 mm from the ukulele). The C, E and A strings did not move, and no arm touched the
instrument except on the G button and the G string. `out/scripted-1/demo.mp4` is the combined overhead and side video
(2560 x 704, 33.9 s). `out/control-wrong_button/` is the control that turns the C string's button instead and scores 0
(see TECHNICAL.md, "Native runs").

## Scoring and real-world checks

`task.json` gives the full rubric. After the trial the judge plucks the G string once over the soundhole and reads a
phone tuner app that shows cents. Within 5 cents of G4: 1.0 if the trial video shows the robot plucking the G string
after its last turn of the G button and, in the video's last frame, both grippers' centre points are at least 50 mm
from the ukulele and tuner; otherwise 0.75. Over 5 and up to 15 cents off: 0.75; up to 30: 0.5; up to 50: 0.25; over
50 either way: 0. Any of the C, E or A strings more than 15 cents from its start scores 0 (turning the C string's
button, which sits beside the G string's, fails), as does the ukulele leaving its blocks or the strap coming undone,
the tuner being knocked off or a string breaking. In simulation the same bands apply to each string's computed pitch
at the last step, the G button counts as turned after a pluck once it is more than the gear's 4-degree free play from
its angle at that pluck, and the trial ends at the run limit or once 1.0 has held for 1 s.

## Setup and reset

A Kala KA-15S (open-gear tuners with pearloid buttons) with Aquila Super Nylgut strings in standard re-entrant
tuning, a Snark ST-2 in vibration mode at A4 = 440 Hz, two black closed-cell foam blocks (EVA or similar) (one under the lower bout, one under the
neck at the 3rd fret, each with a cheek) and one 25 mm hook-and-loop strap, placed as `task.json` says. Reset by
retuning C, E and A if they drifted and bringing the G string to F#4 from below (so the gear's free play is always
taken up the same way); nothing is consumed. A fresh simulation run resets the scene.

## Physics, sources and license

No sound is simulated. Contacts, the buttons' drag, the string's spring across the strings and the needle use PhysX;
each string's pitch is computed from its button's angle (0.22 cents per degree, about 4 degrees of gear free play), a
pluck is a stated rule (the string pushed 2 mm across the strings and springing back) and the tuner's display, a
stand-in for the ST-2's, is driven from the computed pitch for 3 s after each pluck. The ukulele's published sizes
(Kala's specification) are used as published; the cents per degree, turning direction, free play, drag and the other
dimensions are estimates to be measured on a real KA-15S, and each open gear is drawn as a box.
[TECHNICAL.md](TECHNICAL.md) lists every parameter, its source, how to measure it, the scoring rules, the native runs
and what has and has not been checked.

Code and procedural scene assets are MIT (`LICENSE`); no manufacturer CAD, textures or images are bundled. The robot
and stock YAM station are external dependencies. Run `python -m pytest -q tests` (or
`python -m unittest discover -s tests`) for the offline checks; they do not replace the native trial.

## Third-party assets and licenses

Bundled here, and the task authors' own work under the MIT license in `LICENSE`: the code and the scene file
`rdb_ukulele_tuning/ukulele.usda`, whose ukulele, tuner, foam blocks and strap are boxes, capsules, cylinders, convex
hulls and simple meshes generated by `tools/build_assets.py` at the real parts' sizes; the tuner's display is drawn by
the task's own code. No manufacturer
CAD, meshes, textures or images are used; the product names (Kala KA-15S, Snark ST-2) identify the parts whose sizes
were used.

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
