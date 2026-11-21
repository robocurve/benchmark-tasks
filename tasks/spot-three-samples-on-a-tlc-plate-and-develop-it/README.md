# Spot three samples on a TLC plate and stand it in the developing tank

> **Status: the scripted demonstration scores 1.0** (3,033 of 4,800 steps in Isaac Sim, run twice with the final code, byte-identical scorer traces). **Dream-1 (`dreamlabs/dream-1-beta`) scores 0.0** on the final code: it lifted capillary 1 from its rack, the capillary slipped from the grip and tipped over, and the trial was voided at step 4,770 of 4,800 (nothing else was touched; no spot was made).

The task instruction is: "Counting from the robot's left, the first spotter goes with the first vial and the first mark on the baseline, the second with the second, the third with the third. Fill each spotter by dipping its tip in its vial, touch it once, briefly and lightly, to its mark, and return it to the rack. Then stand the plate in the developing tank, spotted face out, bottom edge in the solvent and baseline above it, leaning back against the back wall, and put the lid on."
The fixed scene holds a glass-backed TLC plate (25 x 75 mm) lying silica up on a narrow aluminium bar, a pencil
baseline and three ticks on it; three empty glass capillary spotters standing in a foam rack; three vials of dyed
water (red, blue, green) in a block; and the developing chamber, a Sigma-Aldrich Z146226 rectangular glass TLC
developing tank with 5 mm of water, its flat glass lid lying on the bench.

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
python tools/dryrun_demo.py                                  # kinematic check of reach, flow and step count, no simulator
python -m rdb_tlc_spotting.run scripted --out out/scripted   # the scripted demonstration (add --live to watch it)
python -m rdb_tlc_spotting.run dream --out out/dream         # one Dream-1 trial
```

Configure the Dream Labs client separately (`~/.dreamlabs/credentials`); no credentials are included. Both commands
use the same fixed layout, seed 0, zero object jitter, 20 Hz control (240 Hz physics) and the scorer
`tlc_spotting_score`. The limit is 4,800 actions at 20 Hz (240 simulated seconds). `run.py scripted` exits 0 only if
the demonstration scores 1.0 (the probe and control variants below always exit 0). `--live` shows the Isaac Sim window and does not change the trial. For probes,
`RDB_TLC_DEMO` selects a part or a control (`plate_only`, `spot_only`, `one_lane`, `press_hard`, `no_dip`) and
`RDB_TLC_STEPS` caps the steps; the recorded run uses neither.

Runtime per trial, measured for the scripted trial on one NVIDIA A10 (24 GB) with the window shown, the GPU shared with one other simulation process: 1428 s from start-up to the result (3,033 steps). The Dream-1 trial (4,770 steps, 177 requests to the Dream-1 service) took 1,333 s on the same kind of machine, headless, the GPU not shared. Allow about 4 more minutes for the first start of Isaac Sim on a new machine.

Each command writes, in its `--out` folder: `RESULT.json` (read `score`; also the steps, wall time, the reason the
trial ended, file hashes, the scorer's final state and diagnostics), `media/setup.png` (the first overhead frame),
`media/top_cam.mp4` (the overhead camera, 640 x 360), `media/side_cam.mp4` (a review camera, 1280 x 704, never given
to a policy), `media/end_top_cam.png`, `media/end_side_cam.png` and `media/grader-trace.jsonl` (the scorer's state
every step: loads, spots, gouges, the plate in the tank, the lid), plus Inspect Robots' own log. The Dream-1
trial also writes `dream-input-rgb.zip`, `dream-action-trace.jsonl` and `DREAM_USAGE.json`. Videos are recorded at 10
frames a second (every second step). A broken spotter is not drawn: the trial stops at the zero, so the video ends
with the spotter looking whole; `RESULT.json` (`final_grade.void`) and the scorer trace give the force. The `out/`
folders named here are not in every copy of this package (their videos and logs are large): `validation.json` gives
each recorded run's result, scorer summary and code hashes, and the uploaded `demo.mp4` and `model_run.mp4` are the
combined videos of `out/scripted-11` and `out/dream`.

Dream-1 receives 320 x 176 overhead and two wrist RGB images, plus robot joint and gripper positions; nothing else.
The scripted reference may read simulator state (the bodies' poses, the spotters' contact forces, what the rules
recorded); it commands the robot and moves nothing in the scene itself. Uploaded videos should combine the complete
synchronized overhead and side recordings of one trial, as the recorded run's `demo.mp4` does:
`ffmpeg -i media/top_cam.mp4 -i media/side_cam.mp4 -filter_complex "[0:v]scale=1280:704:flags=neighbor[t];[t][1:v]hstack=inputs=2" -c:v libx264 -pix_fmt yuv420p -movflags +faststart demo.mp4`.
The task's setup image is that video's first frame, overhead and side views together (`ffmpeg -i demo.mp4
-frames:v 1 setup.png`; the recorded run's `setup.png`), since the 640 x 360 overhead frame alone shows the spotters,
vials and tank only as small grey shapes.

## Recorded run (scripted demonstration)

`out/scripted-11/` (and `validation.json`): score 1.0 in 3,033 steps, ended by `success`. The left hand filled each spotter in its own vial and dabbed it once on its tick (lane 1: 2.57 mm, 0.26 mm off its tick; lane 2: 2.57 mm, 0.39 mm off its tick; lane 3: 2.57 mm, 0.9 mm off its tick; no bare glass) and stood each back in its rack hole. The right hand pinched the plate's top end, lifted it so it swung down, stood its bottom edge on the bench behind the bar's far end and moved toward the robot until the bar's end face had turned it near upright (21.4 to 3.4 degrees off), carried it into the tank and stood it 36.5 mm in front of the back wall, and let go: it settled leaning 28.17 degrees on the back wall, spotted face out, its baseline 4.88 mm above the water. It then set the lid on the tank (1.43 mm off centre, 0.0 degrees from level, 0.3 degrees askew) and both hands went back. `out/scripted-11/demo.mp4` is the combined overhead and side video and `out/scripted-11/setup.png` its first frame (the task's setup image). `out/scripted-12/` is the same code run again: score 1.0 in 3,033 steps, its scorer trace byte-identical. `out/scripted-10/`, `out/scripted-9/`, `out/scripted-8/` ran the same demonstration before the changes of 20 November (the instruction did not yet state the left-to-right pairing, a spotter past 60 degrees counted as dropped even in a hand, a plate lying face up was a zero, and `adapter.py` and `tlc.py` had two stale descriptions); the scene, physics and demonstration did not change, and their scorer traces are byte-identical to this one. Control `out/control-press_hard-2/` (`press_hard`, capped at 2,000 steps, after all three touches): lane 1: a 2.57 mm spot 0.26 mm off its tick (3/3); lane 2: a 2.05 mm spot 0.94 mm off its tick, gouged (2/3); lane 3: a 2.57 mm spot 0.91 mm off its tick (3/3); plate checks 0/3; 8/12 at the cap, where the full demonstration had 9/12. Control `out/control-no_dip-2/` (`no_dip`, capped at 2,000 steps, after all three touches): lane 1: no spot (0/3); lane 2: no spot (0/3); lane 3: no spot (0/3); plate checks 0/3; 0/12 at the cap, where the full demonstration had 9/12.

## Dream-1 run

`out/dream/` (and `validation.json`, `runs.dream`): `dreamlabs/dream-1-beta` through `inspect-robots-dream` (commit
b5e936a2) with `dreamlabs-client` 1.0.14, on the final code (its hashes equal the recorded run's) with the same scene,
instruction, scorer, cameras and 20 Hz control as the demonstration. Score 0.0: for about 4,700 steps it moved both arms
over the bench without touching anything; then the left gripper lifted capillary 1 from its rack, the pinch slipped and
the capillary tipped over with no gripper holding it, so the trial was voided at step 4,770 of 4,800 (177 action
requests, cap 300). No capillary was filled and no spot was made; the plate, the lid and the tank were never touched.
`out/dream/model_run.mp4` is the combined overhead and side video (2560 x 704, 238.6 s), made with the same ffmpeg
command as `demo.mp4`; the images Dream-1 received are in `dream-input-rgb.zip` and its replies in
`dream-action-trace.jsonl`. An earlier Dream-1 trial on the code before the changes of 20 November
(`out/dream-oldcode/`) also scored 0.0 (voided at step 2,022, capillary 1 broken); it is kept for the record only.

## Scoring and real-world checks

`task.json` gives the full rubric: twelve checks, one twelfth each. In a real trial a phone on a stand outside the
arms' reach films the plate on its bar throughout, zoomed so the plate fills the frame; it is not a robot input.
(a) and (b) are read from its last frame before the plate is lifted, against the 6 mm tick spacing, because the spots
start to move once the plate is in the water; (c) with a 10x loupe on the dried plate after the trial; (d) to (f)
from a side photo of the tank with a ruler and a protractor; the rest from the trial video. Per lane: (a) one spot of
its own sample covers the tick; (b) the spot is at most 3 mm across (half the tick spacing); (c) no bare glass longer
than 1 mm near it. Plate: (d) standing in the tank with its bottom edge in the water and its baseline at least 3 mm
above it; (e) leaning on the tank's back wall, spotted face away from it, at most 35 degrees from vertical (with 5 mm
of water the baseline stays 3 mm clear up to 37 degrees); (f) the lid on, centred, level and square. 1.0 also needs
the spotters back in the rack and the hands clear. 0 if a spotter breaks or is dropped (more than 60 degrees from
upright with no gripper holding it, or its tip on the bench outside the rack and vials), the plate is dropped (face
down on the bench, or off the bench; lying face up is not a zero), the baseline goes under the water, or the tank is
tipped past 10 degrees or pushed more than 5 mm off its tape outline. The simulated trial ends 1 s after 1.0, at a
zero or at the run limit; a real trial ends when both arms are back and still for 1 s, at a zero or at the run limit.

## Setup and reset

As `task.json` says: MilliporeSigma 1.15327 glass TLC plates; Warner Instruments 30-0053 capillaries (1.5 x 0.86 x
100 mm); three 4 mL vials with 1 mL of water and food colouring (non-hazardous); a Sigma-Aldrich Z146226 rectangular
TLC developing tank with its lid and 44 mL of water (5 mm); a 3 in length of 1/2 in square aluminium bar; foam blocks
drilled for the rack (30 x 110 x 25 mm, 5 mm holes) and the vials (30 x 120 x 20 mm, 16 mm holes); nitrile finger
cots over the gripper pads; tape to outline the tank's footprint; a phone on a stand to film the plate. Each trial
uses a new plate (pencil the baseline and ticks through a template, a card with the baseline and the three tick
positions cut out); rinse and dry the spotters and stand them back; top up the vials and the tank and stand the tank
back inside its outline; lay the lid back on its mark; check the finger cots. Wipe up spilt water or dye with paper
towels and dry the bench and the robot; pick up broken glass with tweezers or a brush into a broken-glass box; wear
safety glasses at the bench. A fresh simulation run resets the scene.

## Physics, sources and license

No liquid or silica is simulated. The spotters, plate, tank and lid are rigid bodies moved only by contact;
filling, spotting, gouging and breaking are stated rules on the measured contact forces and poses (`tlc.py`), with
estimated thresholds; [TECHNICAL.md](TECHNICAL.md) lists every parameter, how to measure it, the scoring rules, the
demonstration, the native runs and what has and has not been checked.

Code and procedural scene assets are MIT (`LICENSE`); no manufacturer CAD, textures or images are bundled. The robot
and stock YAM station are external dependencies. Run `python3 -I -m unittest discover -s tests` for the offline
checks; they do not replace the native trial.

## Third-party assets and licenses

Bundled here, and the task authors' own work under the MIT license in `LICENSE`: the code and the scene file
`rdb_tlc_spotting/tlc.usda`, whose plate, bar, capillaries, rack, vials, vial block, tank and lid are boxes, convex
hulls and drawn surfaces of revolution generated by `tools/build_assets.py` at the real parts' sizes; the spots are drawn by the task's own code
(`skin.py`). No manufacturer CAD, meshes, textures or images are used; the product names (Merck 1.15327,
MCS/Warner 30-0053, Sigma-Aldrich Z146226) identify the parts whose sizes were used.

Third-party assets used at run time, none of them bundled here:

| Asset | Used for | Source | License |
|---|---|---|---|
| YAM arm model: URDF, link and gripper meshes (STL), collision meshes and the USD converted from them | both robot arms | `yam_arena/assets/yam/` in the official YAM runtime, [kairosiann/yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) at commit bd62771e, taken from I2RT's [i2rt](https://github.com/i2rt-robotics/i2rt) repository at commit 120c3c81 | MIT (I2RT Robotics; `LICENSE.i2rt` in that folder) |
| Black and white cover parts of the arms' links 2 and 3 | the arms' appearance | I2RT's link 2 and 3 meshes split into two parts, from the dreamlabs 1.0.23 package (`yam_arena/assets/yam/dream/`) | MIT (I2RT Robotics, as above) |
| Station: table, top-camera stand and RealSense D435 camera body (boxes), dome light | the room around the task | `yam_arena`'s calibrated station (`station.py`, `scene.py`) in the same runtime, built from primitive shapes and colours in code | the runtime's README states it has no license yet; nothing from it is copied into this folder |
| Overhead camera (RealSense D435 lens) and the two wrist cameras (RealSense D405 lens) | the policy's images and the overhead video | pinhole cameras defined in code by `yam_arena`'s tabletop rig (`rig.py`, `embodiment.py`); no camera meshes | as for the station |
| Side camera | the side video only, never a policy input | defined in this task's `recording.py` | MIT (this task) |
| Ground plane (`default_ground_plane.usda`) | the floor under the table | NVIDIA's Isaac Lab asset server, loaded at run time by Isaac Lab's `GroundPlaneCfg` | NVIDIA content, used under the Isaac Sim 6.1 license |
| `OmniGlass.mdl` material | the look of the glass parts (appearance only; no effect on physics or scoring) | shipped with Isaac Sim and referenced by name in `tlc.usda` | NVIDIA, under the Isaac Sim 6.1 license |
| Isaac Sim 6.1 | simulator and renderer | NVIDIA | NVIDIA proprietary software license |
| Isaac Lab 17.0.2 and Isaac Lab-Arena 0.3.1 | simulation framework | NVIDIA ([IsaacLab](https://github.com/isaac-sim/IsaacLab), [IsaacLab-Arena](https://github.com/isaac-sim/IsaacLab-Arena)) | BSD-3-Clause; Apache 2.0 |
| Inspect Robots 1.0.0, inspect-robots-yam 0.38.0, inspect-robots-dream 0.1.0 | trial runner, YAM embodiment, Dream-1 policy client | PyPI and [inspect-robots-dream](https://github.com/kairosiann/inspect-robots-dream) | MIT |
