# Count 3 drops into a test tube, cork it, then count 4 shakes, from memory

This task tests counting and memory, twice. Other pipette tasks show their result in a colour or a level. Here nothing in view records either count: the tube looks the same after 0, 1, 2 or 3 drops and after any number of shakes, so the arms must keep count of their own actions and stop exactly on each number. The left arm lifts a glass pipette out of its holder, a lab stand, by its rubber bulb, adds 3 drops to a test tube in an open rack and stands the pipette back. The right arm then puts a cork in the tube, lifts it, shakes it from side to side with its wrist, like a bell, 4 times, and stands it back. The arms work in turn. A fourth drop, a drop outside the tube, a dropped pipette or tube, or a lost cork scores 0 and does not recover. A fifth shake stops the score at 0.7, the same as stopping short.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source <ARENA_DIR>/yam.env`), as on the Brev launchable.
- `dropkit.py`, `usdkit.py` and `textures/` are in this folder. They build the pipette, tube, cork, rack and holder: importing `dropkit` hooks `yam_arena.scene` without modifying it, so the parts, declared as subclasses of `Box`, spawn from USD files the kit writes at start-up.
- Importing `dropkit` also wraps `yam_arena.demo.grip_rotation` so that the scripted demo can tilt its grip, which a straight-down plan cannot do. Only the scripted policy's `ArmPlan` calls that function, so this changes nothing in a model's run, and any waypoint without a tilt passes through unchanged.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 2400 \
    --instruction "Add exactly 3 drops of the chemical in the pipette to the test tube. The chemical is already in the pipette. Stand the pipette back in its holder. Then put the cork in the tube, lift the tube out of the rack, shake it from side to side with your wrist, like a bell, exactly 4 times, stand it back in the rack and let go." \
    --store-frames --log-dir out
```

## Scripted demo and failures

`bash run_scripted.sh 3 4 OUT_DIR` runs the demo: the left arm grips the bulb lightly, lifts the pipette out of its holder, holds its tip 5 mm above the tube's rim, squeezes and releases three times, and stands it back. The right arm then picks up the cork, sets it in the tube, grips the tube below it, lifts it 7.5 cm, tilts it 30 degrees each way four times, and stands it back. The plan is open loop, holds the counts as numbers, and needs `-E object_jitter=0`. The tilt is about the axis along the arm, from its base to the grip, because at these heights a tilt either way about another axis reaches a joint limit.

The same script runs the other outcomes, as `run_scripted.sh DROPS SHAKES OUT_DIR`, with `DROP_MODE` set for a fault: `hard` grips the bulb hard in the holder, `spill` also squeezes on the way to the tube, `dropped` lets go on the way, `early_lift` lifts the tube before corking it, and `no_tube` stops after the cork.

`inspect-robots video <log>.json` writes one MP4 per camera, and `python make_video.py <frames dir> out.mp4 --setup setup.png` joins them and writes the setup screenshot. `make_video.py` expects files named `scene-0-e0_<camera>.mp4`.

## Scoring

`--scorer task_score` prints the total. Per-step `info` is not saved, so the kit also writes the state, with each drop's and shake's time, to `drop_scores/score_*.json` (or `$LIFT_SCORE_DIR`) every 10 control steps and when the score changes. `dropkit.task_score` applies the scoring text in `task.json`, and `_Tracker` counts the drops and shakes and latches the failures, which do not recover.

Neither count shows in the scene. The tube and the pipette hold the same blue chemical, both levels are held fixed, the bulb returns to the same shape, a falling drop shows for under 0.25 s, and the tube ends upright in its rack. In the video, each squeeze shows the fingers closing on the bulb and a drop falling, and each shake shows the gripper and tube tilting each way, so a reviewer can count them.

## Runtime and seeds

The scene has no random parts with `-E object_jitter=0`, and the seed does not change it. All runs used seed 0.

A Dream-1 trial of 2,400 steps (120 s simulated) took 307 s of wall time on an RTX 5090, with `-P rows_per_request=20` and cameras at 320 x 176, and its slowest 30 steps took 6.5 s. At 20 steps per call, a session stays under the gateway's limit of about 8 s without a request. `bash run_dream.sh OUT_DIR "<instruction>" 20 2400` runs the trial and stops it if the session is lost.

The scripted demo of 1,317 steps (65.85 s simulated) took 220 s of wall time on an RTX 5090 at 640 x 360, and its slowest 30 steps took 9.4 s.

## Asset sources

- Pipette, tube, cork, rack and holder: our own meshes, generated by `dropkit.py` with `usdkit.py`. Sizes and shapes follow common lab products: a 3-inch glass pipette with a rubber bulb, a 20 x 100 mm glass test tube, a tapered cork, an open stainless steel rack and a lab stand with a ring. The tube is short because the arms cannot reach straight down above about 22 cm over the table.
- Textures, from ambientCG (CC0), resized to 512 px: Metal032 (`steel_normal`), Rubber004 (`rubber_normal`), Cork001 (`cork_*`, its colour map made less red).
- Arms, table, lights and cameras: the harness's station, unchanged.

## Simplifications

- No liquid is simulated: one full squeeze and release makes one drop, drawn falling from the tip.
- Both levels are held fixed in the glass. Three real drops raise the tube's level about 0.6 mm, under a pixel at 320 x 176. A real pipette's column would fall about 2.5 mm a drop, so a real trial hides it with a paper sleeve. In a shake the tube's surface tilts with the glass, which real liquid would not.
- The cork rests in the bore by its weight, with no press fit, so a tilt past about 60 degrees can lose it. A real pressed-in cork holds better. It is drawn as a cone and collides as two boxes: a plug in the bore and a top that lands on the rim.
- The bulb is two side pads on sliders, sprung at 3 N/mm with stops at 3.5 mm, under a drawn rubber shape. It collides as boxes.
- The tube collides as a round hull with a flat bottom below 6 cm, and as four flat walls above it, to grip and to hold the cork.
- A real squeeze can give more than one drop, so a real trial counts drops, not squeezes.
