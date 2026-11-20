# Start and zip a baby jacket

The separating pin starts fully outside its retaining box. The robot must align and insert it before zipping the jacket to its collar and letting go. The scene is fixed; seed 0 is used for both recordings.

## Recorded evidence

| Recording | Score | Actions | Simulated time |
| --- | --- | --- | --- |
| Dream-1 | 0.0 | 1,200 | 60.0 s |
| Scripted expert | 1.0 | 1,178 | 58.9 s |

The expert first seats the pin at 29.1 s and begins closing at 36.6 s. Final slider travel is 179.43 mm of 180 mm, the pin remains seated, and both gripper TCPs are clear. `results/` contains the scores, expert state trace and framework revision. Videos and the initial setup screenshot are uploaded to the task board and referenced by `task.json`'s media ID.

The expert sends arm/gripper joint targets using the YAM inverse kinematics utilities. It never sets the pin, slider or garment poses during the demonstration. Its diagonal grasp clears the orange attachment; its insertion servo estimates velocity from several position samples. The recordings share identical scene geometry and physics. The exploratory expert invocation allowed a larger action cap, but completed in 1,178 actions, below this task's 1,200-action limit. The submitted CLI enforces that limit.

## Reproduce a trial

Use the standard RoboCurve YAM Isaac Lab Arena launchable with cameras enabled. The recorded YAM Arena revision is `0059d02a1a91bc6834c1d0846022891df0c78730`. Required Python packages come from its supplied environment, including Isaac Lab Arena, `yam_arena`, Inspect Robots and its Dream policy plugin, NumPy, trimesh, Pillow, OpenCV and imageio-ffmpeg. FFmpeg is used for video export. The mesh and texture are included; the optional Triangle package is only needed to regenerate them.

Run each trial in a fresh Python process from this folder:

```bash
source /data/IsaacLab-Arena/yam.env

# Validate the parked-arm scene and save setup/loose-pin images.
python task.py --output /tmp/jacket-preview

# Record and score the full scripted demonstration, seed 0.
python task.py --expert --max-steps 1200 --output /tmp/jacket-expert

# Dream credentials must already be configured privately through the SDK.
# Run 1,200 actions and export the combined three-camera MP4.
bash run-dream.sh /tmp/jacket-dream
```

Use a new output directory for each recording. Pass `--live` to `task.py` to open the Isaac Sim viewport. The expert writes `scripted-demo.mp4`, `scripted-score.json` and `states.json`. Dream writes its Inspect Robots evaluation, score, action log and camera frames; `run-dream.sh` exports `dream.mp4`. Each recorded trial lasts at most one simulated minute. The successful expert took approximately 3.2 minutes wall time on an L40S including startup; startup and inference latency vary by machine.

## Physical dimensions and printed attachments

The jacket uses a woven polyester shell/lining with quilted polyester batting, approximately 240 mm long, 370 mm across sleeves and 8 mm thick. The one-way separating zipper has 180 mm of travel, molded acetal teeth, polyester tape and zinc-alloy slider/pin/box hardware.

The 10 × 2.5 × 2.5 mm pin and 10 × 7 × 4.5 mm box follow the 10 mm component pair listed in [PandaHall repair set FIND-PH0028-52](https://www.pandahall.com/p-4233884-48pcs-12-styles-clothing-accessories-zinc-alloy-zipper-repair-accessories-insert-box-and-pin-fix-retainer-mixed-color.html). This is a dimensional reference, not a claim that every repair component fits every zipper. The simulated 3.2 × 3.2 mm straight internal channel is an approximation. The starting sequence follows [YKK's insertion instructions](https://ykkamericas.com/wp-content/uploads/2021/10/ykk-zipper-instruction-manual-compressed.pdf): seat the pin through the slider into the box before closing.

Two custom PETG attachments are part of the task's specified physical setup:

- Orange slider pull: 18 × 16 × 24 mm block bonded to the metal slider, leaving its sliding path clear.
- Blue reinforced-tape grip: 14 × 10 × 6 mm assembled clip. Print two 14 × 10 × 2.8 mm plates. Sandwich the reinforced zipper tape in a 0.4 mm combined tape/adhesive gap and clamp to 6 mm finished thickness. Orient the 14 mm dimension along the zipper, with the clip center 18 mm collarward and 2 mm above the pin center.

Print at 0.2 mm layer height and 40% infill. Bond with J-B Weld PlasticBonder Tan (50133), cleaning and lightly sanding PETG/metal mating surfaces, mixing equal parts and holding at least one hour at room temperature. Keep adhesive clear of the insertion channel, teeth and slider. `fabrication/` includes millimeter-unit STL files and editable OpenSCAD geometry. The printed clip and its bond to reinforced tape are a proposed reproducible counterpart to the simulated grip; no physical hardware trial is claimed.

## Mechanism and scoring

The pin has three passive translation joints, no position drives, and fixed rotation. The slider stays within 2 mm of the hem until the measured pin offset is within 0.5 mm in each axis. Once the slider moves 6 mm after insertion, the pin is retained by tightening joint limits. The latch changes constraints without setting object poses. The jacket has deformable panels joined across the shoulders with separate lower hems and partially rigid zipper-tape rails.

Score zero until insertion. Once seated, score 0.25 + 0.70 × closed fraction, capped at 0.95 until the slider is within 3 mm of the top and both gripper TCPs are more than 40 mm from cloth vertices and grip centers; then award 1.0. An unseated pin scores zero. `zipper/scoring.py` implements the rubric.

The fixed pin rotation, rigid pin-to-grip link, simplified channel, approximate hardware masses/friction and animated teeth limit fidelity. Actual tape buckling near the clip, tooth interlocking and cloth snags are not modeled. Both enlarged solid grips represent the specified custom attachments, rather than standard zipper hardware.
