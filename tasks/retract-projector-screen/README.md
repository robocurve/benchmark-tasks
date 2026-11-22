# Retract a manual projector screen

**Instruction** “Put the projector screen away. Guide it back into the case.”

Putting away a classroom or meeting-room screen requires controlling the weighted bar until it docks. Pull the factory tab down to release the catch, then reverse direction and guide the bar upward as the reel pulls it toward the case. The robot has to maintain control through the transition from pulling against spring tension to restraining spring-driven motion. Releasing the tab can leave the screen fully stowed while still slamming the bar into the case. The scorer checks the return motion as well as the final position. [Manufacturer operating instructions](https://elitescreens.com/wp-content/uploads/userguide_ManualSeries.pdf)

## Physical setup and reset

- Use an Elite Screens Manual Series M84UWH, or the equivalent M80UWH in the same series with guided return. Use the original pull tab and mounting brackets. The M84UWH has a 2,014 mm steel case, 2,010 mm weight bar and 1,936 mm wide coated screen material. The nominal case section is 69 × 82 mm. [Manufacturer dimensions](https://www.elitescreens.com/wp-content/uploads/manual_dims_mm.pdf)
- Attach both ends to a structural wall or an anchored steel frame using the supplied brackets as specified by the manufacturer. Run the arms from the front with nothing behind the screen path. Mount the case so that the factory tab is 1,132 mm above floor when stowed. From the midpoint between the YAM base centres, the cloth plane is 498 mm forward and the tab is 80 mm toward the right arm. The YAM station tabletop is 747.5 mm above floor. Use the short-extension stop so the weight bar and the pull tab remain above the tabletop throughout the release stroke.
- Start at the nearest stable latch stop to 275 mm below the stowed bar position. Mark the actual starting position on a stationary vertical ruler beside the bar and record it for all trials. Check that approximately 53 mm of additional downward travel clears the catch, with at least 30 mm between the gripper path and the table. Follow the model's normal release motion. [Manufacturer operating instructions](https://elitescreens.com/wp-content/uploads/userguide_ManualSeries.pdf)
- With the robot disabled, reset by guiding the factory tab down to the starting mark, letting the latch take the load, and checking that the bar remains stationary for one second. Keep people clear of the moving bar during trials.

## Scoring

Use a fixed front/side video at 20 fps or more, with the vertical ruler and the whole weight bar visible. Measure travel from the stowed position. Score zero if the catch was never released or, after release, the bar travels upward faster than 220 mm/s (more than 22 mm over 0.10 seconds). Otherwise score 1.0 if the bar rests within 12 mm of stowed at the end and both gripper tips are at least 50 mm in front of the tab. Score 0.5 if the bar remains between 110 and 170 mm below stowed. Both scores require at most 5 mm of bar movement over the last 0.20 seconds. All other outcomes score zero. Use the same ruler and camera for each trial.

## Run

Activate the YAM Arena environment and run from this directory.

```bash
python task.py
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E headless=true \
  --instruction 'Put the projector screen away. Guide it back into the case.' \
  --scorer task_score --max-steps 1200 --seed 0
```

The control limit is 1,200 steps at 20 Hz (60 seconds). The scene uses a fixed initial pose with no jitter. The scripted arm moves the original tab by physical contact.

## Fidelity and attribution

The screen housing, weight bar, tab and fabric graphic adapt NVIDIA USDCraft-10K asset `ds_R329_05`, *Narrow Wall Screen*, licensed Apache-2.0. The adaptation widens the vendor asset to the scale of the commercial screen and relocates the pull tab below the bar. A fixed header hides the stored portion of the fabric graphic. The weight bar is a rigid sliding body. An approximate passive axial spring and latch model reacts to its measured motion independently of the arm policy. Actual internal pawl geometry, fabric sway and spool motion are omitted. [NVIDIA asset catalog and license](https://huggingface.co/datasets/nvidia/USDCraft-10K)
