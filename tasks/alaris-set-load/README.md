# Load an IV set into an Alaris pump module: blue upper fitment into its recess first, then the safety clamp pressed home

A large-volume infusion pump module stands clamped to its pole with its door open. A primed set hangs from a holder beside it. The robot loads the set in the maker's order: it lowers the blue upper fitment into its recess at the top of the mechanism, then presses the safety clamp into its recess below the mechanism, without lifting the fitment back out or stretching the set, and leaves the set untwisted.

**Ability tested:** seating a soft line between two keyed seats. BD's order is given in the instruction; the difficulty is the seating, not the order, and a picture of where the parts go does not seat them. The fitment is taken from the holder by its fluted body, its tubing slid into the recess's slot from the front, and its collar lowered onto the ledge, where it "fits loosely" inside a low wall. The silicone pumping segment then hangs down the mechanism channel with the safety clamp at its end; the clamp is taken, squared to its recess and pressed home. The fitment must stay in its recess while the clamp goes in, and the clamp must go in square: a turned or tilted clamp does not enter the recess, a set pulled too far stretches the segment, and a clamp spun a full turn leaves the set twisted though it goes in square.

**Why it matters:** starting an IV infusion is daily work for registered nurses and licensed practical nurses (O*NET 29-1141.00: "Administer medications to patients and monitor patients for reactions or side effects."; 29-2061.00: "Administer prescribed medications or start intravenous fluids, noting times and amounts on patients' charts."), and loading the set is how a pump infusion starts. The steps are BD's own, from the BD Alaris System user manual (Pump Module Set Loading): "Hold blue upper fitment above fitment recess and lower into recess. The blue upper fitment fits loosely in the recess." "Ensure that tubing is not twisted or stretched." "Press the safety clamp into recess below mechanism." BD warns: "Ensure the upper fitment is not elevated above the fitment recess on the pump." The manual's next steps (pushing the tubing into the air-in-line detector and closing the door) are left out of this task.

**Hands:** one hand loads the set; the other arm is not needed and stays at rest. The score does not check which hand does what.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `cordkit.py` and `demokit.py` are in this folder. Importing `mechkit` and `cordkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Load the primed set into the pump module: blue upper fitment into its recess first, then press the safety clamp in, without lifting the fitment or stretching the set; leave the set untwisted, let go and move clear."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                        # full score, with a video
python demokit.py task.py OUT --no-record --mode none,fitonly     # the failure modes below
```

The demo is a scripted reference. It reads the poses of the fitment and the clamp, which the cameras see; a policy gets only the cameras. The right hand takes the fitment level by its fluted body, lifts it off the holder's fork and draws it out through the fork's open end, brings it in front of the recess, slides its tubing into the slot and lowers the collar onto the ledge, and lets go. It then takes the hanging safety clamp across its sides, squares it to its recess and presses it home past the recess's two sprung side pins, lets go and backs away. The grip holds the clamp's turn; a 5 mm lead-in at the recess's top and bottom squares its tilt as it goes in. Once the fitment is seated, the clamp hangs at its recess's mouth, stopped by the pins; the act is the press past them without lifting the loose fitment. An orange band on the back of the clamp's top goes under the recess's lip when the clamp is home, so the cameras show the press. The fitment's collar has flats that the recess's walls key square, so the segment hangs the clamp square below it.

## Building it for a real trial

- **Pump:** a BD Alaris PC unit with an Alaris 8100 pump module, clamped to an IV pole so that the module's foot is about 40 mm above the bench, its door open. The module is unpowered for the trial.
- **Set:** a BD Alaris pump module infusion set, primed with water, its upper tubing cut about 3 cm above the blue fitment and its lower tubing about 2 cm below the safety clamp, so that only the segment between them takes part.
- **Holder:** a clamp stand beside the pump with a fork 210 mm above the bench (two prongs 6 mm apart, open toward the robot) holding the fitment by its collar, the segment and clamp hanging below.

Calibrate before trials:

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Fitment, clamp and segment sizes and masses | as in `setup` (fitment and clamp 8 g each, segment 6 g) | **Not measured.** Calipers and a scale on the set |
| Recess sizes | ledge slot 6 mm, collar wall 3 mm; clamp recess 17 x 23 mm, 10 mm deep | **Not measured.** Calipers on the module |
| The clamp recess's clip | a sprung pin each side, 2.5 mm into the opening (800 N/m, up to 2 N each), then a 2 N catch once the clamp is home | **Not measured.** Push the clamp in and pull it out with a push-pull gauge and read both forces |
| Segment torsion | GJ 1e-4 N m^2, raised from silicone's about 1e-5 | **Not measured.** Twist a 164 mm length by a known torque and read the angle |

## Safety for a real trial

The pump is unpowered and the set is primed with water only: no needle, no drug. Keep the pole clamped and an emergency stop in reach.

## Scoring

See `scoring` in `task.json`, and `score`, `success`, `fit_seated` and `clamp_seated` in `task.py`. `python3 check_score.py` runs that scorer on nineteen hand-built states (the full result, each failure and each lazy route) with no simulator. In a real trial, by calipers, ruler, protractor and video:

- **0.2:** the blue fitment seated: its collar on the recess's ledge, centred in the recess within 2 mm, upright within 10 degrees.
- **0.6:** also the safety clamp pressed home (its back on the recess's back stop within 1.5 mm, read as the clamp body's face, beside the thumb pad, no more than 11.5 mm proud of the module's face with a caliper's depth rod; centred within 2 mm across and 3 mm up or down; square within 12 degrees), the fitment seated first, and from the start to the end: the fitment never more than 3 mm above its seat once seated (side-on video with a millimetre scale) and the segment never stretched (two ink marks 100 mm apart on it never more than 2 mm further apart, on video); and at the end the set untwisted (the clamp's net turn about the tubing, counted through whole turns from a line on its thumb pad on the video, within 30 degrees).
- **1.0:** also both grippers at least 5 cm from the fitment, the clamp and the segment, all of this held for 2 s.
- The score is read at the end of the run. A clamp pressed onto the recess's pins before the fitment is seated keeps the score at 0.2 at most, even if it is pressed home later. 0 if the fitment or the clamp falls off the table.

In the simulator a stretch is read as the gaps opened at three joints in a row of the segment's links (about 100 mm of it), more than 2 mm together. The recess's catch pushes a clamp that is home 2 N into it, and lets go of a clamp pulled back out.

## Results

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | The fitment seated (0.4 mm off its seat, upright), the clamp pressed past the side pins onto its back stop (0.4 mm, square within 2 degrees; the orange band on its top goes under the lip), the set's net twist 3 degrees, stretch 0.01 mm; let go; held 2 s | 1.0, at step 468 (23.4 s) |
| `half` | The clamp pushed only 2 mm into the recess's mouth and let go: the pins push it back and it stays 7 mm short of home | 0.2 |
| `fitonly` | The fitment seated, the clamp left hanging (it stops at the recess's mouth, against the pins) | 0.2 |
| `none` | The arms wait | 0.0 |
| Dream-1 | The right hand reaches toward the set on its holder in the first 10 s, then draws back and moves about over the table; the set never leaves its holder | 0 |

No seed is needed: nothing in the scene is random. Runtime per trial, on one shared GPU: the demo's 468 steps took about 7 minutes while recording three cameras (about 1.1 steps per second); a run to the 2,400-step limit takes about 35 minutes there.

## Assets

All meshes are made by the task's own code from plain shapes. No third-party assets, no textures.

The task statements quoted in `task.json` are from the O*NET database (O*NET OnLine, task statements for 29-1141.00 and 29-2061.00) by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence (https://www.onetonline.org/link/summary/29-1141.00, https://www.onetonline.org/link/summary/29-2061.00). O*NET is a trademark of USDOL/ETA. The loading steps are quoted from the BD Alaris System with Guardrails Suite MX User Manual P00000611, software 12.1.2 (2021-08), pp. 87 to 91, Pump Module Set Loading (https://infusystem.com/images/catalog_manuals/English/Pole_Mounted_Pump_Manuals/BD-Alaris-System-User-Manual-v12.1.2-US-Only.pdf). BD and Alaris are trademarks of Becton, Dickinson and Company, named for identification only.

## Licence

MIT
