# Fit the spring clamp on the glass filter holder and test the joint with a low lift

For a physical trial, put a stable, padded tray with raised sides under the whole lift area to catch the flask and contain fragments. Check that both grippers can reach the parts and withdraw past the rim. Record the empty flask in the tray at setup and over the tray's catch area from takeoff through set-down on each lift attempt. Zero the gauge at the surface supporting the flask foot on the pad. The top of the blue stripe marks 20 mm and the gauge top marks 26 mm. Record the lowest edge of the foot against the gauge from the side. Keep it at or below the gauge top throughout the lift and within the marked band for one continuous second to count the hold. Stop lifting if the flanges separate. If glass breaks, stop the trial and leave the fragments in the tray.

At the end of every physical trial, including an assembly-only trial, film the final assembly before opening the clamp. Score the assembly from that footage. If the glass is intact, keep the flask supported in the tray, steady the funnel, squeeze the clamp handles to remove the clamp and lift the funnel straight off. Film the membrane as you uncover it. Before touching it, show its whole face and edge against the frit. Check for tears or punctures and confirm that the edge is still centred all the way around. The check must pass for both bench credit levels, including 0.5 when the trial ends after assembly. Broken glass, a flask outside the tray's coverage during a lift, membrane damage or displacement, or missing or inconclusive required footage earns zero.

## Station card and simulation prompt

The station card gives the operator sequence and bench scoring checks. [Station card](task.json)

The current simulator sends the following prompt to the policy.

> Grip the glass funnel by its upper cylinder and lower it straight onto the membrane on the frit, the porous disc that supports the membrane. Hold the funnel steady. With the other gripper, squeeze the spring clamp handles, slide the jaws over both glass rims (flanges) and let them close. Move that gripper clear. With the flask on its mark, lift the assembly straight up by the upper cylinder alone. Stop lifting if the flanges separate. Keep the lowest edge of the flask foot at or below the gauge top. Hold that edge between the top of the blue stripe and the gauge top for 1 continuous second. Set the flask down upright on its mark, release the funnel, then move both grippers clear and keep them clear for 0.5 s.

## Original recorded simulation instruction

> Seat the glass funnel over the membrane on the fritted base. Hold it steady while fitting the spring clamp around both flanges. Take the other hand clear. Grasp only the upper funnel and lift the entire dry assembly until the bottom of the flask clears the bench by at least 20 mm for one second. Lower it upright onto its mark, release it and withdraw both hands.

## Run one trial

The recorded low-lift replay used Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena 0.3.0, Inspect Robots 1.0.0, inspect-robots-yam 0.38.0 and yam-arena 0.1.0 on Linux with an NVIDIA GPU. The stock YAM station, arm meshes and cameras are runtime dependencies. Run these commands in the current task folder to evaluate the simulation prompt and grader. Use the separately archived source in `evidence/frozen/` to reproduce the qualified low-lift run.

```sh
uv pip install --no-deps .
python -m rdb_filter_clamp.run scripted --out out/low-lift-scripted
python -m rdb_filter_clamp.run dream --out out/low-lift-dream
```

Use a fresh output directory for each run. The Dream-1 client needs a configured credential. Both runs start from the same scene with seed 0 and zero object jitter. The limit is 2,400 actions at 20 Hz, or 120 seconds of simulation. Dream-1 receives overhead and wrist RGB at 320 × 176 plus robot joint and gripper positions. The scripted controller reads object poses and jaw angles and commands only robot joints and grippers.

## Score and replay

The terminal simulation score is in `RESULT.json`. A seated and clamped assembly earns 0.5. For 1.0, take off from the mark, hold the flask foot 20–26 mm above its resting surface for one continuous second by the upper funnel alone, then set the flask down upright and keep both hands clear for half a second. The centre of the foot must be within 4 mm horizontally of the mark before takeoff, at set-down and throughout the hands-clear interval. Setting down outside that tolerance invalidates the lift. At each 20 Hz control sample, clearance above 26 mm permanently sets the score to zero, including after a valid hold. Incomplete assemblies and other latched failures also earn 0.

The category is Other because this dry trial checks whether the clamp holds the glass joint under the weight of the empty flask. Native scores measure assembly and lift with rigid glass on a bare bench and a membrane mesh fixed to the base. Bench credit remains unverified because the supplied evidence lacks physical footage of the tray at setup and throughout any lift and of the exposed membrane face and edge after the trial.

For each run, retain `RESULT.json`, `media/grader-trace.jsonl` and `media/side_cam.mp4` together. Compare the foot with both gauge edges through the lift and hold, then check that the joined parts return to the mark. The per-step trace records foot clearance, sole supporting arm, consecutive lift ticks and hands-off ticks. The current grader also logs horizontal displacement from the mark. The setup image, overhead video and model images accompany the trial. Contact forces and independent funnel, clamp and flask poses appear in the result diagnostics.

## Setup, reset and tests

The fixture represents the MilliporeSigma XX1014700 glass vacuum filter holder with its XX1014703 spring clamp, a size 8 silicone stopper and a 1 L filtering flask. The physical specification uses a white 47 mm, 0.45 µm mixed cellulose ester membrane. The maker's guide gives a 23.5 cm assembled holder height, 7.6 cm holder diameter and 300 mL funnel capacity. `TECHNICAL.md` identifies the sources and distinguishes published dimensions from estimates.

For the recorded layout, measure coordinates in millimetres from the midpoint between the arm bases, with x forward and y left. The arm bases are at (0, +305) and (0, -305). Place the flask axis at (330, -30), the funnel at (270, 175), the clamp at (300, -215) and the gauge at (330, -109). The simulated bench is the height reference. The base flange is 288 mm above it, and the gauge is 24 mm wide and 26 mm high. On the physical station, align the gauge to the padded resting surface before the trial.

The native scene uses 240 Hz physics and 20 Hz control. The flask, stopper and base form a free 0.72 kg body. Contact between the spring-driven jaws and the flange bevels carries the load. Part masses, spring strength and friction are estimates. Reset restores the starting poses and clears the scorer's lift history.

Use the same NumPy and SciPy versions as the qualified asset build to check regeneration.

```sh
uv pip install "numpy==2.3.1" "scipy==1.17.0"
python -m pytest -q tests
```

The scene test compares the regenerated USD text with the packaged resource and checks that the asset path used by the simulator has the same bytes. The native supervisor verifies the loaded asset hash and generates `validation.json` and the independent `evidence/frozen/` archive after the run.

With a supervisor-produced archive available, verify the frozen USD's SHA-256 against its `frozenFiles` entry in `validation.json`. Compare that file with the live package asset to see the exact scene changes.

```sh
git diff --no-index -- evidence/frozen/rdb_filter_clamp/filter_clamp.usda rdb_filter_clamp/filter_clamp.usda
```

The optional YAM kinematic test checks controller reach. Only the native PhysX trial tests load transfer through the clamp. Task code and generated geometry are MIT licensed.

## Original native qualification

These results, the hashes in the original revision of `validation.json` and media `M-JZ8C9K` belong to revision `864082418a7afde18f3461e8cec77e7330d53005`, which used the original prompt and a 20 mm minimum. Use that source revision to reproduce those runs. The scripted controller targeted 25 mm and the recorded peak was 24.62 mm.

| Original run | Score | Control steps | Wall time |
| --- | ---: | ---: | ---: |
| Scripted reference | 1 | 1,205 | 276 s |
| Dream-1 | 0 | 2,400 | 686 s |

## Recorded low-lift qualification

The seed-0 native replay of source tree `517a5ed7c13b5ff36c898f4ba3c4ce36ccc5e5e4` used the exact inputs in `evidence/frozen/`. Dream-1 scored 0 in 2,400 steps. Both scripted runs scored 1.0 in 1,185 steps, with a 22.074 mm peak clearance in each. The idle control scored 0. The supplied native test output reports 31 passes for that replay. [Replay inputs and scores](validation.json)

The frozen runtime was Isaac Lab-Arena 0.3.0 and its recorded environment label was `isaaclab-arena-0.3.1-yam-filter-clamp`. The current environment label uses 0.3.0. The recorder writes `media/side_cam.mp4` and `media/top_cam.mp4` within each run's output directory. The supplied replay summary contains no recording links or media IDs for those runs. 

The native attempt on source tree `1bccc772e4bd6cb902e03cf010156486075c094b` ended at 0.5 after 2,400 steps. At step 1,009, the last table-supported sample, the flask foot was 10.778 mm from the mark. At step 1,010 its clearance was 2.530 mm and it was still 10.133 mm from the mark. The trace recorded zero qualifying lift ticks, including at step 1,049 when clearance reached 22.074 mm.

The next native attempt, on tree `0ccd77391093e4a2ed683a9023ce632cbbeda922`, also ended at 0.5 after 2,400 steps. The lift began at step 925. At step 1,241, the foot cleared the 2 mm table tolerance and was 0.851 mm from the mark, with the left gripper as sole support. The highest supplied lift sample was 5.503 mm at step 1,245. At that phase boundary the 320-step lift loop ended, the controller logged 5.0 mm and zero hold ticks, and lowering began. The event log shows no physical safety stop. [Native attempt summary](review-evidence/current-reference-failure.json)

The scripted lift uses the measured flask foot centre throughout the lift and set-down. During preload, the upward command tapers to zero at half the grader's mark tolerance so the gripper can correct lateral drift before takeoff. The controller now uses the time left in the 2,400-step episode after reserving 308 steps for lowering, settling, release and the scored hands-clear interval. It logs step budget exhaustion separately from a stop for flange separation or excess height. The native qualification below identifies the tested trajectory. The qualification archive contains the original scores, runtime versions, loaded asset hash, per-step observations, and both camera recordings.

## Native qualification of this revision

Dream-1 run: 0 (2,400 steps). Scripted demo: 1.0 (1,423 steps). The second scripted run scored 1.0 and the idle control scored 0. Source tree `b4766b62ccd31dae901e7db46b2dd4fc3555caf6`, seed 0. The replay station used Isaac Lab-Arena 0.3.0. Both real-world credit levels also require inspection of the exposed membrane after the trial. The fixed simulated membrane provides no physical inspection result.
