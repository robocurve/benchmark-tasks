# Collapse a travel cup and stow its heat sleeve

> Collapse the empty travel cup for storage. Leave its original sleeve tucked around the bottom of the folded cup and its drink tab closed.

Removing the sleeve, venting a sealed cavity, and nesting the bellows seat the cup into a much smaller packing state. Stowing the sleeve on the folded cup keeps the original pieces together. [Manufacturer's packing procedure](https://stojo.earth/pages/faqs)

## Physical setup and reset

Use the calibrated bimanual YAM station. The fixed arm bases are 61 cm apart. Its rigid laminate worktop measures 110 × 130 × 6 cm and sits 76 cm off the floor. Use its fixed overhead camera and two wrist cameras, plus a fixed oblique close-up for judging. Start both grippers open in the ready pose. The XML model uses the same table and camera calibration in the ABC robot coordinate frame. [Station calibration](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1)

Place an empty full-size 8 oz Stojo Jr. collapsible travel cup upright midway between the arm bases and 27.25 cm in front of their base line. The base center is at `(0.525, 0, 0.7475)` metres in the simulation. The silicone lip points toward the base line. The reference cup is 108 mm high when extended and 48 mm when collapsed. The lid is 79 mm across, the base is 51 mm across, and the full cup's catalog mass is 102 g. The bellows and plug are silicone. The lid, collar and removable heat sleeve are polypropylene. Leave the straw out. The cup and sleeve are the only movable task objects. [Reference cup](https://stojo.co/products/jr-bundle) · [Instruction manual](https://assets.pcna.com/Documents/Stojo%20Instruction%20Manual%201610-02.pdf)

For reset, open the plug, remove the sleeve, extend all three folds, put the sleeve back in its drinking position, and close the tab. Place the cup at the described start position and reset the arms. Match the board's setup photo. Keep the cup empty and stop if a part breaks. On a physical run, record the vent, sleeve and compression from reset through the final two-second hold. Include a ruler in the side and overhead photos.

## Scoring

Score the highest qualifying row in order. Score 0 if one arm strikes the other, a part falls below the tabletop or moves beyond its footprint, a component splits or sustains permanent damage, or the lid-to-base height first passes below 94 mm while the sleeve is on the cup or the drink tab is open less than 45°. The whole sleeve has cleared the cup when it is either below it with at least 2 mm of gap or fully off to one side. Use the first crossing below 94 mm visible in the trial video. The action limit is 1,600 actions at 20 Hz, or 80 simulated seconds.

| Score | Evidence |
|---|---|
| 0 | No higher tier qualifies or a failure applies. |
| 0.25 | The tab opens at least 45° and the whole sleeve clears the cup with the tab still open. |
| 0.50 | In addition, the cup is released, with its original lid attached, within 10° of vertical and 45–52 mm tall throughout the final two seconds. |
| 0.75 | In addition, the original sleeve surrounds the folded base throughout the hold, centered within 3 mm, within 10° of parallel, and within 4 mm of flush with the base edge. |
| 1.00 | In addition, the original tab remains within 5° of its closed position throughout the hold. |

`scoring.py` provides the same partial-credit ladder from native simulation measurements. It checks whole-arm contact at each 2 ms physics step. It treats any contact between an arm and the cup or sleeve as a failure of the released hold for that sample. The stage joints remain constrained to their travel and a breach by more than the integration tolerance invalidates the run. The physical trial additionally checks for material damage.

## Reproduce the scene and evaluate

Use Python 3.12, MuJoCo 3.11.0, NumPy 2.3.1, SciPy 1.17.0, OpenCV headless and FFmpeg. Mesa with its software EGL backend can render all four views on CPU. Install the calibrated `yam_arena` package at the commit named in [ENVIRONMENT.json](ENVIRONMENT.json). Only station calibration and limits are used from that dependency. See its installation notes for the reference environment. A preexisting ABC models directory can be supplied with `ROBOCURVE_ABC_MODELS` instead of fetching the pinned robot meshes.

From this task folder, fetch the official meshes and replay the recorded commands. If you already downloaded the official archive, pass its file path with `--archive`.

```bash
python fetch_assets.py
python run.py --play-commands commands.npz --out runs/replay
python render.py runs/replay
python run.py --out runs/planner
python run.py --mode hands_off --out runs/hands-off
python run.py --mode closed_press --out runs/closed-press
python run.py --mode leave_tab_open --out runs/tab-open
python check_retention.py runs/planner runs/retention
```

The planner takes 858 control steps (42.9 s of simulation). The full-limit hands-off run takes 1,600 steps (80 s). The commands and reset are fixed and there is no random seed. Saved command playback acts on the robot joints alone. The development planner used pose telemetry to search for the commands that are now saved in `commands.npz`. Replay uses neither this planner nor object poses as inputs to the controller. Native measurements and stage grades are in each run's `trace.jsonl` and `summary.json`. The setup image and review MP4 are rendered from the replay's actual states on the accepted layout at one frame per 20 Hz control state, including reset.

With the versions in [ENVIRONMENT.json](ENVIRONMENT.json), `python dream_cup.py --out runs/dream` evaluates Dream-1 against the same fresh scene and scorer. Provide access with `DREAM_API_KEY` in the environment. The policy gets only the three calibrated RGB views and robot joint positions. To verify the I/O locally, add `--dry-run` for eight offline hold actions. It makes no service calls.

## Evidence and fidelity

The planner and fresh command replay both scored 1.00 in 858 actions. The packed cup measured 47.26 mm tall with a 1.75 mm sleeve axis offset and the tab 0.004° from closed. The hands-off, closed-tab press, and successful pack with the tab left open scored 0, 0, and 0.75. In a separate lift check, holding only the collar lifted the seated sleeve 31.3 mm clear of the worktop, with at most 0.91 mm axis offset. The main replay audit recorded no cross-arm contacts at 500 Hz, controlled shoulders at or below 27 N·m, wrists at or below 7 N·m, and fingers at or below 40 N per actuator. The replay's maximum speeds were 2.39 rad/s for shoulders, 9.11 rad/s for wrists and 0.118 m/s for fingers, below the calibrated rig's respective 5.5 rad/s, 20 rad/s and 0.146 m/s limits. [Measured simulation results](VALIDATION.json)

Dream-1 completed 1,600 actions and scored 0.00. Its abrupt gripper requests briefly reached 0.64 m/s in the simulation. The measured scripted trial remained below the nominal 0.146 m/s limit.

The simulation is MuJoCo with official I2RT YAM visual meshes and ABC's stock finger collision surfaces. It uses the calibrated station's positions and camera intrinsics. The independent detail camera uses the same angle and lens as the reference station's review layout, centered on this task's workspace. The robot operates through native contacts and effort-capped joints. The fixed lighting avoids the small changing self-shadow patterns of a legacy shadow map.

The supplied camera geometry and cup envelope are dimensional references. An empty cup with three axial folds captures the measured scale and sequence of manipulation. Each fold has estimated hysteresis, there is a passive hinge and estimated closed-volume air pressure, and the original free sleeve uses a bounded reciprocal interference fit. The nominal full assembly matches the catalog mass and the fitted sleeve separates under opposite arm motion. The estimate collapses the buckling shell into three constrained axial stages. It omits local silicone deformation, tearing, permanent damage and actual leakage. The material forces and exact sleeve interface still require instrumented comparison with a physical sample.

## Assets and licenses

- All cup meshes, materials, scorer and motions are created for this task.
- `assets/abc_sim/models/yam_bimanual_empty.xml` is the unchanged ABC reference XML at commit `d0832d12651d1b260a652861a14648dc5f3660c7`, Apache-2.0. `scene.py` applies table, camera, light and controller changes at load time. [ABC source](https://github.com/amazon-far/abc/tree/d0832d12651d1b260a652861a14648dc5f3660c7) · [Apache license](assets/LICENSE.abc)
- ABC's official I2RT asset archive provides the robot link meshes `model2.stl`, `model2__2.stl` through `model2__17.stl`, `base_visual_gate.stl` and `d405.stl`. The robot asset package is MIT. The archive is pinned by SHA-256 in `fetch_assets.py`. The imported gate's visual mesh is loaded by the reference XML and unused in the trial. [I2RT license](assets/abc_sim/models/assets/i2rt_yam/LICENSE)
- The external `yam_arena` calibration dependency supplies table and camera-stand primitive parameters, camera transforms and robot reference limits. The dependency has no published software license and is used in place. Its source is not redistributed in this task folder. [Pinned station source](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1)
