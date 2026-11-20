# Take out the trash: cinch and remove a loaded drawstring liner

> Pull both drawstrings to close the full bag, lift it out of the bin by the drawstrings, and set it down upright on the table beside the bin without spilling any contents.

**Category:** Societal relatability. Taking out a small kitchen or bathroom bin by pulling its drawstring liner shut and lifting it out is something almost everyone does at home and recognises in a 10-second clip.

A small round bin (an IKEA NYPON 15 cm pot) holds an ordinary, unmodified **simplehuman Code A drawstring liner**, its surplus folded over the rim the usual way, loaded with three light mock-waste blocks (70 x 100 x 45 mm, 50 g each). The two arms pinch the liner's two drawstring loops (flexible flat-tape straps, simulated as jointed strap chains) where they lie on the mat beside the bin, pull them so the folded surplus slides up over the rim and the string slides out of the hem and gathers the mouth, lift the loaded bag out by the strings, set it down upright on the table beside the bin and let go. This is the closure the liner is sold with: pull the drawstrings, then carry by the drawstrings. There is no knot, cord lock, clip or added handle.

## What changed in this revision (latest review)

| Review point | Change |
|---|---|
| Rigid 14 x 10 x 20 mm pinch bodies replace the flexible-loop grasp | **Removed.** Each exposed loop is now a flexible flat strap, like the liner's film-tape drawstring: a chain of 15 thin links (20 mm wide, 10 mm long) on spring-loaded joints, laid on the mat as a U of two strands and a bight (`loop_chain.py`, `removal_assets.py`). The gripper must pinch the strap with its ordinary contact grasp, closing across the doubled tape; nothing is attached. The string tension acts on the strap's end links at the cut-outs and reaches the hand only through the strap's joints. |
| Full-score demo of the updated scene | New scripted demo on the official environment (Results). |
| Scorer awards cinch and carry without checking the drawstrings were pulled and the bag lifted by them | **Cinch** now also needs both loops held between the fingers (one per gripper), the string drawn out of the cut-outs by at least 100 mm in total and taut, and no film ever pinched between the fingers. **Carry** also needs, at the moment the bag clears the rim, both loops held, the string taut, no film within 5 mm of either finger, and no film ever pinched: the load hangs from the loops. Unit tests cover squeezing the bag shut without pulling, loops not held, film pinched before the cinch or during the lift, and a finger resting on the film while lifting (`test_drawstring_score.py`); scripted negative runs are in Results. |
| Related changes | The string model gained a limited viscous term standing for the string's sliding friction in the hem, because in the previous model the bag bounced up to about 15 cm on the capped string while carried; the elastic cap stays at 0.7 N. The scripted demo now uses the adapter's standard gripper force (an earlier 8 N finger-force limit is gone), and the controller reads only visible geometry (strap links, bag mesh) and its own joints; it no longer consults the live score. |

<details><summary>Earlier review of this PR (still applies)</summary>

| Review point | Change |
|---|---|
| Custom cord-lock liner and added handles were not an established method | Replaced by a standard commercially available drawstring liner (simplehuman Code A) used exactly as supplied, with its ordinary closure. Cord-lock housings, their one-way holding force and the polymer handles are gone from scene, physics and protocol. |
| Locks and handles removed the normal difficulty of securing an ordinary liner | **Partly addressed.** The cord locks and their one-way force are gone; closure now depends only on keeping string tension (`drawstring_physics.py`, `test_drawstring_physics.py`), and the real trial uses the liner's unmodified loop with no handle. |
| Threading, cord pairing and assembly unverified | Nothing is assembled. The bin is chosen so the liner verifiably fits: the NYPON pot's 53.7 cm rim is smaller than the liner's 58 cm mouth, so the surplus folds over the rim as people normally fit a liner. The simulation starts in that folded state with the loops lying at the cut-outs, matching [PHYSICAL_PROTOCOL.md](PHYSICAL_PROTOCOL.md), which also gives sources, reset, a hand preflight of liner and closure, and ruler/video scoring. |
| Update simulation, scoring and fidelity note | Folded-cuff liner mesh, thin pot, a load of three 70 x 100 x 45 mm blocks, tiered scorer (`drawstring_score.py`, `test_drawstring_score.py`) with a two-cap containment check (`test_containment.py`), rewritten fidelity table. |
| New full-score demo and Dream-1 run | New scripted demo on the official environment below. Dream-1 was re-run on the new scene on the official environment: a valid run to the step limit (see Results). |
| Runtime | The task now runs on the official Isaac Lab-Arena release/0.3.1 environment with the installed YAM adapter as is; the earlier compatibility overlay is only applied if a newer Arena API is detected. |

The closure threshold went from 60 mm to 80 mm together with the larger blocks: a block's smallest face (70 x 45 mm) has an 83 mm diagonal, so no block can pass through an opening at the 80 mm limit.


</details>

## Scoring

| Score | Outcome |
|---|---|
| 0 | Never cinched as below, any block leaves the bag (latched), bag destroyed, or invalid simulation geometry. |
| 0.25 | Cinched by the drawstrings: opening reached 80 mm or less (smaller than the 83 mm diagonal of a block's 70 x 45 mm smallest face) with all three blocks inside, **while each gripper holds one strap loop between its finger pads, the string has been drawn out of the cut-outs by at least 100 mm in total and is taut, and no film has been pinched between the fingers.** |
| 0.5 | Carried by the drawstrings: after cinching, the opening is at most 80 mm when the whole bag and its contents clear the bin rim by 20 mm, **with both loops held, the string taut, neither finger within 5 mm of the film, and no film ever pinched**. |
| 0.75 | Set down: at the end the bag rests upright on the table (mouth at least 150 mm above it), clear of the bin and table edges by 20 mm, opening at most 80 mm, contents inside. |
| 1.0 | Released: that state, opening included, holds for the final 2 s with both grippers at least 60 mm from film and loops and the bag still (bounds move at most 10 mm). |

Grasp evidence is geometric and read in each gripper's frame: a loop is held when the fingers are closed to 15 mm or less with a strap link between the pads; film is pinched when a film node is between the pads with the fingers closed to 30 mm or less; drawn-out string is the summed distance from the four strap ends to their cut-outs minus its start value; the string is taut when its routed length is no more than 10 mm below its start value. Lifting, pinching or squeezing the film earns no cinch or carry credit.

Highest satisfied level counts. Containment is checked at every step; the opening is checked at rim clearance and over the final 2 s, so it may widen (to about 120 mm in the demos) while the bag swings in transit. Regrasps and corrections are allowed. Run limit: 4,000 control steps at 20 Hz (200 s); requested CLI seed 0 (the installed adapter reports environment seed 42); no object jitter. `DrawstringJudge` runs live at every control step for both the scripted demo and Dream-1, and `evaluate_removal.py` / `replay_run.py` re-score recorded geometry offline.

## Running

The official environment: Python 3.12 with Isaac Lab-Arena release/0.3.1 (`c7edf3a`), Isaac Sim 6.1.0, Isaac Lab 17.0.2 and Inspect Robots, as in [runtime_versions.json](runtime_versions.json); YAM adapter `kairosiann/yam-isaac-lab-arena` at `0059d02a1a91bc6834c1d0846022891df0c78730`, used as installed. `launch.py` puts the adapter on the path directly; only if it detects a newer Arena API (`Embodiment.registry_name`, no `GroundPlane`) does it build the `.runtime/` overlay from `prepare_runtime.py`. From this directory:

```bash
/path/to/env/bin/python launch.py setup --adapter /path/to/yam-adapter --arena-root /path/to/IsaacLab-Arena --out runs/setup_001
/path/to/env/bin/python launch.py demo  --adapter /path/to/yam-adapter --arena-root /path/to/IsaacLab-Arena --out runs/demo_001
/path/to/env/bin/python launch.py model --adapter /path/to/yam-adapter --arena-root /path/to/IsaacLab-Arena --out runs/dream_001   # needs Dream credentials (~/.dreamlabs/credentials)
python replay_run.py runs/demo_001 --out runs/demo_001/independent_replay.json
python -m unittest discover -v
```

The scripted demo is a closed-loop controller that reads simulator state (loop positions, bag mesh) to aim its grasps and choose its set-down height, as the rules allow; it never sets object poses. Robot and gripper use the adapter's standard settings. The model run gives Dream-1 the standard top and wrist cameras at 320 x 176; the overview camera is diagnostic only. Encode the scripted recording with `ffmpeg -framerate 5 -pattern_type glob -i 'runs/demo_001/frames/*.png' -c:v libx264 -pix_fmt yuv420p demo.mp4`.

## Results

All runs are the scripted controller on the official environment (Isaac Lab-Arena release/0.3.1, Isaac Sim 6.1.0, installed YAM adapter), scored live by `DrawstringJudge` with the grasp checks above.

| Run | Result |
|---|---|
| Scripted demo (`LP4_a`, the media demo) | **task_score 1.0** (tier `released`), 1,587 control steps / 79.35 simulated s, 1,724 s wall time. Both strap loops pinched across the doubled tape; cinched with both loops held and pulled at step 358, carried clear of the bin by the loops (no film touched) from step 688, set down upright beside the bin and released; final opening 72.4 mm, mouth 0.251 m above the table, grippers 0.232 m away, bag bounds moved 1.4 mm over the last 2 s, peak film edge stretch 1.39. Independent replay of all recorded states: 1.0. |
| Repeats, same source | `LP4_b` 0.75 (`set_down`): set down upright beside the bin, opening 54.6 mm, but at release one strap stayed wound round a finger (35 mm away). `LP4_d` 0.25: cinched with both loops pulled, then the right strap slipped out of the fingers during the pull. `LP4_c`, `LP4_e`, `LP4_f`, `LP4_g` and `LP4_h` 0: a strap (usually the right one) slipped out of the fingers during the grasp probe or the cuff peel, before any cinch. Two further runs of the same source with an extra release retry (never reached): `LP6_a` 0 and `LP6_b` 0.25, a strap slid out of the fingertips during the pull. |
| Summary | **1.0 in 1 of 10 runs of the final source (0.75 once, 0.25 twice, 0 six times).** The failure modes are the strap slipping out of the fingers during the pull or lift, and a released strap staying wound round a finger. The strap grasp is a real contact pinch on thin links and is not robust in this controller; the scripted demo is not a reliability claim. |
| Development runs (same scene and scorer, earlier controller or string settings) | 1.0 once more (`LP2_b`, 18 mm strap, gentler 8 N finger force); otherwise 0.25 to 0.75, with the same failure modes plus the bag staying in the pot or swinging out of reach on the capped string (`LP2_*`, `LP3_*`, `LP5_*`, `LP_*` in [VALIDATION.json](VALIDATION.json)). |
| Negative controls (same source) | **Loops held but not pulled** (`LPN6_nopull`, the same source with an extra release branch that this mode never reaches: both straps pinched, lifted 5 cm and held 10 s without pulling): 0.0, opening 205 mm, string drawn out 23 mm (100 mm needed). A second run on the final source (`LPN4_nopull`) also scored 0.0 (one strap slipped out during the hold). **Film lifted instead of the loops** (`LPN4_filmlift4`): the loops are pulled until the mouth is cinched (0.25 earned at step 350), then both are let go and one hand pinches the gathered neck film and lifts 25 cm. The scorer records film pinched at step 675 and no carry credit is possible after that; final 0.25. In simulation the film slid out of the pinch and the bag stayed in the pot, so this run does not show a film-carried bag; that case (and a finger resting on the film while lifting) is covered by `test_drawstring_score.py`, which scores it 0.25. |
| Opening while carried | The opening first met 80 mm with both loops pulled, then widened to up to about 120 mm for a few seconds while the bag swung in transit, with all blocks inside. Containment is checked every step; the opening is checked at rim clearance and in the final 2 s. |
| Dream-1 | **0.0, valid run.** Re-run on this revision's scene and scorer (strap loops), official environment, the task's instruction and 4,000-step limit, 320 x 176 cameras: one session from the first action to the end. Because this scene simulates slowly, the model was queried every 4 control steps so the session never sat idle. `model_run.mp4` is this run. |

The scripted demo only shows the scene and scorer can be completed; no physical trial has been run. Run records are in [VALIDATION.json](VALIDATION.json).

## Fidelity

PhysX surface-deformable film (not calibrated) authored in the fitted state, folded over the rim, with a slight crease kept at the fold and loose pleats in the hanging surplus; rigid blocks; standard robot actuation (the adapter's own gripper force) and contact grasps; no robot-object attachment and no scripted object poses after reset. Each exposed loop is a flexible strap: 15 thin rigid links (20 mm wide, 10 mm long, 2 mm numerical thickness, 4 g per loop) on spring-loaded joints that bend freely about the tape's width, twist against a spring and bend little in the tape's plane; the gripper pinches it by contact across the doubled tape. Its twist spring is stiffer than a real film tape (the simulated loop stays on edge; a real one may lie flat), and its thickness and mass are numerical. The elastic string force is capped at 0.7 N (about twice the static load per strand) as a numerical safeguard, so a hard pull or fast lift lengthens the force-only drawn-out string (by up to about 1 m in the demo runs) rather than pulling harder; a limited viscous term standing for the string's sliding friction in the hem damps the hanging bag's bounce. Block containment allows 8 mm on block centres and face centres (film contact offset, film pressed in under load) and 20 mm on block corners, which can protrude through a film triangle (mesh edges up to 20 mm) between its nodes. The string inside the hem is a lumped routed-length spring acting on 20 hem markers and the four strap end links; string drawn out between the strap ends and the cut-outs is force-only and drawn as a line, not a collider; there is no explicit string/hem contact model and no lock. Known gaps, listed in [PHYSICAL_PROTOCOL.md](PHYSICAL_PROTOCOL.md#simulation-correspondence-and-limits): 0.10 mm film (product gauge unpublished), the pot modelled as a straight cylinder, the liner's last ~2 cm on the mat omitted. Containment uses the film mesh with force-free virtual caps. The film-stretch guard flags invalid simulation, not tearing. No physical trial has been run.

## Third-party assets and software

| Item | Source / licence |
|---|---|
| YAM robot meshes, URDF, derived USD | I2RT Robotics, MIT, commit `120c3c81400171174604e503943f8d1ebc891058`; notice in `LICENSES/I2RT-MIT.txt`. |
| Table, floor, camera stand, station coordinates | Operator-installed YAM adapter pinned above; it declares no source licence at that revision and is not redistributed here. |
| Liner film, pot segments, blocks, strap loops | Procedural geometry in this directory; no product mesh, scan, texture or artwork. Product names and dimensions are cited for the physical specification only. |
| Isaac Sim, Isaac Lab, Isaac Lab-Arena, Inspect Robots, Dream client | Installed separately under their own terms; not bundled. |

Novelty against TRI's loaded drawstring-bag closure and other prior art is discussed in [NOVELTY.md](NOVELTY.md).
