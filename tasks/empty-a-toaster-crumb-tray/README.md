# Empty a toaster's crumb tray and slide it back in

People empty toaster crumb trays during ordinary kitchen cleaning; accumulated bread debris can char or burn during later use. Breville's BTA840 Care & Cleaning instructions (printed pp14–15, https://assets.breville.com/Instruction-Booklets/USCM/BTA840-instruction-manual.pdf) identify cooling/unplugging, pulling the tray, emptying and replacing it. Dualit's NewGen instructions (https://help.dualit.com/en-US/toaster-instruction-manuals-259361, printed p16) likewise describe frequent tray emptying. This task isolates dry emptying of loose bread chunks before any damp wiping or deeper internal clean.

## Recorded results

| Policy | Revised score | Recorded actions | Trials |
| --- | ---: | ---: | ---: |
| Scripted reference | 1.0 | 971 / 1100 | 2 |
| Dream-1 beta | 0 | 1100 / 1100 | 1 |

The board hosts the setup image and both uncut videos as **M-BP3VU8**. [Revision evidence](evidence/REVISION.json) records the native replay results. Revised scoring of seed-0 recordings: scripted 1.0 in both trials at action 971 of 1,100. Dream-1 beta 0.0 after 1,100 actions; the loaded tray stayed inserted for the entire trial.

The reference can inspect simulator body/contact state. Dream receives only
three 320 × 176 RGB images and 14 measured joint/gripper positions. The extra
side camera is for review. Guard changes and service session renewals are
recorded in the model summary; one model trial is not a success-rate estimate.

## Score a real trial

Score after the action or at the trial limit. The tray counts as withdrawn once its rear edge clears the opening by 5 mm. With the tray withdrawn at least once, award 0.25 when all 40 pieces can be accounted for in the tray or bin, or 0.6 when the tray is empty and all 40 are inside the bin below the rim. An unaccounted-for piece (including a spill on the worktop) earns 0 at that observation. Award 1.0 when all 40 remain in the bin and the upright tray stays home in its guides, hands clear, for 1 s. Compare both ends of the external pull against the initial home photos: within 5 mm in each direction, with no more than 5 mm of metal floor protruding. Direct handling of pieces, a fallen object, housing displacement over 20 mm or tilt over 10°, or invalid input voids the trial. Otherwise score 0.

For a real trial, use 40 reusable, contrasting, non-adhesive closed-cell EVA foam tiles as dry crumb stand-ins. Cut a 40 × 20 mm rectangle from a 2 mm craft sheet, then cut eight 5 mm strips across its width and five 4 mm sections per strip with scissors and a paper grid. This yields forty 5 × 4 × 2 mm pieces in one batch. Count them in five columns and eight rows and reuse the set. Replace worn or missing pieces from the same sheet. Place one tile at each point of a 5-column by 8-row grid centred on the tray, spanning 94 mm across and 200 mm along its length. Remove the paper layout guide before inserting the tray. Photograph the fully seated upright pull from the front and side with a 5 mm scale next to the slot.

For the endpoint count, stop the robot and inspect the bin, tray and surrounding worktop. If pieces overlap, photograph the home position and release first, then tip the bin into a five-by-eight counting grid and inspect the tray. A piece still in a tray dipped inside the bin belongs to the tray. Count each piece once. Video only needs to establish withdrawal, absence of direct handling, the housing limits and the released one-second hold. Apply the count at the scoring observation; pieces crossing the rim in flight are judged where they finish.

For reinsertion, compare the two outer ends of the ordinary pull to their reset positions relative to the slot. Use the front photo for height and side-to-side alignment and a side or overhead photo for depth. A 5 mm grid on the photos gives the tolerance in each direction at either end. The pull faces out, the tray sits upright in its slot, and no more than 5 mm of metal tray floor shows past the fascia. Watch the pull stay in that home zone for a full second after both grippers let go. A tray left proud or held in place earns at most 0.6.

## Install and run

Prepare the pinned native runtime in [ENVIRONMENT.md](ENVIRONMENT.md), then run
these commands **from this task folder**, in that environment:

```sh
uv pip install --python "$VIRTUAL_ENV/bin/python" -e '.[test]'
python tools/verify_source.py
PYTHONPATH=. python -m pytest -q tests
PYTHONPATH=. python -m rdb_toaster_tray.run scripted --out runs/scripted-new
```

For a fresh model trial, install the model extra and provide your own private
credential file; the output directory must not already exist:

```sh
uv pip install --python "$VIRTUAL_ENV/bin/python" -e '.[dream]'
python tools/evaluate_dream.py --task empty-a-toaster-crumb-tray --out runs/model-new --credential-file /path/to/private/credentials.env
```

The portable Dream runner keeps the stock SDK, inputs, guards and action budget.
Only package discovery and credential-path defaults differ from the original
recording entry point; no new model result is claimed for that packaging change.
One simulated trial lasts at most 55 seconds at 20 Hz.
The recorded model trial took 455.6 seconds of wall time
including initialization and inference; hardware and service latency vary.

## Specification and reset

The exact evaluated instruction, fixed starting layout, numeric rubric,
fidelity assumptions and reset are in [task.json](task.json).
[TECHNICAL.md](TECHNICAL.md) gives the mechanism and physical approximations;
[TASK-DESCRIPTION.md](TASK-DESCRIPTION.md) gives the real procedure and references.
The scripted runner scores every native physics substep and writes its original
`RESULT.json`, videos and grading inputs into its selected output directory.
`tools/verify_source.py` checks source identity, instruction and ordinary budget.

The evidence folder contains clearly marked summaries and hashes of the
original results and independently completed audits. Full raw input archives
are retained separately, rather than represented by truncated traces here.
The original recordings use the same reset, assets, actions and instruction as this revision. The archived summaries identify the scorer at recording time.

## Assets and licensing

The task props, their materials and the review-camera placement are original and covered by the task's [MIT license](LICENSE). Product references supply dimensions and context. The scene loads the following standard assets from the pinned runtime.

| Asset | Source and license |
| --- | --- |
| Two YAM arms with parallel grippers, link meshes, collision geometry, USD conversion and split arm covers | I2RT Robotics, MIT. The runtime ships the notices with the [YAM assets](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam). |
| Calibrated station, worktop, camera stand, lights, overhead D435 and two wrist D405 pinhole cameras | Loaded from the benchmark's YAM runtime. Its [README](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/README.md#license) states that the runtime as a whole has no declared license. These runtime assets are installed separately. |
| Standard ground plane | NVIDIA Isaac Sim content, loaded by Isaac Lab under the [Isaac Sim license](https://docs.isaacsim.omniverse.nvidia.com/latest/common/license-isaac-sim.html). |
