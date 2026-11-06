# Mark a perpendicular cut line

Implemented and verified in the fixed simulated YAM setup. This is a local review package; `task.json.media` remains empty until the separately authorized media upload. No task has been submitted.

## Task and recorded results

Use the carpenter's square and orange marker to draw one straight cut line through the red tick, perpendicular to the board's long edge and across its width. Keep the square seated against the near edge while marking, avoid extra marks, then lift the marker clear of the board.

| Evaluation | Result | Control steps | Wall time on the test VM |
| --- | --- | ---: | ---: |
| Scripted feasibility demo, two reset trials | 1.0, 1.0 | 1, 564 and 1, 571 | 655.7 s combined |
| Hover control, nib kept 15 mm above board | 0.0; zero ink contacts | 1, 517 | 258.6 s |
| Dream-1 beta smoke | 0.0; no runtime errors | 200 | 48.0 s |
| Dream-1 beta evaluation | 0.0; no runtime errors | 2, 400 | 361.5 s |

Both demonstrations covered 98.04% of the target, with angular errors 0.1084 and 0.1070 degrees, zero stray ink and 100% guided ink. Both finished with `demo_motion_complete`. All five trial traces independently regraded to the recorded scores, on the GPU VM and a separate Windows CPU environment. The second episode started with a blank board; the hover control's final board remained blank.

The scripted demo uses measured simulator tool poses to correct grasp/placement error. It commands the same 14 joint/gripper targets as other policies, with no tool attachments or pose writes during an episode. Privileged observations are **disabled by default** and were disabled in both Dream-1 runs. The demo continues after satisfying the task to retract both tools for a clear result view; model evaluation stops normally on success or its action limit.

Frozen implementation revision:

`1f58be809db849dc58db72c2b78abc6196befeee4e9121720fc1a379402bf5f5`

This hashes all task Python modules and USD assets. Documentation and recorded score metadata are outside that implementation hash. The task-local `.gitattributes` preserves the validated Python/USD bytes across Git checkouts, including line endings.

## Verified environment

Use the official [Robocurve simulation walkthrough](https://tasks.robocurve.org/walkthrough). The tested environment used Ubuntu 22.04.5 LTS, one L40S, 64 GB RAM, 8 vCPUs, Python 3.12.16, Isaac Sim 6.1.0.0, NumPy 2.3.1 and Inspect Robots 0.60.0.

- YAM adapter: `0059d02a1a91bc6834c1d0846022891df0c78730` from [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena).
- Arena: `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`.
- Isaac Lab submodule: `91a0906f93802eb859192dcb0bca879f891e92d6`.
- Dream adapter: `b5e936a201cec66c54c297e522c920d98445e339`; `dreamlabs-client` 1.0.14.

The official installation's sample scene needed `ARENA_NUCLEUS_DIR = ISAACLAB_NUCLEUS_DIR` in Arena's `assets/nucleus.py`, because the staging 6.1 apple asset returned 404 while production 6.1 was available. This task's board, square and marker are included local assets. Keep the tested versions: the runtime uses version-specific construction/sensor hooks. Initial shader warm-up takes several minutes; subsequent trials still include simulator startup.

## Install and run

From a prepared simulator VM, substitute the actual task directory:

```bash
source /data/IsaacLab-Arena/yam.env
"$HOME/.local/bin/uv" pip install --no-deps -e /path/to/square-cut-line
inspect-robots list embodiments
TASK_TEXT=$(python -c 'from square_cut_line.task_spec import INSTRUCTION; print(INSTRUCTION)')
```

The embodiment list must contain `square_cut_line`. Run the feasibility demo with a fresh output directory:

```bash
inspect-robots run --embodiment square_cut_line --policy square_cut_line_demo \
  -P mode=demo -E demo_observations=true -E terminate_on_success=false \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  -E output_dir=outputs/demo/traces --instruction "$TASK_TEXT" \
  --scorer task_score --max-steps 2400 --epochs 2 --seed 0 --store-frames \
  --grader none --no-prompt --no-rerun --no-rerun-save --log-dir outputs/demo/logs
```

For the hover negative control, use `-P mode=hover`, one epoch and a separate output directory. It follows the tool-handling sequence but keeps the nib 15 mm above the board; no marks should appear.

For Dream-1, configure its API key in the documented private credentials file and use:

```bash
inspect-robots run --embodiment square_cut_line --policy dream \
  -P model=dreamlabs/dream-1-beta -P timeout_s=60 \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  -E output_dir=outputs/dream/traces --instruction "$TASK_TEXT" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 --store-frames \
  --grader none --no-prompt --no-rerun --no-rerun-save --log-dir outputs/dream/logs
```

Do not enable `demo_observations` for model evaluation. The recorded API runs used an external persistent request-budget wrapper, which only counted SDK calls and did not change actions or observations. This task used 54 SDK calls across smoke/full evaluation, approximately $0.072 at the published rate.

## Score and inspect a trial

The task exports `contacts.jsonl`, `ink.json`, `metrics.json`, `run_config.json`, source manifests, and native camera PNGs under each `traces/trial-*` folder. The independent regrader uses measured forces and poses rather than trusting stored score/ink flags:

```bash
python -m square_cut_line.regrade outputs/demo/traces/trial-0000
for log in outputs/demo/logs/*.json; do
  case "$log" in *.live.json) continue ;; esac
  inspect-robots video "$log"
done
inspect-robots view outputs/demo/logs
```

Exit 0 from the regrader means the contact reconstruction, ink history, source revision and reported metrics agree. Source changes intentionally invalidate older evidence. Save the original source snapshot with recorded runs. Standard video export creates one H.264 MP4 per camera and epoch.

## Fixed scene

Axes: x away from the arm bases, y left, z up; metres. Arm bases are at(0,+/-0.305, 0.75); tabletop z=0.7475.

| Object | Geometry and initial arrangement |
| --- | --- |
| Board | Fixed wooden board 140x280x16 mm, centre(0.29, 0, 0.7555), topz0.7635. Fixings are outside the working area and simplified away. |
| Reference tick | Printed red 4x2 mm visual-only mark at(0.222,-0.03) on the top. The target line spans x0.22..0.36 at y=-0.03. |
| Square | Loose 180 g rigid plastic model. Stock24x250x36 mm; blade 170x20x6 mm. Stock root starts(0.18, 0.27, 0.7655), yaw0. Blade centre relative to root is(0.085,-0.1075, 0.001). |
| Marker | Loose 40 g long-nose marker. Orange barrel 22x22x80 mm;4x4x23 mm narrow nose; physical nib sphere radius 2 mm. Total length 107 mm. Root(0.18,-0.19, 0.8175); barrel centre 15 mm above root; nib centre 50 mm below root. |
| Holder | Fixed plastic 40x40x18 mm base centred at(0.18,-0.19). Four 15 mm-high walls form a 6x6 mm opening that supports the narrow nose. Nib rests18 mm above the bench. |

The square and marker each have one rigid body with separate primitive colliders; the L opening is preserved. The board is kinematic. Reset clears ink and restores the documented layout; use zero jitter. For physical trials, replace/clean the marking surface and restore tools without changing geometry or leaving old marks.

## Frozen grading contract

Ink is created only by a filtered marker-board contact localized at the physical nib: normal force>5 mN, point within 4 mm of nib centre, point within 1 mm of board top, nib bottom within 2 mm of it, axis within 25 degrees of upright, and point inside board bounds. Barrel contact/hovering do not draw. Contact loss breaks a stroke; no invisible connection is inserted across a lift or implausible jump. Visible marks are2 mm wide.

Geometry is measured on a 0.25 mm raster of the visible footprint. Dense overlapping stamps, dwell and retraces are handled as the ink they visibly leave. Full geometric credit requires 90% coverage inside a +/-3 mm reference band, gaps<=3 mm, angle error<=2 degrees, stray ink<=5 mm-equivalent, total ink-equivalent length<=210 mm, and transverse RMS<=1.5 mm. Partial 0.5 requires 50% coverage inside a +/-5 mm band, gaps<=5 mm, angle<=5 degrees, stray<=10 mm-equivalent, the same 210 mm limit, and RMS<=3 mm. Otherwise 0. The stray corridor extends 1 mm beyond the target endpoints and 6 mm to either side of the target centreline, including the 1 mm stroke-radius allowance. All grading-region bounds contain the whole board.

Guide use requires the measured stock face within 3 mm of the near edge, blade-edge height within 2 mm, blade yaw/tilt within 2 degrees, and the edge2.5+/-2 mm from target, with the nib within 3.5 mm of that edge. Full credit requires 90% guided ink;50-90% caps credit at 0.5; below50% gives0. Actual lateral contact force is diagnostic, not required. Full credit also requires continuous nib clearance>=5 mm for 0.5 seconds without new ink. The final grade is the minimum of these conditions. See `task.json` and source for the exact numeric definitions.

## Tests and limitations

49 automated tests passed in a separate CPU-only Python 3.12 environment with NumPy, pytest and usd-core. They cover scoring invariance/adversarial marks, nib/shaft/hover gates, guide geometry, lift duration, independent regrading/tamper checks, USD bodies/colliders and controller phase deadlines. The physics evidence above is additional to these tests.

Rigid contact is simulated; ink flow, tip compliance/wear, wood deformation and smearing are not. Numerical contact tolerances approximate the nib/surface interface. Guide use is inferred from measured geometry. This is validated for the fixed layout, not arbitrary tool poses, hardware deployment, or learned-policy success.

The task code and included tool assets are original and MIT licensed. External robot/simulator dependencies retain their own licenses. No upstream example implementation is bundled here.

Novelty was screened against 207 live board entries and relevant benchmark catalogs on 2026-11-04; no close marking-task duplicate was found. Recheck before submission; the idea is not reserved by this local implementation.
