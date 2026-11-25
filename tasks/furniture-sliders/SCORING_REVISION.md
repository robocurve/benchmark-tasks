# Scoring revision: observable transport retention

Review of TK-6RE8Q6 found that continuous +/-0.5 mm foot-to-pad and pad-to-floor gaps cannot be judged from video. Scoring version 2 removes both gap tests from the moving condition and keeps the final stationary support checks.

| Assessment | During the transport segment | Final released state |
| --- | --- | --- |
| Foot/pad association | Each foot-base centre stays within 20 mm horizontally of the same distinct pad centre | All four foot corners at least 2 mm inside the usable foam edge |
| Height | Each foot-base centre within 10 mm of panel height plus loaded slider thickness; each pad underside centre within 10 mm of panel height | Lowest foot-to-pad and pad-to-floor gaps within +/-0.5 mm, checked at rest with a feeler gauge |
| Upright | Table and each pad within 15 degrees | Table within 3 degrees; pads within 5 degrees |
| Continuity | Same four pairings at every 20 Hz sample; footprints stay on the panel; any failure resets the segment | Same final pairings; 2 seconds settled, hands 50 mm clear, no robot support |
| Distance/goal | At least 150 mm net table displacement in a retained segment continuing through release | Entire tabletop inside the marked destination |

The limits refer to observable body positions and orientations, not contact sensors. The 20 mm alignment radius is 40% of the 50 mm foot width; the 10 mm height tolerance is 20% of that width. They can be assessed against known object dimensions in the low views, with overhead video for travel and the goal. They are deliberately much coarser than a bearing gap. The height bands are absolute relative to the floor, so repeated small upward motions cannot accumulate into a carry. Separation or unloading inside those bands is not distinguished during transport: this is a visual retention proxy, not continuous force/contact metrology. The final precise support check remains mandatory.

Use synchronized 20 fps overhead and opposing low views, including the two fixed oblique evidence views, with a ruler/reference image for scale. Track the same pads, compare base centres with the 20 mm alignment region, and check gross lift/tipping against the height/upright limits. Follow the full real-trial procedure in [README.md](README.md), including how to handle obscured views. Neither evidence camera is supplied to Dream. Whole-object lifting or upending before the qualifying segment remains legal; a visibly carried interval outside the height bands cannot establish transport credit.

## Implementation and evidence

`scoring.py` separates `matches` (stationary support) from `transport_matches` / `transport_retained` (visible retention). Pairing changes or retention loss reset the transport segment. Final full credit still requires all four stationary supports, the goal and released settling. The strict one-pad load/contact gate remains diagnostic. Physical construction, materials, masses, robot, instruction, controller and 6000-action budget are unchanged.

The original rear video hid a far base behind a nearer leg during transport. Numeric rescoring of that trace did not resolve the visibility problem. The replacement uses two fixed oblique evidence views: (1.05, +/-0.35, 0.96) m, aimed at (0.42, 0, 0.79) m, 640 × 352 at 20 fps, focal length 18 mm and horizontal aperture 20.955 mm. Neither is a policy input; the cameras add no bodies, colliders or lights. Their locations and a ruler-based real-trial calibration are specified in [README.md](README.md).

An explicitly labeled saved-pose preview was used only to choose camera placement. A **new continuous run** then repeated the unchanged controller from the declared bare start, with no pose writes or initialized pads. It scored **1.0 in 5838/6000 actions**. Independent rescoring agrees. The movement/release interval, steps **5468–5838 (273.40–291.90 s)**, has **200.403 mm** net displacement and unchanged four-pair retention in all **371** samples. Every frame of this interval was reviewed in base crops, with full-view sheets for context. Both streams contain every observation.

Across this interval, the camera geometry projects a 10 mm lift to at least **5.19 pixels** and a 20 mm horizontal displacement to at least **4.71 pixels in one view** (72 sampled directions). Conservative ray checks retain visible pad rims throughout; recorded-image inspection is documented separately. These checks establish centimetre-scale visibility, not submillimetre bearing gaps or continuous force. Final stationary support still uses the feeler-gauge procedure. If all views obscure a pair in another trial, that interval cannot establish the continuous visual witness.

| Recording | Actions | Independently assessed score | Evidence treatment |
| --- | --- | --- | --- |
| `full-camera-001` | 5838 | 1.0 | New continuous demo, five recorded views |
| `dream-001` | 6000 | 0.0 | Original genuine episode and video preserved |

The original model input remains the same three native RGB streams at 320 × 176, measured 14D state and instruction. The old recording-only rear view in its video is disclosed. No hosted inference was repeated. Its known SDK session-renewal behavior remains documented without altering its actual score.

The **48 CPU tests** pass, including transport height/alignment boundaries, pad loss, tipping, identity changes, slow lifting, off-panel supports, missing samples, final contact restoration, independence from force diagnostics, camera privacy and strict gate compatibility. Independent rescoring checks the physical specification and scene/controller helpers. Reusing a recording with the older evidence camera requires the explicit flag below and records both camera-source hashes; this does not waive physical equality or future live-evaluation guards.

```bash
python -B -m unittest discover -s . -p 'test_*.py' -v
python -B tools/rescore_revision.py /path/to/full-camera-001 --out /path/to/new-demo-rescore.json
python -B tools/rescore_revision.py /path/to/dream-001 --allow-recording-camera-change --out /path/to/new-model-rescore.json
python -B tools/prepare_demo.py /path/to/full-camera-001 --out /path/to/new-demo-export
```

Original snapshots, results, measured states, inference logs and media are preserved. The replacement demo uses a separate task-specific media upload; `task.json` and [provenance.json](provenance.json) record its ID and upload status. The previous media bundle M-7HHEF3 is historical evidence, not the replacement recording.
