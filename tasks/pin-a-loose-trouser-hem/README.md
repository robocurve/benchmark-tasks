# Secure a loose trouser hem with a safety pin

Secure the loose trouser hem with the safety pin, with its point safely enclosed.

**Scripted demo: 1.0. Dream-1: 0.0 after all 1,800 actions. Both native runs were independently rescored.**

## Setup and reset

Use an inside-out cotton-twill trouser cuff, approximately 440 mm in circumference and 240 mm long, with a 20 mm turned hem. Unpick the middle 100 mm of its front stitching and remove loose thread. A 140 x 190 x 60 mm wooden sewing support separates the front from the back of the leg. The loose hem overhangs it by 46 mm toward the left arm. Its visible draped width is 180 mm; approximately 261 mm of front fabric and 180 mm of back fabric form the circumference.

An ordinary 75 mm steel safety pin starts open on a 20 x 40 x 15 mm wooden rest toward the right arm, with its coil overhanging by 25 mm. Both hands start open. The [Spentex M0710 blanket/kilt pin](https://www.spentex.co.uk/wp-content/uploads/2023/06/30191-SPEN-Product-Catalogue-2023-ART-SCREEN-DPS.pdf) is a commercial size reference. The simulated pin approximates this size rather than reproducing that product exactly.

Restore the photographed layout, replace punctured fabric, inspect the pin and protective finger pads, and leave the pin open. No needle is already threaded. `geometry.py` specifies all parts and fixed starting coordinates.

## Scoring

- **1.0:** The closed pin holds both front and turned-hem layers within the loose 100 mm section, with its point inside the shield. The hem stays secured for three seconds after release.
- **0.5:** Both layers remain threaded, but the pin is open or still held.
- **0.25:** At least one correct layer remains partly threaded on the shaft.
- **0:** Otherwise, including any back-layer puncture, a puncture outside the loose section, more than four holes, or a tear over 5 mm.

Judge the threading, shield and released hold from close video and a ruler. The scorer accepts any intact pinning route that retains both layers. A layer that slides off the open needle loses credit; puncture count alone does not secure the hem.

## Simulation

Flexible cloth panels, a sprung pin, a compressible needle tip and native finger contacts. A 1.0 N tip load plus geometric intersection creates an object-to-object sliding hole. The spring must be compressed and the point aligned with its shield to close. The catch retains the point with a 15 N break-force limit; it does not redundantly weld the already hinged arm orientation. This puncture law is approximate; it is not calibrated to a particular cuff and pin. [Published needle analysis](https://www.mdpi.com/2073-8994/18/4/635) provides a newton-scale comparison, not product calibration.

The 6 mm coil has a 2.6 mm thickness and its plane matches the pin. Regularized slender-body inertias and a finite friction patch suppress numerical spin. Both demo and Dream use a 20,000 N/m gripper gain with a 40 N/finger force ceiling. No task object is attached to a hand or moved by a commanded pose. The cloth, puncture, spring and contact parameters remain approximations.

The scripted demo may read physical state. Dream uses only the standard cameras and joint observations. Both runs use identical object physics and robot settings.

## Run and inspect

Use the Isaac 6.1 / Python 3.12 / YAM environment in `ENVIRONMENT.json`. From the repository root:

```sh
python tasks/pin-a-loose-trouser-hem/run.py --mode demo --out /tmp/hem-demo
python tasks/pin-a-loose-trouser-hem/run.py --mode dream --out /tmp/hem-dream
python tasks/pin-a-loose-trouser-hem/rescore.py /tmp/hem-demo
python tasks/pin-a-loose-trouser-hem/rescore.py /tmp/hem-dream
```

The fixed layout has no random seed. Limit: 1,800 actions, 90 seconds at 20 Hz. Each run records native camera videos, physical states and immutable source files. Rescoring checks source hashes, trace continuity, action bounds, break events and final outcome. Inspect the native videos as well. `--capture-every 4` is for faster development diagnostics only and marks the run as unsuitable submission evidence.

The scripted demo completes 1,560 actions in 78 simulated seconds and 16.5 minutes on a Secure A40. It retains both correct layers, closes the shield, and remains secured for 40.75 seconds after release, with no broken material joints. The peak puncture load is 1.14 N. The full trace independently scores 1.0. Dream completes all 1,800 actions in 21.0 minutes on the same A40 and scores 0.0. Its physical episode is continuous; the SDK recovered 36 expired sessions across 73 requests and 37 server sessions. A separate earlier run interrupted by a laptop connection failure is excluded from the submission.

The public evidence bundles contain every commanded action, the final four seconds of native body/hand states, the full-trace audit result and the complete trace hash. Check them without Isaac:

```sh
python tasks/pin-a-loose-trouser-hem/check_evidence.py tasks/pin-a-loose-trouser-hem/evidence/demo-outcome.json.gz
python tasks/pin-a-loose-trouser-hem/check_evidence.py tasks/pin-a-loose-trouser-hem/evidence/dream-outcome.json.gz
```

These compact records verify the final outcome and action bounds; they are not a physics replay or the earlier body-state history. The run commands above generate complete traces for independent rescoring.

## Assets

The cuff, pin and supports are procedural geometry created for this task and released under the included MIT license. No third-party object meshes or scans are bundled. The external YAM and Isaac environment is pinned in `ENVIRONMENT.json`; that file is the pre-trial environment snapshot. `VALIDATION.json` records the completed task validation.
