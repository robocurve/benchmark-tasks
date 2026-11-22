# Cold surrogate fixture and instrument calibration

This is a build specification for a proposed bench replication. Hardware trials have yet to be performed. Use a dedicated unpowered roller section, an inert dummy arm and a separate low-energy indicator circuit. The test needs neither a connected ironer nor a person in the fixture.

### Components and datums

Dimensions below are in millimetres. The origin is directly below the roller axis. Positive x enters the nip, y follows the roller shaft, and z points upward. The full shape and joint dimensions are in `fixture.py`.

| Assembly | Build target |
|---|---|
| Feed surface | 396 × 422 mm smooth board with interchangeable low-friction face. Top at z = 70, rear edge at x = −21. Mark the textile clearance line at x = −29 and the wrist line at x = −165. |
| Roller | Stationary 64 mm outside diameter × 178 mm working length. Sleeve a rigid core with textile-covered resilient padding. Fix the shaft against rotation. Axis at x = 0, z = 45. |
| Shoe and release | Short 163 mm working face, three tangent contact strips, pivot at x = 60, z = 88. Pivot rod, paired bushings, two cheek plates, weight and outboard hand lever. The modeled shoe assembly is 1.15 kg with its centre of mass 60 mm forward of the pivot. The red handgrip is 16 mm thick, 42 mm wide and 78 mm tall. Mark the two opposing lateral faces for camera review. Adjust ballast with a scale. Set the closing travel stop at −9.5 degrees relative to the assembly datum so the empty shoe meets the roller crown. |
| Forearm and glove | Elbow pivot, low-friction swivel, passive elbow and wrist flexion, fabric-covered forearm and glove. Fixed upper sleeve extends off the near table edge. Wrist approximately 211 mm from elbow. Glove extends a further 164 mm to the middle fingertip. Use a compliant adult training glove over a light mannequin shell. |
| Garment | Sew a 119 × 36 mm loose cotton hem to the spring tab on the work-shirt cuff. Use flexible fabric for the physical sample. The simulation divides this hem into seven 17 mm patches. |
| Wrist load switch | Guided 0.50 N/mm spring with at least 36 mm travel, contrasting index, 5 mm scale and adjustable 10 mm switch. A 0–20 N spring scale or calibrated gauge calibrates the instrument. |
| Latched display | A dry-contact switch at the threshold trips a latched red 36 mm pop-up flag in a separate case. Reset between trials. The measured sleeve load is upstream of the display. |
| Measurement | Steel rule, 1 mm shims or feeler gauges, dial indicator and calibrated force gauge. A marked overhead camera captures the wrist index, sleeve line and flag. A side camera resolves clearance and glove support. |

### Bench acceptance checks

1. Zero the extension index with the relaxed sleeve. Apply axial loads of 1.0, 2.5 and 5.0 N with a gauge. Expected motion is 2, 5 and 10 mm. Record actual travel in both directions over three cycles. Record the load at which the flag first deploys and confirm it stays visible after unloading. A physical replica needs a calibration record before its trials can be compared with simulation.
2. Seat the cloth between the stationary roller and shoe. Verify that about 30 mm of hem initially extends beyond the roller centreline and that the resting wrist mark remains forward of it. Measure closure force with a thin gauge at the crown. Compare it with the native initial contact load in the trial record. Measure the smallest face-to-roller gap across the three strips when operating the lever.
3. Mark the glove support region on the feed table. Use a flush force pad under this region for the terminal support reading. Use pressure film or a second force sensor for direct glove-face impacts. Ordinary video can resolve the flag and clearance lines. The 4 N impact and 0.45 N support thresholds require calibrated instruments for equivalent quantitative scoring.
4. Leave the system untouched for 60 s. The hem must stay caught and the flag retracted. In matched trials repeat the same forearm sweep with the shoe held open, left closed, or released before withdrawal. Record switch travel, textile clearance and any persistent flag before resetting. Run a separate trial with the released arm still held, then trials with the released arm under an open, hovering gripper and with a withdrawn, closed gripper. For the final placement trial, record both measured jaw travels and both distal-arm envelopes against a marked work prism. In station coordinates the prism spans x = −417 to 100, y = −254 to 168 and z = 0 to 200 mm. Each wrist and gripper envelope must stay at least 35 mm outside the prism and each jaw at least 80% through its open stroke for 31 observations at 20 Hz. A synchronized overhead and side view or calibrated robot encoders can document that interval.
5. Use the same glove, seam construction, table finish and cuff orientation across a comparison. Record uncaptured and captured pull-force curves before reporting hardware results. Differences in fabric friction, roller padding and forearm fit can change the ordering and magnitude of the measured forces.

Measure the smallest spacing from the moving right links to the feed board, station bench and fixed robot bases over the entire draw. Calibrated encoders and CAD provide a geometric lower bound. Use synchronized video of both red grip faces and a pressure strip or force-sensing pads when verifying two loaded fingers. An open gripper hovering above a supported glove earns only half credit.
