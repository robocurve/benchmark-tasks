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
3. Install a flush force pad beneath the glove's support area. Zero it with the glove removed, calibrate around 0.45 N with a known vertical load, and then record the supported glove. Read wearer contact on the separate instrumented skin described below.
4. Leave the fixture untouched for 60 s. Repeat the same forearm draw with the shoe lifted, closed and released early. Record an ordinary grasp and a hard grasp at the same forearm position. In separate trials graze bare board before completing extraction, keep the sleeve held, hover inside the work area and close the empty jaws after withdrawal. Record both distal-arm envelopes against the work prism (x = −417 to 100, y = −254 to 168, z = 0 to 200 mm). The supported, settled glove must be released and each distal envelope at least 35 mm outside the prism for 31 observations at 20 Hz. Synchronize the encoder readout with overhead and side video.
5. Use the same glove, seam construction, table finish and cuff orientation across a comparison. Record uncaptured and captured pull-force curves before reporting hardware results. Differences in fabric friction, roller padding and forearm fit can change the ordering and magnitude of the measured forces.

Measure the smallest spacing from the moving right links to the feed board, station bench and fixed robot bases over the entire draw. Calibrated encoders and CAD provide a geometric lower bound. Use synchronized video of both red grip faces and a pressure strip or force-sensing pads when verifying two loaded fingers. An open gripper hovering above a supported glove earns only half credit.

### Instrument the wearer across its entire surface

Use a compliant dummy forearm, articulated glove shell and removable sleeve. Fit thin calibrated pressure-sensing tiles to the outside of the simulated contact surfaces, with a conformal replaceable cover. Preserve the fixture's external dimensions and document the contact friction after fitting the sensors. Record the developed surface map, tile serials, taxel areas, installed range and date with each dataset. This is a proposed physical build. No physical force measurements are claimed.

| Channel | Required coverage | Source attribution |
|---|---|---|
| Forearm | Circumferential coverage of the proximal and distal sleeve, fold, wrist cuff, elbow and exposed fixed upper sleeve. Include ends and seams wherever a link can touch. | Sum normal loads delivered by either robot, including links, gripper bodies and both pads. Exclude load borne by the table. |
| Glove | Palm and dorsum, both edges, knuckles, wrist-side boundary, and every accessible face, sidewall, web and tip of all four fingers and thumb. | Sum direct loads from either robot and the entire pressure shoe. Exclude support under the hand. |
| Table | Flush force pad under the resting glove, plus an independently instrumented bare-board zone for the brush control. | Report glove support separately from robot contact with bare board. |

Use an electrically isolated contact-identification layer aligned to the pressure taxels. Instrument the possible counterfaces with distinct contact codes for left robot, right robot, pressure shoe and feed board. Cover all accessible robot link, pad and shoe surfaces. Each loaded taxel records its counterface code and local normal pressure on the same clock. Validate attribution with the glove resting on the board while applying a known load to its back. Confirm that the backing pad reads the support reaction and the direct channel reads the applied load. A single net-force plate cannot distinguish balanced opposing fingertip loads. Photograph and probe every seam and transition. Split a mixed-source cell spatially or invalidate the trial if the sources cannot be resolved.

For each simultaneous acquisition, multiply each taxel's calibrated normal pressure by its effective area and sum across the appropriate source and region. With kPa and mm² the conversion factor is 0.001 N per kPa·mm². Assign each physical patch of surface to one active sensing layer. Account for cover preload, curvature, hysteresis and edge response in the installed calibration. Never subtract opposing loads. For example, two simultaneous 7.5 N robot pad loads on opposite sides of the forearm read 15 N. Two non-overlapping 7.5 N pulses peak at 7.5 N.

Acquire synchronized **full-surface frames** at 960 Hz or faster, with measured sensor bandwidth and a common hardware timing signal. Keep the raw forces. For comparison with the simulator's impulse-per-step reading, integrate each channel over each 1/240 s interval and divide by that interval's duration. Keep all 12 values per 20 Hz action and the peak for the full trial. The forearm and glove channels latch at or above 15 N and 4 N respectively. Record the axial spring separately. Drive labeled peak-hold readouts for forearm and glove and film them beside the distinct wrist flag. Record timing uncertainty and the response to brief force pulses in the trial report.

### Calibrate and challenge the assembled sensors

1. Zero each channel after fitting the cover. Apply a calibrated external reference load normal to the local surface with both rounded and flat indenters. Load and unload each mapped region at least three times, testing cell centres, edges, tile seams, fingertips, web spaces and high-curvature transitions.
2. Test the forearm at 2, 8, 12, 15, 18 and 25 N. Test the glove at 1, 3, 4 and 6 N, including lateral and downward loads. Apply two opposed probes with independent reference load cells. Check both 7.5 + 7.5 N on the sleeve and 2 + 2 N on the glove. Verify the individual channels, sum and latched display. A single pressure-to-force coefficient for an entire bladder cannot establish this calibration across contact areas.
3. Repeat while the glove is bearing on the support pad and while a robot touches bare board. Apply short synchronized pulses at several phases of the 240 Hz time base. Compare the reference and measured integrated force in each interval. Record sensor drift, saturation range, peak error, bandwidth and expanded uncertainty.
4. Treat a missing surface, ambiguous attribution, saturated taxel, dropped frame or failed timing check as an invalid measurement. If a physical reading's uncertainty interval straddles a scoring limit, mark the trial indeterminate and repeat with a measurement that can resolve it. Archive the coverage map, calibrated arrays, reference loads, encoder data, timestamps, raw samples and synchronized video with the trial. The thresholds characterize this apparatus. They establish no human injury limit.
