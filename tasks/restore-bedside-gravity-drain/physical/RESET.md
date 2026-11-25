# Reset and independently survey the trainer

**Public start.** Use the raised case for the scored public trial. The submitted setup image is in media set `M-R3WQTE`. Use the [dimensioned station view](STATION_LAYOUT.svg) and the measurements below to register a new apparatus. Capture a photograph of the assembled bench for the reset record. A photograph of the physical build has not yet been taken.

## Reset dimensions

All z values are mm in the scene world. The stepped post spacers change the inert bed and its source head together. The rack and the robot mounts remain fixed. Rack tooth indices start at 0. A supported pin at a selected tooth is calculated to rest 0.25 mm below the listed nominal zero. This is within the source's ±1 mm reset survey.

| Case | Ear | Initial zero | Initial sill | Effective source head | Bed top | Start → target tooth | Role |
|---|---:|---:|---:|---:|---:|---|---|
| Raised | 1060 | 940 | 1040 | 1210 | 972 | 4 → 14 | Public task |
| Lowered | 940 | 1060 | 1160 | 1090 | 852 | 14 → 4 | Opposite correction / high-open diagnostic |
| Aligned | 1060 | 1060 | 1160 | 1210 | 972 | 14 → 14 | Diagnostic, immediate resumption available |
| Shifted | 1084 | 964 | 1064 | 1234 | 996 | 6 → 16 | Translated starting geometry diagnostic |

1. Park and isolate arm motion and support the carriage while releasing its tab. Close the tap against its closed stop and inspect the two fixed setting marks. Install the chosen four bed spacers. Place the fixed support independently from the moving bed. Survey the green centerline, source vent tip and magenta zero from the same ruler. Log the separately measured effective source head from the sight branch against the table; meniscus and bubble-pressure offsets can separate the two source readings. Write actual values in [SETUP_REPEATABILITY.csv](SETUP_REPEATABILITY.csv).
2. Move the zero to its initial tooth, support the carriage as the spring seats, then remove support. Check the engagement witness against the seated mark. The residual stroke must be ≤0.5 mm. Move the unloaded jaws clear. Measure initial zero and ear within ±1 mm of table values, sill minus zero = 100 ±1 mm, and tap angle ≤2°. The ±10 mm task alignment band applies after action begins.
3. Fill within the calibrated limits, prime and stabilize the supply following [WATER_CALIBRATION.md](WATER_CALIBRATION.md). Return every lock, tap stop, witness and hose clip to its recorded configuration. Verify source height = ear+150 mm, two open air gaps, remaining collection capacity, tray dry and the calibrated flow restriction setting. Start with dry/tared collection. Measure blank drift and record balance validity.
4. Set each stock arm to the station READY configuration, joints 1–6 in radians [0, 1, 1, −0.6, 0, 0], jaws open. At reduced speed, check joint direction and tool frame on the dry station before first use. The original public instruction is the sentence printed at the top of the task README. Present it unchanged.
5. Before step 1, record the overview, both wrist views and observer evidence with the motion clock visibly synchronized. Keep the setup photo, ruler, pin stop, tap stop, tare, scale display and hardware ID with the trial ID. Reject a wrong reset or deficient instrument before invoking the policy. End within 60 s, or at a verified earlier full-score endpoint.
6. For setup repeatability, another operator breaks down the height and tare, independently rebuilds the raised setting five times and logs each result. Every surveyed height, retained gap, tap reset, tare, as-built offset and repeated nominal water measurement must pass on all five runs. Record a repair and restart this series if any check fails. Perform diagnostic layouts separately.
