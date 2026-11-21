# Tabletop trainer calibration

The benchmark tests whether a repaired contact edge reverses a powered shutter at two locations. A spring-backed block acts as a repeatable test obstruction. All limits below describe the miniature fixture. Hardware fabrication and physical validation are outstanding.

## Geometry and reset

1. Use the frame's tabletop mounting surface as the height datum. Align both shutter guides with the vertical scale. The frame origin, body transforms, joint axes, limits, masses and drive constants are in native/shutter.usda.
2. Place the open shutter carrier at 150 mm. Set the block carriage at the left mark, +66 mm from the centreline. Set the right cassette at the 48 mm fault end of its slide. The seated end is 0 mm. Use the fixed witness line on the carrier to read cassette offset.
3. Verify that each shoe moves independently through its 9 mm travel. The model uses 300 N/m return stiffness and 1.2 N s/m damping. The block pickup bushing has 16 mm upward travel. At rest its shoulder seats under gravity. The proof spring has 200 N/m stiffness and 22 mm downward travel. Measure compression relative to the 18 mm nominal separation of the cap and bushing centres.
4. With the drive inhibited, check the markings and spring displacements using a scale or dial indicator. For a physical prototype, record measured spring force versus displacement after subtracting rest preload and measure travel in both directions to expose sticking. The nominal incremental forces for 5 mm deflection are 1.5 N for one shoe and 1.0 N for the proof spring. Record the measured values before comparing hardware to the simulation.
5. For the powered comparison use an enclosed, limited-energy tabletop trainer with a captive proof piece. The lightest instrumentation to review a proof is a side video with the millimetre scale, the local shoe-travel indicator and a peak indicator on the witness. Hardware force instrumentation would be needed to compare the simulator's contact-force readout directly. No person serves as a test obstruction.

## Simulated reference measurements

The model commands descent at 65 mm/s and return at 90 mm/s. A local shoe movement of 2 mm triggers the powered return. An independent overload return also reopens the shutter at 15 mm compression. Record the first contact, the loaded shoe, maximum compression, reversal position and final open position. The terminal frame alone cannot distinguish the two returns.

| Initial cassette / block location | Peak witness compression | Reopening from lowest point | Credited site |
|---|---:|---:|---|
| Faulty / left mark | 5.39 mm | 78.27 mm | Left |
| Faulty / right mark | 15.10 mm | 108.60 mm | None, crush trip |
| Seated / right mark | 5.58 mm | 78.54 mm | Right |
| Seated / left mark | 5.39 mm | 78.27 mm | Left |
| Seated / centre seam | 9.46 mm | 82.33 mm | None, off mark |

These are robot-free CPU PhysX measurements using the shipped fixture. The spring-backed witness measures equipment response. Its 15 mm limit has no established mapping to human injury. The task makes no claim of product certification.
