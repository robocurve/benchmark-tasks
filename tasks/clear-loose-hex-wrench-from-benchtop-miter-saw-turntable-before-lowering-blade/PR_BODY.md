### Summary
Add high-impact occupational benchmark task `clear-loose-hex-wrench-from-benchtop-miter-saw-turntable-before-lowering-blade` under BLS SOC 47-2031 (Carpenters, >940,000 workers) and SOC 51-7042 (Woodworking Machine Setters, Operators, and Tenders, >130,000 workers; total commercial workforce >1.07M).

In architectural woodworking, framing, and finish trim carpentry, carpenters and machine operators routinely operate compact 7-1/4" (185 mm) compound miter saws. During blade changes or kerf adjustments, arbor hex wrenches are frequently left lying across the turntable kerf insert plate. Under OSHA 1910.213(s) woodworking machinery safety standards, lowering the 4,500 RPM carbide-toothed blade onto a hardened tool-steel hex wrench shears teeth, creates explosive projectile shrapnel, warps the arbor shaft, and poses catastrophic facial, eye, and bodily trauma to the operator.

### Physical Sequence
1. **Clear Hex Wrench**: Right arm approaches the hex wrench, descends to the grasp tab, pinches it firmly with parallel jaws (`yaw = pi/2`), lifts it clear of the turntable (>45 mm), and transfers it across to the side magnetic tool tray at `(0.300, -0.220, 0.770)` m.
2. **Lower Cutting Head**: Left arm approaches the front D-handle (`yaw = -pi/2`), grasps it firmly, and pulls the spring-counterbalanced cutting head down along its 75-degree circular arc until seated flush against the mechanical stop (`< 5` deg from horizontal).
3. **Stability Dwell**: Both arms release and retract clear (`>= 40` mm) and the assembly remains stable for at least 20 simulation steps (1.0 s at 20 Hz).

### Hardened Anti-Cheat & Scoring Rubric
- `0.00`: Hazard tripped (cutting head lowered past 15 deg while hex wrench remains on turntable), wrench dropped below table, or no operational progress.
- `0.20`: Arbor hex wrench lifted clear (>45 mm) of the saw turntable kerf insert.
- `0.40`: Hex wrench transferred and deposited safely into the side magnetic tool tray.
- `0.75`: Hex wrench safe in tool tray AND cutting head seated flush against mechanical depth stop (`< 5` deg).
- `1.00`: Wrench safe in tray, cutting head closed flush, both hands retracted clear (`>= 40` mm), and assembly stable for 20 consecutive steps (1.0 s dwell).
