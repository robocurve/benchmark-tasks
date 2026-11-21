# clear-loose-hex-wrench-from-benchtop-miter-saw-turntable-before-lowering-blade

## Occupational Context
In architectural woodworking, framing, and finish trim carpentry (BLS SOC 47-2031
Carpenters: >940,000 workers; BLS SOC 51-7042 Woodworking Machine Setters, Operators, and Tenders:
>130,000 workers; total commercial workforce >1.07M), operators routinely use compact 7-1/4" (185 mm)
compound miter saws to make high-precision crosscuts and miter angles. During blade changes or kerf
plate adjustments, arbor hex wrenches are frequently left lying across the turntable kerf insert.
Under OSHA 1910.213(s) woodworking machinery safety standards, attempting to lower the cutting head with
a 4,500 RPM carbide-tipped circular blade onto a hardened steel tool shears carbide teeth, warps the arbor
shaft, and creates violent projectile shrapnel causing severe facial, eye, and bodily trauma.

## Physical Setup & Kinematics
- **Miter Saw Base Chassis**: Heavy cast-aluminum base plate (200x220x35 mm) centered at `(0.280, 0.000, 0.7475)` m,
  equipped with revolving turntable center plate and high-contrast kerf slot insert (turntable surface at `0.7825` m).
- **Upper Cutting Head**: Aluminum swing arm and motor canister attached via rear hinge towers at
  `HINGE = (0.200, 0.000, 0.8175)` m, counterbalanced with calibrated 1.2 N*m friction spring hinge. The cutting head
  starts propped open at 75 degrees (`HANDLE_OPEN = (0.225, 0.000, 0.985)` m) and rotates down to
  horizontal 0 degrees flush against the mechanical depth stop (`HANDLE_CLOSED = (0.402, 0.000, 0.810)` m).
- **Hardened Steel Hex Wrench**: Standard 6 mm tool-steel arbor hex wrench (45 g mass) resting across the turntable
  kerf insert at `(0.300, 0.000, 0.7945)` m, fitted with a transverse upright grasp tab for 100% reliable parallel-jaw pinch grasp.
- **Side Magnetic Tool Tray**: 160x140x15 mm steel tool tray with perimeter retention lips centered at
  `(0.300, -0.220, 0.7475)` m on the right table surface.

## Task Sequence
1. **Grasp & Clear Hex Wrench**: Right arm approaches the hex wrench, descends to the grasp tab,
   pinches it firmly with parallel jaws (`yaw = pi/2`), lifts it clear of the turntable (>45 mm),
   and transfers it across to the side magnetic tool tray at `(0.300, -0.220, 0.770)` m.
2. **Lower Cutting Head**: Left arm approaches the front D-handle (`yaw = -pi/2`), grasps it firmly,
   and pulls the spring-counterbalanced cutting head down along its 75-degree circular arc until
   seated flush against the mechanical stop (`< 5` deg from horizontal).
3. **Stability Dwell**: Both arms release and retract clear (`>= 40` mm) and the entire assembly
   remains stable for at least 20 simulation steps (1.0 s at 20 Hz).

## Scoring Rubric
- `0.00`: Hazard tripped (cutting head lowered past 15 deg while hex wrench remains on turntable), wrench
  dropped below table, or no operational progress.
- `0.20`: Arbor hex wrench lifted clear (>45 mm) of the saw turntable kerf insert.
- `0.40`: Hex wrench transferred and deposited stably inside the side magnetic tool tray.
- `0.75`: Hex wrench safe in tool tray AND cutting head lowered flush onto bottom mechanical stop (`< 5` deg).
- `1.00`: Wrench safe in tray, cutting head bottomed out flush, both hands retracted clear (`>= 40` mm),
  and assembly stable for 20 consecutive steps (1.0 s dwell).
