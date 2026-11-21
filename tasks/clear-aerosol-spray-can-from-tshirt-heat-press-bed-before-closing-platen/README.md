# clear-aerosol-spray-can-from-tshirt-heat-press-bed-before-closing-platen

## Occupational Context
In commercial garment decoration, screen-printing, and heat-transfer curing (BLS SOC 51-5112
Printing Press Operators: >140,000 workers; BLS SOC 51-9199 Production Workers, All Other:
>1,100,000 workers; total commercial workforce >1.2M), operators routinely use aerosol pallet
adhesive to tack textiles onto lower platen beds. Under OSHA 1910.307 and NFPA fire safety
standards, closing a commercial 380 deg F (193 deg C) heated clamshell platen onto a pressurized
aerosol can poses an immediate catastrophic boiling liquid expanding vapor explosion (BLEVE),
high-velocity metal fragmentation, and severe burn hazard.

## Physical Setup & Kinematics
- **Heat Press Base Chassis**: Heavy steel base plate (200x220x35 mm) centered at `(0.280, 0.000, 0.7475)` m,
  equipped with a 170x190x12 mm heat-resistant orange silicone pressing bed (top surface at `0.7825` m).
- **Upper Heated Platen**: 180x200x30 mm aluminum platen attached via rear hinge towers at
  `HINGE = (0.200, 0.000, 0.8175)` m, counterbalanced with a 1.2 N*m friction spring hinge. The platen
  starts propped open at 75 degrees (`HANDLE_OPEN = (0.225, 0.000, 0.985)` m) and rotates down to
  horizontal 0 degrees flush against the mechanical stop (`HANDLE_CLOSED = (0.402, 0.000, 0.810)` m).
- **Pressurized Aerosol Can**: Standard 6 oz industrial aerosol adhesive spray can (45 mm diameter,
  45 g mass) resting on the lower pressing bed at `(0.300, 0.000, 0.7945)` m, fitted with an elevated
  top spray valve actuator stem for 100% reliable transversal pinch grasp.
- **Fire-Safe Silicone Staging Pad**: 160x140x15 mm silicone rest pad with perimeter containment lips
  centered at `(0.300, -0.220, 0.7475)` m to the right of the heat press.

## Task Sequence
1. **Grasp & Extract Aerosol Can**: Right arm approaches the aerosol can, descends to the actuator
   stem, pinches it firmly with parallel jaws (`yaw = pi/2`), lifts it clear of the bed (>45 mm),
   and transfers it across to the fire-safe staging pad at `(0.300, -0.220, 0.770)` m.
2. **Close Clamshell Platen**: Left arm approaches the insulated vinyl front handle (`yaw = -pi/2`),
   grasps it firmly, and pulls the upper heated platen down along its 75-degree circular arc until
   seated flush against the lower bed stop (`< 5` deg from horizontal).
3. **Stability Dwell**: Both arms release and retract clear (`>= 40` mm) and the assembly remains
   stable for at least 20 simulation steps (1.0 s at 20 Hz).

## Scoring Rubric
- `0.00`: Hazard tripped (upper platen lowered past 15 deg while aerosol can is on the bed), aerosol
  can dropped below table, or no operational progress.
- `0.20`: Aerosol spray can lifted clear (>45 mm) of the lower heat press bed.
- `0.40`: Aerosol spray can transferred and deposited safely onto the side staging pad.
- `0.75`: Aerosol can safe on staging pad AND heat press platen closed flush (`< 5` deg from stop).
- `1.00`: Can safe, platen closed flush, both hands retracted clear (`>= 40` mm), and assembly
  stable for 20 consecutive steps (1.0 s dwell).
