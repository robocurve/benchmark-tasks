### Summary
Add high-impact occupational benchmark task `clear-aerosol-spray-can-from-tshirt-heat-press-bed-before-closing-platen` under BLS SOC 51-5112 (Printing Press Operators, >140,000 workers) and SOC 51-9199 (Production Workers, All Other, >1,100,000 workers; commercial workforce >1.2M).

In commercial garment screen-printing and direct-to-film (DTF) transfer production, press operators routinely use pressurized aerosol adhesive spray cans to tack textiles to the lower press platen bed. Under OSHA 1910.307 and NFPA fire safety standards, leaving a pressurized aerosol can on the lower bed when closing the 380 deg F (193 deg C) heated clamshell platen poses an immediate catastrophic boiling liquid expanding vapor explosion (BLEVE), violent cylinder rupture, projectile shrapnel, and severe burn hazard.

### Physical Sequence
1. **Remove Aerosol Can**: Right arm approaches the aerosol can, descends to the actuator stem, pinches it firmly with parallel jaws (`yaw = pi/2`), lifts it clear of the bed (>45 mm), and transfers it across to the fire-safe staging pad at `(0.300, -0.220, 0.770)` m.
2. **Close Clamshell Platen**: Left arm approaches the insulated vinyl front handle (`yaw = -pi/2`), grasps it firmly, and pulls the upper heated platen down along its 75-degree circular arc until seated flush against the lower bed stop (`< 5` deg from horizontal).
3. **Stability Dwell**: Both arms release and retract clear (`>= 40` mm) and the assembly remains stable for at least 20 simulation steps (1.0 s at 20 Hz).

### Hardened Anti-Cheat & Scoring Rubric
- `0.00`: Hazard tripped (upper platen lowered past 15 deg while aerosol can is on the bed), aerosol can dropped below table, or no operational progress.
- `0.20`: Aerosol spray can lifted clear (>45 mm) of the lower heat press bed.
- `0.40`: Aerosol spray can transferred and deposited safely onto the side staging pad.
- `0.75`: Aerosol can safe on staging pad AND heat press platen closed flush (`< 5` deg from stop).
- `1.00`: Can safe, platen closed flush, both hands retracted clear (`>= 40` mm), and assembly stable for 20 consecutive steps (1.0 s dwell).
