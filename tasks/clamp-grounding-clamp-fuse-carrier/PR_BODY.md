# Clamp Safety Grounding Lead Before Pulling Ceramic Fuse Carrier

## Summary
Implements bimanual safety prerequisite compliance for commercial high-voltage pull-out fused disconnect boxes (OSHA 29 CFR 1910.269(n) and NFPA 70E §120.5 personal protective grounding prerequisites).

Before servicing high-voltage distribution disconnects, technicians must mechanically attach a spring-loaded grounding clamp to the enclosure busbar to discharge induced voltages. This task enforces this safety prerequisite using an implicit hazard fault detector: attempting to extract the ceramic fuse carrier prior to grounding trips a safety interlock, locking the score at 0.0. After grounding is established, the robot extracts the fuse carrier from dual spring-clip terminals and deposits it flush into the safety parts tray, retracting both arms to equilibrium dwell.

## Occupational Relevance
- **Primary SOC**: `47-2111 Electricians` (762,600 US jobs, BLS OEWS May 2023).
- **Secondary SOC**: `49-2095 Electrical and Electronics Repairers, Powerhouse and Substation` (23,400 US jobs).
- **Total Workforce**: Over 1,165,000 workers performing lock-out/tag-out, equipotential grounding, and electrical distribution service.
- **Physical Difficulty**: Bimanual coordination, spring-loaded duckbill jaw grounding, linear clip extraction, containment placement, and equilibrium dwell.

## Validation Suite & Qualification Scores
- **demo_1** (yam_scripted, seed 0): **1.00** (hold=29 at step 320, full credit)
- **demo_2** (yam_scripted, seed 1): **1.00** (hold=29 at step 320, full credit)
- **ctrl_no_ground** (seed 0, cheat=`no_ground`): **0.00** (hazard fault detector trips at step 100, locked at 0.0)
- **ctrl_ground_only** (seed 0, cheat=`ground_only`): **0.50** (clamp grounded, fuse carrier untouched)
- **ctrl_no_tray** (seed 0, cheat=`no_tray`): **0.75** (fuse carrier extracted, held outside tray)
- **dream_1** (policy=dream, 800 steps, seed 0): **0.00** (baseline model achieves 0.00)

## Mechanical Setup & Coordinate Architecture
- **Commercial Disconnect Enclosure (`disconnect_box`)**: Heavy-gauge steel NEMA 3R chassis (140x180x80 mm, 1.2 kg) centered at `(0.22, 0.00, TOP + 0.060)` with door hinged open at top `+X` rim clear of lateral transfer paths.
- **Copper Busbar Terminal**: Vertical copper terminal tab (40x10x35 mm) on broad mounting shelf (80x80x12 mm) at `(0.22, +0.120, TOP + 0.045)`.
- **Bronze Grounding Clamp (`grounding_clamp`)**: Cast bronze duckbill clamp (50x40x18 mm base, 45g) initialized at `(0.18, +0.20, TOP + 0.010)`. Grounding scored at `dr <= 25 mm, dz <= 30 mm`.
- **Ceramic Pull-out Fuse Carrier (`fuse_carrier`)**: Molded ceramic body with insulated capture T-bar (40g) seated at `(0.22, 0.00, TOP + 0.055)`.
- **Safety Parts Tray (`safety_tray`)**: Molded containment tray at `(0.22, -0.18, TOP + 0.015)`. Scored at `dr <= 45 mm, dz <= 25 mm`.

## Media
- **Media ID**: `M-ASMKT6`
- Screenshot: `setup.png`
- Scripted Demo: `demo.mp4`
- Baseline Evaluation: `model_run.mp4`
