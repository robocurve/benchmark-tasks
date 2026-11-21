# Clamp Safety Grounding Lead Before Pulling Ceramic Fuse Carrier

## Overview

In commercial electrical maintenance, HVAC service, and industrial power distribution, electricians and substation technicians frequently service commercial fused pull-out disconnect boxes (such as Eaton/Square D 60A NEMA 3R fused disconnects, ~$35 off-the-shelf commercial equipment). Under OSHA 29 CFR 1910.269(n) and NFPA 70E §120.5 personal protective grounding mandates, de-energized distribution busbars can carry hazardous induced capacitive voltage or backfeed electrical potential.

This benchmark task evaluates bimanual robotic manipulation for safety prerequisite compliance and multi-mechanism extraction:
1. **Implicit Safety Prerequisite**: Before disturbing the high-voltage ceramic pull-out fuse carrier, the robot must identify the exposed copper ground busbar and securely clamp a bronze spring-loaded grounding duckbill clamp jaw onto the busbar tab (establishing an equipotential zone).
2. **Hazard Fault Detector**: If the robot attempts to pull or extract the fuse carrier before the grounding clamp is mechanically secured to the ground busbar, the ungrounded fault detector trips (immediate 0.0 fail score).
3. **Fuse Carrier Extraction**: After grounding is verified, the robot grasps the insulated pull-handle of the ceramic fuse carrier and withdraws it from the heavy-duty spring-clip receiver terminals.
4. **Safe Component Placement**: The extracted fuse carrier is carried across the workspace and deposited flush into the molded safety parts tray.
5. **Clear Arms & Equilibrium Dwell**: Both manipulator arms retract clear (>= 35 mm clearance) and maintain stationary stability for 0.75 s (15 consecutive control steps at 20 Hz).

## Occupational Relevance

- **Primary SOC**: `47-2111 Electricians` (762,600 US jobs, BLS OEWS May 2023).
- **Secondary SOC**: `49-2095 Electrical and Electronics Repairers, Powerhouse and Substation` (23,400 US jobs, BLS OEWS May 2023).
- **Additional Trade Impact**: `49-9041 Industrial Machinery Mechanics` (380,000+ US jobs).
- **Total Workforce**: Over 1,165,000 workers regularly perform lock-out, de-energization, grounding, and component service on electrical distribution apparatus.
- **Physical Difficulty**: Bimanual safety interlock execution, spring-loaded duckbill jaw clamping, high-force linear clip withdrawal, precision placement, and multi-step stillness hold.

## Mechanical Components

1. **Commercial Disconnect Enclosure (`disconnect_box`)**:
   - Heavy-gauge painted steel NEMA 3R enclosure (140x180x80 mm chassis, 1.2 kg) mounted at `(0.22, 0.00, TOP + 0.060)`.
   - Hinged steel door swung open upward at top rim (`+X`) providing unhindered lateral transfer access.
   - Vertical copper ground busbar tab (40x10x25 mm) on a broad copper mounting shelf (80x80x12 mm) positioned at `(0.22, +0.120, TOP + 0.045)`.
2. **Bronze Grounding Duckbill Clamp (`grounding_clamp`)**:
   - Cast bronze grounding clamp body with duckbill jaw opening and broad stable rectangular base (50x40x18 mm, 45g).
   - Molded high-dielectric insulated rear grip post and T-bar cross handle (38x22x10 mm) for robot parallel clamp.
3. **Ceramic Pull-out Fuse Carrier (`fuse_carrier`)**:
   - Commercial molded ceramic insulator body (75x40x22 mm, 40g) holding 60A cylindrical cartridge fuses.
   - Insulated central finger pull-handle stem and capture T-bar (36x22x10 mm) for extraction.
   - Dual brass receiver terminal spring clip alignment.
4. **Safety Parts Tray (`safety_tray`)**:
   - Molded anti-static safety parts tray (140x110x25 mm, 0.25 kg) centered at `(0.22, -0.18, TOP + 0.015)`.
   - Molded containment rim walls for secure setdown.

## Scoring Breakdown

- **0.20**: Grounding clamp lifted from table (> 20 mm vertical lift).
- **0.50**: Grounding clamp jaw securely clamped over copper ground busbar tab (dr <= 25 mm, dz <= 25 mm).
- **0.75**: Fuse carrier extracted from enclosure spring clips without tripping ungrounded fault interlock.
- **1.00**: Fuse carrier deposited flush in safety parts tray (dr <= 45 mm, dz <= 25 mm), both arms cleared >= 35 mm, and held stationary for 0.75 s (15 consecutive steps at 20 Hz).
- **0.00 (Fail)**: Attempting to withdraw or displace fuse carrier before grounding clamp is secured (ungrounded fault trigger), base tipped > 15°, or components dropped off bench.
