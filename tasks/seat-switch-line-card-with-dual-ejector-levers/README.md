# Seat a switch line card by swinging dual ejector levers closed into chassis catches

## Task Description
In data center infrastructure maintenance and telecommunications central offices, network technicians install and hot-swap high-performance modular switch line cards into enterprise chassis (e.g. Cisco Catalyst 9400 / 9600 series, Arista 7500R, and IEEE 1101.10 CompactPCI/VME64x standard enclosures). 

Inserting a multi-gigabit line card requires coordinated dual-arm manipulation:
1. The line card (4.0 kg rigid blade assembly) is advanced along guide card tracks in a standard 19-inch 4-post rack bay until its front-mounted mechanical cam ejector levers contact the chassis pivot pins (gap ~25 mm to ~8 mm).
2. The robot grasps both left and right cam ejector levers (initially opened at 85°) and rotates them inward synchronously.
3. The hardened cam lobes engage behind the chassis front flange pins, converting the rotary torque into linear mechanical advantage (~8.5:1 leverage ratio) to overcome the backplane connector mating insertion resistance (~80 N total across dual high-density backplane blocks) over the final 8 mm travel.
4. Throughout the final 8 mm mating phase, both levers must advance symmetrically: excessive skew (>1.5 mm difference between left and right insertion depth) causes connector pin jamming or bending and trips an immediate, permanent task failure (`bent_pins` latch).
5. Both ejector handles swing fully shut (<=2.5° from the card faceplate) until their spring-loaded release catches engage the card detent latches with an audible/tactile snap.
6. The robot releases both handles (withdrawing grippers >30 mm clear), and the assembly withstands an automated 20 N axial retention proof pull along the insertion axis to verify positive mechanical locking without dislodging.

## Real-World Importance & Occupational Link
- **Occupational Link**: Network and Computer Systems Administrators (O*NET 15-1244.00) and Telecommunications Equipment Installers and Repairers (O*NET 49-2022.00). According to the US Bureau of Labor Statistics (BLS), over 580,000 workers maintain enterprise computing, routing, and switching infrastructure.
- **Hardware Reproducibility**: Designed to standard IEEE 1101.10 / PICMG 2.0 mechanical specifications (widely commercialized by Schroff / nVent 20817-606 and Hartmann Elektronik). Standard 1U horizontal sub-rack slot mounted in an EIA-310-D standard 19-inch rack bay (482.6 mm outer mounting width) with 403 mm width between guides, 520 mm depth, 40 mm height, and dual ergonomic cam handles with integral slide locks. Mating force matches measured DIN 41612 / IEEE 1101.10 high-density 5-row backplane connector insertion forces (0.25 N per contact pin pair = ~80 N total for 320 pins).

## Key Parameters
- `CHASSIS_SLOT_WIDTH`: 0.403 m (standard 19" rack 1U horizontal slot aperture)
- `CHASSIS_DEPTH`: 0.520 m
- `LINE_CARD_MASS`: 4.0 kg
- `CARD_WIDTH`: 0.400 m
- `CARD_LENGTH`: 0.460 m
- `CARD_THICKNESS`: 0.035 m
- `CAM_LEVER_LENGTH`: 0.110 m
- `CAM_THROW`: 0.008 m (8.0 mm final insertion travel)
- `MAX_SKEW_TOLERANCE`: 0.0015 m (1.5 mm differential left vs right) during connector engagement
- `RETENTION_PULL_FORCE`: 20.0 N
- `PULL_DISPLACEMENT_LIMIT`: 0.0015 m (1.5 mm)

## Success Criteria & Stages
- `Stage 0.25`: Line card advanced along guide rails until cam claws engage chassis pins (`both end gaps <= 0.025 m`).
- `Stage 0.50`: Both ejector levers rotated >= 45° toward closed position, advancing card into connector guide range (`both end gaps <= 0.008 m`), maintaining skew <= 1.5 mm.
- `Stage 0.75`: Both levers rotated fully shut (both angles <= 2.5° from faceplate) and locked into detent catches (`both end gaps <= 0.0010 m`). Levers open > 2.5° cannot award 0.75.
- `Stage 1.00`: Grippers clear (> 0.030 m) and card assembly withstands a 20 N retention proof pull along the withdrawal axis without dislodging (`displacement <= 0.0015 m` held for 1.0 s).
- `Defect Latches (Permanent 0.0)`:
  - `skew_bent_pins`: Left vs right insertion depth disparity exceeds 1.5 mm once either side enters the 8 mm mating zone.
  - `card_dropped`: Card blade tilts excessively (>15° pitch/roll) or falls off guide tracks.
