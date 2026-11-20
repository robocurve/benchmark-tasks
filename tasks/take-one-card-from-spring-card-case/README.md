# Take one membership card from a spring-backed card case without spilling the pack

A receptionist, cashier or visitor retrieves a single plastic membership card from a compact spring-backed desktop card dispenser without spilling or double-feeding the remaining cards.

## Task Overview

- **Category:** Societal relatability
- **Domain:** Hospitality / retail / office customer service (ID / loyalty card distribution)
- **Mechanism:** Spring follower under a 5-card stack pushes against a top thumb window with an exact 1.15 mm single-card outlet throat. High friction against the top PVC card advances exactly one card forward while the 1.15 mm throat blocks the second card (1.52 mm two-card interference). The second arm stabilizes the loose case on its desktop riser.

## Reproducibility and Fabrication

The `fabrication/` subdirectory provides a complete, reproducible fabrication package for physical testing and 3D printing:
- `case_dimensions.svg`: Orthographic dimensioned drawings (top view, centerline section A–A with 1.15 mm throat detail, bench fixture coordinates).
- `fabrication/case_base.stl`: 100 x 64 x 14 mm ABS lower housing with 10.85 mm front dam and 2x spring guide posts.
- `fabrication/case_lid.stl`: 100 x 64 x 2 mm ABS upper frame with 66 x 40 mm thumb window.
- `fabrication/follower_plate.stl`: 84 x 52 x 1 mm 6061-T6 aluminium follower plate.
- `fabrication/desk_riser.stl`: 50 x 38 x 50 mm hardwood riser block.
- `fabrication/BOM.csv`: Bill of materials including standard commercial items (ISO/IEC 7810 ID-1 CR80 PVC cards, music wire compression springs k=1.25 N/mm each / 2,500 N/m pair, M2 screws).
- `fabrication/ASSEMBLY.md`: Step-by-step assembly and setup instructions.

## Running and Scoring

Execute a trial with:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/take-one-card-from-spring-card-case/task.py \
  -E object_jitter=0 \
  -E control_hz=20 \
  -E cam_width=320 \
  -E cam_height=176 \
  --instruction "Take one membership card out of the blue card case and leave it on the cream mat. Leave all the other cards inside the case and let go." \
  --scorer task_score \
  --max-steps 1200 \
  --seed 0 \
  --store-frames
```

## Scoring Criteria

- **0.25:** Exactly one card protrudes >= 20 mm beyond the front outlet while all other 4 cards remain inside the case.
- **0.75:** Exactly 1 card rests level on the cream output mat, with all other 4 cards remaining inside the case.
- **1.0:** The 0.75 condition is maintained with all parts stable (< 5 mm/s linear velocity, < 0.12 rad/s angular velocity) and both robot grippers released and clear (>= 15 mm clearance) for at least 1.0 s of continuous fresh observation.
- **0.0:** Any spilled card, damaged pack, wrong card count, or missing/stale data.
