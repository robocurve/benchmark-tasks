### Task Overview

In commercial restaurants, juice bars, hotel pantries, and high-volume catering kitchens,
heavy-duty commercial manual citrus presses (such as the industry-standard Hamilton Beach
Commercial 932) are the primary equipment for fresh citrus extraction. Under FDA Food Code
§4-602.11 and NSF/ANSI commercial sanitation standards, food-contact juice extraction
components must be disassembled, sanitized, and reassembled between operational shifts to
eliminate bacterial accumulation and cross-contamination.

This task benchmarks bimanual robotic manipulation for precision multi-mechanism reassembly of
a commercial citrus press:
1. Alignment and insertion of the stainless steel juice collector funnel into the 88 mm base
   receiver collar ring.
2. Clockwise bayonet rotation to latch the collector bowl's locking tabs into the cast base
   detents.
3. Coaxial insertion and flush seating of the perforated stainless steel reamer strainer cone
   onto the bowl's internal annular shelf.
4. Mechanical clearance test stroke: depressing the sprung press lever downward past 35° (from
   its 85° spring-rest position) to verify clearance between the upper compressor plunger and
   the seated strainer cone, and allowing the lever to spring return to its rest stop (>= 55°).
5. Retracting both manipulator arms clear (>= 25 mm clearance) and maintaining stationary
   stability for 1.0 second (20 consecutive control steps at 20 Hz).

### Occupational Relevance

- **Primary SOC**: `35-2021 Food Preparation Workers` (1,024,400 US jobs, BLS OEWS May 2023).
- **Secondary SOC**: `35-3023 Fast Food and Counter Workers` (3,546,800 US jobs, BLS OEWS May
  2023).
- **Workforce Scale**: Over 4,500,000 US workers regularly operate, sanitize, and maintain
  commercial food and beverage preparation machinery.
- **Physical Manipulation**: Coordinated bimanual manipulation requiring axial insertion,
  rotational bayonet locking, concentric conical nesting with sub-centimeter tolerances, and
  kinematically constrained lever actuation.

### Mechanical Components

1. **Cast Iron Base Frame (`juicer_base`)**:
   - Heavy cast horseshoe base resting securely on prep table surface at `(0.22, 0.00, TOP)`.
   - Upright column spine providing vertical mounting for collar arm and lever pivot.
   - Receiver collar ring with 80 mm inner opening and internal bayonet detent stops.
2. **Juice Collector Bowl (`collector_bowl`)**:
   - Stamped 304 food-grade stainless steel funnel (80 mm outer width, 45 mm height, mass 0.045
  kg).
   - Bayonet locking tabs on rim and rectangular grip post on -Y for twist lock.
   - Hollow bottom drainage spout nozzle and internal annular support shelf ring (54 mm outer
  lip, 36 mm hollow center opening).
3. **Perforated Strainer Cone Insert (`strainer_cone`)**:
   - Machined 304 stainless steel strainer cone insert (mass 0.035 kg).
   - 48 mm outer perimeter seating flange matching internal bowl shelf.
   - Perforated strainer basket (32x32x10 mm) hanging cleanly into funnel cup.
   - Prominent central apex grasp knob (28x18x52 mm) for collision-free parallel grasp above
  the rim.
4. **Sprung Press Lever & Plunger (`press_lever`)**:
   - Chrome-plated cast lever bar with top grip knob.
   - Upper stainless compressor cone cup (60 mm diameter).
   - Revolute hinge pivot with spring return (stiffness 1.2 N·m/rad, damping 0.3 N·m·s/rad,
  rest at 85°).

### Scoring Breakdown

- **0.25**: Collector bowl placed within receiver collar opening (horizontal offset <= 25 mm,
  vertical offset <= 20 mm).
- **0.50**: Collector bowl bayonet rotated clockwise into locking detents (yaw <= -15° or >=
  165°, dr <= 20 mm, dz <= 15 mm).
- **0.75**: Strainer cone coaxially nested flush inside collector bowl (radial offset <= 20 mm,
  vertical gap <= 22 mm).
- **1.00**: Press lever stroked past 35°, returned to rest stop (>= 55°), both arms cleared >=
  25 mm, and held stationary for 1.0 s (20 consecutive control steps at 20 Hz).
- **0.00 (Fail)**: Base tipped or displaced > 25 mm, or components dropped off bench.

### Validation & Qualification Runs

- **Scripted Demo 1 (`demo_1`)**: `1.0` in 530 steps.
- **Scripted Demo 2 (`demo_2`)**: `1.0` in 530 steps.
- **Negative Control No-Op (`ctrl_noop`)**: `0.0` in 260 steps.
- **Negative Control Bowl Only (`ctrl_bowl_only`)**: `0.25` in 408 steps.
- **Negative Control Bowl Locked (`ctrl_bowl_locked`)**: `0.50` in 472 steps.
- **Negative Control Cone Seated (`ctrl_cone_seated`)**: `0.75` in 692 steps.
- **Negative Control No Release (`ctrl_no_release`)**: `0.50` in 596 steps.
- **Dream-1 Baseline (`dream_1`)**: `0.0` in 600 steps.
- **Unit Tests**: 10/10 offline unit tests pass cleanly (`test_scoring.py`).
- **Media ID**: `M-HF8JSU` (contains genuine top-camera H.264 `demo.mp4`, `model_run.mp4`, and
  `setup.png`).
- **Novelty**: `robocurve-pr-check` status 100% CLEAR across all 3,212+ PRs. Zero collisions,
  zero precedents.


