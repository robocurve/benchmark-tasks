# Scene design: wet-ivc1 (redesign 3, 2026-11-15)

Station coordinates: `TOP = 0.7475 m`, `+X` forward (toward the rack), `+Y` left, `+Z` up. Generic product names;
sources in README "Real mechanism" and "PROVENANCE".

| Object | Part type | Dimensions (mm) | Mass (g) | Material | Start position (m) | Source |
|---|---|---|---|---|---|---|
| `mat` | fixed | 400 x 260 x 2 | fixed | blue-slate silicone, grip-dot texture | (0.200, 0, TOP) | bench mat, authored |
| `cage` base | loose | 391 x 199 x 160 (148 lower tub, flanges z 125-135, handle rails z 135-156) | 600 + 2 x 10 valves = 620 (estimate, unverified) | clear polycarbonate (IOR 1.586), corncob bedding, wire-bar lid, nest, card | (0.210, 0, TOP + 0.002) facing +X | mouse IVC cage spec sheet (391 x 199 x 160 mm, 172 mm with bottle) |
| `lid` filter top | loose | 376 x 164 x 25 shell, PTFE filter sheet under a clear grid; plug skirt 1 mm inside the 368 x 140 tub mouth with chamfered lead-in (6 mm sides, 13 mm ends); 70 x 80 grip spine between two side-handle pockets; 88 x 80 bottle bay | 120 | clear polycarbonate, white PTFE | flat on the bench, skirt down, centred (0.245, 0.215) | radial silicone gasket, latch-free (maker spec sheet) |
| gasket | part of base | 384 x 160 ring, 6 mm band at z 133-139 | - | translucent silicone | on base rim | maker spec sheet |
| `cage.valve_l/r` | X slides -8..0 mm | 12 x 12 mm pistons, protrude 10 mm, y +-50 mm, z 150 mm (cage frame) | 10 each | stainless piston, silicone face, grey housing | 0 (closed) | US 4,343,261; US 5,044,316; PubMed 24602544 |
| `bottle` | loose | 55 x 55 x 129 | 180 | clear polycarbonate, water (IOR 1.333), blue cap | (0.255, 0, TOP + 0.0995), pocket floor 33 mm | IVC 260 mL water bottle dims (brochure) |
| `rack` bay | fixed | bay 216 wide (inner walls y +-108), walls 278 high x 435-825; runners 17 x 410 (x 405-815) top z 145, inner edges y +-91; cell floor 4; rear stop face x 808 (y +-30, z 110-155); plenum wall x 817-833; nozzles r 5 at y +-50, z 170, tips x 811; header bar x 455-530 at z 276 | fixed | brushed 304 stainless, anodized slate header, silicone nozzles, rubber bumpers | centred (0.610, 0, TOP) | IVC rack bay class; nozzle/valve layout per PubMed 24602544 |
| bench props | visual only | real SimReady models | - | PBR | beside the mat, out of the arm paths | nvidia/simready-assets (README) |

## Mechanism checks (measured in mj_check MuJoCo, `runs/.../redesign3/mj_check-3b.json`)

- Filter top: the left arm squares it on the bench (both open fingers push its front end back), pinches the spine at
  its centre of mass, lowers the skirt into the tub mouth 6 mm behind register and releases; the chamfers seat it
  (end 0.5-0.9 mm from register over the 7 jitter/friction runs; sealed threshold 1.5 mm).
- Valves: docked nose on the stop (gap 0.0 mm) -> pistons in 6.7-7.0 mm (open threshold 5 mm, travel limit 8 mm).
  Spring push 2 x (0.7 - 0.3) N is below the runner friction, so the cage stays home after the arms leave.
- Runners: hanging cage underside at TOP + 20 mm, 16 mm above the cell floor.
- Reach: worst IK 0.085 mm over the whole plan; no arm-arm or arm-table contact. A straight-down grip over a docked
  cage is 224-304 mm out of reach (TCP 0.10-0.20 m, official IK), so the top goes on while the cage is on the bench.
