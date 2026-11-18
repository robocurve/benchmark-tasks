# portion-disher-level-and-sweep-release build deviations (2026-11-18)

1. Handle grip geometry: replaced the under-handle hanging lug with an inline elastomer T-pommel grip sleeve and cheek guards at `GRIP_Y = -0.195 m` (`Z_MOUTH = 0.104 m`), enabling a clean vertical Y-pinch grasp without colliding with the handle neck or exceeding right-arm IK reach.
2. Rack and wiper orientation: mirrored the flanged thumb tab to the `+Y` (left-arm) side (`TAB_Y = +0.0725 m`), oriented the wiper hinge along `+Z` (`0..180 deg`), and placed the rack's detent block at `+0.015 m` in Y so the detent physically blocks the wiper drive arm at ~20 deg until the rack is drawn `>= 27 mm` along `+Y`.
3. Thumb rack return spring softened from ~12 N (`400 N/m`) to `60 N/m` (`1.8 N` at `30 mm` with `1.0 N` Coulomb friction) so compliant bimanual arms can draw the rack cleanly while steadying the suspended disher.
4. Disher body origin centered at `(DX, DY, TOP + 0.128)` to coincide with the cohesive mash plug origin so the breakable bond carries zero cantilever moment arm during transit.
5. Scoring uses purely observable physical end-state geometry (`wiper_sweep` angle `sweep_deg`, mash position, vessel marks, disher rest pose, and gripper TCP clearance) with no mutable episode counters or test backdoors.
