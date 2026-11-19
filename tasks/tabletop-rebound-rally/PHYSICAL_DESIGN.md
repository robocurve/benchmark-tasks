# Physical design: tabletop rebound rally

A custom tabletop practice board returns a real-size table-tennis ball to a held paddle. It uses the unchanged stock YAM station table, arm bases, room and cameras. It is a practice drill, not a regulation-table game. The physical build and contact parameters are not qualified on hardware; simulation and model results are recorded in the README.

## Board and support

The [SANWEI rebounding board](https://sanweisport.com/en/product/accessories/others/rebounding-board/) specifies 15 mm MDF and optional table-tennis rubber. This design independently chooses a 600 mm wide × 300 mm high × 15 mm MDF panel with a 2 mm rubber face. It is not a dimension-exact SANWEI replica. The source's reported weight and dimensions are ambiguous; they are not used as a panel mass.

| Component | Calculation | Estimated mass |
|---|---|---:|
| MDF panel | 0.600 × 0.300 × 0.015 m × 700 kg/m³ | 1.89 kg |
| Rubber face | 0.600 × 0.300 × 0.002 m × 1,000 kg/m³ | 0.36 kg |
| **Panel and face** | Sum; supports excluded | **2.25 kg** |
| Supports | Two aluminum feet, each 160 × 40 × 30 mm, plus two M6 mechanical clamps | Separate from panel mass |

Densities are design estimates, not manufacturer measurements. Feet and clamps anchor the board to the station table; they are not included in the 2.25 kg panel figure. A rectangular layer's principal inertia about its centre is m/12 times the sum of squares of the other two side lengths; combine both layers with the parallel-axis theorem if the board is simulated dynamically. Here the board is fixed and uses a single 17 mm assembly collision envelope; the 2 mm rubber face is represented by contact properties rather than separate compliant geometry. See [dimension drawing](physical_design.svg) and [physical ledger](physical_design.json).

## Paddle and ball

[STIGA Royal Carbon 5-Star](https://www.stigasports.com/en/product/royal-carbon-5-star) lists 170 g in its description and 165 ± 5 g in its specifications; 170 g is the upper end of that stated range and is the chosen paddle mass reference. The simulated paddle is a separately specified round-blade wood/rubber approximation, not a dimension-exact or carbon-layup reproduction: a 156 mm diameter × 7 mm wood blade, two 154 mm diameter × 3 mm rubber faces, and a 110 × 26 × 26 mm wood handle. Approximate component masses are 50.842 g blade, 45.451 g per face and 28.257 g handle; the total is 170 g. The assumed wood density is 380 kg/m³ and effective rubber density 813.37 kg/m³. The ledger places centre of mass 108.392 mm from the handle origin along the blade direction and gives the inertia tensor.

The ball is 40 mm diameter and 2.7 g, following [ITTF Laws 2.3](https://documents.ittf.sport/sites/default/files/public/2026-02/2026_Statutes_v1_consolidated_clean.pdf). Its thin-shell approximation has each principal inertia 7.2 × 10⁻⁷ kg·m² and a rigid spherical contact envelope.

## Fidelity and limits

The tabletop board, paddle and ball require a physical build and bounce tests; numerical restitution and friction are estimated, not measured source properties. The rigid simulation omits rubber and shell deformation, wear, air drag and spin lift. Stock station cameras and supports must stay unchanged, and reproducible reset must specify the initial ball and paddle poses and fixed-board mounting. No hardware validation or submission readiness is claimed. The small playing surface is an explicitly described tabletop practice setting, not a claim to reproduce regulation-table distances or competitive play.
