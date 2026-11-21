# Adhesive realism requirements — revision

Reference: SIHL PRESSURETAC BT 28/3, https://sihl.com/en/products/pressure-tac-bt-283/ and https://sihl.com/en/pressuretac-airline/ (checked 2026-11-19).

Manufacturer facts: linerless coating bonds only coating-to-coating; form a loop, press ends together, smooth out. Grammage 177 ± 8 g/m²; paper thickness 170 ± 12 µm. These are reference product properties, NOT current simulation properties. The current 1 mm, 2 g rigid panels are a coarse discretization and do not reproduce this stock.

The inspected manufacturer page does not specify required application force, dwell, peel strength or shear strength. Its paper tensile strength must not be treated as adhesive bond strength. Generic VHB tape application pressures are not applicable evidence for this product.

Current 1 N / 0.4 s gate and 8 N / 0.15 Nm joint limits are uncalibrated simulation parameters. A breakable fixed joint does not reproduce pressure-dependent bond growth or peel mechanics. Contact force in newtons is not pressure in pascals; geometric overlap is not measured real contact area. Do not label force divided by geometric overlap as measured adhesive pressure.

Release gate: real compressive contact on matching coated faces, no proximity-only bond, no adhesive attachment to gripper or suitcase, no gross interpenetration, and released stable loop. Audit all label panels, not just two end panels. Validity requires corrected contact mechanics before a new complete demo. Do not count scripted pose placement or force-free welding as evidence.

Expanded audit invalidated endpoint-press-032 as a successful physical manipulation: panel01 midplane penetrated a right finger by approximately 3.839 mm, although endpoint contact forces were recorded. Correct the approach and adjacent panel clearance before further release tuning. Quantitative adhesive fidelity requires product testing/data; until then describe the model explicitly as an approximation.

## Robot pressing motion
The receiving gripper must open around both aligned coated end faces, then close along the sheet normal. The supporting gripper remains on the other end, outside the receiving fingers. No arm-to-arm collision may provide the apparent press. Record the actual contact force on the designated patches during closure and dwell; separately verify release, retained loop, outward barcode and both grippers clear. Contact force is not a calibrated contact-pressure field.
