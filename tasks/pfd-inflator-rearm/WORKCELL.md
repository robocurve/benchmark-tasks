# Inert service trainer and grading

The bench represents the handoff between inspection of a fired manual inflator and garment repacking. A spent cylinder can engage both bayonet shelves. A technician can check the seal, reject a spent spare, observe the mechanical status window and leave the assembly supporting its own weight. The MD3075 manual and MA7203 instructions describe the seal and indicator checks, reset lever and bayonet fitting. The mixed-return stock, two bins, visible witness stripes, upright clamped body and sensing lands are features of this training fixture. The original instructions control work on actual equipment. [MD3075, printed pp. 6–8](https://www.machovec.com/mustang/documentation/manuals/md3075_10aug2011.pdf#page=8) · [MA7203 illustrations](https://www.machovec.com/mustang/documentation/manuals/ma7203_05oct2011.pdf)

## Parts and reset

All dimensions below are millimetres. Coordinates are in the calibrated station frame. The table top is at Z 747.5. The assembly drawing uses local X/Y 0/0 at station X/Y 310/−70 and local Z 0 on the table. Open trainer.scad with OpenSCAD. Its part selector displays the frame, lever, indicator follower and each dummy cylinder separately. The drawing gives nominal solid geometry and placement. Provide the pivot pin, return spring, linear guide and mounts to suit the chosen fabrication process. Actual fit and spring return require a bench check.

| Part | Nominal geometry and reset |
|---|---|
| Jacket surface | Immobilised orange textile panel 240 × 195 × 4 and 50 × 290 × 1.5 webbing. Centre at the socket. |
| Socket and frame | Rigid polymer or machined frame anchored to the tabletop. Two opposed banks of 9 × 2.7 × 6 shelves, centred at radius 12.4 and station Z 935. A segmented guide ring surrounds them at radius 15.8. Two stops limit the locked rotation. |
| Three inert cartridges | Unpressurised metal dummies of mass 102 g each, Ø25.4 body, 139 metal height, Ø10.4 neck, two opposing retaining lugs. Match the external finish and opposed stripes. Two pierced faces each have a Ø2 × 2.8 recess and a lower sensing land. The intact face is flat with the taller land. All three have a represented seal ring. Use dummy parts throughout. |
| Lever | Metal or rigid polymer rocker, pivot at (287, −70, 952), axis Y, stops at −32° and 0°. Light return spring biases the lever to 0°. The lever toe obstructs the tall land until raised. |
| Status window | Freely guided 6 g follower, 12 vertical travel. Twin feet touch the cartridge land. Red and green bands move behind a fixed mask. The sensing land transfers the trainer's seal condition into a window indication. |
| Supply | Two 60 × 50 × 9 grey rigid pads at X/Y (340, −190) and (420, −267). Set the intact spare upright on A and the pierced spare on B. |
| Return containers | Grey collection cup centre (203, 14), clear inside 46 × 46. Red reject tray centre (210, −155), clear inside 168 × 162. Both have 4 thick, 18 high dividers, with the table as the floor. Both start empty. |

Reset the pierced original in the socket on both shelves with the witness marks aligned, window red and lever resting at its stop. Park both arms at the supplied YAM ready posture. For a dry bench check, each dummy must rotate to the index and remain suspended after release. Check that the raised lever clears the intact land and, after settling, the intact dummy gives green and a pierced dummy gives red. Bench calibration must preserve both outcomes even though either dummy fits the socket. The simulation uses gravity, friction, rigid contacts, a sprung hinge and a vertical slider. Gas charge, leak testing, separate seal-ring inspection, bladder and packing are outside this exercise.

## End-of-episode grading

Use the window, witness marks, a millimetre scale and two photographs or video views one second apart, with continuous observation for any intervening grasp or motion. Examine both sides of the bayonet under the load of the hanging dummy. For the bins, check that the entire projected cylinder is inside the inner boundary and rests on the floor. A cylinder in a bin may lie on its side when it fits. Check the spare identities from their seal faces. The full handoff and safe isolation are mutually exclusive terminal states.

| Score | All conditions for the final uninterrupted second |
|---|---|
| 1.0 | Removed original in grey cup, pierced spare in red tray, intact cylinder hanging from both shelf banks, marks within ±6° of either opposed locked index, window green, hands clear of the cartridges and moving mechanism. |
| 0.5 | Pierced spare in red tray, intact spare upright on a supply pad, window red, original still locked on both banks or fully collected in grey cup, hands clear. |
| 0.0 | Every other ending, including the untouched initial state, a mechanically locked spent spare, unsupported work, held parts or an unstable ending. |

The simulator reads the two actual stock meshes to identify the recessed face. At 240 Hz it checks both independent shelf contacts, no table/pad support, bottom more than 50 above the table, lug-centre radial error below 3, cylinder axis within arccos(0.99) of vertical and index error at most 6° modulo 180°. The window is green at 2.8–4.6 lift with the intact cylinder contacting the follower. Red is below 1 lift. Gripper and arm contacts with any monitored moving body invalidate release. Across at least 241 consecutive physics samples the intact lug centre, original base and rejected spare base each remain within 1 of their reference position. Grading uses the episode end and permits any grasp, arm order or observation strategy. Green alone earns zero.
