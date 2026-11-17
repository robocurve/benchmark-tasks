# Clasp and physical reset

The complete [fabrication specification](fabrication/BUILD.md), [editable CAD](fabrication/clasp.scad) and [dimensioned drawing](fabrication/clasp-drawing.svg) define the two spring leaves, grooved pin, brass housing, fasteners, assembly and manual reset. The spring stock is sourced by grade and part number. This is a fabrication design, not a claim of a physically tested clasp.

Use a push-in pin-and-socket spring-detent clasp on a 10 mm leather bracelet.
A magnetic clasp, lobster clasp or screw clasp is not an equivalent. The
benchmark dimensions below define the equivalent mechanism; they are not
attributed to an unmeasured retail product. The simulated housings have flat
sides for the ordinary parallel gripper pads to contact.

| Part | Dimensions and function |
| --- | --- |
| Leather band | 195 mm exposed length (199 mm cut length, including two 2 mm hidden mounting tabs), 10 mm wide, 2 mm thick, plus the two metal housings |
| Each metal grip housing | 10 mm long, 12 × 7 mm outside cross-section; flat opposed grip faces |
| Male pin | 3 mm exposed length, 2.2 × 2.2 mm square section, beyond a 3 mm long, 5 × 5 mm stop collar |
| Female entrance | 4 × 4 mm clear opening, centred on end face; at least 6 mm clear bore depth |
| Catch | Spring-detent engagement begins after 2.5 mm of pin entry; the shoulder prevents free withdrawal |
| Closed spacing | Housing centres approximately 13.5 mm apart; facing housing ends approximately 3.5 mm apart (including the 3 mm stop collar) |
| Band attachment | Each leather end secured inside the rear of its housing; no robot attachment, tabs or auxiliary handles |

The clasp must be pushed along its bore axis. Sideways overlap, touching the
outside faces or putting the pin beside the housing cannot close it. A simple
spring detent may be released manually for reset; release is not part of the
robot's task. Before a hardware series, check five hand closures, each followed
by suspending a 100 g mass from the closed band for 3 seconds. Reject a clasp
that fails to engage or opens in that check; record its identity and dimensions.
This is an operator preparation check, not an extra action for the robot.

The simulator uses colliding bore walls and pin geometry. Its simplified catch
creates a clasp-to-clasp constraint only after measured insertion reaches the
stated engagement depth. It never connects either end to a gripper. The passive
band joints transmit loads to the clasp bodies; gripper forces, slippage and
missed insertion are resolved by PhysX. Its 30 N break-force and 0.20 Nm break-torque settings are engineering approximations, not manufacturer specifications.

For reset, open the clasp manually, put the 220 × 54 mm rigid silicone wrist
form on two fixed rigid supports (25 x 80 x 25 mm), and lay the unfastened bracelet below its midpoint, with both ends folded back.
Form centre: (340, 0, 800) mm. Support centres: (235, 0, 760) and (445, 0, 760) mm. Fix the form ends to the supports and the supports to the table so the form cannot roll or translate. Clasp centres: (340, +76.8, 761) and
(340, −76.8, 761) mm in the simulated frame. Both pin and bore axes point in
+Y; the female opening faces −Y. The pin is completely outside the socket.
Restore the setup photograph, inspect the leather and catch, and open both
robot hands. There are no pins, strings or concealed supports holding the
clasp ends during the trial.

For partial credit, insertion depth is the 3 mm exposed pin length minus the
remaining gap between the stop collar and the female housing face. A close photograph with a
ruler therefore measures it without looking through the metal housing.
A catch that opens again, or a damaged band, cannot retain earlier credit.

The leather leaves the **rear** of the female housing (+Y) and the rear of the
male housing (−Y). Each end turns back through a loose bend of approximately
5 mm radius in the vertical plane; the middle span passes below the wrist.
The band is not initially around the form. `bracelet_geometry.py` gives the
205 mm housing-centre path, bend positions and rear attachments. Allow it to settle
under gravity before starting and match the setup image; do not hold the bends
with a clip, adhesive or string. The native reset includes one second of
passive settling with open hands before recording.

Two fixed acrylic rests, each **6 × 10 × 10 mm**, support the clasp housings
10 mm above the tabletop so the gripper pads can enclose their sides. Centres:
(340, +76.8, 752.5) and (340, −76.8, 752.5) mm. Secure the rests to the table;
the clasp bodies merely sit on them and must lift freely. Nothing attaches
the bracelet to a rest or hand. Include the rests in every physical trial.

The native band panels weigh 0.10 g per 5 × 10 × 2 mm panel (nominal
1,000 kg/m³ leather density); each housing/catch assembly is approximated
as 6 g. These are engineering values, not measurements of a retail product.

The 12 mm housing width exceeds the 10 mm band width, allowing the finger
pads to grip the metal without pinching the leather behind it. No enlarged
robot-only handle is added; this is the clasp housing itself.

The 195 mm exposed leather span joins the rear faces of the two 10 mm housings.
Each fold begins outside the metal, at least 5 mm behind its housing centre;
the band must not pass through the housing or socket.

The 5 mm square collar is wider than the 4 mm bore and physically stops
insertion at 3 mm. The spring catch engages at 2.5 mm, leaving approximately
0.5 mm clearance to the collar at engagement. This is a specified equivalent
clasp mechanism, not an unverified claim about a retail product.

The reduced catch permits axial twist and up to 8° swing rather than locking
all rotation. The 2.2 mm square pin fits inside the 4 mm square bore at every
axial angle. Native catch-break events invalidate the trial.
