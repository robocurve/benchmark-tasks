# Snap a circuit board out of its panel over the break edge

Separate the circuit board from its panel and tray it.

This tests registration of a V-score on a break edge and controlled downward bending while the other hand restrains the panel. The spent frame is scrap: it may stay on the break block or be laid flat on the bench, but not in the tray. It represents small-batch populated-board depaneling, associated with [O*NET electrical and electronic equipment assemblers, 51-2022.00](https://www.onetonline.org/link/summary/51-2022.00). No IPC process-compliance claim is made.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=snap_board_out_of_panel.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Separate the circuit board from its panel and tray it." \
  --scorer task_score --max-steps 1800 --epochs 2 --store-frames --log-dir ./demo-run
```

For a model trial, replace `--policy yam_scripted` with `--policy dream`. The output of `--scorer task_score` is the score.

Each trial allows 1,800 control steps (90 seconds at 20 Hz). The open-loop demo lasts 29.5 seconds, followed by the ordinary stop grace period, and uses only the known starting layout and time. `score()` reports the current end-state tier; `success()` requires 1.0, and termination requires another continuous second of success.

The frame and board are two dynamic rigid bodies with positive masses and no independent starting jitter. Their motion and separation come from robot contact, gravity and passive constraints. All geometry is original primitives built with the supplied parts kit; no third-party art assets are included.

## Rebuild the setup

Station tabletop z=0.7475 m, spanning x -0.19 to 0.91 m, y -0.65 to 0.65 m; x forward, y left; arm bases (0,+/-0.305,0.75) m. All dimensions metres and all yaw zero. Fixed grey steel break block with a flush bonded conductive-rubber bench-mat facing (included in the stated top height): origin (0.205,0.085,0.7475), box local centre (0,0,0.020), size (0.130,0.170,0.040); 130 mm straight front edge along x at y=0 (x 0.140-0.270 m); the 80 mm score line is centred on it. Dynamic green FR-4 frame, 0.040 kg, root (0.205,0.025,0.7893): substrate box centre (0,0.073,0), size (0.080,0.146,0.0016); grey hollow conductive-plastic tooling rail represented by box (0,0.130,0.0088), size (0.080,0.032,0.016). Gold copper fiducial discs at (+/-0.028,0.023,0.00095), radius 0.003, height 0.0003; black ink tooling-hole marks at (+/-0.028,0.080,0.00095), radius 0.002, height 0.0003, all z-axis cylinders (marks, not holes). Dynamic green FR-4 board, 0.030 kg, root (0.205,-0.020,0.7893): substrate box (0,0,0), size (0.080,0.090,0.0016); blue plastic terminal housing (0,-0.005,0.0083), size (0.032,0.020,0.015); black plastic component boxes (-0.027,0.025,0.0048), size (0.012,0.014,0.008), and (+0.027,0.025,0.0038), size (0.012,0.014,0.006); gold copper score stripe (0,0.044,0.00095), size (0.076,0.001,0.0003). Fixed black conductive-rubber landing nest, origin (0.205,-0.048,0.7475): floor box (0,0,0.001), size (0.092,0.104,0.002); side lips (+/-0.045,0,0.003), size (0.004,0.104,0.002); front lip (0,-0.050,0.003), size (0.086,0.004,0.002). The block face closes the back, giving an 86 x 96 mm landing pocket. Fixed dark-grey conductive-plastic ESD tray, origin (0.205,-0.215,0.7475): floor box (0,0,0.002), size (0.140,0.144,0.004); side walls (+/-0.068,0,0.010), size (0.004,0.144,0.012); end walls (0,+/-0.070,0.010), size (0.132,0.004,0.012), clear opening 132 x 136 mm. All fixed items are bench-mounted. Friction static/dynamic: block and frame proxy 0.80/0.65 (rubber-facing contact calibration; not bare steel/FR-4), board/nest/tray 0.60/0.45; contact offsets 0.0002 m, rest offsets zero, restitution zero. Board/frame joined at the V-score by a passive fixed joint, break torque 0.480 Nm and force 100 N. The two dynamic roots are never independently jittered.

Start exactly as authored, without jitter. A real panel is consumable: replace it each trial. Its edge must be deburred or otherwise guarded for safe handling. The fixed landing pocket constrains a component-up board to root x=205+/-3 mm, y=-48+/-3 mm, and yaw within 3.943 degrees; this is a geometric pickup envelope, not a claim of measured repeatability.

## Scoring

The break is judged from a 20 fps side video with a scale in view (in simulation, the 20 Hz board/frame poses); the tray tiers from the end state. There is no best-score latch. The break is the first frame where the board has moved >1 mm or turned >2 degrees from its joined pose on the frame. A valid break needs all of:

- (a) In the last joined frame, both ends of the 80 mm score line lie over the block's front edge (x 0.140-0.270 m), within 2 mm of it in y and within 2 mm of score height (z 0.7883 m, the mid-plane of a substrate resting on the block top).
- (b) No joined frame, the last one included, shows upward bend >1 degree or twist >1 degree (1.6 mm at the board's tip).
- (c) In the first separated frame and the next (consecutive frames, 50 ms apart): both score-line ends stay over the block edge (x 0.140-0.270 m) and within 2 mm of score height; frame tilt <=5 degrees; board twist <=12 degrees; the board's score end within 4 mm of the frame's score line horizontally and -1 to +45 mm below it, reaching >=2 mm below it; the board not lifted (centre never more than 1 mm above its joined height) and lower than in the frame before at least once.

Smaller attached angles than (b)'s 1 degree are not judged. Bend after the break is unrestricted: the board may drop flat. Loading direction is judged only from these two frames, so a break that completes between two frames counts. "Below the frame's score line" is measured square to the frame's substrate, which is level within 5 degrees, so a scale held against the block face reads it within 0.4%.

Score 0, permanently: no valid break; the board or the frame below the tabletop minus 50 mm (off the bench). Score 0 while the board is back within 1 mm and 2 degrees of its joined pose; returning never clears a violation. Otherwise the score is 0.5, raised by the tray tiers:

- 0.75: the board's whole box (components included) is inside the tray's 132 x 136 mm opening, component side up within 15 degrees, bottom -2 to +30 mm from the tray floor, and the frame rests flat (tilt <=5 degrees) either on the block (supported underside -2 to +4 mm from the block top, centre of mass >=20 mm inside every block edge; a 7 mm overhang can qualify) or on the bench (every substrate corner -2 to +4 mm from the tabletop, all on the table (x -0.19-0.91 m, |y| <= 0.65 m), and no part of the 80 x 146 mm substrate rectangle, at any yaw, over the tray's 140 x 144 mm outline).
- 1.0: the 0.75 conditions with the board bottom -2 to +4 mm from the tray floor, both TCPs >=30 mm from both parts' boxes, and both parts below 0.010 m/s and 0.15 rad/s for 1 s without a break (from pose changes online, reported velocities offline). The trial ends after 1 s at full score.

A flipped, on-rim or stacked board, a board more than 30 mm above the tray floor, or a frame outside the -2 to +4 mm band of the block top or bench, tilted over 5 degrees or over the tray outline, earns 0.5. A board or frame still in a gripper but inside the 0.75 limits earns 0.75. The spent frame is scrap, so it may stay on the block or be laid flat anywhere on the bench outside the tray outline.

| Score | Current condition |
| --- | --- |
| 0 | No valid break, board or frame off the bench, or separation undone |
| 0.5 | Valid break; board or frame outside the tray-tier conditions |
| 0.75 | Board whole in the tray opening, upright, bottom -2 to +30 mm; frame flat on the block or the bench; held, moving or elevated |
| 1.0 | Board bottom -2 to +4 mm, upright; frame flat on the block or the bench; released and still 1 s |

The tray test uses the complete board/component box. Moving the frame to the tray cannot substitute for the named board. Without witnessed break history, a perfect final tray layout scores zero. A pose-only scorer cannot detect a broken joint whose pieces never depart their joined relation, or destruction absent from the rigid-body model.

In the rigid-joint model the board stays attached only while it is within 1 mm and 2 degrees of its joined pose, so every attached-phase angle is small. Attached angles are judged only against the 1-degree upward-bend and twist limits, which are visible on video (1.6 mm at the tip); the loading direction comes from the visible post-break window (board end >=2 mm below the score line, not lifted, twist <=12 degrees), so the score does not depend on which 50 ms frame caught the fracture. The -2 mm side of the block and bench bands allows the up to 2 mm resting penetration of simulated contact. After fracture both ends can fall together, so their relative bend angle does not measure the applied bending moment. A ruler and an oblique view establish registration over the 130 mm edge, score height, anchor travel, supported underside height and the 20 mm centre-of-mass margin.

## Passive physics and demo

The left arm pushes the rear tooling rail with closed pads through a 30 mm stroke: 25 mm registration plus 5 mm compliance allowance. The rear-push TCP moves from y=178.4 to 148.4 mm with about 1 mm initial pad/rail gap, then backs off 20 mm, rises and presses down at the centre of the 32 mm-deep rail, supported by the 146 mm frame. The rail accommodates 7.15 mm lateral descent lag and 6.37 mm pad extent with 2.48 mm margin. Static/dynamic friction 0.80/0.65 on the block and frame represents dry conductive-rubber facing contact.

The right arm closes its pads in free space, then presses the free end through 8, 16, 24 and 38 mm overtravel stages. The centred press and frame hold oppose the bending moment. After the bend, the left arm releases before the right wrist approaches the terminal housing. The right arm rises, opens and turns 90 degrees in free space, pinches the terminal housing, rises 10 mm over 0.8 seconds to clear the nest lips, retreats 18 mm along -y over 0.9 seconds, then lifts and carries to the tray. It opens 7 mm above the nominal floor, rises and parks. Both arms finish clear for at least 4 seconds (3.2 s at 0.8x durations).

The 0.480 Nm break torque represents a residual FR-4 web 0.671 mm thick: M=sigma*b*h^2/6 with effective flexural stress 80 MPa and width 80 mm in the 1.6 mm substrate. This scalar threshold simplifies fracture and component stress; the history check rejects off-axis failure. The threshold is 3.2 times a 0.150 Nm pre-bend load envelope; gravity contributes about 0.0131 Nm.

At measured vertical stiffness 290 N/m and 1.4 mm lost travel, the 38 mm press gives an analytical force forecast of 10.614 N. With a conservative 70.7 mm lever, that is 1.563 times the break torque (required force 6.789 N). The 24 mm frame preload at the same stiffness supplies 6.554 N at a lever of at least 120 mm, a 1.639 torque margin. Static mat capacity is 5.557 N versus a 2.146 N lateral estimate using 8.0/7.15 mm offsets and 5 mm/N sideways compliance. The minimum commanded press TCP height is 764.1 mm, with 2 mm conservative front-lip clearance. The maximum press leaves the pad bottom 1.758 mm above the landing floor and outside the front lip. The 30 mm registration stroke budgets 3.846 mm elastic travel at 0.5494 N friction within its 5 mm allowance.

The uniform-collider centre of mass stays 44.994 mm behind the break edge during registration. Block height gives a 24.975-degree free-end floor intercept. The terminal pinch at yaw 90 degrees has 11.842 mm pad overlap, 2.758 mm fingertip/nest vertical clearance and 27.656 mm nominal open-pad lateral allowance after landing scatter. The 96 mm pocket leaves 6 mm total longitudinal room around the 90 mm board, so the shallow rise clears the lips before retreat.

## Reset

At every reset, the joint is disabled before both body poses and velocities are restored. The joined constraint frames and 0.480 Nm/100 N limits are restored, and the joint is re-armed before the 0.5-second settling period. Break history, permanent violations and stillness memory are cleared for the new episode. For physical trials, replace the consumable scored panel and deburr or guard its edges.

## Scene and demo parameters

Every authored shape centre, size, radius, height, mass, colour, friction and starting coordinate is listed in the setup above. Their purposes are: 80/90/146 mm substrate sizes represent a small PCB and handling rail; 1.6 mm is substrate thickness; 32 mm-deep rear rail permits a closed-pad side push; 15 mm terminal height provides a visible post-break grip; component asymmetry makes the component side visible; 3/2 mm discs are visible fiducial/hole marks, 0.3 mm thick; the 1 mm score stripe locates the joint without introducing a physical hinge. Green (0.08,0.40,0.17), blue (0.05,0.36,0.80), grey (0.46,0.50,0.55), black (0.045,0.06,0.07), gold (0.90,0.67,0.12), and tray (0.20,0.23,0.30) distinguish objects in the top camera. Marks are small colliders in the rigid proxy.

| Numbers | Reason |
| --- | --- |
| x=205 mm; tray y=-215 mm; parks (100,+270)/(100,-300) mm | Work envelope within 360 mm of each base, separate wrists and clear camera |
| 40 mm break-block height; 130 x 170 mm top | Bend clearance, stable support, low fall into catch nest |
| 25 mm initial score offset; 1 mm initial vertical gap | Visible registration step and settle clearance |
| 92 x 104 x 2 mm nest floor; 86 x 96 mm opening; 2 mm lips | Bound flat-board pickup scatter without obstructing open fingers |
| Tray 132 x 136 mm opening; 4 mm floor/walls; 12 mm wall height | Contain board while admitting 94 mm open pads |
| 40/30 g roots; friction .80/.65 and .60/.45; 0.2 mm contact offsets | Small panel masses; slide on rubber-faced steel, dissipative ESD catch; thin-board contacts |
| 0.480 Nm / 100 N joint limits | Select bending at usable arm force; resist ordinary translation |
| 38/24 mm overtravel; 290 N/m; 1.4 mm tracking loss | At least 1.5x press and hold force budgets |
| Conservative press lever 70.7 mm, hold lever 120 mm | Include finite fingertip footprint in moment budget |
| 0.150 Nm pre-bend envelope; 80 MPa effective stress | Pre-bend load margin; rectangular-web estimate |
| TCP hold x=-10 mm, y=130 mm; press x=0 mm, y=-84 mm relative to X | Contact substrate/rail and keep wrist centres apart |
| Rear push TCP offset 7.4 mm; 30 mm stroke | Mesh-derived 6.37 mm tip reach, approach gap and compliance allowance |
| 135 mm travel / 160 mm park above table | Lift component housing clear within IK height envelope |
| Pickup TCP substrate-top+16 mm; release +7 mm; yaw90 | Clear nest lips and block while gripping terminal; gravity seating |
| 1 mm / 2 deg latch; endpoints over the 130 mm edge (x 0.140-0.270 m); 2 mm edge and height tolerance | Detect departure and register score line on the edge unambiguously |
| Joined samples, last included: upward bend or twist >1 deg invalid | Visible wrong-sense loading (1.6 mm at the tip); ignores handling transients and sampling phase |
| First separated frame and the next: board end >=2 mm below the frame's score line, not lifted (<=1 mm), twist <=12 deg | Confirm downward separation even when the detached board drops flat |
| 4 mm horizontal anchor error; -1..45 mm downward travel; 1 mm upward allowance | Reject pulling/upward bend while allowing immediate free fall |
| 5 deg frame tilt; 20 mm CoM inside all block edges; 4 mm supported underside height tolerance | Frame supported on block, signed contact tolerance |
| Frame on bench: 5 deg tilt, every corner -2..+4 mm from tabletop, on the table (x -0.19-0.91 m, \|y\| <= 0.65 m), substrate rectangle at its yaw clear of the 140 x 144 mm tray outline | Scrap frame may be set down flat; not in the tray |
| Sticky loss: board or frame below tabletop minus 50 mm | Parts knocked off the bench stay zero |
| Tray up <=15 deg, bottom -2..+4 mm (full) / +30 mm (staged) | Upright seated full credit, meaningful held/moving partial tier |
| Release 30 mm, speeds 10 mm/s and .15 rad/s, still 1 s, success 1 s | Observable release and uninterrupted rest |
| 20 Hz, 90 s / 1,800 steps | Policy time budget; demo plus hold below 60% |
| Demo 1.5 s initial parks; 1/.7/.6/.5 s approach, backoff and waits; 1.5 s slide; .8 s hold approach; 1 s preload | Slow contact registration and enough time to settle |
| 4.3 s left hold; 1 s board approach and touch; four .55 s bend stages, .5 s dwell | Both hands react bending; left clears before terminal approach |
| .7 s rise/open, 1 s hover/descend, .7 s pinch, .8 s shallow rise, .9 s retreat, 1 s lift, 1.6 s carry, 1.1 s lower, .7 s release, 1 s rise/park | Gentle transport with distinct release and retreat |
| 10 s left clear and 4 s right clear, plans extended equally; 1.0x/0.8x durations | Stable final observation and faster robustness check |
| 32 position / 4 velocity solver iterations, 1 m/s depenetration, .05 linear/angular damping | Stable rigid-body contacts |

## Pickup clearance

A continuous swept-volume calculation covers all five board shapes and six frame shapes over landing root x +/-3 mm, y [-51,-45] mm, yaw +/-3.943 degrees, a 3-degree pinch tilt cone, tracking/slip +/-4 mm horizontally and +/-1 mm vertically, and up to 3.152 mm initial root elevation from resting tilt. Frame score endpoints span +/-2 mm from the edge and score height, with frame tilt up to 5 degrees. Rotation displacement is bounded by 2*r*sin(tilt/2); smoothstep motion is bounded continuously, with frame shapes sliced along y and x separation excluded from the distance bound.

Minimum board/frame clearances are 9.871 mm during shallow rise and retreat, 6.027 mm during main lift, 69.731 mm during carry and 155.027 mm during lowering. The 6.027 mm path minimum exceeds 3 mm including the 0.4 mm combined contact offsets. Before retreat, all board shapes clear the nest lips vertically by at least 3.848 mm. These bounds describe geometric clearance within the stated pose envelope; they do not model grip retention or dynamic tracking outside it.
