# Rotary vise and splitter build

The two-sheet drawing shows mounting datums and a scaled section of the catch entering, lifting and withdrawing. The STEP assembly opens at the parked position, with numbered components in millimetres. The catch reaches the near turn 7.9 mm behind the red tip to leave room for the key bow. Verify fit and opening force on the physical ring lot before grading. [Assembly and opening sequence](fixture-drawings.pdf) · [Machinable STEP solids](fixture.step) · [Simulation validation](../VALIDATION.md)

## Ring and parts list

Use **RS PRO 866-579**, US stock **70658568**, nickel-plated carbon spring steel. RS lists 38 mm OD, 33.9 mm ID, 1.98 mm wire and 3.7 mm overall thickness. The commercial pack contains 20. The simulated round-wire approximation gives 34.04 mm ID and 3.96 mm stacked thickness. The modeled 340° overlap, tip rounding and stiffness are design assumptions. Check resting clearance and cut-end profile on the lot. [Manufacturer dimensions, page 2](https://docs.rs-online.com/0117/A700000007926496.pdf) · [US ordering page](https://us.rs-online.com/product/rs-pro/866579/70658568/) · [UK ordering page](https://uk.rs-online.com/web/p/key-accessories/0866579)

| STEP parts | Buy or make | Qty per fixture | Estimated extended USD |
|---|---|---:|---:|
| Ring | RS PRO 866-579 | 1 | 0.36 |
| 01, 25 | 16 mm birch ply or aluminum base and offcut key rest. Drill six Ø6.6 bench holes and four Ø4.5 rest holes | 1 set | 9 |
| 02, 12, 13 | 6061 offcuts for posts, bridge and rail backing. Braze or bolt the bracket pieces. A small felt plug and M3 screw set spindle drag | 1 set | 18 |
| 03 | 6001-2Z or 6001ZZ shielded bearings, 12 × 28 × 8 | 2 | 10 |
| 04–06 | MISUMI SFJ12-262 ground shaft, two 12 mm ID collars, e.g. Ruland MCL-12-F, and two Ø16 × Ø12.1 × 2 inner-race spacers | 1 shaft, 2 each | 22 |
| 07–09 | Aluminum or PETG wheel/knobs, four Ø5 × 140 steel pins with M5 threaded ends, four Ø7 × Ø5.2 × 99 spacers, eight M5 washers and eight M5 nuts incl. four nylocs | 1 set | 9 |
| 10–11 | Machined brass vise with M2 × 8 flat-ended clamping screw and M4 × 6 shaft grub screw | 1 set | 3 |
| 14, 16 | MISUMI SFJ6-200 shafts, unsealed 6 × 12 × 19 linear bushings (THK LM6 or dimensional equivalent) | 2 each | 19 |
| 15, 17, 18 | POM carriage, two Ø3 × 44 dowel pins, two 3 mm collars or retained stop washers and two Ø6 × Ø3.15 × 2 spacers | 1 set | 5 |
| 19–22 | Steel backing shoe and lifting arm, 1.4 mm spring-steel catch blank, 16 mm polymer handle and two M2 × 4 catch screws | 1 set | 6 |
| 23 | Shop leaf SRK-23, 301 full-hard stainless strip, 70 × 6 × 0.30, plus clamping strip and two M2.5 × 6 screws | 1 | 2 |
| 24 | Miniature Bearings Australia ES-0635-0508-04-M extension spring, 6.35 OD, 50.8 free length, 0.005 N/mm, 0.2 N initial tension, with two M3 spring eyes | 1 | 5 |
| 26 | 2 mm brass blank, 47.5 × 18 overall, bored Ø7.0 with 2 mm of brass between hole and bow edge | 1 | 1 |
| Mounting and stops | Six M6 bench bolts, four M4 stand screws, six M5 × 25 post/bridge screws, one M4 × 6 wheel screw, four M3 × 5 rod retainers, two M3 × 12 radial stops with locknuts, two M2 × 12 shoe screws, M3 drag adjuster, washers and shims | 1 set | 8 |

Estimated materials total **$117.36** using stock offcuts, excluding tools, labor, shipping and tax. Only the ring price is a supplier quote ($7.20 minimum pack). Other figures are allowances. Use the nominal dimensions to source equivalents. [Bearing dimensions](https://us.misumi-ec.com/blog/ball-bearings/deep-groove/6001-ball-bearings-12x28x8mm-z-zz-2rs/) · [Ground shaft configurations](https://us.misumi-ec.com/pdf/fa/2019/2019_US_0202_003.pdf) · [Linear bushing](https://store-na.thk.com/Product/lm6) · [Spring catalog, search ES-0635-0508-04-M](https://files.minibearings.au/pdf/catalogues/Springs.pdf?v=9937)

## Assemble and set the stops

1. Bolt the base and key rest to the same bench. Place the bearing axis 240 above the tabletop. Machine both Ø28 H7 seats coaxially from the same datum. Fit the bearings, spacers and shaft collars with the shaft axially free by about 0.1 mm. Fasten the wheel and brass vise using their M4 screws. Its four grip centres lie on a Ø170 bolt circle. Leave approximately 0.2 mm end float on each rotating knob. Set light felt drag and balance the bare wheel with a small trim washer until it stays at each quarter turn with the hand released.
2. Clamp **only the rear strand** between the inner jaw and polished M2 screw. Put the green rear tip on +Y and the exposed red near tip 20° above +Y. Align the spindle centre to the ring centre. The upper turn and both cut ends must move freely. Use narrow red and green ink bands on the last 2 mm of the ends. Avoid tape or paint buildup between turns. The brass key starts flat on its stand at the hole-centre coordinates on sheet 1.
3. Align both guide rods with the radius **45° above +Y** in the front view. The catch meets the ring **25° behind the red tip** along the near turn. The guide upright is centred at Y = 210.7 mm and mounts at X = 245 and 255 mm in the bench datum. Seat the bushings against the shoulders in part 15 and retain their outer shells with a slip-fit retaining compound. Adjust both M3 stops for **140.5 mm radial travel**. At the inward stop the nose extends 2.50 mm past the wire centreline. Shim the shoe for 0.10 mm clearance to the rear surface. Mount the axial guide pins, leaf and stop collars for **0–8 mm axial travel**. The nose is 0.06 thick, 1.20 wide and tapers over 5.00 to a 1.40 heel. Stone the metal edges smooth. Machine the blade and shoe in metal.
4. Hook the radial spring with **90.8 mm eye spacing at park**. This gives about 0.40 N at park and 1.10 N fully in, before gravity and friction. Set the lift leaf to 61 mm free length between clamp edge and follower, straight at its return stop (nominal zero preload). Its calculated rate is about 0.035 N/mm and the 8 mm load about 0.28 N. Check incremental force with a small spring scale. Release each axis from mid-travel to check free return and move the parked ring through a full revolution to check clearance.
5. Approach the **outside cusp between the round wires** behind the red tip. The shoe supports the back turn. The tapered nose starts separating the wires as it advances. Lift the near turn toward the operator, start the key hole over the marked tip, then withdraw the catch radially before winding. With the jaws and guides fixed, check that the **2.00 mm** key edge passes beside the catch. Remove the key and confirm the turns reclose. If a lot has a different end shape, record the discrepancy and fit the stop and shoe on the sample before using it. [Commercial curved-catch reference](https://xuron.com/index.php/main/consumer_products/5/48)

The STEP supplies pilot and clearance holes. Tap Ø1.6 for M2, Ø2.0 for M2.5, Ø2.5 for M3, Ø3.3 for M4 and Ø4.2 for M5 where listed. Ream the 3 mm lift-pin guide holes to 3.05 mm. The simulation uses primitive collision shapes for the spindle support, jaws, guides and handles, passive spring drives and viscous bearing drag. The hardware uses clamped bearings, spring eyes and an adjustable friction pad. The working tip, shoe height, ring position, grip locations and travels are the matching dimensions.

## Check an actual ring

Record the ring lot and key thickness. Mount a millimetre scale in the ring plane for the front camera and another at the tip depth for the edge camera. Photograph a relaxed ring three times, removing and reseating it each time. Mark the same reference axes on the clamp. Read the two perpendicular outer diameters, both tip gaps, cut-end profile, overlap angle and whether the turns touch. Keep the original images and the range of each reading. A thin feeler should enter under its own light contact; driving it between the coils changes the measurement.

Repeat the same views after the operator releases the key and retracts the splitter. Use the visible limits in the task README. Verify that the intact-ring readings and their repeat ranges fit inside each published band, and that the 0.25 mm break threshold is resolvable against the scale. If repeat readings straddle a limit, improve the view and repeat the measurement before grading. Physical sample repeatability and opening forces have yet to be measured.

The supplied STEP and PDF can be used directly. To regenerate them in a separate Python environment with CadQuery 2.6.1 and Matplotlib installed, run `python make_fixture.py` and `python make_drawings.py` here. `python make_fixture.py --parts parts` also exports individual manufactured solids.
