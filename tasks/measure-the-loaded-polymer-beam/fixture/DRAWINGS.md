# Apparatus and mount schedule, version 2

All dimensions in mm. World X points away from the arm bases, Y to the left, Z up from the bench surface. The commercial components are at full scale. This is a proposed, reproducible assembly, not a verified OEM fit.

| Component | Size and location | Source and status |
|---|---|---|
| ES05 base | 162.1 X by 99.1 Y by 20 Z; load axis X320, Y0; base centre X359.55 | Maker footprint; 20 mm height estimated from the drawing |
| ES05 column | Ø18, axis X412, Y0; from Z20 to the 355.6 maximum top | 92 mm load-axis offset and 355.6 mm from the maker drawing; diameter estimated |
| ES05 head | 60 x 60 x 75 block on the column, centre X395, Y0, Z242.5 | Estimated; the maker states the lever mechanism can be repositioned along the column |
| ES05 lever | Hub X380, Y-70, Z242.5; 105 mm hub to knob centre; knob Ø26 x 28, centre at rest X299.6, Y-70, Z310.0; rest 40° above horizontal toward the operator | Scaled from the maker drawing (data sheet page 2, using the 162.1 mm base as the scale) |
| AC1060 plate | 70.1 x 82.8 x 18.5; #10-32 holes on a 12.7 mm grid, central 5/16-18 | Maker drawing; long side along Y |
| G1095 lower fixture | 211.6 long (Y), 57.2 wide (X), 87.6 maximum height; Ø10 x 51.4 rollers 150 apart | Maker drawing |
| G1097 upper anvil | 57.2 wide (X), 33 thick (Y), 49.5 high; Ø10 roller | Maker drawing |
| M5-20 force gauge | 100 N x 0.02 N; #10-32 male output stud; 145 x 71.1 x 35 body envelope | Maker capacity and resolution; envelope nominal |
| Low-force indicator | 0.01 mm digital indicator with 0.2 to 0.5 N measuring force (for example Mitutoyo 543-715, retailer listing); spindle tip under the bar's centre, body below the fixture top on drop bracket C1 | Retailer listing; mount drawn here as an estimate |
| TIVAR 1000 bar | 200 long (Y) x 25 wide (X) x 10 thick; 46.5 g at 0.93 g/cm³ | Maker sheet: density and 87 ksi (about 600 MPa) flexural modulus |
| Two specimen cradles | Machined aluminium, 70 x 220 x 10 mm base, two 40 x 25 x 20 mm saddles at ±75 mm; bar centres X200 (far, layout A) and X110 (near, layout B), Y130, Z35 | Drawn dimensions |
| Bench display (simulation only) | 190 x 90 x 6, lying flat, centre X395, Y-155; force (N) top row, deflection (mm) bottom row | Repeats the two readings because the simulated instrument faces are blank; in a real trial the robot reads the instruments' own screens |

Eye-end adapter A1: 2024 aluminium plate 70.1 X by 82.8 Y by 12 thick, four Ø5.2 holes on a 50.8 x 50.8 square, central Ø8.4 hole for the 5/16-18 fastener, Ø14 x 6 counterbore underneath. The lower fixture's eye end is held in a split 303 stainless clevis, 30 x 25 x 25, by an 8 mm pin through Ø8.2 ears. The plate and clevis sit inside the bottom 12 mm of the 87.6 mm fixture envelope; they do not add height to the stack. Check the ear gap against the real fixture eye before machining. It must not rotate under 50 N; witness marks and a ruler check that the apparatus moves less than 5 mm.

Eye-end adapter A2: #10-32 female coupler, Ø12 x 18, threaded 12 deep, joining the M5-20 stud to a matching 303 stainless clevis (30 x 25 x 20) that holds the G1097 eye on an 8 mm pin. Check the eye bore before machining; adjust only the bore and the spacer length. Neither adapter is a robot action.

Indicator mount C1: an aluminium drop bracket clamped to the front face of the G1095 bar holds the indicator on the load axis, spindle up, with its body below the fixture top; the 48 mm between the fixture top and the bar's underside is shorter than the indicator, so only the spindle rises between the rollers. With the bar unloaded on the rollers, raise the indicator until the spindle has moved about 1 mm into its travel and zero the display; at 1 mm and 2 mm of bend the spindle is about 2 mm and 3 mm in, where the measuring force is about 0.2 to 0.3 N, below the bar's 0.46 N weight, so the bar stays on both rollers. Because the bracket moves with the lower fixture, the reading is the bar's centre relative to the supports. The simulation draws the indicator schematically and does not model its measuring force. Check the bracket against the real fixture before a trial.

Lever: the head is set on the column so the nose rests 6 mm above the bar with the lever on its rest stop. The nose meets the bar about 19° below rest; the 2 mm reading is near 25°, with the knob centre near X278, Z268. The return spring (0.3 N m/rad, 0.3 N m preload at the rest stop), damping (0.05 N m s/rad) and the 18 mm/rad pinion (38 mm over about 121° of lever) are estimates. Check hub position, lever length and rest angle on the real stand before a trial.

Checks before a real trial: span 150.0 ± 0.5 mm; both rollers at the same height; the 200 mm bar clears the column 92 mm behind the load axis; unloaded bar centre 131.1 mm above the bench; gauge and anvil stack within the 179 mm clearance; a 45 N test through the lever with hands clear of the roller pinch; both displays and both roller witness marks visible on video. No purchase or measured assembly is claimed.
