# Raise a bed's side rail by its release knob after moving the resident's forearm out of its path

Instruction: `Raise the near side rail on the resident's bed, with nothing of hers caught in it.`

A care-training bed with an adult care manikin lying on her back. The near half rail is down. It rises only while its
release knob is held out, as on many home-care and long-term-care beds: one hand holds the knob and the other lifts
the rail until it latches at the top. The resident's left forearm and hand stick out over the near edge, straight
across the lowered rail. Lifting the rail now would trap her forearm between the rail and the bed frame, a known
side-rail entrapment injury. So her forearm has to be swung back in, about the elbow, until it lies on the mattress
along her side, and only then is the rail raised.

Nursing assistants (O*NET 31-1131.00, 1,351,760 US jobs, BLS OEWS May 2023) do this many times a shift, after every
turn, bath and linen change.

What makes it hard physically:
- **Handling a limp limb.** The forearm is pinched near the wrist end, lifted just clear of the mattress edge and
  carried round its elbow (about 90 degrees) without lifting it high or bending the elbow past its stops.
- **Two hands, two places.** The knob is beyond the rail's foot end and the rail is lifted near its head end, about
  55 cm apart: one arm holds the knob out (about 2-4 N) while the other carries the rail along its arc (out and up 24
  cm).
- **Order and release.** Knob back in at the top so the plunger catches, then let go of the rail.

## Scene

Metres; origin on the table top below the midpoint of the arm bases; x away from the robot, y to its left.

| object | type | size | material (real part) | position |
|---|---|---|---|---|
| bed platform | fixed | 70 x 110 x 18 cm | steel-framed platform, grey | x 0.30-1.00, y -0.55-0.55, on the table |
| mattress | fixed | 70 x 110 x 8 cm | blue vinyl-covered foam | on the platform, top 0.26 m above the table, near edge x 0.30 |
| half rail panel | jointed, 2.6 kg | 70 x 16 x 2 cm | steel-framed panel, light-grey infill | spans y -0.45 to 0.25; low: inner face at x 0.285 |
| rail top tube | welded to panel, 1.1 kg | 32 mm dia, 70 cm | steel tube | on two end posts, 4 cm clear above the panel (the hand-hold); low: 3 cm below the mattress top |
| pivot arms (2 pairs) | jointed, 0.6 kg per pair | 13.2 cm centres | steel flat bar | pairs at y -0.36 and 0.16; each pair is a parallelogram, so the rail swings out and up 24 cm without tilting and ends outside the mattress edge again (sim: one arm per pair, plus a joint that keeps the panel upright) |
| release plunger | jointed knob, 30 g | knob 28 mm dia x 45 mm | spring plunger with red pull knob | on the frame face at y 0.37, 15.5 cm up (2.5 cm below the deck top); knob face 6.7 cm out from the frame |
| care manikin | fixed torso and head | adult upper body | vinyl care manikin in a gown | supine, head at y -0.48, torso x 0.51-0.84 |
| manikin left arm | jointed, 0.6 + 0.45 + 0.2 kg | upper arm 28 cm x 6 cm dia, forearm 26 cm x 5.4 cm dia (rounded), flat hand 17 cm | manikin's own arm | upper arm along her side at x 0.45 from y -0.24 to the elbow at y 0.04; elbow bent 90 deg; forearm and hand straight out over the near edge towards the robot |

Mechanism: the foot-end lower arm's pivot boss has two holes, one lined up with the plunger at the low stop (0) and
one at the top stop (130 deg of swing). The plunger's spring pin (worked by the knob through a short rod) sits in the
hole unless the knob is held out 12 mm or more. So the knob must be held out while the rail is lifted off its low stop
(let go there and the pin drops straight back in); once the rail is moving the pin rides on the boss face, and at the
top it snaps into the second hole and latches the rail up. The rail is real weight (4.9 kg moving, steel), and a
strong
gas spring on the head-end pivot arm carries 91% of its weight torque at every angle, as on assisted hospital
siderails: lifting takes about 1.6 N at the stops and up to about 4 N at mid-travel, measured at the tube. Raised, the
tube top is 22.6 cm over the mattress, above the 22 cm minimum of
IEC 60601-2-52 (the medical-bed safety standard). The manikin's elbow has friction: the
forearm stays where it is put.

Named product the rig copies: the Gendron 3648 home-care bed's half rail operation. Its manual: "grab top section of
rail with one hand while pulling the Release Knob with the other hand", then "allow the Release Knob to spring back
into locked position". The swing-up parallelogram motion and the gas-spring lift assist follow split siderails such
as the Linet Eleganza 3's (its manual: a gas spring supports the operation of the split siderails). Its weight matches
steel half rails (retail listings give about 22 lb per pair, about 5 kg each, for the Drive 15201BV). It latches only
at its low and top stops. Side-rail limb entrapment is a
recognised hazard (FDA hospital bed entrapment guidance, seven entrapment zones).

## Rail geometry (side view, looking along +y; heights above the table top, x from the arm bases' line)

| point | low stop | top stop |
|---|---|---|
| arm pivots on a bracket just inside the frame face (each pair: 16.3 and 17.5 cm) | x 0.321, 17.5 cm | same |
| arm ends on the panel's bottom bracket | x 0.265, 5.5 cm (arm 65 deg below level) | x 0.265, 29.5 cm (65 deg above) |
| panel 70 x 16 x 2 cm, centre | x 0.275, 9.4 cm (bottom 1.4 cm) | x 0.275, 33.4 cm |
| top tube centre (32 mm, on 4 cm posts) | x 0.275, 23.0 cm (3 cm under the mattress top) | x 0.275, 47.0 cm (21 cm over it; tube top 22.6 cm over) |
| plunger holes in the foot-end lower arm | engage at the low stop | engage at the top stop |

Arm length 13.2 cm, swing 130 deg, rise 24 cm. Mattress top 26 cm, deck top 18 cm, mattress edge x 0.30. Release knob
centre x 0.256, y 0.37, 15.5 cm up.

## Real rig (low cost)

Dimensioned build drawing of the rail, its arm pairs, the latch holes and the plunger rod: `rail_drawing.svg`.


- Gripper pads: deburr all cut steel, cap the top tube's ends with rounded plastic plugs and edge-cap the panel frame
  (rubber U-channel), so no sharp edge meets the gripper pads or the manikin.
- Platform: 70 x 110 cm plywood on an 18 cm steel or timber frame, clamped to the table; 8 cm vinyl-covered foam
  mattress. Table tape marks at its corners.
- Rail: a 70 x 16 x 2 cm panel (20 x 20 x 2.5 mm steel square-tube frame with a mid crossbar and a 3 mm HDPE infill,
  about 2.6 kg) with a 32 mm
  x 2 mm steel tube (1.1 kg) on two 4 cm end posts above it; two pairs of steel flat bars (25 x 5 mm, 13.2 cm hole
  centres; the two bars of a pair side by side 30 mm apart in y and hinged 12 mm apart vertically at both ends, see
  rail_drawing.svg) on M8 shoulder bolts make the parallelogram links to two brackets on the frame. Assist: an
  off-the-shelf gas spring (about 130 N, 100 mm stroke) from a frame bracket to a pin on the head-end lower arm 40 mm
  from its pivot, the bracket placed so the spring's lever shortens towards the stops: about 5.1 N m at the pivot with
  the arms level and about 2.1 N m at the stops, 91% of the rail's weight torque at each angle; mounting points are
  in rail_drawing.svg. Plunger: an off-the-shelf spring indexing plunger (M12, 15 mm pin stroke) inside the frame at
  the foot-end pivot (y 0.16); its pin enters 10 mm into two holes drilled in that lower arm's pivot boss, 130 deg
  apart on a 40 mm radius (low stop to top stop). A 6 mm rod inside the frame runs 21 cm from the pin to the red knob
  at y 0.37, so the knob must come out 12 mm (10 mm plus 2 mm clearance) to free the arm; a stop limits the pull to 25
  mm. Return spring about 150 N/m.
- Manikin: any adult nursing-care manikin with a jointed shoulder and elbow (e.g. a full-body or upper-body
  patient-care trainer); the elbow friction should hold the forearm where it is put (about 0.4 N m).
  Low-cost stand-in (about 50 USD) if none is at hand: trunk = a 33 x 58 x 17 cm foam cushion in a hospital gown with
  a 19 cm foam ball as the head; arm = 40 mm PVC pipe in foam pipe insulation (upper arm 28 cm, 6 cm outside; forearm
  26 cm, 5.4 cm outside), shoulder on a plain door hinge screwed to a 10 mm plywood plate under the cushion; elbow on
  an
  adjustable friction (torque) hinge set so a 0.4 N m twist just moves it; hand = a padded work glove over a 17 x 8.5
  cm
  plywood blade. Weigh in sand to about 0.6 / 0.45 / 0.2 kg per segment.
- Parts: under 150 USD for the bed rig and rail, about 200 USD with the stand-in manikin.

## Run a trial

Requires yam-isaac-lab-arena and Isaac Lab-Arena 0.3.1; `task.py` needs nothing else in this folder.

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Raise the near side rail on the resident's bed, with nothing of hers caught in it."
```

Run limit 2,400 steps (120 s at 20 Hz). The layout is fixed (`object_jitter=0`); nothing is randomised.

## Scoring (score() in task.py)

Score 0 if, at any step, the rail is off its low stop (pivot arms more than 3 deg from it) while any part of her
forearm or hand is over the mattress edge (x < 0.30) below the top of the rail's swept space; if the forearm or hand
is
ever more than 30 cm above the mattress; if the elbow goes past its stops; if her arm is handled roughly (the
elbow swings faster than 90 deg/s or the hand moves faster than 0.5 m/s, averaged over 0.15 s; a video frame count
shows it); if the rail is forced more than 1.5 cm
off its arms' arc (a bent rail); or if, at the end, her forearm and hand are
not resting on the mattress wholly inside its edge (bounding boxes at x >= 0.30, their undersides between 1 cm below
and 2.5 cm above the mattress top, so an arm held up in the air does not count). The bed is fixed in sim; in a real
trial a bed moved over 2 cm from its tape
marks also scores 0.

Otherwise: 0.25 forearm and hand resting inside; 0.5 rail lifted off its low stop (pivot arms 20 deg or more, about
3 cm up); 1.0 rail latched at its top stop (within 3 deg, knob back in so the plunger holds it) with every gripper
point at least 10 cm from the rail, the knob and her arm at the end.

In a real trial: watch the video for the arm over the edge while the rail moves; at the end, a photo shows the arm on
the mattress and the rail up; push down on the rail top to check it is latched.

## Notes

- One arm cannot do it: the knob and the rail's reachable part are about 55 cm apart and the knob must stay out while
  the
  rail leaves its low stop; let go there, the plunger catches again at once.
- Letting the knob go once the rail is moving: the pin rides on the boss face; the rail still has to be carried to the
  top, where it latches (let go of the rail early and it drops back to its low stop and relatches).
- Forcing the rail without the knob: the plunger holds it (about 1 deg of give under a 20 N push).
