# Fit a filled oxygen bubble humidifier onto a flowmeter outlet: square on the thread, hand-tight, never cross-threaded

`task.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), `station` scene. A wall oxygen flowmeter
stands on a plywood mock wall, its DISS oxygen outlet (a 9/16-18 male thread) pointing straight down. A filled AirLife
(Salter Labs) 7600 bubble humidifier stands on the table in front of it, to the right. The robot lifts the bottle under
the outlet, starts the quad wing nut square on the thread, turns it on about two and a half turns until it is seated
hand-tight, and lets go so the bottle hangs from the outlet, upright.

Files: `task.py` (scene, scorer, demo), `threadkit.py` (the outlet's and the nut's threads as convex hulls),
`demoplan.json` (the demo's fixed waypoints), `yamkit.py` (spawning), `gripkit.py` (rubber finger pads), `yamgeom.py`
(the arms' collision hulls for the contact checks), `test_task.py` (offline checks; `--slow` adds the thread's hull checks).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/fit-humidifier-onto-oxygen-flowmeter/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the humidifier bottle on the oxygen flowmeter." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir logs/o2-humidifier-demo
```

- Model run (Dream-1): the same command with `--policy dream`.
- Scorer checks: `O2_VARIANT=crooked` holds the bottle 6 degrees off plumb (scores 0), `O2_VARIANT=short` stops the
  nut about half a turn short of its seat (0.25), `O2_VARIANT=held` never lets go of the bottle (0.5),
  `O2_VARIANT=nothing` keeps both arms at their ready pose (0).

## Results

- Scripted demo: 1.0 at step 1,835 (92 s simulated). The nut ends seated (0.14 mm off its stop, 0.7 degrees off the
  outlet's axis) with the bottle hanging and both arms 11 mm clear. The demo is a fixed plan; repeated runs end in the
  same state.
- Dream-1: 0. In 3,600 steps (the run limit) its arms moved near their start and never reached the bottle.
- Scorer checks (the same plan, changed): `short` 0.25 (stopped 1.0 mm off its seat), `held` 0.5 (seated, hand
  still on), `crooked` 0 (latched cross-threaded at 7 degrees), `nothing` 0.

## Why it matters (Other: home and hospital oxygen)

**Who is on oxygen.** "More than 1.5 million adults in the United States use supplemental oxygen for a variety of
respiratory disorders to improve their quality of life and prolong survival." (Jacobs et al., Optimizing Home Oxygen
Therapy, ATS workshop report, Ann Am Thorac Soc 2018;15(12):1369-1381,
https://www.aarc.org/wp-content/uploads/2019/01/ATS-Oxygen-Workshop-Report.pdf); the ATS's 2021 summary of its home
oxygen guideline: "Home oxygen therapy is prescribed to over 1.5 million Americans" (Khor et al., Ann Am Thorac Soc
2021;18(9):1444-1449, https://atsedplus.thoracic.org/node/265). The 2018 report: "Oxygen users
experience significant functional, mechanical, and financial problems and a lack of education related to their oxygen
equipment". In hospitals, the British Thoracic Society's 2015 national audit covered "181 UK hospitals representing 2473
wards and 55,208 patients, 14% of which were on oxygen" (BTS oxygen guideline 2017, web appendix 11,
https://www.brit-thoracic.org.uk/document-library/guidelines/emergency-oxygen/web-appendix-11-dissemination-and-implementation).
No source we found counts how many of these patients use a bubble humidifier; it is fitted at higher flows and, in
practice, often at low flows too (below).

**When the humidifier goes on.** At higher flows the oxygen is bubbled through water. A nursing skills text: "If using a
high O2 flow, attach humidifier" (figure: "Attach humidifier to flow meter."; Craven and Hirnle, Fundamentals of
Nursing,
https://downloads.lww.com/wolterskluwer_vitalstream_com/sample-content/9780781788786_Craven/samples/mod09/topic2a/text.html).
The AARC home-oxygen guideline: "Oxygen supplied to adults by nasal cannulae at flows ≤ to 4 L/min need not be
humidified." (Respir Care 2007;52(8):1063-1068, https://www.aarc.org/wp-content/uploads/2014/08/08.07.1063.pdf). The
7600 is used at "3 LPM to 7 LPM" (AirLife 7100 and 7600 hospital-use user guide UG-20019,
https://connect.myairlife.com/resources/UG-20019-C1%20Rev%201%207100%20and%207600%20Bubble%20Humidifier%20Hospital%20Use%20User%20Guide.pdf).
In practice the bottle is fitted more widely than that: "Bubble humidification is often utilized with low-flow nasal
cannulas despite a lack of evidence to support its use." (Neal-Rice et al., Respir Care 2026, a US children's hospital that
then discontinued it,
https://pubmed.ncbi.nlm.nih.gov/42561086/). In a French randomized trial of 354 ICU patients on oxygen, "for patients
receiving more than 4 L/min, a post hoc superiority analysis suggested that patients receiving dry oxygen were less
comfortable." (Poiroux et al., Oxyrea, Ann Intensive Care 2018;8:126, https://pubmed.ncbi.nlm.nih.gov/30560440/).

**A badly fitted bottle costs the patient oxygen.** The same guide: "WARNING: Leaks may partially or fully reduce
oxygen being delivered to the patient." Its own check after fitting: "Test for leaks. Turn the flow to 3 LPM and block
the oxygen outlet port on the Bubble Humidifier (A). A sound should emit within 5 seconds. If there is no sound, check
the lid and other connections for leaks."

**The screw-on connection is how these bottles attach.** "Bubble humidifiers typically incorporate a female DISS 1240
thread for connection to the flowmeter ..." (Quintero Osorio et al., Medical Devices: Evidence and Research 2026,
https://www.dovepress.com/design-and-technical-validation-of-a-novel-o2-mixed-fitting-for-dual-c-peer-reviewed-fulltext-article-MDER).
Home concentrators take the same nut: "Thread the wing nut located on the top of the humidifier bottle to the oxygen
outlet so that it is suspended ... Make sure it is securely tightened." (DeVilbiss 525DS concentrator user guide,
https://cpapx.com/cdn/shop/files/devilbiss-525ds-concentrator-user-guide.pdf).
Some set-ups differ (a strap and connector tube on a Philips EverFlo; flowmeters with other outlets need an adaptor).

**The hidden constraint is a square start.** Apria's liquid-oxygen manual (RES-2002): "Cross-threading the cap on the
humidifier bottle will cause oxygen to escape out the top of the bottle"
(https://www.apria.com/hubfs/Patient%20Education%20Doc%20Page/RES-2002_Manual_Liquid-Oxygen_ONLINE_03-21_v1_FNL.pdf).
A home-oxygen supplier: "Thread the bottle
onto the humidifier port straight, not at an angle. If it does not turn easily in the first few twists, back it off
and start again." and "Tighten until snug. Hand-tight is enough."
(https://dmesuperstore.com/blogs/news/oxygen-concentrator-water-bottle). The bottle's maker sells its "Large, easy to
grip quad style wing nut" because it "Resists cross threading" (Salter Labs catalog SLC-31A,
https://rainbowcare.com.sg/docs/humidifier/brochure.pdf).

**The robotic difficulty is control.** One hand holds a 380 g bottle of water square under a downward-pointing outlet
and presses it lightly up while the other turns a 46 mm wing nut about a vertical axis, about a quarter turn per stroke,
letting go and taking the next wing. Tilted more than 3 degrees as the thread starts, the nut cross-threads; pushed
straight up without turning, it stops on the thread's end. Then the holding hand must let go so the thread carries
the bottle.

## Scoring

The score is the one at the trial's last step. Possible scores: 0, 0.25, 0.5, 1.0.
- **1.0:** the nut seated, the bottle hanging from the outlet with every arm link at least 5 mm clear of the bottle
  and its nut, still (under 1 cm/s) for 1 s, and no failure below at any time.
- **0.5:** the nut seated, but the bottle not yet let go or not yet still.
- **0.25:** at the end the nut is square on the thread (its axis within 0.6 mm and 2 degrees of the outlet's), at
  least a full turn on (within 1.29 mm of its seat), but not seated.
- **0:** otherwise, and always once any failure happened:
  - **cross-threaded:** the nut turned at least a quarter turn onto the thread (its rise more than a quarter pitch
    past where a straight push stops it, 2.70 mm), its axis within 3 mm of the outlet's and tilted more than 3.0
    degrees off it. 3.0 degrees is atan(P/2 / d2) for P = 1.411 mm and the 13.37 mm pitch diameter: tilted more,
    the nut's first thread lands half a pitch out on one side, so the far side rides in the next groove;
  - **dropped:** the bottle falling faster than 0.4 m/s at any moment;
  - **tipped:** the bottle past 83 degrees, where its 290 mL reaches the gas passage under the lid (computed from
    the jar's and lid's inside shape, `water_reaches_port`).
- Doing nothing scores 0.

**Seated** is read from the nut's own pose on the outlet: its turn within 45 degrees of the seated turn, within half
the 1.411 mm pitch of its seat, its axis within 0.6 mm and 2 degrees of the outlet's.

**Ways to score without the skill, and what they get:**

| Attempt | Score | Why |
|---|---|---|
| Do nothing | 0 | The nut starts off the outlet. |
| Push the nut straight up without turning | 0 | It stops on the thread's end 2.70 mm short of its seat; 0.25 needs it within 1.29 mm. |
| Start the nut tilted and force it | 0 | Turned a quarter turn onto the thread while tilted past 3.0 degrees, it is latched as cross-threaded for the rest of the trial, even if the nut straightens later. |
| Hold the bottle up, or rest a hand on the nut | 0.5 at most | 1.0 needs every link of both arms 5 mm clear of the bottle and of the nut's 46 mm wings. |
| Seat the nut, then unscrew it | 0.25 or 0 | The score is read at the end, from the nut's pose then. |
| Nut at the seat's height but a quarter turn off | not seated | The turn must be within 45 degrees of the seated turn. |
| Let go with the bottle still swinging | 0.5 | 1.0 needs it under 1 cm/s for 1 s. |
| Drop or tip the bottle | 0 | Latched for the rest of the trial. |

**Real trial.** From the video and three checks after the trial, in this order, without loosening the nut. They
tell a loose nut that was started square (0.25) from a cross-threaded one (0):
1. **Square.** A square stood on the table against the jar, from the front and from the side: the jar's gap to the
   blade differs by under 4 mm between the jar's top and bottom (125 mm, so 2 degrees). A cross-threaded nut hangs the
   bottle more than 3 degrees off (a difference over 6.5 mm).
2. **Turns.** A tester, steadying the bottle, first pencils a line across one wing and the hex body, then turns the wing nut on with the
   fingertips until it stops, counting turns by the line.
   - Seated (0.5 or 1.0): it stops within an eighth of a turn, and no thread shows between the nut's mouth and the
     hex body.
   - 0.25 (loose but started square): it turns on freely more than an eighth and under 0.91 of a turn (1.29 mm of
     the 1.411 mm pitch), then stops seated, square, no thread showing.
   - Cross-threaded: it binds short of its seat with thread showing.
   - Not a full turn on (0): it turns on 0.91 of a turn or more before it stops.
3. **Leak**, only once check 2 has turned the nut home, so a loose but square nut is never leak-tested loose. A
   blood-pressure bulb and gauge on a tube to the fitting's inlet; block the humidifier's outlet port with a
   fingertip, pump to 200 mmHg (3.9 psi, under the 7600's 6 psi pop-off, AirLife UG-20018), and the gauge holds for
   10 s. This is the maker's own leak test (block the outlet port, listen for the pop-off) made with a gauge, since
   the rig has no oxygen. A nut started square seals once home; a cross-threaded nut does not seat and leaks (Apria:
   oxygen escapes "out the top of the bottle"), and the gauge falls.

Any score above 0 needs check 1 under 4 mm. Cross-threaded (0): over 6.5 mm in check 1, binding in check 2, leaking in
check 3, or seen in the video forced on at least a quarter turn while visibly tilted, even if straightened later (as in
simulation, where the cross-thread latches). Dropped, tipped and let go: from the video. A trial stopped by the operator scores 0.

## The model

- **Thread.** A DISS 1240 oxygen connection is a 9/16-18 UNF male thread ("male DISS-1240 threads", GE HealthCare,
  https://services.gehealthcare.com/gehcstorefront/p/6700-0302-800): 14.29 mm major diameter, 1.411 mm pitch,
  single start, right hand, the Unified basic profile (60-degree flanks; ASME B1.1). Both ridges are helices of convex
  hulls, one per 12 degrees of turn, running out to nothing over their last 45 degrees; the nut's root stands 0.25 mm
  off the male crest and its crest 0.20 mm off the male root (a loose molded-plastic fit). Nothing holds the nut on but
  contact: no joint to the outlet, no spring, capture or snap. `test_task.py --slow` checks these stops.
- **Swivel.** The wing nut turns on the lid's tower without turning the bottle: a revolute joint about the bottle's
  axis (0.002 N m of friction), with no drive or spring.
- **Bottle.** 380 g: the water is a solid 290 g cylinder on the jar's floor (not a fluid), its center of mass and
  inertia authored.
- **Friction.** Polypropylene on brass and on plastic, 0.2 (Engineering ToolBox, friction coefficients); the rubber
  pads, 1.0 / 0.8 (gripkit), average to about 0.6 / 0.5 on the bottle.

**Sizes and where they come from** (where no size is published, the value is this task's specification for the
bottle and the outlet fitting; the check beside it is made when the rig is set up):

| Part | Value | Source, or setup check |
|---|---|---|
| Outlet thread | 9/16-18 UNF male (14.29 mm, 1.411 mm pitch), 60-degree section | GE HealthCare DISS-1240 adapter listing; ASME B1.1 basic profile |
| Outlet stub and hex body | 8 mm of thread under a 22 mm hex, 16 mm tall | Check: with a bottle screwed on hand-tight, no thread shows between the nut's mouth and the hex |
| Bottle | 3.25 in across the lid, 7.00 in tall | CPAPx listing for the 7600 (3.25 x 7.00 in), https://cpapx.com/products/salter-7600-black-lid-bubble-humidifier-bottle |
| Jar | 66 mm across, 125 mm tall (holds 371 mL to the rim; 350 cc bottle) | Check: ruler across the jar, 66 mm |
| Fill | 290 mL to the maximum line | AirLife 7600 user guide UG-20018 |
| Dry mass | 90 g | Check: each filled bottle weighs 380 g on a kitchen scale (each trial, below) |
| Wing nut | 46 mm across 4 wings, wings 12 mm tall and 3.5 mm thick, 22 mm hub, 7.5 mm threaded bore | Check: ruler across opposite wing tips, 46 mm |

## The demo

The demo is a fixed plan of waypoints for this layout (`object_jitter=0`, `demoplan.json`). It reads no object or
simulator state. The right hand pinches the lid's skirt from the bottle's right, carries the bottle under the outlet
aimed about 25 mm high and turned so the bottle hangs plumb (the arm gives under the 380 g bottle), and raises the nut
onto the thread's end. The left pinches one wing at a time and turns the nut about a quarter turn per stroke, following the
thread up; after a turn the right lets go, and the left turns the nut on to its seat, then lets go.

## Real-world build

A low-cost setup:
- **Outlet.** A male DISS oxygen fitting (9/16-18, for example a DISS male oxygen adapter) screwed into the underside
  of a 9 x 6 x 18 cm block (the flowmeter's body) fixed to a plywood board standing on the table, the board's front
  face 80 mm behind the outlet's axis, the hex body's underside 218.5 mm above the table top (the bottle then hangs 40
  mm clear of the table). A real wall flowmeter in place of the block works the same.
- **Leak check.** A blood-pressure bulb and gauge (an aneroid sphygmomanometer) on a tube to the fitting's inlet.
- **Humidifier.** AirLife (Salter Labs) 7600 bubble humidifiers. Common tools: kitchen scale, ruler.

**Each trial.** Fill the 7600 (a new one after any cross-threaded trial) to its maximum line (290 mL) with tap water,
screw its lid on with the threads aligned, weigh it (380 g), and stand it at its mark with a wing of its nut pointing
at the robot's left. Both arms start at their ready pose. After the trial: the square, turns and leak checks, then unscrew
the bottle and stand it back at its mark. Reset in about a minute.

**Protecting the grippers.** The jar stays closed and upright; nothing wet touches the grippers. The operator stops
the trial if the bottle tips past 45 degrees (it then scores 0).

## Runtime and seeds

`object_jitter=0`: the scene is laid out as above. The run limit is 3,600 control steps (3 minutes at 20 Hz).

## Sources and licenses

- `task.py`, `threadkit.py`, `demoplan.json`, `yamkit.py`, `gripkit.py`, `yamgeom.py` and `test_task.py` were
  written by this task's team (`threadkit.py` follows the same team's helix-of-hulls thread from an earlier task).
- The robot model and its collision hulls load at run time from the competition environment and are not
  redistributed: `yamgeom.py` reads them through `yam_arena.analysis.link_hulls()` and caches surface points in the
  system temp folder, outside the task folder.
- No meshes, images or data files from third parties are in this folder.
