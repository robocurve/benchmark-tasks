# Plasma-cut a steel plate along a hand-held straightedge until the offcut drops

A 3 mm mild-steel plate is clamped on a cutting stand with a 55 mm strip overhanging; a white scribed line marks the cut, 200 mm from the plate's far edge to its near edge. A steel straightedge lies on a pad at the left and a hand plasma torch with a drag shield on its rest at the right. The robot has to set the straightedge on the plate one shield radius (10 mm) from the line and hold it down with one hand, drag the torch's shield along it with the other from edge to edge at a cutting speed, so the strip drops into the tray, then put both tools back and move both hands away. Instruction: "Cut the overhanging strip off the plate along the white line with the plasma torch, guiding the torch along the straightedge, then put the torch and the straightedge back where they were and move both hands away."

**How this differs from the TIG seam task (`tig-seam`).** That task lays a bead along a joint: a torch held at arc length in free air while the other hand dabs filler, scored by fused and filled segments. This one is a separation cut with contact-guided motion. The torch is dragged in contact, its shield riding on the plate and bearing sideways on a loose straightedge that the other hand must place to the millimetre and keep from creeping. The pace is 4 times faster (12.7 mm/s against 3 mm/s) with a window on both sides. Nothing is added to the work: the outcome is a part cut off, a straight edge on the line, and an offcut that physically drops only when the whole line is cut through.

Install once, from this folder: `pip install -e . --no-deps`

Run a trial:

    inspect-robots run --embodiment yam_plasma -E task=task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 --policy yam_scripted --no-prompt \
      --instruction "Cut the overhanging strip off the plate along the white line with the plasma torch, guiding the torch along the straightedge, then put the torch and the straightedge back where they were and move both hands away." \
      --scorer task_score --max-steps 2400 --seed 0

The score is printed as `task_score`. Use `--policy dream` for Dream-1.

- **Runtime per trial:** about 3 minutes on one L40S GPU (890 steps: the 829-step demo plus the policy's 3 s hold); 10-20 minutes on a shared GPU.
- **Seeds:** none. The start is fixed and `object_jitter=0`; `--seed 0` was used for the demo.
- **Scoring tests, no simulator** (in the simulator's Python environment): `python -m pytest -q test_score.py`. `python task.py` tests the demo's reach and length (829 steps). `python dev/replay.py` plays the demo's plan with ideal tracking through the scorer.
- **The scripted demo** is open loop. It uses only the fixed starting layout, which a policy can see, and reads no simulator state. The left arm takes the straightedge by its handle, carries it over the clamps, sets it on the plate and presses it down. The right arm takes the torch and waits beside the plate until the straightedge is held down, then brings the shield against the straightedge's overhanging end just beyond the plate's far edge and draws it toward the near edge at 12.7 mm/s, leaning on the straightedge and resting lightly on the plate. The strip drops as the last segment is cut. Both tools then go back and both arms move away. The plan aims against the arms' measured give under load (the torch hand sags 10 mm with the torch in it; the straightedge hand ends 9.7 mm to the side and 0.39 degrees round as it presses down), measured from logged runs.
- **Shortcut variants** (`PLASMA_DEMO=...`, debugging only), with the scores measured in the simulator (the full demo scored 1.0 in two runs in a row):

  | Variant | What it does | Score | What happened |
  |---|---|---|---|
  | `nothing` | no motion | 0.0 | nothing cut |
  | `push` | no torch; the gripper pushes down on the offcut | 0.0 | the 40 bridges hold; the offcut did not move |
  | `freehand` | the straightedge stays on its pad; the torch runs along the line alone | 0.0 | 28 segments cut through about 2 mm off the line, none against the straightedge; the offcut stays |
  | `unheld` | the straightedge is set on the plate and let go | 0.26 | the torch pushes it up to 3.7 mm off; all 40 cut and the offcut drops, but only 13 segments are within 2 mm |
  | `fast` | travel at 30 mm/s | 0.02 | one segment (where the shield comes onto the plate) cut through; the offcut stays |
  | `slow` | travel at 4 mm/s | 0.02 | all 40 cut and the offcut drops, 39 overheated |
  | `half` | the cut stops halfway | 0.38 | 19 sound segments; the offcut did not move |
  | `snap` | the cut stops two segments short, then the shield pushes the offcut's far end down | 0.76 | the last two bridges break and the offcut drops; 38 sound segments, no credit for the drop |
- **What the physics does:** the torch, straightedge, plate, offcut and hands are rigid bodies with real contact. The shield is a cylinder collider that slides on the plate and along the straightedge's edge (copper on steel, friction 0.35; steel on steel 0.5). The straightedge is loose: only the hand's press and friction keep it in place. The arc and the cut are bookkeeping from the torch's measured pose (see `task.json`'s fidelity and the top of `task.py`). The kerf, dross, the arc's glow and an uncut gouge are drawn from the score's record. `PLASMA_DEBUG=1` prints the cut's state every step; `dev/trace.py` summarises such a log.
- **Third-party assets:** none. Every part is built from boxes, cylinders and spheres in the task's own code.

## Cut settings and where the numbers come from

The simulated cutter is a 30 A-class hand air plasma cutter (15-30 A output) set to 20 A on 3 mm mild steel. A manufacturer's operator manual for a machine of this class gives, at 20 A on a 120 V supply: a rated cut of 3 mm at 762 mm/min (12.7 mm/s) and a maximum travel speed on 3 mm of 1,245 mm/min (20.75 mm/s), the speed above which the arc no longer cuts through. The same manual says to drag the torch's shield lightly on the workpiece for hand cuts, to use a straightedge as a guide for straight cuts, that sparks spraying up mean the cut is not going through and the torch must move more slowly, and that moving too slowly leaves heavy low-speed dross. From these:

| Criterion | Value | Source or reasoning |
|---|---|---|
| Rated travel, the demo's speed | 12.7 mm/s | the manual's 3 mm capacity at 20 A |
| Cut through | 20.75 mm/s or slower over a 5 mm segment (0.241 s or more of arc) | the manual's maximum speed for 3 mm at 20 A |
| Overheated (dross, melted top corner) | slower than 6.35 mm/s (more than 0.787 s of arc on a segment) | a modelling choice, not a published figure: half the rated speed. The manual says only that moving too slowly leaves heavy low-speed dross and gives no number, so the threshold is confirmed on scrap before a real trial (below) and the real edge is judged by the dross and corner measurements, not by speed |
| Standoff | nozzle 1-4 mm above the plate: the shield's rim from touching (1 mm of numerical penetration allowed) to 2 mm above it | drag cutting with the shield on the plate; a mechanised-torch chart for a 45 A machine with the same kind of consumables uses 1.5 mm |
| Straightedge offset | 10 mm from the line | the drag shield's radius |
| Kerf | about 1.4 mm (real trial only) | a mechanised-torch cut chart for a 45 A machine with the same kind of consumables, at 3 mm; not simulated |

## Why the task matters (category Other)

The case is the work that hand torch cutting is used for and the harm it does to the people doing it, not a headcount
of welders. Each figure below is quoted from its source; where a figure covers more than torch cutting, it says so.

**A. Cutting up contaminated steel in nuclear cleanup.** This is where a robot that can guide a hand torch is wanted now.
- Scale. The Department of Energy carries $538.6 billion of the federal government's $667.3 billion environmental
  liability (US Treasury, Notes to the FY 2025 Financial Statements, Note 14). GAO: "As of May 2025, EM estimated that
  the remaining cleanup work at all its sites would cost more than half a trillion dollars", and it lists the liability
  as a High-Risk area (GAO-26-107820, March 2026). EM's enacted budget was $8.767 billion in FY2024 and FY2025, and its
  mission includes "deactivation and decommissioning of thousands of excess facilities" (DOE FY 2026 Congressional
  Justification, Vol. 6). 19 commercial power reactors at 15 sites are being decommissioned (EIA).
- Cutting is central, and plasma is a main method. Reactor internals "must be carefully surveyed and, in most cases, cut
  into smaller pieces"; "Plasma arc cutting was the primary method of segmenting the RPV internals at Shoreham and the
  Yankee Nuclear Power Station" (EPRI 1003029, 2001). Segmentation doses at three plants were 57, 73 and 177
  person-rem (same report; those plants used other cutting methods). "Laser and plasma cutting systems are routinely
  used for size reduction tasks within the nuclear decommissioning sector" (Anderson et al., Nucl. Eng. Des. 457,
  115085, 2026, University of Strathclyde with the UK National Nuclear Laboratory).
- Much of it is still done by hand, in suits. Rocky Flats Building 771 held 240 stainless-steel gloveboxes: "Workers
  wearing cumbersome protective clothing cut up glovebox sections", and "Using plasma-arc torches to cut up stainless
  steel gloveboxes significantly improved safety" (DOE fact sheet). At Savannah River, "Some tasks require that the
  equipment be operated by a person wearing a plastic suit or full face respirator", the tools including "a plasma
  cutting torch" (Fogle and Younkins, ANS Robotics and Remote Systems meeting, 1997). Outside the US, Sellafield
  estimates 700 gloveboxes to size-reduce; the first, in 2024, "required operators to perform cutting manually while
  wearing air-fed suits and respirators" (Anderson et al., 2026).
- Robots lower the dose, and autonomy is the stated goal. "A study on robotic glovebox dismantling facilities found that
  radiation exposure doses to workers were less than 1/5th of those in manual cutting facilities"; "Remote autonomous
  cutting technologies can enhance operational effectiveness ... by moving operators away from cutting interface"; the
  authors cite "the nuclear sector's drive to automate cutting processes" (Anderson et al., 2026). Their test cut is a
  robot plasma-cutting 3 mm plate "representative of nuclear glovebox material" at 15 mm/s: the same thickness and
  close to this task's 12.7 mm/s. Gloveboxes, tanks and piping are cut where they stand, with portable tools, not on a
  cutting table, which is why a hand torch and a hand-placed guide are the right test.

**B. Scrap, ship recycling and demolition.**
- US scrap purchases were $19.7 billion in 2025, about 57 million tonnes consumed (USGS Mineral Commodity Summaries
  2026). "One of the most common tools used to break apart large metal pieces is the gas cutting torch ... other torches
  use plasma" (OSHA 3348, metal scrap recycling guidance).
- Robotics researchers funded by a scrap recycler: "Currently, the cutting operation is conducted manually by skilled
  workers using a gas torch ... This manual operation is slow and laborious"; the robot would take on "the slow,
  dangerous and tedious cutting operation"; "We are not aware of any work in the literature on robotic methods for gas
  cutting that is general enough to be applied to metal scrap recycling" (Akl, Alladkani and Calli, IEEE CASE 2021,
  funded in part by European Metal Recycling). The EU's SHEREC project (EUR 7.95 million, 2024-2027) aims to "automate
  the cutting ... in ship recycling", citing "unacceptably high levels of fatalities and injuries".
- The harm: NIOSH found 12 of 13 air samples from torch "Burners" at a scrap recycler at or over the OSHA lead action
  level or limit (Rodriguez, NIOSH, 2005); OSHA's interpretation for bridge demolition "which requires torch cutting of
  lead-coated surfaces" presumes exposure above 2,500 ug/m3, 50 times the limit (1996); scrap and recycling wholesalers
  had a 2016 fatality rate "nearly seven times" that of private industry (NIOSH, 2020; all causes).

**C. Fires.** NFPA counts 4,580 US structure fires a year involving hot work, with 22 civilian deaths, 171 injuries and
$484 million of direct damage (2014-2018). Of the 2,600 a year in non-home properties, 25% involved cutting torches
(Swiss Re risk fact sheet citing Ahrens, NFPA, June 2021): about 650 fires a year (25% of 2,600, our arithmetic). The Chemical Safety Board has
counted "over 60 fatalities since 1990" from hot work on tanks; in two of the fatal cases it summarises, cutting
torches were the ignition (Daytona Beach 2006, 2 killed; Garner, Arkansas 2009, 3 killed; CSB bulletin 2009-01-SB).

**D. The people who do it regularly.** O*NET task ratings (share rating the task relevant x share doing it more than
weekly) x BLS OEWS May 2025 employment, counting only tasks whose only cutting tool is a torch:

| Occupation (SOC) | O*NET task (ID, release) | OEWS 2025 | Relevant | More than weekly | Regular workers |
|---|---|---|---|---|---|
| Welders, cutters, solderers and brazers (51-4121) | "Dismantle metal assemblies or cut scrap metal, using thermal-cutting equipment, such as flame-cutting torches or plasma-arc equipment." (10249; 51-4121.06, rated 07/2012, O*NET 23.0-25.0) | 416,210 | 84.9% | 59.1% | about 209,000 |
| Valve and regulator repairers (49-9012) | "Disassemble and repair mechanical control devices or valves ... using power tools, hand tools, and cutting torches." (11779, 30.2) | 48,240 | 72.5% | 63.5% | about 22,000 |
| Rail car repairers (49-3043) | "Remove locomotives, car mechanical units, or other components, using pneumatic hoists and jacks, pinch bars, hand tools, and cutting torches." (9987, 30.2) | 21,350 | 96.8% | 76.5% | about 16,000 |

About 247,000 in all. This is a floor, not the case for the task: torch cutting is also part of tasks that list other
tools (pipefitters, ironworkers, rebar workers, millwrights, maintenance workers), which are not counted because the
data do not split them by tool.

**Why the cut is guided.** For straight hand cuts the makers direct a guide: "For straight-line cuts, use a straight
edge as a guide" (Hypertherm Powermax45 SYNC operator manual); "brace the edge of the drag shield on a guide (a ruler,
piece of wood, cardboard pattern, etc.)" (Miller, How to select and operate a hand-held plasma cutter); and collision
repair procedures for sectioning say to "scribe the final cut line on the part and set up a guide" (I-CAR).

**Sources (URLs).** Treasury Note 14: https://fiscal.treasury.gov/system/files/2026-04/2025-note-14-financial-statements.pdf;
GAO-26-107820: https://www.gao.gov/products/GAO-26-107820; DOE EM budget: https://www.energy.gov/sites/default/files/2025-06/doe-fy-2026-vol-6-em_0.pdf;
EIA: https://www.eia.gov/energyexplained/nuclear/us-nuclear-industry.php; EPRI 1003029: https://restservice.epri.com/publicdownload/000000000001003029/0/Product;
Anderson et al. 2026: https://strathprints.strath.ac.uk/96760/; Rocky Flats 771: https://www.energy.gov/sites/prod/files/edg/news/archives/documents/Building771FactSheet.pdf;
Fogle and Younkins: https://inis.iaea.org/records/06bm2-by573; USGS scrap: https://pubs.usgs.gov/periodicals/mcs2026/mcs2026-iron-steel-scrap.pdf;
OSHA 3348: https://www.osha.gov/Publications/OSHA3348-metal-scrap-recycling.pdf; Akl et al.: https://par.nsf.gov/servlets/purl/10300191;
SHEREC: https://cordis.europa.eu/project/id/101136056; NIOSH 2005: https://stacks.cdc.gov/view/cdc/197415;
NIOSH 2020: https://cdc.gov/niosh/bulletin/2020/wholesale-recycling.html; Swiss Re: https://corporatesolutions.swissre.com/dam/jcr:04741b43-c09f-4d31-9997-730a3b92fb78/res-hot-workf-factsheet.pdf;
CSB: https://www.csb.gov/assets/1/20/csb_hot_work_safety_bulletin_embargoed_until_10_a_m__3_4_10.pdf; O*NET: https://www.onetcenter.org/database.html.

Limits: no source gives the share of decommissioning or scrap cutting done by hand torch, or counts torch cutters as an
occupation; the cutting-torch fire share is from an insurer quoting NFPA; O*NET ratings are survey estimates, and task
10249's date from 2012.

## Real-world cutting setup

These are the trial's fixed settings, set once before a trial and not changed during it.

- **Cutter:** a 120 V hand air plasma cutter (20 to 30 A class) set to 20 A, with its shielded (drag) consumables: a drag shield 20 mm across whose rim stands 2 mm below the nozzle. Products and prices for buying or renting one are in the next section. Follow the cutter's manual for the work clamp and for routing the torch lead away from the robots' cables. Dry, oil-free compressed air at the pressure and flow the cutter's manual states, through a filter and regulator. Before the trial a welder makes a 100 mm test cut on scrap of the same plate along a straightedge at the rated 12.7 mm/s, and two more at 6 and 21 mm/s: the first must meet every segment criterion below, the slow one must show the dross or melted corner that fails a segment and the fast one must fail to cut through. If not, the current is adjusted in 1 A steps (18-22 A) and then fixed.
- **Arc control:** the robot presses no trigger. The torch's trigger circuit is wired to a hold-to-run switch held by the trial supervisor outside the screens. The supervisor closes it only while the shield is on the plate (within 2 mm of it), between the plate's two edges and within 10 mm of the line, as seen on the filtered camera view, and releases it when the torch leaves the near edge, lifts, or stops, or at any time to stop the trial. The simulation's arc follows the same criterion from the torch's pose.
- **Emergency stop:** one emergency-stop button beside the supervisor cuts both the robots' power and the cutter's output; releasing the arc switch is the second stop.
- **Torch and lead:** the hand torch weighs about 0.3 kg with consumables. A glass-filled nylon grip block (30 x 36 x 26 mm) is clamped on the handle with its centre 90 mm behind the head's axis. The lead and air hose hang from an overhead spring balancer above the stand, adjusted so that, with the torch held over the line, a spring scale at the grip reads the simulated 0.33 kg within 0.05 kg and under 1 N of sideways pull anywhere along the cut.
- **Straightedge:** a bright mild-steel flat bar 260 x 25 x 5 mm with a 30 x 88 x 5 mm steel tab welded to the middle of its back edge and a 30 mm red nylon handle block screwed to the tab, 95 mm from the working edge (0.39 kg in all). The working edge is measured straight within 0.2 mm before each trial and wiped clean of spatter.
- **Plate and stand:** 3 mm hot-rolled mild-steel plate, 200 x 215 mm, wire-brushed along the line, held on the welded steel stand by two strap clamps on the edge away from the cut, with 55 mm overhanging. The cut line is scribed and filled with a white paint marker. A witness line is scribed on the kept part 20 mm from the cut line, with a tick every 5 mm (41 ticks, every tenth punch-marked). The witness line and ticks are on the real plate only, not in the simulated scene.
- **Cutting circuit:** the stand rests on insulating pads on the station table and the work clamp is fixed to the stand, so the current returns through the plate, clamps and stand and never through a robot. Both grips (the torch's nylon block, the straightedge's nylon block) are insulating.
- **Sparks and dross:** the jet and molten metal leave the underside of the cut. The steel tray under the line and the overhang holds 10 mm of dry sand to catch dross and the hot offcut (the simulated tray is empty). The stand's edge is 15 mm back from the line so the jet doesn't cut it. Welding blankets cover the station table round the stand and tray and hang 30 cm down its front edge; a spark curtain stands 0.5 m beyond the plate's right side, the direction the sparks travel.
- **Arc light:** welding screens to EN ISO 25980, at least 1.8 m tall, close every open side of the cell. Nobody is inside while the arc can fire. Anyone looking at the arc directly uses a shade 5 filter or darker (plasma cutting under 40 A).
- **Cameras:** each robot camera has a fixed shade 5 filter (or an auto-darkening one with its dark state at shade 5) and a clear, replaceable cover glass against spatter. Before each session, a 100 mm test cut on scrap is recorded through the filtered robot cameras and the supervisor's filtered view, and both must show the torch and its shield, the guide (the straightedge) and the cut line throughout, with the arc not washing out the image; if it does, the auto-darkening filter's shade or the camera exposure is adjusted and the test repeated. The supervisor watches the same filtered view to run the arc switch. The simulated cameras are unfiltered.
- **Fume control:** cut outdoors or in an open bay, with the cut downwind of the supervisor and anyone nearby and a fan beside the cell drawing the plume away from the robots' cameras. OSHA's welding fact sheet (FS-3647) lists plasma cutting as a source of metal fume and advises exhaust near the plume and staying upwind outdoors; it also notes that working outdoors does not by itself guarantee adequate ventilation, so a local exhaust arm with its hood 20-30 cm from the cut is used where one is available. Only clean, uncoated mild steel is cut: painted, galvanised or oily plate is not allowed.
- **Fire protection:** nothing combustible within the screens or within 10 m in the direction of the sparks unless covered by blankets; a CO2 or dry-powder extinguisher at the cell; a 30 minute fire watch after the last cut of a session.
- **Heat on the robot:** aluminised (or leather) sleeves over both grippers and forearm links and over the wrist camera cables. The torch hand is 90 mm from the arc and above it; the straightedge hand is 105 mm from the line on the far side of the bar.
- **Hot parts:** the offcut and the cut edge are left until under 50 C (IR thermometer or temperature crayon; about 5 minutes) and moved with tongs before then. The robot is not asked to touch the offcut, and the hands-clear criterion keeps both grippers 6 cm from it at the end.
- **Noise and air:** plasma cutting at 20 A is about 90-100 dB(A) at the torch: hearing protection inside the room. The cutter's post-flow air is left to finish before anyone enters.

## Equipment and cost for real trials

Prices were seen in US retailer and rental listings on 11_16_26 and change often; any cutter with the capabilities in
the first column will do.

| Item | Capability needed | Example product | Price |
|---|---|---|---|
| Plasma cutter (buy) | 120 V, 20 A, drag-tip torch, cuts 3 mm steel | Forney Easy Weld 20P (item 251): 120 V, 20 A, drag torch with shield cup 85394, internal regulator and air dryer, needs 1.5 CFM at 60-100 psi (Forney operator manual) | about $390 (Home Depot) |
| Air compressor (buy) | at least the cutter's flow at 90 psi | Fortress 6 gal (Harbor Freight 59840), 2.7 SCFM at 90 psi | about $230 |
| Inline air dryer | dry, oil-free air | disposable desiccant filter (Harbor Freight Merlin 64637) | about $7 |
| Consumables | electrode, tip | Forney 85392 kit, 2 electrodes and 2 tips | about $30 a kit |
| Plate | 3 mm (1/8 in) hot-rolled mild steel | 12 x 12 x 1/8 in A36 flat stock, cut in two (Zoro) | about $25 for two plates |
| Fan | moves the plume away | any 20 in box fan | about $30 |
| **Purchase route total** | | cutter, compressor, dryer, consumables, fan | **about $690**, then about $13 of plate a trial |
| Plasma cutter (rent) | as above | Sunbelt Rentals catalogue 0270195, a Hypertherm Powermax30 (120/240 V) | about $120 a day at an independent rental shop (Sunbelt quotes on request) |
| Air compressor (rent) | 4 CFM at 90 psi | 20 gal portable compressor | about $30 a day |
| **Rental route total** | | cutter and compressor, plus dryer and consumables | **about $150 a day**, enough for a session of trials |

The simulated cut speeds come from a Powermax30-class manual (above). With another cutter, the three test cuts on
scrap before a trial (fast, rated and slow) set the current so the real edge meets the same criteria. The Powermax30's
maker gives 1 to 2 hours of arc-on time per electrode and nozzle set; one trial is 10 to 30 seconds of arc, so a set
lasts well over a hundred trials. The welding screens, blankets, shade 5 filters, extinguisher and hold-to-run switch
in the setup above are standard welding-shop items.

## Scoring: real measurement and simulated stand-in

The real trial is scored from the kept part (still clamped) and the offcut, once under 50 C, segment by segment between the ticks. Tools: a 150 mm steel ruler reading 0.5 mm, a 300 mm steel straightedge, a 0.5 mm feeler gauge, a small engineer's square, a lamp and a camera. Each segment is photographed from above with the ruler and ticks in view, and the trial video shows the rest.

The score is 0 if the torch or the straightedge falls off the table or the other gripper's fingertips come within 45 mm of the nozzle while the arc is on. Otherwise 0.8 x the fraction of the 40 segments that are sound (rounded to 0.001); 0.9 with all 40 sound and the offcut lying free in the tray; 1.0 with, as well, both tools back in place, both hands 6 cm clear and everything still for the last 1 s. Doing nothing scores 0.

Other outcomes:
- **Offcut still attached:** it stays where it is; the cut-through segments still count toward the 0.8.
- **Offcut pushed or snapped off:** bending it off does not make the remaining segments cut through: a torn or bent bridge is plain on the real edge. In the simulation each segment's joint breaks at the load that would fold or tear a 5 x 3 mm bridge of mild steel (2.8 N m, the plastic bending moment at 250 MPa yield; 6 kN in tension at 400 MPa), so a last bridge or two can be snapped by a push of about 15 N at the offcut's far end while the whole uncut strip (40 bridges, 112 N m) cannot. A broken joint's segment stays not cut through: the score is at most 0.8 x 39/40 = 0.78, with no credit for the drop.
- **Cut through but off the line, wavy, bevelled or overheated:** the offcut may drop, but those segments are not sound.
- **Two fast passes over the same segment:** the simulation adds their arc time, so the second pass cuts through; the real edge is judged as it is.
- **Straightedge left on the plate, torch not on its rest, a hand resting on a tool or the offcut:** at most 0.9.
- **Offcut dropped but not in the tray** (knocked away): at most 0.8.

| Criterion | Real measurement (how) | Simulated stand-in |
|---|---|---|
| Arc on | The supervisor's hold-to-run switch, closed only by the criterion above. | The torch in a hand (a gripper's fingertip midpoint within 2.5 cm of the grip block's centre), within 30 degrees of square, the shield's rim from 1 mm into the plate's top to 2 mm above it, the rim's centre over the plate between its edges and within 10 mm of the line. |
| Cut through | Along the segment no bridge of metal joins the offcut to the kept part: with the offcut gone, the edge shows a cut face its whole length, not a torn or bent one; with it still attached, the lamp behind shows light through the kerf along the whole segment. | 0.241 s or more of arc time with the nozzle over the segment (travel at 20.75 mm/s or slower). The segment's fixed joint to the offcut is then removed. |
| Edge on the line | Steel ruler square to the witness line at the segment: the kept part's cut edge is 18 to 22 mm from it. | The shield rim's centre (the nozzle) never more than 2 mm from the line while the arc is over the segment. |
| Edge straight (cut along the straightedge) | Lay the 300 mm straightedge against the cut edge: the 0.5 mm feeler enters nowhere in the segment. A cut run along a held straightedge passes; a freehand cut, or one made while the straightedge crept, wavers and fails. The video shows the shield against the straightedge. | For 80% or more of the segment's arc time, the shield's side within 0.75 mm of the straightedge's working edge (from both poses), the rim below the bar's top and within its length, and the straightedge flat on the plate (underside within 1 mm of the plate's top, tilted under 3 degrees). |
| Cut face square | Engineer's square on the plate against the cut face: no gap over 0.5 mm down the 3 mm thickness. | The torch never more than 10 degrees from square while the arc is over the segment. |
| Not overheated | No dross bead hanging more than 1.5 mm below the bottom edge (ruler) and the top corner not melted back more than 1 mm. | No more than 0.787 s of arc time on the segment (travel at 6.35 mm/s or faster, and no standing still with the arc on). |
| Offcut dropped | The offcut lies in the tray, free of the plate. | Its centre 25 mm or more below where it started, over the tray. |
| Hand away from the arc (else 0) | From the video, with the 5 mm ticks as scale: the gripper that is not holding the torch stays 45 mm or more from the nozzle whenever the arc is on. | The other gripper's fingertip midpoint within 45 mm of the nozzle with the arc on; latched. |
| Tools on the table (else 0) | Neither the torch nor the straightedge leaves the table. | Either one's centre more than 1 cm below the table's top; latched. |
| Tools back, hands clear and still | Ruler against marks at the rest and the pad: the torch's grip block within 15 mm in plan and 6 mm in height of its start, tilted under 10 degrees, turned under 20; the straightedge's handle block within 20 mm in plan and 6 mm in height, flat, turned under 20 degrees; both grippers 6 cm or more from the torch, the straightedge and the offcut; nothing moving in the last 1 s of the video. | The same numbers from the poses; still means under 1 cm/s and 0.5 rad/s for the torch, the straightedge and the offcut. |

Not simulated, and so judged only in the real trial: the kerf's 1.4 mm of lost metal, dross and the melted corner as shapes, warping of the thin plate, the lead's drag, sparks and fume, and the arc's light in the cameras.
