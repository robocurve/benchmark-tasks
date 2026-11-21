# Safety closure, counted dispensing and scoring

The robot starts with a closed bottle. The benchmark uses the dimensions of a retail stock bottle and cap.
A bayonet mechanism models the need to push and turn a child-resistant closure. Its travel, spring and
angular thresholds are simulation parameters. Retail packaging may use a two-shell clutch and a screw thread.

## Geometry and physical sequence

Two opposed lugs on the cap overlap retaining ledges on the bottle neck. With the cap seated,
a counterclockwise stop extends 1.7 mm below the lug's upper face. A deeper stop blocks clockwise motion.
An annular collar on a 4 mm downward-travel prismatic joint bears against the edge of the skirt.
Its axial spring has stiffness 110 N/m and damping 0.35 N·s/m. It stays on the bottle throughout the trial.
The cap and bottle interact through their convex colliders. Neither body is attached to a gripper.
There are two 41-degree windows in the retaining ledges, centred at 49.5 and 229.5 degrees.
Matching stripes on the cap and bottle provide references for monitoring relative twist. The cap skirt's modelled inner diameter is 40.6 mm to clear the ledges. Its nominal dispensing region is 36 mm across.

The right gripper steadies the bottle. The left gripper pinches the cap, compresses the collar, turns
the cap counterclockwise and lifts it free. The robot turns the empty cap open side up before dispensing.
During the pour the left gripper holds it clear of other objects and the right gripper tips the bottle.
Returning extras and tipping the cap into the cup are performed with one hand, the left, while the right arm waits.
All 30 tablets move through rigid-body dynamics, including gravity and contact with the containers.

## Verifying a trial

The opening scorer measures translation and rotation in the bottle frame at 20 Hz. Both fingertips on each
of two different grippers must press at least 1 N each on their respective object. From the closed alignment,
the cap must compress at least 1.5 mm and stay compressed by at least 1 mm through 20 degrees of turn.
Its orientation must reach the 34–68 degree release range, then it must rise 15 mm from its initial height
while still above the neck and pinched. Moving the bottle through space with its cap attached leaves these
relative measurements unchanged. The trace records the measured sequence, including failed opening attempts.

Any tablet escaping before the verified opening voids the run. A tablet entering the cup must previously
have lain inside the held cap for at least 0.25 s. With tablets in it, the cap must be at least 5 mm off the
table, with a valid two-fingertip grasp. The scorer treats a support force of 0.02 N or more from the bottle
or cup as external support. The card defines the dose tiers, release and stillness windows. For a hardware
trial, video from above and from the side can show the alignment stripe, axial travel, grip, tablet path
and final inventory. Two fingernail-sized witness marks and a ruler beside the station give distance and angle references.

## Recorded scripted trial

In the seed-0 Isaac Sim trial, all 30 tablets started in the bottle, and the seated cap's outer top was
94.6 mm above the bottle base. Inspect Robots awarded 1.0 at control step 2,363 (118.15 s of simulated
time, 528.91 s wall time). The trace records the following sequence.

| Step | Observed motion or count |
| ---: | --- |
| 395 | The left gripper depressed the cap by 1.69 mm. Its fingertips pressed with 9.00 and 8.77 N while the right gripper held the bottle. |
| 440 | The cap turned 20.03° counterclockwise while compressed by 3.11 mm. |
| 463 | The lugs reached the release window at 34.13° of turn and 2.91 mm of compression. |
| 507 | At 51.58° of turn, the gripper had lifted the cap 17.57 mm. The opening scorer registered removal with both hands still gripping. |
| 1,503 | After turning the empty cap over and regrasping it, the robot poured five tablets into the held cap. All 25 others remained bottled. |
| 1,946 / 1,959 / 1,973 | The robot tipped three extra tablets from the cap back into the bottle, one at a time. Two remained in the held cap. |
| 2,186 / 2,190 | The two remaining tablets entered the medicine cup, one at a time. |
| 2,363 | Both grippers were clear and the objects were still. The upright cup held two tablets, the upright bottle held 28, and the cap was empty. |

The peak recorded arm-to-tablet force and arm-to-arm force were both 0.0 N. The return of three extras
is visible in the per-step bottle and cap counts in `media/grader-trace.jsonl`.

## Dispensing method

The school-nurse checklist calls for opening the bottle, holding the cap in one hand, pouring a counted dose
into it with the other hand, then transferring the tablets to a clean medicine cup.
The Open RN checklist also describes using the cap to transfer tablets from a multidose bottle into the cup.
[School Nurse Toolkit](https://educate.iowa.gov/pk-12/student-supports/specialized-support/school-nurse-toolkit/section-8-medication-administration-delegation-school-nurse/general-medication-administration-skills-checklist/tablets-capsules) · [Open RN checklist](https://med.libretexts.org/Courses/University_of_South_Carolina_Upstate/Nursing_Skills_(OpenRN)/05%3A_Administration_of_Enteral_Medications/5.04%3A_Checklist_for_Oral_Medication_Administration)

## The objects, and where their numbers come from

| Object | Value used | Source | Status |
| --- | --- | --- | --- |
| Bottle | 120 cc white HDPE packer, 38-400 neck, 1.989 in (50.5 mm) across, 3.402 in (86.4 mm) tall, 13.63 g | Container & Packaging item B723WH, <https://www.containerandpackaging.com/products/hdpe-packer-bottle/B723WH> | read on the page |
| Bottle bore, shoulder, neck | bore 31 mm; shoulder from 62 to 71 mm up; neck 35.2 mm across under the thread | none: the page gives no drawing | estimates, to be measured on the bottle |
| Cap | 38-400 white polypropylene push-down-and-turn child-resistant cap, 1.756 x 0.677 in (44.6 x 17.2 mm) outside | The Cary Company item 68WARG, <https://www.thecarycompany.com/38mm-38-400-white-child-resistant-cap-argus-loc-pdt-te-liner> | read on the page |
| Cap inside, cap mass | 36 mm across, 13.2 mm deep; 6 g | none | estimates |
| Tablets | round, brown, 10 mm, coated, 200 mg ibuprofen | DailyMed label of a 250-count bottle (Chain Drug Marketing Association, NDC 63868-268-25): "Color: Brown, Shape: Round, Size: 10 mm", <https://dailymed.nlm.nih.gov/dailymed/drugInfo.cfm?setid=c13455a3-307a-4fe1-e053-2a95a90adf8e> | read on the label page |
| Tablet thickness, edge, mass | 5 mm at the middle, 2.6 mm edge band, 0.33 g | none | estimates, to be measured |
| Medicine cup | 1 oz graduated polypropylene cup, top 1 7/8 in (47.6 mm), bottom 1 1/8 in (28.6 mm), 1 3/8 in (34.9 mm) tall | WebstaurantStore, Choice item 346MEDCUP1Z, <https://www.webstaurantstore.com/choice-1-oz-disposable-polypropylene-graduated-medicine-cup-case/346MEDCUP1Z.html> | read on the page |
| Cup mass | 1.1 g | the same page: 12.52 lb shipping weight for 5,000 cups | upper bound |
| Friction | tablet on plastic and tablet on tablet 0.35; plastic 0.6 | none | estimates |

For a hardware trial, use a matching bottle and push-down-and-turn safety cap, along with 30 inert
placebo tablets of the listed size. The product label supplies the tablet dimensions and brown colour
used for visibility in the white cap. Check the cap's actual travel and opening angle against the marked
closed position, and record those measurements alongside the trial.

## Third-party assets and licenses

All geometry is procedural, generated by `tools/build_assets.py` from `layout.py`; the scene code, the scorer
and the scripted controller are written for this task and provided under the MIT License in `LICENSE`. No
manufacturer mesh, image or CAD model is bundled; product pages were read for dimensions only.

| Asset or dependency | Use | License | Bundled here |
| --- | --- | --- | --- |
| YAM arm and gripper model | as distributed with yam-arena | MIT (LICENSE.i2rt) | No |
| Standard station (table, cameras, lighting) | yam-arena 0.1.0 | its README states no license yet | No |
| Isaac Lab-Arena, Isaac Sim | simulation runtime | Apache 2.0; NVIDIA license | No |
| Inspect Robots, inspect-robots-yam | evaluation harness | as distributed | No |
