# Flip the plastic cap off a new injection vial

One gripper holds a new 10 mL saline vial upright on the bench; the other pushes the vial's plastic flip-off button up at its edge until the seal breaks and the button comes clear, without touching the bared rubber stopper or tipping the vial.

## Why it matters

Every dose drawn from a new single-dose vial starts with this act. It is routine for the 817,870 US medical assistants (BLS OEWS, May 2025, SOC 31-9092; O*NET 31-9092.00 Core task: "Prepare and administer medications as directed by a physician.") and for registered nurses who draw doses (3,379,720 employed, BLS OEWS May 2025, SOC 29-1141; O*NET 29-1141.00 Core task: "Administer medications to patients and monitor patients for reactions or side effects.").

Sources: https://www.bls.gov/oes/ , https://www.onetonline.org/link/summary/31-9092.00 , https://www.onetonline.org/link/summary/29-1141.00 , West Pharmaceutical, flip-off seal cap removal force testing: https://www.westpharma.com/blog/2023/october/flip-off-vial-containment-seal-cap-removal-force-testing

## Real items

| Item | Maker and model | Link |
|---|---|---|
| 0.9% Sodium Chloride Injection, USP, 10 mL single-dose plastic Fliptop vial, light green button | Pfizer (Hospira), NDC 0409-4888-10 | https://www.pfizerhospitalus.com/product/00409-4888-10 |

| Part | Scene | Real | Source |
|---|---|---|---|
| Crimp (seal) diameter | 20.0 mm | 20 mm seal | West page |
| Button diameter, height | 21.0 mm, 4.5 mm | about 21 mm, 4.5 mm | product photo, scaled to the crimp |
| Body diameter | 24 mm | about 24 mm | product photo |
| Height to button top | 58 mm | about 58 mm | product photo |
| Stopper opening | 8 mm | about 8 mm | product photo |
| Seal release | 12 N at the edge | 6 N to 35 N | West page (ISO 8362-6 maximum, YBB 00372003-2015 minimum) |

## Scoring

| Score | State |
|---|---|
| 0.25 | One gripper holds the vial, a finger on each side of its body |
| 0.5 | The seal broke while the vial was held: the button's near edge lifted off the crimp (a push down never breaks it; the 12 N release is a simulation value, so the film shows the button visibly lifting off the crimp) |
| 0.75 | The button is over 15 mm from the crimp's top |
| 1.0 | Over 30 mm, vial still held and upright (under 10 deg), both hands still for 3 s |
| 0 | Stopper touched after the button lifted, vial tipped over 30 deg or its base over 10 mm below the bench top, or seal broken with the vial unheld |

Definitions. Lifted: no point of the button is over the 8 mm opening, or its lowest point there is over 2 mm above the crimp's top. Stopper touched: a fingertip within 7.5 mm of the vial's axis (the 4 mm opening radius plus a 3.5 mm margin) and under 4 mm above the crimp's top (since fingertip points are sparse), or any of 48 button points over the opening and within 0.5 mm of the crimp's top. In a real trial the check is a close side camera on the vial's top: anything between the opening and the crimp's top plane counts.

## Run record

Scripted demo: 1.0 at step 294 (14.7 s), RTX 3090 Ti, rerun on RTX 4070 Ti SUPER. Dream-1: 0.0 (the model never held the vial; the run was valid, one session, 400 steps).

## Run

    python demokit.py task.py OUT --mode full

Lazy routes (GPU): `none` 0.0; `onearm` (no hold) 0.0; `hover` (no hold, the pushing hand goes in under the lip and touches nothing) 0.0; `pull` (open jaws straddle the button and lift) 0.25; `knock` (push the vial top sideways) 0.25; `pressdown` (hold the vial, press straight down on the button 8 mm in from its edge, 20 N commanded) 0.25: a push down never breaks the seal; `sidepush` (hold, the same push at the button's side edge) 0.25.

## Fidelity

Rigid bodies on PhysX. The button is joined to the crimp by a hinge that lets only its near edge lift, 0 to 15 deg about the far rim (a 4 N m/rad spring), and the button still collides with the crimp, so a push down meets the crimp and a push at a side edge does not release it in the simulation. A rule releases the hinge when the spring's moment reaches 0.252 N m (12 N at the 21 mm lever of the button), a 12 N lift at the near edge.

| Number | Simulation | Real | Source | Difference moves difficulty |
|---|---|---|---|---|
| Crimp (seal) diameter | 20.0 mm | 20 mm seal | West page | none |
| Button diameter, height | 21.0 mm, 4.5 mm | about 21 mm, 4.5 mm | product photo, scaled to the crimp | none |
| Body diameter, height to button top | 24 mm, 58 mm | about 24 mm, 58 mm | product photo | none |
| Stopper opening | 8 mm | about 8 mm | product photo | none |
| Seal release force at the near edge | 12 N | 6 N to 35 N | West page (YBB 00372003-2015 minimum, ISO 8362-6 maximum); the 12 N is an estimate | lower is easier; scored across the window, see below |
| Hinge stiffness before release | 4 N m/rad (0.252 N m / 4 N m/rad = 0.063 rad, about 3.6 deg) | not published; bound 2.6 to 10.6 N m/rad, from 0.5 to 2 mm of edge travel at the 21 mm lever before release | estimate with stated bound | stiffer is harder |
| Crimp top curl | 1.2 mm inward over its top 1.5 mm | not published; bound 0.5 to 1.5 mm inward (rolled aluminium seal edge, ISO 8362-3 seal drawings) | estimate from product photos, with stated bound | a deeper curl is harder |
| Masses | vial 15 g, button 2 g | not published; bound 12 to 20 g and 1 to 4 g (10 mL saline plus a plastic vial) | estimate with stated bound | heavier vial is easier to hold |

Demo scores across the bounds (each one a full run of the README command with the one number set; the push stroke is set to 30 mm where the stiffer or weaker seal needs it):

| Setting | Push stroke | Score |
|---|---|---|
| Release 6 N, 12 N (as run), 20 N, 30 N, 35 N | 14 mm for 6 N and 12 N, 30 mm for 20, 30 and 35 N | 1.0 |
| Hinge 2.6 N m/rad | 30 mm | 1.0 |
| Hinge 10.6 N m/rad | 14 mm | 1.0 |
| Curl 1.5 mm inward | 14 mm | 1.0 |
| Curl 0.5 mm inward | 30 mm | 1.0 |
| Vial 12 g with button 1 g; vial 20 g with button 4 g | 30 mm; 14 mm | 1.0 each |
| Pressdown and sidepush at hinge 10.6 N m/rad | as the route | 0.25 each |

Real-trial measurement of the estimates: pull the button at its near edge with a hook on a force gauge (0.1 N resolution) on five new vials of the listed product and record the peak; the accepted range is 6 N to 35 N. Measure the hinge: the edge's travel before release with a dial indicator at the near edge during that pull (accepted 0.5 to 2 mm). Measure the curl with calipers or a macro photo (accepted 0.5 to 1.5 mm), and weigh a vial and a button on a 0.1 g scale (accepted 12 to 20 g and 1 to 4 g). The scorer passes across all of these ranges, as the table shows.

Real-trial kit: one new 10 mL Fliptop vial per trial of the listed product (about $1 each, sold by the case through medical suppliers; the saline is prescription-only in the US, so an empty sterile 10 mL vial with the same 20 mm flip-off seal, sold by laboratory suppliers, serves equally). The seal does not reseat, so each trial opens a new vial, stood upright at the marked spot on the bench.
