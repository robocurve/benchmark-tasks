# Pin a crimped contact into cavity 3 until the locking fingers click, prove it with a tug, then fit the secondary lock

In wire-harness assembly, crimped wires are pinned into connector housings by hand, cavity by cavity, following a
pinout. A contact goes into its cavity from the wire side until its locking fingers snap in behind the contact's
shoulder (the "click"); the assembler tugs the wire to check it holds, and when all contacts are in, pushes the
secondary lock (a wedgelock) into the connector's mating face; it only goes fully home when every contact is seated. A contact that hasn't clicked in looks
inserted but backs out when the connector is mated: a classic harness fault.

Here a sealed 4-way housing lies loose on the table. One arm holds it; the other takes the crimped wire right behind
the crimp, lines the contact up with cavity 3 (as on the pinout card) and pushes it in until it clicks, tugs the wire
to prove the lock; then the hands swap: one holds the housing by its rear half while the other pushes the wedgelock
into the mating face. The judgments tested: the right cavity from a pinout, pushing close
behind the crimp (a wire pushed far back bends instead of driving the contact), holding the loose housing against the
push and the tug, and fitting the secondary lock only after the contact is seated (pressed in over a contact short of
the click, it damages the housing).

## Credit, and how the review is met

This task takes up the idea of an earlier submission, `contact-insert`, reviewed as TK-8FNXAK. The hold on the idea lapsed with no further revision. This version starts from that submission's files (its parts, scene, bench and demo code, reused and extended) and answers that review and then the review of this version (TK-5CM48W), point by point below. The scoring and the demo changed (code in `ct_parts.py`, `task.py`, `demo.py`), so every run was redone on this revision: the scripted demo, its seven variants and Dream-1 (`task.json` `scores`).

| Reviewer point | What changed |
| --- | --- |
| Importance: the jobs category does not establish 500,000 US workers; if this is robot-building work, change the domain to Impact on AI timelines and describe a genuine robot-harness assembly application | `category` is `Impact on AI timelines`; `context` and *Why this matters for AI timelines* describe pinning, tugging and locking the sealed harness connectors that wire robots, with sources; the occupation claim is removed |
| Clarity and reproducibility: specify an available housing, matching contact and secondary lock, or fabrication files | *Real-trial parts* and `task.json` `setup`: TE DEUTSCH DT06-4S plug, W4S wedgelock and 0462-201-16141 size-16 socket contact (sold as one kit), HDT-48-00 crimper, 18 AWG wire. The remaining difference (the W4S enters the mating face, the simulated comb a top slot) is stated in *Fidelity* and `task.json` `fidelity` |
| Construct validity, scoring, feasibility: require the robot's own tug in the scoring and demonstrate it within the action limit; the independent retention test can remain | New latched `robot_tug` check (`ct_parts.wire_gripped` and the update step); `locked()` in `task.py` now needs it; the demo tugs after the click while the other hand holds the housing; new `no_tug` variant (0.25). The independent retention test is kept as check (c). Demo rerun on this revision: success at step 1,796 of 3,000; `no_tug` measured 0.25 |
| Scoring: replace the concealed 0.3 mm contact-depth check with observable checks, and specify how the secondary lock's seating is checked | Score uses a white reference band on the wire (within 1.0 mm, a ruler), the robot's tug and the retention test; the 0.3 mm tolerance now only drives the simulated finger hold, not the score. Lock seating: simulation, tab on the top within 0.3 mm with every finger closed; real trial, W4S snapped in flush with the mating face (straightedge and 0.3 mm feeler gauge) |
| Instruction | "into the slot on top of the housing" became "into its slot in the housing" (now "into the housing's mating face", see TK-5CM48W below) |
| Runs | Scripted demo, its seven variants and Dream-1 rerun on that revision's task file (SHA-256 prefix `6bacb68a6465`); all rerun again for TK-5CM48W (see *Runs of this revision*) |
| TK-5CM48W, feasibility: model the secondary lock as the W4S wedgelock inserted into the mating face, with a full-score demo of that placement | The lock is now a wedgelock: a 26 x 22 mm face plate with four wedge blades that slide into the housing's mating face along channels beside the cavities and end beside the locking fingers, where an open finger stops them. The top slot is gone. The demo swaps hands after the tug: the right hand holds the housing's rear half while the left takes the wedgelock from its holder by the plate's edges, turns it square to the mating face, lines it up and pushes it home. Demo on this revision: 1.0, success at step 1,991 of 3,000 |
| TK-5CM48W, feasibility: a low-cost way to prepare the contacts | *Real-trial parts*: an IWISS IWD-1620 size-16 solid-contact crimper (8 indents, 16-20 AWG, $24.99) crimps the 0462-201-16141 contacts onto 18 AWG wire in place of the HDT-48-00 |
| TK-5CM48W, note: how the independent 30 N retention test is done without disrupting the trial | *Scoring*, check (c): in a real trial it is done after the run has ended, so the robot's run is never touched; see there for why the result is the same |
| Blind review | The earlier submission is credited by its title and review ID only, without its author's username |

## Why this matters for AI timelines: robots building robots

Robot arms, mobile bases and humanoids are wired with harnesses: crimped contacts pinned into sealed connector housings,
cavity by cavity from a pinout, then tugged and locked. That step is still done by hand: "Wire harness production
remains one of the most manual areas in automotive manufacturing" (Automotive Manufacturing Solutions, *Has the wire
harness finally become robot-ready?*, 9 Sep 2026:
https://www.automotivemanufacturingsolutions.com/automation/has-the-wire-harness-finally-become-robot-ready/2733395 ).
Sealed 4-way connectors with size-16 contacts, such as TE's DEUTSCH DT family (environmentally sealed, cable-to-cable
and cable-to-device, 13 A per size-16 contact: https://www.te.com/en/products/connectors/automotive-connectors/intersection/deutsch-dt-series-connectors.html ),
can carry the power and signal runs between a robot's battery, motor drives and sensors. A robot that can pin a contact
by pinout, prove it with its own tug and fit the secondary lock can help build the harnesses of the next robots, one of
the manual steps between building one robot and building many. The published DT assembly steps are the ones this task
instructs: grasp the crimped contact behind the barrel, push it straight into the grommet "until a click is felt", a
slight tug confirms it is locked, then insert the wedgelock (BuyDeutsch, *DT series assembly instructions*:
https://buydeutsch.com/blogs/learning-corral/deutsch-dt-series-connector-assembly-instructions ).

The simulated parts are our own simplified designs sized like that plug; *Real-trial parts* below lists the purchasable
equivalents.

**Status:** the scoring now requires the robot's own tug and the demo now performs it; the scripted demo, its seven
variants and the Dream-1 trial were rerun on this revision (below and `task.json` `scores`).

## Runs of this revision (task file SHA-256 prefix `dc30e735b9ae`, parts module `ct_parts.py` `c42c2d0815e2`)

- **Scripted demo (`demo.py --variant full`):** score `1.0`, success at step `1,991` (the run stopped at step `2,051`
  of the `3,000`-step limit; robot tug and retention test passed, wedgelock home on the mating face). Result:
  `runs/demo_result.json`.
- **Dream-1 (`dreamlabs/dream-1-beta`, seed 0, CPU physics):** score `0.0`, ended at the `3,000`-step limit
  (`max_steps`); the arms moved toward the parts but never took the wire or the wedgelock (54 of its 62 requests were
  retried after "expired"). Log: `runs/dream_log.json`.
- **Variants:** `full` 1.0, `hold_on` 0.75, `no_lock` 0.5, `no_tug` 0.25, `shallow` 0.25, `wrong_cavity` 0 match the
  ladder; `forced` scored 0.25, not 0 (see *The demo*). Results: `runs/variants/<variant>/result.json`.

## How this task differs from existing tasks

The skill tested here is **pinning a pre-crimped contact into a sealed connector housing so that the cavity's locking
fingers snap in behind its shoulder**. Whether it has locked is hidden: a half-seated contact looks the same as a locked
one from outside and only shows itself when the wire is tugged, or later, when the contact backs out as the connector is
mated. The secondary lock (TPA/wedge) then verifies the whole housing: it only slides home when every contact is fully
seated, and forcing it over a half-seated contact damages the housing.

| Task | What it tests | Why this task is different |
| --- | --- | --- |
| #2660 rj45-crimp | Ordering eight bare wires into an RJ45 plug and making the joint with a ratchet crimper; pull test | There the *tool* makes the connection and the plug has no per-contact lock. Here the contact is already crimped and must **lock itself** in its cavity; the skill is the insertion feel, the tug that proves the lock, and the secondary lock that only fits over fully seated contacts. No crimping tool is used. |
| #741 crimp-a-terminal-onto-a-cable-end | Crimping a tube lug onto a cable | The crimp step that comes *before* this task. Here the crimp is done; the task starts where #741 ends. |
| #1970 han-hinged-frame | Loading whole Han-Modular modules into a hinged frame | Module level, not contact level; no locking fingers, no tug test, no secondary lock. |
| #268 push-in-terminal, #705 push-in-terminal-tool, #2596 | Landing a wire in a push-in terminal | A terminal clamps a bare or ferruled wire end; nothing locks inside a housing cavity and there is no cavity map. |
| #464 harness-branches-to-reachable-sockets, #2219 | Plugging finished harness connectors into sockets | Mating complete connectors, not building one. |
| Cord clamp (a pending task) | Clamping a cable's sheath under a strain-relief bar | A screwed clamp on the outside of a cable. This task is inside the connector, contact by contact. |

## Files

| file | what it is |
| --- | --- |
| `task.py` | the yam_arena task file: scene (`OBJECTS`), `success`, `score`, `distance`; docstring = what is on the table |
| `ct_parts.py` | the custom parts (housing with its locking fingers, contact and wire, secondary lock, holder, lead rest, pinout card): geometry, USD writer, the per-step update (cavity, seating, fingers, the tug, the latches) and the hook that lets yam_arena spawn them |
| `demo.py` | scripted two-arm demo, closed loop on the simulated object poses, and its variants |
| `ct_bench.py` | test bench for the connector alone, a "virtual hand" pushing the contact (the table under *Connector bench*) |
| `preview.py` | renders the starting layout from the station cameras |
| `runs/full_video/setup.png` | the starting layout, the first frame of this revision's scripted demo (top camera) |
| `task.json` | submission metadata |

## Run and score a trial

Environment: yam_arena (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, PhysX at 240 Hz). Run at 20 Hz control with object
jitter off (one fixed layout, no seeds needed):

    inspect-robots run --embodiment yam_arena -E task=task.py -E control_hz=20 -E object_jitter=0 -E device=cpu \
        --policy <policy> --scorer task_score --max-steps 3000 \
        --instruction "Push the crimped contact into cavity 3 of the connector, as on the pinout card, from the wire side, until it clicks; hold the housing with one hand and push the wire close behind the crimp with the other. Tug the wire lightly to check it is locked, then push the secondary lock fully into the housing's mating face. Then let go of everything."

CPU physics (`-E device=cpu`) is what the task was built and tested on (the contacts are sub-millimetre). The cameras
render on the GPU. `task_score` gives 1.0 when the episode ends in success, otherwise `score()` at the last step.
Runtime: about 30 s of start-up, then about 9 control steps per second with the three cameras rendering (the earlier
revision's scripted demo: 1,869 steps in 205 s with 640 x 360 video; with the robot's tug: 1,856 steps in 366 s; this
revision's, with the wedgelock: 2,051 steps in 810 s with video while three other simulations shared the GPU); a full
3,000-step trial is about 6 minutes.

Scripted demo and bench:

    python demo.py --variant full --out demo_runs/full      # result.json, steps.jsonl, one video per camera
    python demo.py --variant all --no-video                  # all seven variants in one process
    python ct_bench.py --case seat,half,forced --out bench_runs

## Scoring

| score | condition |
| --- | --- |
| 1.0 | the contact in cavity 3, locked, the secondary lock fully in; no fingertip within 20 mm of the housing, contact, wire or lock for 1 s, all still (success, ends the run) |
| 0.75 | the same, but something still touched |
| 0.5 | in cavity 3 and locked; the secondary lock not fully in |
| 0.25 | in cavity 3 only (including a clicked contact the robot never tugged) |
| 0 | otherwise; or (latched) the contact ever locked in another cavity; or the secondary lock pressed down to the locking fingers' height while a contact was in a cavity but not clicked (the housing damaged) |

- **In cavity 3:** the contact's nose at least 5 mm into cavity 3 from the rear face, on its axis (within 1 mm),
  pointing in (within 20 degrees). Cavities are numbered as seen from the rear (1 top left, 2 top right, 3 bottom
  right, 4 bottom left), as on the pinout card.
- **Locked:** three observable checks, all needed.
  1. **Mark:** a 2 mm white reference band on the wire, its centre 15 mm behind the crimp end, within 1.0 mm of its
     seated position (seated, it lies 3.9 mm outside the rear face in simulation). Real trial: a ruler from the rear
     face; the seated position is measured once on a reference contact whose wedgelock went home.
  2. **The robot's own tug:** with the contact clicked, a hand closed on the wire within 60 mm of the crimp draws it
     back along the cavity for at least 0.5 s while the other hand holds the housing. In simulation: every fingertip
     of that hand within 8 mm of the crimp barrel or the first 60 mm of wire, and the contact drawn 0.15-1 mm back from
     its seated depth (off the step, onto its fingers, which sit 0.26 mm behind a seated shoulder: the wire is taut).
     Real trial: the video shows the hand closed on the wire, the wire taut in line with the cavity, and the mark
     moving under 1 mm.
  3. **The independent retention test:** once no fingertip has been within 25 mm of the contact or the first 60 mm of
     wire for 1 s, the wire is pulled out of the housing, the housing held: 30 N, ramped over 1 s, held 0.5 s; it holds
     if the contact moves less than 1 mm. Real trial: done after the run has ended and the end photo is taken, so
     nothing touches the cell while the robot works: the supervisor puts the housing in a soft-jaw vise and pulls the
     wire with a spring scale to 30 N, the mark against a ruler. Doing it at the end gives the same result: a contact
     that holds moves under 1 mm, so the later checks are unchanged, and the wedgelock goes home only over clicked
     contacts, so a fitted wedgelock never hides an unclicked one. In simulation the task applies the pull as soon as
     the hands have been off the wire for 1 s, when no hand is near it. (For reference, common sealed-connector specifications require a size-16 contact to stay put
     under at least 25 lbf, 111 N, tested by MIL-STD-1344 method 2007.1; 30 N is a light check.) **In simulation** see
     *Fidelity*.
- **Lock in:** the secondary lock fully home, every locking finger closed. In simulation: the wedgelock's plate flat
  on the mating face (within 0.3 mm, square to it, centred within 1 mm across and 2.2 mm up and down, the blades'
  play in the channels), its blades in the channels. The plate is 2.0 mm out from the face when the blades' tips reach the fingers: pushed further while a
  contact is in a cavity but short of the click (or a blade loaded 20 N against an open finger), the lock was forced
  (latched 0). Real trial (DT06-4S): the W4S wedgelock pushed into the plug's mating face until it snaps and sits
  flush; a straightedge across the face leaves no gap a 0.3 mm feeler gauge enters.
- Measurable with a ruler, a feeler gauge, a straightedge, a spring scale and the video.

No unintended strategy scores: the contact only counts from the rear on the cavity's axis; the robot's tug only counts
on a clicked contact in cavity 3 (pulling an unclicked one just pulls it out); pushed in short of the click it is not
locked (and the secondary lock pressed over it scores 0 for good); clicked into another cavity it
can't come out without an extraction tool (latched 0); the secondary lock without a locked contact in cavity 3 scores
nothing.

## Setup

All positions from the arm bases' midline (x forward, y left); the table top is the reference height.

- **Housing** (loose): sealed 4-way connector housing, black glass-filled nylon, 24 mm wide (x) x 40 mm long (y) x
  24 mm tall, lying on its side, its rear face (wire side) centred at x 0.30 m, y 0.02 m, facing -y (toward the right
  arm). Four square cavities 3.9 mm across, 10 mm apart (centres 5 mm left/right and above/below the face's centre),
  each with a lead-in funnel (6 mm across at the face), numbered by moulded dots. Its mating face (the front, facing
  +y) has four 1.1 mm wide channels (widening to 2.3 mm over the last 3 mm at the face), one on each side of each
  column of cavities, running 22 mm back to the locking
  fingers, for the wedgelock's blades. Inside each cavity two locking fingers (see below).
- **Contact and wire**: a crimped size-16 socket contact (gold; 21 mm long: nose 2 mm across, shoulder 3.8 mm across,
  crimp barrel 3.7 mm across) on a red 1.0 mm2 wire (2.6 mm across, 140 mm, its far end free), lying on a grey
  two-block rest (blocks 20 mm tall and 20 mm wide; the contact on the first, the wire on the second, a 38 mm gap
  right behind the crimp), the crimp end at x 0.305 m, y -0.09 m, the nose toward the housing (+y).
- **Secondary lock**: an orange wedgelock: a face plate 26 x 22 x 6 mm (1 mm wider than the housing on each side) with four wedge blades (0.8 mm thick, 16 mm
  wide, 22 mm long, beside each column of fingers), standing blades-down in a grey holder block (24 x 34 x 26 mm, a
  18 x 20 x 24 mm pocket for the blades, the plate on its rim) at x 0.22 m, y 0.17 m, left of the housing.
- **Pinout card**: white, 50 x 36 mm, at x 0.19 m, y 0.10 m: the four cavities as seen from the rear, cavity 3 red.
- **Reference mark**: a 2 mm white band on the wire, its centre 15 mm behind the crimp end.

## Real-trial parts

| part | purchasable equivalent |
| --- | --- |
| housing | TE DEUTSCH DT06-4S 4-way plug (black, sealed); the pinout card marks the cavity moulded "3" on its rear grommet |
| secondary lock | TE DEUTSCH W4S wedgelock (orange), pressed into the plug's mating face |
| contact | TE DEUTSCH 0462-201-16141 size-16 solid socket contact |
| kit | the three above sold together as a DT06-4S kit (1 plug, 1 W4S, 4 contacts): https://www.masterelectronics.com/en/te-connectivity-deutsch-automotive/dt064scomplete-54386879.html |
| crimp tool (low cost) | IWISS IWD-1620 size-16 crimper for solid barrel contacts, 8 indents, 16-20 AWG, $24.99: https://www.iwiss.com/products/iwd-1620-deutsch-size-16-crimper-solid-barrel-contact-circular-connector-contacts-crimping-tool-8-indents-wire-crimper-awg16-20 |
| crimp tool (reference) | TE DEUTSCH HDT-48-00 hand crimper for solid size 12, 16 and 20 contacts: https://www.digikey.com/en/products/detail/te-application-tooling/HDT-48-00/5399432 |
| wire | 140 mm of red 18 AWG automotive wire, crimped per the tool's instructions, the white band painted 15 mm behind the crimp end |
| contact preparation | strip 6 mm, crimp the contact with either tool, then check each lead before use: it holds a 30 N pull on the spring scale with the contact in a vise, and the wire's strands are all inside the barrel's inspection hole. Leads are reused between trials (extracted with the DT removal tool); one whose crimp was pulled loose is remade |
| rest, lock holder, pinout card | blocks and card to the sizes in *Setup* (any rigid material; the card printed) |

Check the contact's wire range on its datasheet before buying wire. Contacts are removed between trials with the DT
removal tool for size-16 contacts; a plug whose grommet is torn is replaced.

## How the parts are built

yam_arena's task files only offer single primitives and library assets, and yam_arena has no extension point for
jointed parts. `ct_parts.py` is the task's own minimal mechanism; nothing in yam_arena is modified. A `Part` subclasses
yam_arena's `Box`; `task.py` calls `ct_parts.install()`, which wraps `yam_arena.scene.task_object`, `object_states`
and `make_env` when that module is imported, and spawns the parts from USD files it writes to a temporary folder.

**Geometry and licensing:** every part's geometry was created by us for this task and is written into `ct_parts.py` as
coordinates; no manufacturer CAD, meshes or other third-party assets are used. Sizes follow common sealed 4-way
connectors with size-16 contacts and 1.0 mm2 automotive wire; the details are our own.

**The housing** is a free articulation: a body of box colliders (square cavities, lead-in funnels, the step that stops
the contact's shoulder, the pin exits, the wedgelock's channels from the mating face) and, in each cavity, two locking fingers on prismatic joints
with springs (4 N/mm), one on each side. A finger's rear edge is bevelled (30 degrees) and its front face undercut: the
contact's shoulder (a 31 degree ramp) pushes both fingers apart, and once it is past they snap in behind it.

**The contact and wire** are a free articulation: the contact (one body: nose, socket, shoulder, neck, crimp barrel) and
fourteen 10 mm wire links joined by two bending joints and a twisting joint each, with springs (EI 8e-5 N m2, our
estimate for a 1.0 mm2 wire).

**The secondary lock** is a free articulation: a wedgelock, its face plate and four wedge blades on stiff springs
(along the blades). The blades slide in from the mating face along the channels and end beside the fingers: a closed
finger leaves them room, an open one (a contact short of the click) is in their way.

**The per-step update** (from the patched `object_states`, once per control step, before scoring) finds the cavity the
contact is in and how deep, the fingers' opening, the lock's depth, runs the tug and latches the faults.

### Connector bench

`ct_bench.py`: the parts in the task's scene, the arms parked, the contact set at cavity 3's mouth. A "virtual hand"
holds the housing (the push's reaction plus a weak spring) and pushes the contact along the cavity, harder while it
moves slower than 10 mm/s (up to 40 N). Results (bench run 35):

| case | what the hand does | result | score |
| --- | --- | --- | --- |
| seat | pushes until seated, lets go; the tug follows | clicks at a peak push of 5.1 N (the fingers open 0.56 mm), seated 11.3 mm in, the fingers closed; the tug moves it 0.46 mm: passed | **0.5** in the earlier revision; **0.25** expected now (the bench has no robot tug), not re-run |
| half | pushes until a finger is 0.7 mm open (the shoulder under the fingers), lets go | stays short of the click (9.0 mm in, a finger 0.54 mm open); no tug (not seated) | **0.25** |
| forced | the same, then presses the secondary lock in (40 N) | forced (latched) | **0** |

The table above is from the earlier top-slot lock. The bench's `lock` and `forced` cases were ported to press the
wedgelock into the mating face, but in their first run the placed wedgelock was thrown out of the scene (a solver
blow-up), so the bench does not check the new lock; the demo and its variants do.

## The demo

Left arm takes the housing's front half from above (tilted 30 degrees, its wrist leaning away from the other hand)
and holds it on the table. Right arm takes the wire 15 mm behind the crimp over the rest's gap (tilted the other way),
lifts the contact, lines its nose up with cavity 3 (closed loop on the measured nose) and pushes it in along the
cavity until its fingers reach the housing's rear face; lets go, takes the wire again 30 mm behind the crimp and
pushes on until the contact clicks (if the push stalls short of the click: back off 1.5 mm and push again). Still
gripping the wire, with the left hand still holding the housing, it **tugs**: draws the wire back along the cavity
(3 mm/s, at most 12 mm of command) until the contact is pulled 0.2 mm back onto its locking fingers, holds that 1 s,
eases off, lets go and moves home. The left hand lets go of the housing and moves away; with no hand near the wire,
the task's independent retention test tugs. The hands then swap: the right hand grips the housing's rear half from
above and holds it, and the left takes the wedgelock out of its holder by its plate's two side edges (the hand tilted
30 degrees about the fingers' closing axis), lifts it out, turns it so its blades point back along the cavities,
lines it up in front of the mating face (closed loop on the measured lock: position and squareness), pushes it in
along the channels until the plate is on the face, lets go and backs away. The right hand lets go of the housing and
both arms move home.

**This revision** reaches success at step 1,991 of 3,000, score 1.0 (`runs/demo_result.json`). The earlier
revision (top-slot lock, with the tug) reached it at step 1,796.

Variants proving the ladder with the arms (`--variant all --no-video` runs all seven in one process and also checks the
reset); scores measured on this revision (`runs/variants/<variant>/result.json`), each matching the ladder:

| variant | what the arms do | score (this revision) |
| --- | --- | --- |
| `full` | as above | **1.0** (success) |
| `hold_on` | as above, but the hand stays on the secondary lock | **0.75** |
| `no_lock` | the contact clicked and tugged, no secondary lock | **0.5** |
| `no_tug` | the contact clicked, the secondary lock fitted, but the robot never tugs | **0.25** |
| `shallow` | the contact pushed in short of the click, no secondary lock | **0.25** |
| `forced` | the contact short of the click, the secondary lock pushed in over it | **0.25** (expected 0): released short of the click, the contact was pushed back out by its own fingers' springs to 5.3 mm in, clear of them, so no finger was held open and nothing was forced; the wedgelock then stopped 22 mm out, at the channels' mouths, after a knock during its line-up. The contact is in cavity 3 but not locked: 0.25 |
| `wrong_cavity` | the contact clicked and tugged in cavity 2, the lock fitted | **0** |

## Fidelity: what is simplified

- **The locking fingers are held once they have clicked.** The click itself is physical (the shoulder pushes the
  sprung fingers apart and they snap in behind it), but between parts this small the solver allows a few tenths of a
  millimetre of interpenetration, and a 30 N pull dragged the shoulder back past the fingers in our bench runs. So
  once a contact is seated with both fingers snapped in, the task holds that cavity's fingers shut (their joint limits
  closed to 0.1 mm) until the next reset; on a real connector the undercut fingers hold far more than 30 N.
- **The robot's own tug is read from the contact, not a force sensor:** it counts when every fingertip of one hand is
  within 8 mm of the wire and the clicked contact is drawn back 0.15-1 mm from its seated depth onto its fingers for
  0.5 s. The parts' solver slop (a few tenths of a millimetre) is close to that 0.15 mm threshold.
- **The wedgelock is simplified:** it enters the mating face as the W4S does and only goes home over clicked
  contacts, but its shape is our own (a flat plate with four straight blades), it has no snap of its own (home is the
  plate on the face), and its blades back up the fingers by position only.
- **The simulated retention test is 8 N**, enough to pull out a contact that the fingers don't hold (only the wire seal's friction
  holds it); a contact whose fingers have clicked and are held counts as holding the 30 N of a real trial.
- **Forcing the secondary lock is judged by position**: its blades' tips pushed to the fingers while a contact is in
  but not clicked (or a finger is open more than 0.3 mm), or a blade loaded 20 N. Physically the blades are also
  stopped by an open finger, but not reliably.
- The finger springs are light (4 N/mm: a click at about 5 N of push); real size-16 contacts typically need more.
- Parts are heavier than real for the solver: housing 200 g (real ~10 g), contact 10 g (~1 g), wire 1.5 g per 10 mm
  (~0.15 g), fingers 20 g; their dimensions are true to size.
- The contact lies on a two-block rest so that a gripper can take the wire right behind the crimp (the 2.6 mm wire
  lying on the table is too thin for the YAM's fingers).
- The wire is a chain of rigid links with springs; its stiffness is our estimate.
- Friction: housing outside 0.7 (textured), cavities and fingers 0.25, contact 0.15, wire 0.5.

## Known weak points

- The damage latch (a wedgelock forced over a contact held short of the click: 0) is implemented as before (blade
  position and load) but was not exercised in this revision's runs: in the `forced` variant the released contact
  springs back clear of the fingers, and the bench's lock cases blew up (see *Connector bench*).
- The wedgelock line-up is closed loop but slow to recover from a knock: in the `forced` run it stayed 3 mm off and
  the blades stopped at the channels' mouths.
- The click is sensitive: in our runs the robot's second push sometimes stalls just short of the click; the demo backs
  off and pushes again (up to three times), as an assembler would. Identical bench insertions occasionally jammed.
- Pushing the wire far behind the crimp is meant to bend it rather than drive the contact; in the bench, a hard push
  60 mm back made the simulated wire unstable rather than bend it cleanly.
- The scripted demo needs closed-loop corrections (the YAM's soft wrist) and is tuned to this layout: the two hands
  work tilted away from each other, and they swap after the tug (the hand that held the housing fits the lock). Runs from the start
  position repeat exactly.
- `ct_parts.install()` wraps three functions of `yam_arena.scene`; a yam_arena update that changes them stops the task
  with an error. It must run before `yam_arena.scene` is first imported (true for `-E task=task.py`).
- The scripted demo runs through `demo.py` (the same `YamArenaEmbodiment` and task-file scoring as `inspect-robots
  run`), not the CLI.
