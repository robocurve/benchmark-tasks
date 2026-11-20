# Carry a wafer carrier into a rinse tank by its top bar

**Instruction:** "Put the wafer carrier into the rinse tank using its handle, then take the handle off the carrier and stand it on the bench deck."

On a polypropylene wet bench, a 125 mm PFA wafer carrier holding 25 wafers stands on the deck in front of a recessed
quick-dump-rinse tank, with its squeeze handle on it. The handle is a springy PFA inverted U whose feet hook into a
notch in each end wall of the carrier. Under its top bar is a curved bow: squeezing the bow up toward the bar spreads
the legs and frees the feet. That is how the handle goes on and comes off, and it is also why a hand that closes
over bar and bow together lets the carrier go. The carrier must be lifted and carried by the bar alone, lowered into
the tank onto its two rests without tipping or dropping (the carrier is open-topped, so wafers walk out of their slots
and break), and only then freed by a squeeze.

Sources: MIT MTL RCA clean SOP (https://mtlsites.mit.edu/fabrication/sops/rca_icl.pdf): "Using the top captured
handle, place the wafer carrier into SC1"; "slowly pull the carrier from HF". Entegris Specialized Handling Products
catalog 5367 (https://www.megatech.com/files/ww/catalog-specialized-handling-products-5367.pdf): 182 Series 125 mm
carrier, 25 slots at 4.76 mm, 143 x 152 x 146 mm; A72-50-03 squeeze handle, 163 x 140 x 183 mm, "attaches to the
notches on both end walls of carrier"; the catalog also lists a squeeze handle with a lock bar (A040-101).

## Run and score a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key configured, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lower_a_wafer_carrier_into_the_rinse_tank_by_its_squeeze_handle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the wafer carrier into the rinse tank using its handle, then take the handle off the carrier and stand it on the bench deck." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/carrier-dream

# The scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lower_a_wafer_carrier_into_the_rinse_tank_by_its_squeeze_handle.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Put the wafer carrier into the rinse tank using its handle, then take the handle off the carrier and stand it on the bench deck." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/carrier-demo
```

The run prints `task_score`; it is scored on the final state (or stops once 1.0 has held).

- One fixed layout (`object_jitter=0`); the seed changes nothing. Runtime per trial: up to 2,400 steps (120 s of sim
  time at 20 Hz); the scripted demo reaches full score in about 42 s (840 steps), and the run then stops.
- Demo modes (`KIT_DEMO`): `demo` (default); `squeeze` (the natural grab over bar and bow, then lift and carry);
  `drop` (carried over the tank and let go there, the hand snatched straight back); `sag` (opened over the tank and
  pulled back slowly); `held` (carried over the tank and held); `seat` (seated with the handle left on, let go); `none`
  (arms stay home).
- Tests: `PYTHONPATH=<yam-isaac-lab-arena> PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q` in this folder.

## Runs

In Isaac (PhysX), this task file, the commands above:

- Scripted demo: 1.0 in 43 s. Lifted on the handle with both tines engaged at 11 s, over the tank at 15 s, seated on
  both rests at 21 s (tilt at most 5.2 deg on the way), the bow squeezed to its stop and both tines out of the notches
  by 30 s (the tines leave the notches at 29.0-29.5 s with the carrier on both rests, so the tine rule does not
  fire), the handle stood on its feet on the deck by 39 s; then released and still.
- Dream-1: 0.0. Within 16 s it pulls the carrier, still on its handle, off the deck edge so it lands on its side
  (tilt 89 deg, a latched zero), then works at the handle for the rest of the 2,400 steps.

## Scoring (the scorer is `grade()` in the task file)

| Score | Condition |
|---|---|
| 0 (latched) | at any time: once lifted, a tine leaving its notch while the carrier is not sitting on both rests (the tines may come out only there, to take the handle off); the carrier falls (within 0.25 s its centre goes down 20 mm further than either hand went), tips more than 15 deg, a gripper comes within 3 mm of it, or the bench moves more than 5 mm |
| 0.25 | the carrier lifted with every bottom corner at least 20 mm above the deck, both tines in their notches |
| 0.5 | then carried over the tank: its centre within 20 mm of the tank's centre in x and y |
| 0.75 | at the end, seated on both rests: bottom rails within 3 mm above the rests at all four corners, upright within 5 deg |
| 1.0 | also the handle at least 10 mm from the carrier and standing on its feet on the deck (feet within 3 mm, within 10 deg of upright), no gripper within 30 mm of either, both still (2 mm) for the last 2 s |
| 0 | anything else, including the untouched start and a carrier never lifted on its handle |

**Measuring it in a real trial.** Every condition's camera or instrument and its number are in task.json's `scoring`
field: a fixed side camera level with the rests' tops through the tank's clear -y wall, a tape, a steel rule, a phone
level and a 3 mm tick on each rest.

## Layout

One fixed layout, every object with its type, size, material, mass and pose: task.json's `setup` field (the
screenshot shows it).

## Parts (real bench)

- Entegris A182-50M-0215 125 mm wafer carrier and A72-50-03-0215 squeeze handle (Entegris or distributors; catalog
  5367 above). 25 test-grade 125 mm silicon wafers, plus 25 spares.
- A drained polypropylene rinse tank (any 200 x 200 mm inside, 180 mm deep PP tank) set into a 175 mm deck, with two PFA
  or PTFE bar rests; or a mock-up of the same in PP sheet, bolted to the table.

## Simulation and the real case

- Rigid bodies. The wafers are fixed in the carrier (drawn, mass included), so the 15 deg tip rule stands in for
  wafers walking out of their slots.
- The handle's PFA flexure is modelled as sliders: the legs are sprung closed at 2000 N/m and the bow sprung down at
  2000 N/m (20 N at its 10 mm stop). One code rule (`Couple`) sets the legs' spring target 1.2 mm per mm of bow
  travel, standing in for what the bent bow does to the legs; the tines leave the 6 mm notches at 5 mm of bow travel.
- The tines hold the carrier by contact in the notches; nothing is welded. The carrier moves only through the arms.
- The bolted bench is a 40 kg free body; moving it more than 5 mm scores 0. The tank is drained and no water is
  modelled: a filled bath would buoy the carrier and wafers about 3 N (their 330 cm3 of PFA and silicon) and damp the
  descent, so the drained tank, with the full 9 N on the handle to the rests, is the harder case.
- Friction 0.4 on every part, static = dynamic. The carrier's and handle's masses are estimates, to weigh.

## The scripted demo

The right hand pinches the top bar alone (lower finger in the 33 mm gap above the bow, about 15 N), lifts, carries
the carrier over the tank and lowers it onto the rests, each loaded move servoed on the carrier's live pose to lead out
the arm's sag. Then it opens, comes in across bar and bow, squeezes the bow to its stop (about 28 N a finger), lifts
the handle off, stands it on the deck and goes home.

| Mode | Expected | physcheck (MuJoCo) |
|---|---|---|
| demo | 1.0 | 1.0 at 41.9 s |
| squeeze (natural grab) | 0 | 0: the feet spread, the handle comes away alone |
| drop (let go, hand snatched back) | 0 | 0: it pivots on the leaving finger and tips past 15 deg; the fall rule is checked by test_9 |
| sag (opened, pulled back slowly) | 0.5 | 0.5: the arm sags with the load, so it is lowered with the hand (not a fall) and wedges unseated |
| held | 0.5 | 0.5 |
| seat | 0.75 | 0.75 |
| none | 0 | 0 |

## Safety

Run behind the cell's guard with the e-stop in reach. The tank is drained and no chemicals are used. Test-grade wafers
or aluminium discs of the same size; a broken wafer is swept up by a person with a cleanroom brush into a sharps bin. The grip is the gripper's own
squeeze (under 40 N).

## Licences

All geometry is built from boxes in the task file from the catalog's dimensions; no third-party meshes, scans or
scenes are used.
