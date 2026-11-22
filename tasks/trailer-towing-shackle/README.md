# Shackle a trailer's towing lug and screw the pin fully home

A small two-wheeled utility trailer stands on the table, its drawbar ending in a steel block with a towing lug toward
the robot. A 1/2 in screw-pin anchor shackle (Crosby G-209, 2 t working load limit) stands upright in a slotted stand
left of the trailer; its red screw pin stands in a holder on the right. The robot must hang the shackle's two ears
around the lug, push the pin through the upper ear, the lug's hole and into the threaded lower ear, and screw the pin
in until its collar sits on the upper ear and the pin is hand-tight.

**Instruction:** `Shackle the trailer's towing lug: the lug between the shackle's ears and the pin screwed fully home.`

## Why this task

Rigging a load (attaching a sling, chain or cable to it so it can be lifted or pulled) is daily work for about
900,000 US workers. O*NET (release 30.0) lists it among the tasks of these occupations, and its incumbent ratings give
the share who do it daily or more often (Task Ratings file, frequency scale, "daily", "several times daily" and
"hourly or more"); employment is O*NET's 2025 figure for each occupation:

| Occupation (O*NET-SOC) | O*NET task (ID) | Daily or more | US workers | Rigging daily |
|---|---|---|---|---|
| Laborers and freight, stock and material movers, hand (53-7062) | Attach slings, hooks, or other devices to lift cargo and guide loads. (10790) | 23.8% | 2,940,300 | 698,300 |
| Helpers, production workers (51-9198) | Attach slings, ropes, or cables to objects such as pipes, hoses, or bundles. (12726) | 63.6% | 167,400 | 106,500 |
| Structural iron and steel workers (47-2221) | Fasten structural steel members to hoist cables, using chains, cables, or rope. (11553) | 71.8% | 68,300 | 49,100 |
| Sailors and marine oilers (53-5011) | Break out, rig, and stow cargo-handling gear, stationary rigging, or running gear. (14468) | 73.8% | 33,000 | 24,300 |
| Riggers (49-9096) | Attach loads to rigging to provide support or prepare them for moving, using hand and power tools. (13894) | 87.4% | 22,500 | 19,700 |
| **Total** | | | | **897,900** |

The screw-pin shackle is the standard link in that work: the sling, chain or cable is attached to a lug, padeye or
eye by pinning a shackle through it and screwing the pin home, and the pin holds only if its collar is on the ear and
it is hand-tight. O*NET's Tools Used file (commodity 24101641 "Shackle") also lists shackles by name for crane and
tower operators, boilermakers, glaziers, roustabouts, HVAC installers, wind turbine technicians and roofers' helpers.
The same operation couples tow straps and chains to towing lugs in vehicle recovery and farming, the setting used here.
Sources: https://www.onetcenter.org/database.html (release 30.0: Task Statements, Task Ratings, Tools Used) and the
employment figures on each occupation's O*NET OnLine page (https://www.onetonline.org/link/summary/<code>).

**How the shackle is used for towing:** the
shackle pinned through the drawbar's towing lug is the attachment point; the tow strap or chain from the towing
vehicle (a tractor, tug or forklift) is looped over the shackle's bow, and the pull goes through the bow, the pin and
the lug. The pin is what holds the trailer: it must be screwed fully home, collar on the ear and hand-tight, or it can
work loose and the trailer comes off.

The shackle is a loose part: two ears 20.6 mm apart must straddle a 15 mm lug and stay square while a 5/8 in pin goes
through three holes in a row with under 1 mm of play each. One hand has to hold the shackle in place while the other
brings the pin; then the pin has to be screwed in five and a half turns, regripping it between half turns, with the hole it
threads into held in place only by the shackle hanging on the lug. It is not a plain peg-in-hole: the hole the pin
goes into is formed only while the shackle is held around the lug, and it moves if the shackle is let go too early
or knocked.

## Hardware

The shackle and pin are a standard, rated part: **Crosby G-209 screw pin anchor shackle, 1/2 in, stock no. 1018455**
(galvanised; WLL 2 t; Crosby G-209 data sheet). Dimensions from the data sheet, as built in the simulation:

| Data-sheet dimension | Value | Simulation |
|---|---|---|
| A, width between the ears | 0.81 in (20.6 mm) | 20.6 mm |
| B, pin diameter | 0.63 in (16 mm); thread 5/8-11 UNC | 15.8 mm pin, thread pitch 2.31 mm (5/8-11) |
| C, inside length | 1.88 in (47.8 mm) | 47.8 mm |
| D, bow diameter | 0.50 in (12.7 mm) | 12.7 mm (square section) |
| F, ear outside diameter | 1.19 in (30.2 mm) | 30.2 mm |
| Weight with pin | 0.72 lb (0.33 kg) | shackle 0.25 kg + pin 0.11 kg |

**How the pin is secured:** a screw pin is screwed in until its collar sits on the ear ("fully seated") and then
tightened by hand against it (hand-tight); rigging practice says a shackle whose pin cannot be fully seated must not
be used. A collar on the ear, tightened, is what the task scores. (Mousing the pin with wire, used for long-term or vibrating service, is outside this task.)

**The pin goes down as it is screwed in:** the thread pulls the pin 2.31 mm further into the lower ear with every
clockwise turn, so the collar comes down onto the upper ear as the pin is turned. The simulation does the same (see
"The thread" below): the pin's height follows its turn; it cannot be pushed down without turning it, and turning it
is what brings the collar down.

## Design

| Object | Size | Mass | Notes |
|---|---|---|---|
| Trailer | Deck 42 x 34 cm, 2 cm thick, underside 10 cm up, 5 cm rails on both sides and the back; one axle, two 18 cm wheels 4 cm wide | fixed | Blue painted steel deck, rubber wheels. From x 40 to 82 cm, centred at y -8 cm |
| Drawbar block | 8 x 8 cm, 7.75 cm tall, x 32 to 40, y -12 to -4 cm | fixed | Painted steel; a riser joins it to the deck |
| Towing lug | 15 mm steel plate, 40 mm wide, projecting 5 cm toward the arms, top flush with the block's top (centre 7 cm up); 19 mm round hole centred at x 29, y -8 cm | fixed | Welded to the block |
| Shackle | Crosby G-209 1/2 in (table above): 12.7 mm bow, round ears 30.2 mm across and 12.7 mm thick, 20.6 mm apart, 47.8 mm inside length | 0.25 kg | Galvanised steel. Upper ear: plain 17.5 mm hole. Lower ear: 5/8-11 thread. Stands upright, threaded ear down, bow toward the arms, at x 27, y 7 cm in a slotted pine stand (10 mm base) |
| Pin | Its screw pin: 5/8 in x 46 mm under a 25 mm collar; flat eye 26 x 8 mm on top; 1.5 mm x 45 degree chamfer at its end | 0.11 kg | Steel, red (Crosby's red pin). Stands eye up in a hole through a pine block 4 x 4 x 3 cm at x 22, y -23 cm |

Positions are from the midpoint between the two arm bases (x forward, y left, z up).

- **The thread:** 5/8-11 UNC, 2.31 mm per turn, through the whole 12.7 mm lower ear, as on the real shackle. From
  the moment the thread catches (the pin's end resting on the lower ear) to the collar on the upper ear, the pin turns
  5.5 times clockwise and goes down 12.7 mm.
- **The seat and hand-tight:** once the collar is on the ear, the ear is a hard stop: turning the pin further only
  tightens it against the ear (the thread is loaded), and the pin counts as hand-tight once it has been turned
  against the seat with at least 0.3 N m. The YAM gripper reaches about 1 N m on the pin's eye before its fingers
  slip (measured in the demo). Turned back by an eighth of a turn, the pin is no longer tight.
- **Why the shackle starts upright:** the gripper's fingers (26 mm wide) do not fit inside the shackle's 20.6 mm
  opening, and turning a flat shackle upright would need a horizontal grip the left arm cannot reach. The stand
  holds it the way it hangs on a rigging rack.
- **Why the lug is 7 cm up:** holding the pin straight down above a higher lug is out of the right arm's reach.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

| Condition | Score |
|---|---|
| The shackle or the pin at any point fell off the table, or the pin is not through the upper ear, the lug and into the lower ear | 0 |
| Pin through all three, **collar on the upper ear and hand-tight** (success, held 1 s) | 1.0 |
| Pin through all three, threaded at least one full turn, but not on the ear or not tight | 0.5 |
| Pin through all three, not threaded | 0.25 |

- Full credit needs the collar on the ear and tightened: turning alone gives no credit beyond 0.5, because the
  collar only comes down as far as the thread has been turned, and a collar resting on the ear without being
  tightened against it is not hand-tight. In the simulation "on the ear" is within 0.3 mm (the physics solver's
  give), and "tight" is the 0.3 N m against the seat described above.
- "Through" means the pin's axis passes through the lug's hole at the lug's mid-plane, tilted under 15 degrees, its
  end below the lug and its collar above it; and it lies on the shackle's axis within 3 mm, tilted under 10 degrees,
  its end into the lower ear.
- In a real trial: **on the ear:** no gap between the collar and the upper ear (a 0.05 mm feeler blade does not go
  in anywhere around the collar); **hand-tight:** the operator tries to turn the pin further clockwise by its eye
  with the fingers and it does not turn. Threaded at least one turn: the pin does not lift out when pulled up by
  hand after the trial.
  The video shows anything falling off.

Unintended strategies and what stops them:
- Putting the pin through the shackle without the lug between the ears: no credit (the pin must pass through the
  lug's hole).
- Pushing the pin through the lug alone, or lying the parts on the trailer: no credit.
- Pressing the pin down instead of screwing it: the collar stays 12.7 mm up until the thread is turned home (0.25).
- Screwing the pin down until the collar just touches, without tightening it: 0.5.
- Turning the pin before it is through all three: the thread only catches with the pin's end resting centred on the
  lower ear.
- Throwing or dropping the parts off the table: 0.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Shackle the trailer's towing lug: the lug between the shackle's ears and the pin screwed fully home." \
  --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/trailer-towing-shackle-demo
```

1. **Left arm** grips the shackle's bow, leaning 35 degrees forward, lifts it out of its stand, carries it in front of
   the lug, lines the ears up with the lug and slides them over it, and holds it there.
2. **Right arm** lifts the pin out of its holder by the eye, brings it over the lug and lowers it through the upper ear
   and the lug onto the lower ear, where the thread catches. It lets go and rises.
3. **Left arm** lets go of the shackle and moves away.
4. **Right arm** screws the pin home: it pinches the eye, turns it half a turn clockwise while following the pin down
   (1.15 mm per half turn), lets go, rises, turns its hand back and pinches again, eleven times; the eleventh half
   turn goes 35 degrees further, which brings the collar onto the ear and tightens it against the ear (the fingers
   slip on the eye at about 1 N m). A twelfth half turn follows only if the pin is not tight yet; in the run below the
   trial ended on success before it.

`yam_scripted` plays fixed waypoints, so the demo is open loop. It uses offsets measured in simulation for each arm's
sag under load and for the shackle's 2.35 degree tilt in the left grip.

`SHACKLE_DEBUG=1` prints the shackle's and pin's poses, the thread's state and the turn every 0.2 s. It is off by
default.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: trailer, lug, shackle, pin and stands, the thread emulation, `score()` / `success()`, and `demo()` |
| `jointkit.py` | Rigid parts made of boxes, joints between parts that can be switched on and off and driven at run time, and per-physics-step and reset hooks (hooks yam_arena without modifying it) |
| `test_trailer_towing_shackle.py` | The task file loads, the real thread (pitch, 5.5 turns to seat), hand-tight needs torque on the seat, the round lug hole, enough half turns in the demo, the demo's reach |
| `task.json` | Submission details |

## Runtime and seeds

A demo run (Isaac Sim start, reset, 1,920 control steps) takes about 9 minutes of wall time on an RTX workstation.
Run limit: 2,400 steps (2 minutes at 20 Hz): the 5.5 turns take 11 regripped half turns. The layout is fixed, so always pass `-E object_jitter=0`. There is no
other randomness.

## Tests

```bash
python -m pytest -q      # in this folder, in the yam_arena environment; no GPU needed
```

## Verified in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1), with the command above.

| Run | `task_score` | What the log and video show |
|---|---|---|
| Scripted demo | **1.0** | The shackle goes on the lug, the pin drops through onto the lower ear and the thread catches; eleven regripped half turns screw it down 12.7 mm until the collar is on the ear, and the last one tightens it (1.0 N m against the seat). Success at step 1,920 (96.0 s) |
| Dream-1 | **0** | 2,400 steps (step limit). The left arm draws back toward its base; the right hand moves down near the pin holder and stays there; neither the shackle nor the pin reaches the lug |
| `python -m pytest -q` | 8 passed | |

## Real-world build

- **Trailer:** any small two-wheeled utility or garden trailer, or a mock-up of its drawbar, braked or chocked so it
  cannot roll. The lug is a 15 mm steel plate welded to the drawbar's end with a 19 mm round hole.
- **Shackle:** Crosby G-209 1/2 in screw pin anchor shackle (stock no. 1018455) with its own screw pin, as bought.
  Its thread runs through the whole 12.7 mm lower ear: 5.5 turns from the first catch to seated, as simulated.
- **Stand and holder:** a pine stand with a slot that holds the shackle upright by its lower ear, and a pine block with
  a 17 mm hole for the pin.
- **Safety:** the parts are light and blunt; nothing is under load during the trial.

## Still approximate or unverified

- The thread is emulated (a joint that turns and slides, with friction), not modelled as a helix.
- Turns to seat: 12.7 mm ear thickness / 2.31 mm pitch = 5.5 turns. The data sheet gives the ear's outside
  diameter but not its thickness; 12.7 mm (the bow diameter) is our figure, to check on a real shackle.
- Hand-tight is modelled as 0.3 N m against the seat, a threshold we chose (we found no published hand-tight torque
  for shackle pins); a person's fingers give more. The real-trial check is the operator's own fingers (above).
- All parts are rigid; round parts are 16-sided prisms, the bow has a square section, and the wheels are drawn as
  stacked slabs.
- The demo relies on offsets measured in simulation; with jitter it would miss.
- One Dream-1 run on this version (2,400 steps) scored 0: neither the shackle nor the pin reached the lug.
