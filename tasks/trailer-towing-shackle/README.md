# Shackle a trailer's towing lug and screw the pin fully home

A small two-wheeled utility trailer stands on the table, its drawbar ending in a steel block with a towing lug toward
the robot. A 1/2 in screw-pin anchor shackle (Crosby G-209, 2 t working load limit) stands upright in a slotted stand
left of the trailer; its red screw pin stands in a holder on the right. The robot must hang the shackle's two ears
around the lug, push the pin through the upper ear, the lug's hole and into the threaded lower ear, and screw the pin
in until its collar is seated on the upper ear.

**Instruction:** `Shackle the trailer's towing lug: the lug between the shackle's ears and the pin screwed fully home.`

## Why this task

Coupling a trailer, cart or skip to whatever will tow it is routine work for industrial truck and tractor operators
(O*NET 53-7051, about 800,000 US workers) and riggers (O*NET 49-9096). **How the shackle is used for towing:** the
shackle pinned through the drawbar's towing lug is the attachment point; the tow strap or chain from the towing
vehicle (a tractor, tug or forklift) is looped over the shackle's bow, and the pull goes through the bow, the pin and
the lug. The pin is what holds the trailer: it must be screwed fully home, collar seated on the ear, or it can work
loose and the trailer comes off.

The shackle is a loose part: two ears 20.6 mm apart must straddle a 15 mm lug and stay square while a 5/8 in pin goes
through three holes in a row with under 1 mm of play each. One hand has to hold the shackle in place while the other
brings the pin; then the pin has to be screwed in turn by turn, regripping it between half turns, with the hole it
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

**How the pin is secured:** a screw pin is screwed in hand-tight until its collar sits on the ear ("fully seated");
rigging practice says a shackle whose pin cannot be fully seated must not be used. That seated collar is what the
task scores. (Mousing the pin with wire, used for long-term or vibrating service, is outside this task.)

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

- **The thread:** 5/8-11 UNC, 2.31 mm per turn. From the moment the thread catches to the collar seated on the upper
  ear the pin turns 2.5 times clockwise and goes down 5.8 mm. In the simulation the engaged thread is shortened to
  those 2.5 turns (the bottom 5.8 mm of the lower ear; above it a plain 17.5 mm bore) so that the screwing fits the
  1-minute run; on the real shackle the thread runs through the whole 12.7 mm ear (about 5.5 turns). The pitch, the
  direction, the seating condition and the regrips are the real ones.
- **Why the shackle starts upright:** the gripper's fingers (26 mm wide) do not fit inside the shackle's 20.6 mm
  opening, and turning a flat shackle upright would need a horizontal grip the left arm cannot reach. The stand
  holds it the way it hangs on a rigging rack.
- **Why the lug is 7 cm up:** holding the pin straight down above a higher lug is out of the right arm's reach.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

| Condition | Score |
|---|---|
| The shackle or the pin at any point fell off the table, or the pin is not through the upper ear, the lug and into the lower ear | 0 |
| Pin through all three and **fully seated**: its collar within 1 mm of the upper ear (success, held 1 s) | 1.0 |
| Pin through all three, threaded at least one full turn, not seated (collar more than 1 mm above the ear) | 0.5 |
| Pin through all three, not threaded | 0.25 |

- Full credit needs the collar seated: turning alone gives no credit beyond 0.5, because the collar only comes down
  as far as the thread has been turned.
- "Through" means the pin's axis passes through the lug's hole at the lug's mid-plane, tilted under 15 degrees, its
  end below the lug and its collar above it; and it lies on the shackle's axis within 3 mm, tilted under 10 degrees,
  its end into the lower ear.
- In a real trial: a 1 mm feeler gauge (or a 1 mm shim) between the collar and the upper ear: if it does not fit,
  the pin is seated. Threaded at least one turn: the pin does not lift out when pulled up by hand after the trial.
  The video shows anything falling off.

Unintended strategies and what stops them:
- Putting the pin through the shackle without the lug between the ears: no credit (the pin must pass through the
  lug's hole).
- Pushing the pin through the lug alone, or lying the parts on the trailer: no credit.
- Pressing the pin down instead of screwing it: the collar stays 5.8 mm up until the thread is turned home (0.25).
- Turning the pin before it is through all three: the thread only catches with the pin's end resting centred on the
  lower ear.
- Throwing or dropping the parts off the table: 0.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Shackle the trailer's towing lug: the lug between the shackle's ears and the pin screwed fully home." \
  --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/trailer-towing-shackle-demo
```

1. **Left arm** grips the shackle's bow, leaning 35 degrees forward, lifts it out of its stand, carries it in front of
   the lug, lines the ears up with the lug and slides them over it, and holds it there.
2. **Right arm** lifts the pin out of its holder by the eye, brings it over the lug and lowers it through the upper ear
   and the lug onto the lower ear, where the thread catches. It lets go and rises.
3. **Left arm** lets go of the shackle and moves away.
4. **Right arm** screws the pin home: it pinches the eye, turns it half a turn clockwise while following the pin down
   (1.15 mm per half turn), lets go, rises, turns its hand back and pinches again, five times; the fifth half turn goes
   35 degrees further because the fingers slip a little on the eye. The collar seats on the upper ear.

`yam_scripted` plays fixed waypoints, so the demo is open loop. It uses offsets measured in simulation for each arm's
sag under load and for the shackle's 2.35 degree tilt in the left grip.

`SHACKLE_DEBUG=1` prints the shackle's and pin's poses, the thread's state and the turn every 0.2 s. It is off by
default.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: trailer, lug, shackle, pin and stands, the thread emulation, `score()` / `success()`, and `demo()` |
| `jointkit.py` | Rigid parts made of boxes, joints between parts that can be switched on and off and driven at run time, and per-physics-step and reset hooks (hooks yam_arena without modifying it) |
| `test_trailer_towing_shackle.py` | The task file loads, the real thread (pitch, turns to seat), the round lug hole, enough half turns in the demo, the demo's reach |
| `task.json` | Submission details |

## Runtime and seeds

A demo run (Isaac Sim start, reset, 1,048 control steps) takes about 7 minutes of wall time on an RTX workstation.
Run limit: 1,200 steps (1 minute at 20 Hz). The layout is fixed, so always pass `-E object_jitter=0`. There is no
other randomness.

## Tests

```bash
python -m pytest -q      # in this folder, in the yam_arena environment; no GPU needed
```

## Verified in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1), with the command above.

| Run | `task_score` | What the log and video show |
|---|---|---|
| Scripted demo | **1.0** | The shackle goes on the lug, the pin drops through onto the lower ear and the thread catches; five regripped half turns screw it down until the collar is seated. Success at step 1,048 (52.4 s) |
| Dream-1 | **0** | 1,200 steps (step limit). Both arms draw back toward their bases in the first seconds and stay there; neither picks up the shackle or the pin |
| `python -m pytest -q` | 7 passed | |

## Real-world build

- **Trailer:** any small two-wheeled utility or garden trailer, or a mock-up of its drawbar, braked or chocked so it
  cannot roll. The lug is a 15 mm steel plate welded to the drawbar's end with a 19 mm round hole.
- **Shackle:** Crosby G-209 1/2 in screw pin anchor shackle (stock no. 1018455) with its own screw pin, as bought.
  Its thread runs through the whole 12.7 mm lower ear: about 5.5 turns from the first catch to seated (the
  simulation shortens this to 2.5; see "Still approximate").
- **Stand and holder:** a pine stand with a slot that holds the shackle upright by its lower ear, and a pine block with
  a 17 mm hole for the pin.
- **Safety:** the parts are light and blunt; nothing is under load during the trial.

## Still approximate or unverified

- The thread is emulated (a joint that turns and slides, with friction), not modelled as a helix.
- The engaged thread is shortened from the real ear's ~5.5 turns (12.7 mm / 2.31 mm, not stated on the data sheet)
  to 2.5 turns so the screwing fits the 1-minute run; pitch, direction and the seated-collar condition are the real
  ones. With the full thread the demo would need about 6 more half turns (about 38 s at its pace).
- All parts are rigid; round parts are 16-sided prisms, the bow has a square section, and the wheels are drawn as
  stacked slabs.
- The demo relies on offsets measured in simulation; with jitter it would miss.
- One Dream-1 run on this version (1,200 steps) scored 0: neither arm picked up the shackle or the pin.
