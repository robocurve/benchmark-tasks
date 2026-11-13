# Shackle a trailer's towing lug and lock the pin with one twist

A small two-wheeled utility trailer stands on the table, its drawbar ending in a steel block with a towing lug toward
the robot. A 1/2 in D shackle stands upright in a slotted stand left of the trailer; its red quick-lock pin stands in
a holder on the right. The robot must hang the shackle's two ears around the lug, push the pin through the upper ear,
the lug's hole and into the lower ear, and lock it with one twist.

**Instruction:** `Shackle the trailer's towing lug: the lug between the shackle's ears and the pin screwed home to lock it.`

## Why this task

Coupling a trailer, cart or skip to whatever will tow or lift it is routine work for industrial truck and tractor
operators (O*NET 53-7051, about 800,000 US workers) and riggers (O*NET 49-9096). The shackle is a loose part: two ears
20.6 mm apart must straddle a 15 mm lug and stay square while a 15.8 mm pin goes through three holes in a row with
under 1 mm of play each. One hand has to hold the shackle in place while the other brings the pin, then the pin must
be locked: an unlocked pin can work out under load. It is not a plain peg-in-hole: the hole the pin goes into is
formed only while the shackle is held in place around the lug, and it moves if the shackle is let go too early or
knocked.

## Design

| Object | Size | Mass | Notes |
|---|---|---|---|
| Trailer | Deck 42 x 34 cm, 2 cm thick, underside 10 cm up, 5 cm rails on both sides and the back; one axle, two 18 cm wheels 4 cm wide | fixed | Blue painted steel deck, rubber wheels. From x 40 to 82 cm, centred at y -8 cm |
| Drawbar block | 8 x 8 cm, 7.75 cm tall, x 32 to 40, y -12 to -4 cm | fixed | Painted steel; a riser joins it to the deck |
| Towing lug | 15 mm steel plate, 40 mm wide, projecting 5 cm toward the arms, top flush with the block's top (centre 7 cm up); 19 mm square hole centred at x 29, y -8 cm | fixed | Part of the block |
| Shackle | 1/2 in D shackle: 12.7 mm bar, round ears 27 mm across and 12.7 mm thick, 20.6 mm apart, 47.8 mm inside length | 0.25 kg | Galvanised steel. Upper ear: plain 17.5 mm hole. Lower ear: the pin's thread under a 3.5 mm plain lead-in. Stands upright, threaded ear down, bow toward the arms, at x 27, y 7 cm in a slotted pine stand (10 mm base) |
| Pin | 15.8 mm x 46 mm under a 25 mm collar; flat eye 26 x 8 mm on top; 1.5 mm x 45 degree chamfer at its end | 0.11 kg | Steel, red. Quick-lock thread (see below). Stands eye up in a hole through a pine block 4 x 4 x 3 cm at x 22, y -23 cm |

- **Quick-lock pin:** the pin's thread has a steep lead, so half a turn takes it from the thread's first catch to the
  collar seated on the upper ear (9.2 mm of travel). Turned 120 degrees clockwise, it is locked.
- **Why the shackle starts upright:** the gripper's fingers (26 mm wide) do not fit inside the shackle's 20.6 mm
  opening, and turning a flat shackle upright would need a horizontal grip the left arm cannot reach. The stand
  holds it the way it hangs on a rigging rack.
- **Why the lug is 7 cm up:** holding the pin straight down above a higher lug is out of the right arm's reach.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

| Condition | Score |
|---|---|
| The shackle or the pin at any point fell off the table, or the pin is not through the upper ear, the lug and into the lower ear | 0 |
| Pin through all three and locked: turned at least 120 degrees clockwise on its thread, or its collar within 1 mm of the upper ear (success, held 1 s) | 1.0 |
| Pin through all three, thread caught, turned at least 45 degrees | 0.5 |
| Pin through all three, not turned | 0.25 |

- "Through" means the pin's axis passes through the lug's hole at the lug's mid-plane, tilted under 15 degrees, its
  end below the lug and its collar above it; and it lies on the shackle's axis within 3 mm, tilted under 10 degrees,
  its end into the lower ear.
- In a real trial: a line on the eye and a matching mark on the upper ear show the turn (120 degrees is a third of a
  turn); a look at the pin shows whether it passes through all three; the video shows anything falling off.

Unintended strategies and what stops them:
- Putting the pin through the shackle without the lug between the ears: no credit (the pin must pass through the
  lug's hole).
- Pushing the pin through the lug alone, or lying the parts on the trailer: no credit.
- Turning the pin before it is through all three: the thread only catches with the pin's end resting centred on the
  lower ear.
- Throwing or dropping the parts off the table: 0.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Shackle the trailer's towing lug: the lug between the shackle's ears and the pin screwed home to lock it." \
  --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/trailer-towing-shackle-demo
```

1. **Left arm** grips the shackle's bow, leaning 35 degrees forward, lifts it out of its stand, carries it in front of
   the lug, lines the ears up with the lug and slides them over it, and holds it there.
2. **Right arm** lifts the pin out of its holder by the eye, brings it over the lug and lowers it through the upper ear
   and the lug onto the lower ear, where the thread catches. It lets go and rises.
3. **Left arm** lets go of the shackle and moves away.
4. **Right arm** grips the eye again and gives it one clockwise twist (half a turn plus 35 degrees, the wrist's limit;
   the fingers slip a little on the eye). The pin locks.

`yam_scripted` plays fixed waypoints, so the demo is open loop. It uses offsets measured in simulation for each arm's
sag under load and for the shackle's 2.35 degree tilt in the left grip.

`SHACKLE_DEBUG=1` prints the shackle's and pin's poses, the thread's state and the turn every 0.2 s. It is off by
default.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: trailer, lug, shackle, pin and stands, the thread emulation, `score()` / `success()`, and `demo()` |
| `jointkit.py` | Rigid parts made of boxes, joints between parts that can be switched on and off and driven at run time, and per-physics-step and reset hooks (hooks yam_arena without modifying it) |
| `test_trailer_towing_shackle.py` | The task file loads, the lock geometry, one twist in the demo, the demo's reach |
| `task.json` | Submission details |

## Runtime and seeds

A demo run (Isaac Sim start, reset, 550 control steps) takes about 4 minutes of wall time on an RTX workstation. Run
limit: 1,200 steps (1 minute at 20 Hz). The layout is fixed, so always pass `-E object_jitter=0`. There is no other
randomness.

## Tests

```bash
python -m pytest -q      # in this folder, in the yam_arena environment; no GPU needed
```

## Verified in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1), with the command above.

| Run | `task_score` | What the log and video show |
|---|---|---|
| Scripted demo | **1.0** | The shackle goes on the lug, the pin drops through onto the lower ear and the thread catches; one twist locks it. Success at step 550 (27.5 s) |
| Dream-1 | **0** | 1,200 steps (step limit). Both arms draw back toward their bases within the first seconds and stay there; neither touches the shackle or the pin, and nothing on the table moves |
| `python -m pytest -q` | 6 passed | |

## Real-world build

- **Trailer:** any small two-wheeled utility or garden trailer, or a mock-up of its drawbar, braked or chocked so it
  cannot roll. The lug is a 15 mm steel plate welded to the drawbar's end with a 19 mm hole.
- **Shackle:** a standard 1/2 in screw-pin D shackle. The pin is a quick-lock variant (steep thread: half a turn seats
  it); a standard 5/8-11 screw pin needs four turns instead.
- **Stand and holder:** a pine stand with a slot that holds the shackle upright by its lower ear, and a pine block with
  a 17 mm hole for the pin.
- **Safety:** the parts are light and blunt; nothing is under load during the trial.

## Still approximate or unverified

- The thread is emulated (a joint that turns and slides, with friction), not modelled as a helix.
- All parts are rigid; round parts are 16-sided prisms, and the wheels are drawn as stacked slabs.
- The demo relies on offsets measured in simulation; with jitter it would miss.
- One Dream-1 run (1,200 steps) scored 0: neither arm reached the shackle or the pin.
