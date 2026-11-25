# Reconnect a pop-up stopper under a wall-hung basin: rod through the stopper, pivot nut on snug, stopper open

**Instruction** (`task.py`): "Push the pivot rod through the stopper's hole into the drain port, thread the pivot nut on
until snug, press the rod down so the stopper lifts open, then let go and move both hands clear."

## The hard part: starting the pivot nut square, under the basin

The pop-up's pivot rod is out of its drain: its ball rests at the mouth of the port at the drain's back, and the pivot
nut hangs on the rod behind the ball. One hand pushes the rod 10 mm in, so that its tip goes through the hole in the
stopper's shaft and the ball seats, and holds it there; the other slides the nut forward along the rod and sets it on
the port's thread start, square and on the axis: started crooked, the plastic thread does not catch. The nut goes on
one turn by hand until its washer is snug on the ball; then the rod's rear is
pressed down so the stopper lifts about 10 mm (Kohler: about 3/8 in), and the washer's grip on the ball holds it
there when let go, as a pop-up does before its clevis strap is hooked up.

## Scene

Positions in m from the midpoint between the arm bases (+x away from the arms, +y left, z up from the bench). One
fixed layout (`object_jitter=0`): the seed changes nothing. Both arms at ready.

| Item | Product | At |
|---|---|---|
| Basin | Duravit ME by Starck wall-hung washbasin 550 x 440 x 180 mm, bowl 520 x 290 mm (EU 233555AA00, US 2335550000), hung low: its underside 246 mm over the bench at the drain | front edge x 0.295, wall x 0.735 |
| Drain | 1-1/4 in pop-up body (31.8 mm bore, 54 mm flange); its pivot port faces back (+x), axis 155 mm over the bench; port thread 25 x 1.6 mm (estimate), first full crest marked | axis (0.455, 0.000) |
| Trap | Dearborn P9701 1-1/4 in plastic P-trap (J-bend 5 in, arm 7-1/2 in), J swung 20 deg left, arm cut to 17 cm | under the drain |
| Stopper | Danco 11041 (132 x 35.6 mm), closed; its 6th hole (the five above taped) facing the port | in the drain |
| Pivot rod | Danco 88532 ball rod (150 x 5.5 mm, 18.3 mm ball, 4.5 mm tip), 22 x 14 x 16 mm clevis connector 68 mm behind the ball; ball 10 mm out of its seat | ball at (0.495, 0.000) |
| Pivot nut | plastic, 31 mm across its knurl, 16 mm long; its washer (4 rubber beads) faces the ball | on the rod behind the ball |

Pictures, each with a 320 x 176 twin as the policy sees it: `start_top.png` (`start_top_policy.png`),
`start_left_wrist.png` (`start_left_wrist_policy.png`), `start_right_wrist.png` (`start_right_wrist_policy.png`),
`under_basin_left_wrist.png` (`under_basin_left_wrist_policy.png`: the left wrist as the nut is set on the thread
start), `end_top.png` (`end_top_policy.png`). As in a real repair, the top camera sees the basin and the hands work by
the wrist views under it.

## Scoring (`score()` in `task.py`; the same numbers in `task.json`)

**Threaded**: the nut counts as threaded only if its thread caught at a square start: its front face set on the port's
first thread (at most 0.4 mm onto the washer), coaxial within 0.5 mm and square within 1.5 deg; a nut started crooked
does not catch. Turned back 3 deg past the start it comes off; it can start again once 3 mm off the port.

Whole-trial 0: the rod's ball or the nut below 30 mm over the bench (fallen). At the end:

- **1.0**: the rod in the port through the stopper's hole (ball within 3 mm of its seat, tip 3 mm or more past the
  stopper's middle), the nut threaded on snug (within 20 deg of where its thread stops), the stopper lifted 8 to 22 mm
  over the flange (within 4 mm of the drain's axis, within 8 deg of upright) for 1.0 s with at most 2 mm drift, both
  hands at least 30 mm clear of the stopper, rod, nut and drain, and the stopper, rod and nut still (under 25 mm/s).
- **0.5**: the rod in the port through the stopper's hole and the nut threaded on snug, short of 1.0.
- **0.25**: the rod in the port through the stopper's hole, the nut not snug.
- **0**: otherwise.

Judge: snug is the nut turned on by hand until it stops; a 150 mm rule on the stopper's lift; the trial video for a
fall and for 1 s of stillness after the hands leave.

## Mechanics in the simulation

| Stated force | Where (`task.py`, `scene.py`) | Acts on, reaction on | What ends it |
|---|---|---|---|
| Thread | `_step_thread()`: once the nut is set square on the start, a hidden carrier (a slide and three hinges on the port's axis) is welded to it where it stands; the slide follows the turn, 1.6 mm a turn; `set_joint` and `push` move only the carrier, never the nut | the nut; the drain | turned 3 deg back past the start (the weld is cut) |
| Thread running torque 0.03 N m | the carrier hinges' springs, capped at 0.03 N m, their rest following the turn | the nut; the drain | (none) |
| Thread's end | the hinges' stops, one turn (360 deg) from the start | the nut; the drain | (none) |
| Washer | 4 rubber beads on the nut (4.3 N in all at snug, friction 1.0) on the ball: contact only | the rod's ball; the nut, and through the ball the seat in the drain | the nut turned back |

The stopper is lifted only by the rod's tip under its hole's top bar, and held only by the washer's grip on the ball.

Peak contact forces in the reference run (each pair pushes both ways): the ball on its seat in the drain 33 N (a
late stroke pressing nut and ball on), the washer on the ball 6.2 N (peak while the strokes turn it; 4.3 N at rest), the nut's front on the ball's seat ring 2.4 N,
the tip under the stopper's hole bar 0.5 N, the stopper on the drain 1.8 N. Hands: pads pinching the nut 23.6 N
each, the stroking fingertip 23 N, pads on the clevis 14.8 N each: every finger under 40 N.

**Torque to snug (estimate)**: at snug the four beads press the ball with 4.3 N in all (trial `turned_on_snug`);
turning drags them round at 7.8 mm with friction 1.0: 4.3 x 1.0 x 0.0078 = 0.034 N m, plus 0.03 N m of thread: 0.064 N
m. The trial's rising torque turned the nut to its stop at 0.072 N m: lighter than a real nut
hand-tightened (the maker: hand-tighten, no tool).

**Why the stopper stays open**: the same contacts, 6.5 mm from the ball's pitch axis, take 4.3 x 1.0 x 0.0065 = 0.028
N m to turn the ball; the rod and stopper put 0.018 N m on it (the rod's rear 0.029 N m down, the stopper on the tip
0.011 N m up), so the rod stays where it was pressed. Trial `pressed_open_and_let_go`: pressed 23.5 deg and let go,
the stopper stood 11.85 mm open from 1 s to 30 s. Grip and friction are estimates.

## Key figures and their sources

`parts.py` holds all 54 figures with their sources and links (18 maker, 1 standard, 3 listing, 21 estimate, 11 ours).

| Figure | Value | Source |
|---|---|---|
| Basin; bowl | 550 x 440 x 180 mm; 520 x 290 mm | maker: https://www.duravit.com/de-de/p/ME-by-Starck-Waschtisch-233555AA00/ (US 2335550000) |
| P-trap: tube, J-bend, legs apart, arm | 1-1/4 in, 5 in, 3-1/2 in, 7-1/2 in | maker: Dearborn P9701 submittal drawing (Oatey) |
| Hand-tighten the pivot nut, no tool | | maker: Oatey lift rod drain sheet |
| Stopper lift when open | about 9.5 mm (3/8 in) | maker: Kohler 115999 sheet |
| Stopper: height, head, to 6th hole | 132.1, 35.6, 115.3 mm | listing: Danco 11041 (mrosupreme.com); EU Cornat TEC317414 |
| Rod: shaft, ball, tip | 150 x 5.5, 18.3, 4.5 mm | maker: Danco support answers for 88532 |
| Drain bore; flange | 31.8 mm (1-1/4 in); 54 mm | standard: ASME A112.18.2; maker: Delta RP5651 |
| Port thread; nut | 25 x 1.6 mm; 31 x 16 mm, 10 g | estimate: ball bore plus walls; 16 per inch |
| Washer beads | 1.1 N each at snug, friction 1.0 | estimate: holds 1.6 times the rod's weight |
| Thread running torque | 0.03 N m | estimate: half a dry steel 5/8-18 thread's |
| Catch: coaxial, square | 0.5 mm, 1.5 deg | estimate: 2/3 of a 0.75 mm flank; 60 % of atan(0.8/25) |

## Fidelity: what the simulation stands in for

- The thread is its last turn before snug (the nut's crests ride over the port's lead before it), carried by the
  hidden carrier; the catch, turn-back and stop are rules, the washer and every lift are contacts.
- The port's lead and its first crest (the marked land at the thread start) do not touch the nut: the kit filters a
  pair of bodies for the whole run only. A caught nut is held by the thread alone; one not yet caught stops where its
  washer meets the seated ball, which is where the thread starts.
- Under the fingertips' up to 23 N the carried nut gives up to 1.2 mm and 3.3 deg (the hidden chain); a real thread would not.
- The nut collides as 36 facets round the front 6.5 mm of its knurled skirt and 12 flats behind, where the pads
  pinch it. The fingertips turn it by stroking down its side, where a thumb would roll it.
- No water, lift rod or clevis strap.

## First minutes of a real trial (about 6 minutes)

- Port thread: thread gauge and calipers (est. 1.6 mm pitch, 25 mm). 1 min.
- Washer grip: hand-tighten the nut, hang a luggage scale on the clevis 68 mm behind the ball and pull down until the
  rod turns (est. 0.028 N m: 0.4 N). 2 min.
- Stopper: press the rod down until the stopper is up 10 mm (rule), let go, read it again after a minute. 2 min.
- Reset: nut off to the rod's rear, rod pulled back 10 mm, stopper closed with its hole to the port. 1 min.

A different reading replaces the estimate.

## Runs

- **Reference run** (`--check` sk19): 1.0 at 59.9 s of the 90 s limit (0.5 at 41.1 s). The thread caught at 13.25 s
  (0.29 mm off the axis, 0.19 deg); 20 strokes turned the nut to its stop; the rod pressed 22 deg, stopper 10.7 mm.
- **Trials** (tr3), all pass: start settles 1.4 mm; caught at the first step; 3 and 2 deg starts never catch; turned
  back it lets go; snug 0.5; let go at 23.5 deg, 11.85 mm from 1 s to 30 s, 1.0; both falls 0.

## Files and commands

`task.py` rules; `parts.py` figures; `scene.py` shapes; `demo.py` reference run; `checks.py` reach, clearances;
`trials.py` PhysX trials; `test_sink_stopper.py` rule tests.
