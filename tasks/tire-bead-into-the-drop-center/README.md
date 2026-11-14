# Hand-mount the last section of a kids' bike tire by pushing the bead into the rim's center channel, no lever under it

A 12-inch kids' bike wheel lies flat on a hub stand with its tire almost on: one bead (the tire's stiff inner edge) is
fully on, the other is on except the last 60 degrees over the valve, which sits up on the rim's flange (its raised edge).
"Finish putting this tire on the rim." means what anyone fixing a kid's bike does: push the bead that is already on down into the
rim's center channel (the drop center, a deeper groove round the middle of the rim), on the side away from the valve. That
shortens the bead's path round the rim and frees a little extra length, so the last section rolls over the flange by
hand, while one hand holds the bead beside it down in the channel (it pops back up as the section is lifted). Then set the
valve stem straight toward the hub and let go. The tempting wrong moves are pushing on the last section without making
slack first (it stops on the flange's edge) and pushing the tire lever from the table in under the bead, which can pinch the tube between
lever and rim.

## Spec

- Category: Societal relatability. Most people have ridden a bike, and fixing a flat on a child's bike is a familiar home repair, and the last section of a tight kids' tire is where people get stuck or reach for a lever and pinch the tube.
- Rule and sources: Park Tool, [Tire and Tube Removal and Installation](https://www.parktool.com/en-us/blog/repair-help/tire-and-tube-removal-and-installation):
  install the bead by hand where possible, since a lever pushed in past the bead can pinch the tube, and keep the
  valve straight toward the hub. Shop practice for tight tires: squeeze the bead that is on into the rim's center channel
  all round, starting opposite the valve, to gain the slack for the last section, and hold it down beside the last
  section while rolling that over, since it pops back up there.
- Mechanism: one `yamkit` articulation (the wheel). Five bead arcs slide down into the channel: the three away from
  the valve stay where pushed (`mechkit` two-detent `Detents`: on the shoulder or in the channel), the two beside the
  last section are pushed back up unless held (`Creep` in task.py, a 400 N/m force spring, 3.6 N at the channel floor).
  The last section is on a hinge with its own law (`LastSection` in task.py: a hard stop at the flange's edge until
  there is enough slack), and the valve stem turns about the vertical (`Free`). The lever is a loose prop.

| Quantity | Value | Why |
|---|---|---|
| Bead seat | radius 10.15 cm | ISO 203 (bead seat diameter 203 mm), the 12-inch kids' size |
| Flange top | 4.5 mm above the bead seat, z 9.55 | a hooked kids' rim |
| Channel (drop center) | 4 mm below the bead seat, 6 mm wide, in the rim's middle | a single-wall 203 rim |
| Bead arc travel | 9 mm down from its shoulder into the channel | half the rim's 17 mm inside width plus the bead toe |
| Arcs beside the last section | pushed back up onto the shoulder unless held | lifting the last section pulls the bead next to it up out of the channel |
| Slack per arc | 4.2 mm | a 60-degree arc moved 4 mm closer to the hub: 4 mm x 1.047 rad |
| Slack needed | 10 mm | lifting the last 60 degrees 4.5 mm onto the flange top takes 4.7 mm; its climb at each end about 2.5 mm more; rounded up |
| Slack counted | from each end of the last section round the wheel, while each arc is at least 6 mm down | bead still on its shoulder is held there by the tube's pressure, so length freed beyond it can't reach the last section; 6 mm is the channel's click zone (within 3 mm of its floor) |
| So | an arc beside the last section, held, and the next two (12.6 mm); two are not enough (8.4 mm) | the three far arcs alone give nothing until the arc beside is held down; in proportion between 6 and 9 mm down |
| Why it takes two hands | the law above, not a scoring rule: the arc beside the last section must be held down while the section rolls over, and their press spots are 9.7 cm apart edge to edge, more than one gripper's 9.4 cm full opening | one hand holds, the other rolls; any way of getting that (a lever pressing the arc, swapped hands) is allowed |
| Last section | starts tilted 20 degrees up about the tread; stops at 6 degrees short of seated without slack | 6 degrees is where its lip, 1.2 cm over the flange top at the start, comes down to the flange's edge |
| Bead on | the last section within 2.6 degrees of seated | there its lip passes below the flange top; within 2.5 degrees it is pulled in and stays |
| Valve straight | within 10 degrees of the line to the hub | 5.6 mm at the cap of the 3.2 cm stem, a lean you can see; it starts at 20 degrees (1.1 cm) |
| Strike | the last section's bead lip driven toward its seat faster than 12 cm/s over a control step (its 20-degree roll in about 0.13 s), counted up to 2.5 degrees short of seated, where it snaps in by itself | a bead struck down over the flange can trap the tube under it and pinch it; a careful roll by hand takes about half a second, about 3 cm/s at the lip, so the limit is nearly 4 times that and over 4 times the demo's fastest step (2.8 cm/s). Sim only. The sim has no tube, so a strike is a stand-in for a pinched tube and caps the score at 0.5 rather than zeroing it: speed alone does not prove damage. A real trial does not time the roll; it leak-tests the tube inside the inflated tire, and a leak scores 0 |
| Lever pinch | spoon tip more than 1 mm inside the bead seat line, down in the rim (radius 8.8 to 10.05 cm), at or below the bead's height there | that is the tube's space, where a lever pushed in past the bead can trap it against the rim; a tip that only hooks the bead on the flange or shoulder (careful lever use) or lies on top of it is not. Sim only: a real trial does not score where the lever went; the leak test decides |
| Let go | no gripper touching the tire, rim or valve: its own shape (palm and fingers at their present opening) more than 2 mm from them | 2 mm is the simulator's resting contact gap plus the fit of the boxes to the YAM's collision meshes; a hand hovering just over the tire is let go, one resting on it is not; a real trial checks the final photo |
| Slack started | at least two bead arcs at least 6 mm down in the channel at the end (fully down they free 8.4 mm of the 10 mm needed) | the step that makes the job possible, visible as bare rim shoulder above the bead |

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `yamkit.py` | Spawns the modeled wheel (an articulation) and the lever, and gives the scorer the grippers. It patches `yam_arena` in memory at import; no files change. |
| `mechkit.py` | Joint laws: the far bead arcs' two detents and the valve's free turn. |
| `guardkit.py` | Scoring guards: the pinch latch (with the lever tip's path between steps tested), wheel-frame geometry, and the contact check (the gripper's own shape touching the tire, rim or valve). |
| `test_task.py` | Offline checks of the slack law and of the scorer (every outcome below, careful and pinching lever use, near misses, a hand left on the tire, fast lever motion, reset). No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/tire-bead-into-the-drop-center/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Finish putting this tire on the rim." --scorer task_score --max-steps 1200

Model run (Dream-1): the same command with `--policy dream`. Render the videos of a run with
`inspect-robots video <log.json>`. Scorer tests: `python tasks/tire-bead-into-the-drop-center/test_task.py` (prints
`all pass`).

## Runtime and seeds

One trial takes up to 1,200 steps (60 seconds at 20 Hz); the scripted demo ends at full score in 506 steps (about
25 s). No seeds: the layout is fixed (`object_jitter=0`).

## The demo

1. Right arm, closed fingers pointing down: presses the tire's sidewall 1 cm outside the flange at 228, 270 and 312
   degrees, 9 mm down each time, pushing those three bead arcs into the channel, then pushes the arc beside the last
   section down at 170 degrees and holds it (16.8 mm of slack reach the last section).
2. Meanwhile the left arm lowers its closed fingers beside the valve stem, on its leaning side, and pushes it straight
   toward the hub, then backs off.
3. While the right hand holds, the left arm presses the last section's sidewall down along its arc until the bead drops
   inside the flange. Both hands lift off (the held arc springs back up onto its shoulder; the section stays in) and start
   back toward home; the run ends at 1.0 in 506 steps, once both hands have been clear for a second.

## Scoring

The same three end-state conditions are scored in the sim and in a real trial: the bead on, the valve straight, the
hands clear. Tube damage is scored differently, because the sim has no tube.

| Outcome | Sim | Real trial |
|---|---|---|
| Bead on, valve within 10 degrees of the line to the hub, no hand touching the tire, rim or valve (contact: the gripper's own shape, palm and fingers at their present opening, inside the simulator's 2 mm resting contact gap). A hand left on the tire can pull the bead back up, and the job ends with the wheel ready to inflate | 1.0 | 1.0 if the tube passes the leak test |
| Bead on, but the valve leaning more than 10 degrees (pulled sideways in the rim hole, it can tear off at its base) or a hand still touching the tire: the hard part is done, but the wheel is not ready to inflate | 0.5 | 0.5 if the tube passes the leak test |
| Bead not on, but at least two arcs at least 6 mm down in the channel: the step that makes the job possible is started (fully down, two arcs free 8.4 mm of the 10 mm needed), but the tire is not on | 0.25 | 0.25 if the tube passes the leak test |
| Anything else, including doing nothing or pressing the last section without slack so it stops at the flange's edge | 0 | 0 |
| The tube leaks in the leak test after the trial, whatever caused it | (no tube) | 0 |
| The lever's spoon tip in the tube's space at any moment (irreversible): it can trap the tube against the rim and puncture it | 0 (stand-in for a pinch) | not scored |
| The last section's bead lip driven toward its seat faster than 12 cm/s over a control step (irreversible) | capped at 0.5 (stand-in for a pinch) | not scored |

How the bead gets on is not scored (one hand or two, a lever used carefully, any order), and where the arms end is
not scored. Possible scores: 0, 0.25, 0.5, 1.0.

The two sim stand-ins exist only because the sim has no tube to test. In a real trial they would take credit away from
an intact, correctly mounted tire for how it was mounted, so they are dropped there and the leak test alone scores tube
damage. In the sim the lever's tip is checked at every control step and along its path since the last one, so a quick
poke under the bead between two camera frames still counts. There is no debounce: the lever lies on the table 8 cm
below and 11 cm outside the rim, so only a hand can put its tip there.

## Real-world build

- Wheel: any 12-inch (ISO 203) kids' bike wheel with a 12.5 x 2.25 (57-203) gumwall tire and a Schrader tube at 5 psi
  (0.34 bar, set with a gauge at every reset), on a hub stand (a post 1.8 cm across through the hub in place of its axle, bolted to a weighted plate clamped to the table) with the rim
  8.5 cm above the table. Mount the first bead, tuck the tube in, and work the second bead on to 60 degrees each side of
  the valve. Kids' tires on 203 rims are tight: whether a given tire needs exactly three arcs in the channel is
  checked once by hand (count how much of the bead has to be pushed into the channel before the last section rolls
  over at 5 psi, and that the bead beside the last section rises back out of the channel when let go). This is done
  once, when the rig is built: if that tire needs a different pressure to take about half the bead, that pressure is
  written on the rig and set with the gauge at every reset in place of 5 psi, so every trial starts the same.
- The slack law is the sim's stand-in for rubber stretch: it reproduces the real limit (the last section goes on only
  once enough bead sits in the channel), and a real trial checks the outcome, the bead on all round, instead.
- Slack in a real trial is the bead's own position: the bead visibly sits down in the channel, the rim's bare shoulder showing
  above it, wherever it has been pushed in; the sim's 4.2 mm per arc stands for that.
- Tire lever: a blue nylon lever (11.5 cm, Park Tool TL-1.2 or similar) on a tape outline on the table.
- Valve: a protractor card (marks at 0 and 10 degrees) taped to the rim bed beside the stem.
- Checking a trial: bead on = the tire's molding line evenly just above the flange all round with no tube showing under
  the bead; valve = the stem against the card (10 degree mark); slack started = bare rim shoulder showing above the bead
  where it sits in the channel; tube = the leak test below. The lever's position and the roll's speed are not scored in
  a real trial: the leak test is the real check behind both sim stand-ins for a pinched tube.
- Leak test, after every trial, with the tube always inside the mounted tire (an unsupported tube is never inflated,
  since it can balloon and burst well below the tire's pressure):
  1. If the bead is not on at the end (a 0 or 0.25 trial), the tester first rolls the last section on by hand, without
     a lever, so the whole tube is enclosed by the tire and rim. This does not change the trial's score.
  2. Inflate to 10 psi and check that the molding line sits evenly just above the flange on both sides all round, with
     no tube showing under either bead. If not, let the air out and re-seat the bead by hand before going on.
  3. Inflate to the pressure printed on the tire's sidewall (read off the rig's tire when it is built and written on
     the rig), never above it, with a gauge, and with hands and face clear of the bead line while it rises.
  4. Dunk the wheel in a tub of water, tire and valve under, turning it slowly (or brush soapy water along both beads
     and the valve). A pinched tube leaks into the tire, and the air comes out at the bead line and the valve hole.
     Then read the gauge again 5 minutes after inflating. Bubbles, or a drop of 2 psi or more, mean a leak: the trial
     scores 0 and the tube is replaced.
  5. Let the tire down before the reset.
- Safety: the arms press 1 cm into a soft tire on a clamped stand; no sharp edges, nothing under pressure beyond 5 psi
  during the trial. The leak test is done by the tester after the arms have stopped, only on a fully mounted tire, and
  never above the sidewall's rated pressure.
