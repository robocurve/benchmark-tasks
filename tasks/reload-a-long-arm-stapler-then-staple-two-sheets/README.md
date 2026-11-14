# Reload a long-arm stapler and staple two sheets together

An empty long-arm stapler lies on the table, its nose toward the middle, with a fresh strip of standard staples on a
tray beside it and two sheets of A6 paper on its paper rest. Its staple magazine is a front-loading drawer: a steel
channel with a rail along its floor, closed at the nose by a cap with a red pull tab, which slides out of the nose
against a return spring and is pulled shut again when let go. Inside the housing a staple pusher (the follower) on a
long spring presses toward the nose (2.0 N at the nose, 2.5 N behind a full strip).

In the scripted demo the right hand pulls the drawer out by its tab and holds it. The left lays the strip in the open
channel, legs down over the rail, and lets go. The right lets the drawer slide back in. The strip rides in with it,
meets the follower at the housing's mouth and is pressed with its front staple against the nose, under the driver. Then
the left slides the two sheets in under the nose against the paper guide and presses the top arm past its firing point.
The front staple is sheared off the strip and driven through both sheets, and the strip is one staple shorter; the
follower pushes it on to the nose. The left slides the stapled sheets back out onto the paper rest to show them joined,
and both arms go back to where they started. The task is one task file,
[`reload_a_long_arm_stapler_then_staple_two_sheets.py`](reload_a_long_arm_stapler_then_staple_two_sheets.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: holding a spring-loaded drawer open with one hand while the other lays a small part into it the right way
up (legs down, over a rail with about 0.85 mm of side play); then using the tool it loaded: lining paper up in a throat
against a guide, pressing a sprung lever through its firing travel (about 12 N), and leaving both hands clear.

Why two hands: the drawer's return spring pulls it shut (5 N shut, 9 N fully out) as soon as it is let go, and with the
drawer shut the channel is closed by its cap and the housing: there is no other way in. A strip can only be laid in
while the drawer is held out. With one hand (the right doing its part exactly as in the demo, the left never moving) the
demo scored 0.0: the drawer was pulled out 16 cm, held and let back in empty, and the strip stayed on its tray.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The stapler's base and housing and the tray are fixed;
the drawer, the follower, the top arm, the strip of staples and the two sheets are loose rigid bodies made of shapes
from `bodykit.py`, at real size.

- The drawer: a `Slider` from `bodykit.py` (a PhysX prismatic joint) between the housing and the drawer, from shut out
  to 20 cm, with a spring drive pulling it shut: 20 N/m with its target 25 cm past shut, so 5 N shut and 9 N fully out.
  Its channel is a tray 3 cm wide inside with 3 mm lips, with a rail 1 cm wide and 5.8 mm tall along its floor: full
  width up to 2.8 mm, then a 45-degree roof with a 2 mm flat top, so a strip set down within about 4.5 mm of the rail
  slides onto it. A seated strip's legs end 0.3 mm above the floor.
- The strip: 210 standard 26/6 staples (12.7 mm crown, 6 mm legs, 0.5 mm wire), 105 mm long and 6.3 g. A real strip is a
  glued, rigid stick, so it is one rigid body with the cross-section of a staple: a crown on top and two legs, which
  straddle the rail with 0.85 mm of play on each side. The two front staples are bodies of their own, held to the strip
  by fixed joints (the strip's glue) until the driver shears them off. They carry 1.5 g each (a real staple weighs 0.03
  g) so the solver can hold them to the strip; the strip's body has the other 3.3 g, so the strip's total is real.
- Orientation by geometry: inside the housing's mouth a strip guide (a steel plate across the tray) clears a strip
  seated on the rail by 1 mm. A strip laid upside down (legs up) or on its side rides on top of the rail, about 5.5 mm
  or more higher, and runs into the guide as the drawer goes in: it is pushed off the rail or it keeps the drawer from
  shutting, and it never reaches the follower. This was checked on an earlier version of the drawer (a 2.2 cm channel
  with 9 mm walls; the rail, the strip and the guide's 1 mm clearance as now): an upside-down strip laid on the rail
  (`STAPLER_FLIP=1`) was pushed off at the mouth and the drawer shut empty (score 0), and a strip that missed the rail
  and lay tilted against it jammed the drawer 105 mm open. On the final 3 cm tray the scripted hand failed to pick the
  upside-down strip off its tray, so that check could not be repeated. A strip lying across the tray does not fit in it.
  End for end a strip is the same, as a real one is.
- The follower: its own body (a 10 g steel block that straddles the rail) on a `Slider` in the housing, from the nose
  back 20 cm, pushed toward the nose by a long preloaded spring (a spring drive of 5 N/m whose target lies 40 cm past
  the nose, damped at 4 N s/m): 2.0 N at the nose, 2.5 N behind a full strip, in the range of a stapler's pusher spring.
  With the stapler empty it rests at the nose. A strip carried in by the drawer meets it at the mouth and pushes it
  back; it then holds the strip's front staple against the nose. A red marker on it shows through a slot along the
  housing's near side.
- The top arm: a `Hinge` at the housing's far end, sprung up 4 degrees (0.84 N m per degree, about 12 N on the pad at
  the firing point), with its stop at 4.5 degrees.
- Driving a staple is a rule, tied to the live state. After every control step the task file reads the top arm's angle.
  When it is pressed past 3 degrees (11 mm at the pad), the stapler is loaded at that moment (see Scoring) and the arm
  has come back up to within 1 degree of rest since the last staple, the driver drives the front staple: its glue to the
  strip is switched off, and the staple is moved down onto the paper under the nose, its crown on the top sheet and its
  legs through every sheet that lies flat on the base under the nose with the staple at least 2 mm inside its edges. A
  fixed joint (a `Glue`) to each of those sheets holds it there, so two sheets stapled together move as one. With no
  paper under the nose, the staple drops out onto the table beside the stapler. The follower then pushes the strip, now
  one staple shorter, on to the nose. Only the two front staples are bodies of their own, so the model drives at most
  two staples per trial.
- The sheets: stiff 0.5 mm boards (A6, 10.5 x 14.8 cm, 1.24 g each, the mass of 80 g/m2 paper; real paper is 0.1 mm
  thick and bends, which PhysX rigid bodies cannot). The base under them is
  slick, so a fingertip pressed on the top sheet slides both.
- Contacts: the follower's push on the strip, the nose's push on the front staple, and any touch between an arm and the
  drawer, the top arm, the strip, the staples or the sheets are read from PhysX contact forces after every control step.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the drawer, the follower, the top arm, the strip and
both sheets still: each moved less than 0.5 mm and turned less than 0.5 degrees over the last second) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704
for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E
cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=reload_a_long_arm_stapler_then_staple_two_sheets.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Reload the long-arm stapler and staple the two sheets together. Finish with both arms at rest, clear of the stapler and the paper." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/stapler-dream
```

The scripted demo, which reaches full score:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=reload_a_long_arm_stapler_then_staple_two_sheets.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Reload the long-arm stapler and staple the two sheets together. Finish with both arms at rest, clear of the stapler and the paper." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/stapler-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/stapler-dream/*.json; do inspect-robots video "$log"; done
```

Set `STAPLER_DEBUG=1` to print what the score is built from twice a second: how far the drawer is out and the follower
pushed back, where the strip sits on the rail, the follower's and the nose's contact forces, whether it is loaded, the
top arm's angle, the staples driven and the sheets they went through, any arm contact, the arms' speed and the TCPs.
`STAPLER_ONE_HAND=1` runs the one-hand check (the left arm stays where it started); `STAPLER_FLIP=1` starts the strip
upside down on its tray (a check that it cannot be loaded that way).

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,400 steps (545 s on the clock, with Dream answering in 0.70 s on average), and about 5 minutes for the scripted demo (708 steps, 35.4 s of simulated time, 309 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-N7ZF8M. Its review said:

> Construct validity (not met): The described sliding channel returns the strip into the housing but specifies no follower or other mechanism to feed staples to the nose.
>
> Scoring (not met): The loading tier does not specify the strip's longitudinal position or staple orientation, so an unusably loaded strip can receive credit.
>
> What to change: Specify a working long-arm stapler, including the follower or other mechanism that advances staples to the firing point, and how the strip is placed for it to feed. Update the loading score to require a correctly oriented and positioned strip, and verify the complete reload-and-staple sequence with that mechanism. Consider removing the exact return-to-start requirement from the full-score tier, since ending with the grippers safely clear is sufficient.

What changed in response:

- **A working feed mechanism (construct validity):** The magazine is now a front-loading drawer: a steel tray with a
  10 mm rail along its floor, closed at the nose by a cap with a pull tab. It slides out of the nose up to 20 cm
  against a return spring (5 N shut, 9 N fully out). The follower (staple pusher) is a 10 g block on its own slide in
  the housing. It straddles the rail, and a long preloaded spring pushes it toward the nose: 2.0 N at the nose, 2.5 N
  behind a full strip. With the stapler empty it rests at the nose.
- **How the strip is placed and fed:** With the drawer held out, the strip is laid in the open tray legs down, over the
  rail. Released, the drawer is pulled shut. The strip rides in, meets the follower at the housing's mouth and pushes
  it back, and the follower then holds the front staple against the nose, under the driver. This is all joint and
  contact physics; no rule moves the strip. In the demo the contact forces read 2.38 N at the follower and 2.44 N at
  the nose.
- **A real strip, oriented by geometry:** The strip is a full strip of 210 standard 26/6 staples (12.7 mm crown, 6 mm
  legs, 105 mm, 6.3 g). It is one rigid body, as a glued strip is, with a staple's cross-section, so its legs straddle
  the rail with 0.85 mm of play. A strip guide inside the mouth clears a seated strip by 1 mm. A strip that is upside
  down, on its side or off the rail rides 5.5 mm or more higher and runs into the guide: it is pushed off or it jams
  the drawer open, and it never reaches the follower. End for end a strip is the same, as a real one is.
- **Firing from the live state:** The two front staples are bodies of their own, held to the strip by its glue. A
  staple is driven only while the stapler is loaded at that instant (as defined below) and the top arm is pressed 3
  degrees (11 mm at the pad, about 12 N). The arm must also have come back up since the last staple. Driving is a rule,
  stated in fidelity: the front staple's glue is released, and the staple is fixed through each sheet lying flat under
  the nose with the staple at least 2 mm inside its edges. The follower then pushes the strip, now one staple shorter,
  0.5 mm on to the nose.
- **Loading score (scoring):** Loaded is read live at the end, never latched. It requires all of:
  - the drawer shut within 1 mm;
  - the strip seated legs down on the rail: within 1.5 mm of its middle line and 1 mm of resting on it, upright within
    8 degrees, along the channel within 5 degrees;
  - its front staple against the nose within 1 mm, pressed on it with at least 0.5 N of contact force;
  - the follower pressing on the strip's back with at least 0.5 N.
- **Full score:** The full score is loaded, plus the sheets stapled (a staple through both), plus both arms at rest
  (every joint under 0.05 rad/s) with no arm in contact with the drawer, the top arm, the strip, the staples or the
  sheets. The return-to-start requirement is removed, as suggested. success() also needs the moving parts still for
  1 s. Every tier has a real-trial measure:
  - open the drawer and look, or fire a test staple into scrap card;
  - lift the top sheet by a free corner;
  - check the video.
- **Also:**
  - The instruction is now goal-only: "Reload the long-arm stapler and staple the two sheets together. Finish with both
    arms at rest, clear of the stapler and the paper."
  - The title is free of method, and the category is now Societal relatability.
  - Real sizes. The strip was 70 mm and 30 g. The sheets were 10 x 7 cm, 3 mm boards; they are now A6 (10.5 x 14.8 cm,
    1.24 g, the mass of 80 g/m2 paper) as 0.5 mm stiff boards.
  - New demo (1.0) and Dream-1 (0.0) videos.
