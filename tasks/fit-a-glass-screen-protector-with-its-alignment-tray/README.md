# Fit a phone's glass screen protector with its alignment tray, from peeling the back film to lifting the empty tray off

Putting a tempered-glass screen protector on a new phone is an everyday job: people do it at home, or watch it done
at a phone-shop counter. Alignment-tray kits such as Spigen's Glas.tR EZ Fit turn it into a fixed sequence of real
steps. An iPhone 16 (no case) lies screen up on the desk; beside it lies the kit's tray, skirt down on its back film
"1", the glass held inside the tray by the tray's sticker "2". The instruction asks only for the protector to be put on
the phone with its tray and the empty tray left off the phone; it does not say how.

The task is the task file
[`fit_a_glass_screen_protector_with_its_alignment_tray.py`](fit_a_glass_screen_protector_with_its_alignment_tray.py)
for the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## The real procedure

Spigen's description of the EZ Fit kit: "Simply place the tray over the phone, press the center and watch the glass
adhere in place." Spigen support: "For any screen protectors that utilize an EZ FIT tray or aligner, please remove the
phone case before proceeding." A published review of the kit
([itnerd.blog, Glas.tR EZ Fit for iPhone 13][review]) gives the steps: "put the installation tray on the screen after
removing the backing tape on the screen protector", "slide your finger up and down the channel in the middle of the
screen protector", "Wait 30 seconds then pull the tab off." So the real steps are: peel the back film "1" off the tray
by its tab; hold the tray level and set it down round the phone; press the middle of the glass; wait 30 s; peel the
sticker "2" off the tray by its tab (it lets the glass go); lift the tray off. This task asks for all of them; none
is done before the trial starts.

## Why two hands, visible in the scene

Both the film and the sticker are held by adhesive, and the tray weighs 25 g. Pull a tab with one hand and the tray
comes with it: the film slides the tray across the desk instead of peeling, and the sticker drags the seated tray and
the phone along. So one hand holds the tray (across its width, by its two long walls) while the other peels each tab,
as a person does. Between the two peels the tray must come down level and square round the phone: it fits round it
with 0.4 mm to spare a side, with a 3 mm lead-in at the bottom of its skirt, and resting on the phone's 1.8 mm black
border it holds the glass 0.4 mm above the screen, centred. The glass must be pressed in the middle through the tray's
window while the tray is seated, then left for the maker's 30 s before the sticker is peeled; then the tray is lifted
straight off, leaving the glass on the screen.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on
Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs everything, including the
Dream-1 policy. The task needs no models from Arena's object library and uses no third-party assets: the phone, the
tray, the glass, the film and the sticker are built in these files from measurements.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a joint.
`bodykit.py` adds bodies made of several shapes and joints between them: importing it hooks `yam_arena.scene` when the
simulator starts, without changing yam_arena. This copy also turns on PhysX contact reporting for a chosen body (here
the glass), so the task file reads the contact force between the glass and the phone.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Phone, loose: Apple iPhone 16, 14.76 x 7.16 x 0.78 cm, 170 g, rounded corners (9 mm), switched off, no case, screen
  wiped clean; screen up, long side along y, its top end (camera) toward the left arm; centre at x 25.5, y -9.0.
- Alignment tray, loose, 25 g, clear moulded plastic (drawn grey): outside 7.84 x 15.44 cm, plus a flat 3 x 0.6 cm
  handle at each end; a skirt 4.5 mm deep that fits round the phone with 0.4 mm to spare a side, its bottom inside
  edge chamfered 3 mm at 45 deg as a lead-in; a seat frame (inside 6.88 x 14.48 cm, 1.5 mm deep) that rests on the
  phone's 1.8 mm black border; a 2 mm deck above with a 5.4 x 12.0 cm press window; 8 mm tall outside walls. It lies
  skirt down on the film "1", long side along y, centre at x 25.0, y 9.0 (its right-hand end 2.3 cm from the phone's
  top end).
- Glass protector, 6.8 x 14.4 cm, 0.5 mm (glass with its films), 7 g (drawn pale blue), held in the tray's seat frame by
  the sticker "2" (0.4 mm play a side); with the tray seated it hangs 0.4 mm above the screen.
- Back film "1": a 0.5 mm sheet under the whole tray (8.0 x 15.6 cm), with a red pull tab 2.0 x 2.2 cm (1.5 mm thick)
  standing up 4 mm past the tray's right-hand (-y) end, 2.2 cm nearer the robot than the tray's middle line.
- Sticker "2": an 8 mm wide yellow strip along the deck's far long side (3.2 cm from the middle line), with a blue pull
  tab 2.0 x 2.2 cm standing up 4 mm past the tray's right-hand end.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial, as in the screenshot.

How the adhesives are modelled (rigid bodies; see Fidelity):
- The film's adhesive is a slide joint along the tray (the peel) with 25 N s/m of drag (0.6 N at a 2.4 cm/s pull); it
  lets the film go only at the end of the peel (165 mm, the film's length) and only to a pull of 2 N or more there.
- The sticker's adhesive is the same over 50 mm. The sticker holds the glass up on a 200 N/m spring (so the glass can
  be pressed down onto the screen) and lets it go once 1.5 N acts on it sideways (the peeled sticker drags it against
  the frame).
- The glass's adhesion is a rule: glass that has been pressed onto the screen (5 N or more) sticks where it lies once
  the sticker has let it go, lying flat on the screen; from then on it moves with the phone. Glass let go onto the
  screen without a press lies loose and earns no credit for the glass.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the poses of the phone, tray, glass, film and sticker, the contact force between the
glass and the phone, and where each gripper is.

- Film off: the film's sheet no longer under the glass (its centre 110 mm or more from the glass's, or 10 mm or more
  below it).
- Seated: the tray within 1.5 mm and 2 deg of centred on the phone, resting on it (within 1 mm).
- Pressed: the glass pressed onto the screen with 5 N or more (the PhysX contact force between glass and phone) by a
  finger whose centre is within 20 mm of the glass's centre, with the tray seated and the film off.
- Sticker off: the sticker's centre 100 mm or more from the tray's centre, or 15 mm or more above it.
- Waited: the sticker came off 30 s or more after the first press, with the tray seated then.
- Glass on: flat on the screen (every corner within 0.5 mm of it), every corner within 1.5 mm of its true place along
  each edge (a border of 1.8 mm on every side: each border at each corner between 0.3 and 3.3 mm), the sticker off.
- Tray off: no part of the tray within 10 mm of the phone's outline lower than 20 mm above the screen. Hands clear: both
  grippers' centres 30 mm or more from the tray, the glass and the phone. Still: tray, glass and phone moving less
  than 1 mm over the last 1 s.

Score: 1.0 for film off, pressed before the sticker came off, waited, glass on, tray off, hands clear and still.
0.75: the same, but the tray still on the phone, or the hands not clear, or not still. 0.5: film off, glass on and
pressed, but the sticker came off less than 30 s after the press (or before it). 0.25: film off only. 0: the film
still on the tray. Doing nothing scores 0.

A real trial is scored from the video, a calliper and a photo: film and sticker off, the press (a fingertip held on
the middle of the glass through the window for 1 s or more while the tray sits flat on the phone) and the 30 s from the
video's timestamps; the four borders at each corner with a calliper; and a photo from above with the screen off and a
lamp reflected in it at a low angle, in which no air gap (a white, silver or rainbow patch, where the glass has not
stuck) may touch any edge of the glass. Small bubbles wholly inside the edges do not count.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fit_a_glass_screen_protector_with_its_alignment_tray.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the glass screen protector to the phone with its alignment tray, and leave the empty tray off the phone." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/screen-protector-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The left gripper takes the tray across
its width by its two long walls, 6 cm toward its +y end, and lifts it 6 cm, level. The right gripper comes down with
its fingers 30 mm apart either side of the film's tab, closes on it and pulls it along under the tray and past its end
(190 mm in 8 s), the left hand holding the tray up; the film lets go and the right hand drops it on the desk to the
right. The left hand carries the tray over the phone, lowers it to 6 mm above, waits, lowers it round the phone,
lets go and moves away. The right hand presses the middle of the glass through the window with its closed fingertips
(commanded 12 mm below first touch, about 7 N) for 2 s and lifts off. While the 30 s pass the left hand comes back and
holds the seated tray across its width; the right hand takes the sticker's tab and pulls it 75 mm along the tray, lifts
the freed sticker away and drops it beside the film. The left hand lifts the tray straight off and puts it down where
it started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fit_a_glass_screen_protector_with_its_alignment_tray.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the glass screen protector to the phone with its alignment tray, and leave the empty tray off the phone." \
  --scorer task_score --max-steps 1900 --store-frames --log-dir ~/yam-eval/screen-protector-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/screen-protector-dream/*.json; do inspect-robots video "$log"; done
```

`FITGLASS_DEBUG=1` prints, every 10 steps (`FITGLASS_DEBUG_EVERY=1` for every step), the glass's contact force on the
phone, the tray's offset and turn from centred on the phone, each stage (film off, pressed, sticker off and when,
waited, glass held), the glass's offset and lift, whether the tray is off, how close the nearest gripper is, and where
each gripper is.

`FITGLASS_ONE=1` runs the demo the wrong way, with the right hand alone (the left stays at the ready pose and nothing holds the tray): the right hand takes the film's tab of the tray lying on the desk and pulls. The tray comes with the film: it slides 22 mm and turns 5 deg until its corner catches on the phone; the film then slides about 130 mm along under it but does not let go (it needs its whole 165 mm and a 2 N pull), and carrying the tab away drags the tray off with it. The film never comes off: score 0. The sticker behaves the same way on the seated tray: its drag and its 2 N let-go pull the tray and the 170 g phone along unless a hand holds the tray.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell, with two runs of this task sharing it: the scripted demo while rendering 1280 x 720 video took 516 s (1,734 steps, 86.7 s of simulated time, ending on success, which includes the maker's 30 s wait); a Dream-1 trial of 2,400 steps at 320 x 176 took 660 s (0.28 s per step). Debug runs of the demo without video took 6 to 9 minutes. Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Fidelity

Rigid bodies, 20 Hz control. The films are rigid 0.5 mm sheets and their peel is a slide along the tray (see the
adhesive model above) rather than a bending peel; the glass and its films are one 0.5 mm plate. Adhesion is the rule
above. The tray's sizes, the tab sizes, the adhesive forces and the masses are estimates to be measured on the real
kit. The phone's 1.6 mm camera plateau is not modelled (the phone lies flat). Not asked for: cleaning and dusting the
screen before the trial (done at reset), the "press and slide" stroke (a press in the middle), pushing out bubbles and
peeling the glass's front film afterwards (the owner's last steps).

## Building it for real

An iPhone 16 (or any phone with an EZ Fit kit), switched off, without its case; Spigen Glas.tR EZ Fit kits for it
(two trays a box; refill packs are sold). Tape marks for the phone and the tray. For each trial: peel the previous
glass off the screen and wipe the screen (the kit's wipe and microfibre cloth), put the phone at its mark screen up,
and lay a fresh tray skirt down at its mark with film 1's tab standing at its right-hand end. Scoring needs a calliper
and a desk lamp for the low-angle photo.

[review]: https://itnerd.blog/2021/09/30/review-spigen-ez-fit-glas-tr-slim-screen-protector-for-the-iphone-13/
