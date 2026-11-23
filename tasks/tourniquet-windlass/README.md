# Tighten a tourniquet's windlass to the green and lock it

A training thigh lies across the table with a windlass tourniquet (a C-A-T Gen 7) strapped round it, snug. Beyond it,
a pressure gauge on a bladder under the band reads 60 mmHg; its dial is green from 250. The instruction is:
**"Tighten the tourniquet on the training leg until its gauge reads green, then lock the rod in its clip."**

To do it, the robot has to:
- turn the windlass rod, which twists the band tighter and raises the gauge;
- keep hold of it against the band's pull, about 2 N m in the green: the band turns the rod back the moment nothing
  holds or locks it;
- go on far enough: the rod has to turn about 210 degrees, and a gripper's wrist runs out of travel partway, so the
  robot has to plan its grip or regrip on the way (the demo regrips while the other hand holds the rod);
- lock it only when the gauge is green. The tourniquet's own clip has a hook on each edge of the band, beside the
  windlass, and the rod lines up with one of them twice every half turn. Turning anticlockwise, the first two times
  come 30 and 78 degrees in, with the gauge at about 90 and 140; locking there looks done but scores 0.25. The rod
  lines up again 210 degrees in, with the gauge at about 270;
- lock it by pushing the rod down into the hook, past the hook's lips (about 10 N directly over the hook), then let go.

## Why it matters

Uncontrolled bleeding from a limb is a leading cause of preventable death after injury, and a windlass tourniquet is
the standard first-aid tool for it. Armies, ambulance services and public courses such as Stop the Bleed teach this
sequence on this device: twist the windlass until the bleeding stops, lock the rod in the windlass clip, then secure
it with the strap and note the time (this task leaves out the last two). A rod locked too early leaves the limb
bleeding; one let go unlocked unwinds. Robots that help with casualty care, at mass-casualty scenes, in evacuation or
where medics can't reach, would need the same skill. Here the gauge stands in for "until the bleeding stops".

The task tests three things:
- **Reading the goal from an instrument.** The instruction asks for the gauge in the green, not a number of turns,
  and the clip lines up twice before the gauge gets there.
- **Holding against a load.** The rod is always pulled back, so a regrip needs something else holding it first,
  and letting go before it is locked undoes the work.
- **Locking a snap fit.** The rod's end has to go down into a hook only 13 mm wide, on the hook's line, and be pushed
  past the hook's lips, while the band pulls the rod sideways.

No task on the board tightens a tourniquet. The closest are #917 (release a phlebotomy tourniquet before withdrawing
the needle), #918 (add a fresh pad on top of a soaked dressing and hold it) and #409 (two-rescuer CPR on a manikin).

## Changes in this version

This version answers the review of TK-EL693A. The tourniquet's own clip, the domain, the scoring order and the
bladder's size are unchanged from the last version.

- **Winding resistance.** The band now pulls the rod back with 1.8 N m at 250 mmHg and 1.94 N m where the rod locks
  (270 mmHg), up from 0.22 N m. The pull is proportional to the gauge's pressure, and builds up over the first quarter
  turn as the strap wraps the rod. A drag of 0.2 N m s/rad stands in for the strap's friction. The value is derived
  from the only published windlass-torque measurement we found, scaled to this thigh. See "Simulation fidelity" for
  the derivation, and "Calibration" for measuring it on the trainer with a luggage scale.
- **Snap force.** The clip's lips now resist the rod. Pushing it past them takes 10 N at the hook, and pulling it back
  out takes 10 N again. The rod counts as locked only once it is past them. We found no published value, so 10 N is an
  estimate. "Calibration" describes measuring it with a kitchen scale.
- **Demo.** Re-tuned for the new loads, and still 1.0.
  - The right hand turns the rod with its hand running up to 40 degrees (the demo's cap) ahead of the rod.
  - The left holds the rod against the band while the right regrips.
  - To lock the rod, the left takes its other end and pushes it down past the hook's lips while the right keeps the
    rod on the hook's line. Before, the right lifted its own end.
  - The `--early-lock` demo variant was dropped. The scoring rule for a lock under the green (0.25) is unchanged.
- **Dream-1** was re-run on the new loads: 0 (it never brought the gauge to green or locked the rod).
- **Bladder.** The custom heat-sealed bladder is replaced by a piece of a standard mountain-bike inner tube. The
  README gives the parts, a 10-minute preparation, the connection to the gauge and bulb, the sealed port, and a
  leak check.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the tourniquet, the clip's hooks and their lips' snap, the band's pull, the gauge, the lock, the monitor, `success()` and `score()` |
| `geometry.py` | Dimensions and layout: the thigh, the band and windlass, the clip, the gauge, the pressure rule, the scoring tolerances |
| `build_assets.py` | Writes `assets/tourniquet.usda` and the dial's texture (needs `pxr`, PIL and the DejaVu fonts) |
| `assets/` | The tourniquet on its training thigh, with the gauge |
| `scenekit.py` | USD bodies and reset hooks for yam_arena task files |
| `demokit.py` | Helpers for the scripted demo |
| `demo.py` | The scripted full-score demo, and variants that keep holding, let go to regrip, or keep a hand on after locking |

`scenekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. Tested with yam-isaac-lab-arena at `bd62771`
(Isaac Lab-Arena 0.3.1).

All the assets are generated by `build_assets.py` from `geometry.py`; there are no third-party models. The dial's
texture is drawn with DejaVu Sans and DejaVu Sans Bold (DejaVu fonts 2.37, Ubuntu's `fonts-dejavu-core`). They are
under the DejaVu Fonts License, which is the Bitstream Vera Fonts license with the DejaVu changes in the public domain
(<https://dejavu-fonts.github.io/License.html>). It lets the fonts be used free of charge, including to render text
into an image. No font files are included here.

## Run a trial

From this folder, in a shell with the yam_arena environment active. New shells on the walkthrough's Brev instance
have it; elsewhere, `source <ARENA_DIR>/yam.env`.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Tighten the tourniquet on the training leg until its gauge reads green, then lock the rod in its clip." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tourniquet-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

- **Seeds:** there is one fixed layout and no randomisation (`object_jitter=0`), so the seed has no effect.
- **Runtime:** a full 2,400-step Dream-1 trial took 5 min 56 s on an RTX 5090 (mean inference latency 0.64 s),
  plus about a minute to start Isaac Sim. A successful trial ends earlier: the scripted demo stops after 804 steps.

Dream-1 scored 0 on this version (the new band resistance and clip snap). Its right hand went to the rod and the
band several times, but the gauge never reached the green and the rod was never locked in a hook; nothing else was
disturbed. 1,314 of its 2,400 action rows were beyond the arm's limits and were clipped. 50 inference requests (median
latency 0.50 s) in one continuous Dream-1 session, with no errors or retries.

## Scripted demo

```bash
python demo.py task.py demo_out                     # full score: demo_*_cam.mp4, setup.png, result.json
python demo.py task.py hold_out --hold              # turns it into the green but keeps holding it: 0.5
python demo.py task.py slip_out --slip              # lets go to regrip with nothing holding the rod: 0
python demo.py task.py keep_out --keep-hold         # locks it in the green but keeps a hand on it: 0.5
```

1. **First grip.** The right hand takes the rod's end toward the arms and the hip from above and turns the rod
   anticlockwise (seen from above).
   - It passes the first two times the rod lines up with a hook: 30 and 78 degrees in, gauge about 90 and 140.
   - It goes on to about 175 degrees (gauge about 235), where its wrist is near the end of its travel.
   - Against the band's pull, up to 1.7 N m here, the hand runs up to 40 degrees (the demo's cap) ahead of the rod: the wrist
     gives and the fingers splay round the rod. The hand also tips to keep the rod level.
2. **Regrip.** The left hand takes the rod's other end, its grip leaning 45 degrees along the rod so the arms keep
   clear.
   - It pushes on to about 196 degrees (gauge about 256) and holds the rod there against 1.85 N m.
   - The right lets go, returns to its ready pose and takes the same end again with its wrist turned the other way
     round. It takes up the pull, and the left lets go.
3. **Into the green.** The right turns the rod to 210 degrees (gauge 270), where the end it does not hold lines up
   with the hip-side hook, and holds it within a quarter of a degree of the hook's line against 1.94 N m.
4. **Lock.** The left takes that other end beyond the hook, 7.3 cm from the rod's middle, leaning out along the rod.
   - It tips the end down, leading the rod's measured tilt, while the right keeps the rod on the hook's line and tips
     its own end up with it. While the end comes down between the hook's walls, the left goes on only with the rod
     within half a degree of the hook's line.
   - The end goes into the hook's channel and meets the lips at 16 degrees. It is pushed past them at 19.9 degrees,
     against their 10 N, and seats on the hook's floor at 21.
   - Both hands let go and back off.

Every move follows the rod's measured angle and tilt, and the script checks the gauge on its needle before locking.

The demo succeeds (1.0) in 804 steps (40 s).

The variants:
- `--hold` keeps hold in the green without locking: 0.5 (726 steps).
- `--slip` lets go to regrip with nothing holding the rod: 0 (470 steps), because the band turns it back to where it
  started.
- `--keep-hold` locks it in the green but keeps a hand on it: 0.5 (932 steps).

## Scoring

Judged at the end of the trial from a photo of the gauge and the clip, and the trial's video for letting go. Take the
first row that applies.

- **Green:** the gauge reads 250 mmHg or more with no part of the robot touching the band, the bladder, the thigh or
  the gauge. Pressing on any of them could push the reading up without tightening the windlass, so a reading taken
  while the robot touches them doesn't count as green.

| Score | When |
|---|---|
| 1.0 (success) | For the whole last 1 s: green, the rod locked in the clip, and no part of the robot within 1 cm of the rod or the clip. This ends the trial |
| 0.5 | Green, with the rod locked, or held (part of the robot within 1 cm of the rod) level, not pressed down onto the band |
| 0.25 | The rod is locked in the clip but the gauge is not green (locked too early), wherever the robot is |
| 0 | Anything else: not tightened, or let go unlocked so the band turned it back |

- **Locked:** an end of the rod has snapped down past a hook's lips onto the hook's floor, where the lips sit over it
  and hold it down. A rod resting on a hook's lead-in or walls, above the lips, is not locked. To check after the
  trial, pull the seated end lightly upward by hand: a locked rod stays in.
- **Chances:** turning anticlockwise, the rod lines up with a hook 30, 78 and 210 degrees in (gauge about 90, 140 and
  270). Turning clockwise, it lines up 102 and 150 degrees in (about 160 and 210), and next only past the gauge's stop.
- **In simulation:**
  - **The gauge** reads 60 mmHg plus 1 mmHg per degree the rod has been turned, either way, from where it started;
    the needle shows it, and a stop holds it at 300.
  - **Locked:** the rod lies within 1.5 degrees of a hook's line, seen from above, and has been pushed past the hook's
    lips against their spring. That means it is tipped to 19.9 degrees or more, its centre line below the lips'
    undersides; it seats at 21.
    - The lips hold it down from then on.
    - It unlocks only if pulled back up to under 19.6 degrees, which takes 10 N at the hook.
    - "Simulation fidelity" describes the lips' spring.
  - **Let go:** for the last 20 control steps, both arms stay at least 1 cm from the rod and the clip. The check uses
    yam_arena's collision hulls of every link, the gripper body and both fingers, at their simulated poses.
  - **Held:** part of either arm within 1 cm of the rod. A held rod counts toward 0.5 only within 5 degrees of level,
    since pressing the windlass down onto the band could push the reading up.
  - **Touching:** part of either arm within 3 mm of the thigh with its band and bladder (a cylinder of the band's
    radius) or of the gauge (a disc the size of its case).

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left. Heights are above
the table top.

- **Training thigh.** A firm, skin-toned foam cylinder, 12.7 cm across and 40 cm long, lying across the table along
  y with its axis at x 28, 6.55 cm above the table; the hip end is on the robot's left. It rests on two pairs of
  wooden chocks (3 x 4 x 3.5 cm tall) at y +-15, their inner faces 5.6 cm either side of the axis, and is held down by a
  webbing strap over it at each end, so it can't roll.
- **Tourniquet.** A C-A-T Gen 7 windlass tourniquet, unmodified: the blue trainer (North American Rescue item 30-0033,
  NSN 6910-01-560-2972) or a standard one. Its band is 3.8 cm wide; its ribbed windlass rod is about 15 cm long and 12
  mm across. It is routed round the thigh at y 0 and fastened snug, its free end stuck back along the band and taped
  flat so nothing is loose to pull (the band is not to be re-tensioned), and its time strap folded flat away from the
  clip. The windlass sits on top of the thigh, the rod's axis 14.85 cm above the table.
  - **Clip.** The tourniquet's own windlass clip, on the band beside the windlass, toward the arms. Its two C-shaped
    hooks sit at the band's edges, about 4.3 cm toward the arms from the windlass and 1.9 cm either side of the band's
    centre line, where the band curves down the thigh. A locked rod therefore lies along the band, about 24 degrees
    to one side, with its end tipped about 21 degrees down into a hook. The rod is pushed down into a hook past its
    lips, which then close over it. The simulation models each hook as a channel 13 mm wide, 0.5 mm clear of the rod
    each side, with lips reaching 1 mm in (an 11 mm opening for the 12 mm rod) 7 to 8 mm above its floor, and a
    lead-in above them.
  - **Bladder.** Under the band, a 5 x 42 cm air bladder is wrapped once round the thigh (39.9 cm round), overlapping
    itself by 2 cm. It is only a little wider than the band, so it reads close to the pressure under the band. A much
    wider bladder would bulge out either side of the band and read low. It is made from a bicycle inner tube in about
    10 minutes:
    - **Parts** (about US$15, plus the gauge):
      - a 26 x 1.95-2.125 in mountain-bike inner tube, which lays flat about 5.1 cm wide. For example, the Sunlite
        26 x 1.95-2.125 lists a flat-fold width of 51 mm, at about US$8. A 700c road tube is too narrow: it lays flat
        only about 2.5 cm wide;
      - a 51 mm (2 in) binder clip;
      - a 1/4 in straight barbed hose connector, or a reducer from 1/4 in to the gauge tubing's size;
      - two small zip ties and some electrical tape;
      - a sphygmomanometer Y connector, a sphygmomanometer bulb with its release valve, and a pinch clamp.
    - **Make it:**
      1. Cut a 47 cm piece of the tube, away from the valve.
      2. Fold one end over twice, about 1.5 cm each time, and clamp it flat across its whole width with the binder
         clip. Fold the clip's handles flat.
      3. Push the barbed connector 2 cm into the other end. Gather the rubber round it, pull the two zip ties tight
         over it 1 cm apart, and wrap the neck with tape. That leaves 42 cm of bladder between the clip and the neck.
      4. Connect the bladder through the Y connector to the gauge's tube and the bulb.
    - **Fit it:**
      - Wrap the bladder flat round the thigh at y 0, with its ends overlapping 2 cm on the far side of the thigh (away
        from the arms) and the clipped end underneath; the clip makes a small lump under the band there, which the
        calibration checks take into account.
      - The neck turns out from under the band's knee-side edge. Its tube runs along the table to the gauge's post.
      - Route the band over the bladder and fasten it snug.
    - **Seal it:**
      1. Pump the bladder to 60 mmHg and close the bulb's valve.
      2. Wait 5 minutes: the gauge must not fall more than 2 mmHg.
      3. Close the bulb's branch with the pinch clamp and take the bulb off, so nothing in reach can change the
         pressure.
      4. At the 210-degree check under "Calibration", lock the rod and watch the gauge for 2 minutes: it must not fall
         more than 4 mmHg. For comparison, OIML R16-1 allows a clinical sphygmomanometer 4 mmHg a minute.
      5. If it leaks, re-clip or re-tie the leaking end.
    - The tube is stretchy rubber, so only pump it under the snug band. The calibration checks below tie the reading to
      the rod's turn.
  - **Start.** The rod starts level, turned 30 degrees clockwise (seen from above) from where its end would line up
    with the hip-side hook: one end toward the arms and the hip, 54 degrees off the band's line.
- **Gauge.** An aneroid gauge, 12 cm across, reading 0 to 300 mmHg, with a stop at 300 and green tape stuck on the dial
  from 250 to 300, on a 14 mm square post on a 7 x 7 cm foot at x 43, y -15, its dial's centre 12 cm above the table, tilted 50
  degrees up toward the arms.
- **Calibration.** With the rod as set, the gauge reads 60. The simulation adds 1 mmHg per degree of turn, either
  way. On the real trainer, turn the rod by hand and check the gauge where it lines up with a hook:
  - turning anticlockwise (seen from above), under 150 at 30 and 78 degrees, and about 270 (at least 250) at 210;
  - turning clockwise, under 250 at 102 and 150 degrees.

  Start the band tighter or looser if not. Also check that the rod sweeps over the clip while level, and that pushed
  down into a hook it snaps in and stays in when let go.
- **Band pull and snap force.** Measure both on the trainer before the first trials, and the pull again every 10
  trials.
  - **Band pull:**
    1. Tie a loop of thin cord round the rod 6 cm from its middle, and hook a digital luggage scale (0-50 kg, 10 g
       steps) on it.
    2. With the rod turned anticlockwise to 78, 150 and then 210 degrees, pull the scale level and at right angles to
       the rod, in the tightening direction, until the rod just moves on. Read the peak.
    3. Ease the rod back slowly and read the scale while it holds the rod still.
    4. The band's pull is the mean of the two readings in kg, times 9.81 x 0.06 m.

    The simulation uses 0.86, 1.51 and 1.94 N m at those angles: 1.5, 2.6 and 3.3 kg on the scale. If the trainer
    differs by more than a quarter, set `TAU_GREEN` in `geometry.py` to the 210-degree pull times 250/270 (it is the
    pull at 250 mmHg).
  - **Snap force:**
    1. Stand the thigh, on its chocks, on a kitchen scale (5 kg, 1 g steps) and zero it.
    2. Turn the rod 30 degrees, where an end lines up with the hip-side hook. The band pulls only weakly there.
    3. Press the rod straight down with a fingertip directly over the hook (5 cm from the rod's middle, where the
       simulation applies its 10 N), slowly, until it snaps in. A press at the rod's tip reads about a third less.
    4. The snap force is the peak reading minus the reading when the end first touched the lips, times 9.81.
    5. Pull the seated end straight up with the luggage scale's cord until it comes out, and read the peak.

    The simulation uses 10 N, about 1.0 kg, both ways. Set `F_SNAP` in `geometry.py` to the mean of three tries.
- **Arms** start at the ready pose with open grippers.
- **Safety.** A training thigh only, never a person. The rod snaps back when released unlocked; keep hands clear of
  it and of the arms.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control.
- **The tourniquet** is an articulation fixed to the world. Its base is the thigh, the chocks, the band, the bladder
  and the gauge.
  - The windlass turns about the vertical on top of the band. The rod can tip on it about its middle, up to 30
    degrees either way, against a spring of 0.3 N m/rad: the strap holding it level.
  - **The band's pull** is a torque from task code, against the turn:
    - 1.8 N m x (gauge reading / 250 mmHg), building up linearly over the first 90 degrees as the strap wraps the rod;
    - 0.22 N m at 30 degrees, 0.86 at 78, 1.51 at 150, 1.8 at 190 (250 mmHg) and 1.94 at 210, where the rod locks;
    - a drag of 0.2 N m s/rad stands in for the strap's friction, so a rod let go unwinds at about a turn a second.
  - **The clip.** Each hook's floor, walls and lead-in are fixed shapes that collide, laid along the line a seated
    rod takes. The plate joining the hooks across the band is drawn only. The lips are drawn, and task code gives
    them their snap:
    - When the rod lies on a hook's line and its end comes down to the lips (its axis 4.4 mm above where it seats),
      task code adds the lips' spring to the rod's tilt spring. It pushes back, rising to 10 N at the hook as the end
      reaches the point where it is past them (its axis 1 mm above where it seats).
    - The rod counts as locked only if it is still pushed down past that point a control step (50 ms) after the spring
      came on, so it takes at least 10 N.
    - The lips then hold it down, pressing it onto the hook's floor with about 5 N net of the strap's pull. Pulling it back up past them takes
      10 N again.
    - Task code switches the tilt joint's drive between the strap's spring and the strap's plus the lips'. PhysX
      applies the drive implicitly at every physics step. No joint breaks, so side loads can't unlock the rod.
- **Where the band's pull comes from.** It is derived, not measured on this trainer.
  - **The measurement.** Fultz et al., "Tourniquet Pressure Magnitudes and Distribution During Limb Applications",
    J Spec Oper Med 2026;26(2):61 (doi:10.55460/J.Spec.Oper.Med.2026.BYA5-RH3Y), measured a median windlass-rod torque
    at occlusion of 3.31 N m on adult thighs, at a median 336 mmHg. They used a SOF-T Gen 5, a 3.8 cm windlass
    tourniquet like the C-A-T.
  - **Scaled to this thigh.** The band's tension, and so its pull on the rod, goes with the pressure and the limb's
    radius (Laplace). Their thighs' size isn't given in the abstract. Assuming about 54.5 cm round, the median thigh in
    Slaven et al., J Spec Oper Med 2015;15(1):39 (doi:10.55460/4g5t-09t4), scaling to 250 mmHg and this 40 cm thigh
    gives 3.31 x 250/336 x 40/54.5 = 1.8 N m.
  - **The floor.** It can't be less than about 0.9 N m. 250 mmHg under a 3.8 cm band on a 6.35 cm radius takes 80 N
    of tension, and both sides of the strap pull on the rod at about 5.5 mm: 2 x 80 x 0.0055 = 0.88 N m. Real bands
    lose much of their pull to friction. Pulling a C-A-T's strap with a known force on thighs gives only about a
    quarter of the Laplace pressure (Hooke et al., Mil Med 2021;186(S1):378, doi:10.1093/milmed/usaa266; Valliere et
    al., J Am Coll Surg 2018;227(3):332, doi:10.1016/j.jamcollsurg.2018.06.005), so the real pull sits well above this
    floor.
  - **The range.** The same abstract gives 2.50 N m at 261 mmHg on upper arms, about the same torque per mmHg as on
    thighs; read that way, 250 mmHg would take about 2.4 N m. So the real pull is likely 1.8-2.5 N m; the sim uses the
    lower end until the trainer is measured.
  - **The arms' force.** At 1.94 N m, a grip 5 cm from the rod's middle needs about 39 N across the fingers, close to
    the YAM's limit; gripping or pushing at the rod's end (about 7 cm) needs about 28 N. The demo turns the rod with a
    wrist twist and lets the fingers splay round it, and pushes at the rod's end to lock it.
  - Measure it on the trainer as under "Calibration".
- **The thigh's underside** is 2 mm off the table; the band and bladder (about 6 mm together) pass under it at y 0,
  where the chocks lift the real thigh clear.
- **The snap force is estimated.** We found no published value for a C-A-T clip, so 10 N is an estimate for a
  thumb-pressed plastic clip. Measure it as under "Calibration".
- **Monitor:** after each control step, task code:
  - adds up the rod's turn (the joint reads modulo a turn), sets the gauge's needle from it and sets the band's pull;
  - switches the lips' spring on and off as above, and latches and unlatches the lock;
  - checks how close the collision hulls of either arm, fingers included, come to the rod and the clip.
- **Simplified:**
  - Pressure is linear in the turn here; the real pressure under a band rises faster than linearly.
  - The real strap twists and bunches under the rod; here it is not modelled.
  - The band's pull is the same turning either way and doesn't depend on speed beyond the drag. A real strap's
    friction makes it pull back less than it resists.
  - The clip's position and the rod's size follow the C-A-T's instructions for use and published sizes, not a
    measurement, so the calibration checks above set the real build's start.
  - The rod and clip are drawn dark so they show against the blue band.
- **Resets:** the reset untwists the rod, levels it, switches off the lips' spring and resets the gauge.

## Reset between trials

1. Pull the rod's seated end up out of the clip, past the lips.
2. Untwist the rod back to its start: level, one end toward the arms and the hip, 30 degrees clockwise from lining up
   with the hip-side hook.
3. Check that the gauge reads 60 mmHg. If it doesn't, put the bulb back on its branch, open the clamp, re-pump, then
   clamp the branch and take the bulb off again.

Every 10 trials:
- re-check the gauge at the 30, 78 and 210-degree hook positions, as under "Calibration";
- re-check the band's pull at 210 degrees with the luggage scale;
- check that the bladder holds its pressure.
