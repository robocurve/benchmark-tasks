# Lock a used safety needle's hinged sheath one-handed on the table, then bin the syringe

**Instruction:** "Lock the used needle's safety sheath by pressing it on the table with the holding hand, then
put the syringe in the sharps tray."

After an injection, medical assistants (O\*NET 31-9092.00), registered nurses (29-1141.00) and licensed
practical nurses (29-2061.00) shield the needle at once with the hand that holds the syringe, then drop it in
the sharps container. The trial starts just after the syringe was laid down beside the practice pad, so every trial starts
from the same pose; a worker would usually shield it straight from the injection grip. A hinged-sheath safety needle (Terumo SurGuard 3 class) can be shielded by finger, by
thumb, or by pressing the sheath on a hard surface until it clicks. A parallel gripper that holds the barrel
has no spare thumb, so it uses the surface method: pick the syringe up by the barrel, turn the open sheath
downward, set its end on the table, rock the barrel over it so the sheath swings shut around the needle, and
press until the catch clicks. The other gripper stays away from the bare needle the whole time. The syringe
then goes in the sharps tray.

## Running a trial

```
inspect-robots run --embodiment yam_arena -E task=close_safety_needle_shield_one_handed.py \
    --instruction "Lock the used needle's safety sheath by pressing it on the table with the holding hand, then put the syringe in the sharps tray." \
    --policy yam_scripted --scorer task_score --max-steps 1400
```

- Scene: `station`. Control at 20 Hz. Run limit: 1,400 steps (70 s). The scripted demo takes 28 s, and a trial
  takes about 3 to 6 minutes on one GPU.
- No object jitter: one fixed layout, and the seed changes nothing.
- `--scorer task_score` gives 1.0 on success and otherwise the score at the end (`SCORE_OVER = "end"`).
- Offline checks: `pytest test_close_safety_needle_shield_one_handed.py` (generated scorer tests: start, goal,
  partial and cheat layouts).

## Scoring

The scorer reads the syringe (pose, sheath hinge angle, catch position), the tray's pose, and each gripper's
centre point (midway between its finger pads) and finger opening from the robot's joint state.

**Locked** means the sheath is no more than 1.5 deg short of closed (it may sit up to 3 deg past closed, on
its stop) and the catch is back out to within 0.3 mm of its rest: it has dropped behind the needle. A sheath that only
rests on the needle with the catch still lifted is not locked. **Bare** means not locked.

| Rule | Value |
|---|---|
| Sticky zero (a) | While the needle is bare, a gripper other than the holding one has its centre point within 100 mm of the needle tip. The holding gripper is the one nearest the middle of the barrel. |
| Sticky zero (b) | While the needle is bare, the holding gripper's centre point is within 100 mm of the needle tip but more than 5 mm beyond the barrel's shoulder or flange, measured along the syringe (it came at the hub or needle). |
| Sticky zero (c) | The bare needle tip is inside the pad or a tray wall (1.5 mm tolerance). |
| Sticky zero (d) | The syringe's origin is more than 20 mm below the table top (it fell off). |
| Sticky zero (e) | At the step the sheath locks: its lowest point is more than 3 mm above the table top, or the needle tip is within 10 mm of the pad or of the tray's outline seen from above, or a gripper's centre point is closer to the sheath's bounding box than 15 mm plus half that gripper's finger opening (a finger could be on the sheath). |
| Released and at rest (every credit) | Each gripper centre at least 30 mm from the syringe's bounding box. Between 50 ms steps the syringe's two ends and the tray move less than 0.5 mm and the sheath turns less than 0.25 deg. Held for 1 s. |
| 1.0 | Locked; needle tip inside the channel at least 3 mm from the sheath's far end; the whole syringe inside the tray's inner outline seen from above (2 mm tolerance), lowest point below the rim, tray upright on the table. |
| 0.4 | Locked and covered, at rest anywhere else. |
| 0.15 | Bare, in the tray. |
| 0.10 | Bare, not in the tray, sheath within 45 deg of closed. |
| 0 | Anything else: the untouched start, a syringe still held or still moving. |
| Success | 1.0 held for 1 s (`SUCCESS_HOLD_S`). |

**Measuring it in a real trial**

1. Use a real unit with the cannula tip ground blunt. The catch grips the shaft, so the sheath still locks.
   Run behind the cell's guard with the e-stop in reach, and handle the unit with forceps.
2. Film from above and from the side at once, each view with a 100 mm scale bar in it. Rules (a) and (b) are
   read frame by frame while the needle is bare: the distance from each gripper's centre point to the needle
   tip is taken from both views together (3D).
3. Ink the needle tip: a mark on the pad or tray wall shows rule (c).
4. A side video of the click shows where the sheath was when it locked: rule (e).
5. After the robot lets go, pull the sheath's far end open with a 3 N spring scale: it must not reopen.
   With a ruler, check that the needle tip is at least 3 mm inside the sheath's far end. Photograph the tray
   from above.

## Layout and dimensions (metres, world frame; arm bases at (0, +/-0.305, 0.750); table top 0.7475)

| Object | Type | Material | Size | Mass | Pose |
|---|---|---|---|---|---|
| Syringe with safety needle | free articulation: syringe body, hinged sheath, sprung catch | polypropylene, steel cannula | 123 mm long overall, barrel 10.5 mm | 7.05 g (body with needle 6.4 g, sheath 0.6 g, catch 0.05 g) | origin on the barrel axis at (0.230, -0.040, 0.75275); needle toward -y; sheath open 100 deg toward -x |
| Practice injection pad | fixed box | silicone | 100 x 100 x 20 mm | (fixed; a real pad is about 0.25 kg) | centre (0.230, 0.128), on the table |
| Sharps tray | free, five boxes | polypropylene, red | 110 x 160 x 60 mm outside, 3 mm walls and floor | 0.35 kg | centre (0.130, -0.225), long side along y |

**Syringe** (syringe frame: origin on the barrel axis; y toward the plunger, needle toward -y; the open sheath
toward -x; z is the hinge axis; millimetres):

| Part | Geometry |
|---|---|
| Barrel | cylinder, diameter 10.5, y -30..+36 (two colliders, split at y -6) |
| Finger flange | 22 (x) x 2 (y) x 10.5 (z) at y +36..+38 |
| Plunger rod and thumb rest | rod diameter 6, y +38..+46; thumb rest diameter 10.5, y +46..+48 |
| Hub collar | cylinder, diameter 6.4, y -42..-30 |
| Hub nose | cylinder, diameter 4.4, y -50..-42 |
| Cannula (25 G) | 0.5 x 0.5 square rod, y -75..-50, on the axis |
| Hinge pin | axis z through (x -4, y -44) |
| Sheath (closed position) | back wall x -4.5..-3.5; two side walls z +/-(3.5..4.5), x -4.5..+4.0; all y -86..-44 |
| Catch (closed position) | nose: sphere radius 1.5 at (2.1, -69, 1.31); heel: box x 0.6..2.1, y +/-0.6 about -69, z -0.19..2.5; stem to the side wall |

| Mechanism | Value |
|---|---|
| Sheath hinge | 0 deg = closed along the needle; ready position 100 deg open; hard stops at 100 deg and 3 deg past closed. Friction 0.012 N m static, 0.010 N m moving (0.3 N at the sheath's far end): the sheath stays where it is put. |
| Catch | slides along the hinge axis, 1.2 mm travel; spring 600 N/m preloaded to 3.0 N (3.26 N at full lift). |
| Click | The needle meets the nose 8 deg before closed, on a 45 deg lead-in, lifts it 0.44 mm, and passes; the catch drops behind the needle 0.8 deg before closed. The square heel then blocks the needle: the sheath has about 4 deg of play between the heel and the stop and cannot reopen. |
| Force to click | Needle on the nose: 4.5 to 7 N across friction 0.1 to 0.35, at 25 mm from the pin. At the sheath's far end (42 mm): 2.4 to 3.8 N, nominal 3 N (the low end of the 3 to 6 N estimated for a real unit). |

**How the lock force and sheath size were estimated, and how to check them.** The maker publishes neither.
The sheath's length, width and hinge position are scaled from the maker's product photographs, using the
25 mm needle and the hub as the scale. The click force is set to about 3 N at the sheath's far end, the low
end of a 3 to 6 N band for a one-finger or thumb press. To check a real unit: measure the sheath with
calipers; press the open sheath's far end down on a kitchen or letter scale until it clicks and read the
peak (grams x 0.0098 = newtons). If the real force differs, change `CATCH_PRELOAD` in proportion.

## Real-world parts

- 3 mL hypodermic syringe with hinged-sheath safety needle, 25 G x 1 in (Terumo SurGuard 3, e.g.
  SG3-03L2525, or the SurGuard 3 needle on any 3 mL Luer-lock syringe). For trials, grind the tip blunt.
- Silicone injection practice pad, about 100 x 100 x 20 mm.
- Open-top polypropylene sharps collection tray, about 110 x 160 x 60 mm, weighted to 0.35 kg.
- A firm laminate table top.

## Simplifications

- The catch is a rigid sprung plunger with a round nose and a square heel; the real one is a moulded plastic
  hook inside the sheath. Both let the needle in and not out.
- The needle is rigid, and its collider is a 0.5 mm square rod. A real 25 G needle flexes a little as the
  catch passes.
- Sheath dimensions and the click force are estimates (see above).
- The pad is rigid. No puncture, blood or liquid is simulated.
- An open tray stands in for a lidded sharps container.
- Nurses usually shield in the air with the thumb; the surface method is the maker's third listed way and is
  the one a two-finger gripper can do with the holding hand alone.

## Files and licences

- `close_safety_needle_shield_one_handed.py`: the task (scene, scorer, scripted demo).
- `test_close_safety_needle_shield_one_handed.py`: generated scorer tests.
- `yam_parts.py`, `yam_joints.py`: our own helper modules for compound parts and passive joints.
- Everything in the scene is built from box, cylinder and sphere primitives. No third-party assets.
