# Open a popped car hood: push its hidden safety-catch lever aside, lift until the struts take over

Instruction: `The hood has been popped from inside. Open it fully.`

Someone has pulled the hood release in the cabin. The hood has jumped up 3 cm at its front edge and stopped there: a
safety hook under the hood, near the middle of the front edge, has caught the striker loop on the grille
cross-member. This is the moment every driver knows: reach into the 3 cm gap, feel for the catch lever, push its tab
sideways to swing the hook off the striker, and lift the hood off the hook. Lifted to about 23 cm above its popped height at the front edge, the
gas struts take over and raise it to its full-open stop.

What makes it hard physically:
- **A hidden lever in a narrow gap.** The tab sits 1.4 cm behind the front edge, under the hood, in a gap about 3 cm
  high; it has to be found and pushed 1.5 cm sideways, not up or down.
- **A loaded hook.** The pop spring presses the hood up against the hook, so the loaded hook is stiff to swing. Pressing
  the front edge down about a centimetre first takes that load off, as people do.
- **Order and hands.** The hook has to be held clear while the hood is lifted off it (let the lever go first and the
  hook catches again), and fingers must be out from under the edge before the hood could come back down.

## Scene

Metres; origin on the table top below the midpoint of the arm bases; x away from the robot, y to its left. Heights
are above the table top.

| object | type | size | material (real part) | position |
|---|---|---|---|---|
| hood | jointed, 14 kg | 1.40 m wide, 1.01 m front edge to hinge line, 2.5 cm thick (skin and inner frame) | pressed steel, silver | hinge line at x 1.35, 0.30 up; front edge at x 0.346, 0.167 up when popped (0.137 shut) |
| grille cross-member | fixed | 7 x 140 x 12 cm, with a 10 cm wide opening in the middle (a 6 cm high block fills its bottom) | black steel, plastic trim | front face x 0.39, top 0.12 up |
| striker loop | fixed | 8 mm round bar, 4 cm wide loop | steel | in the opening at x 0.416, y -0.03, 0.136 up |
| safety hook + lever | jointed, 120 g | arm 3 cm, post 3 cm, lip 14 mm; lever 4.6 cm to a 1.6 x 1.8 cm tab | 4-5 mm steel plate, black | pivot under the hood 6 cm behind the front edge at y 0; tab 1.4 cm behind the edge, about 0.14 up |
| fenders, engine block, cowl | fixed | plain blocks; the bay under the hood is cut down to the table (14 cm deep at the engine) | painted steel, grey block | either side, under the hood, at the hinge line |

## Mechanism

- **Hinge and struts.** The hood swings 0 to 50 deg about its rear hinge line. Two gas struts and a pop-up spring give
  a net opening torque about the hinge of 2 N m up to 2.6 deg (popped, the hood presses up against the hook), then the
  struts are weaker than the weight: lifting takes about 6 N at the front edge from 3.5 to 12 deg. From 12 to 15 deg the
  strut geometry turns over, and above 15 deg (front edge about 23 cm above its popped height) the struts lift the hood on their own (5 N m) to the
  50 deg stop, damped (about 3 s).
- **Safety hook.** One steel part on a vertical (hood-normal) pivot under the hood: a 4.6 cm lever forward to the tab
  and a 3 cm arm to the side carrying a post with a 14 mm lip. The lip sits under the striker bar, so the hood cannot
  rise. A return spring (0.004 N m per deg, preloaded 3 deg towards its rest stop) holds it there. Pushing the tab towards
  the robot's left swings the lip back off the bar; past 19.5 deg (tab 1.5 cm aside) it is clear. The lever stops at 30
  deg.
- The hook holds the hood only by contact of its lip with the bar; nothing is switched by distance.

## Real rig (low cost)

- Hood: any salvaged steel hood from a compact sedan or hatchback that uses two gas struts (no prop rod), 12-16 kg,
  about 1.4 m wide, with its two hinges, its strut pair and its safety catch. The exact donor does not matter; the rig
  is accepted by the measured checks below, not by part number. If the donor's struts are missing or tired, fit an
  aftermarket hood-strut pair chosen from a strut catalogue by extended length and rated force, and adjust the lower
  mounting points until the checks pass.
- Frame: 40 x 40 x 4 mm steel angle, bolted into a rectangle carrying the hinge brackets (hinge line 30 cm above the
  table) and the cross-member. The frame is fixed to the table with four M8 bolts through the table top, or with four
  heavy C-clamps if the table cannot be drilled; it must not move when the hood slams onto its stop.
- Cross-member: 70 x 120 mm steel or plywood box section across the frame under the front edge, top 12 cm up, with a
  10 cm opening in the middle; striker loop of 8 mm round steel bar welded or bolted in the opening.
- Safety hook: donor catches differ by model (tab position, which way it swings, how far), so the donor's own catch
  is removed and replaced by the one drawn below (also as `catch_drawing.svg`, top and side views), cut from 5 mm steel
  plate and welded from three pieces: plate (lever + arm), tab block, post with its lip.
- Pop-up spring: the donor's latch spring, or a 30 mm compression spring under the front edge.
- Open stop: a steel limit strap (or chain) from the frame to the hood's inner frame, length set so the hood stops at
  50 deg, with a rubber bump pad where the hood meets it.
- Guarding: rubber U-channel edge trim on the hood's front and side edges and on the cross-member top; rubber bump
  pads on the frame under the hood edge where it lands when shut. Between trials and for any work under the hood, a
  steel prop rod (12 mm tube, or the donor's if it has one) is clipped in so the hood cannot fall if a strut fails. Keep people
  out of the area under the hood while a trial runs.
- Parts: about 150 USD with a salvaged hood.

Catch drawing (mm). Frame: origin on the pivot axis at the plate's mid-thickness, which sits 6 mm below the hood's
underside; F towards the hood's front edge, R towards the robot's right, D down.

| piece | F | R | D | notes |
|---|---|---|---|---|
| lever bar | -3 to 49 | -5 to 5 | -2.5 to 2.5 | 5 mm plate, 10 mm wide |
| tab | 41 to 51 | -8 to 8 | 1 to 19 | 10 x 16 x 18 block under the lever's end; its centre 1.4 cm behind the front edge |
| arm | -6 to 6 | -3 to 33 | -2.5 to 2.5 | same plate as the lever, 12 mm wide |
| post | -4 to 4 | 26 to 34 | 0 to 30 | 8 x 8 square bar |
| lip | -12 to 2 | 26 to 34 | 26 to 30 | 14 x 8 x 4, pointing rearward from the post's foot |
| pivot | 0 | 0 | - | 6.2 mm hole, M6 shoulder bolt square to the hood skin, through a 40 x 30 x 3 mm bracket riveted under the skin |
| stops | - | - | - | two 4 mm pins in the bracket: rest stop at 0 deg, open stop at 30 deg (tab to the robot's left) |
| spring | - | - | - | torsion spring about the bolt, 0.004 N m per deg, preloaded 3 deg against the rest stop |
| striker bar | -6 (centre) | 10 to 50 (4 cm wide loop) | 21.5 (centre) | 8 mm round steel, along R, 0.5 mm above the lip's top when popped |

Total about 120 g. Pushing the tab towards the robot's left by 19.5 deg (1.5 cm at the tab) swings the lip off the
bar.

Acceptance checks (luggage scale with a hook, hooked on the front edge at its middle, pulled square to the hood):
- Popped: released from shut, the front edge rises to 3 +/- 0.5 cm and rests on the hook.
- Lift: between 6 and 21 cm above shut (3.5 to 12 deg), the force to raise the edge slowly is 4 to 8 N.
- Self-rise: released 26 cm or more above shut (15 deg), the hood rises unaided to the stop at 50 +/- 1 deg;
  holding it down anywhere between 15 deg and
  the stop takes 3 to 7 N.
- Hook: popped, the tab hangs 1.4 cm behind the front edge at the middle, about 14 cm above the table, in a gap of
  3 +/- 0.5 cm; pushed towards the robot's left (sideways, not up or down) with the edge pressed down about a
  centimetre, it swings 1.5 cm aside under 1 to 3 N, the lip comes off the striker bar, and the tab springs back to its
  stop when let go.

## Run a trial

Requires yam-isaac-lab-arena and Isaac Lab-Arena 0.3.1; `task.py` needs nothing else in this folder.

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
  --instruction "The hood has been popped from inside. Open it fully."
```

Run limit 1,600 steps (80 s at 20 Hz). The layout is fixed (`object_jitter=0`); nothing is randomised.

## Scoring

Score 0 if the hood, coming back down after being lifted 10 cm, sinks below 7 cm above its popped height (5.7 deg)
while a gripper point is under its front 15 cm, above the cross-member top (fingers caught under a sinking hood).
"Coming down" means its angle is falling faster than 0.2 deg/s (front edge about 3.5 mm/s). Once the hood has settled
back down (still for 1 s, within 3 cm of its popped height), reaching under it again is a fresh attempt and is not
penalised until it is next lifted 10 cm.

Otherwise: 0.25 the lever swung at least 19.5 deg (hook clear of the striker) at some point; 0.5 the hood's front edge
lifted at least 10 cm above its popped height (7.4 deg) at some point; 1.0 the hood at its full-open stop (within 1 deg
of 50 deg) at the end, with every gripper point at least 15 cm from the hood, the hook and the cross-member (each
measured to its finite box: the cross-member's 7 x 140 x 12 cm block, front face to rear face), and every gripper
point moving slower than 2 cm/s, held for 0.5 s.

In a real trial: the video shows the tab pushed aside and the hood lifting off the hook, and whether a falling hood
came down on a gripper; at the end a photo shows the
hood up on its stop and both grippers away from it.

## Notes

- Lifting without the lever: the hook holds; the hood does not move (score 0).
- Pushing the lever alone frees the hook: the pop spring lifts the front edge a little under 2 cm more and the hood stays there
  (0.25).
- Letting go of the edge below 15 deg: the hood sinks back slowly towards its popped height (0.5 if it had been lifted 10
  cm).
- Pressing the front edge down about a centimetre before pushing the tab takes the pop spring's load off the hook; the
  demo does this.
- One gripper could push the tab and then lift by the edge; the demo uses one hand on the lever and the other on the
  edge, as people usually do.
