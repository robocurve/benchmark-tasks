# Fit a server's cable management arm to its rails and support bar

Install the cable management arm on the server's rails and secure it on its support bar, then let go.

## Why it matters

Bringing AI compute online means racking thousands of servers, and every slide-out rack server gets a cable
management arm (CMA): a folding two-basket arm that carries the server's power and network cables so they follow the
server out on its rails for service and fold back behind it. Fitting one is part of racking each server and is done by
hand. A CMA left with a clip unlatched, or with its free end off the support bar, lets the cables sag below the server
and catch on the server beneath it (the maker's guide: "Ensure that the cables do not sag below the U space so they
will not get caught on the lower servers").

This is not a pick-and-place: the CMA is a loose 0.85 kg (estimate), 440 mm long, two-link part whose outer arm swings
freely on its hinge. Its two clips are on different links, at the end away from the hinge, and each must be pushed
home on its own tab until a latch tooth springs up behind it. Carrying it level and square to the tabs takes both
hands, one near each end, and the last 30 mm of the push folds the inner latch tooth. Once the inner clip is on, the
outer arm is swung shut about the hinge until its clip snaps onto its tab; a hand still gripping the inner basket has a
finger between the two baskets, which stops the outer arm a few degrees (up to 25 mm at the clip) short, so the grip
has to be let go first. The guide has the support bar's stop bracket turned open before the CMA is placed and shut afterwards;
here it starts shut, standing where the CMA's hinge end must go, so the order matters: open it, fit the CMA, shut it.

## Real hardware and sources

- Rails and CMA: Lenovo ThinkSystem Toolless Slide Rail Kit with 1U CMA, part 7M27A05701 (listed in Lenovo Press
  LP1838, "ThinkSystem and ThinkEdge Rail Kit Reference",
  https://lenovopress.lenovo.com/lp1838-thinksystem-and-thinkedge-rail-kit-reference). The same CMA is sold as the
  ThinkSystem 1U CMA Upgrade Kit for Toolless Slide Rail, 7M27A05699.
- Procedure: Lenovo "CMA Installation Guide", P/N SP47A24965, Third Edition (November 2018),
  https://pubs.lenovo.com/sr630/cma_installation_guide.pdf, Part II: the support bar's pin goes in the rear slot of
  one rail and its bracket onto the opposite rail's outer bracket; the CMA's inner mounting bracket connects to the
  rail's inner bracket and its outer mounting bracket to the outer bracket; then "Press down the stop bracket on the
  support bar and rotate it to the open position", "Place the CMA onto the support bar and then rotate the stop bracket
  back into the closed position". The guide recommends the left side viewed from the rear, as here.
- Server: any 1U server the kit supports (for example a ThinkSystem SR630), 434.4 mm wide and 43 mm high. It stays
  fully pushed in and is never touched.
- Rack: a 19-inch (EIA-310) four-post rack; only its two rear posts and the rails' rear ends are in reach. In a real
  trial the rack's lower units are below the bench line: the server's underside is 58 mm above the bench top.

Not published by the maker, so estimated and stated as such: the CMA's mass (0.85 kg in all: 0.45 kg inner basket with
its clip and hinge, 0.40 kg outer basket; the 7M27A05699 kit ships at 3.52 lb, about 1.6 kg, with its box, six
brackets, twelve screws and the support bar), the baskets' section (45 x 40 mm), the clips' and tabs' shapes (a 30 mm
slot round a 20 x 30 x 2 mm tab, 0.6 mm (inner clip) or 2 mm (outer clip) clearance each side and 0.5 mm above and
below, its last 6 mm flared open by 2.5 to 3 mm as a lead-in), the gap behind the server for plug bodies (43 mm),
the latch (a 2 mm tooth standing 2.5 mm proud of the tab on a 0.005 N m/rad spring), the hinge's drag (1 N m s/rad) and the stop bracket's
detent (0.1 N m/rad, carried open past 0.8 rad). fixture/geometry.json and fixture/layout.svg give every box of the
simulated scene, generated from task.py by fixture/drawing.py.

## Run and score

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Install the cable management arm on the server's rails and secure it on its support bar, then let go." \
  --scorer task_score --max-steps 1200 --epochs 1 --seed 0 --store-frames --log-dir demo
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the cable management arm on the server's rails and secure it on its support bar, then let go." \
  --scorer task_score --max-steps 1200 --epochs 1 --seed 0 --store-frames --log-dir dream
```

One published layout (the screenshot). A trial takes about 45 s of simulated time and ends at success. Run with `object_jitter=0`. `python near_misses.py` (with `yam_arena` importable) runs the scoring
controls. `TASK_DEMO=naive` plays a variant that leaves the stop bracket alone and pushes the CMA straight in.

## Scoring, exactly as the code does it

`score()` is computed in a step callback that keeps a record for the trial, reset by the simulator's reset hook.

- Whole trial, 0 and stays 0: the CMA's lowest point more than 10 mm below the bench top (off the bench) at any step;
  any state (the CMA, its hinge, the latch teeth, the stop bracket, either gripper) unreadable at any step. How the
  CMA is held or turned on the way is not scored.
- Let go: every point of both grippers and all four fingertip links, sampled every 3 mm over every face of their
  collision hulls, more than 30 mm from every box of the CMA (both baskets, both clips, the hinge) and of the stop
  bracket's plate.
- Still: the CMA's five reference points, the two teeth and the stop bracket (turned at 40 mm) move less than 2 mm
  over the hold; a tier must hold, let go and still, for 1.0 s, and the score is the tier still held at the end.
- Clip latched: the clip's front-centre point back from fully home by -1 to 3.5 mm along the tab, within 2.5 mm
  sideways and 2 mm in height of the tab's line, its basket within 3 degrees of square in yaw and 3 degrees in tilt,
  and that tab's tooth turned back less than 0.25 rad (standing in the window). The inner clip is checked against the
  inner tab and the outer clip against the outer tab only, so a clip on the wrong tab is not latched.
- Resting on the bar: both baskets' undersides at their hinge ends between 1 mm below and 1.5 mm above the support
  bar's top.
- Stop bracket shut: its hinge within 0.15 rad of shut.
- Tiers: 1.0 both latched, resting, shut; 2/3 both latched and resting; 1/3 one latched (or both latched but the
  hinge end not resting on the bar); otherwise 0. `success()` is `score() == 1`.

### What scores low, and why

- Doing nothing, or carrying the CMA to the rack and setting it down: 0.
- Pushing the CMA in without opening the stop bracket: the bracket stands where the CMA's hinge end must go, so the
  inner basket stops against it with both clips short of their tabs (0). Forcing it twisted the CMA out of the hands
  in the first development trace and dropped it on the bench.
- Pushing the CMA in with its outer arm left swung open: only the inner clip latches (1/3); the outer clip is about
  110 mm short of its tab.
- Swinging the outer arm shut while still gripping the inner basket: the gripping finger sits between the baskets and
  stops the outer arm short (1 to 3.5 degrees in development traces), so at most the inner clip latches (1/3).
- Holding the CMA turned on its inner clip (for example by pushing its hinge end on after the clip is home): the outer
  clip comes in 2 to 4 mm off its tab's line and stops short (seen in development traces).
- Both clips on but the stop bracket left open: 2/3.
- Clips pressed against the tabs but not pushed past the teeth (teeth folded): not latched.
- Anything still in a hand, or a finger within 30 mm, or still moving: no credit.

`near_misses.py` checks every tier and its boundaries (3.5 mm back, 2.4 and 3 mm sideways, 1.5 mm high, a 3 degree
turn, a tooth at 0.30 rad, the stop bracket at 0.15 and 0.16 rad, a hinge end 3 mm off the bar, the wrong tab), the
1 s hold, stillness, the 29.9 and 30.1 mm let-go boundary, a face of a gripper 20 mm over the CMA with every corner
more than 30 mm away, the whole-trial rule, a CMA tilted 50 degrees on the way (not penalised) and unreadable grippers.

## Closest board tasks

- A blind headrail hung in two gated brackets (TK-BFJD6N, accepted) and a blanking panel fitted at a named rack unit
  (accepted): one rigid part pushed into two fixed catches. Here the part is a loose two-link arm: its two clips are on
  different links, the second link swings freely on the hinge and is brought to its tab along an arc after the first
  clip is on, and the hand that carried it has to get out from between the links first.
- A robot axis's cable carrier folded round and clicked into its carriage bracket (pending) and a cable carrier loaded
  and its crossbars snapped shut (accepted): a chain of many identical links on a robot axis, not a two-arm assembly
  fitted to a server rack's rails with a support bar and a stop bracket that set the order.
- A server bezel hooked at one end and swung shut until it latches, and a server's tool-less rail hung in a rack (both
  closed as overlapping): one rigid part. Here the rails are already in and the part is articulated.

## Fidelity

The rack rear, server, rails, support bar and CMA are at true size. The CMA's internal shapes, its mass, its hinge's
drag (1 N m s/rad of damping, with 0.05 N m of joint friction) and the clips' fit are estimates sized on the kit. Each
clip's latch tooth is modelled on its rail tab and the clip has a window it springs into; on the real kit the sprung
catch and its ring release are in the clip, so the same engagement is on the other half of the joint. The stop
bracket's press-down lock is a two-position detent: past 0.8 rad it is carried fully open, short of it fully shut.
No cables are fitted, as in the maker's sequence (the CMA goes on before the cables). The server is fixed and never
moved. The units above the server are empty, as they are while a rack is being filled: rack makers' safety
instructions have racks loaded from the bottom up, heaviest items first, so a server's CMA is fitted before the unit
above it is filled. There is one published
layout.

## Real trial

A short four-post 19-inch rack, or two rear posts bolted to the bench at the spacing in fixture/geometry.json; the
Lenovo kit 7M27A05701 (rails, support bar, 1U CMA); and in the rails any 1U server the kit supports. The server is
never powered or moved, so a decommissioned unit is enough. Mount the rails so the server's underside is 58 mm above
the bench, install the support bar as the guide's Part II steps 2 and 3 describe, and turn its stop bracket shut. Lay
the CMA on the bench at its start position. Read the score from the video and a photo with a ruler, as task.json
says: in the stock kit the catch is in the CMA's clip, so a clip is latched when it has clicked home, its front is
within 3.5 mm of its bracket's stop and it stays there when let go.

## Results

On the submitted source, in Isaac Lab-Arena with yam_scripted, seed 0, 20 November 2026: the scripted demo scored 1.0.
It turns the stop bracket open with the right hand, takes the inner basket in both hands, lifts the CMA level, pushes
the inner clip home until its tooth springs up, lets go with both hands, swings the outer arm shut with the left hand
until the outer clip snaps on, swings the stop bracket shut with the right hand, and parks both arms; the end state
held, let go and still, for 1 s. It plays fixed waypoints computed from the published start layout; it reads no
simulator state. Dream-1, same instruction, same seed: 0.0, running to the 1,200-step limit without a refusal.

Development traces on earlier versions of this scene (not the submitted source) showed the failure modes listed under
"What scores low": pushing the CMA against the shut stop bracket wrenched it out of the hands; with a hand still
gripping the inner basket the outer arm stopped 1 to 3.5 degrees short of its tab in five traces, leaving at most
the inner clip latched. No physical robot run has been made; the masses, latch springs, hinge drag and fits are the
estimates listed above.

## Licence

Code MIT; the scene geometry and fixture drawings CC BY 4.0. No third-party mesh, photo or manual is redistributed;
the maker's guide is cited by its public URL. The arms, station and simulator are the program's own.

