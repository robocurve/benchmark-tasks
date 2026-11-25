# Serve a ping-pong ball: toss it up, hit it on the way down, one bounce on each side of the net

Instruction: `Serve the ping-pong ball so it bounces once on your side and once past the net, then put the paddle down.`

Kitchen-table ping-pong is a common home game: a clamp-on retractable net set (for example Franklin Sports
Table Tennis To Go, which clamps to any table) turns a dining or kitchen table into a small ping-pong table. Every
rally starts with a serve, and the serve is a true two-hand act with distinct roles: the free hand tosses the ball
straight up out of its open fingers, and the paddle hand strikes it on the way down so it bounces once on the server's
half, passes over the net and bounces on the far half. The hard part is visible in a 10 s clip: a 2.7 g ball in free
flight, a paddle swing timed and aimed to it, and a ball that must land in two places in a row.

## Requirements & Assets

- `yam-isaac-lab-arena` and `Isaac Lab-Arena 0.3.1` with `inspect-robots`.
- Third-party assets: none. All geometry is generated from USD primitives in `task.py`.

## Run a Trial

```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 600 \
  --instruction "Serve the ping-pong ball so it bounces once on your side and once past the net, then put the paddle down."
```

Run limit: `600` steps (`30 s` at `20 Hz`). The layout is deterministic.

## Scene (table frame: origin on the table top midway between the arm bases (0, +-30.5 cm); x forward, y left, z up)

| Object | Real product / build | Dimensions & material | Pose (cm) |
|---|---|---|---|
| Table | The station's oak table used as the play table | `110 (x) x 130 (y) cm` top, `74.75 cm` above the floor | top spans `x -19..91`, `y -65..65`, `z 0` |
| Net | Clamp-on retractable net set (Franklin Sports Table Tennis To Go type), net raised to the regulation `15.25 cm` | Navy mesh (`4 mm` collision) with `12 mm` white top tape; `2 x 2 cm` posts on clamps outside each side edge | across the table at `x 36.0` (the middle), full width `y -66..66`, height `0..15.25` |
| Centre line | White tape | `3 mm` wide, `1 mm` high | along `y 0` |
| Paddle | Standard recreational paddle (e.g. STIGA Pro Carbon+ / Franklin set paddle size) | Blade `15.7 x 15.0 cm` ellipse, `10 mm` thick incl. red and black rubbers; handle `10.0 x 3.2 x 2.6 cm` wood; `0.17 kg` | lying flat, handle centre `(28.0, -25.0)`, handle along `+y`, blade toward `+y` (blade centre `(28.0, -11.65)`), red side up |
| Ball | ITTF-approved 40+ ball (e.g. Nittaku / STIGA 3-star) | `40 mm`, `2.7 g` white | resting in the cap at `(24.0, 22.0)`, centre `z 2.52` |
| Ball cap | Upturned plastic bottle cap (38 mm closure) | `30 mm` bore, `3 mm` wall, `12 mm` tall, blue | centred `(24.0, 22.0)`, fixed (taped) |

## Scoring (every rule is read from the ball's state at every 240 Hz physics step; whole-trial score at the end)

- **Score 0** if the ball is struck by the paddle while still touching the tossing hand, if it touches the tossing hand
  again more than `0.15 s` after first leaving the fingers and after rising `3 cm` above the release point (touches
  within that window are the toss stroke itself; the release point is the last touch), if after the toss it touches anything other
  than the paddle (table, net, cap, the other arm) before the strike, or if it is struck a second time.
- **0.25 (toss and strike)**: the ball left the left hand's fingers (last PhysX contact with the left gripper), rose at
  least `16 cm` above that point (ITTF Law 2.6.2 toss height), and was then struck by the paddle while falling
  (vertical velocity < 0 just before the contact) with the blade moving into the ball at `>= 0.2 m/s` (paddle velocity
  over the last 6 physics steps along the face normal), so a ball that just drops onto a still paddle is not a strike.
  A drop without a toss, or a ball hit while rising, gets 0.
- **0.5**: plus the first contact after the strike is a bounce on the server's half (table top, `x < 36.0`).
- **0.75**: plus the next contact is a bounce on the far half (`x 36.0..91.0`) - so the ball passed over the net
  without touching it (a net touch or any other contact in between ends the credit at 0.5).
- **1.0**: plus, at the end, the paddle lies flat on the table (face within `10 deg` of horizontal, its handle centre
  `<= 3 cm` above the top, inside the table edges, still `<= 1 cm` over the last `2 s`), not held, both jaws open (`>= 20 mm` finger opening), every gripper point `>= 10 cm` from the
  paddle, and both arms still (`<= 1 cm` over the last `2 s`).

How the events are read, per 240 Hz physics step: a contact is a velocity change that gravity and air drag don't explain.
With the ball within `12 mm` of the blade a change of more than `0.1 m/s` is paddle contact (consecutive contact steps are
one strike; a new strike needs `0.1 s` without blade contact). Anywhere else a change of more than `0.4 m/s` is a
contact: a table bounce when the ball's bottom is within `6 mm` of the table top and its vertical velocity turns from
down to up, otherwise "something else" (net, arm, cap).

Real-trial check: a phone video at 60 fps from the side shows the toss height against a ruler taped to the net post,
the paddle contact on the way down, and the two bounces either side of the net; the end of the video shows the paddle
lying on the table and the hands clear.

## Sim fidelity

- Ball: rigid sphere `40 mm`, `2.7 g`, restitution `0.89` against the table (ITTF ball bounce test: dropped from
  `30.5 cm`, rebounds `23-26 cm`, e `0.87-0.92`), friction `0.25`, linear damping `0.4/s` as a stand-in for air drag
  (drag on a 40 mm ball at 2-3 m/s is about 1 m/s^2), continuous collision detection on.
- Paddle: rigid, true size and mass; rubber restitution is taken as the ball's (`0.89`, "max" combine), which is
  higher than a real sponge rubber (~0.8); friction `0.9`.
- Grip (sim stand-in for pad friction): Grasp model (sim stand-in for pad friction; no rigid weld): a 6-axis spring-damper joint between gripper and paddle handle engages only when both finger pads touch the paddle handle (PhysX contact on each pad) and the jaws are commanded closed. It is anchored at the current relative pose, so nothing snaps. Each axis is force-limited to what two rubber pads can transmit: 50 N pull (2 pads x mu 0.5 x 50 N squeeze), 0.50 N m twist about the pad normal, 1.50 N m tipping. Twist cap = pull cap x 10 mm mean pad-patch radius; tipping cap = 2 x 50 N x 15 mm half pad length. When the load needed exceeds a cap, the paddle handle slips in the jaws (the anchor moves) and the slip is logged. It releases when the jaws are commanded open or both-pad contact is lost for 5 control steps. Sources: YAM gripper effort limit 50 N per finger (yam_arena GRIPPER effort, i2rt linear_4310 actuator); rubber pad friction mu 0.5; pad patch ~20 x 30 mm.
- The ball is held by the left fingers with ordinary contact friction only (no grip assist, no joint): the hand is
  turned so the jaws close vertically, the lower finger carries the ball like a palm, and the toss is plain contact.
- The net is rigid (a real net gives a little).

## Demo Video & Shortcut Checks

A scripted demo (reads object poses and the ball's velocity from simulator state; all motions are physical joint
commands, no teleports) picks up the paddle by its handle with the right hand and the ball from its cap with the left.
The left hand turns so its jaws close vertically, with the lower finger under the ball like a palm, and the right hand
holds the paddle face forward, nearly upright, behind and below where the ball will fall. The left hand makes a fast
upward stroke and opens on the way up, so the ball leaves it and rises about 23 cm, then pulls away. The demo reads the
ball's flight, predicts where it will fall, and pushes the paddle face into it on the way down (it reads how the
paddle sits in the hand and sets the blade height from that). The ball bounces on the
near half (x ~28 cm), clears the net and bounces on the far half (x ~57 cm). The paddle pulls back at once, is laid flat
on the table, and both arms open and park. The demo scores `1.0`.

Shortcut checks, each a full 600-step run, all below `1.0`:

- Arms idle: `0.0`.
- Left arm only (toss, the paddle is never swung): `0.0`. The ball falls onto the still paddle and arm in the ready
  pose: no moving-blade strike, and it touched something other than a striking paddle first.
- Right arm only (the ball stays in its cap, the paddle swings through the hit point): `0.0`.
- Open the fingers without a toss (ball just dropped), then swing: `0.0`. The ball reaches the table before the paddle.
- Swing at the ball while it is still in the fingers: `0.0`.
- Full serve but keep holding the paddle at the end: `0.75`.

## Sources

- ITTF Handbook, Laws of Table Tennis: 2.1 table (`274 x 152.5 cm`, `76 cm` high), 2.2 net (`15.25 cm`), 2.3 ball
  (`40 mm`, `2.7 g`), 2.6.2 service toss (`at least 16 cm` after leaving the palm).
- ITTF Technical Leaflet T3 (ball): bounce `23-26 cm` from `30.5 cm` onto a steel block.
- Franklin Sports Table Tennis To Go set (retractable net that clamps to most tables, 2 paddles, 3 balls).
