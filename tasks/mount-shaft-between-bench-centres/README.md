# Mount a shaft between bench centres

Turned shafts, spindles and pins are checked for runout by mounting them between the two 60-degree points of a bench
centre and turning them under a dial indicator. On the common compact bench centre the left point is fixed and the
right point sits on a sprung sleeve that a hand lever draws back. The inspector holds the lever down with one hand,
sets one centre hole of the shaft on the fixed point with the other, swings the shaft into line, and eases the lever
up so the sprung point enters the other centre hole. A point that rides on the protecting chamfer or the end face
instead of its seat gives a false runout reading.

What the task tests, physically: holding a sprung lever down with one hand while the other engages a 60-degree point
in a centre hole at an angle, swings a 160 mm shaft about that point into line with a second point 2.6 mm off its
end face, and then lets the sprung point find its seat (a 10 mm protecting chamfer is the only lead-in).

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=mount_shaft_between_bench_centres.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Mount the shaft between the centres: hold the lever down, seat one end on the fixed point, swing it in line, ease the lever up." \
  --scorer task_score --max-steps 2400 --epochs 2 --store-frames --log-dir ./demo-run
```

For a model trial, replace `--policy yam_scripted` with `--policy dream`. The output of `--scorer task_score` is the
score, taken at the end of the run. Each trial allows 2,400 control steps (120 seconds at 20 Hz). There are no seeds:
the layout is fixed. The scripted demo lasts 26.6 s. `success()` requires a score of 1.0.

Offline checks (no simulator): `pytest test_mount_shaft_between_bench_centres.py test_red_team.py`.

## Scoring

All of it is read from the shaft's pose, the sleeve's and the lever's joint positions and the two gripper positions;
nothing reads a contact force.

- **In its seat:** a point's tip within 1.0 mm of the shaft's axis and 5.0 to 6.8 mm inside an end face. Seated, the
  tip is 5.9 mm in. A tip resting on the 120-degree chamfer is at least 3.3 mm off the axis and less than 1 mm in, so
  the two cases cannot be confused. Either end of the shaft may go on either point (both ends are drilled alike).
  Real trial: the shaft turns freely by hand without knocking, and a close photo shows each point inside its seat.
- **Bearing on an end:** a point's tip within 8 mm of the axis, touching the end face or inside the hole, but not in
  its seat.
- **Lever at rest:** the knob within 2 mm of its rest height.
- **Hands off:** both gripper centre points at least 30 mm from the shaft's box and from the knob.
- **Still:** over the last 1 s the shaft's centre moved less than 0.5 mm and its axis turned less than 0.5 degree.
  Turning about its own axis does not count: between centres the shaft is free to turn, which is the point of them.

| Score | End state |
|---|---|
| 1.0 | a point in its seat at each end, lever at rest, hands off and still, held for the last 1 s |
| 0.75 | a point in its seat at each end and the shaft still, but a gripper within 30 mm of the shaft or the knob, or the lever not at rest |
| 0.5 | one point in its seat and the other bearing on the other end's chamfer or face, hands off and still (a false-runout mount) |
| 0.25 | one point in its seat, otherwise (for example the free end still held) |
| 0 | anything else, including the shaft still in its V-blocks |

Sticky rules (they hold for the rest of the episode):
- **Dropped:** the shaft ever lies on the table or on the bench's top, or leaves the table: 0.
- **Forcing cap:** the sprung centre is to be drawn back by its lever. If the work ever pushes the sleeve more than
  3 mm back from where it last stood while the lever's finger is not on the sleeve's lug (the shaft rammed against
  the point), the run scores at most 0.5. A point that is let down onto a misaligned end and stops short on the
  chamfer has not been pushed back, so recovering from that (lever down, realign, ease up) still earns 1.0.

`test_red_team.py` pins these rules with timed sequences: the honest mount, ramming then mounting (0.5), stopping on
the chamfer then correcting (1.0), a point on the chamfer or end face (0.5, or 0.25 while held), a drop then put
back (0), a hand on the knob (0.75), a 0.6 mm wobble (not still), spin about the axis (still), a reversed shaft (1.0).

## The demo is open loop

It uses the known start layout, the bench's drawing and time only; it reads no object or scorer state.

- Right hand: closed fingers come down on the knob, follow its arc down to the lever's stop (36 mm) and are commanded
  12 mm further, so the stop, not the hand, sets the sleeve 10.0 mm back. After the shaft is in line the hand follows
  the knob's arc back up over 3.5 s; the sprung point enters the right centre hole near the top of the stroke, and
  the lever's return spring takes the knob the rest of the way to rest.
- Left hand: pads 40 mm apart come down round the middle of the shaft between the V-blocks and close (a 16 mm bar is
  centred by the two fingers' equal springs). It lifts the shaft, carries it over the bench turned 12 degrees about
  the fixed point (free end toward the robot), lowers it to the centre line 12 mm short of the fixed point and slides
  it along its own axis onto the point, 2 mm past the seat so the arm's give keeps the hole on the point. It then
  swings the shaft into line about the point in 1-degree steps (0.3 s each), holds it there while the lever is eased
  up, opens, lifts clear and parks.
- A gripped part rides 3 to 4 mm low in the hand, so every carried pose is commanded 4.5 mm higher.

Timing: 26.6 s of a 120 s run limit (4.5 times).

## The hardware and how to rebuild it

Millimetres; x forward from the line through the two arm bases, y to the robot's left, z up from the table top.

**Bench centre.** A compact cast-iron concentricity tester of the kind sold as Helios Preisser 0780104 (Mahr 818
family): bench 350 x 110 mm, centre height 75 mm, 200 mm between centres, about 8 to 13 kg depending on the listing.
The maker's catalogue gives a fixed left point, a right point on a sprung, retractable sleeve with about 10 mm of
free travel, and eccentric clamps on both tailstocks; dealers describe the right point as drawn back by a hand lever
and returned by its spring. Here it stands on the table along y: bench x 125 to 235, y -135 to +215, 40 tall
(modelled fixed: 8 kg of cast iron does not move under these forces). Centre line x 180, z 115. The measuring stand
is removed from its T-slot for loading, as it would be swung clear.

| Part | Geometry (mm) |
|---|---|
| tailstock heads | 64 long (y) x 46 wide (x), z 73 to 143; foot 60 wide down to the bench; a hardened V-prism on top to z 153; an eccentric clamp lever on the near side of each foot |
| left head | y +145.4 to +209.4; fixed point: 60-degree cone 14 across, tip relieved to 1 across, tip at y +126.1, on a 14 mm shank |
| right head | y -109.6 to -45.6, hollow round the sleeve's back end; the foot is set in 10 mm at the back under the lever; knurled cap on the back face |
| sprung sleeve | 14 diameter with the same 60-degree point; slides along y, 0 (fully out, tip at y -20.6) to 10.5 back; spring 4.0 N at full extension, 0.4 N/mm (estimates); a lug on its back end, on the robot's side |
| lever | pivot pin across x at (166, -81, 93); a finger rises 22 mm to the centre line and bears on the lug (0.5 mm gap at rest); the arm leaves the back of the head through a slot and ends in a black 20 mm ball knob at (166, -154.6, 103); stop at 28.5 degrees, where the finger has drawn the sleeve 10.0 mm and the knob is 36.3 mm lower; return spring 0.031 N m at rest |
| knob force | about 1.5 N when the finger meets the lug, 2.8 N at the stop (from the spring, the 22 mm finger and the 69 mm arm; estimates) |

**Shaft.** Ground C45 (AISI 1045) steel test shaft, 16.0 x 160, 0.252 kg, both ends centre-drilled DIN 332-1
B 3.15 x 6.7: a 3.15 pilot 7.0 deep, a 60-degree seat out to 6.7 across, a 120-degree protecting chamfer out to 10.0
across at the end face. Mounted it spans y +132 to -28 on the centre line; each tip bears on its seat 5.9 inside the
end face, and the sleeve sits 1.5 back from full extension, pressing about 4.6 N. With the lever on its stop the
sprung tip is 2.6 clear of the right end face, so the shaft cannot drop in square: one end goes onto the fixed point
first and the shaft swings into line.

**V-blocks.** Two black-oxide steel V-blocks, 50 wide (x) x 40 long (y) x 40 tall, 90-degree V 40 across and 20 deep,
x 55 to 105, y -18 to +22 and +82 to +122. The shaft starts in them along y, its middle at (80, +52), axis at z 31.3.

**Start.** Lever at rest, sprung point fully out, both grippers open at the ready pose.

### Sources and estimates

- Bench centre dimensions, free travel, fixed and sprung points, eccentric clamps: Helios Preisser catalogue
  "Rundlaufprüfgeräte" (0780101 to 0780107: "Rechter Reitstock mit versenkbarer, gefederter Spitze, Freihub ca.
  10 mm"); dealer listings for 0780104 (350 x 110 bench, 75 mm centre height, 200 mm between centres). The lever:
  dealer descriptions of the same family ("zieht sich bei Betätigung eines Hebels um 8 mm zurück"; "kann mittels
  Hebels nach hinten gezogen werden und wird über Federkraft wieder zurückgestellt").
- The lever's shape and the knob's position are taken from the maker's product photograph: the lever leaves the back
  of the right tailstock below the sleeve, pivots on a pin in the side of the head and ends in a ball knob about
  45 mm behind it. Its internal ratio is not published.
- Estimates, to measure on the real bench centre: the spring's preload and rate, the finger length (so the lever
  ratio and knob travel), the knob force, the head's inside, the sliding friction. Centre-hole dimensions are from
  DIN 332-1.

## Fidelity and simplifications

- Rigid bodies. The sprung centre is a slide joint with a linear spring and its own stop; the lever is a hinge with a
  return spring and its own stop. The lever does not drive the sleeve through a joint: its finger bears on the
  sleeve's lug by contact, so the lever only pulls, and the work can push the sleeve back by itself, as on the real
  tailstock.
- Each centre hole is modelled as 16 convex sectors around the hole (60-degree seat, 120-degree chamfer and pilot
  hole as flat facets); the points are convex 60-degree cones of 24 facets. The facets put a seated tip about 0.1 mm
  shallower than a round hole would.
- No dial indicator is fitted: the task ends at a correctly mounted shaft; the runout reading that follows is not
  scored.
- Friction estimates: steel on the hardened points 0.15, steel on the V-blocks 0.20.

## Reset

Hold the lever down, lift the shaft off and lay it back in its V-blocks with its middle at the mark; let the lever
up. Wipe both points and both centre holes with a clean cloth. Replace the shaft if it was dropped or a centre hole
is marked. The bench and V-blocks stay in place; the tailstock stays clamped.

No third-party assets: all geometry is primitives and meshes generated in the task's code, with the included kit
(`yam_parts.py`, `yam_joints.py`, `yam_mesh.py`).
