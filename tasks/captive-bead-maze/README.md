# Captive wooden bead maze

Move the red wooden bead to the green end of its wire maze, leaving it there with both hands clear.

This tabletop toddler-style bead maze tests contact-guided manipulation around bends in two planes. The wooden bead has an actual through-bore and remains captive through native contact with the steel wire; there is no slider, path joint or scripted bead motion. A contact-free loose ring buzz-wire task tests a different skill.

This is an explicitly specified DIY fixture, not a measured reproduction of a named retail product. Make one 30 mm wooden craft bead with a straight 10 mm reamed bore, sand and seal its edges, mass 10 g. Bend 4 mm steel rod over 50 mm radius formers following wire-drawing.svg and wire-centreline.csv (millimetres relative to the wire origin); task.py PATH gives the matching world coordinates. The centreline begins with a 50 mm horizontal run, turns 90 degrees horizontally, then turns upward 90 degrees and forward 90 degrees in orthogonal planes, and ends with an 80 mm horizontal run. All three bends have 50 mm centreline radius. The resulting wire spans 230 x 100 x 100 mm. Secure both wire ends into wooden posts on a 300 x 220 x 12 mm plywood base clamped to the bench; use rounded caps over cut ends. Thread the bead before securing the second end. Materials cost target below $50: craft bead, rod, plywood and clamps; no specialist equipment. A drill, reamer and wood radius former are sufficient. Check bore with a 10 mm gauge, rod with callipers and bend template; ensure the bead slides by hand across all bends before the trial.

The simulation uses a 256-voxel SDF bead, exact static wire triangles and 0.1 mm contact offsets. The bore and wire sizes are fixture specifications; they are not manufacturer measurements. The bead is 30 mm in diameter, not enlarged for robot access. Gravity, friction and rigid contact carry all motion. Wood and wire elasticity are omitted.

Scoring uses final released position (highest matching level): 0.25 beyond the horizontal bend, 0.5 beyond the rising bend, 0.75 at the green end but moving, 1.0 there with both grippers clear and all 20 consecutive poses within 1 mm. A bead falling below the table or escaping the wire permanently scores zero. The exact partial-credit markers are centreline sample86 at (100.000,51.309,100.017) mm and sample146 at (100.017,100.000,151.309) mm relative to the wire origin; goal is (196,100,200) mm relative to that origin. For partial credit compare the bead centre to its nearest CSV centreline sample; both levels require all fingertips more than 45 mm from its centre. Check real trials by ruler and video: bead centre nearest the corresponding drawn centreline marker, hand clearance at least 45 mm, goal within 12 mm of its marker, and no visible motion over one second.

Files: task.py, meshkit.py, partkit.py, demo.py, task.json, wire-drawing.svg and wire-centreline.csv. Layout fixed; object_jitter=0; no random seeds.

```
python demo.py task.py outputs/demo
DEMO_VARIANT=short python demo.py task.py outputs/short
DEMO_VARIANT=held python demo.py task.py outputs/held
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Move the red wooden bead to the green end of its wire maze, leaving it there with both hands clear." --scorer task_score --max-steps 1400 --store-frames --log-dir outputs/dream
```

Run limit 1,400 steps, 70 seconds. Reset with bead centre 32 mm along the initial straight, bore along the wire. Inspect the rod and bore for damage before every run. The base is secured and all edges rounded. No library assets or third-party meshes are used. Native-contact Isaac demonstration: success true, task_score 1.0 in 794 steps. Reported worst commanded IK position error: right 48.16 mm, left 0 mm. This is a controller diagnostic, not bead placement error: the actual released bead met the scorer's goal and one-second stillness checks. Negative controls: stopping before the bends scored 0.0 in 404 steps; retaining the bead in the gripper scored 0.0 in 862 steps. One Dream-1 run scored 0.0 at the 1,400-step limit (run 20261123_043028_d2a67611). No Dream retries or score selection.
