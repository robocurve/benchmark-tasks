# Choose the tweezers that fit a carrier's tab slot and peel a display module's protective film

Dream-1 scored **0.0 in 2,400 steps** (step limit). The recorded scripted demo scored **1.0 in
1,298 steps**, and the same demo scored 1.0 on three consecutive simulator resets. All runs use
seed 0 and zero object jitter. The demo path run with the paddle-tip tweezers and the nine
gripper-only baselines release no film, and the demo without the holding arm drags the carrier
295 mm and scores 0.

The loose 250 g carrier holds a display module and 66 × 130 mm protective film.
A recessed starter tab has an 8 mm grip surface and a 5.6 mm central entry notch. It hangs
in a 6 mm slot, narrower than the robot's closed fingertips (7.9 mm), so only a tool reaches it.
Two passive 120 mm tweezer tools stand side by side: a fine-tip pair (2.4 mm blades) that
passes the notch, and a paddle-tip pair (7 mm blades) whose blades land on the tab's side
bridges. The robot picks the right pair, grasps the tab, removes the film while the other
arm steadies the carrier, places the film outside the carrier, and returns the tool. The carrier and
starter-tab geometry are benchmark-designed fixtures, not an industry standard.

`fixture_geometry.py` defines every carrier and tab component. `tool_geometry.py`
and `toolkit.py` define both stands and both tools. `motion.py` contains the validated
bimanual trajectory (68 s planned; the trial ends on success at 65 s). All quantities use metres, kilograms and seconds.

## Environment and commands

Install [yam-arena](https://github.com/kairosiann/yam-isaac-lab-arena) at
`bd62771e60320433893bd1d2df0fc79b5ddcd6d1` using its setup instructions and source
`yam.env`. This provides Isaac Sim 6.1, Isaac Lab 3.0 and Inspect Robots. Simulation
requires an RTX GPU. `probe.py --mode direct`, `validate.sh` and `audit_geometry.py` also need
trimesh and SciPy (tested with trimesh 4.11.1 and SciPy 1.17.0; `pip install trimesh scipy`).
Use zero object jitter: independent jitter would break the connected film geometry.

Run from the repository root:

```bash
python -m pytest tasks/display-film-tweezers/tests -q
python tasks/display-film-tweezers/probe.py --mode hold --steps 200 --out /tmp/display-hold
python tasks/display-film-tweezers/probe.py --mode direct --scenario all --steps 380 --out /tmp/display-direct
python tasks/display-film-tweezers/probe.py --mode decoy --steps 800 --out /tmp/display-decoy
python tasks/display-film-tweezers/probe.py --mode demo --measure-load --steps 1400 --epochs 3 --out /tmp/display-demo
bash tasks/display-film-tweezers/validate.sh /path/to/python /tmp/display-validation   # tests, all baselines and the demo, audited
python tasks/display-film-tweezers/audit_geometry.py --reference-trace /tmp/display-demo/0-trace.json --out /tmp/display-geometry
bash tasks/display-film-tweezers/record.sh /path/to/python yam_scripted /tmp/display-scripted
FILM_DREAM_ROWS=16 bash tasks/display-film-tweezers/record.sh /path/to/python dream /tmp/display-dream
```

Recording uses the instruction in `task.json`, seed 0, 20 Hz control and a 2,400-step
limit (120 simulated seconds). The scripted demo finishes in 1,298 steps (65 simulated
seconds). On an RTX 4070 Ti SUPER a demo trial takes about 7 minutes of wall time and a
2,400-step Dream-1 trial about 11 minutes, including Isaac Sim start-up. The Dream-1 run used
`FILM_DREAM_ROWS=16` with `record.sh` (16 rows of each 48-row chunk per request): at about
0.3 s per step the default 48 rows outlast the gateway's 8 s idle limit, and every request then
reopened the session. On an L40S the default is fine. Diagnostic JSON preserves source
hashes, measured root poses, measured joints, scores and camera frames. Optional
contact sampling records transmitted tab load at every 240 Hz physics step.

## Physics and scoring

The film is a rigid tab and twenty-six hinged 5 mm strips. The first three (15 mm), next to
the tab, are a non-adhesive starter zone; the other twenty-three (115 mm) are bonded. The
starter zone is long enough for the tab to rise out of its slot before the first bond is loaded,
so the first release happens during the peel, not the vertical exit. The
current peel-front bond releases at 60 degrees of upward opening. Compression or in-plane shear alone
cannot release a bond. The 0.025 N m/rad adhesive springs are an approximation,
not a fit to measured PE peel forces. Film thickness is inflated to 1 mm and the
tab's mass to 3 g for contact stability: at its real sub-gram mass the 7.5 g
tweezer halves pushed through it. Wrinkles, stretch, tearing, scratches and ESD are not modeled.

Each tweezer tool comprises two 7.5 g rigid halves connected by a 150 N/m opening spring;
the fine-tip pair has 2.4 x 0.8 mm blades and the paddle-tip pair 7 x 0.8 mm blades.
Robot contact alone moves and closes a tool; there is no tool attachment to the robot.
The parallel jaw motion approximates tweezer flexure. Solver iterations are 128/16.
During development a tool-only bench test held a fixed 1 mm coupon in the tweezers at 3 N
and 6 N, vertically and at 170 degrees, with under 2 mm slip in the hand; that test is not part
of this folder. In the integrated demo the peak tab contact load while peeling is 3.0–3.3 N
and blade-tab penetration stays under 0.1 mm (`probe.py --measure-load`, `audit_scene.py`).

Carrier excursions over 30 mm horizontally, 5 mm vertically or 10 degrees yaw/tilt
invalidate the whole trial. Film seam separation over 3 mm also invalidates it.
Consecutive release of 6/12/18 bonded strips (30/60/90 mm) earns 0.25/0.5/0.75. Full
credit additionally requires all 23 released, the film supported by the table and entirely 2 mm outside
the carrier, every piece within 30 mm of the table, no piece moving more than 2 mm or turning more
than 5 degrees during the hold, and both tools upright in their own stands. Film rest is judged from
poses, as two photos a second apart would judge it: the 0.3 g strips of a folded sheet can report
several mm/s and rad/s of solver velocity while their poses stay fixed. Return requires, for each tool, a
tip-centre horizontal error ≤4.5 mm (inside its stand's cavity), vertical error ≤5 mm, tilt ≤5 degrees,
opening 6–10 mm, and speeds below 10 mm/s and 0.5 rad/s. Both measured TCPs must be at least 80 mm from all task objects. These
completion conditions must remain true for a full second.

Direct baselines cover two-finger and single-finger entry, angled entry, side and
far film edges, and carrier lifting/pushing. The decoy baseline runs the demo path with the
paddle-tip pair: its blades stop on the tab's side bridges at film height, the tab stays in
the slot and no film is released. A failed scripted baseline is evidence
about that attempt, not proof that every possible strategy is blocked. Reset uses
the same fixed layout and restores all adhesive joints; physical trials need fresh
film and the same starter-tab folds/cuts.

Display film removal with tweezers is illustrated in the manufacturer's
[assembly process](https://www.palmtech.com.tw/technical/assembly). The benchmark
focuses on tool selection, contact tool manipulation and supported peeling, not optical quality.
