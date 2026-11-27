# Tabletop rebound rally

Use a held paddle to complete three consecutive ball returns against a tabletop rebound board. One completed return is a held-blade strike, board-front contact, exactly one table bounce, then the next held-blade strike. Four paddle strikes, three board contacts and three intervening table bounces earn full credit. The initial feeder bounce starts the drill and is not a scored return.

Category: **Societal relatability**. This is solo recreational table-tennis practice: use a paddle to send the ball to a return board and keep the next table bounce in play. It is a compact practice drill on the supplied station, not a regulation-table match. A commercial rebounding-board construction and published paddle mass anchor the independently dimensioned apparatus. Physical build qualification and benchmark acceptance are separate from simulator feasibility.

## Physical setup

The original YAM station table (1.10 × 1.30 m, top Z=0.7475 m), arm bases, room and three calibrated 640×360 cameras are retained. All coordinates are metres in its world frame. The fixed board is centred at (0.750, 0.100, 0.8975), with a 17 mm X × 600 mm Y × 300 mm Z assembly envelope: 15 mm MDF and 2 mm rubber. The panel/face mass estimate is 2.25 kg from declared densities. Two aluminum feet, each 160 × 40 × 30 mm, are centred at (0.800, −0.140, 0.7625) and (0.800, 0.340, 0.7625). The physical design uses two M6 table clamps; their anchor effect is represented by fixed supports/board, with no clamp-thread simulation. The board is one rigid collision envelope; rubber is represented through contact properties rather than a separate compliant layer.

The paddle is a 170 g approximation with a Ø156 × 7 mm wood head, two Ø154 × 3 mm rubber faces, and a 110 × 26 × 26 mm wood handle. Its handle centre starts at (0.320, 0.120, 0.910), blade centre 130 mm along +Y, with face normal +X. It is physically held in the left jaws; no attachment constraint is used. The right arm is free in the standard ready pose. A Ø40 mm, 2.7 g plastic ball starts at (0.680, 0.250, 1.000). At t=2.000 s a feeder releases it with linear velocity (−1.2, 0, −0.5) m/s and zero spin. The feeder is a specified initial condition, not a scored robot skill. After release, the ball moves only through native gravity and contacts. Reset restores this feed and the initial paddle/joint poses. Nothing is consumed.

See [physical design](PHYSICAL_DESIGN.md), [dimension drawing](physical_design.svg), [mass/inertia ledger](physical_design.json), and [asset provenance](ASSETS.md). The paddle uses the upper end of STIGA's published 165±5 g range (its description also says 170 g); the geometry and material densities are explicit build approximations. The board uses a commercial 15 mm MDF construction reference, without interpreting its ambiguous published stand weight/dimensions as panel measurements. Ball dimensions/mass follow [ITTF Laws 2.3](https://documents.ittf.sport/sites/default/files/public/2026-02/2026_Statutes_v1_consolidated_clean.pdf).

## Scoring

Final score is completed returns / 3, capped at one: 0, 1/3, 2/3 or 1. A miss, double bounce, side/back-board contact, wrong contact sequence or ball leaving bounds ends the drill and preserves already completed returns. Ball below table−50 mm, X<−100 mm or |Y|>620 mm is out. Table contact must have ball-centre Z within 8 mm of table+20 mm. Board contact must be on the front, with −0.2≤Y≤0.4 and ball-centre height table+15…285 mm (5 mm edge tolerance).

Only the blade or its rubber faces count as a strike. Both gripping fingertips must have native paddle-contact reports within two physics ticks of that strike. An unheld/handle strike or any robot/ball contact erases all credit and terminates. Different surfaces entered simultaneously end the rally; invalid contacts have priority. Terminal outcomes are absorbing, so subsequent contacts cannot repair a miss or add post-success events. Physics contact reports are observed at 480 Hz; scores are exposed every 20 Hz control step.

## Run

Use an installed Isaac Sim / Isaac Lab / YAM Arena environment with RTX rendering and Inspect Robots. Validated runtime: Python 3.12.12, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Inspect Robots 0.60.0. Run from this directory; `run_local.py` registers the embodiment and demo policy without editable installation. CPU physics is 480 Hz, control and cameras 20 Hz. Seed 0 and no object jitter define the single fixed layout. Limit: 480 actions, 24 simulated seconds; a miss normally ends sooner.

```sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export TASK_TELEMETRY_DIR="$PWD/logs/demo-native"
python -m unittest discover -v
python run_local.py run --embodiment yam_tabletop_rally --policy rally_scripted --instruction "Using the held paddle, complete three consecutive returns from the rebound board: hit the board, let the ball bounce once on the table, and strike it again." --scorer task_score --max-steps 480 --seed 0 --store-frames --log-dir logs/demo --no-prompt --grader none --no-rerun
```

For Dream-1, set its credentials privately as required by the installed client, change policy to `dream -P model=dreamlabs/dream-1-beta`, and use separate log and telemetry directories. Other settings are identical. The feasibility controller uses native ball position/spin, a thin-shell impact predictor, joint-space IK and ordinary robot actions; its nominal feeder preparation is refined after the real initial bounce. It has privileged state access beyond the model's camera/joint observations. It never commands ball or paddle motion directly. `TASK_TELEMETRY_DIR` records diagnostics only. `python probe.py <new-output-directory>` additionally writes native camera videos and contact events.

## Fidelity

Native rigid contacts include ball spin from friction and explicit thin-shell ball inertia (7.2×10⁻⁷ kg·m² per axis). Paddle compound mass, centre of mass and inertia are authored from the declared components and checked at runtime. Ball contact restitution is 0.9, static/dynamic friction 0.12/0.10 with max restitution/min friction combination; these are estimates, not measured commercial properties. Board/paddle surfaces use 0.9 restitution. The 480 Hz discrete spherical contacts resolve the demonstrated speeds; no trajectory is animated after the feeder release. Rubber/shell deformation, air drag, spin lift, wear and acoustic effects are omitted. Numerical feasibility does not qualify hardware bounce behavior.

## Validated simulation results

The final Inspect runs share environment revision `4bccea0c576f1c8c2ae35521f1d25d64678f047779285405f57b933efbe3281e`. Scripted demo: **1.0**, 76 actions, 26.994 s wall time. Dream-1 beta: **0.0**, 52 actions, 26.937 s wall time, terminated as miss. Both use the same layout, cameras, default CLI bounds/delta approvals, seed and 480-step cap. These are simulator results, not physical robot trials.

The native audit verifies the supplied station geometry/camera rig, the 170 g paddle and 2.7 g thin-shell ball with authored inertias, and the full sequence of four held-blade strikes plus three intervening board/table pairs. Strike times are 2.3250, 2.8333, 3.2646 and 3.7521 s; both gripping fingertips have native contact at every strike. The controller prepares during outbound flight and refines its prediction after board/table contact. Six native H.264 views, setup images, Inspect results and native contact/pose traces are supplied separately from the public source. No media ID is assigned locally.

The calibrated overview can be occluded by the left arm/paddle during the fast rally, and the wrist views can show the handle rather than the ball. Native contact/pose traces are therefore essential supporting evidence for individual impacts; the videos alone do not resolve every contact. No camera was moved to improve visibility.
