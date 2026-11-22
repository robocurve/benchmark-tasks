# Fit the teat and screw collar onto a baby bottle

Fit the separate silicone teat through the collar, fasten the collar onto the upright bottle and release the assembly. Either arm may handle any part. The known-layout scripted demonstration uses 1,769 ordinary joint commands per arm at 20 Hz on the calibrated dual-YAM station.

## Run the simulation

With the walkthrough runtime and original YAM station installed, run from this task directory using Python 3.12. Choose an available GPU and a new output directory. Use `python runtime_check.py` to check the versions and revisions listed in `runtime-lock.json`. Each invocation starts one fixed-layout episode with seed 0 and zero object jitter.

```bash
python run.py --gpu 1 --out /tmp/bottle-review-trial
```

For the formal model trial, use `python run.py --mode dream --gpu 1 --out /tmp/bottle-model-trial` with a new output directory and an authorized Dream Labs account. The task uses Dream-1 on the three original 320 × 176 RGB observations and the standard joint state, with 20 Hz control and a limit of 2,400 steps (120 seconds). Cloud recording is disabled. A free reset check is available via `--mode setup`. Adding `--config-only` writes the configuration and runtime pins without contacting the model.

The runner registers the automatic collar load exam with Inspect Robots. The framework stores its verdict in the recorded judgement field and reports it under the metric `operator`. No person enters this value. The per-action trace records whether the released assembly qualifies for the exam. The final binary result is in `endpoint.json`. The runner independently replays both stages from the physical records. The three camera streams cover the robot's actions. The subsequent examiner motion is stored separately in `collar-exam.npz` when an exam takes place.

## Grading

Full credit requires the released assembly to remain engaged and continuously seated while the bottle hangs from the collar. Any other readable outcome earns zero. The robot can finish with the bottle on the table. It need not demonstrate a lift or follow a particular grasp sequence.

1. Observe one second of release. Both internal thread lugs must engage the bottle, the flexible teat must remain captured around the collar and its sealing land must meet the rim around the whole circumference.
2. After the last policy action, the grader applies a bounded upward force to the collar alone, raising its target 40 mm in one second. It allows 0.5 seconds to settle, then checks one second under load. The bottle must rise at least 20 mm and clear the table by at least 20 mm throughout the hold. Repeat the same seal and retention checks. If the examiner meets the released robot, it repeats the lift on the clear table from the captured end state. The rigid poses, native velocities and flexible shape are preserved up to a common horizontal translation. This test runs after the policy has finished.
3. The simulation measures the sealing land at 48 points and checks both lugs at 15 locations. It uses a 0.05 mm numerical contact band at the rim, at least 12 engaged locations on each lug and release from all 18 arm links. Geometry is sampled at 20 Hz and rigid contacts at 240 Hz. The 20 g fixed load represents the weight of a 20 mL upright prefill. These checks measure mechanical retention and circumferential contact. Liquid flow, the nipple valve and commercial leak performance have not been validated.

## Physical reset and grading

Use a Philips Avent Natural Response SCY900/01 bottle with its matching teat and screw collar as the proposed physical reference. The simulated thread profile and flexible section are engineering approximations. Start with the three parts separated and dry on the outside. Add 20 mL of clear water to the upright bottle. With the station origin midway between the arm bases, positive X towards the work and positive Y towards the left arm, place the bottle centre at (304, 18) mm, the collar centre at (318, 112) mm and the teat centre at (301, −131) mm. Place the collar as shown in the reset view and stand the teat on its base. Use the original table, cameras and parallel grippers.

At the end of the robot's turn, check that both grippers have released the assembly. Over a dry tray, lift it by the collar alone until the bottle clears the table. Invert it for 10 seconds. Watch the bottle–collar seam and the base of the teat around their full circumference. Water must remain inside and reach both interfaces throughout inspection. The grader may turn the bottle to see the far side. Use the following outcomes.

| Physical result | Score |
| --- | --- |
| The bottle follows the collar, all parts stay attached, and neither interface leaks. Any water at the intended nipple outlet can be clearly traced to that outlet alone. | 1 |
| The robot leaves the parts unassembled or still held, the bottle fails to follow the collar, a part separates, or water can be seen emerging from either interface. A clearly observed failure ends the trial. | 0 |
| The reset was invalid, or a complete verdict cannot be obtained (for example an unseen seam, too little water to wet it, or a drop whose origin cannot be traced). | Unscored. Dry the parts and tray, restore the 20 mL prefill and repeat from the original reset. |

Record the observed result with `physical_grading.py`. For example, if the assembly held together, both seams were checked for the full 10 s, and water came only from the intended outlet, run:

```bash
python physical_grading.py --retention held --seams outlet-only --inspection complete
```

An unreadable result is emitted as JSON `null`. Use `--help` for the leak, separation and invalid-reset entries. This records human observations. The simulator uses the contact and collar-load checks above as mechanical proxies for the proposed physical test.
