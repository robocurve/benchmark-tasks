> The side-approach revision reached score 1.0 in a continuous, from-reset YAM scripted demonstration (4,291 control steps, fixed initial layout). Dream-1 beta scored 0.0 in one official 6,000-step trial with zero errored trials. Bit installation requires releasing the bit, not parking either arm; holding the housing is allowed.

# Fit the Matching Bit and Power-Tighten a Loose Drawer Knob

A maintenance robot must inspect the screw recess, choose the matching bit by its working-end shape, fit it into an empty powered screwdriver, steady a free drawer, drive one loose knob screw, and leave the stopped tool released in its stand. Securing cabinet hardware is routine work for general maintenance and repair workers and cabinet installers. The bit is a removable tool attachment; the drawer contains exactly one workpiece screw.

The tool must settle stably on its U-shaped stand with its complete orientation within 8° of the starting orientation; exact placement and orientation are not required. The grey recessed bed is visual-only, while the two U supports provide contact. The current revision removes the unused button, removes intermediate arm parking and brings the tool in from the left after the right arm grips the near wooden wall. The free drawer and both supports are shifted 10 mm toward the table centre. In the validated demonstration, the right gripper approaches above the lip, descends with open jaws, then closes around the wall; both fingertips register contact before tightening. After returning the tool, both arms withdraw to ready and remain clear for four seconds to make the end state visible. The scorer requires open hands clear of the tool, not either arm's exact ready pose. A continuous physical simulation demonstration scored 1.0. This establishes fixed-layout feasibility in the official simulator; it does not establish robustness to randomized layouts or real-world hardware.

## Run

Use the official YAM / Isaac Lab Arena environment with Inspect Robots and Dream-1 installed. From this task directory:

```bash
python demo.py runs/preview --preview-only
python demo.py runs/assembly --assembly-only
python demo.py runs/demo
```

The demo starts from reset, sends only YAM joint and gripper commands, and rejects checkpoint restoration. Its 640 × 352 camera frames are for evidence; model input is 320 × 176. The validated run scored 1.0 with success in 4,291 of the allowed 6,000 control steps. The tool's final complete-orientation error was 1.02°; it was stopped in the stand with both hands open and clear, and met the one-second stability test. `setup.png` is the reset frame of this revision. The official top-camera demonstration is recorded in the run evidence.

```bash
inspect-robots run --embodiment yam_arena --policy dream --policy-checkpoint dreamlabs/dream-1-beta \
 -E task=task.py -E top_cam=demo -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
 --instruction "Inspect the loose drawer-knob screw. Fit the matching bit into the stopped powered screwdriver, tighten the screw with that tool, then release the stopped tool in its stand." \
 --scorer task_score --max-steps 6000 --store-frames --seed 42 --no-prompt --log-dir runs/model
```

Control runs at 20 Hz and physics at 240 Hz. Report the scripted demo and Dream-1 scores separately from their actual result files.

## Score

The scorer returns the highest **currently satisfied** tier. It retains only the event history needed to prove that bit assembly preceded powered screw motion; loss of the bit or backing out the screw removes the corresponding credit.

| Score | Measured condition |
| --- | --- |
| 0 | No valid installed bit, or the installed bit has been lost. |
| 0.2 | Correct cross bit seated in the stopped chuck, with its tip more than 20 mm from the screw; both gripper TCPs more than 60 mm from the bit grip (holding the tool housing is allowed; no arm parking requirement); bit-to-chuck position stable within 0.5 mm for 20 consecutive control steps. The bit must still be mounted. |
| 0.8 | The 0.2 assembly occurred first. With the mounted bit inside the screw recess (within 1 mm radial, -2.5 to +2.3 mm axial of the screw-head centre, 8° of the screw axis and 15° of its rotating cross orientation), the motor and screw turned forward while the screw fed and the knob gap closed. At least 1.3 mm net feed and equivalent powered spindle/screw rotation are credited; screw feed remains at least 1.3 mm and knob gap at most 0.2 mm. After the tip withdraws more than 10 mm and both hands withdraw more than 100 mm from the knob, it holds a 5 N opening load with at most 0.25 mm deflection and gap at most 0.2 mm for 20 steps. Tool return is not required for 0.8. |
| 1.0 | The 0.8 condition remains true; the motor is stopped, the tool lies within 25 mm of its initial stand position and its complete orientation is within 8° of its starting orientation, both grippers are at least 80% open and more than 60 mm from the tool grip, and the end pose holds for 1 s with at most 2 mm translation, 1° rotation and 0.02 rad spindle motion. |

Motor activation uses the nose-pressure slider: compression above 1.5 mm with the bit tip engaged in the recess starts it; compression below 0.8 mm, loss of tip engagement, or docking stops it. There is no grip interlock or manual switch. Fixed rubber grip shoulders are housing geometry, not controls. The tool need not be returned to earn tightening credit. A loose bit turned by hand, an empty tool spinning, a tool spinning away from the screw or a motor tap after hand-tightening does not create powered feed credit. The 5 N load and 0.25 mm limit are task thresholds; they are not calibrated hardware standards. A full negative-control matrix remains pending.

For a real-world trial, mark the spindle and screw head visibly. Use continuous close-up video to check the bit was installed before work, the engaged spindle and screw rotated together, and the tool was stopped and released. Use a depth gauge or calibrated close-up for screw feed, a feeler gauge for the knob gap, and a small spring scale plus ruler to check the 5 N holding load and deflection. The simulator's recess-volume and angle engagement gate is a contact approximation. Physical trials should observe seated contact rather than infer it from the final gap alone.

## Scene and reset

Coordinates are millimetres in the YAM station frame: x away from the robot toward the drawer, y across the table, z up; table top z=747.5. The robot, table and station come from official YAM. All task-specific solids are procedural rigid bodies with approximate wood, zinc-steel and dark-polymer appearances. No third-party mesh or texture is included.

| Object | Starting body origin (x,y,z) mm | Size and material |
| --- | --- | --- |
| Drawer and knob | (322,360,747.5) | Free 1.4 kg wooden drawer, inner face up; front panel 340×146×16. Guided dark-polymer knob approximately 42×42, initially 1.5 mm loose. One zinc-steel cross screw, 12 mm head, at drawer-local (-32,-60,99). |
| Drawer supports ×2 | (322,205,747.5), (322,515,747.5) | Fixed dark-polymer blocks, 146×23×76 each. |
| Powered screwdriver | nominal (225,-15,812.5), settled near (225,-15,814.5) | Free 320 g polymer/metal prototype, approximately 52×42×190; empty 6.45 mm hex chuck; laid horizontally with shaft axis -y. |
| Tool supports ×2; backstop | (225,-24,747.5), (225,-106,747.5); (225,63,747.5) | Fixed colliding dark-polymer U supports about 70×12×55.5 each; backstop 52×8×74. The U supports carry the tool at reset and on return. |
| Recessed stand bed | centre near (225,-65,769) | Grey 95×70×43 visual-only block between the supports; it does not generate contact forces. |
| Cross, hex, slotted bits | (45,-130,817.5), (45,-40,817.5), (45,50,817.5) | Three separate free 14 g steel-coloured bits, approximately 98 mm overall, with 90 mm long 6.35 mm hex shanks. Distinct working tips, common colour. |
| Three bit racks | (85,-130,747.5), (85,-40,747.5), (85,50,747.5) | Fixed dark-polymer supports, each about 97.5×32×76. |
| Three profile plaques | (175,-130,747.5), (175,-40,747.5), (175,50,747.5) | Fixed dark-polymer 32×32×2.2 plaques with light cross, hex and slotted shapes respectively. |

Reset the drawer inner-face-up across both supports; set the knob gap to 1.5 mm and the single screw bearing angle and axial-feed joint to zero. Return the three bits to their named racks in the initial horizontal poses. Leave the chuck empty and motor stopped, put the driver in its stand, and put both arms in the standard YAM ready pose. Match the replacement setup screenshot and all three camera images before a scored trial. The free drawer is not bolted to its supports.

## Physical approximation

The screw head is enlarged to 12 mm for vision and collision legibility. The 6.35 mm hex bit is retained by the socket geometry; a physical pressure slider with contact-gated push-start (no manual button) gates a 15 rpm, 0.10 N·m motor setting. The single screw bearing and axial carriage are coupled at ideal M4 pitch, 0.7 mm per revolution; the same bearing drives the guided knob. The screw must rotate about 669° for 1.3 mm feed, beyond the YAM wrist roll's 240° total travel without repeated regrasp. The simulation does not resolve helical thread flanks, calibrated clamp preload, real material compliance, thread stripping or product-calibrated motor torque. Fixed rubber grip faces use uncalibrated static/dynamic friction of 1.5/1.3; the socket material is unchanged. It has not been validated against real hardware.

References for dimensions and mechanism: [Würth ISO 7045 M4 screw](https://eshop.wuerth.de/Flat-head-screw-with-H-cross-recess-ISO-7045-steel-48-zinc-plated-blue-passivated-A2K-SCR-PAN-ISO7045-48-H2-A2K-M4X10/5103004010.sku/en/US/EUR/), [Würth furniture-handle screw](https://eshop.wurth.com.au/Product-categories/Furniture-handle-screw/31021504180701.cyid/3102.cgid/en/GB/AUD/), and [Bosch GO Gen 3 tool](https://www.bosch-professional.com/gb/en/products/bosch-go-gen3-06019H2201). These are comparison products, not claims that the scene is a digital twin. The submitted task includes no downloaded third-party mesh or texture: its drawer, tool, bits and fixtures are procedural geometry. The standard YAM robot and station are supplied by the simulation runtime and are not redistributed here. [Isaac Lab-Arena is Apache-2.0 licensed](https://github.com/isaac-sim/IsaacLab-Arena#license); Isaac Sim components retain their [separate NVIDIA terms](https://github.com/isaac-sim/IsaacLab-Arena#license).
