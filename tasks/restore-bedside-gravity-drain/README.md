# Restore a paused ventricular drain after the bed moves

> The bed has finished moving. Return the bedside drain to service at the posted setting and leave it operating.

A mannequin on the bed provides the reference for a separate gravity drain on a pole. The bedside order card authorizes resumption at +10 cm H₂O. A bed-height change can leave the controls and their printed settings untouched while changing the driving fluid head. An incorrectly levelled ventricular drain can overdrain or underdrain cerebrospinal fluid. The mannequin, water-flow law and posted order represent that decision on a simulated trainer. [Clinical background](https://www.rch.org.au/rchcpg/hospital_clinical_guideline_index/External_Ventricular_Drains_and_intracranial_pressure_monitoring/)

## Reproduce the public reset

Run from the task folder with IsaacLab-Arena-YAM and Inspect Robots active. The primary scene is `assets/raised.usda`. Control is 20 Hz and physics is 240 Hz. The default case is the public raised-bed layout.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 --scorer task_score --max-steps 1200 \
  --seed 0 --instruction "The bed has finished moving. Return the bedside drain to service at the posted setting and leave it operating." \
  --no-prompt --store-frames --log-dir "$PWD/run"
```

Use `--policy dream -P model=dreamlabs/dream-1-beta` in the policy slot for the hosted rollout. Both policies see the same overview, wrist cameras, instruction and public reset. An operator replicating the trainer uses the dimensional witnesses, a ruler and an independently tared water balance. Independent verification should use an inert water trainer and a mannequin.

| Component | Dimensions, placement and behavior |
|---|---|
| Workstation | Stock 6-DOF YAM arms and parallel jaws on the 0.7475 m-high station table. Bases at x = 0 and y = ±0.305 m. The bed-platform posts rest on the station. The drain pole has a separate floor foot and horizontal mounting bracket. |
| Patient reference | Inert molded head, 190 × 150 × 155 mm, ear profile, fixed headrest and draped torso. The ear witness is at x = 0.478, y = 0.152, world z = 1.060 m. |
| Drain | The initial zero is at world z = 0.940 m. A 0.40 kg polymer carriage with a 215 × 200 × 9 mm backing board moves along a metal guide. The lift neck is 30 × 22 × 18 mm. The spring-return rack pin engages teeth at 12 mm pitch. |
| Brake and tap | Brake tab 28 × 20 × 20 mm, maximum slide 8 mm. The valve has a 20 mm lever arm, physical angular end stops, a finite-torque detent and a graspable knob. The posted sill is mechanically fixed 100 mm above zero. |
| Fluid | The modeled source head is 150 mm above the ear. A resistance-limited path with an air break gives forward flow only. A blue column and visible meniscus fill a separate 100 mL graduated receiver. Each 5 mL tick represents a 12.5 mm rise. The red 35 mL tick helps compare 30 s collections measured from a tared start. The hose follows the measured mounting points. |
| Time and observation | The policy receives the native camera streams and normal YAM proprioception. A separate observer samples fixture bodies and both complete arms at every physics tick. |

## Reading the apparatus

The bedside order card gives the prescribed setting. A green horizontal sight on the mannequin transfers the ear height to a white centerline on a 20 mm green band. The magenta pointer moves with the drain zero. The chamber sill sits 100 mm above that pointer. The pointer and band can be compared directly even when the bed and drain occupy different absolute heights.

The left gripper supports the teal lift neck while the right gripper withdraws the spring-loaded brake pin. An unsupported board can drop onto a lower tooth. The seated pin takes the weight when the jaws withdraw. The right gripper then operates the rotary patient-line tap. The Codman EDS 3 provides the clinical analogue for a pole-mounted adjustable drainage system, a locking screw or clamp, a leveling sight, and a stopcock. The simulation models two usable stopcock positions with the auxiliary path capped and uses a rack and pin for reproducible mechanical retention. It models external handling of an inert bedside setup.

The bed-height change has opposite effects in the two misaligned resets. After a bed rise, opening at the old height increases the driving head. After a bed drop, the old sill lies above the simulated supply head and the receiver stays dry. The already-aligned reset permits immediate resumption.

## Interpreting the water and score

The receiver shows cumulative modeled water. Read its change between the start, midpoint and end of the final 30 s. At nominal head the aligned receiver gains 7.5 mL per 15 s, the low receiver gains 25.5 mL, and the high receiver gains zero. Water continues accumulating on camera after the arms withdraw. The color, volume scale and conductance make the water-bench result readable. A red scale mark compares water quantities, and carries no patient-specific meaning.

Full credit requires 30 s continuously within ±10 mm of ear level, with the brake within 0.5 mm of its stop, the arms at least 30 mm clear, and the tap at or above 88°. Each 15 s half must deliver 6–9 mL. Correcting an initially wrong height and leaving the drain retained, released and closed at or below 2° for at least 2 s earns 0.5. Opening beyond 10° out of tolerance, touching the mannequin or routed circuit as defined by the safety envelopes, disturbing the reference, excess flow above 4 mL per 5 s, or closed leakage above 1 mL per 5 s latches zero. Other outcomes earn zero. Trace integrity and full recording checks precede qualification.

## Clinical basis and measurement limits

Bedside EVD practice uses a prescribed drainage height and an anatomical leveling reference. With a supine head in a neutral position, the external auditory meatus or tragus approximates the level of the foramen of Monro. The manufacturer's graduated panel aligns its zero with the reference and the chamber is set to the prescribed height above it. Repositioning changes that relationship. [AANN bedside competency](https://aann.org/uploads/Neuroscience_Resources/EVD.pdf) · [London Health Sciences Centre device guide](https://www.lhsc.on.ca/media/8397/download)

Visual estimates of level vary between observers. The coplanar sights and ruler target make the simulation comparison reproducible. The ±10 mm scoring band, fixed +10 cm H₂O order and accelerated water circuit are specific to this benchmark. Clinical thresholds and patient drainage volumes cannot be inferred from these water readings. Published bench studies also distinguish experimental hydraulic behavior from clinical observations. [Published leveling study](https://pubmed.ncbi.nlm.nih.gov/9307930/) · [Published EVD bench apparatus and measurement study](https://pmc.ncbi.nlm.nih.gov/articles/PMC8239198/)
