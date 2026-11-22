# Repair a drywall opening from the front

The scene contains the loose-stock wall with the 1.26 kg reference-profile driver, a moving trigger and an eight-pocket collated feed. It runs on the original calibrated dual-YAM station. Each new trial starts with both strips and the patch on their supports. The known-layout diagnostic can use simulated object poses. The scored policy receives station observations.

The instruction delivered at reset to both Dream and the demonstration is quoted below.

> Repair the opening from the front. Pick up the loose timber strips and the collated screwdriver. Fasten one strip behind the upper half and one behind the lower half, at both jambs. Fit the patch flush and screw both wings to each strip, using eight screws in total. Remove the carrying handle. Support the screwdriver while releasing its trigger, set it down on the table, and leave both grippers clear. Leave the patch ready for mesh and joint compound.

For full credit the trigger must already be released when the driver touches down under gripper control. Both policies have 7,200 actions at 20 Hz (360 simulated seconds). Physics runs at 240 Hz.

## Commercial fastening tool and physical evidence

The reference tool is the **Bosch Professional GTB 12V-11 drywall screwdriver with the Bosch MA 55 collated-screw magazine**. Bosch lists the attachment as compatible and identifies plasterboard-to-wood fastening as an intended use. The simulated tool reproduces the envelope, working mass, spring-return trigger, pressure shoe and finite collated feed at workcell scale. [Bosch's product specification](https://www.bosch-professional.com/gb/en/products/gtb-12v-11-06019E4003) · [Dimensions and scoring](WORKCELL.md)

The creator “Bosch and Festool tools” filmed the same pairing at a workbench. Around 2:12 the operator places the loaded magazine nose against timber and drives screws. Around 2:34 the operator repeats the operation and lifts the workpiece for inspection. We could not obtain the identical driver and magazine within the revision window to test them on a physical YAM. The video supplies human-operated hardware evidence. Our dual-YAM trials use the simulated tool. [Workbench test at 2:12](https://www.youtube.com/watch?v=ar1G789SydA&t=132s) · [Second drive at 2:34](https://www.youtube.com/watch?v=ar1G789SydA&t=154s)

Martin Marino also compares the GTB 12V-11 with a conventional driver on a green-faced board sample and shows screw-head depth. This second review illustrates the board operation. The first video above identifies and demonstrates the collated attachment. [Martin Marino’s board test](https://www.youtube.com/watch?v=zkVYLIHxLg4&t=545s)

## Reproduction

Activate the pinned RoboCurve walkthrough environment and run these commands from the submission checkout. Use a new output directory for each rollout.

```bash
python tasks/front-access-drywall-patch/run.py --mode demo --log-dir /tmp/drywall-repair-demo
python tasks/front-access-drywall-patch/verify.py /tmp/drywall-repair-demo --media /tmp/drywall-repair-media
python tasks/front-access-drywall-patch/run.py --mode dream --config-only --log-dir /tmp/drywall-dream-config
```

The verifier replays the native fastening, support, trigger and contact evidence and exports the three-camera film. The final command previews the model's instruction and action budget locally. A real Dream rollout uses the same runner in `--mode dream` with configured Dream credentials.
