# Check an installed front brake pad through the wheel

Measure the outer brake pad through the wheel, return both gauges, and mark the measured thickness range on the inspection card.

## Work performed

Automotive service technicians check friction lining thickness during disc brake inspection. Toyota's service tip specifies a ruler or pad thickness gauge on the friction lining. Brake inspection and work on pads and retaining hardware are part of ASE's A5 Brakes task list. The U.S. Bureau of Labor Statistics counted 825,800 automotive service technician and mechanic jobs in 2025 across all duties. The task exercises the constrained inspection motion through a mounted wheel. It measures a thickness interval using two stock gauge heads and records the interval on a bench inspection card. The vehicle's service limit remains a separate service decision. [Toyota Tech Tip T-TT-0132-11](https://static.nhtsa.gov/odi/tsbs/2014/SB-10060762-2280.pdf) · [ASE A5, Disc Brake Diagnosis and Repair](https://www.ase.com/uploads/ASE_Automobile_Study_Guide_2025.pdf#page=28) · [BLS, Automotive Service Technicians and Mechanics](https://www.bls.gov/ooh/Installation-Maintenance-and-Repair/Automotive-service-technicians-and-mechanics.htm)

Lisle's patented axle-parallel approach inserts the offset head through a wheel opening, then sets the transverse lip of the offset measuring block between the rotor face and the inside of the backing plate. The caliper and carrier stay installed. [Lisle 81850 kit](https://www.lislecorp.com/specialty-tools/brakes/combination-brake-lining-thickness-gauge-set) · [Lisle US9500462B2, Fig. 7](https://patents.google.com/patent/US9500462B2/en#description)

## Hardware and reset

- Use the right-front corner of a Toyota Camry ACV40 2.4 with the original floating caliper, carrier, both pads, dust shield, hub and wheel mounted in their service positions. A non-operating donor corner secured upright to a bench stand reproduces the same access. Use a Brembo 09.A417.11 ventilated disc (296 mm diameter, 28 mm thickness, 49 mm height) and the P83117N pad set (158 × 57 mm plan envelope, 18 mm stated total thickness when new). Use the hub-compatible Toyota 42611-33750 16-inch seven-spoke wheel with a 215/60R16 tire. Confirm the donor caliper and wheel fit against the VIN. [Brembo Camry fitment](https://www.bremboparts.com/asiapacific/en/catalogue/toyota-camry-saloon-v4-2-4-acv40/000057752-1) · [Toyota caliper and carrier drawing](https://toyota.epc-data.com/camry/acv40/15619/chassis/4705/) · [Toyota seven-spoke wheel](https://toyota.oempartsonline.com/oem-parts/toyota-wheel-alloy-7-spoke-16-inch-4261133750)
- Qualify one used outboard P83117N pad for the training fixture. Before assembly, measure the friction lining from its working face to the lining-side surface of the steel backing at three adjacent, unchamfered points nearest the accessible end. Use a calibrated brake-pad thickness gauge or depth gauge that excludes the backing plate and shim. Record all three readings and a photo. Select a pad with **8.90–9.10 mm at all three points** and a clear edge, then mark and reuse that exact pad for every reset. Check the remaining gap at the same point after assembly using the two specified heads. The simulation uses a nominal 9.00 mm lining. A physical specimen measurement is pending.
- Use Lisle 81920 (8 mm) and 81930 (10 mm) acetal gauges. Identify the 8 mm gauge in blue and the 10 mm gauge in orange. On physical tools, place identifying color labels on the handles and keep the measuring faces bare. On the bench both are edge-up in separate rests, tips pointed toward the wheel. Each rigid plastic rest has a flat handle support and a separate stem support with a 7 mm guide slot. Set the hub center 332 mm above the bench and the two measuring-head origins about 153 mm above the bench at reset. See `reset.json` for positions and dimensions.
- Keep the wheel bolted to the hub and index one opening to expose the leading end of the outboard pad, as shown in the setup image. The inspection point is at the end of the pad between the carrier horn and caliper bridge. The modeled seven-spoke pattern has a gap centered 30° from the caliper radial reference. Leave the cast caliper, its carrier and the wheel in place throughout the trial.

- Place the bench inspection card and yellow marker at their `reset.json` coordinates. The flat 161 × 202 × 6 mm card has four printed positions labeled START, <8 MM, 8–10 MM, and 10+ MM. The marker is a 34 mm diameter, 28 mm tall rigid cylinder. Reset it to START. The middle bracket means at least 8 mm and below 10 mm.

## Score

The 8 mm head passes through the spoke opening and enters the pad-to-disc gap. Touch the rotor reference and the inside of the backing plate with the same measuring head to show the fit. The 10 mm head uses the same opening and rotor reference but stops at the outer edge of the backing plate. The observed result is at least 8 mm and less than 10 mm. For a physical trial, use a light and fixed close-up through the opening to see the 8 mm head below the edge and the 10 mm head stopped outside it. Confirm rotor contact at each reading, verify both gauges are released in their rests, and check that the marker is centered in the measured bracket.

The simulator tracks passage through a physical spoke opening from the outboard side. Four consecutive valid samples establish each contact witness. At .25 the small measuring head contacts the rotor in the gap. At .50 it also reaches the underside of the backing. At .75 the 10 mm head has sustained rotor contact and a radial stop at the backing edge. At 1.0 both tools are back in their rests and the marker has been placed in the 8–10 mm box. The released marker must settle within 8 mm of the box center and 2 mm of its seated height, and the grippers must stay clear for 20 controls. The contact check uses the head's actual position and normals. Spoke, caliper, barrel and stem contacts cannot establish a reading.

## Simulation scope

The parked donor corner models the cast obstruction, carrier horns, both pads, hub, rotor, wheel barrel, seven spokes and tire as rigid collision solids. The hub assembly is fixed to the stand. The reset uses catalog pad, disc, wheel and tire envelopes. The cast profile, spoke offsets and opening clearances are representative estimates. The simulation represents pad-to-disc running clearance as contact, omits the dust shield and brake actuation, and uses rigid gauges with approximate body profiles. The model colors the gauge bodies for video readability and leaves the nominal measuring geometry unchanged. The marker and card are a bench job aid for recording the interval. This is a measured access and manipulation task for a stationary fixture.

## Replay

From this directory with the RoboCurve simulator environment active:

```bash
mkdir -p .local
GAUGE_DIAGNOSTICS=.local/installed-trace.jsonl python capture_detail.py run \
  --embodiment yam_arena --policy yam_scripted -E task=task.py \
  -E control_hz=20 -E object_jitter=0 -E cam_width=640 -E cam_height=360 \
  --instruction 'Measure the outer brake pad through the wheel, return both gauges, and mark the measured thickness range on the inspection card.' \
  --scorer task_score --max-steps 1800 --seed 0 --store-frames --no-prompt --log-dir .local/installed
python -m unittest test_scoring
```

For matched contacts at off-band dimensions, set `GAUGE_CONTROL_GAP=narrow` (6.6 mm) or `GAUGE_CONTROL_GAP=wide` (11.4 mm). These are simulator controls with the same installed assembly and approach.

## Verified demonstration

The nominal 9.00 mm seed-0 replay scored **1.0 in 1,106 controls**. The blue 8 mm head entered about 1.51 mm beneath the backing plate, while the orange 10 mm head stopped at the edge. The left gripper picked up the yellow marker from START and released it 0.38 mm from the center of the 8–10 mm box. The final close-up shows the result after both arms retreat. The trace recorded zero arm-to-arm or arm-to-wheel assembly contacts. Nine offline checks passed. Replaying the earlier 6.6 mm and 11.4 mm control traces through the new scorer with the marker already set yielded 0.0 and 0.25.
