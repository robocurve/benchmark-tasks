# Working revision of the wall and fastening tool

Use the calibrated dual-YAM station. The work face sits 350 mm along the station x-axis. It has a 200 × 200 mm opening and a closed 89 mm cavity. The facing and the 197 × 197 mm repair panel are 12.7 mm thick. Two separate 251 × 25 × 16 mm backing strips begin on their supports. The keyed handle begins attached to the upright patch. Reset coordinates, wall boundaries, keyed openings and rack measurements are specified in [repair_scene.py](repair_scene.py).

The tool rests on the table. Its commercial reference is the Bosch GTB 12V-11 with the compatible MA 55 magazine. The simulation approximates its envelope and mass distribution. The modeled assembly weighs 1.26 kg and has a separate spring-return switch and axial shoe. Its eight distinct 30 mm screws are spaced 12 mm apart on the carrier. Each drive needs the trigger, pressure, a correctly indexed pocket, positive spindle motion and engagement with the backing. A complete shoe withdrawal permits the next feed. [Tool dimensions](tool_profile.py) · [Tool joints and carrier](wall_driver.py) · [Manufacturer's compatibility statement](https://www.bosch-professional.com/gb/en/products/gtb-12v-11-06019E4003)

See the [creator-filmed tool test](https://www.youtube.com/watch?v=ar1G789SydA&t=132s) for a loaded MA 55 driving into timber. Within this revision window we could not obtain the same hardware for a physical YAM test.

## End-state grading

| Score | Observable outcome |
|---|---|
| 0.25 per backing row, up to 0.5 | A strip is secured at both jambs. The two credited strips span opposite halves of the opening. |
| 0.75 | Both backing rows are secured, with the released, flush patch fastened to both for one second. |
| 1.0 | Each backing strip also secures both panel wings. The released assembly holds through the 2 N axial load and recovery. The driver is at rest on the table with its trigger released and the robot clear for one second. |

The four corner face deviations must stay below 2 mm. Per-edge gap is between −0.5 mm (simulation tolerance) and 3.5 mm. During the full-score proof the corner displacement must stay within 1 mm over 0.5 s at rest, 1 s under the load and 1 s after unloading. The proof applies an actual extra force to the free patch. The model receives observations and can choose its manipulation strategy. To grade putaway on a physical bench, watch the trigger return while the tool is airborne, confirm a gripper still supports the casing as it meets the tabletop, and observe a full second after both grippers clear it. The simulator checks for contact between a fingertip and the casing at touchdown, with the switch already released. It then checks all tool components against both arms, sustained tabletop contact, switch travel below the 2 mm activation point and housing motion within 2 mm and 2° during the hold. Either arm may park the tool. [Scoring geometry](grading.py) · [Force integration](fastener_load.py) · [Independent replay](verify.py)
