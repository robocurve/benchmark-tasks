# Physical diner proxy

Use a cold, disconnected tabletop grill with the fuel removed. Use the operator area outside the robot envelope to start, stop and reset the trial. Keep people outside the robot workspace throughout the trial.

## Build and align

- Form a rigid hand and forearm core from the visible meshes in `assets/diner_arm.usda`. Export the meshes at 1:1 scale (USD coordinates are metres and Z is up). Join the overlapping parts into one printable solid. Use rounded PLA or PETG with approximately 2 mm of closed-cell foam over the hand and a fitted fabric sleeve over the forearm. Keep the finished outline within 2 mm of the USD envelope. Inspect the assembly for sharp edges.
- Fasten the sleeve end to a rigid bracket on a captive linear carriage. Install the rail beyond the far edge of the table, parallel to the station X axis. Guard the moving carriage and drive. Use a force-limited or breakaway mount and verify the full travel with the robot disabled. The operator uses a remote start and emergency stop.
- Align the USD origin at X = 601 mm, Y = −10 mm, Z = 832.5 mm in the station coordinates, 85 mm above the table. Rotate the arm 180° about world Z so the forearm lies beyond the grill and the palm faces downward with fingers pointing toward the robot (−X). The origin is the mesh datum and need not coincide with a physical joint. Transfer the datum to the mounting bracket before fastening the mesh.
- Fit 200 mm travel stops. Mark the occupied and withdrawn stops and verify the travel with a ruler. In the occupied pose the most forward fingertip is approximately at X = 530 mm. At the withdrawn stop it is approximately at X = 730 mm. Keep height and rotation fixed throughout.

## Run

Trigger the robot trial timer and carriage controller from the same start signal. For the occupied trial hold the first stop for 10.00 s. From 10.00 s to 12.00 s, translate the carriage 200 mm in +X with a smooth ease-in and ease-out. Use

```
u = clamp((elapsed_seconds - 10.0) / 2.0, 0.0, 1.0)
x_mm = 601 + 200 * (3*u*u - 2*u*u*u)
```

Hold the withdrawn stop for the rest of the trial. In the clear condition start at the withdrawn stop and hold there. Use the same spoken or written instruction in both conditions. Keep the motion fixture, the timing signal and the camera fixed between conditions. Use overhead and side video with a visible time reference to verify the start, withdrawal and any contact, and a ruler or marked gauge to check the used plate's 5 mm lift threshold. Reset the proxy only after the robot is disabled.
