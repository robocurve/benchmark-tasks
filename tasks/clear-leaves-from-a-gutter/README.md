# Clear dry leaves from a gutter

The native demo scores **1.0**; Dream-1 scores **0.00**. Both full recorded episodes passed frozen-source and submitted-scorer replay.

Remove loose leaves from a full-size roof-edge section and collect them in a bucket. The overhang, curved channel and cross-bracket constrain tool access. Working at height is outside this task.

## Run and score

Use the pinned Linux NVIDIA Isaac/YAM runtime in `ENVIRONMENT.json`, or install it with `bash bootstrap.sh`. From this folder:

```bash
OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y python run.py --mode demo --out /absolute/path/to/demo
OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y python run.py --mode dream --out /absolute/path/to/dream
python rescore.py /absolute/path/to/demo
python score_checks.py /absolute/path/to/demo /absolute/path/to/dream
```

The allowance is 1,600 controls at 20 Hz (80 simulated seconds). The scripted plan uses 880 controls and took 9.6 minutes including rendering on an RTX 6000 Ada. The matching Dream baseline completed 1,600 controls and took 26.7 minutes, including rendering and API latency. There is one fixed layout and no random seed.

Count whole leaves inside the bucket after both hands release for one second. The scoop may remain in the bucket; every leaf corner must be inside its wall and below its rim. Divide by three. Partial leaves and leaves outside the bucket earn nothing. The bucket must remain within 40 mm of its starting centre and 18.2 degrees of upright. A leaf falling off the table or broken equipment invalidates the trial. A photo, ruler and count suffice; no force measurement is needed.

Both episodes use the same physics and cameras. Final scoring also explicitly rejects a recorded broken joint; this defensive guard does not change either recorded result. Both traces are checked against the frozen and submitted scorers. The ordinary YAM fingers operate at their configured 50 N effort. Review cameras are recording-only. Dream credentials are read by the official client from its local environment or credential file; run one model episode per key. Full state histories and frozen source hashes support CPU rescoring. The baseline recovered from 32 expired model sessions: the physical episode continued, but model memory restarted. The public request log records these interruptions; the baseline score should not be attributed solely to task difficulty.

## Setup and approximation

Dimensions, materials and table-frame coordinates are in `task.json`; `geometry.py` supplies every starting pose. Secure the full-size PVC gutter/fascia section on the bench. Its 620 × 180 × 18 mm timber fascia rests on two 60 × 60 × 90 mm timber feet; clamp or screw the feet to a sturdy bench. The 620 × 200 × 14 mm roof panel provides the overhang. Use standard table-mounted robot bases, without cantilever stands or a ladder. Use three intact dried cooking bay leaves from one packet, 50–70 × 20–30 mm with a shallow natural curl of 4–10 mm. Keep them dry at room temperature. [Schwartz dried bay leaves](https://www.schwartz.co.uk/products/bay-leaves) are one ordinary source; equivalent whole dried bay leaves are suitable. Select a matching trio once, reuse intact leaves, and restore the pictured cluster beside the closed end between trials. Avoid wet, sticky or crumbling leaves. Replace torn leaves and restore the picture between trials.

Each simulated leaf has four passive articulated panels with finite bending stiffness, gravity and contact. This approximates dry flexible leaf handling; fine veins, tearing, moisture and brittle fragments are omitted. The scoop has a hollow raised handle and a thin curved mouth. All collection and placement use native contact; there is no attachment of leaves to the scoop or bucket. The 0.3 g simulated mass, stiffness and friction are engineering estimates, not measurements of a particular packet.

## Assets

Task geometry is generated here under the included MIT licence. The runtime supplies the I2RT robot meshes under its [MIT asset licence](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt). Table, floor and camera supports are runtime primitives. Robot assets are not redistributed here.
