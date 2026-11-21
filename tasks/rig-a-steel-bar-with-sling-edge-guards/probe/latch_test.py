"""Why does the probe's latch rest at +2.8 deg? Hook rig of sling_probe.usda only (the sling and guards parked far
away), 1 s of physics per variant, reading the latch DOF and the latch's angle about x measured from the hook and latch
link poses.   python latch_test.py"""

import json
import math
import os
from pathlib import Path

import numpy as np

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
from isaaclab.app import AppLauncher  # noqa: E402

app = AppLauncher(headless=True, device="cuda:0").app
import carb  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402

carb.settings.get_settings().set_bool("/physics/tensors/recursiveLeafPatternMatch", False)
DT = 1 / 240
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device="cuda:0", render_interval=24))
cfg = sim_utils.UsdFileCfg(usd_path=str(Path(__file__).with_name("sling_probe.usda")))
cfg.func("/World/Probe", cfg)
sim.reset()
import omni.physics.tensors as tensors  # noqa: E402

view = tensors.create_simulation_view("torch")
view.set_subspace_roots("/")
rig = view.create_articulation_view("/World/Probe/HookRig")
sling = view.create_articulation_view("/World/Probe/Sling/link_11")
dof = list(rig.shared_metatype.dof_names)
links = list(rig.shared_metatype.link_names)
L = dof.index("latch_hinge")
one = torch.arange(1, dtype=torch.int32, device="cuda:0")
# park the sling far away so nothing touches the hook
rt = sling.get_root_transforms().clone()
rt[0, :3] = torch.tensor((1.5, 0., .5), device="cuda:0")
sling.set_root_transforms(rt, one)


def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def measure():
    q = rig.get_dof_positions().cpu().numpy()[0]
    lt = rig.get_link_transforms().cpu().numpy()[0]
    Rh, Rl = quat_R(lt[links.index("hook"), 3:]), quat_R(lt[links.index("latch"), 3:])
    rel = Rh.T @ Rl
    geo = math.degrees(math.atan2(rel[2, 1], rel[1, 1]))      # rotation about x of the latch in the hook frame
    return {"dof_deg": round(math.degrees(float(q[L])), 3), "geometric_deg": round(geo, 3),
            "swing_deg": [round(math.degrees(float(q[dof.index(n)])), 3) for n in ("swing_x", "swing_y")]}


out = {"dof_names": dof, "link_names": links, "stiffness": rig.get_dof_stiffnesses().cpu().numpy()[0].tolist(),
       "damping": rig.get_dof_dampings().cpu().numpy()[0].tolist(), "limits": rig.get_dof_limits().cpu().numpy()[0].tolist(),
       "targets": rig.get_dof_position_targets().cpu().numpy()[0].tolist(), "start": measure()}
rows = []
for step in range(240):
    sim.step(render=False)
    if step in (0, 5, 23, 239):
        rows.append({"step": step, **measure()})
out["as_built"] = rows
print(json.dumps(out, indent=1), flush=True)
# zero the latch spring: does it fall open by gravity, and from where?  (property setters take CPU tensors)
st = rig.get_dof_stiffnesses().clone().cpu()
st[0, L] = 0.
rig.set_dof_stiffnesses(st, torch.arange(1, dtype=torch.int32))
rows = []
for step in range(240):
    sim.step(render=False)
    if step in (0, 23, 239):
        rows.append({"step": step, **measure()})
out["no_spring"] = rows
print(json.dumps(out, indent=1))
app.close()
