"""Load actual station, save native camera PNGs and measured settled object poses."""
import sys,json,os
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ratchet_task.embodiment import RatchetEmbodiment,FEEDBACK,array,ROOT
from ratchet_task.config import INSTRUCTION
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.constants import READY
import numpy as np
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
e=RatchetEmbodiment(task=str(ROOT/'task.py'),device='cpu',top_cam='demo',control_hz=20,cam_width=1280,cam_height=720)
o=e.reset(Scene(id="asset-preflight",instruction=INSTRUCTION),seed=17)
import imageio.v3 as iio
for name,im in o.images.items():iio.imwrite(out/(name+'-start.png'),im)
initial={k:v.copy() for k,v in FEEDBACK['poses'].items()}
for i in range(100):result=e.step(Action(data=np.r_[READY,1.,READY,1.]))
for name,im in result.observation.images.items():iio.imwrite(out/(name+'-settled.png'),im)
from pxr import UsdGeom
cache=UsdGeom.BBoxCache(0,[UsdGeom.Tokens.default_],useExtentsHint=False)
objects={}
for n,p in FEEDBACK['poses'].items():
    prim=e._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+n)
    bound=cache.ComputeWorldBound(prim).ComputeAlignedBox()
    extra={}
    if n in ['bolt','handle','drive','pawl']:
        body=e._base.scene[n];extra=dict(mass_kg=array(body.data.default_mass).tolist(),inertia_kg_m2=array(body.data.default_inertia).tolist())
    objects[n]=dict(position=p.tolist(),rotation=FEEDBACK['rotations'][n].tolist(),**extra,drift_m=float(np.linalg.norm(p-initial[n])),bbox_size_m=list(bound.GetSize()))
json.dump(dict(objects=objects,revision=e.info.environment_revision,physics_hz=e.physics_hz,native_cameras={n:[e._base.scene[n].cfg.width,e._base.scene[n].cfg.height] for n in ['top_cam','left_cam','right_cam']}),open(out/'assets-result.json','w'),indent=2)
print('ASSET_LOAD_COMPLETE',json.dumps(objects),flush=True)
e.close()
