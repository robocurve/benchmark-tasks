from pathlib import Path
import argparse,json,time,traceback,hashlib
p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--source',required=True);args=p.parse_args();OUT=Path(args.out).resolve();OUT.mkdir(parents=True,exist_ok=True)
from isaaclab.app import AppLauncher
app=AppLauncher(headless=True,enable_cameras=True,device='cuda:0').app
try:
 import numpy as np,torch,carb
 from PIL import Image,ImageDraw
 from scipy.spatial.transform import Rotation
 import window
 window.install(measurement=True)
 from yam_arena import station,YAM_URDF
 from yam_arena.urdf import forward_kinematics
 from yam_arena.constants import READY,ARM_JOINTS
 import yam_arena.scene as sm
 from yam_arena.embodiment import BimanualYamEmbodiment
 from pxr import UsdGeom,UsdShade,Gf,Sdf
 D=Path(__file__).parent.resolve();SOURCE=Path(args.source).resolve()
 initial_path=D/'dirty.png';last_texture=SOURCE/'final_contamination.png';rgba=np.array(Image.open(initial_path).convert('RGBA'));rgba[:,:,3]=0;Image.fromarray(rgba).save(OUT/'clean_texture.png')
 settings=carb.settings.get_settings();prior_auto=settings.get('/rtx/post/histogram/enabled');settings.set('/rtx/post/histogram/enabled',False)
 station.ITEMS={};original=sm.station_assets
 def assets(cam):
  d=original(cam);d.pop('bottle',None);return d
 sm.station_assets=assets
 emb=BimanualYamEmbodiment(enable_cameras=True,overview_cam=True,contact_sensors=False,rig=station.rig(),cam_width=640,cam_height=352);emb.camera_config.left_cam.spawn.clipping_range=(.0002,20.)
 env=sm.make_env(emb,scene='station',control_hz=20,device='cuda:0');b=env.unwrapped;env.reset()
 from isaaclab.sim.utils.stage import get_current_stage
 stage=get_current_stage()
 from test_card import geometry
 centre,ex,ey,W,H=geometry()
 action=torch.tensor([[*READY,1.,*READY,1.]],device=b.device,dtype=torch.float32)
 for _ in range(40):env.step(action)
 cam=b.scene['left_cam']
 def arr(x):return x.torch.cpu().numpy() if hasattr(x,'torch') else x.cpu().numpy() if hasattr(x,'cpu') else np.asarray(x)
 # Freeze the measurement geometry: no physics steps or arm motions between conditions.
 camera_before={'position':arr(cam.data.pos_w)[0].tolist(),'quaternion_opengl':arr(cam.data.quat_w_opengl)[0].tolist(),'K':arr(cam.data.intrinsic_matrices)[0].tolist()}
 C=np.array(camera_before['position']);R=Rotation.from_quat(camera_before['quaternion_opengl']).as_matrix();K=np.array(camera_before['K'])
 def pixel(P):
  v=(P-C)@R;return [float(K[0,0]*v[0]/(-v[2])+K[0,2]),float(K[1,1]*(-v[1])/(-v[2])+K[1,2])]
 roi_corners=[centre+ex*x+ey*y for x,y in [(-W/2+.008,-H/2+.008),(W/2-.008,-H/2+.008),(W/2-.008,H/2-.008),(-W/2+.008,H/2-.008)]]
 roi_points=[pixel(p) for p in roi_corners];mask=Image.new('L',(640,352),0);ImageDraw.Draw(mask).polygon([tuple(p) for p in roi_points],fill=255);mask.save(OUT/'roi_mask.png')
 cases=[('clean_reference',OUT/'clean_texture.png'),('dirty_before',initial_path),('after_wiping',last_texture),('clean_repeat',OUT/'clean_texture.png'),('dirty_repeat',initial_path)]
 for label,path in cases:
  for _,_,tx in window.DIRT:tx.GetInput('file').Set(str(path))
  for _ in range(20):b.sim.render();cam.update(.05,force_recompute=True)
  frames=[]
  for j in range(5):
   for _ in range(3):b.sim.render();cam.update(.05,force_recompute=True)
   rgb=arr(cam.data.output['rgb'])[0,:,:,:3].astype('uint8');Image.fromarray(rgb).save(OUT/f'{label}_{j}.png');frames.append(rgb.astype(np.float32))
  mean=np.mean(frames,axis=0);np.save(OUT/f'{label}_mean.npy',mean/255.);Image.fromarray(np.round(mean).astype('uint8')).save(OUT/f'{label}.png');print('CAPTURED',label,flush=True)
 camera_after={'position':arr(cam.data.pos_w)[0].tolist(),'quaternion_opengl':arr(cam.data.quat_w_opengl)[0].tolist()}
 def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
 (OUT/'provenance.json').write_text(json.dumps({'scope':'post-trial standardized image assessment','source_manipulation':str(SOURCE),'source_result_sha256':sha(SOURCE/'result.json'),'initial_texture':str(initial_path),'initial_sha256':sha(initial_path),'final_texture':str(last_texture),'final_texture_sha256':sha(last_texture),'chart_size_m':[W,H],'chart_sha256':sha(D/'test_chart.png'),'roi_polygon_pixels':roi_points,'camera_before':camera_before,'camera_after':camera_after,'auto_exposure_before':prior_auto,'auto_exposure_during':settings.get('/rtx/post/histogram/enabled'),'physics_frozen_between_cases':True,'samples_per_condition':5,'board_centre_world':centre.tolist()},indent=2));env.close()
except Exception:
 (OUT/'error.txt').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
finally:app.close()
