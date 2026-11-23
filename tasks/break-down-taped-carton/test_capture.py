"""CPU capture/camera-contract tests; no native physics equivalence claim."""
import ast
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import capture
import review


class Array:
 def __init__(self,value):self.value=np.asarray(value);self.reads=0
 def detach(self):return self
 def cpu(self):return self
 def numpy(self):self.reads+=1;return self.value


class NativeCaptureTests(unittest.TestCase):
 def fixture(self):
  names=['base','link1','link2','link3','link4','link5','gripper','tip_left','tip_right']
  paths={name:'/links/'+name for name in names};paths['tip_left']='/links/gripper/tip_left';paths['tip_right']='/links/gripper/tip_right'
  scene={}
  for index,side in enumerate(('left','right')):
   poses=np.zeros((1,9,7),dtype=np.float32);poses[0,:,:3]=np.arange(27).reshape(9,3)+index*100;poses[0,:,6]=1.
   scene[side+'_robot']=SimpleNamespace(body_names=names.copy(),data=SimpleNamespace(body_link_pose_w=SimpleNamespace(torch=Array(poses))))
  panels=['panel_'+str(i) for i in range(12)];quat=[0.,0.,np.sin(.3),np.cos(.3)]
  objects={name:np.r_[[.1,.2,.8+i*.01],quat] for i,name in enumerate([*panels,'cutter'])}
  return scene,paths,panels,objects

 def test_all31_poses_use_native_actor_data_once_per_arm_and_real_object_orientation(self):
  scene,paths,panels,objects=self.fixture();owners=capture.native_body_owners(scene,paths,panels)
  result=capture.native_body_world_poses(scene,owners,objects)
  self.assertEqual(len(result),31)
  for side in ('left','right'):
   body=scene[side+'_robot'];path='/World/envs/env_0/'+side.capitalize()+'Arm/links/link3'
   np.testing.assert_array_equal(result[path],body.data.body_link_pose_w.torch.value[0,3])
   self.assertEqual(body.data.body_link_pose_w.torch.reads,1)
  for name in [*panels,'cutter']:
   np.testing.assert_array_equal(result['/World/envs/env_0/'+name],objects[name])
   self.assertNotEqual(result['/World/envs/env_0/'+name][5],0.)
  tip=owners['/World/envs/env_0/LeftArm/links/gripper/tip_left']
  self.assertEqual(tip['parent_body_owner'],'/World/envs/env_0/LeftArm/links/gripper')
  self.assertEqual(json.loads(json.dumps(result,allow_nan=False)),result)

 def test_missing_invalid_or_reordered_native_data_fails_without_fk_fallback(self):
  for invalid in ('nan','quaternion','shape','order','missing_object'):
   scene,paths,panels,objects=self.fixture();owners=capture.native_body_owners(scene,paths,panels)
   value=scene['left_robot'].data.body_link_pose_w.torch.value
   if invalid=='nan':value[0,0,0]=np.nan
   if invalid=='quaternion':value[0,0,6]=0.
   if invalid=='shape':scene['left_robot'].data.body_link_pose_w.torch.value=value[:,:8]
   if invalid=='order':scene['left_robot'].body_names.reverse()
   if invalid=='missing_object':objects.pop('cutter')
   with self.subTest(invalid=invalid),self.assertRaises(ValueError):capture.native_body_world_poses(scene,owners,objects)
  scene,paths,panels,objects=self.fixture();scene['right_robot'].body_names.reverse()
  owners=capture.native_body_owners(scene,paths,panels)
  self.assertEqual(owners['/World/envs/env_0/RightArm'+paths['tip_right']]['body_index'],0)
  with self.assertRaises(ValueError):capture.native_body_owners(scene,paths,panels[:-1])

 def test_flatten_export_is_output_only_hash_bound_and_reports_dependencies(self):
  calls=[]
  class Layer:
   def Export(self,path):calls.append(('export',path));Path(path).write_bytes(b'exact composed appearance');return True
   def GetExternalAssetDependencies(self):return ['texture.png','shader.mdl']
  class Stage:
   def Flatten(self):calls.append(('flatten',));return Layer()
  with tempfile.TemporaryDirectory() as directory:
   path=Path(directory)/'capture.usdc';result=capture.export_scene(Stage(),path)
   self.assertEqual(result['scene_sha256'],hashlib.sha256(path.read_bytes()).hexdigest())
   self.assertEqual(result['external_asset_dependencies'],['shader.mdl','texture.png'])
   self.assertEqual(calls,[('flatten',),('export',str(path))])

 def test_virtual_views_match_review_calls_and_use_actual_aperture(self):
  cfg=SimpleNamespace(focal_length=18.,horizontal_aperture=23.5,vertical_aperture=None,
                      horizontal_aperture_offset=.1,vertical_aperture_offset=0.,clipping_range=(.03,20.))
  views=capture.virtual_review_views(cfg);observed={}
  scene={name:SimpleNamespace(set_world_poses_from_view=lambda eye,target,name=name:observed.update({name:(eye[0],target[0])})) for name in review.NAMES}
  with patch.dict('sys.modules',{'torch':SimpleNamespace(tensor=lambda values,**kwargs:values)}):
   review.place(SimpleNamespace(scene=scene,device='cpu'),(.32,0.,.82))
  self.assertFalse(views['camera_observations']);self.assertEqual(views['kind'],'virtual_offline_view_configuration')
  self.assertEqual(views['optics']['horizontal_aperture_mm'],23.5);self.assertIsNone(views['optics']['vertical_aperture_mm'])
  for name,(eye,target) in observed.items():
   self.assertEqual(views['views'][name]['eye_world_m'],list(eye));self.assertEqual(views['views'][name]['target_world_m'],list(target))
  tree=ast.parse(Path(review.__file__).read_text())
  camera=next(n for n in ast.walk(tree) if isinstance(n,ast.Call) and ast.unparse(n.func)=='CameraCfg')
  values={keyword.arg:ast.literal_eval(keyword.value) for keyword in camera.keywords if keyword.arg in ('height','width')}
  self.assertEqual(values,{'height':views['height'],'width':views['width']})

 def test_manifest_preserves_existing_first_row_time_and_never_claims_observations(self):
  value=capture.capture_manifest({'scene_sha256':'abc'},{'owner':{}},{'run.py':'sourcehash'},{'camera_observations':False},no_video=True)
  self.assertEqual(value['first_time_s'],.05);self.assertEqual(value['capture_hz'],20.)
  self.assertEqual(value['source_sha256'],{'run.py':'sourcehash'});self.assertEqual(value['quaternion_order'],'xyzw')
  self.assertFalse(value['interpolated']);self.assertFalse(value['media_eligible']);self.assertTrue(value['state_only'])


class StateOnlyConfigTests(unittest.TestCase):
 def test_optional_camera_flag_preserves_physical_kwargs_and_default_review_behavior(self):
  for no_video in (False,True):
   calls=[];original_kwargs={'enable_cameras':True,'gravity_compensation':True,'wrist_cam_mount':'link','robot_contact_offset':.001}
   def init(instance,*args,**kwargs):calls.append(('init',args,kwargs))
   review_cfg=SimpleNamespace(add_cameras=lambda instance:calls.append(('review',)))
   capture.configure_embodiment(object(),('arg',),original_kwargs,init,review_cfg,no_video=no_video)
   self.assertEqual(calls[0][2],{**original_kwargs,'enable_cameras':not no_video,'cam_width':320,'cam_height':180})
   self.assertEqual(original_kwargs['enable_cameras'],True)
   self.assertEqual([call[0] for call in calls],['init'] if no_video else ['init','review'])

 def test_render_suppression_is_flag_only_and_rejects_accidentally_constructed_cameras(self):
  rendered=[];base=SimpleNamespace(scene=SimpleNamespace(sensors={}),sim=SimpleNamespace(render=lambda:rendered.append('render')))
  capture.suppress_state_only_rendering(base,no_video=False);base.sim.render();self.assertEqual(rendered,['render'])
  capture.suppress_state_only_rendering(base,no_video=True);base.sim.render();self.assertEqual(rendered,['render'])
  base.scene.sensors={'left_cam':object()}
  with self.assertRaises(RuntimeError):capture.suppress_state_only_rendering(base,no_video=True)

 def test_runner_disables_app_cameras_and_guards_all_image_io_before_reset(self):
  tree=ast.parse(Path(__file__).with_name('run.py').read_text());guarded_calls=[];unguarded=[]
  def walk(node,guard=False):
   if isinstance(node,ast.If) and ast.unparse(node.test)=='not a.no_video':
    for child in node.body:walk(child,True)
    for child in node.orelse:walk(child,guard)
    return
   if isinstance(node,ast.Call):
    name=ast.unparse(node.func)
    if name in ('review.place','review.panels','imageio.get_writer','imageio.imwrite','video.append_data'):
     (guarded_calls if guard else unguarded).append(name)
   for child in ast.iter_child_nodes(node):walk(child,guard)
  walk(tree);self.assertFalse(unguarded)
  self.assertEqual(set(guarded_calls),{'review.place','review.panels','imageio.get_writer','imageio.imwrite','video.append_data'})
  calls=[node for node in ast.walk(tree) if isinstance(node,ast.Call)]
  app=next(node for node in calls if ast.unparse(node.func)=='AppLauncher')
  self.assertEqual(ast.unparse(next(k.value for k in app.keywords if k.arg=='enable_cameras')),'not a.no_video')
  suppress=next(node for node in calls if ast.unparse(node.func)=='capture.suppress_state_only_rendering')
  reset=next(node for node in calls if ast.unparse(node.func)=='env._env.reset')
  self.assertLess(suppress.lineno,reset.lineno)


if __name__=='__main__':unittest.main()
