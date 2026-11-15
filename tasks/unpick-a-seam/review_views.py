"""Fixed native review cameras, excluded from policy observations.

Adapted from the team's wheel/paper recorder. These cameras render the actual
simulated scene; they do not move objects, change contacts or infer success.
Coordinates below are in the original station frame, shifted with the world.
"""
import numpy as np


CONFIGS=[('judging_front',(.10,-.05,.86),(.299,0.,.8127),(0.,0.,1.),18.,(1280,720)),
         ('judging_detail',(.15,0.,.845),(.298,0.,.8127),(0.,0.,1.),45.,(1280,720))]


def configure(embodiment):
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import CameraCfg
    from isaaclab_arena.utils.cameras import ArenaCameraCfg
    from isaaclab_arena.utils.configclass import make_configclass
    from pxr import Gf
    from yam_arena.embodiment import RENDERER
    fields=[(name,CameraCfg,value) for name,value in vars(embodiment.camera_config).items()
            if isinstance(value,CameraCfg)]
    for name,eye,target,up,focal,resolution in CONFIGS:
        q=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(*up)).GetInverse().ExtractRotationQuat()
        config=CameraCfg(prim_path='{ENV_REGEX_NS}/'+name,height=resolution[1],width=resolution[0],
            data_types=['rgb'],spawn=sim_utils.PinholeCameraCfg(focal_length=focal,horizontal_aperture=20.955,
                clipping_range=(.005,10.)),offset=CameraCfg.OffsetCfg(pos=eye,
                rot=(*q.GetImaginary(),q.GetReal()),convention='opengl'),renderer_cfg=RENDERER)
        fields.append((name,CameraCfg,config))
    embodiment.camera_config=make_configclass('SeamJudgingCameraCfg',fields,bases=(ArenaCameraCfg,))()


def pixels(base,name):
    value=base.scene[name].data.output['rgb'];value=getattr(value,'torch',value)
    image=value.detach().cpu().numpy()[0] if hasattr(value,'detach') else np.asarray(value)[0]
    if image.ndim!=3 or image.shape[2]<3 or min(image.shape[:2])<1:
        raise RuntimeError('Malformed native review image: '+name)
    # Preserve real occlusions. A uniform frame must not terminate physics.
    return np.ascontiguousarray(image[:,:,:3],dtype=np.uint8)


def composite_frame(base):
    from PIL import Image
    canvas=Image.new('RGB',(1280,720),(24,24,24))
    for name,position in (('top_cam',(0,0)),('judging_front',(0,360)),('judging_detail',(640,180))):
        canvas.paste(Image.fromarray(pixels(base,name)).resize((640,360),Image.Resampling.LANCZOS),position)
    return np.asarray(canvas)


def capture_final(base,output):
    from PIL import Image
    metadata=[]
    for name,eye,target,up,focal,resolution in CONFIGS:
        image=pixels(base,name)
        if image.shape[:2]!=(resolution[1],resolution[0]):raise RuntimeError('Wrong native review resolution')
        Image.fromarray(image).save(output/(name.replace('_','-')+'.png'))
        metadata.append(dict(name=name,eye_station_m=eye,target_station_m=target,resolution=resolution,
                             policy_input=False,pixel_standard_deviation=float(np.std(image))))
    return metadata
