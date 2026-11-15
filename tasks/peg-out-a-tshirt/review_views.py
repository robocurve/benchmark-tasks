"""Fixed native judging cameras; their images never enter a policy input."""
import numpy as np

CONFIGS=[('judging_front',(-.35,-.65,1.38),(.35,0.,.95),(0.,0.,1.),30.,(960,720)),
         ('judging_rear',(.90,.60,1.42),(.46,0.,1.00),(0.,0.,1.),35.,(960,720))]


def configure(embodiment):
    """Register with the native YAM renderer before the environment is built."""
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
    embodiment.camera_config=make_configclass('ShirtJudgingCameraCfg',fields,bases=(ArenaCameraCfg,))()


def images(base,*,warnings=None,step=None):
    """Native pixels for assessor recording only; no resizing or policy input."""
    result={}
    for name,eye,target,up,focal,resolution in CONFIGS:
        value=base.scene[name].data.output['rgb'];value=getattr(value,'torch',value)
        pixels=value.detach().cpu().numpy()[0] if hasattr(value,'detach') else np.asarray(value)[0]
        if pixels.ndim!=3 or pixels.shape[:2]!=(resolution[1],resolution[0]) or pixels.shape[2]<3:
            raise RuntimeError(f'Malformed native judging image: {name}')
        spread=float(np.std(pixels[:,:,:3]))
        if spread<2. and warnings is not None:
            warnings.append(dict(step=step,camera=name,pixel_standard_deviation=spread,
                issue='Low-variance native frame, possibly occluded; retained unchanged for review'))
        result[name]=np.ascontiguousarray(pixels[:,:,:3],dtype=np.uint8)
    return result


def metadata():
    return [dict(name=name,eye_m=eye,target_m=target,resolution=resolution,policy_input=False)
            for name,eye,target,up,focal,resolution in CONFIGS]
