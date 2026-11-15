"""Fixed native judging cameras; their images never enter a policy input."""
import numpy as np

CONFIGS=[('judging_key',(.650,.270,1.010),(.351,.170,.925),(0.,0.,1.),50.,(1280,720)),
         ('judging_gauge',(.480,-.150,.900),(.360,0.,.800),(0.,0.,1.),65.,(1280,720))]


def configs(fixture_layout='reference'):
    from fixture_layout import offset
    delta=offset(fixture_layout)
    return [(name,tuple(np.asarray(eye)+delta),tuple(np.asarray(target)+delta),up,focal,resolution)
            for name,eye,target,up,focal,resolution in CONFIGS]


def configure(embodiment,fixture_layout='reference'):
    """Register with the native YAM renderer before the environment is built."""
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import CameraCfg
    from isaaclab_arena.utils.cameras import ArenaCameraCfg
    from isaaclab_arena.utils.configclass import make_configclass
    from pxr import Gf
    from yam_arena.embodiment import RENDERER
    fields=[(name,CameraCfg,value) for name,value in vars(embodiment.camera_config).items()
            if isinstance(value,CameraCfg)]
    for name,eye,target,up,focal,resolution in configs(fixture_layout):
        q=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(*up)).GetInverse().ExtractRotationQuat()
        config=CameraCfg(prim_path='{ENV_REGEX_NS}/'+name,height=resolution[1],width=resolution[0],
            data_types=['rgb'],spawn=sim_utils.PinholeCameraCfg(focal_length=focal,horizontal_aperture=20.955,
                clipping_range=(.005,10.)),offset=CameraCfg.OffsetCfg(pos=eye,
                rot=(*q.GetImaginary(),q.GetReal()),convention='opengl'),renderer_cfg=RENDERER)
        fields.append((name,CameraCfg,config))
    embodiment.camera_config=make_configclass('WheelJudgingCameraCfg',fields,bases=(ArenaCameraCfg,))()


def capture_final(base,output,fixture_layout='reference'):
    from PIL import Image
    metadata=[]
    for name,eye,target,up,focal,resolution in configs(fixture_layout):
        value=base.scene[name].data.output['rgb']
        value=getattr(value,'torch',value)
        pixels=value.detach().cpu().numpy()[0] if hasattr(value,'detach') else np.asarray(value)[0]
        if pixels.shape[:2]!=(resolution[1],resolution[0]) or pixels.ndim!=3:
            raise RuntimeError(f'Missing or blank native judging image: {name}')
        Image.fromarray(pixels[:,:,:3].astype(np.uint8)).save(output/(name.replace('_','-')+'.png'))
        metadata.append({'name':name,'eye_m':eye,'target_m':target,'resolution':resolution,'policy_input':False,'pixel_standard_deviation':float(np.std(pixels[:,:,:3]))})
    return metadata


def composite_frame(base):
    """One simultaneous review frame, downsampled from three native cameras.

    The robot view provides context; close views make key engagement and the rim gauge
    inspectable. This never changes observations supplied to a policy.
    """
    from PIL import Image
    canvas=Image.new('RGB',(1280,720),(24,24,24))
    for name,size,position in (
            ('top_cam',(640,360),(0,0)),
            ('judging_gauge',(640,360),(0,360)),
            ('judging_key',(640,360),(640,180))):
        value=getattr(base.scene[name].data.output['rgb'],'torch',base.scene[name].data.output['rgb'])
        pixels=value.detach().cpu().numpy()[0,:,:,:3].astype(np.uint8)
        # A moving arm may fully occlude a close camera with a uniform surface.
        # Keep that genuine frame. Only missing/malformed output is an error;
        # final unobstructed photographs receive a separate content check.
        if pixels.ndim!=3 or pixels.shape[2]!=3 or min(pixels.shape[:2])<1:
            raise RuntimeError(f'Missing review camera output: {name}')
        canvas.paste(Image.fromarray(pixels).resize(size,Image.Resampling.LANCZOS),position)
    return np.asarray(canvas)
