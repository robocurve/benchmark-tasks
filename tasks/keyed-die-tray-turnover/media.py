"""Stack the three original cameras at native timing."""
import subprocess
import numpy as np
from PIL import Image

def image_at(run,frame):
    def camera(cam):
        directory=run/'frames'
        files=list(directory.rglob(f'*_{cam}_cam_{frame:06d}.npy'))
        if len(files)!=1:
            raise ValueError(f"Missing or multiple images: {cam}, {frame}")
        return Image.fromarray(np.load(files[0]))
    return combine(camera('top'),camera('left'),camera('right'))


def combine(top,left,right):
    panel=Image.new('RGB',(640,528))
    panel.paste(top.resize((640,352),Image.Resampling.BILINEAR),(0,0))
    panel.paste(left,(0,352))
    panel.paste(right,(320,352))
    return panel


def export_video(run,path,total):
    frames=list((run/'frames').glob('*'))
    if len(frames)!=1:
        raise ValueError("Expected one episode")
    first=next(frames[0].glob('*_top_cam_000000.npy'))
    stem=first.name.removesuffix('_top_cam_000000.npy')
    files=[[frames[0]/f'{stem}_{cam}_cam_{i:06d}.npy' for cam in ['top','left','right']] for i in range(total)]
    settings=['ffmpeg','-hide_banner','-loglevel','error','-nostdin','-y','-f','rawvideo','-pix_fmt','rgb24',
              '-video_size','640x528','-framerate','20','-i','pipe:0','-c:v','libx264','-crf','19',
              '-pix_fmt','yuv420p','-movflags','+faststart',str(path)]
    with subprocess.Popen(settings,stdin=subprocess.PIPE) as writer:
        for triple in files:
            cams=[Image.fromarray(np.load(x)) for x in triple]
            panel=combine(*cams)
            writer.stdin.write(panel.tobytes())
        writer.stdin.close()
        if writer.wait()!=0:
            raise RuntimeError("Video encoding failed")


