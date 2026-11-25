"""Compute final image-error recovery from ordinary RGB photographs."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image

def evaluate(folder):
    folder=Path(folder)
    mask=np.asarray(Image.open(folder/'roi_mask.png').convert('L'))>0
    if int(mask.sum())<1000:raise ValueError('Measurement ROI is too small')
    names=['clean_reference','dirty_before','after_wiping','clean_repeat','dirty_repeat']
    images={n:np.asarray(Image.open(folder/(n+'.png')).convert('RGB'),dtype=np.float64)/255. for n in names}
    if any(im.shape!=images['clean_reference'].shape for im in images.values()):raise ValueError('Measurement images have different dimensions')
    mse={n:float(np.mean((images[n][mask]-images['clean_reference'][mask])**2)) for n in names}
    if mse['dirty_before']<=max(10*mse['clean_repeat'],1e-8):raise ValueError('Initial degradation is below the measurement acceptance threshold')
    recovery=1-mse['after_wiping']/mse['dirty_before']
    return {'metric':'fixed-ROI RGB MSE to clean reference','roi_pixels':int(mask.sum()),'mse':mse,'recovery_fraction_raw':recovery,'threshold':.9,'image_quality_milestone_passed':bool(recovery>=.9),'dirty_repeat_apparent_recovery':1-mse['dirty_repeat']/mse['dirty_before']}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');p.add_argument('--out');a=p.parse_args();result=evaluate(a.folder)
    out=Path(a.out) if a.out else Path(a.folder)/'image_quality_score.json';out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
