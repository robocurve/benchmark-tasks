"""Native RGB planar alignment for the checked near-overhead wrist pose.

This detector uses coloured physical references, never simulator object poses.
It is specialized to this plate and does not establish a general visual policy.
"""
import cv2
import numpy as np


def points_on_plane(pixels,cam_pos,cam_rot,lens,height):
    """Intersect native-RGB rays with commanded feature or fixed board height.

    No hidden object pose is read. Separate planes account for parallax between
    references on top of the plate and marks on the board beneath it.
    """
    pixels=np.asarray(pixels,dtype=float)
    rays=np.column_stack(((pixels[:,0]-lens.cx*.5)/(lens.fx*.5),
                          -(pixels[:,1]-lens.cy*176/360)/(lens.fy*176/360),
                          -np.ones(len(pixels))))@np.asarray(cam_rot).T
    if np.any(abs(rays[:,2])<1e-6):raise ValueError('View rays are parallel to registration plane')
    distances=(height-cam_pos[2])/rays[:,2]
    if np.any(distances<=0):raise ValueError('Registration plane is behind camera')
    return np.asarray(cam_pos)+distances[:,None]*rays


def alignment_world_error(measurement,cam_pos,cam_rot,lens,window_height,target_height):
    windows=points_on_plane(measurement['window_centres'],cam_pos,cam_rot,lens,window_height)
    targets=points_on_plane(measurement['target_centres'],cam_pos,cam_rot,lens,target_height)
    error=(targets-windows).mean(0);error[2]=0.
    return error


def alignment(rgb):
    if rgb is None or np.asarray(rgb).shape != (176,320,3):
        return dict(valid=False,reason='missing native RGB')
    rgb=np.asarray(rgb,dtype=float)
    if not np.isfinite(rgb).all():return dict(valid=False,reason='nonfinite RGB')
    r,g,b=rgb.transpose(2,0,1)
    pink=((r>1.5*g)&(r>1.3*b)&(b>1.3*g)&(r>25)).astype('uint8')
    n,labels,stats,centres=cv2.connectedComponentsWithStats(pink)
    points=np.array([c for s,c in zip(stats[1:],centres[1:]) if 3<=s[4]<=100])
    if len(points)!=8:return dict(valid=False,reason='need eight visible pink corners',pink_count=len(points))
    points=points[np.argsort(points[:,0])];groups=[points[:4],points[4:]]
    if groups[1][:,0].min()-groups[0][:,0].max()<8:
        return dict(valid=False,reason='window groups ambiguous')
    cyan=(r<.5*g)&(g>.35*b)&(g>35)&(b>50)
    windows=[];targets=[];areas=[]
    for corners in groups:
        lo=np.floor(corners.min(0)).astype(int);hi=np.ceil(corners.max(0)).astype(int)
        if min(hi-lo)<5:return dict(valid=False,reason='window references too small')
        mask=np.zeros(cyan.shape,dtype='uint8')
        mask[max(0,lo[1]):min(176,hi[1]+1),max(0,lo[0]):min(320,hi[0]+1)]=1
        ys,xs=np.nonzero(cyan&mask.astype(bool))
        if not 4<=len(xs)<=150:return dict(valid=False,reason='target absent or ambiguous',target_pixels=len(xs))
        windows.append(corners.mean(0));targets.append([xs.mean(),ys.mean()]);areas.append(len(xs))
    windows=np.array(windows);targets=np.array(targets)
    return dict(valid=True,window_centres=windows.tolist(),target_centres=targets.tolist(),
                target_pixels=areas,error_px=(targets-windows).mean(0).tolist(),
                per_window_error_px=(targets-windows).tolist())
