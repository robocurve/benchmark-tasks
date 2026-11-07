"""RGB-only entrance-view gate; not a 3-D clearance estimator or scoring input."""
import cv2
import numpy as np


def entrance_view(rgb):
    if not isinstance(rgb,np.ndarray) or rgb.shape!=(176,320,3) or rgb.dtype!=np.uint8:
        return dict(usable=False,reason='missing_or_invalid_rgb')
    r,g,b=np.moveaxis(rgb.astype(float),-1,0)
    pink=((r>45)&(r>1.8*g)&(r>1.25*b)&(b>1.4*g)&(b>15)).astype(np.uint8)
    cyan=((g>35)&(b>35)&(g>1.6*r)&(b>1.6*r)).astype(np.uint8)
    # Join small shaded fragments of one marker; never reject a nearby marker
    # just because its area exceeds the old diagnostic's80-pixel upper bound.
    grouped=cv2.morphologyEx(pink,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    count,labels,stats,centres=cv2.connectedComponentsWithStats(grouped)
    candidates=[i for i in range(1,count) if 4<=stats[i,cv2.CC_STAT_AREA]<=600]
    candidates=sorted(candidates,key=lambda i:stats[i,cv2.CC_STAT_AREA],reverse=True)[:4]
    result=dict(usable=False,reason='at_least_three_corners_required',corners=len(candidates))
    if len(candidates)<3:return result
    points=centres[candidates].astype(np.float32)
    inferred=False
    if len(points)==3:
        # Three observed corners of the rigid rectangular opening can bound the
        # missing fourth. Only use this view-quality approximation near end-on;
        # it is not a measured fourth marker or a recovered3-D clearance.
        pairs=[(i,j) for i in range(3) for j in range(i+1,3)]
        i,j=max(pairs,key=lambda pair:np.linalg.norm(points[pair[0]]-points[pair[1]]))
        k=next(k for k in range(3) if k not in (i,j));u=points[i]-points[k];v=points[j]-points[k]
        if abs(float(u@v))/(np.linalg.norm(u)*np.linalg.norm(v))>.25:return result
        points=np.vstack((points,points[i]+points[j]-points[k]));inferred=True
    hull=cv2.convexHull(points)
    if len(hull)!=4:return result
    area=float(cv2.contourArea(hull));w,h=cv2.minAreaRect(hull)[1]
    ratio=min(w,h)/max(w,h) if max(w,h)>0 else 0.
    mask=np.zeros(pink.shape,np.uint8);cv2.fillConvexPoly(mask,hull.astype(np.int32),1)
    inset=max(3,int(min(w,h)*.12));mask=cv2.erode(mask,np.ones((inset,inset),np.uint8))
    guide=int((mask*cyan).sum())
    usable=600<=area<=12000 and ratio>=.55 and guide>=20
    return dict(usable=usable,reason='visible_entrance_and_guide' if usable else 'oblique_or_hidden',
                corners=len(candidates),inferred_corner=inferred,area_px2=area,
                axis_ratio=ratio,guide_pixels_inside=guide)


class ViewGate:
    """Pause manipulation on lost view; require three fresh usable frames to resume.

    The predetermined observer transit is allowed while the collar is stationary.
    A persistently missing view stops progress rather than falling back to time.
    """
    def __init__(self):self.good=0;self.bad=0;self.blocked=True
    def update(self,rgb,required):
        measurement=entrance_view(rgb)
        self.good=self.good+1 if measurement['usable'] else 0
        self.bad=self.bad+1 if not measurement['usable'] else 0
        if self.good>=3:self.blocked=False
        if self.bad>=3:self.blocked=True
        return (not required or not self.blocked),measurement
