"""Draft physical judging measures, computed from the final deposited surface.

No model events, commanded nozzle poses or intended shell centres earn credit.
The corresponding ruler/photo checks are documented in JUDGING.md. Thresholds
are provisional until a physical reference and an actual robot demo exist.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np


def _peaks(values, *, minimum, spacing, prominence, eligible=None):
    """Position-independent peaks; plateaus collapse to their centre."""
    a=np.asarray(values,float)
    candidates=[]
    i=1
    while i<len(a)-1:
        j=i
        while j+1<len(a) and abs(a[j+1]-a[i])<1e-10:j+=1
        if a[i]>=minimum and a[i]>a[i-1] and j+1<len(a) and a[j]>a[j+1]:
            k=(i+j)//2
            window=a[max(0,k-spacing):min(len(a),k+spacing+1)]
            if (a[k]-float(window.min())>=prominence
                    and (eligible is None or eligible(k))):candidates.append(k)
        i=max(i+1,j+1)
    chosen=[]
    for k in sorted(candidates,key=lambda k:a[k],reverse=True):
        if all(abs(k-j)>=spacing for j in chosen):chosen.append(k)
    return sorted(chosen)


def _head_height(profile,width,k,dx):
    region=np.arange(max(0,k-round(.002/dx)),min(len(profile),k+round(.004/dx)+1))
    region=region[width[region]>=.75*width[k]]
    return float(profile[region].max())


def _footprint_components(mask):
    """Visible icing connected across finite grid-cell edges, not projection.

    A corner-only touch has zero contact width and cannot join a tail. Detached
    specks remain separate; they do not invalidate a neighboring real shell.
    """
    mask=np.asarray(mask,bool);seen=np.zeros(mask.shape,dtype=bool)
    for row,column in np.argwhere(mask):
        if seen[row,column]:continue
        pending=[(int(row),int(column))];seen[row,column]=True;cells=[]
        while pending:
            i,j=pending.pop();cells.append((i,j))
            for x,y in ((i-1,j),(i+1,j),(i,j-1),(i,j+1)):
                if 0<=x<mask.shape[0] and 0<=y<mask.shape[1] and mask[x,y] and not seen[x,y]:
                    seen[x,y]=True;pending.append((x,y))
        component=np.zeros(mask.shape,dtype=bool)
        indices=np.asarray(cells);component[indices[:,0],indices[:,1]]=True
        yield component


def _connected_candidates(surface,dx):
    """Measure each candidate using one connected head-and-tail footprint."""
    spacing=max(1,round(.006/dx));candidates=[]
    for component in _footprint_components(surface>=.0005):
        a=np.where(component,surface,0.)
        profile=a.max(axis=0);width=component.sum(axis=0)*dx
        heads=_peaks(width,minimum=.006,spacing=spacing,prominence=.002,
                     eligible=lambda k:width[k]>.016 or _head_height(profile,width,k,dx)>=.0025)
        for n,k in enumerate(heads):
            stop=heads[n+1] if n+1<len(heads) else surface.shape[1]-1
            start=max(0,k-round(.006/dx))
            # A remote connection outside this shell cannot connect its head
            # to an otherwise separate tail inside the measured interval.
            for local in _footprint_components(component[:,start:stop+1]):
                if local[:,k-start].sum()*dx<.006:continue
                one=np.zeros_like(surface)
                one[:,start:stop+1]=np.where(local,surface[:,start:stop+1],0.)
                candidates.append((k,stop,one))
    # Head spacing applies across components as well as within one component.
    chosen=[]
    for item in sorted(candidates,key=lambda item:(item[2][:,item[0]]>=.0005).sum(),reverse=True):
        if all(abs(item[0]-other[0])>=spacing for other in chosen):chosen.append(item)
    return sorted(chosen,key=lambda item:item[0])


def _ridge_length(surface, first, last, dx):
    """Continuous visible ridge pair anywhere on the measured shell body.

    The ridges may run down its tapered shoulder. Track every qualifying pair,
    so a third crest appearing between them does not erase two real ridges.
    Neighboring crests may move at most twice the cross-section spacing.
    """
    previous=[];longest=0.
    for column in range(max(0,first),min(surface.shape[1],last+1)):
        v=surface[:,column]
        peaks=_peaks(v,minimum=.001,spacing=max(1,round(.0015/dx)),prominence=.0003)
        current=[]
        for i,left in enumerate(peaks):
            for right in peaks[i+1:]:
                if min(v[left],v[right])-float(v[left:right+1].min())<.0003:continue
                length=dx+max((length for a,b,length in previous
                    if abs(a-left)*dx<=2*dx+1e-12
                    and abs(b-right)*dx<=2*dx+1e-12),default=0.)
                current.append((left,right,length));longest=max(longest,length)
        previous=current
    return longest


def assess(x,y,volume,*,dx,emitted_m3,lost_m3=0.,airborne_m3=0.,damage_depth_m=0.,
           completed=False,tools_clear=False,other_tool_contact=False,_reverse_checked=False):
    try:x,y,volume=(np.asarray(a,float) for a in (x,y,volume))
    except (ValueError,TypeError):return {"score":0.,"reason":"invalid numeric surface data"}
    numbers=(dx,emitted_m3,lost_m3,airborne_m3,damage_depth_m)
    if (not all(type(v) in (float,int) and np.isfinite(v) and v>=0 for v in numbers)
            or not all(type(v) is bool for v in (completed,tools_clear,other_tool_contact))
            or x.ndim!=1 or y.ndim!=1 or len(x)<3 or len(y)<3 or dx<=0):
        return {"score":0.,"reason":"invalid grid or judgment types"}
    if (volume.shape!=(len(x),len(y)) or not all(np.isfinite(a).all() for a in (x,y,volume))
            or np.any(volume<0) or not np.allclose(np.diff(x),dx,rtol=1e-7,atol=1e-10)
            or not np.allclose(np.diff(y),dx,rtol=1e-7,atol=1e-10)):
        return {"score":0.,"reason":"invalid or incomplete material telemetry"}
    h=volume/dx**2
    X,Y=np.meshgrid(x,y,indexing="ij")
    target=(np.abs(X-.235)<=.009)&(Y>=-.047)&(Y<=.047)
    spill_area=float(((h>=.0005)&~target).sum()*dx**2)
    result={"score":0.,"status":"geometric assessment; complete trial and contact evidence required",
            "valid_shells":0,"shells":[],"spill_area_mm2":spill_area*1e6,
            "damage_mm":damage_depth_m*1e3}
    balance_error=emitted_m3-float(volume.sum())-lost_m3-airborne_m3
    result['volume_balance_error_m3']=balance_error
    if abs(balance_error)>1e-9:
        result['reason']='deposited, airborne and lost material do not match measured extrusion'
        return result
    if not completed or damage_depth_m>.001 or spill_area>20e-6 or lost_m3>1e-9 or other_tool_contact:
        result["reason"]="incomplete evidence, cake/tool damage, spill, or unobserved lost material"
        return result
    cross=np.abs(x-.235)<=.009
    active=(y>=-.047)&(y<=.047)
    if cross.sum()<3 or active.sum()<3:
        return {"score":0.,"reason":"material grid does not resolve the marked strip"}
    yy=y[active];a=h[cross][:,active]
    # Find shapes from the actual final surface, without fixed shell centres.
    # A low shoulder on a tail is not a distinct head under the written rubric.
    # Exclude it before spacing suppression and tail segmentation. Oversized
    # or over-height mounds remain separators: a shell cannot borrow a tail
    # beyond an intervening invalid deposit.
    covered_by_valid_shell=np.zeros(len(yy),dtype=bool)
    for k,stop,shape_surface in _connected_candidates(a,dx):
        profile=shape_surface.max(axis=0)
        width=(shape_surface>=.0005).sum(axis=0)*dx
        # A later disconnected deposit cannot supply this head's tail. End the
        # candidate interval at the first break in the visible footprint.
        following=np.arange(k+1,stop+1)
        breaks=following[profile[following]<.0005]
        if len(breaks):stop=int(breaks[0]-1)
        lower=k+max(1,round(.004/dx));upper=min(stop,k+round(.022/dx))
        if lower>=upper:continue
        candidates=np.arange(lower,upper+1)
        candidates=candidates[profile[candidates]>=.0005]
        if not len(candidates):continue
        head_w=float(width[k])
        head_h=_head_height(profile,width,k,dx)
        # A shell may finish in a thinner pointed tip. Inspect its qualifying
        # tapered section, not just the single narrowest occupied grid column.
        qualifying=candidates[(width[candidates]>=.001)&(width[candidates]<=.65*head_w)&
                              (profile[candidates]>=.0005)&
                              (profile[candidates]<=min(.003,.65*head_h))]
        # The rule asks whether a qualifying tail section exists. A long tail
        # can have several equally narrow columns; choosing the first minimum
        # incorrectly truncates it and can turn an elongated shell into a mound.
        tail=int(qualifying[-1]) if len(qualifying) else int(candidates[np.argmin(width[candidates])])
        tail_h,tail_w=float(profile[tail]),float(width[tail])
        tapered=(.0025<=head_h<=.008 and .006<=head_w<=.016 and
                 .0005<=tail_h<=min(.003,.65*head_h) and .001<=tail_w<=.65*head_w)
        # Locate the leading rise in the measured footprint. Union the observed
        # intervals so adjacent shapes cannot double-count the same length.
        previous=np.arange(max(0,k-round(.006/dx)),k+1)
        narrow=previous[width[previous]<=.65*head_w]
        beginning=int(narrow[-1]) if len(narrow) else k
        length=float(yy[tail]-yy[beginning])
        ridge_length=_ridge_length(shape_surface,beginning,tail,dx)
        ridged=ridge_length>=.002-1e-12
        # A round mound also narrows beyond its centre. Require a visibly
        # elongated footprint so that taper alone cannot certify a shell.
        tapered=tapered and length>=1.25*head_w
        if tapered:covered_by_valid_shell[beginning:tail+1]=True
        result["shells"].append({"valid":bool(tapered),"visible_ridges":bool(ridged),
                                  "continuous_ridge_length_mm":ridge_length*1e3,
                                  "head_y_mm":float(yy[k]*1e3),"head_height_mm":head_h*1e3,
                                  "head_width_mm":head_w*1e3,"tail_height_mm":tail_h*1e3,
                                  "tail_width_mm":tail_w*1e3,"length_mm":length*1e3})
    count=sum(s["valid"] for s in result["shells"])
    valid_length=float(covered_by_valid_shell.sum()*dx)
    result["valid_shells"]=count
    result["valid_decorated_length_mm"]=valid_length*1e3
    covered=(a>=.0005).any(axis=0)[(yy>=-.043)&(yy<=.043)]
    longest=run=0
    for c in covered:
        run=0 if c else run+1;longest=max(longest,run)
    result["largest_gap_mm"]=longest*dx*1e3
    result["score"]=(.75 if valid_length>=.060 else .5 if valid_length>=.040 else
                       .25 if valid_length>=.020 else 0.)
    full=(valid_length>=.080 and count>=5 and longest*dx<=.002 and tools_clear
          and airborne_m3<=1e-9 and all(s["visible_ridges"] for s in result["shells"] if s["valid"]))
    if full:result["score"]=1.
    result["reason"]="final material shape; ridge preservation and released, continuous finish required for full score"
    if not _reverse_checked:
        # The instruction permits either piping direction. Mirror the measured
        # surface around Y=0; neither its placement nor quality changes.
        reversed_result=assess(x,-y[::-1],volume[:,::-1],dx=dx,emitted_m3=emitted_m3,
                               lost_m3=lost_m3,airborne_m3=airborne_m3,damage_depth_m=damage_depth_m,
                               completed=completed,tools_clear=tools_clear,other_tool_contact=other_tool_contact,
                               _reverse_checked=True)
        if reversed_result['score']>result['score']:
            reversed_result['piping_direction']='-Y'
            return reversed_result
    result['piping_direction']='+Y'
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("state",type=Path)
    p.add_argument("--out",type=Path)
    a=p.parse_args()
    with np.load(a.state,allow_pickle=False) as s:
        r=assess(s["x"],s["y"],s["volume"],dx=float(s["dx"]),
                 emitted_m3=float(s["emitted_m3"]),lost_m3=float(s["lost_m3"]),
                 airborne_m3=float(s["airborne_m3"]),damage_depth_m=float(s["damage_depth_m"]))
    data=json.dumps(r,indent=2)+"\n"
    if a.out:a.out.write_text(data)
    else:print(data,end="")


if __name__=="__main__":main()
