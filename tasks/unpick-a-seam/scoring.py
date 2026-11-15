"""Visible seam-opening rubric; a cut command alone earns no credit."""
import math


def assess(*,released_stitches,blade_crossed_fabric,
           maximum_new_tear_m,complete,thread_bridge_present,tools_clear,material_valid=False):
    if (type(released_stitches) is not int or not 0<=released_stitches<=6
            or type(maximum_new_tear_m) not in (float,int)
            or not math.isfinite(maximum_new_tear_m) or maximum_new_tear_m<0
            or not all(type(v) is bool for v in (blade_crossed_fabric,complete,thread_bridge_present,tools_clear,material_valid))):
        return {'score':0.,'reason':'invalid measurements'}
    if not complete or not material_valid or blade_crossed_fabric or maximum_new_tear_m>.002:
        return {'score':0.,'reason':'incomplete evidence or fabric damage'}
    if released_stitches==6 and thread_bridge_present:
        return {'score':0.,'reason':'inconsistent seam-opening evidence: a joined stitch remains'}
    if released_stitches==6 and tools_clear:
        return {'score':1.,'reason':'all six stitches visibly open, fabric intact and tool released'}
    score=.75 if released_stitches==6 else .5 if released_stitches>=4 else .25 if released_stitches>=2 else 0.
    return {'score':score,'reason':'visibly open stitches confirmed in final inspection'}
