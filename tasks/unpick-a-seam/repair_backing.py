"""Passive repair-board candidate; no cloth targets or attachments.

A rear board and front guide rails restrict large normal buckling while the
26 mm seam window stays open. These are ordinary fixed solids connected to
the bench stand by the listed foot, side posts and standoffs. They do not
clamp, weld or pin the fabric; the lower patch can slide sideways out of them.
"""
import numpy as np

WINDOW_Z=(.7997,.8257)
BOARD_WIDTH=.065


def fixture_boxes():
    """Names, dimensions and world centres of the actual guide solids."""
    return [
        ('clamp_repair_backing',(.003,.065,.1152),(.3033,0.,.8051)),
        ('clamp_repair_foot',(.0382,.065,.005),(.3209,0.,.7500)),
        ('clamp_front_upper_rail',(.003,.060,.036),(.2971,0.,.8437)),
        ('clamp_front_lower_rail',(.003,.060,.0472),(.2979,0.,.7761)),
        ('clamp_front_upper_mount',(.003,.100,.004),(.2971,0.,.8597)),
        ('clamp_front_lower_mount',(.003,.100,.004),(.2979,0.,.7545)),
        ('clamp_left_top_standoff',(.020,.006,.004),(.3056,.047,.8597)),
        ('clamp_right_top_standoff',(.020,.006,.004),(.3056,-.047,.8597)),
        ('clamp_left_low_standoff',(.020,.006,.004),(.3064,.047,.7545)),
        ('clamp_right_low_standoff',(.020,.006,.004),(.3064,-.047,.7545)),
        ('clamp_left_repair_post',(.008,.006,.1152),(.312,.047,.8051)),
        ('clamp_right_repair_post',(.008,.006,.1152),(.312,-.047,.8051)),
    ]


def description():
    return {
        'mechanism':'Fixed rear sewing board and front guide rails, connected to the bolted bench stand',
        'physical_material':'Rigid board and metal guides; ordinary frictional contact',
        'seam_window_z_m':list(WINDOW_Z),
        'rear_board_front_x_m':.3018,
        'rear_board_width_m':BOARD_WIDTH,
        'cloth_edge_overhang_each_side_m':(.080-BOARD_WIDTH)/2,
        'front_upper_rest_clearance_m':.0008,
        'front_lower_rest_clearance_m':.0008,
        'rear_lower_rest_clearance_m':.0012,
        'cloth_attachments':False,
        'lower_patch_can_slide_sideways':True,
        'native_validation':'pending',
    }


def initial_clearance(meshes,thread_nodes,radius_m=.0004):
    """Conservative reset-only AABB distances; not a dynamic validity check."""
    clouds=[(np.asarray(mesh.rest),np.array((.0002,0.,0.))) for mesh in meshes]
    clouds.extend((np.asarray(nodes),np.full(3,radius_m)) for nodes in thread_nodes)
    minimum=float('inf')
    for _,size,position in fixture_boxes():
        low=np.asarray(position)-np.asarray(size)/2
        high=np.asarray(position)+np.asarray(size)/2
        for points,pad in clouds:
            # Cloth is planar normal to X. Its actual thickness pads X only;
            # the thread is conservatively padded in all directions.
            cloud_low=points.min(0)-pad;cloud_high=points.max(0)+pad
            gap=np.maximum(np.maximum(low-cloud_high,cloud_low-high),0.)
            distance=float(np.linalg.norm(gap))
            if distance<=1e-12:raise ValueError('Repair board intersects authored cloth or thread envelope')
            minimum=min(minimum,distance)
    return minimum
