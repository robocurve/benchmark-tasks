"""Unvalidated contact-guided ordinary jaw controller for a scripted demo.

Native cloth positions choose finger targets only. No simulator mutator,
attachment, material edit or success signal is available. The fixed target
remains the lower aperture bound; feedback can relieve compression by opening.
This is not a Dream observation or a claim of force-sensor fidelity.
"""
import numpy as np
from interpret_motion import rotation_xyzw


def maximum_pad_node_depth(nodes, poses):
    """Depth inside the actual URDF convex collision hulls, not visual meshes."""
    from yam_arena.analysis import link_hulls
    nodes = np.asarray(nodes, float)
    if nodes.ndim != 2 or nodes.shape[1] != 3 or not np.isfinite(nodes).all():
        raise ValueError('Finite native cloth nodes required')
    deepest = 0.
    for name in ('tip_left', 'tip_right'):
        pose = np.asarray(poses[name], float)
        local = (nodes - pose[:3]) @ rotation_xyzw(pose[3:])
        for hull in link_hulls()[name]:
            low, high = hull.vertices.min(0), hull.vertices.max(0)
            candidates = local[np.all((local >= low) & (local <= high), axis=1)]
            if not len(candidates):
                continue
            signed = (candidates @ hull.planes[:, :3].T - hull.planes[:, 3]).max(1)
            deepest = max(deepest, float(max(0., -signed.min())))
    return deepest


class ContactPinch:
    """Unvalidated, bounded jaw relief with slower closure after first contact.

    A caller must hold the ordinary arm pose while establishing contact; the
    fixed trajectory must not simply advance into its lift before this closes.
    The runner enables it only in an explicitly selected diagnostic. Native node overlap
    is a compression diagnostic, not a force measurement or a grasp certificate.
    """
    def __init__(self, close_slew=.003, open_slew=.006):
        if not 0. < close_slew <= .01 or not 0. < open_slew <= .02:
            raise ValueError('Use bounded normalized finger increments')
        self.close_slew = float(close_slew)
        self.open_slew = float(open_slew)
        self.targets = {}
        self.records = []

    def action(self, side, reference_opening, measured_opening, nodes, poses,
               step, *, release=False):
        if side not in ('left', 'right'):
            raise ValueError('Unknown hand')
        reference = float(reference_opening)
        measured = float(measured_opening)
        if not 0. <= reference <= 1. or not 0. <= measured <= 1.:
            raise ValueError('Finger values must use ordinary normalized units')
        depth = maximum_pad_node_depth(nodes, poses)
        first_contact = not release and side not in self.targets and depth > .0001
        if release:
            self.targets.pop(side, None)
            target = reference
        elif first_contact:
            # In recorded089 the first contact occurs at opening0.223, well
            # before the eventual pinch. Never clamp this to a smaller maximum:
            # doing so would command more closure precisely when relieving it.
            target = max(reference, measured)
            self.targets[side] = target
        elif side in self.targets:
            target = self.targets[side]
            if depth > .0003:
                target += min(self.open_slew, .5 * (depth - .00015) / .0939)
            elif depth < .0001:
                target -= self.close_slew
            target = min(1., max(reference, target))
            self.targets[side] = target
        else:
            target = reference
        self.records.append(dict(step=int(step), side=side,
            reference_opening=reference, measured_opening=measured,
            commanded_opening=target, max_pad_node_depth_m=depth,
            first_contact=first_contact, release=bool(release), state_writes=False))
        return target
