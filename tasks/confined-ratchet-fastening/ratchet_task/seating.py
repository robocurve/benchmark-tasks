"""Surface-specific seating from fresh PhysX contact reports, in SI units.

No simulator imports: the adapter supplies live poses and exact collider IDs.
Normal contacts alone count; friction anchors and cached/found-contact events
without a compressive impulse cannot establish a loaded seat.
"""
import math
import numpy as np
from ratchet_task.config import (
    HEAD_AF, BRACKET_TOP, BRACKET_BORE_RADIUS, BRACKET_SECTORS,
    SEAT_ROUNDOFF, SEAT_MAX_PENETRATION, SEAT_MAX_EDGE_GAP, SEAT_MIN_FORCE,
)

HEAD_XY = HEAD_AF / math.sqrt(3) * np.array([
    [math.cos(a), math.sin(a)] for a in np.arange(6)*math.pi/3+math.pi/6])
HEX_NORMALS = np.array([[math.cos(a), math.sin(a)] for a in np.arange(3)*math.pi/3])


def seat_geometry(bolt, bolt_rotation, fixture, fixture_rotation):
    """Head underside heights relative to the bracket's live top plane."""
    underside = bolt + np.c_[HEAD_XY, np.zeros(6)] @ bolt_rotation.T
    local = (underside-fixture) @ fixture_rotation
    gaps = local[:, 2]-BRACKET_TOP
    centre = float(((bolt-fixture) @ fixture_rotation)[2]-BRACKET_TOP)
    return dict(seat_gap_m=centre, seat_gap_min_m=float(gaps.min()),
                seat_gap_max_m=float(gaps.max()))


def measure_seat(headers, contacts, *, head_id, bracket_ids, bolt,
                 bolt_rotation, fixture, fixture_rotation, dt):
    """Accept only head-underside versus bracket-top load-bearing contacts.

    IDs identify exact shapes, not any contact involving the bolt actor. The
    point must lie on both seating planes and inside the head footprint, outside
    the bracket hole. The normal must align with both seating normals; signed
    separation and whole-head geometry reject speculative contacts/penetration.
    """
    out = seat_geometry(bolt, bolt_rotation, fixture, fixture_rotation)
    out.update(seat_contact=False, seat_force_n=0., seat_contact_points=0,
               seat_separation_min_m=None, seat_separation_max_m=None,
               seat_contact_pairs=[])
    geometry_ok = (-SEAT_MAX_PENETRATION <= out['seat_gap_min_m'] and
                   out['seat_gap_m'] <= SEAT_ROUNDOFF and
                   out['seat_gap_max_m'] <= SEAT_MAX_EDGE_GAP)
    normal = fixture_rotation[:, 2]
    for header in headers:
        pair = (int(header.collider0), int(header.collider1))
        if pair[0] == head_id and pair[1] in bracket_ids:
            bracket_id = pair[1]
        elif pair[1] == head_id and pair[0] in bracket_ids:
            bracket_id = pair[0]
        else:
            continue
        for c in contacts[header.contact_data_offset:header.contact_data_offset+header.num_contact_data]:
            point, cn, impulse = map(lambda v: np.asarray(v, dtype=float), (c.position, c.normal, c.impulse))
            sep = float(c.separation)
            if not np.all(np.isfinite(np.r_[point, cn, impulse, sep])):
                continue
            bp = (point-bolt) @ bolt_rotation
            fp = (point-fixture) @ fixture_rotation
            on_head = (abs(bp[2]) <= SEAT_MAX_PENETRATION and
                       np.max(np.abs(HEX_NORMALS @ bp[:2])) <= HEAD_AF/2+SEAT_ROUNDOFF)
            on_bracket = (abs(fp[2]-BRACKET_TOP) <= SEAT_MAX_PENETRATION and
                          np.linalg.norm(fp[:2]) >= BRACKET_BORE_RADIUS*math.cos(math.pi/BRACKET_SECTORS)-SEAT_ROUNDOFF and
                          abs(fp[0]) <= .07+SEAT_ROUNDOFF and abs(fp[1]) <= .045+SEAT_ROUNDOFF)
            aligned = (abs(float(cn @ normal)) >= math.cos(math.radians(5)) and
                       abs(float(cn @ bolt_rotation[:, 2])) >= math.cos(math.radians(5)))
            force = abs(float(impulse @ normal))/dt
            if not (geometry_ok and on_head and on_bracket and aligned and
                    -SEAT_MAX_PENETRATION <= sep <= SEAT_ROUNDOFF and force >= SEAT_MIN_FORCE):
                continue
            out['seat_contact_points'] += 1
            out['seat_force_n'] += force
            lo, hi = out['seat_separation_min_m'], out['seat_separation_max_m']
            out['seat_separation_min_m'] = sep if lo is None else min(lo, sep)
            out['seat_separation_max_m'] = sep if hi is None else max(hi, sep)
            path = bracket_ids[bracket_id]
            if path not in out['seat_contact_pairs']:
                out['seat_contact_pairs'].append(path)
    out['seat_contact'] = out['seat_contact_points'] > 0
    return out


class SeatWindow:
    """One control interval, including zero-contact physics steps."""
    def __init__(self):
        self.samples = []

    def add(self, sample):
        self.samples.append(sample)

    def summary(self, clear=True):
        rows = self.samples
        out = dict(rows[-1]) if rows else dict(seat_contact=False, seat_force_n=0.)
        out['seat_contact_fraction'] = sum(r['seat_contact'] for r in rows)/len(rows) if rows else 0.
        out['seat_force_mean_n'] = sum(r['seat_force_n'] for r in rows)/len(rows) if rows else 0.
        out['seat_physics_samples'] = len(rows)
        if clear:
            self.samples = []
        return out
