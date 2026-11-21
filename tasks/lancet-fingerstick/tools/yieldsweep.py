"""yieldsweep.py: does the trainer's unheld ring finger give way before the lancet reaches a force F, for every press
the scorer allows? Quasi-static, along the finger's slide (no sim).

The ring finger rides on a vertical slide behind its knuckle (lancet_fingerstick.hand_parts), pushed up against its
stop by a spring with RING_PRELOAD to spare; it has given way once it is pushed DROP_MAX down, which takes give_force()
of downward force on it. A press is allowed when the shield's face is on the site (site_ok) and the lancet's axis d
is within TILT_MAX of the skin's inward normal n (the square rule, against the local normal at that point). The
shield's spring carries the axial load, so the contact force C on the skin has C . d = F (the force on the shield).
Its direction c is set by friction, coefficient up to mu, in either contact the shield can make: its flat face on the
skin (c within atan(mu) of d) or its rim on the skin (c within atan(mu) of n). So C = F c / (c . d), and its downward
part, the part the slide feels, is F k with k = -c_z / (c . d). The least k over all allowed presses is k_min: every
allowed press at force F gives the finger way when F k_min > give_force(), and the difference is the margin (N).

The sweep: the ring fingertip, s every 0.5 mm over SITE_S, the angle round every 2.5 deg over SITE_ANG on either side
of the pad's middle line (the hand palm up and pitched PITCH, so the site's normals lean 25 to 62 deg from vertical);
d on rings of TILT_MAX/3, 2/3 and 1 round n (24 directions each, plus n); c on rings of atan(mu)/3, 2 atan(mu)/3 and
atan(mu) round d and round n (24 each, plus d and n). WHY it holds only with no friction: a press 62 deg from vertical,
tilted 10 deg more, with friction 0.25 or more, can push the finger along or up its slide rather than down, so the
slide alone does not score every one-handed press 0; the scorer's held rule (finger_held) does.

    python3 tools/yieldsweep.py          # the table the README quotes
"""
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np  # noqa: E402
import lancet_fingerstick as F  # noqa: E402

S_STEP, ANG_STEP, NA, NR = 0.0005, math.radians(2.5), 24, 3
MUS = (0.0, 0.25, 0.5, 1.0)


def cone(n, amax, na=NA, nr=NR):
    """Unit directions: n, and nr rings out to amax round it, na each."""
    n = np.asarray(n, float) / np.linalg.norm(n)
    if amax <= 0.0:
        return n[None]
    e1 = np.cross(n, [0.0, 1.0, 0.0])
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(n, [1.0, 0.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    ph = 2 * math.pi * np.arange(na) / na
    out = [n[None]]
    for a in amax * np.arange(1, nr + 1) / nr:
        out.append(math.cos(a) * n + math.sin(a) * (np.cos(ph)[:, None] * e1 + np.sin(ph)[:, None] * e2))
    return np.vstack(out)


def site_point(s, ang):
    """A point on the ring's distal pad (mechanism frame) s back from its apex, ang round from its middle line."""
    v, u0, _, rd, _ = F.FINGERS["ring"]
    rho = math.sqrt(rd * rd - (rd - s) ** 2) if s < rd else rd
    return F.hand_to_mech((u0 + s, v + rho * math.sin(ang), rho * math.cos(ang)))


def sites(side=0):
    """(s, ang, point, inward normal) over the site; side +1 is the thumb's side of the pad's middle line, -1 the
    other, 0 both."""
    if side == 0:
        return sites(-1) + sites(1)
    out = []
    for s in np.arange(F.SITE_S[0], F.SITE_S[1] + 1e-9, S_STEP):
        for a in np.arange(F.SITE_ANG[0], F.SITE_ANG[1] + 1e-9, ANG_STEP):
            p = site_point(float(s), side * float(a))
            out.append((float(s), side * float(a), p, F.site_of(p)[3]))
    return out


def k_site(n, mu, tilt=F.TILT_MAX):
    """The least downward force on the finger per newton of shield force over every allowed press at one site."""
    am = math.atan(mu)
    Cn = cone(n, am)
    best = math.inf
    for d in cone(n, tilt):
        C = np.vstack([Cn, cone(d, am)])
        cd = C @ d
        ok = cd > 1e-9
        best = min(best, float((-C[ok, 2] / cd[ok]).min()))
    return best


def give_force():
    """N: the downward force that pushes the unheld finger DROP_MAX down its slide."""
    return F.RING_PRELOAD + F.K_RING * F.DROP_MAX


def sweep(mu, side=0, tilt=F.TILT_MAX):
    """(k_min, (s mm, ang deg)) over the whole site."""
    worst = (math.inf, None)
    for s, a, _, n in sites(side):
        k = k_site(n, mu, tilt)
        if k < worst[0]:
            worst = (k, (round(1000 * s, 1), round(math.degrees(a), 1)))
    return worst


if __name__ == "__main__":
    g = give_force()
    print(f"the unheld finger gives {1000 * F.DROP_MAX:.0f} mm at {g:.2f} N down (preload {F.RING_PRELOAD:g} N, "
          f"spring {F.K_RING:g} N/m)")
    for mu in MUS:
        k, w = sweep(mu)
        way = (f"every allowed press gives it way by {g / k:.2f} N on the shield" if k > 0 else
               "some allowed presses push it up, not down")
        print(f"mu {mu}: k_min {k:.4f} at (s mm, ang deg) {w}; {way}; margin at F_FIRE {F.F_FIRE:g} N "
              f"{F.F_FIRE * k - g:+.2f} N")
    n = F.R_HAND @ np.array([0.0, math.sin(F.SITE_TURN), math.cos(F.SITE_TURN)])
    print(f"the demo's site ({1000 * F.SITE_AIM:g} mm, {math.degrees(F.SITE_TURN):.0f} deg): its normal "
          f"{math.degrees(math.acos(n[2])):.1f} deg from vertical")
