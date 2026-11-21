

# -- appended by sim/mk.sh to the stem copies only (o00 j00 x00 w00 f00 k00 c00 a00 g00 y00); the task file itself is
# unchanged.
# Each stem is the demo's own plan with one change, and its expected score (README Sim results; test_sim_stems replays
# each offline):
#   o00  one arm: the left stays parked, the right presses as the demo does (20 mm past contact over 1.5 s).
#        Expected 0, "not held still" (the unheld finger gives way down its slide).
#   j00  one arm, the same press as a 0.15 s jab. Expected 0, "not held still".
#   x00  one arm, a jab 30 mm past contact in one control step (0.05 s). WHY: a press fast enough may fire before the
#        light finger moves; the finger then gives way within 0.5 s of the click, and the window counts it.
#        Expected 0, "not held still" (or "no gripper holding the ring finger", also 0).
#   w00  one arm, a press nearer the apex (8.5 mm back) driven 10 deg off the lancet's axis toward the knuckle. WHY:
#        friction then tilts the contact force, the press the yield sweep finds hardest. Expected 0, "not held still"
#        (or "slid", also 0).
#   f00  both arms; after the twist the right presses the lancet down in the stand before lifting it. WHY: the real
#        lancet, its tab off, fires into the stand's post. In the sim it does not fire: the shield's face sinks into
#        the tab cap and the housing's front meets the cap with the shield 1.05 mm in, 5.2 N (body heights, README Sim
#        results; why the face sinks is not proven). Expected 1.0, the demo finished. The rule is
#        test_fired_in_the_stand_latches.
#   k00  both arms; after firing the right lowers the lancet into the container's opening and keeps holding it.
#        Expected 0.5: fired well, never let go.
#   c00  both arms; the tab twisted the other way (clockwise seen from above). Expected 1.0: either way breaks it.
#   a00  both arms; the left comes in closed to 5 mm, 26 mm off the ring finger's axis along its closing direction
#        (above the finger, on its palm side; its near pad's shell 5 mm from the skin: a gripper closed to 5 mm nearer
#        than 24 mm meets the finger), so it holds nothing, and the right jabs 30 mm past contact in one control step.
#        Expected 0, "not held still" (or "fired with no gripper holding the ring finger").
#   g00  both arms; the left closes on the ring finger but does not lift it against its stop. WHY: the press then
#        meets only the arm's stiffness and the slide's spring. Expected 0, "not held still".
#   y00  both arms, the demo's hold; the right jabs 60 mm past contact along the lancet's axis in one control step.
#        WHY: a legitimate hold and a jab past the hold's pull (the press 48 deg from vertical puts 0.66 of its force
#        down the slide, so 60 mm, about 27 N [EST: the arm's 0.5 N/mm], to pass the hold's 13.5 N). Expected 0, "not
#        held still".
# No index-finger stem (an earlier revision's i00): the left comes in from the ring finger's side of the hand, so the
# index lies beyond the ring finger from it, and no hold on the index misses the ring.
import dataclasses as _dc  # noqa: E402

_STEM = "none"
_JAB_S = 0.15
_X_S, _X_DZ = 0.05, 0.030
_Y_DZ = 0.060
_AIR_ACROSS, _AIR_GAP = 0.026, -0.005
_NEAR_S = 0.0085
_TILT = math.radians(10.0)
_KEEP_Z = TOP + 0.181            # the TCP lowered over the opening: the lancet's face 150 mm up, inside (lid 162 mm)
_PRESS = {"tilt over the site", "steady over the site", "just off the site", "steady just off the site",
          "tip on the site", "load the press", "settle the press", "press until it fires", "hold the press",
          "back off along the axis"}
_HOLD = {"beside the ring finger", "steady beside the ring finger", "in round the ring finger", "grip the ring finger",
         "hold it up on its stop", "keep holding through the press", "ease off", "let go of the ring finger",
         "back out"}


def _r(w, **kw):
    return _dc.replace(w, **{k: (tuple(float(x) for x in v) if k == "pos" else v) for k, v in kw.items()})


def _one_arm(right, seed):
    total = sum(w.duration for w in right)
    park, rot = park_pose(seed)
    return [_w(2.5, park, OPEN_F, "park", rot=rot), _w(total - 2.5, park, OPEN_F, "stay parked", rot=rot)]


def _jab(right, dz, seed):
    """The press made one control step deep, dz past contact along the lancet's axis."""
    site, a, _ = press_pose(seed)
    deep = site - tcp_over_lancet() * a + dz * a
    return [_r(w, duration=_X_S, pos=deep) if w.label == "press until it fires" else
            (_r(w, pos=deep) if w.label == "hold the press" else w) for w in right]


def _no_lift(left, seed):
    """The left's lift on the finger taken out: it stays where it closed."""
    g = hold_pose(seed)[0]
    return [_r(w, pos=g) if w.label in ("hold it up on its stop", "keep holding through the press") else w
            for w in left]


def _no_load(right):
    """The press's load and settle stages taken out (their time given to the tip on the site): a jab starts from
    contact, not from a settled 5 N."""
    out = []
    for w in right:
        if w.label in ("load the press", "settle the press"):
            out[-1] = _r(out[-1], duration=out[-1].duration + w.duration)
        else:
            out.append(w)
    return out


def _stem(right, left, seed):
    if _STEM in ("j", "x", "a", "y"):
        right = _no_load(right)
    if _STEM == "j":
        right = [_r(w, duration=_JAB_S) if w.label == "press until it fires" else w for w in right]
    if _STEM in ("x", "a"):
        right = _jab(right, _X_DZ, seed)
    if _STEM == "y":
        right = _jab(right, _Y_DZ, seed)
    if _STEM == "g":
        left = _no_lift(left, seed)
    if _STEM == "a":
        off = _AIR_ACROSS * hold_pose(seed)[2][:, 0]   # WHY above: below, its tips came within 2 mm of the web
        left = [_r(w, pos=np.asarray(w.pos) + off, finger=_AIR_GAP if w.finger in (GRIP_R, OPEN_R) else w.finger)
                if w.label in _HOLD else w for w in _no_lift(left, seed)]
    if _STEM == "w":
        site, a, _ = press_pose(seed)
        dp = press_pose(seed, _NEAR_S)[0] - site
        side = hand_world(seed, (1.0, 0.0, 0.0), direction=True) * PRESS_DZ * math.tan(_TILT)   # toward the knuckle
        right = [_r(w, pos=np.asarray(w.pos) + dp + (side if w.label in ("press until it fires", "hold the press")
                                                     else 0.0))
                 if w.label in _PRESS else w for w in right]
    if _STEM in ("o", "j", "x", "w"):
        left = _one_arm(right, seed)
    if _STEM == "f":
        out = []
        for w in right:
            out.append(w)
            if w.label == "hold":    # the twist is done: the tab is off, the lancet still in the stand
                out.append(_r(w, duration=1.5, pos=np.asarray(w.pos) - [0, 0, PRESS_DZ],
                              label="press it down in the stand"))
                out.append(_r(w, duration=0.6, label="back up in the stand"))   # 2.1 s in all
        right = out
        # WHY: the left waits as long again, so it still holds the finger through the right's press
        left = [_r(w, duration=w.duration + 2.1) if w.label == "wait parked" else w for w in left]
    if _STEM == "k":
        i = [w.label for w in right].index("let go")
        over = right[i - 1]
        right = right[:i] + [_r(over, duration=1.5, pos=(over.pos[0], over.pos[1], _KEEP_Z),
                                label="into the opening, held"),
                             _r(over, duration=4.0, pos=(over.pos[0], over.pos[1], _KEEP_Z),
                                label="keep holding it there")]
    return right, left


def demo(delay=0.0):  # noqa: F811
    seed = run_seed()
    right, left = _stem(*plan(delay, seed, -TWIST if _STEM == "c" else TWIST), seed)
    return {"right": right, "left": left}
