"""Scripted manipulation skills shared by task oracles (pick/place of rigid objects,
button presses). Generators of 14-D actions built on ``robosci.motion.Motion``."""

from __future__ import annotations

from collections.abc import Iterator

import numpy as np

from robosci.motion import Motion

GRIP_BOX = 0.005  # command past the 8 cm faces: ~20 N squeeze (force-limited)
GRASP_DZ = 0.004  # grasp just above the object centre: no gravity torque about the finger axis
RELAY_SPOTS = ((0.27, -0.215), (0.30, -0.25), (0.24, -0.25), (0.27, -0.28), (0.22, -0.20), (0.25, -0.12))


def body_pos(env, body: str) -> np.ndarray:  # noqa: ANN001
    return env.data.xpos[env.model.body(body).id].copy()


def grasp_point(env, body: str, dz: float = GRASP_DZ) -> np.ndarray:  # noqa: ANN001
    return body_pos(env, body) + np.array([0.0, 0.0, dz])


def drop_point(xy, surface_z: float, half: float) -> np.ndarray:  # noqa: ANN001
    """Target for the object centre: resting on the surface, released 4 mm above it."""
    return np.array([xy[0], xy[1], surface_z + half + 0.004])


def _plan(mo: Motion, arms: tuple[str, ...], grasp, center, target):  # noqa: ANN001, ANN202
    for arm in arms:
        plan = mo.plan_pick_place(arm, grasp, center, target)
        if plan is not None:
            return arm, plan
    return None


def move_body(env, mo: Motion, body: str, side: str, xy, surface_z: float, *,  # noqa: ANN001
              half: float, grip: float = GRIP_BOX, dz: float = GRASP_DZ,
              relay: bool = True, home: bool = True, open_to: float | None = None) -> Iterator[np.ndarray]:
    """Carry a free body upright to ``xy`` on a surface at height ``surface_z``.

    ``half`` is the body's half-height. ``open_to`` caps the finger opening (slide, m;
    gap = 2 x slide) for objects with close neighbours. ``side`` is the preferred arm; falls back to the
    other arm, then (``relay``) to two carries through a free table spot."""
    grasp = grasp_point(env, body, dz)
    center = body_pos(env, body)
    target = drop_point(xy, surface_z, half)
    arms = (side, "right" if side == "left" else "left")
    direct = _plan(mo, arms, grasp, center, target)
    if direct is None:
        if relay:
            for spot in RELAY_SPOTS:
                mid = drop_point(spot, 0.0, half)
                if _plan(mo, arms, grasp, center, mid) and _plan(mo, arms, mid + [0, 0, dz - 0.004], mid, target):
                    yield from move_body(env, mo, body, side, spot, 0.0, half=half, grip=grip, dz=dz, relay=False, open_to=open_to)
                    yield from move_body(env, mo, body, side, xy, surface_z, half=half, grip=grip, dz=dz, relay=False, open_to=open_to)
                    return
        raise RuntimeError(f"no arm can move {body} to {np.round(target, 3)}")
    arm, plan = direct
    kw = {} if open_to is None else {"open_to": open_to}
    yield from mo.pick(arm, grasp, grip, body=body, plan=plan, **kw)
    yield from mo.place(arm, target, center=True)
    if home:
        yield from mo.home(arm)


def noop(env, rng: np.random.Generator | None = None) -> Iterator[np.ndarray]:  # noqa: ANN001
    mo = Motion(env)
    yield from mo.hold(10.0)
