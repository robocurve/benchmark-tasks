"""Model the toy cat and toy bird from signed distance functions, unwrap them and bake their colours. Our own meshes and textures, no third-party asset.

Writes animals/<name>.npz (the skin: v, f, uv, n) and textures/<name>_color.jpg. Offline: pure Python, no simulator. The task loads only the results.

Usage: python make_animals.py [cat] [bird]

The cat stands in a hunting crouch, facing +x: belly low, head low and level with the shoulders, shoulder blades standing above the back, elbows out, hindquarters raised with
the hind legs gathered under them, tail low and straight out behind, its tip a separate small part. Life size: 46 cm from nose to rump. The task scales it.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image

from sdfmesh import bake, chain, cone, ellipsoid, flat_cone, polygonise, smin, smooth_noise

HERE = Path(__file__).resolve().parent


def smoothstep(a: float, b: float, x: np.ndarray) -> np.ndarray:
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def mix(a, b, t: np.ndarray) -> np.ndarray:
    return np.asarray(a, float) * (1 - t[..., None]) + np.asarray(b, float) * t[..., None]


# -- the cat --------------------------------------------------------------------------------


HEAD = np.array([0.214, 0.0, 0.082])
MUZZLE = HEAD + (0.044, 0.0, -0.015)
NOSE = HEAD + (0.063, 0.0, -0.005)
EYE_OFFSET = np.array([0.0345, 0.0215, 0.0085])  # from the head's centre, mirrored in y
EYE_R = 0.0092
SPINE = [(-0.165, 0.158), (-0.100, 0.134), (-0.030, 0.098), (0.040, 0.075), (0.100, 0.064)]
SPINE_R = [0.054, 0.058, 0.056, 0.060, 0.054]
TAIL = [(-0.190, 0.0, 0.168), (-0.240, -0.008, 0.138), (-0.290, -0.020, 0.098), (-0.340, -0.045, 0.062), (-0.390, -0.080, 0.040), (-0.430, -0.115, 0.030), (-0.450, -0.145, 0.030), (-0.460, -0.170, 0.042)]
TAIL_R = [0.0175, 0.0170, 0.0160, 0.0150, 0.0140, 0.0130, 0.0120, 0.0100]
TAIL_PIVOT = 4  # the tail tip, from this point on, is its own small part, which twitches about the vertical through this point


def fore_leg(p: np.ndarray, side: float, step: float) -> np.ndarray:
    """Planted: shoulder, elbow up and out behind it, the forearm down and forward to the paw. `step` moves the paw forward."""
    s = side
    upper = chain(p, [(0.085, s * 0.038, 0.104), (0.030, s * 0.078, 0.106)], [0.030, 0.024])
    fore = chain(p, [(0.030, s * 0.078, 0.106), (0.078 + step * 0.6, s * 0.074, 0.052), (0.118 + step, s * 0.062, 0.024)], [0.0235, 0.0185, 0.0150], 0.008)
    paw = ellipsoid(p, (0.138 + step, s * 0.060, 0.0145), (0.026, 0.0195, 0.0145))
    return smin(smin(upper, fore, 0.012), paw, 0.008)


def reaching_leg(p: np.ndarray, side: float) -> np.ndarray:
    """Lifted off the table and reaching forward past the head, toward the prey."""
    s = side
    upper = chain(p, [(0.085, s * 0.038, 0.100), (0.085, s * 0.072, 0.090)], [0.030, 0.024])
    fore = chain(p, [(0.085, s * 0.072, 0.090), (0.190, s * 0.082, 0.070), (0.268, s * 0.080, 0.056)], [0.0235, 0.0185, 0.0150], 0.008)
    paw = ellipsoid(p, (0.290, s * 0.080, 0.052), (0.027, 0.0205, 0.0150), pitch=np.radians(12.0))
    return smin(smin(upper, fore, 0.012), paw, 0.008)


def hind_leg(p: np.ndarray, side: float) -> np.ndarray:
    """The hindquarters gathered: a heavy thigh forward and down to the knee, the shin back to the hock, the foot flat and forward under the belly."""
    s = side
    thigh = chain(p, [(-0.155, s * 0.040, 0.156), (-0.078, s * 0.072, 0.082)], [0.052, 0.033])
    shin = chain(p, [(-0.078, s * 0.072, 0.082), (-0.182, s * 0.064, 0.058)], [0.0270, 0.0160])
    foot = chain(p, [(-0.182, s * 0.064, 0.058), (-0.122, s * 0.062, 0.018), (-0.070, s * 0.060, 0.0135)], [0.0155, 0.0150, 0.0135], 0.006)
    paw = ellipsoid(p, (-0.058, s * 0.060, 0.0140), (0.025, 0.0190, 0.0140))
    return smin(smin(smin(thigh, shin, 0.018), foot, 0.008), paw, 0.008)


def scapula(p: np.ndarray, side: float) -> np.ndarray:
    return chain(p, [(0.090, side * 0.034, 0.078), (0.076, side * 0.031, 0.142)], [0.034, 0.0215], 0.0)


def ear(p: np.ndarray, side: float) -> np.ndarray:
    base = HEAD + (-0.002, side * 0.0290, 0.020)
    tip = HEAD + (0.002, side * 0.0720, 0.0740)
    return flat_cone(p, base, tip, 0.0290, 0.0100, (1.0, -side * 0.10, 0.75))


def cat_groups(p: np.ndarray) -> dict[str, np.ndarray]:
    spine3 = [(x, 0.0, z) for x, z in SPINE]
    torso = chain(p, spine3, SPINE_R, 0.02)
    torso = smin(torso, scapula(p, 1.0), 0.020)
    torso = smin(torso, scapula(p, -1.0), 0.020)
    neck = chain(p, [(0.105, 0.0, 0.076), (0.170, 0.0, 0.082)], [0.048, 0.039])
    head = ellipsoid(p, HEAD, (0.0475, 0.0445, 0.0385))
    head = smin(head, ellipsoid(p, MUZZLE, (0.0235, 0.0245, 0.0165)), 0.010)
    head = smin(head, ellipsoid(p, HEAD + (0.018, 0.031, -0.012), (0.024, 0.017, 0.018)), 0.010)  # cheeks
    head = smin(head, ellipsoid(p, HEAD + (0.018, -0.031, -0.012), (0.024, 0.017, 0.018)), 0.010)
    head = smin(head, ellipsoid(p, MUZZLE + (0.002, 0.0, -0.014), (0.016, 0.014, 0.0085)), 0.006)  # chin
    return {
        "torso": torso,
        "neck": neck,
        "head": head,
        "ear_l": ear(p, 1.0),
        "ear_r": ear(p, -1.0),
        "fore_l": reaching_leg(p, 1.0),
        "fore_r": fore_leg(p, -1.0, 0.012),
        "hind_l": hind_leg(p, 1.0),
        "hind_r": hind_leg(p, -1.0),
        "tail": chain(p, TAIL, TAIL_R, 0.012),
    }


def cat_field(p: np.ndarray) -> np.ndarray:
    g = cat_groups(p)
    d = smin(g["torso"], g["neck"], 0.028)
    d = smin(d, g["head"], 0.018)
    d = smin(d, g["ear_l"], 0.004)
    d = smin(d, g["ear_r"], 0.004)
    for n in ("fore_l", "fore_r"):
        d = smin(d, g[n], 0.014)
    for n in ("hind_l", "hind_r"):
        d = smin(d, g[n], 0.020)
    return smin(d, chain(p, TAIL[: TAIL_PIVOT + 1], TAIL_R[: TAIL_PIVOT + 1], 0.012), 0.012)


def tail_tip_field(p: np.ndarray) -> np.ndarray:
    return chain(p, TAIL[TAIL_PIVOT:], TAIL_R[TAIL_PIVOT:], 0.0)


COAT = np.array([0.47, 0.44, 0.40])  # a grey tabby
STRIPE = np.array([0.075, 0.068, 0.062])
WHITE = np.array([0.93, 0.91, 0.87])
PINK = np.array([0.80, 0.50, 0.47])


def _bars(phase: np.ndarray, width: float = 0.30) -> np.ndarray:
    """1 on a stripe, 0 between stripes: `width` is the stripe's share of the period."""
    c = np.cos(2 * np.pi * phase)
    edge = np.cos(np.pi * width)
    return smoothstep(edge - 0.18, edge + 0.18, c)


def cat_colour(p: np.ndarray, n: np.ndarray) -> np.ndarray:
    g = cat_groups(p)
    names = list(g)
    which = np.argmin(np.stack([g[k] for k in names], axis=0), axis=0)
    in_group = {k: which == i for i, k in enumerate(names)}
    warp = smooth_noise(p, 3, 0.15, 2)
    fine = smooth_noise(p, 4, 0.035, 2)
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    # Mackerel stripes down the flanks, leaning back, breaking up toward the belly.
    bars = _bars(x / 0.058 + 0.10 * (z - 0.08) / 0.04 + 0.75 * warp + 0.16 * fine, 0.30)
    breaks = smoothstep(-0.25, 0.25, smooth_noise(p, 9, 0.05, 2) + 0.15)  # stripes break into patches
    dorsal = smoothstep(0.016, 0.006, np.abs(y) + 0.005 * fine) * smoothstep(0.06, 0.10, z)
    stripe = np.maximum(bars * breaks * 0.78, dorsal * 0.8)
    # Legs and tail: rings. Head: bars back from the brow.
    legs = in_group["fore_l"] | in_group["fore_r"] | in_group["hind_l"] | in_group["hind_r"]
    stripe = np.where(legs, 0.7 * _bars((z + 0.35 * x) / 0.042 + 0.30 * warp, 0.30), stripe)
    tail = in_group["tail"]
    along = np.hypot(x + 0.19, y)
    stripe = np.where(tail, 0.85 * _bars(along / 0.052 + 0.10 * warp, 0.38), stripe)
    head = in_group["head"] | in_group["neck"]
    rel = p - HEAD
    brow = _bars(y / 0.0200 + 0.5 + 0.05 * warp, 0.30) * smoothstep(0.004, 0.022, rel[:, 2]) * smoothstep(0.040, 0.014, rel[:, 0]) * smoothstep(-0.045, -0.015, rel[:, 0])
    cheek = _bars(rel[:, 2] / 0.0190 + 0.15 * warp, 0.26) * smoothstep(0.030, 0.040, np.abs(y)) * smoothstep(0.02, 0.0, rel[:, 2]) * smoothstep(-0.02, 0.0, rel[:, 0])
    stripe = np.where(head, np.maximum(brow, cheek) * 0.8, stripe)
    rgb = mix(COAT * (0.82 + 0.20 * smoothstep(0.03, 0.12, z))[..., None], STRIPE, stripe * 0.94)
    # White: bib and belly, muzzle, paws.
    belly = smoothstep(0.052, 0.036, z + 0.006 * warp) * (~legs) * (~tail)
    bib = smoothstep(0.055, 0.030, np.linalg.norm((p - (0.135, 0.0, 0.050)) * (0.65, 1.0, 1.0), axis=1) + 0.006 * warp) * (~tail)
    muzzle = smoothstep(0.034, 0.024, np.linalg.norm((p - (MUZZLE + (0.002, 0.0, -0.002))) * (1.0, 0.95, 1.0), axis=1) + 0.002 * warp) * head
    paws = np.maximum(smoothstep(0.040, 0.028, z + 0.004 * warp), smoothstep(0.055, 0.040, np.linalg.norm(p - (0.290, 0.080, 0.052), axis=1) + 0.004 * warp)) * legs
    white = np.maximum.reduce([belly, bib, muzzle, paws])
    rgb = mix(rgb, WHITE, white)
    # Dark around the eyes and a dark tail tip.
    tip = smoothstep(0.26, 0.31, along) * tail
    rgb = mix(rgb, STRIPE, tip)
    for s in (1.0, -1.0):
        rim = smoothstep(EYE_R + 0.0055, EYE_R + 0.0015, np.linalg.norm(p - (HEAD + EYE_OFFSET * (1, s, 1)), axis=1)) * head
        rgb = mix(rgb, STRIPE, rim)
    # The nose, and the insides of the ears.
    nose = smoothstep(0.0085, 0.0055, np.linalg.norm((p - NOSE) * (1.0, 1.0, 1.25), axis=1))
    rgb = mix(rgb, PINK, nose)
    ears = in_group["ear_l"] | in_group["ear_r"]
    inner = smoothstep(0.10, 0.40, n[:, 0] + 0.45 * n[:, 2]) * smoothstep(0.070, 0.056, rel[:, 2]) * ears
    rgb = np.where(ears[..., None], mix(mix(COAT * 0.8, STRIPE, smoothstep(0.052, 0.068, rel[:, 2])), PINK * 1.08, inner), rgb)
    grain = smooth_noise(p, 5, 0.012, 2) * 0.11 + smooth_noise(p, 6, 0.004, 2) * 0.09
    return np.clip(rgb * (1.0 + grain)[..., None], 0.0, 1.0)


# -- the bird -------------------------------------------------------------------------------


BIRD_HEAD = np.array([0.030, 0.0, 0.0745])
BIRD_EYE = BIRD_HEAD + (0.0090, 0.0118, 0.0045)
BIRD_EYE_R = 0.0030


def bird_groups(p: np.ndarray) -> dict[str, np.ndarray]:
    """A canary on a perch, facing +x, 13 cm long, its feet at the origin."""
    body = ellipsoid(p, (0.002, 0.0, 0.042), (0.038, 0.0255, 0.0265), pitch=np.radians(-38.0))
    head = ellipsoid(p, BIRD_HEAD, (0.0195, 0.0172, 0.0172))
    beak = chain(p, [BIRD_HEAD + (0.0145, 0.0, -0.002), BIRD_HEAD + (0.0285, 0.0, -0.0045)], [0.0072, 0.0010])
    wing_l = ellipsoid(p, (-0.012, 0.0225, 0.040), (0.0360, 0.0048, 0.0150), pitch=np.radians(-30.0))
    wing_r = ellipsoid(p, (-0.012, -0.0225, 0.040), (0.0360, 0.0048, 0.0150), pitch=np.radians(-30.0))
    tail = flat_cone(p, (-0.028, 0.0, 0.030), (-0.095, 0.0, 0.002), 0.0145, 0.0032, (0.0, 0.0, 1.0))
    legs = np.minimum(chain(p, [(0.002, 0.009, 0.022), (0.002, 0.009, 0.0)], [0.0028, 0.0022]), chain(p, [(0.002, -0.009, 0.022), (0.002, -0.009, 0.0)], [0.0028, 0.0022]))
    return {"body": smin(body, head, 0.012), "beak": beak, "wings": np.minimum(wing_l, wing_r), "tail": tail, "legs": legs}


def bird_field(p: np.ndarray) -> np.ndarray:
    g = bird_groups(p)
    d = smin(g["body"], g["wings"], 0.002)
    d = smin(d, g["tail"], 0.005)
    d = smin(d, g["beak"], 0.001)
    return smin(d, g["legs"], 0.003)


def bird_colour(p: np.ndarray, n: np.ndarray) -> np.ndarray:
    g = bird_groups(p)
    names = list(g)
    which = np.argmin(np.stack([g[k] for k in names], axis=0), axis=0)
    in_group = {k: which == i for i, k in enumerate(names)}
    yellow = np.array([0.98, 0.80, 0.06])
    pale = np.array([1.00, 0.88, 0.25])
    dark = np.array([0.30, 0.27, 0.10])
    rgb = mix(yellow, pale, smoothstep(-0.5, 0.8, smooth_noise(p, 7, 0.018, 2)))
    feather = _bars(p[:, 0] / 0.0115 + 0.5 * p[:, 2] / 0.0115, 0.40)
    wing = mix(mix(yellow, dark, smoothstep(-0.010, -0.040, p[:, 0])), dark, feather * smoothstep(0.012, -0.020, p[:, 0]))
    rgb = np.where(in_group["wings"][..., None], wing, rgb)
    rgb = np.where(in_group["tail"][..., None], mix(yellow * 0.85, dark, smoothstep(-0.045, -0.070, p[:, 0])), rgb)
    rgb = np.where(in_group["beak"][..., None], np.array([0.93, 0.62, 0.34]), rgb)
    rgb = np.where(in_group["legs"][..., None], np.array([0.72, 0.48, 0.42]), rgb)
    return np.clip(rgb * (1.0 + 0.07 * smooth_noise(p, 8, 0.005, 2))[..., None], 0.0, 1.0)


# -- out ------------------------------------------------------------------------------------


def write(name: str, field, colour, bounds, voxel: float, faces: int, tex: int, eyes, eye_r: float) -> None:
    v, f = polygonise(field, bounds, voxel, faces)
    vv, idx, uv, img, n = bake(v, f, colour, tex)
    os.makedirs(HERE / "animals", exist_ok=True)
    os.makedirs(HERE / "textures", exist_ok=True)
    np.savez_compressed(HERE / "animals" / f"{name}.npz", v=vv.astype(np.float32), f=idx.astype(np.int32), uv=uv.astype(np.float32), n=n.astype(np.float32), eyes=np.asarray(eyes, np.float32).reshape(-1, 3), eye_r=np.float32(eye_r), pivot=np.asarray(TAIL[TAIL_PIVOT], np.float32))
    Image.fromarray(img).save(HERE / "textures" / f"{name}_color.jpg", quality=88)
    print(name, len(idx), "faces,", (HERE / "animals" / f"{name}.npz").stat().st_size // 1024, "KB,", (HERE / "textures" / f"{name}_color.jpg").stat().st_size // 1024, "KB; extents", np.ptp(vv, axis=0).round(3))


def main() -> None:
    which = sys.argv[1:] or ["cat", "bird"]
    if "cat" in which:
        write("cat", cat_field, cat_colour, ((-0.49, -0.24, -0.002), (0.35, 0.14, 0.23)), 0.0028, 26000, 1024, [HEAD + EYE_OFFSET, HEAD + EYE_OFFSET * (1, -1, 1)], EYE_R)
    if "cat" in which:
        write("cat_tail", tail_tip_field, cat_colour, ((-0.49, -0.21, 0.0), (-0.36, -0.05, 0.075)), 0.0012, 1800, 256, [], 0.0)
    if "bird" in which:
        write("bird", bird_field, bird_colour, ((-0.11, -0.04, -0.004), (0.07, 0.04, 0.10)), 0.0011, 6000, 512, [BIRD_EYE, BIRD_EYE * (1, -1, 1)], BIRD_EYE_R)


if __name__ == "__main__":
    main()
