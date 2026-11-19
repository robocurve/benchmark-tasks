#!/usr/bin/env python3
"""Author bespoke Blender 5.2.3 LTS PBR .usdc visual meshes for reticle-gate-vernier-keeper.

Zero physics risk: every mesh is authored in its exact mechkit/labkit part-local frame over invisible PhysX colliders.
"""
from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, "/mnt/data/ops/swarm-top1-gate/pbr_kit")
import blender_pbr_lib as bp  # noqa: E402

PKG = pathlib.Path(sys.argv[sys.argv.index("--") + 1]).resolve()
SLUG = "reticle-gate-vernier-keeper"
OUT = PKG / "assets" / SLUG
TEX = OUT / "textures"

SX, SY = 0.268, 0.000
PIV_X, PIV_Y, PIV_Z = 0.252, 0.035, 0.020
KX, KY = 0.308, -0.181
HANDLE_Z = 0.065
GH_DY, GH_Z = 0.012, 0.074


def add_handle_rot(prefix: str, x: float, y: float, z: float, post_mat, k: dict, yaw: float = 0.0) -> None:
    cy, sy = math.cos(yaw), math.sin(yaw)

    def _p(lx: float, ly: float, lz: float):
        return (x + lx * cy - ly * sy, y + lx * sy + ly * cy, lz)

    bp.box(f"{prefix}_shelf", _p(0.0, 0.0, z - 0.019), (0.050, 0.018, 0.004), k["steel"], bevel=0.0007, yaw=yaw)
    bp.box(f"{prefix}_post", _p(0.0, 0.0, z), (0.024, 0.018, 0.034), post_mat, bevel=0.0010, yaw=yaw)
    bp.box(f"{prefix}_knurl", _p(0.0, 0.0, z), (0.018, 0.0184, 0.014), k["knurl"], bevel=0.0005, yaw=yaw, tile=0.024)
    for sx_s, tag in ((-1.0, "m"), (1.0, "p")):
        bp.box(f"{prefix}_cheek_{tag}", _p(sx_s * 0.0225, 0.0, z), (0.005, 0.014, 0.034), k["slate"], bevel=0.0008, yaw=yaw)
    bp.box(f"{prefix}_cap", _p(0.0, 0.0, z + 0.0185), (0.014, 0.014, 0.003), k["alu"], bevel=0.0006, yaw=yaw)
    bp.socket_screw(f"{prefix}_scr", _p(0.0, 0.0, z + 0.0205), k["steel"], k["black"], r_head=0.0022, h_head=0.0012)


def build_stage() -> None:
    bp.reset()
    k = bp.M()
    bp.rrect_prism("stage_base", 0.0, 0.0, 0.196, 0.284, 0.012, 0.000, 0.012, k["slate"], bevel=0.0014)
    bp.rrect_prism("stage_deck", 0.0, 0.0, 0.184, 0.272, 0.010, 0.012, 0.015, k["alu"], bevel=0.0009)

    for ix, x in enumerate((-0.091, 0.091)):
        for iy, y in enumerate((-0.135, 0.135)):
            yaw = math.pi / 4.0 if x * y > 0 else -math.pi / 4.0
            bp.box(f"corner_rib_{ix}_{iy}", (x, y, 0.007), (0.016, 0.004, 0.010), k["steel"], bevel=0.0006, yaw=yaw)
    for ix, x in enumerate((-0.083, 0.083)):
        for iy, y in enumerate((-0.124, 0.124)):
            bp.cyl(f"washer_{ix}_{iy}", (x, y, 0.0156), 0.0046, 0.0012, k["brass"], bevel=0.0003)
            bp.socket_screw(f"deck_scr_{ix}_{iy}", (x, y, 0.0170), k["steel"], k["black"], r_head=0.0027, h_head=0.0015)

    bp.tube("optical_viewport_bezel", (0.040, -0.090, 0.0168), 0.015, 0.0095, 0.0036, k["black"], verts=40, bevel=0.0005)
    bp.cyl("glass_condenser_lens", (0.040, -0.090, 0.0155), 0.0092, 0.0012, k["glass"], verts=36, bevel=0.0002)
    bp.box("dovetail_way_n", (0.001, -0.072, 0.020), (0.148, 0.006, 0.010), k["brass"], bevel=0.0007)
    bp.box("dovetail_way_s", (0.001, -0.108, 0.020), (0.148, 0.006, 0.010), k["brass"], bevel=0.0007)
    for sx_r in (-0.052, 0.000, 0.052):
        bp.socket_screw(f"way_scr_n_{sx_r}", (sx_r, -0.072, 0.0256), k["steel"], k["black"], r_head=0.0018, h_head=0.0011)
        bp.socket_screw(f"way_scr_s_{sx_r}", (sx_r, -0.108, 0.0256), k["steel"], k["black"], r_head=0.0018, h_head=0.0011)

    bp.box("datum_polarity_hi", (0.064, -0.085, 0.025), (0.008, 0.008, 0.020), k["red"], bevel=0.0008)
    bp.box("datum_polarity_lo", (0.067, -0.095, 0.025), (0.006, 0.008, 0.020), k["black"], bevel=0.0008)
    bp.decal("reticle_vernier_scale", (-0.012, -0.118, 0.0153), (0.092, 0.008), "top", k["scale"])

    px, py = PIV_X - SX, PIV_Y - SY
    bp.rrect_prism("hinge_pier_w", px - 0.014, py, 0.008, 0.024, 0.002, 0.015, 0.033, k["steel"], bevel=0.0008)
    bp.rrect_prism("hinge_pier_e", px + 0.014, py, 0.008, 0.024, 0.002, 0.015, 0.033, k["steel"], bevel=0.0008)
    bp.cyl("hinge_axle_pin", (px, py, PIV_Z), 0.0040, 0.036, k["brass"], axis="X", verts=28, bevel=0.0004)
    bp.box("gate_rest_pad", (px, -0.038 - SY, 0.0175), (0.016, 0.016, 0.005), k["black"], bevel=0.0006)

    bp.box("keeper_way_w", (KX - SX - 0.016, -0.149, 0.021), (0.006, 0.068, 0.012), k["brass"], bevel=0.0007)
    bp.box("keeper_way_e", (KX - SX + 0.016, -0.149, 0.021), (0.006, 0.068, 0.012), k["brass"], bevel=0.0007)
    bp.decal("keeper_vernier_scale", (KX - SX + 0.026, -0.130, 0.0153), (0.034, 0.008), "top", k["scale"])
    bp.export_usdc("stage")


def build_beam_gate() -> None:
    bp.reset()
    k = bp.M()
    bp.rrect_prism("gate_axle_hub", 0.0, 0.0, 0.018, 0.020, 0.003, PIV_Z - 0.004, PIV_Z + 0.004, k["blue"], bevel=0.0008)
    bp.box("gate_handle_riser", (0.0, GH_DY / 2.0, PIV_Z + 0.022), (0.016, 0.016, 0.036), k["blue"], bevel=0.0007)
    bp.box("shutter_beam_arm", (0.0, -0.048, PIV_Z + 0.004), (0.008, 0.100, 0.006), k["steel"], bevel=0.0006)
    bp.box("shutter_tooth", (0.0, -0.095, 0.024), (0.006, 0.014, 0.014), k["amber"], bevel=0.0007)
    bp.box("shutter_striker_cap", (0.0, -0.095, 0.032), (0.010, 0.012, 0.002), k["red"], bevel=0.0004)
    add_handle_rot("gate_handle", 0.0, GH_DY, GH_Z, k["blue"], k, yaw=-math.pi / 2.0)
    bp.export_usdc("beam_gate")


def build_reticle_carriage() -> None:
    bp.reset()
    k = bp.M()
    bp.rrect_prism("reticle_sled", 0.0, 0.0, 0.016, 0.014, 0.003, 0.020, 0.036, k["steel"], bevel=0.0008)
    bp.box("polarity_key_step", (0.012, 0.002, 0.028), (0.008, 0.006, 0.014), k["amber"], bevel=0.0006)
    bp.tube("reticle_cell_ring", (0.0, 0.0, 0.0375), 0.0065, 0.0042, 0.0032, k["black"], verts=36, bevel=0.0004)
    bp.cyl("glass_reticle_disc", (0.0, 0.0, 0.0372), 0.0040, 0.0012, k["glass"], verts=32, bevel=0.0002)
    bp.rrect_prism("reticle_riser", 0.0, 0.0, 0.014, 0.014, 0.003, 0.038, 0.048, k["steel"], bevel=0.0006)
    bp.box("north_datum_lug", (0.0, 0.030, 0.027), (0.014, 0.014, 0.014), k["amber"], bevel=0.0006)
    bp.box("vernier_skirt_rear", (-0.0130, -0.016, 0.027), (0.008, 0.006, 0.014), k["brass"], bevel=0.0006)
    bp.box("vernier_skirt_fore", (0.0170, -0.016, 0.027), (0.016, 0.006, 0.014), k["brass"], bevel=0.0006)
    add_handle_rot("reticle_handle", 0.0, 0.0, HANDLE_Z, k["amber"], k, yaw=0.0)
    bp.export_usdc("reticle_carriage")


def build_vernier_keeper() -> None:
    bp.reset()
    k = bp.M()
    bp.rrect_prism("keeper_carriage", 0.0, 0.0, 0.026, 0.024, 0.003, 0.018, 0.030, k["green"], bevel=0.0009)
    bp.box("vernier_tongue", (0.0, 0.0185, 0.027), (0.008, 0.019, 0.014), k["brass"], bevel=0.0007)
    bp.box("gate_latch_arm_x", (-0.026, 0.016, 0.0415), (0.060, 0.008, 0.007), k["steel"], bevel=0.0006)
    bp.box("gate_latch_finger_y", (-0.056, 0.0415, 0.0415), (0.008, 0.051, 0.007), k["brass"], bevel=0.0007)
    add_handle_rot("keeper_handle", 0.0, 0.0, HANDLE_Z, k["green"], k, yaw=math.pi / 2.0)
    bp.export_usdc("vernier_keeper")


def build_bench_mat() -> None:
    bp.reset()
    k = bp.M()
    bp.rrect_prism("esd_mat_base", 0.0, 0.0, 0.480, 0.460, 0.014, 0.000, 0.0020, k["esd"], bevel=0.0006, tile=0.080)
    bp.rrect_prism("esd_mat_trim", 0.0, 0.0, 0.468, 0.448, 0.012, 0.0019, 0.0022, k["slate"], bevel=0.0002)
    bp.rrect_prism("esd_mat_top", 0.0, 0.0, 0.462, 0.442, 0.010, 0.0020, 0.0024, k["esd"], bevel=0.0003, tile=0.080)
    bp.cyl("esd_ground_snap", (-0.218, 0.208, 0.0028), 0.0055, 0.0014, k["brass"], verts=28, bevel=0.0003)
    bp.export_usdc("bench_mat")


if __name__ == "__main__":
    bp.init(TEX, OUT)
    build_stage()
    build_beam_gate()
    build_reticle_carriage()
    build_vernier_keeper()
    build_bench_mat()
