"""Author beveled/cylindrical PBR visual meshes (.usdc) in Blender 5.2 for kelvin-probe-stepped-coupon-contact."""
import math
import pathlib
import sys

_KIT_DIR = "/mnt/data/ops/swarm-top1-gate/pbr_kit"
if _KIT_DIR not in sys.path:
    sys.path.insert(0, _KIT_DIR)

import blender_pbr_lib as bp

PKG = pathlib.Path(sys.argv[sys.argv.index("--") + 1]).resolve()
OUT = PKG / "assets" / "kelvin-probe-stepped-coupon-contact"
TEX = OUT / "textures"
bp.init(TEX, OUT)

MAT_H = 0.004
STAGE_H = 0.012


def make_labels():
    import subprocess
    cmd = (
        f"python3 -B -c \"import sys; sys.path.insert(0, '{_KIT_DIR}'); "
        f"import make_pbr_textures as mp, pathlib; "
        f"mp.make_label_png(pathlib.Path('{TEX}/clip_label.png'), 'TRANSIT CLIP', bg=(210,128,24), fg=(22,24,28), border=(40,40,40), size=(256,64)); "
        f"mp.make_label_png(pathlib.Path('{TEX}/stage_label.png'), '4-PT KELVIN STAGE', bg=(28,34,42), fg=(230,236,242), border=(88,100,115), size=(256,64))\""
    )
    subprocess.run(cmd, shell=True, check=True)


def build_bench_mat():
    bp.reset(); k = bp.M()
    bp.rrect_prism("esd_mat_slab", 0.0, 0.0, 0.480, 0.440, 0.014, 0.0, MAT_H, k["esd"], bevel=0.0012, tile=0.12)
    bp.box("bay_wall_ny", (0.022, 0.175 - 0.032, MAT_H + 0.003), (0.072, 0.006, 0.006), k["slate"], bevel=0.0008)
    bp.box("bay_wall_py", (0.022, 0.175 + 0.032, MAT_H + 0.003), (0.072, 0.006, 0.006), k["slate"], bevel=0.0008)
    bp.box("bay_wall_nx", (-0.011, 0.175, MAT_H + 0.003), (0.006, 0.064, 0.006), k["slate"], bevel=0.0008)
    bp.box("bay_wall_px", (0.055, 0.175, MAT_H + 0.003), (0.006, 0.064, 0.006), k["slate"], bevel=0.0008)
    bp.cyl("ground_snap", (0.205, 0.185, MAT_H + 0.001), 0.006, 0.002, k["steel"], bevel=0.0004)
    bp.export_usdc("bench_mat")


def build_carrier_stage():
    bp.reset(); k = bp.M()
    lbl = bp.mat("stage_label", "stage_label", rough=0.35, metal=0.1)
    bp.rrect_prism("stage_plinth", 0.008, 0.0, 0.204, 0.128, 0.008, 0.0, STAGE_H, k["slate"], bevel=0.0014)
    bp.box("guide_py", (0.0, 0.032, STAGE_H + 0.009), (0.156, 0.010, 0.018), k["white"], bevel=0.0012)
    bp.box("guide_ny", (0.0, -0.032, STAGE_H + 0.009), (0.156, 0.010, 0.018), k["white"], bevel=0.0012)
    bp.box("polarity_key_rail", (0.012, -0.024, STAGE_H + 0.013), (0.110, 0.005, 0.006), k["amber"], bevel=0.0008)
    bp.box("datum_backstop", (0.102, 0.0, STAGE_H + 0.012), (0.012, 0.054, 0.024), k["steel"], bevel=0.0012)
    bp.box("trunnion_py", (-0.062, 0.034, STAGE_H + 0.023), (0.018, 0.012, 0.046), k["slate"], bevel=0.0014)
    bp.box("trunnion_ny", (-0.062, -0.034, STAGE_H + 0.023), (0.018, 0.012, 0.046), k["slate"], bevel=0.0014)
    bp.cyl("pivot_pin", (-0.062, 0.0, STAGE_H + 0.040), 0.0032, 0.082, k["steel"], axis="Y", bevel=0.0004)
    bp.rrect_prism("clip_dock", 0.012, 0.058, 0.036, 0.012, 0.002, STAGE_H, STAGE_H + 0.012, k["slate"], bevel=0.0008)
    bp.decal("stage_badge", (-0.040, -0.054, STAGE_H + 0.0004), (0.048, 0.012), "top", lbl)
    for sx in (-0.078, 0.090):
        for sy in (-0.052, 0.052):
            bp.socket_screw(f"stage_scr_{sx}_{sy}", (sx, sy, STAGE_H + 0.0008), k["steel"], k["black"])
    bp.export_usdc("carrier_stage")


def build_datum_fence():
    bp.reset(); k = bp.M()
    bp.rrect_prism("anvil_body", 0.004, 0.0, 0.034, 0.056, 0.003, 0.0, 0.020, k["white"], bevel=0.0010)
    bp.box("anvil_brass_face", (-0.015, 0.0, 0.010), (0.004, 0.042, 0.016), k["brass"], bevel=0.0005)
    bp.socket_screw("fence_scr_1", (0.004, -0.016, 0.0206), k["steel"], k["black"], r_head=0.0020)
    bp.socket_screw("fence_scr_2", (0.004, 0.016, 0.0206), k["steel"], k["black"], r_head=0.0020)
    bp.export_usdc("datum_fence")


def build_stepped_coupon():
    bp.reset(); k = bp.M()
    bp.box("lower_step_front", (0.0175, 0.0, 0.004), (0.035, 0.046, 0.008), k["green"], bevel=0.0008)
    bp.box("upper_step_rear", (-0.0175, 0.003, 0.008), (0.035, 0.040, 0.016), k["blue"], bevel=0.0010)
    for idx, (px, py, pz) in enumerate((
        (-0.018, -0.012, 0.0165), (-0.018, 0.012, 0.0165),
        (0.018, -0.012, 0.0085), (0.018, 0.012, 0.0085),
    )):
        bp.cyl(f"copper_pad_{idx}", (px, py, pz), 0.0048, 0.0012, k["copper"], bevel=0.0003)
    bp.box("lug_riser", (0.0, -0.020, 0.017), (0.012, 0.010, 0.014), k["steel"], bevel=0.0008)
    bp.box("lug_bridge", (0.0, -0.046, 0.022), (0.012, 0.056, 0.006), k["steel"], bevel=0.0008)
    bp.box("coupon_lug", (0.0, -0.078, 0.016), (0.022, 0.014, 0.022), k["white"], bevel=0.0012)
    bp.box("lug_flange_nx", (-0.021, -0.078, 0.016), (0.004, 0.026, 0.024), k["steel"], bevel=0.0007)
    bp.box("lug_flange_px", (0.021, -0.078, 0.016), (0.004, 0.026, 0.024), k["steel"], bevel=0.0007)
    bp.export_usdc("stepped_coupon")


def build_probe_arm():
    bp.reset(); k = bp.M()
    bp.box("arm_beam", (0.020, 0.0, 0.0), (0.124, 0.022, 0.010), k["alu"], bevel=0.0012)
    bp.cyl("trunnion_hub", (0.0, 0.0, 0.0), 0.011, 0.068, k["steel"], axis="Y", bevel=0.0008)
    bp.cyl("brass_counterweight", (-0.064, 0.0, 0.002), 0.021, 0.060, k["brass"], axis="Y", bevel=0.0012)
    bp.box("handle_arm", (0.042, 0.032, 0.002), (0.018, 0.050, 0.008), k["steel"], bevel=0.0010)
    bp.box("press_lug", (0.042, 0.064, 0.005), (0.028, 0.022, 0.016), k["red"], bevel=0.0012)
    bp.box("press_flange_nx", (0.023, 0.064, 0.005), (0.004, 0.026, 0.020), k["steel"], bevel=0.0007)
    bp.box("press_flange_px", (0.061, 0.064, 0.005), (0.004, 0.026, 0.020), k["steel"], bevel=0.0007)
    bp.export_usdc("probe_arm")


def build_probe_head():
    bp.reset(); k = bp.M()
    bp.rrect_prism("head_block", 0.0, 0.0, 0.056, 0.038, 0.005, -0.004, 0.008, k["slate"], bevel=0.0012)
    bp.box("rear_step_keeper", (-0.042, 0.0, -0.014), (0.006, 0.036, 0.016), k["white"], bevel=0.0008)
    for idx, (px, py) in enumerate(((-0.018, -0.012), (-0.018, 0.012))):
        bp.cyl(f"upper_pogo_{idx}", (px, py, -0.008), 0.0028, 0.014, k["brass"], bevel=0.0004)
    for idx, (px, py) in enumerate(((0.018, -0.012), (0.018, 0.012))):
        bp.cyl(f"lower_pogo_{idx}", (px, py, -0.012), 0.0028, 0.022, k["brass"], bevel=0.0004)
    bp.socket_screw("head_scr_1", (-0.016, 0.0, 0.0086), k["steel"], k["black"], r_head=0.0020)
    bp.socket_screw("head_scr_2", (0.016, 0.0, 0.0086), k["steel"], k["black"], r_head=0.0020)
    bp.export_usdc("probe_head")


def build_transit_clip():
    bp.reset(); k = bp.M()
    lbl = bp.mat("clip_lbl", "clip_label", rough=0.35, metal=0.1)
    bp.rrect_prism("clip_foot", 0.0, 0.0, 0.044, 0.040, 0.005, 0.0, 0.010, k["amber"], bevel=0.0010)
    bp.box("safety_prop_arm", (-0.032, -0.022, 0.050), (0.016, 0.046, 0.012), k["steel"], bevel=0.0010)
    bp.box("safety_prop_post", (-0.032, -0.008, 0.024), (0.016, 0.016, 0.044), k["steel"], bevel=0.0010)
    bp.box("clip_lug", (0.010, 0.018, 0.022), (0.022, 0.014, 0.024), k["amber"], bevel=0.0012)
    bp.box("clip_flange_nx", (-0.008, 0.018, 0.022), (0.004, 0.026, 0.024), k["steel"], bevel=0.0007)
    bp.box("clip_flange_px", (0.028, 0.018, 0.022), (0.004, 0.026, 0.024), k["steel"], bevel=0.0007)
    bp.decal("clip_tag", (0.0, -0.010, 0.0104), (0.032, 0.010), "top", lbl)
    bp.export_usdc("transit_clip")


make_labels()
build_bench_mat()
build_carrier_stage()
build_datum_fence()
build_stepped_coupon()
build_probe_arm()
build_probe_head()
build_transit_clip()
