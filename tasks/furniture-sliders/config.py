"""One explicit specification; no environment-dependent physics overrides."""
import json
import os
from pathlib import Path

SPEC_PATH = Path(os.environ.get("FURNITURE_SLIDERS_SPEC", Path(__file__).with_name("spec.json"))).resolve()
S = json.loads(SPEC_PATH.read_text())
F = S["furniture"]
P = S["slider"]
T = S["score"]
HZ = S["control"]["hz"]
NUMERICS = S.get('simulation', {'physics_hz':240,'position_iterations':32,'velocity_iterations':0,'contact_offset_m':.0005})
FLOOR_Z = S["floor"]["center_m"][2] + S["floor"]["size_m"][2] / 2
PAD_H = sum(P[k] for k in ("felt_thickness_m", "backing_thickness_m", "foam_thickness_m"))
FOOT_CENTERS = [(x * F["foot_spacing_m"] / 2, y * F["foot_spacing_m"] / 2, 0.)
                for x, y in ((-1, 1), (-1, -1), (1, 1), (1, -1))]


def validation_blockers():
    out = []
    if S["status"] != "measured":
        out.append("Specification has unmeasured properties")
    if not S["measurement_evidence"]:
        out.append("No physical measurement evidence")
    if F["unmeasured"] or P["unmeasured"]:
        out.append("Unresolved geometry, mass or contact measurements")
    if S["materials"]["status"] != "measured":
        out.append("Contact materials uncalibrated")
    if P["model"] != "rigid_calibrated":
        out.append("Rigid foam approximation not validated; deformable foam is not implemented")
    if P["compression_max_m"] is None or P["compression_max_m"] > 0.0005:
        out.append("Compression exceeds rigid approximation tolerance or is unknown")
    if P["force_retention_relative_error"] is None or P["force_retention_relative_error"] > 0.2:
        out.append("Force/retention comparison missing or outside 20 percent")
    if P["indentation_capture_significant"] is not False:
        out.append("Indentation/shear retention not ruled out")
    return out


def validate_spec():
    import math
    def positive(x):
        return isinstance(x, (int, float)) and math.isfinite(x) and x > 0
    for x in (*F["size_m"], F["mass_kg"], F["foot_width_m"], F["foot_spacing_m"],
              F["top_thickness_m"], P["diameter_m"], P["mass_kg"], PAD_H, P["usable_foam_radius_m"]):
        if not positive(x):
            raise ValueError("Nonpositive or nonfinite physical property")
    if P["usable_foam_radius_m"] > P["diameter_m"] / 2:
        raise ValueError("Foam extends beyond the slider")
    if F["foot_spacing_m"] + F["foot_width_m"] > min(F["size_m"][:2]) + 1e-9:
        raise ValueError("Feet extend beyond the selected tabletop")
    if not 0 < F["top_mass_fraction"] < 1 or F["top_thickness_m"] >= F["size_m"][2]:
        raise ValueError("Invalid furniture mass distribution or slab")
    if len(S["layout"]["pads_xy_m"]) != 4 or HZ != 20:
        raise ValueError("Expected four loose sliders and 20 Hz control")
    if NUMERICS['physics_hz']%HZ or NUMERICS['physics_hz']<240:
        raise ValueError('Physics rate must divide into 20 Hz control and be at least 240 Hz')
    for name in ("foot", "foam", "felt", "plastic", "floor"):
        m = S["materials"][name]
        if not 0 <= m["dynamic"] <= m["static"]:
            raise ValueError("Invalid material coefficients")
    if P["model"] not in ("rigid_preview_only", "rigid_calibrated"):
        raise ValueError("This implementation cannot silently replace compliant foam")


validate_spec()
