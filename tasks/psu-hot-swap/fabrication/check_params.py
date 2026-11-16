"""Check the fabrication files against the simulation.

1. Every value in params.scad tagged ``// task: <expr>`` equals ``<expr>`` evaluated with task.py imported as ``T``
   (in mm, N/mm or g). Every assignment in params.scad must carry a ``task:`` or ``fab:`` tag.
2. The part files (every other .scad file here) contain no dimension of their own: their only numeric literals are
   0, 1, 2, 0.5, 90, 180, 270 and 360 (halves, mirrors, right angles); every size comes from params.scad.
3. Every .scad file has balanced brackets and every part file includes params.scad.
4. A few design checks on the fabrication values (springs never go solid, the hook clears the bay wall when drawn
   in, the blade passes inside the supply's slot, each part fits a BED x BED print bed, ...).

Run from the task folder, with yam_arena importable (task.py imports it), as for test_scoring.py:

    python fabrication/check_params.py
"""

from __future__ import annotations

import ast
import importlib
import math
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
TASK_DIR = HERE.parent
TOL = 1e-6
ALLOWED_LITERALS = {0.0, 1.0, 2.0, 0.5, 90.0, 180.0, 270.0, 360.0}

ASSIGN = re.compile(r"^\s*(\$?[A-Za-z_]\w*)\s*=\s*(.+?);\s*(?://\s*(task|fab):\s*(.*))?$")


def load_task():
    sys.path.insert(0, str(TASK_DIR))
    return importlib.import_module("task")


# -- helpers available to the "task:" expressions -----------------------------------------------------------------

def _helpers(T):
    def mm(v):
        if isinstance(v, (list, tuple)):
            return [x * 1000 for x in v]
        return v * 1000

    def box(spec):
        """An axis-aligned box spec from task.py as [x0, x1, y0, y1, z0, z1] in mm."""
        if len(spec) > 3 and spec[3]:
            raise ValueError("box() of a yawed box; use yawbox()")
        c, s = spec[0], spec[1]
        return [(c[i] + k * s[i] / 2) * 1000 for i in range(3) for k in (-1, 1)]

    def yawbox(spec):
        """A yawed box spec: centre (cx, cy), z range, sides l (along) and t (across), height h, yaw in degrees."""
        c, s = spec[0], spec[1]
        return {"cx": c[0] * 1000, "cy": c[1] * 1000, "z": [(c[2] - s[2] / 2) * 1000, (c[2] + s[2] / 2) * 1000],
                "l": s[0] * 1000, "t": s[1] * 1000, "h": s[2] * 1000,
                "deg": math.degrees(spec[3]) if len(spec) > 3 else 0.0}

    psu = T._psu_boxes(T.UNLIT, plug=True)
    latch = T._latch()
    return {
        "T": T, "math": math, "mm": mm, "box": box, "yawbox": yawbox,
        "psu_box": lambda i: box(psu[i]),
        "latch_box": lambda i: box(latch.boxes[i]),
        "latch_spec": lambda i: latch.boxes[i],
        "cage_box": lambda i: box(T.CAGE_BOXES[i]),
        "cage_spec": lambda i: T.CAGE_BOXES[i],
    }


# -- parsing ------------------------------------------------------------------------------------------------------

def strip_comments_and_strings(src: str) -> str:
    src = re.sub(r"/\*.*?\*/", " ", src, flags=re.S)
    src = re.sub(r"//[^\n]*", " ", src)
    return re.sub(r'"(?:\\.|[^"\\])*"', '""', src)


def parse_params(path: pathlib.Path):
    """[(name, value_text, tag, note, line_no)] for every top-level assignment in params.scad."""
    out, errors = [], []
    for n, line in enumerate(path.read_text().splitlines(), 1):
        s = line.strip()
        if not s or s.startswith("//") or s.startswith("module ") or s.startswith("function "):
            continue
        m = ASSIGN.match(line)
        if not m:
            errors.append(f"params.scad:{n}: not a simple tagged assignment: {s}")
            continue
        name, value, tag, note = m.groups()
        if tag is None:
            errors.append(f"params.scad:{n}: {name} has no '// task:' or '// fab:' tag")
            continue
        out.append((name, value.strip(), tag, (note or "").strip(), n))
    return out, errors


def _flat(v):
    if isinstance(v, dict):
        raise TypeError("expression returned a dict; index it")
    if isinstance(v, (list, tuple)):
        return [x for item in v for x in _flat(item)]
    return [float(v)]


def check_task_values(T, params):
    ns = _helpers(T)
    errors, checked = [], 0
    for name, value, tag, note, n in params:
        if tag != "task":
            continue
        expr = note.split(" | ")[0].strip()
        try:
            scad = _flat(ast.literal_eval(value))
        except (ValueError, SyntaxError):
            errors.append(f"params.scad:{n}: {name}: a task value must be a literal number or list, got {value}")
            continue
        try:
            sim = _flat(eval(expr, dict(ns)))  # noqa: S307 -- trusted expressions from our own params.scad
        except Exception as e:  # noqa: BLE001
            errors.append(f"params.scad:{n}: {name}: cannot evaluate '{expr}': {e}")
            continue
        if len(sim) != len(scad) or any(abs(a - b) > TOL for a, b in zip(sim, scad)):
            errors.append(f"params.scad:{n}: {name} = {value} but {expr} = {[round(x, 6) for x in sim]}")
        checked += 1
    return checked, errors


def eval_all(params):
    """Evaluate every param (task literals and fab expressions) in order, for the design checks."""
    env: dict = {}
    for name, value, tag, note, n in params:
        if name.startswith("$"):
            continue
        env[name] = eval(value, {"__builtins__": {}}, dict(env))  # noqa: S307 -- simple arithmetic on earlier params
    return env


# -- part files ---------------------------------------------------------------------------------------------------

NUMBER = re.compile(r"(?<![\w.$])(\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?(?![\w.])")


def check_part_files(folder: pathlib.Path = HERE):
    errors = []
    files = sorted(p for p in folder.glob("*.scad") if p.name != "params.scad")
    if not files:
        errors.append("no part files found")
    for p in [folder / "params.scad", *files]:
        code = strip_comments_and_strings(p.read_text())
        stack = []
        pairs = {")": "(", "]": "[", "}": "{"}
        for i, ch in enumerate(code):
            if ch in "([{":
                stack.append((ch, i))
            elif ch in pairs:
                if not stack or stack[-1][0] != pairs[ch]:
                    errors.append(f"{p.name}: unbalanced '{ch}' at line {code.count(chr(10), 0, i) + 1}")
                    break
                stack.pop()
        else:
            if stack:
                errors.append(f"{p.name}: unclosed '{stack[-1][0]}' at line {code.count(chr(10), 0, stack[-1][1]) + 1}")
        if p.name == "params.scad":
            continue
        if not re.search(r"^\s*include\s*<params\.scad>", p.read_text(), flags=re.M):
            errors.append(f"{p.name}: does not include <params.scad>")
        for m in NUMBER.finditer(code):
            s, e = m.start(), m.end()
            if code[s - 1:s] == "[" and code[e:e + 1] == "]" and re.match(r"[\w\])]", code[s - 2:s - 1] or " "):
                continue                      # a list index, e.g. PLUG[3]
            if float(m.group(0)) not in ALLOWED_LITERALS:
                line = code.count("\n", 0, m.start()) + 1
                errors.append(f"{p.name}:{line}: literal {m.group(0)} (put dimensions in params.scad)")
    return len(files), errors


# -- design checks on the fabrication values ------------------------------------------------------------------------

def design_checks(P):
    c = []

    def need(ok, msg):
        if not ok:
            c.append(msg)

    hw = P["PSU_W"] / 2
    need(P["FLOOR"] == P["LINK"][4], "FLOOR must equal LINK[4] (the tab link rides on the floor)")
    # tab spring: preload at rest, ~5 N at the stop, never solid, pocket inside its block
    tab_min = P["TAB_SPRING_FREE"] - P["TAB_SPRING_PRELOAD"] - P["LATCH_TRAVEL"]
    need(tab_min > P["TAB_SPRING_SOLID"], f"tab spring goes solid ({tab_min} mm at the stop)")
    need(P["TAB_POCKET_D"] < P["SPRING_BLOCK_T"] - 1, "tab spring pocket breaks through its block")
    stop_force = P["LATCH_RATE"] * (P["TAB_SPRING_PRELOAD"] + P["LATCH_TRAVEL"])
    need(abs(stop_force - P["LATCH_RATE"] * P["LATCH_TRAVEL"]) <= 1.0, "tab force at the stop is off by more than 1 N")
    # the tab stops short of the handle, and the drawn-in hook (ramp crest) clears the bay wall
    need(P["TAB_Y"][0] - P["LATCH_TRAVEL"] > P["HANDLE_Y"][1], "tab at its stop reaches the handle")
    need(P["RAMP_TOP"][1] - P["LATCH_TRAVEL"] < P["WALL_Y"][0], "drawn-in hook does not clear the bay wall")
    # hook window: covered by the hook post (its rest stop) on both sides
    tail = P["RAMP_TOP"][0] + (P["RAMP_TOP"][1] - hw) / math.tan(math.radians(P["RAMP_DEG"]))
    need(tail + P["FIT_SLIDE"] < P["CARR_X1"], "hook window runs past the hook post")
    need(P["POST_FRONT"] > P["FIT_SLIDE"], "hook post does not cover the front of the window")
    need(tail <= P["NOTCH_X"][1] + 0.5, "ramp tail is outside the notch at home")
    inner = hw - P["WALL"]
    need(inner - P["POST_T"] / 2 + P["M3_HEAD_D"] / 2 <= inner, "hook screw heads would hit the side wall")
    need(all(P["CATCH"][0] - P["POST_FRONT"] < x < P["CARR_X1"] for x in P["HOOK_SCREW_X"]), "hook screws off the post")
    need(P["CARR_TOP"] < P["CATCH"][4] - P["FIT_SLIDE"], "carriage top would block the hook window")
    r = P["TAB_SPRING_OD"] / 2 + P["FIT_SLIDE"]
    need(P["FLOOR"] + P["FIT_SLIDE"] < P["TAB_SPRING_POS"][1] - r and P["TAB_SPRING_POS"][1] + r < P["CARR_TOP"],
         "tab spring is not within the carriage's end face")
    # contacts: the ridge spreads them by less than their travel; their bodies stay clear of the ridge crest
    spread = P["RIDGE_APEX"][1] - P["CONTACT_Y"][0]
    need(0 < spread < P["CONTACT_TRAVEL"], "ridge spread must be within the contacts' travel")
    need(P["CONTACT_Y"][1] > P["RIDGE_APEX"][1], "carrier body would touch the ridge crest")
    cmin = P["CONTACT_SPRING_INSTALLED"] - P["CONTACT_TRAVEL"]
    need(cmin > P["CONTACT_SPRING_SOLID"], f"contact spring goes solid ({cmin} mm at full travel)")
    need(P["CONTACT_SPRING_FREE"] >= P["CONTACT_SPRING_INSTALLED"], "contact spring shorter than its seat")
    seat = P["CARRIER_Y1"] + P["CONTACT_SPRING_INSTALLED"]
    need(seat + P["STOP_WALL_T"] + P["M3_HEAD_H"] + P["CONTACT_TRAVEL"] <= hw,
         "contact guide-screw head would stand out of the supply's side at full travel")
    # blade: its passage through the back stop lies inside the supply's open slot
    need(P["RIDGE_APEX"][1] + P["FIT_SLIDE"] < P["SLOT_Y"][1], "blade passage wider than the supply's slot")
    need(P["BLADE_Z"][1] + P["FIT_SLIDE"] < P["SLOT_Z1"], "blade passage taller than the supply's slot")
    need(P["BLADE_SCREW_DY"] - P["INSERT_D"] / 2 > P["RIDGE_APEX"][1] + P["FIT_SLIDE"] + 0.5,
         "blade flange inserts break into the blade passage")
    # shell
    need(P["M3_CSK_H"] < P["LID_T"], "lid countersink deeper than the lid")
    need(P["HOUSING_X0"] < P["BODY_X1"], "contact housing must start ahead of the slot")
    need(P["CARRIER_X0"] - P["FIT_SLIDE"] > P["HOUSING_X0"] + P["INSERT_L"], "joint inserts run into the pockets")
    need(P["BALLAST_HALF_W"] + P["RIB_T"] < hw - P["WALL"], "ballast ribs outside the shell")
    ballast_len = (P["HOUSING_X0"] - P["TRAY_REAR_T"]) - (P["CARR_X1"] + P["FIT_SLIDE"] + P["RIB_T"])
    steel = ballast_len * 2 * P["BALLAST_HALF_W"] * (P["PSU_H"] - P["LID_T"] - P["FLOOR"]) * 7.85e-3   # g
    need(steel > P["PSU_MASS_G"], "ballast bay cannot hold enough steel")
    # print-bed fit
    bed = P["BED"]
    a = math.radians(P["LEADIN_DEG"])
    lead_x0 = P["MOUTH_X"] - P["LEADIN_L"] * math.cos(a) - P["LEADIN_T"] * math.sin(a)
    lead_dy = P["LEADIN_L"] * math.sin(a) + P["LEADIN_T"] * math.cos(a)
    cage_x = P["SERVER"][0] + P["REAR_RAIL_D"] - lead_x0
    cage_half_y = max(P["WALL_Y"][0] + lead_dy, P["WALL_Y"][1]) - P["SPLIT_Y"]
    need(cage_x < bed and cage_half_y < bed, f"cage half {cage_x:.1f} x {cage_half_y:.1f} mm exceeds the bed")
    tray_x = P["HOUSING_X0"] - P["INLET"][0]
    need(tray_x < bed and P["PSU_L"] < bed, "tray or lid longer than the bed")
    handle_x = P["FACEPLATE"][1] - P["HANDLE_BAR"][0]
    need(handle_x < bed, "handle longer than the bed")
    return c, {"cage_x": cage_x, "cage_half_y": cage_half_y, "tray_x": tray_x, "ballast_capacity_g": steel}


def main() -> int:
    T = load_task()
    params, errors = parse_params(HERE / "params.scad")
    checked, errs = check_task_values(T, params)
    errors += errs
    nfiles, errs = check_part_files()
    errors += errs
    try:
        design, info = design_checks(eval_all(params))
        errors += design
    except Exception as e:  # noqa: BLE001
        errors.append(f"design checks failed to run: {e}")
        info = {}
    fab = sum(1 for p in params if p[2] == "fab")
    if errors:
        print("FAIL")
        for e in errors:
            print("  " + e)
        return 1
    print(f"OK: {checked} values match task.py; {fab} fabrication values; {nfiles} part files use only params.scad "
          f"dimensions; design checks pass (cage half {info['cage_x']:.1f} x {info['cage_half_y']:.1f} mm, "
          f"tray {info['tray_x']:.1f} mm long)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
