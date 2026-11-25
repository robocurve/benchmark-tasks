"""Survey the authored fixture in its native frame. Run under Python with OpenUSD."""
import argparse
import json
from pathlib import Path

from pxr import Usd, UsdGeom

TASK = Path(__file__).resolve().parents[1]
GEOMETRY = (
    "carriage/backboard", "carriage/liftNeck", "carriage/gripStem",
    "brake/lockingPin", "brake/pressTab", "tap/knob", "tap/lever", "tap/shaft",
    "carriage/spillSill", "patient/tragus", "patient/bed", "mast/benchClamp",
    "mast/verticalPole", "mast/poleFoot", "mast/post", "mast/poleBracket",
    "mast/receiverBack", "mast/threshold35",
)


def mm(values):
    return [round(float(x) * 1000, 4) for x in values]


def read_stage(name):
    stage = Usd.Stage.Open(str(TASK / "assets" / (name + ".usda")))
    if not stage or UsdGeom.GetStageUpAxis(stage) != "Z" or UsdGeom.GetStageMetersPerUnit(stage) != 1:
        raise ValueError(f"Invalid coordinate system for {name}")
    transforms = UsdGeom.XformCache()

    def prim(part):
        item = stage.GetPrimAtPath("/Trainer/" + part)
        if not item:
            raise ValueError(f"Missing fixture part {part}")
        return item

    def origin(part):
        return mm(transforms.GetLocalToWorldTransform(prim(part)).ExtractTranslation())

    def attribute(part, attr):
        val = prim(part).GetAttribute(attr).Get()
        return round(float(val), 8) if isinstance(val, (int, float)) else str(val)

    def shape(part):
        return {"centre_xyz_world_mm": origin(part), "envelope_xyz_mm": mm(prim(part).GetAttribute("xformOp:scale").Get())}

    carriage_z = origin("carriage")[2]
    summary = {
        "ear_world_z_mm": origin("patient/tragus")[2],
        "zero_world_z_mm": carriage_z,
        "sill_centre_world_z_mm": origin("carriage/spillSill")[2],
        "slide_limits_world_z_mm": [round(carriage_z + 1000*attribute("slide", "physics:"+attr+"Limit"), 4) for attr in ("lower","upper")],
    }
    if name != "raised":
        return summary
    teeth = sorted((p.GetName() for p in prim("mast").GetChildren() if p.GetName().startswith("rackTooth")),
                   key=lambda n: int(n.removeprefix("rackTooth")))
    result = {part: shape(part) for part in GEOMETRY}
    for tooth in teeth:
        result["mast/"+tooth] = shape("mast/"+tooth)
    summary["geometry"] = result
    summary["rigid_body_mass_kg"] = {p: attribute(p, "physics:mass") for p in ("carriage","brake","tap")}
    summary["joints_in_authored_units"] = {}
    for joint in ("slide","brakeSlide","tapHinge"):
        keys = ["physics:axis","physics:lowerLimit","physics:upperLimit"]
        keys.extend(attr.GetName() for attr in prim(joint).GetAttributes()
                    if attr.GetName().startswith("drive:") and attr.HasAuthoredValueOpinion())
        summary["joints_in_authored_units"][joint] = {key: attribute(joint,key) for key in keys}
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="compare with packaged survey")
    args = parser.parse_args()
    data = {case: read_stage(case) for case in ("raised","lowered","aligned","shifted")}
    if args.check:
        frozen = json.loads(Path(__file__).with_name("source_survey.json").read_text())
        if data != frozen:
            raise SystemExit("Fixture differs from the packaged OpenUSD survey")
        print("Four USD fixtures agree with the source survey.")
    else:
        print(json.dumps(data, indent=2))


if __name__ == "__main__":
    main()
