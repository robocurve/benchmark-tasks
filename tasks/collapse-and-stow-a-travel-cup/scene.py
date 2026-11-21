"""Calibrated YAM station and contact bodies for a collapsible 250 ml cup."""

import os
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np
from yam_arena import station

DIRECTORY = Path(__file__).resolve().parent
ROBOT = Path(os.environ.get("ROBOCURVE_ABC_MODELS", DIRECTORY / "assets/abc_sim/models"))
if not ROBOT.is_dir():
    ROBOT = DIRECTORY.parents[1] / "resist/assets/vendor/abc_transfer/abc_sim/models"
# The ABC robot coordinates put the midpoint between the mounts at x = 0.2525 m.
# The accepted Arena station uses that midpoint as its x origin.
MOUNT_ORIGIN = np.array((0.2525, 0., 0.))
TABLE_CENTER = np.asarray(station.TABLE_CENTER) + MOUNT_ORIGIN
TABLE_SIZE = np.asarray(station.TABLE_SIZE)
TABLE_TOP = station.TABLE_TOP
LOCATION = np.array((0.525, 0.0, TABLE_TOP))
STAGE_TRAVEL = 0.020
TAB_OPEN = 1.50
RING_EDGES = 16
CAMERAS = {name: name for name in ("top_cam", "left_cam", "right_cam", "judging_front")}
CUP_MASS = .102


def fmt(components):
    return " ".join(f"{component:.8g}" for component in components)


def body(tree, name, pos, **properties):
    return ET.SubElement(tree, "body", name=name, pos=fmt(pos), **properties)


def geom(tree, name, shape, size, pos=(0., 0., 0.), **properties):
    return ET.SubElement(tree, "geom", name=name, type=shape, size=fmt(size),
                         pos=fmt(pos), **properties)


def ring(tree, assets, name, lower, upper, radii, color, collision=True, mass=None):
    """Regular-polygon hollow ring specified by inner and outer radii at its ends."""
    step = 2 * np.pi / RING_EDGES
    inner_lo, outer_lo, inner_hi, outer_hi = radii
    coords = []
    for z, inside, outside in ((lower, inner_lo, outer_lo), (upper, inner_hi, outer_hi)):
        for angle in (-step/2, step/2):
            for radius in (inside, outside):
                coords.extend((radius*np.cos(angle), radius*np.sin(angle), z))
    ET.SubElement(assets, "mesh", name=name, vertex=fmt(coords))
    for index in range(RING_EDGES):
        props = {"name": f"{name}_{index}", "type": "mesh", "mesh": name,
                 "euler": fmt((0, 0, index*step)), "rgba": color,
                 "solref": "0.008 1", "friction": "0.72 0.008 0.0004"}
        if not collision:
            props.update(contype="0", conaffinity="0")
        if mass is not None:
            props["mass"] = str(mass/RING_EDGES)
        ET.SubElement(tree, "geom", **props)


def view(tree, name, eye, look_at, fovy):
    z = np.asarray(eye) - look_at
    z /= np.linalg.norm(z)
    x = np.cross((0, 0, 1), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    ET.SubElement(tree, "camera", name=name, pos=fmt(eye),
                  xyaxes=fmt(np.r_[x, y]), fovy=str(fovy))


def calibrated_view(tree, name, pos, wxyz, intrinsics):
    fx, fy, cx, cy = intrinsics
    ET.SubElement(tree, "camera", name=name, pos=fmt(pos), quat=fmt(wxyz),
                  sensorsize=".0064 .0048", resolution="640 480",
                  focalpixel=fmt((fx, fy)), principalpixel=fmt((320. - cx, cy - 240.)))


def install_reference_table(world):
    """Use the accepted Arena table, camera pose and stand in the ABC robot frame."""
    for name in ("gate_collision", "play_table", "overhead_camera", "left_side_camera",
                 "right_side_camera"):
        world.remove(world.find(f"body[@name='{name}']"))
    for element in list(world.findall("geom")):
        name = element.get("name", "")
        if not name or name == "table_plane" or "_wall" in name:
            world.remove(element)
    for name in ("floor", "floor_collision"):
        world.find(f"geom[@name='{name}']").set("pos", fmt((0., 0., station.FLOOR_Z)))
    world.find("geom[@name='floor']").set("rgba", ".65 .66 .67 1")
    # Match the reference oak and grey floor after MuJoCo's lighting.
    geom(world, "reference_table", "box", TABLE_SIZE / 2., TABLE_CENTER,
         rgba=".92 .86 .74 1", friction=".7 .005 .0001")
    lamp = np.asarray(station.LAMP_POS) + MOUNT_ORIGIN
    # Use stable, direct lighting. The legacy OpenGL shadow map produces
    # changing stair-step artifacts on the closely spaced bellows surfaces.
    world.find("light").attrib.update(pos=fmt(lamp), dir=fmt(TABLE_CENTER-lamp),
                                      diffuse=".60 .60 .60", specular=".07 .07 .07",
                                      castshadow="false")

    camera = station.top_camera()
    calibrated_view(world, "top_cam", np.asarray(camera.pos) + MOUNT_ORIGIN,
                    np.roll(camera.rot_xyzw, 1), station.TOP_CAM_K)
    # The ABC XML link_6 is already in the convention of the optical calibration.
    # Replace the approximate optical camera while preserving the physical wrist collider.
    for side in ("left", "right"):
        wrist = world.find(f".//body[@name='{side}_link_6']")
        old_frame = wrist.find(f".//body[@name='{side}_camera_frame']")
        old_frame.remove(old_frame.find("camera"))
        calibrated_view(wrist, f"{side}_cam", station.WRIST_CAM_POS[side],
                        station.WRIST_CAM_WXYZ, station.WRIST_CAM_K)
    for name, (shape, size, pos, rotation) in station.camera_stand(camera).items():
        size = np.asarray(size) / 2. if shape == "box" else (size[0], size[1] / 2.)
        color = station.HOUSING if name == "camera_housing" else station.STAND
        geom(world, name, shape, size, np.asarray(pos) + MOUNT_ORIGIN,
             quat=fmt(np.roll(rotation, 1)), rgba=fmt((*color, 1.)))


def build():
    xml = ET.parse(ROBOT / "yam_bimanual_empty.xml").getroot()
    xml.find("compiler").set("meshdir", str(ROBOT / "assets/i2rt_yam/assets"))
    xml.remove(xml.find("keyframe"))
    xml.find("option").attrib.update(timestep="0.002", iterations="60", cone="elliptic")
    xml.find("visual/quality").attrib.update(shadowsize="2048", offsamples="4")
    assets = xml.find("asset")
    assets.find("material[@name='black']").set("rgba", ".09 .095 .10 1")
    assets.find("material[@name='white']").set("rgba", ".94 .945 .95 1")
    world = xml.find("worldbody")
    install_reference_table(world)
    for side in ("left", "right"):
        world.find(f"body[@name='{side}_arm']").set(
            "pos", fmt(np.asarray(station.BASES[side]) + MOUNT_ORIGIN))
        gripper = xml.find(f"actuator/position[@name='{side}_gripper']")
        gripper.attrib.update(forcerange="-40 40", kp="750", kv="18")
        for index in range(1, 7):
            actuator = xml.find(f"actuator/position[@name='{side}_joint{index}']")
            force = 27 if index <= 3 else 7
            kp = 135 if index <= 3 else 58
            kv = 8 if index <= 3 else 3
            actuator.attrib.update(kp=str(kp), kv=str(kv), forcerange=f"-{force} {force}")
    # The candy PR uses this viewing direction, portrait height, and 34 mm
    # focal length with a 20.955 mm horizontal aperture. Retarget the axis at
    # the center of the cup's manipulation zone on the same fixed worktop.
    eye_offset = np.array((.74, -.46, 1.17)) - (.366, .067, .797)
    center = LOCATION + (.018, .018, .075)
    fovy = np.degrees(2. * np.arctan(20.955 * 480 / (2. * 34. * 640)))
    view(world, "judging_front", center + eye_offset, center, fovy)

    body_color = ".19 .46 .47 1"
    rubber_color = ".13 .35 .38 1"
    lid_color = ".83 .85 .79 1"
    sleeve_color = ".95 .71 .29 1"

    base = body(world, "cup_base", LOCATION + (0, 0, 0.0005))
    ET.SubElement(base, "freejoint", name="cup_free")
    geom(base, "base_disk", "cylinder", (0.0255, 0.003), (0, 0, 0.003),
         rgba=rubber_color, mass="0.030", friction="1.1 0.01 0.001")
    ring(base, assets, "first_bellows", 0.005, 0.028, (.0225,.0255,.025,.028),
         body_color, mass=0.017)

    stage = base
    names = ["cup_base"]
    shell_radii = [(.029,.026,.0305,.0275), (.0305,.027,.0325,.029), (.034,.0325,.035,.033)]
    # Mount offsets and signed joint axes give concentric nesting under axial load.
    for index, (outer_low, inner_low, outer_high, inner_high) in enumerate(shell_radii, 1):
        stage = body(stage, f"stage_{index}", (0, 0, 0.020))
        names.append(f"stage_{index}")
        ET.SubElement(stage, "joint", name=f"fold_{index}", type="slide", axis="0 0 -1",
                      range=f"0 {STAGE_TRAVEL}", damping="1.1", armature="0.008",
                      solreflimit="0.004 1", solimplimit="0.97 0.999 0.0002")
        ring(stage, assets, f"bellows_{index}", 0.009, 0.032,
             (inner_low,outer_low,inner_high,outer_high), body_color, mass=0.014)
        ring(stage, assets, f"crease_{index}", 0.008, 0.010,
             (inner_low,outer_low,inner_low,outer_low), rubber_color, collision=False, mass=0.)
    # The collar reaches 48 mm above the table when all three folds are nested.
    ring(stage, assets, "collar", 0.019, 0.039, (.0371,.039,.0371,.039),
         lid_color, mass=0.026)
    geom(stage, "lid", "cylinder", (.0395,.0038), (0,0,.0432),
         rgba=lid_color, mass="0.031", friction="0.9 0.01 0.0004")
    geom(stage, "mouth_recess", "box", (.009,.005,.0003), (-.015,0,.04705),
         rgba=".12 .18 .19 1", mass="0", contype="0", conaffinity="0")

    tab = body(stage, "stopper", (.006,0,.050))
    ET.SubElement(tab, "joint", name="tab_hinge", axis="0 1 0", range=f"0 {TAB_OPEN}",
                  damping="0.006", armature="0.0001")
    geom(tab, "stopper_base", "box", (.0145,.0065,.0024),(-.0145,0,0),
         rgba=rubber_color, mass=".007", friction="1.1 .01 .001")
    geom(tab, "stopper_lip", "box", (.0035,.0065,.0033),(-.028,0,.001),
         rgba=rubber_color, mass=".001", friction="1.1 .01 .001")
    names.append("stopper")

    sleeve = body(world, "heat_sleeve", LOCATION + (0,0,.057))
    ET.SubElement(sleeve, "freejoint", name="sleeve_free")
    ring(sleeve, assets, "sleeve", 0., .022, (.03555,.0368,.03555,.0375),
         sleeve_color, mass=.030)
    # Grip ring uses rubber-to-shell interference in its operating position.
    ring(sleeve, assets, "sleeve_inner_bead", .017, .020, (.03555,.037,.03555,.037),
         sleeve_color, mass=.003)

    contact = ET.SubElement(xml, "contact")
    for index, name in enumerate(names):
        for other in names[index+1:]:
            ET.SubElement(contact, "exclude", body1=name, body2=other)
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    cup_bodies = [model.body(name).id for name in (*names, "heat_sleeve")]
    ratio = CUP_MASS / sum(model.body_mass[cup_bodies])
    model.body_mass[cup_bodies] *= ratio
    model.body_inertia[cup_bodies] *= ratio
    mujoco.mj_setConst(model, mujoco.MjData(model))
    return model


class CupPhysics:
    """Reduced constitutive model for the fold hysteresis and trapped air."""

    def __init__(self, model):
        stages = [model.joint(f"fold_{j}") for j in range(1, 4)]
        self.positions = [joint.qposadr[0] for joint in stages]
        self.axes = [joint.dofadr[0] for joint in stages]
        self.tab_pos = model.joint("tab_hinge").qposadr[0]
        self.tab_axis = model.joint("tab_hinge").dofadr[0]
        self.air_reference = 0.0
        self.model = model
        self.fit_force = 0.0
        self.stage = model.body("stage_3").id
        self.base = model.body("cup_base").id
        self.sleeve = model.body("heat_sleeve").id
        self.sleeve_dofs = model.joint("sleeve_free").dofadr[0]

    def apply(self, data, dt):
        data.qfrc_applied[:] = 0.0
        travel = data.qpos[self.positions]
        progress = np.clip(travel / STAGE_TRAVEL, -.02, 1.02)
        tab = float(data.qpos[self.tab_pos])
        fraction = np.clip(tab / TAB_OPEN, -.02, 1.02)
        air = np.clip((tab-.09) / .45, 0., 1.)
        compression = float(sum(travel))
        self.air_reference += min(1., dt*air*35.)*(compression-self.air_reference)
        enclosed_volume = max(.25, 1. - (compression-self.air_reference)/.10)
        air_reaction = (1.-air)*min(40., max(-8., 255.*(1./enclosed_volume-1.)))
        data.qfrc_applied[self.axes] = -np.array((100.,116.,103.))*progress*(progress-.46)*(progress-1.) - air_reaction
        data.qfrc_applied[self.tab_axis] = -.72*fraction*(fraction-.20)*(fraction-1.)
        self.apply_sleeve_fit(data, compression)

    def apply_sleeve_fit(self, data, compression):
        """Shallow reversible axial compliance of the annular interference fit."""
        collapsed = compression > .054
        seat = self.base if collapsed else self.stage
        cup_z = data.xmat[seat].reshape(3,3)[:,2]
        sleeve_z = data.xmat[self.sleeve].reshape(3,3)[:,2]
        same_direction = float(sleeve_z@cup_z)
        lowest_edge = 0. if same_direction >= 0 else -.022
        seat_height = .0005-lowest_edge if collapsed else -.003
        alignment = abs(same_direction) if collapsed else same_direction
        displacement = data.xpos[self.sleeve]-data.xpos[seat]
        axial = float(displacement @ cup_z)
        radial = np.linalg.norm(displacement-axial*cup_z)
        offset = axial-seat_height
        self.fit_force = 0.0
        if radial > .0025 or alignment < .990 or abs(offset) > .004:
            return
        # A lead-out chamfer relaxes the already small retention load at release.
        blend = min(1., (.004-abs(offset))/.001)
        strength = blend*min(1., (.0025-radial)/.001)
        jacobian = np.zeros((3,self.model.nv))
        mujoco.mj_jacBody(self.model, data, jacobian, None, seat)
        sleeve_axis_vel = data.qvel[self.sleeve_dofs:self.sleeve_dofs+3]@cup_z
        relative_speed = sleeve_axis_vel-(jacobian@data.qvel)@cup_z
        self.fit_force = float(-strength*np.clip(1100.*offset+1.2*relative_speed,-2.5,2.5))
        force = self.fit_force*cup_z
        data.qfrc_applied[self.sleeve_dofs:self.sleeve_dofs+3] += force
        mujoco.mj_applyFT(self.model, data, -force, np.zeros(3), data.xpos[seat], seat,
                          data.qfrc_applied)

    def tab_angle(self, data):
        return float(data.qpos[self.tab_pos])

    def folds(self, data):
        return data.qpos[self.positions].copy()
