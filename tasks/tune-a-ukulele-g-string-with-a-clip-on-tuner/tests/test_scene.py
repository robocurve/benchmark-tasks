"""The generated scene file matches the layout, and the layout holds together. Standard library only."""

import math
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "tools"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import build_assets  # noqa: E402
from rdb_ukulele_tuning import fixed_scene as fixture  # noqa: E402
from rdb_ukulele_tuning import layout as L  # noqa: E402

TEXT = fixture.USD.read_text()


def block(name):
    """The text of the first prim called ``name`` (up to the next prim at the same depth)."""
    pattern = rf'def (\w+) "{re.escape(name)}"'
    match = re.search(pattern, TEXT)
    if match is None:
        raise KeyError(name)
    start = match.start()
    indent = TEXT.rfind("\n", 0, start) + 1
    depth = start - indent
    following = re.search(rf'\n {{{depth}}}def \w+ "', TEXT[start + 5:])
    return TEXT[start:start + 5 + following.start()] if following else TEXT[start:]


def value(text, attribute):
    return float(re.search(rf"{re.escape(attribute)} = \(?([-\d.e]+)", text).group(1))


def rotate(q, v):
    """Rotate vector ``v`` by the quaternion ``q`` = (w, x, y, z)."""
    w, x, y, z = q
    u = (x, y, z)
    t = build_assets.scale(build_assets.cross(u, v), 2.)
    return build_assets.add(build_assets.add(v, build_assets.scale(t, w)), build_assets.cross(u, t))


class SceneFile(unittest.TestCase):
    def test_scene_file_is_what_the_builder_writes(self):
        # The text must match exactly; numbers may differ by one unit in their last written digit (6 significant
        # digits): the builder takes the table's height from yam_arena when it is installed (0.7474999999999999) and
        # 0.7475 otherwise, which flips the rounding of two foam-block positions by 1 micrometre.
        built = build_assets.usd()
        number = re.compile(r"-?\d+(?:\.\d*)?(?:e[-+]?\d+)?")
        self.assertEqual(number.sub("#", TEXT), number.sub("#", built))
        for a, b in zip(number.findall(TEXT), number.findall(built)):
            a, b = float(a), float(b)
            self.assertLessEqual(abs(a - b), 1.01e-5 * max(abs(a), abs(b)), (a, b))

    def test_one_fixed_base_articulation(self):
        self.assertEqual(TEXT.count("PhysicsArticulationRootAPI"), 1)
        self.assertIn('def Xform "Ukulele" (\n        prepend apiSchemas = ["PhysicsArticulationRootAPI"', TEXT)
        root = block("root_joint")
        self.assertIn("PhysicsFixedJoint", root)
        self.assertNotIn("physics:body0", root)                          # welded to the world
        self.assertIn(f"rel physics:body1 = <{build_assets.ART}instrument>", root)
        self.assertIn(f"quatf physics:localRot0 = {build_assets._q(fixture.ROTATION_WXYZ)}", root)   # turned on its side
        self.assertIn("enabledSelfCollisions = 0", TEXT)
        for name in fixture.LINKS[:-1]:                    # every body is authored turned as the instrument is
            self.assertIn(f"quatf xformOp:orient = {build_assets._q(fixture.ROTATION_WXYZ)}", block(name))

    def test_bodies_joints_and_masses(self):
        art = TEXT[TEXT.index('def Xform "Ukulele"'):TEXT.index('def Scope "Static"')]
        bodies = re.findall(r'def Xform "(\w+)" \(\n\s+prepend apiSchemas = \["PhysicsRigidBodyAPI"', art)
        self.assertEqual(tuple(bodies), fixture.LINKS)
        masses = [float(m) for m in re.findall(r"float physics:mass = ([\d.e-]+)", art)]
        self.assertEqual(len(masses), len(fixture.LINK_MASSES))
        for got, want in zip(masses, fixture.LINK_MASSES):
            self.assertAlmostEqual(got, want)
        joints = re.findall(r'def Physics(?:Revolute|Prismatic|Fixed)Joint "(\w+)"', TEXT)
        self.assertEqual(joints, ["root_joint", *fixture.JOINTS])
        self.assertEqual(len(re.findall(r'"PhysicsRigidBodyAPI"', TEXT)), len(fixture.LINKS))     # nothing else moves

    def test_buttons_turn_freely_against_drag(self):
        for name in L.STRINGS:
            joint = block(f"peg_{name}")
            self.assertIn('physics:axis = "X"', joint)
            self.assertNotIn("Limit", joint)                              # no end stops
            self.assertAlmostEqual(value(joint, "drive:angular:physics:maxForce"), L.BUTTON_DRAG)
            self.assertAlmostEqual(value(joint, "drive:angular:physics:damping"), L.BUTTON_DAMPING)
            self.assertEqual(value(joint, "drive:angular:physics:stiffness"), 0.)
            anchor = re.search(r"localPos0 = \(([^)]*)\)", joint).group(1)
            for got, want in zip(anchor.split(","), L.button_centre(name)):
                self.assertAlmostEqual(float(got), want, places=6)
            body = block(f"button_{name}")
            x, y, z = fixture.to_station(L.button_centre(name))
            got = re.search(r"double3 xformOp:translate = \(([^)]*)\)", body).group(1).split(",")
            for g, want in zip(got, (x, y, z + fixture.TOP)):            # authored in world coordinates
                self.assertAlmostEqual(float(g), want, places=5)

    def test_string_stretch_spring_returns_in_about_a_tenth_of_a_second(self):
        joint = block("pluck_slide")
        self.assertIn('physics:axis = "X"', joint)
        self.assertAlmostEqual(value(joint, "physics:lowerLimit"), -L.PLUCK_TRAVEL)
        self.assertAlmostEqual(value(joint, "physics:upperLimit"), L.PLUCK_TRAVEL)
        k = value(joint, "drive:linear:physics:stiffness")
        c = value(joint, "drive:linear:physics:damping")
        m = value(joint, "physxJoint:armature") + L.PLUCK_MASS
        self.assertAlmostEqual(k, L.STRING_TENSION * (1 / .225 + 1 / .121), delta=1.)   # tension x (1/a + 1/b)
        x, v, t, back_at, overshoot = .003, 0., 0., None, 0.                 # released 3 mm out
        dt = 1e-5
        while t < .3:
            v += (-k * x - c * v) / m * dt
            x += v * dt
            t += dt
            if back_at is None and abs(x) <= L.PLUCK_BACK:
                back_at = t
            if back_at is not None:
                overshoot = max(overshoot, abs(x))
        self.assertTrue(.04 < back_at < .1, back_at)                        # back within 0.5 mm in about 0.1 s
        self.assertLessEqual(overshoot, L.PLUCK_BACK)                       # and stays there
        self.assertLess(back_at, L.PLUCK_RETURN_S)
        self.assertGreater(k * L.PLUCK_OUT, .5)                              # 2 mm takes about 1 N: not a brush

    def test_needle_is_driven_about_the_screen_normal(self):
        joint = block("needle_pivot")
        self.assertIn('physics:axis = "Z"', joint)
        self.assertEqual(value(joint, "drive:angular:physics:targetPosition"), L.NEEDLE_SIGN * L.NEEDLE_REST_DEG)
        self.assertEqual((value(joint, "physics:lowerLimit"), value(joint, "physics:upperLimit")), (-60., 60.))
        across, up, out = L.screen_frame()
        q = build_assets.SCREEN_Q
        for local, want in (((1., 0., 0.), across), ((0., 1., 0.), up), ((0., 0., 1.), out)):
            for got, w in zip(rotate(q, local), want):
                self.assertAlmostEqual(got, w, places=6)
        # a positive joint turn about the screen's normal swings the needle (pointing up the screen) to the viewer's left
        a = math.radians(10.)
        turned = build_assets.add(build_assets.scale(up, math.cos(a)), build_assets.scale(build_assets.cross(out, up), math.sin(a)))
        self.assertLess(sum(t * c for t, c in zip(turned, across)), 0.)
        self.assertEqual(L.NEEDLE_SIGN, -1.)

    def test_lamps_are_drawn_hidden_in_their_own_scope(self):
        lamps = TEXT[TEXT.index('def Scope "Lamps"'):TEXT.index('def Scope "Materials"')]
        names = re.findall(r'def Cube "(\w+)"', lamps)
        self.assertEqual(names, [*L.LAMPS, L.IN_TUNE_LAMP[0]])
        self.assertEqual(lamps.count('visibility = "invisible"'), len(names))
        self.assertNotIn("PhysicsCollisionAPI", lamps)
        self.assertIn("inputs:emissiveColor", block("lamp_note"))

    def test_rigid_colliders_only(self):
        self.assertNotIn("physxDeformable", TEXT)
        self.assertEqual(set(re.findall(r'physics:approximation = "(\w+)"', TEXT)), {"convexHull"})
        for k in range(len(L.BODY_OUTLINE) - 1):
            collider = block(f"body_collider_{k:02d}")
            self.assertIn('visibility = "invisible"', collider)
            self.assertIn("PhysicsCollisionAPI", collider)
        for name in ("body_shell", "soundhole", "fret_05", "strap_top", "pointer", "screen"):
            self.assertNotIn("PhysicsCollisionAPI", block(name), name)
        for name in ("button", "stretch", "string_g_0", "string_g_1", "tuner_head", "headstock", "housing_g",
                     "neck_rest_base", "body_rest_base"):
            self.assertIn("PhysicsCollisionAPI", block(name), name)


class Layout(unittest.TestCase):
    def test_published_sizes(self):
        # Kala's specification of the KA-15S, inches to metres
        for value, inches in ((L.SCALE, 13.625), (L.LENGTH, 21.062), (L.BODY_LENGTH, 9.5), (L.UPPER_BOUT, 5.187),
                              (L.LOWER_BOUT, 7.0), (L.WAIST, 4.467), (L.BODY_DEPTH, 2.367), (L.NUT_W, 1.375),
                              (L.NECK_W12, 1.672)):
            self.assertAlmostEqual(value, .0254 * inches, delta=.0006, msg=inches)
        self.assertEqual(L.FRETS, 12)
        self.assertAlmostEqual(L.TAIL_Y - L.BODY_Y0, L.BODY_LENGTH)
        self.assertAlmostEqual(L.TAIL_Y - L.TIP_Y, L.LENGTH)
        self.assertAlmostEqual(L.fret(12), L.SCALE / 2)
        self.assertAlmostEqual(L.neck_width(L.fret(12)), L.NECK_W12)
        self.assertAlmostEqual(L.neck_width(0.), L.NUT_W)
        self.assertAlmostEqual(L.TOP_Z - L.BACK_Z, L.BODY_DEPTH)
        # the outline's widths: upper bout (the widest point before the waist), waist, lower bout
        outline = L.BODY_OUTLINE
        waist_at = min(range(4, 12), key=lambda k: outline[k][1])
        self.assertAlmostEqual(2 * max(w for _, w in outline[:waist_at]), L.UPPER_BOUT)
        self.assertAlmostEqual(2 * outline[waist_at][1], L.WAIST)
        self.assertAlmostEqual(2 * max(w for _, w in outline[waist_at:]), L.LOWER_BOUT)
        self.assertEqual([y for y, _ in outline], sorted(y for y, _ in outline))
        self.assertEqual((outline[0][0], outline[-1][0]), (L.BODY_Y0, L.TAIL_Y))
        # the soundhole sits in the upper bout, clear of the fretboard's end and the bridge
        self.assertLess(L.FB_END, L.SOUNDHOLE_Y - L.SOUNDHOLE_R)
        self.assertLess(L.SOUNDHOLE_Y + L.SOUNDHOLE_R, L.BRIDGE_Y[0])
        self.assertLess(L.SOUNDHOLE_R + .01, L.body_half_width(L.SOUNDHOLE_Y))

    def test_orientation_as_the_setup_says(self):
        # on its side in playing position: soundboard toward the robot, neck along y, headstock on the robot's right
        to = fixture.direction_to_station
        self.assertEqual(to((0., 0., 1.)), (-1., 0., 0.))                 # out of the soundboard: toward the robot
        self.assertEqual(to((1., 0., 0.)), (0., 0., 1.))                  # toward the G string: up
        self.assertLess(fixture.to_station((0., L.TIP_Y, 0.))[1], fixture.to_station((0., L.TAIL_Y, 0.))[1])
        heights = {name: fixture.to_station(L.string_point(name, L.PLUCK_Y))[2] for name in L.STRINGS}
        self.assertEqual(max(heights, key=heights.get), "g")             # the G string is the top string
        self.assertEqual(min(heights, key=heights.get), "a")
        for name in ("g", "c"):                                           # their buttons point straight up ...
            self.assertEqual(to(L.button_axis(name)), (0., 0., 1.))
        for name in ("e", "a"):                                           # ... the E and A buttons straight down
            self.assertEqual(to(L.button_axis(name)), (0., 0., -1.))
        self.assertGreater(L.POST_Y["g"], L.POST_Y["c"])                  # the G button is the one nearer the nut
        self.assertEqual(L.START_CENTS["g"], -100.)
        self.assertEqual(L.OTHERS, ("c", "e", "a"))
        # nothing sticks into the table
        self.assertGreater(fixture.CENTRE_Z - max(w for _, w in L.BODY_OUTLINE), .005)
        self.assertGreater(fixture.to_station(L.button_tip("a"))[2], .03)

    def test_strings_reach_their_posts_as_on_the_real_headstock(self):
        # Kala's front photo: the outer strings (G, A) run straight to the posts nearer the nut; the inner ones (C, E)
        # angle outward, just inside those posts, to the posts farther from it, which are only a little farther out
        for outer, inner in (("g", "c"), ("a", "e")):
            self.assertLess(abs(L.POST_Y[outer]), abs(L.POST_Y[inner]))
            self.assertTrue(.002 <= abs(L.POST_X[inner]) - abs(L.POST_X[outer]) <= .003)
            s = L.SIDE[outer]
            nut = L.string_point(outer, 0.)
            self.assertAlmostEqual(L.POST_X[outer] - s * L.POST_R, nut[0], places=6)    # straight, on the post's inner side
            nut = L.string_point(inner, 0.)
            post = (L.POST_X[inner] - s * L.POST_R, L.POST_Y[inner])
            x = nut[0] + (post[0] - nut[0]) * L.POST_Y[outer] / post[1]                  # where it passes the near post
            self.assertGreater(s * (L.POST_X[outer] - s * L.POST_R - x), L.STRING_R + .002)   # inside it, 2.8 mm clear
            self.assertGreater(abs(x), abs(nut[0]))                                         # angling outward

    def test_buttons_and_gears_as_the_real_tuners(self):
        # Kala's back photo: open-gear tuners with pearloid buttons; each gear is drawn as one chrome box
        for name in L.STRINGS:
            self.assertIn("/Fixture/Materials/pearloid>", block(f"button_{name}"))
            self.assertIn("/Fixture/Materials/chrome>", block(f"housing_{name}"))
        self.assertEqual(build_assets.MATERIALS["pearloid"][3], build_assets.MATERIALS["chrome"][3])   # same friction

    def test_pluckable_stretch_lies_over_the_soundhole(self):
        x, y, z = L.pluck_point()
        self.assertEqual(y, L.SOUNDHOLE_Y)
        self.assertLess(math.hypot(x, L.PLUCK_LEN / 2), L.SOUNDHOLE_R)
        self.assertGreater(L.PLUCK_LEN, 2 * .0154)                   # longer than two finger pads are wide
        self.assertGreater(z - L.TOP_Z, .008)                        # room under the string for a fingertip
        gap = abs(L.string_point("c", y)[0] - x)
        self.assertGreater(gap, L.PLUCK_TRAVEL + 2 * L.STRING_R)     # pushed to its stop it does not reach the C string

    def test_nothing_presses_on_the_strings(self):
        self.assertGreater(L.STRAP_Y[0], L.BRIDGE_Y[1])              # the strap is behind the bridge
        self.assertGreater(L.STRAP_Y[0], L.SCALE)
        self.assertLess(L.STRAP_Y[1], L.TAIL_Y)
        # the neck's V-block stays below the lowest (A) string where it holds the neck
        rest_top = fixture.CENTRE_Z - L.neck_width(L.NECK_REST_Y) / 2 - .0005
        a_bottom = fixture.to_station(L.string_point("a", L.NECK_REST_Y))[2] - L.STRING_R
        self.assertGreater(a_bottom - rest_top, .003)
        # the body's V-block cheeks stay below the strings at the bridge
        cheek_top = fixture.CENTRE_Z - L.body_half_width(L.BODY_REST_Y) + L.REST_CHEEK[1]
        self.assertGreater(fixture.to_station(L.string_point("a", L.SCALE))[2] - cheek_top, .03)

    def test_tuner_clears_the_posts_and_the_buttons(self):
        corners = [L.on_screen(sa * L.TUNER_HEAD[0] / 2, sb * L.TUNER_HEAD[1] / 2, -L.TUNER_HEAD[2] * back)
                   for sa in (-1, 1) for sb in (-1, 1) for back in (0, 1)]
        self.assertGreater(min(c[2] for c in corners), L.TOP_Z + L.POST_H + .004)       # in front of the posts' tops
        self.assertLess(max(c[1] for c in corners), min(L.POST_Y.values()) - L.POST_R - .004)   # and beyond them
        tip = L.button_tip("g")
        self.assertGreater(min(c[2] for c in corners) - (tip[2] + L.BUTTON_WIDE / 2), .02)
        self.assertLess(max(abs(c[0]) for c in corners), abs(L.button_centre("g")[0]) - L.BUTTON_LEN / 2)

    def test_screen_layout(self):
        sw, sh = L.TUNER_SCREEN
        for across, up in L.LAMPS.values():
            self.assertLessEqual(abs(across) + L.LAMP_SIZE[0] / 2, sw / 2)
            self.assertLessEqual(abs(up) + L.LAMP_SIZE[1] / 2, sh / 2)
        px, py = L.NEEDLE_PIVOT
        for degrees in (-L.NEEDLE_MAX_DEG, 0., L.NEEDLE_MAX_DEG):
            a = math.radians(degrees)
            tip = (px + L.NEEDLE_LEN * math.sin(a), py + L.NEEDLE_LEN * math.cos(a))
            self.assertLess(abs(tip[0]), sw / 2)
            self.assertLess(tip[1], L.IN_TUNE_LAMP[1][1] - L.IN_TUNE_LAMP[2][1] / 2)      # under the green lamp
        self.assertLess(L.IN_TUNE_LAMP[1][1] + L.IN_TUNE_LAMP[2][1] / 2, min(u for _, u in L.LAMPS.values()) - L.LAMP_SIZE[1] / 2)
        across = [a for a, _ in L.LAMPS.values()]
        self.assertEqual(across, sorted(across))                    # F#, G, G# from the viewer's left to right

    def test_placement_on_the_station(self):
        # the right hand pinches the G button from above in front of its own base, the left plucks over the soundhole
        grip = fixture.to_station(L.button_tip("g"))
        pluck = fixture.to_station(L.pluck_point())
        self.assertEqual(fixture.BASES["right"][1], -.305)
        self.assertTrue(.25 < grip[0] < .32 and -.305 < grip[1] < -.15 and .13 < grip[2] < .17, grip)
        self.assertTrue(.22 < pluck[0] < .30 and 0. < pluck[1] < .15 and .10 < pluck[2] < .14, pluck)
        self.assertLess(math.dist(pluck[:2], fixture.BASES["left"][:2]), .40)
        # the screen faces the robot and up
        n = fixture.direction_to_station(L.screen_frame()[2])
        self.assertLess(n[0], -.5)
        self.assertGreater(n[2], .5)


if __name__ == "__main__":
    unittest.main()
