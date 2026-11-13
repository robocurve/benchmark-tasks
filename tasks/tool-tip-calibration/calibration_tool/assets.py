"""Build a pointed steel tool and an exposed reference contact."""

import math

from pxr import Gf, PhysxSchema, UsdGeom

from .usd_bodies import Body


def tool():
    body = Body('tool', mass=.055, friction=(.6,.5), contact_offset=.0001)
    body.cylinder((0.,0.,.028), .005, .080, (.1,.35,.9))
    body.box((0.,0.,-.012), (.024,.024,.008), (.25,.25,.25))
    mesh = UsdGeom.Mesh.Define(body.stage, '/Part/point')
    count = 32
    points = [Gf.Vec3f(.005*math.cos(2*math.pi*i/count), .005*math.sin(2*math.pi*i/count), .068)
              for i in range(count)] + [Gf.Vec3f(0.,0.,.088)]
    faces = list(range(count))[::-1]
    for i in range(count):
        faces += [i, (i+1)%count, count]
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([count] + [3]*count)
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateSubdivisionSchemeAttr('none')
    body._finish(mesh.GetPrim(), (.75,.75,.78), True, mesh=True)
    PhysxSchema.PhysxContactReportAPI.Apply(body.stage.GetDefaultPrim()).CreateThresholdAttr(0.)
    return body.save()


def reference():
    body = Body('reference', mass=.8, kinematic=True, contact_offset=.0001)
    body.box((0.,0.,.025), (.06,.06,.05), (.15,.15,.15))
    body.cylinder((0.,0.,.0505), .001, .001, (.85,.85,.85))
    return body.save()


BUILDERS = {'tool': tool, 'reference': reference}
