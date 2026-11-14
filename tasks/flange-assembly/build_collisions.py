"""Build an explicitly perforated, conservative collision approximation of the CAD flange."""
import math
from pathlib import Path

root = Path(__file__).parent / 'assets'
R, step = .075, .002
holes = [(0., 0., .027), (.05499, 0., .008), (-.05499, 0., .008),
         (0., .05499, .008), (0., -.05499, .008)]
prims = []
for i in range(75):
    y0, y1 = -R + i*step, -R + (i+1)*step
    outer = math.sqrt(max(0., R*R - max(abs(y0), abs(y1))**2))
    intervals = []
    for x, y, radius in holes:
        dy = 0. if y0 <= y <= y1 else min(abs(y-y0), abs(y-y1))
        if dy < radius:
            dx = math.sqrt(radius**2-dy**2)
            intervals.append((x-dx, x+dx))
    intervals.sort()
    cursor = -outer
    def box(left, right):
        if right-left < .0001: return
        idx = len(prims)
        prims.append(f'''        def Cube "plate_{idx}" (prepend apiSchemas = ["PhysicsCollisionAPI"])
        {{
            double size = 1
            token visibility = "invisible"
            double3 xformOp:translate = ({(left+right)/2:.8f}, {(y0+y1)/2:.8f}, 0.008)
            double3 xformOp:scale = ({right-left:.8f}, {step:.8f}, 0.016)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}''')
    for left,right in intervals:
        if left > cursor: box(cursor, min(left, outer))
        cursor = max(cursor, right)
    if cursor < outer: box(cursor, outer)
# A 24-segment tapered neck leaves the 54 mm bore clear.
for i in range(24):
    a,b = 2*math.pi*i/24,2*math.pi*(i+1)/24
    points = []
    for z,rout in ((.016,.04),(.051,.03)):
        for r,theta in ((.027,a),(rout,a),(rout,b),(.027,b)):
            points.append((r*math.cos(theta),r*math.sin(theta),z))
    pts=', '.join(f'({x:.8f}, {y:.8f}, {z:.8f})' for x,y,z in points)
    prims.append(f'''        def Mesh "neck_{i}" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"])
        {{
            point3f[] points = [{pts}]
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0,3,2,1, 4,5,6,7, 0,1,5,4, 1,2,6,5, 2,3,7,6, 3,0,4,7]
            uniform token physics:approximation = "convexHull"
            uniform token subdivisionScheme = "none"
            token visibility = "invisible"
        }}''')
for filename in ('flange_fixed.usda','flange_with_stub.usda'):
    path=root/filename
    text=path.read_text()
    prefix=text[:text.index('    def "Collision"')]
    path.write_text(prefix+'    def "Collision"\n    {\n'+'\n'.join(prims)+'\n    }\n}\n')
print(f'{len(prims)} convex collision pieces; four 16 mm holes')
