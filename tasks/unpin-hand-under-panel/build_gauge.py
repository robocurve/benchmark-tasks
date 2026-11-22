"""Generate mirrored printable pointer blanks and the local-datum cut list in mm."""
import csv
import math
from pathlib import Path
import struct
from fixture import BODIES, SHAPES

def rocker(side):
    # Y,Z profile coordinates. The 10 mm follower is opposite a 65 mm pointer.
    outline=[(-12,-4),(3,-4),(3,65),(-2,65),(-2,3),(-12,2)]
    angles=sorted({math.atan2(z,y)%(2*math.pi) for y,z in outline} |
                  {2*math.pi*j/96 for j in range(96)})
    def cross(a,b):return a[0]*b[1]-a[1]*b[0]
    def radius(direction):
        hits=[]
        for a,b in zip(outline,outline[1:]+outline[:1]):
            edge=(b[0]-a[0],b[1]-a[1])
            den=cross(direction,edge)
            if abs(den)<1e-9:continue
            t=cross(a,edge)/den
            u=cross(a,direction)/den
            if t>0 and -1e-9<=u<=1+1e-9:hits.append(t)
        return min(hits)
    rings=[[] for _ in range(4)]
    for angle in angles:
        d=(math.cos(angle),math.sin(angle))
        length=radius(d)
        for ring,x,r in zip(rings,(-2,2,-2,2),(1.6,1.6,length,length)):
            ring.append((x,side*r*d[0],r*d[1]))
    faces=[]
    for i in range(len(angles)):
        j=(i+1)%len(angles)
        for a,b,d,c in ((1,3,1,3),(2,0,2,0),(3,2,3,2),(0,1,0,1)):
            v=(rings[a][i],rings[b][i],rings[c][j],rings[d][j])
            for tri in ((v[0],v[1],v[2]),(v[0],v[2],v[3])):
                faces.append(tri if side==1 else tri[::-1])
    return faces

def write_stl(path,faces):
    with path.open("wb") as stream:
        stream.write(b"Pointer blank, dimensions in mm".ljust(80,b" ")+struct.pack("<I",len(faces)))
        for a,b,c in faces:
            u=[y-x for x,y in zip(a,b)]
            v=[y-x for x,y in zip(a,c)]
            n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            length=math.sqrt(sum(x*x for x in n))
            stream.write(struct.pack("<12fH",*(x/length for x in n),*a,*b,*c,0))

def build(out):
    out.mkdir(exist_ok=True)
    for name,side in (("left",1),("right",-1)):
        write_stl(out/("pointer-"+name+".stl"),rocker(side))
    origins={}
    for name,parent,pt,*_ in BODIES:
        origins[name]=[a+b for a,b in zip(origins.get(parent,(0,0,0)),pt)]
    with (out/"dimensions-mm.csv").open("w",newline="") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow(["part","body","shape","reset_x","reset_y","reset_z","length_x","width_y","height_z"])
        for name,body,pt,size,col,kind in SHAPES:
            writer.writerow([name,body,kind,*[round(1000*(a+b),3) for a,b in zip(origins[body],pt)],
                             *[round(1000*x,3) for x in size]])

if __name__=="__main__":
    build(Path(__file__).resolve().parent/"fabrication")
