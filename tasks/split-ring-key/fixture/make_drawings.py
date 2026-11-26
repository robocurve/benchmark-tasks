"""Draw the fabrication plates from the nominal CAD, in millimetres."""
import math
from pathlib import Path

import cadquery as cq
import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.collections import PolyCollection
from matplotlib.patches import Circle, Polygon, Rectangle
import numpy as np

from make_fixture import BENCH_HOLES, END_ANGLE, GUIDE_ANGLE, RADIUS, TRAVEL, add_fixture

OUT = Path(__file__).with_name("fixture-drawings.pdf")
INK, TEAL, RED, GREEN = "#172b34", "#186277", "#c73b34", "#25865b"
mpl.rcParams.update({"font.family":"DejaVu Sans", "font.size":8.4, "text.color":INK,
                     "axes.labelcolor":INK, "axes.titlecolor":TEAL, "pdf.fonttype":42})


def sheet(title, number):
    f=plt.figure(figsize=(11.69,8.27))
    f.text(.055,.942,title,weight='bold',fontsize=20)
    f.text(.945,.942,f"SRK  /  {number}",ha='right',color=TEAL,weight='bold')
    f.add_artist(plt.Line2D([.055,.945],[.92,.92],transform=f.transFigure,c=TEAL,lw=1.1))
    return f


def panel(f,rect,title):
    a=f.add_axes(rect)
    a.set_aspect("equal"); a.axis("off"); a.set_title(title,loc="left",weight="bold",fontsize=10,pad=7)
    return a


def mesh_view(ax,asm,horiz,vert):
    right,up=np.asarray(horiz,dtype=float),np.asarray(vert,dtype=float)
    right/=np.linalg.norm(right); up-=right*np.dot(up,right); up/=np.linalg.norm(up)
    normal=np.cross(right,up)
    tri=[]; depths=[]; colors=[]
    for item in asm.children:
        vertices,faces=item.obj.moved(item.loc).tessellate(.35,.29)
        v=np.array([p.toTuple() for p in vertices]); ix=np.array(faces)
        tris=v[ix]; norm=np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0])
        norm/=np.maximum(1.e-14,np.linalg.norm(norm,axis=1))[:,None]
        front=(norm@normal)>1.e-5
        tris=tris[front]; norm=norm[front]
        light=.61+.39*np.abs(norm@np.array([-.38,-.55,.74]))
        color=np.array(item.color.toTuple()[:3])
        colors.extend(light[:,None]*color); depths.extend(tris.mean(axis=1)@normal)
        tri.extend(np.stack([tris@right,tris@up],axis=2))
    ix=np.argsort(depths)
    ax.add_collection(PolyCollection(np.asarray(tri)[ix],facecolors=np.asarray(colors)[ix],
                                     edgecolors="face",linewidths=.13,rasterized=True))
    ax.autoscale_view()


def dimension(ax,start,end,text,offset=(0,0)):
    ax.annotate("",end,start,arrowprops=dict(arrowstyle="<->",color=TEAL,lw=.8,shrinkA=0,shrinkB=0))
    x=(start[0]+end[0])/2+offset[0]; y=(start[1]+end[1])/2+offset[1]
    ax.text(x,y,text,ha="center",va="center",color=TEAL,fontsize=8,
            bbox=dict(facecolor="white",edgecolor="none",pad=.7))


def foot(f,text):
    f.text(.055,.034,text,fontsize=7.4,color="#46616b")


def assembly_page(pdf,asm):
    f=sheet("Rotary vise and spring-return splitter","01")
    a=panel(f,[.045,.38,.46,.50],"Assembled view  ·  parked")
    mesh_view(a,asm,[.58,-.82,0],[.37,.26,.89]); a.margins(.03)
    b=panel(f,[.56,.43,.38,.44],"Side elevation  ·  looking along Y")
    mesh_view(b,asm,[1,0,0],[0,0,1]); b.margins(.09)
    b.plot([-62,288],[240,240],':',lw=.7,color=TEAL)
    dimension(b,(-63,0),(-63,240),"240",(0,0))
    dimension(b,(0,199),(100,199),"100")
    b.text(2,182,"ring plane to grip centres",fontsize=7)
    b.plot((0,0),(196,238),':',lw=.5,color=TEAL)
    c=panel(f,[.055,.095,.32,.26],"Plan of base plate  ·  drilling datum")
    outline=[(-20,-100),(280,-100),(280,100),(270,100),(270,305),(200,305),(200,100),(-20,100)]
    c.add_patch(Polygon(outline,fc="#d4e2e7",ec=INK,lw=.9))
    for x,y in BENCH_HOLES: c.add_patch(Circle((x,y),3.3,fc="white",ec=INK,lw=.65))
    c.scatter([0],[0],marker="+",c=RED); c.text(8,1,'O',color=RED)
    c.annotate("+X  rear",(140,-20),(27,-20),arrowprops=dict(arrowstyle="->",lw=.7),va="center")
    c.annotate("+Y",(30,80),(30,36),arrowprops=dict(arrowstyle="->",lw=.7),ha="center")
    c.set_xlim(-39,315);c.set_ylim(-118,320)
    f.text(.405,.343,"Bench and spindle datums",color=TEAL,fontsize=10,weight="bold")
    f.text(.405,.315,"O is the ring axis projected on the tabletop. All coordinates are mm.\n"
           "Main plate 300 × 200 × 16. Side extension reaches Y = 305.\n"
           "Six Ø6.6 through-holes at (X, Y)\n"
           "(−5, −85), (265, −85), (−5, 85), (215, 85), (215, 290), (255, 290).\n"
           "Spindle Ø12 × 262, from X = 15 to 277. Two Ø28 × 8 bearing seats\n"
           "start at X = 237 and 255. Wheel mid-plane X = 214, grip radius 85.\n"
           "Key hole starts at (−25, −140, 220).",va="top",linespacing=1.5)
    foot(f,"Dimensions govern. The STEP assembly contains the holes, seats and contact surfaces. Tap pilot holes to the sizes in BUILD.md.")
    pdf.savefig(f,dpi=180);plt.close(f)


def motion(ax,phase):
    # Section at the blade, with the key edge projected from its own section.
    # W points towards the operator. The projected wire shapes are illustrative.
    shifts=[4.4,-.5,-.5,4.4]; front=[.99,1.505,4.297,2.99]; rear=[-.99,-1.09,-.99,-.99]
    shoe_z=-2.08
    ax.add_patch(Rectangle((-2+shifts[phase],shoe_z-1),5,1,fc="#485e69",ec=INK,lw=.7))
    if phase!=3:
        z=[0,.207,3.0][phase]
        p=np.array([[-2,-.03],[3,-.70],[4.5,-.70],[4.5,.70],[3,.70],[-2,.03]])+[shifts[phase],z]
        ax.add_patch(Polygon(p,fc="#78a1b2",ec=INK,lw=.8))
    if phase in (2,3):
        # Key is in the neighbouring tangent section after it catches the red end.
        ax.add_patch(Rectangle((-3.9,0),3.6 if phase==2 else 4.05,2,fc="#dab461",ec="#7b5b20",lw=.7))
        if phase==3:
            ax.annotate("catch out",(3.5,2.9),(1.5,3.9),arrowprops=dict(arrowstyle="->",lw=.7),fontsize=7)
    ax.add_patch(Circle((0,rear[phase]),.99,fc="#acb9b9",ec=INK,lw=.8))
    ax.add_patch(Circle((0,front[phase]),.99,fc="#e3a29d",ec=RED,lw=1.0))
    if phase==0: ax.plot([1.45,3.5],[-.03,-.03],c=TEAL,lw=.65,ls=":")
    ax.set_xlim(-4.5,7);ax.set_ylim(-3.65,6.6)
    ax.plot([-3.9,-1.9],[-3.4,-3.4],color=INK,lw=1.7)
    ax.text(-1.65,-3.42,"2 mm",fontsize=6.8,va="center")


def splitter_page(pdf):
    f=sheet("How the catch opens touching turns","02")
    f.text(.055,.87,"Sections at the catch  ·  red = near turn, grey = far turn, blue = steel catch",fontsize=10)
    for i,title in enumerate(("1  Touching at rest","2  Enter from the outside","3  Lift and start the key","4  Withdraw and wind")):
        ax=panel(f,[.053+i*.228,.52,.211,.29],title);motion(ax,i)
    f.text(.055,.485,"Equal scale in all four views. The gold outline projects the key edge into the catch section. The shoe supports the far turn.",fontsize=8.3)
    ax=panel(f,[.055,.16,.42,.26],"Catch profile  ·  section through the 1.20-wide tip")
    p=[(-2,-.03),(3,-.70),(7,-.70),(7,.70),(3,.70),(-2,.03)]
    ax.add_patch(Polygon(p,fc="#a6c2cb",ec=INK,lw=1))
    dimension(ax,(-2,-1.55),(3,-1.55),"5.00 taper")
    ax.plot([-2,-2],[-1.7,.23],color=TEAL,lw=.5);ax.plot([3,3],[-1.7,-.72],color=TEAL,lw=.5)
    ax.annotate("0.06 nose",(-2,.03),(-2.4,1.7),arrowprops=dict(arrowstyle="->",lw=.7),fontsize=8)
    ax.annotate("1.40 heel",(3,.68),(3.7,1.7),arrowprops=dict(arrowstyle="->",lw=.7),fontsize=8)
    ax.axvline(.5,ls=":",lw=.65,color=TEAL)
    ax.text(.6,-2.12,"wire centre at inward stop",fontsize=7,color=TEAL)
    ax.set_xlim(-3.2,7.7); ax.set_ylim(-2.7,2.4)
    f.text(.535,.404,"Slide settings and fit check",color=TEAL,fontsize=10,weight="bold")
    tip_angle, guide_angle = abs(END_ANGLE), abs(GUIDE_ANGLE)
    arc = RADIUS * math.radians(guide_angle - tip_angle)
    f.text(.535,.376,f"Viewed from the front, the red tip is {tip_angle:g}° above +Y. Align the\n"
           f"guide {guide_angle:g}° above +Y, {arc:.1f} along the near turn behind the tip.\n"
           "The nose reaches 2.50 past the wire centre at the inward stop.\n"
           f"Set the shoe clearance to 0.10, radial travel to {TRAVEL:g} and lift to 0–8.\n"
           "Lift until a 2.00 key edge clears the open tip. Start the key, then\n"
           "withdraw the catch radially and let the handle return before winding.\n"
           "Fit to the actual ring with the checks in BUILD.md.",va="top",linespacing=1.52)
    foot(f,"Dimensioned design sequence. Intermediate wire shapes are illustrative. The simulation recordings and physical lot measurements are separate evidence.")
    pdf.savefig(f,dpi=180);plt.close(f)


if __name__=="__main__":
    assembly=cq.Assembly(); add_fixture(assembly)
    with PdfPages(OUT,metadata={"Title":"Split-ring vise and splitter","Author":""}) as pdf:
        assembly_page(pdf,assembly);splitter_page(pdf)
