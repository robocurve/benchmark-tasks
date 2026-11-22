"""Fixture in metres, kg and SI joint units. Origins are relative to the bench datum."""
from math import cos, radians, sin

WORLD = (0.318, 0., 0.7475)
INSTRUCTION = "Free the trapped work glove and settle the conveyor cover on its rests."
SIDES = ("left", "right")
INCLINE = radians(4)
WITHDRAW = (cos(INCLINE), 0., -sin(INCLINE))
GLOVE = (.99,.58,.055)
CUFF = (.92,.96,.05)
METAL = (.33, .41, .46)
GRIP_MU = 1.35
BLUE = (.42,.53,.56)
DARK = (.14, .17, .20)
# name, parent, initial local position, joint, type, axis, limits, mass, k, damping
BODIES = [
    ("frame", None, (0,0,0), None, None, None, None, 1., 0., 0.),
    ("guide", "frame", (0,0,.075), "heave","slide",(0,0,1),(-.024,.044),.03,0,1.),
    ("panel", "guide",(0,0,0),"roll","hinge",(1,0,0),(-.32,.32),.45,0,.012),
    ("hand", "frame",(0,0,0),"withdraw","slide",WITHDRAW,(0,.080),.19,0,8.0),
]
for side, sign in zip(SIDES, (1,-1)):
    BODIES.append(("key_"+side,"frame",(0,sign*.079,0),"advance_"+side,"slide",(0,0,1),(0,.048),.07,0,80.))
    BODIES.append(("digits_"+side,"hand",(0,0,0),"pad_"+side,"slide",(0,0,1),(-.016,.001),.024,655.,1.8))

# name, body, center, full lengths (boxes), color. Ellipsoids use full diameters.
SHAPES = []
def shape(name,body,center,size,color,kind="box"):
    SHAPES.append((name,body,center,size,color,kind))

shape("bed","frame",(.034,0,.010),(.41,.24,.020),DARK)
for i in (-1,1):
    shape("rail"+str(i).replace("-","m"),"frame",(-.035,i*.102,.025),(.195,.008,.010),METAL)
for side,sign in zip(SIDES,(1,-1)):
    shape("landing_"+side,"frame",(0,sign*.052,.035),(.024,.010,.030),METAL)
shape("crossbeam","panel",(0,0,-.006),(.030,.199,.014),METAL)
shape("panel_rib","panel",(0,0,.004),(.010,.194,.008),BLUE)
shape("sheet","panel",(-.073,0,.010),(.152,.202,.002),BLUE)
for i in range(9):
    shape("stripe"+str(i),"panel",(-.003,-.084+i*.021,.013),(.017,.010,.002),(.99,.65,.025) if i%2==0 else (.08,.085,.085),"guide_visual")
for sign in (-1,1):
    shape("flange"+str(sign).replace("-","n"),"panel",(-.073,sign*.100,.006),(.15,.002,.008),METAL)
    shape("guide_column"+str(sign).replace("-","n"),"frame",(-.116,sign*.111,.073),(.012,.012,.105),METAL,"guide_visual")
    shape("yoke_post"+str(sign).replace("-","n"),"frame",(-.023,sign*.109,.036),(.010,.010,.032),METAL,"guide_visual")
    shape("troughwall"+str(sign).replace("-","n"),"frame",(-.098,sign*.092,.031),(.104,.005,.022),METAL)

for k,y in enumerate((-.041,0,.041)):
    shape("stiffener"+str(k),"panel",(-.042,y,.013),(.090,.005,.004),METAL)
shape("folded_heel","panel",(-.090,0,.005),(.005,.110,.012),METAL)
shape("guide_post","frame",(-.115,.112,.071),(.012,.015,.103),METAL,"guide_visual")
shape("guide_boss","guide",(-.116,.112,.025),(.015,.014,.012),DARK)
shape("wrist","hand",(.162,0,.057),(.067,.062,.038),(.095,.12,.12),"ellipsoid")
shape("gauntlet","hand",(.190,0,.060),(.070,.081,.046),GLOVE,"ellipsoid")
shape("sleeve","hand",(.295,0,.066),(.230,.082,.059),CUFF,"ellipsoid")
for i,x in enumerate((.235,.263)):
    shape("reflector"+str(i),"hand",(x,0,.066),(.020,.080,.055),(.68,.77,.78),"ellipsoid")
# These guide surfaces illustrate ideal prismatic bearings. The joint supplies
# their physical constraint, avoiding a second friction/contact path.
shape("cuff_cradle","hand",(.125,0,.034),(.100,.069,.009),METAL,"guide_visual")
for label,sign in (("l",1),("r",-1)):
    shape("withdraw_track_"+label,"frame",(.154,sign*.026,.025),(.182,.008,.006),METAL,"track_visual")
shape("palm","hand",(.101,0,.053),(.120,.090,.034),GLOVE,"ellipsoid")
shape("thumb","hand",(.078,.048,.051),(.069,.031,.024),GLOVE,"ellipsoid")
for side, sign in zip(SIDES,(1,-1)):
    key="key_"+side
    # Flat lift tongues carry removable high-friction sleeves; the upper ends are open.
    shape("saddle_"+side,key,(0,0,.042),(.025,.022,.007),METAL)
    shape("outrigger_"+side,key,(.0205,sign*.0145,.039),(.063,.049,.007),METAL)
    shape("stem_"+side,key,(.041,sign*.029,.074),(.024,.025,.090),(.12,.36,.40))
    shape("backing_"+side,"digits_"+side,(.004,sign*.022,.061),(.033,.029,.008),GLOVE)
    for n,(y,front) in enumerate(((.010,-.022),(.030,-.018)) if sign==1 else ((-.010,-.020),(-.030,-.004))):
        end=.066
        label=("middle","index","ring","little")[(0 if sign==1 else 2)+n]
        # Separate phalanx shells, knuckle seams and dark work-glove fingertips.
        for k,(u,v) in enumerate(((0,.34),(.30,.68),(.63,1.))):
            x=front+(end-front)*(u+v)/2
            z=.055 if k<2 else .056
            shape(label+str(k),"digits_"+side,(x,y,z),((end-front)*(v-u),.018,.020),
                  (.16,.18,.17) if k==0 else GLOVE,"ellipsoid")
        shape(label+"_knuckle","digits_"+side,(front+.62*(end-front),y,.064),(.015,.012,.004),(.26,.25,.22),"ellipsoid")
# Each fingertip pivots at its distal crease. Contact forces also act through
# the vertically sprung carrier shared by that adjacent pair of fingers.
DIGIT_OWNER={}
for side,sign,parts in (("left",1,(("middle",.010,-.022),("index",.030,-.018))),
                        ("right",-1,(("ring",-.010,-.020),("little",-.030,-.004)))):
    for label,y,front in parts:
        x=front+.31*(.066-front)
        body="distal_"+label
        BODIES.append((body,"digits_"+side,(x,y,.055),"flex_"+label,"hinge",(0,1,0),
                       (-.10,.10),.0015,.65,.004))
        DIGIT_OWNER[body]="digits_"+side
        for i,(n,o,pt,size,color,kind) in enumerate(SHAPES):
            if n==label+"0":
                SHAPES[i]=(n,body,(pt[0]-x,0,pt[2]-.055),size,color,kind)
    # A small follower contacts a tab on the digit carriage. Its 65 mm
    # pointer swings above a graduated strip fixed to the moving wrist slide.
    flag="flag_"+side; pivot=(.121,sign*.075,.049)
    BODIES.append((flag,"hand",pivot,"dial_"+side,"hinge",(1,0,0),(-1.05,1.05),.001,.00045,.00009))
    shape("flag_tab_"+side,"digits_"+side,(.121,sign*.065,.051),(.009,.012,.004),DARK)
    shape("follower_link_"+side,"digits_"+side,(.068,sign*.043,.048),(.110,.005,.004),DARK,"guide_visual")
    shape("follower_cross_"+side,"digits_"+side,(.121,sign*.054,.049),(.006,.024,.004),DARK,"guide_visual")
    shape("flag_elbow_"+side,flag,(0,-sign*.005,-.001),(.005,.012,.004),METAL,"guide_visual")
    shape("flag_feeler_"+side,flag,(0,-sign*.010,-.0025),(.007,.004,.004),METAL)
    shape("flag_arm_"+side,flag,(0,0,.031),(.006,.004,.062),METAL,"guide_visual")
    shape("flag_tip_"+side,flag,(0,0,.065),(.015,.012,.009),(.98,.045,.05),"guide_visual")
    for j in range(10):
        shape("scale_"+side+str(j),"hand",(.135,sign*(.075-.005*j),.119),
              (.009,.004,.002),(.95,.93,.83) if j%2==0 else DARK,"guide_visual")
HUMAN={"hand","digits_left","digits_right"}

def inertia(name):
    """Mass properties at the specified body origin, diagonal about the COM."""
    if name.startswith("flag_"):return (0,0,0),(.0000018,.0000018,.00000006)
    if name.startswith("distal_"):return (-.010,0,0),(.00000008,.00000022,.00000022)
    if name=="panel": return (-.014,0,-.001),(.00130,.00027,.00153)
    if name=="guide": return (0,0,.005),(.000012,.000012,.000012)
    if name=="hand": return (.095,0,.047),(.00008,.00030,.00032)
    if name.startswith("digits_"): return (.016,.017 if name.endswith("left") else -.017,.055),(.000003,.000013,.000015)
    if name.startswith("key_"): return (.026,.018 if name.endswith('left') else -.018,.060),(.000055,.000023,.000065)
    return (0,0,0),(.01,.01,.01)
