"""Geometry in metres about the roller centreline. Positive x points into the ironer."""
from math import pi

WORLD=(.358,0.,.7475)
SIDES=("left","right")
INSTRUCTION="Free the caught sleeve and leave the gloved arm resting on the feed table."
TITLE="Free a caught sleeve from a rotary ironer"
SLUG="free-sleeve-from-ironer"
GRIP_MU=1.1
PITCH=.017
COUNT=7
K_LOAD=500.
TRIP_N=5.
BODIES=[]
SHAPES=[]
PROPS={}
def body(name,parent,xyz,joint=None,kind=None,axis=(0,0,1),bounds=(0,0),mass=1.,k=0.,d=0.,com=(0,0,0),diag=(.001,.001,.001)):
    BODIES.append((name,parent,xyz,joint,kind,axis,bounds,mass,k,d))
    PROPS[name]=(com,diag)
def inertia(name):return PROPS[name]
def shape(name,owner,xyz,size,color,kind="box"):
    SHAPES.append((name,owner,xyz,size,color,kind))
steel=(.53,.60,.64); ink=(.10,.13,.14); green=(.10,.31,.32)
linen=(.91,.84,.65); blue=(.065,.29,.60); yellow=(.96,.64,.08)
red=(.84,.075,.045); white=(.92,.94,.90); orange=(.91,.25,.055)
body("frame",None,(0,0,0),mass=5.)
shape("feed_table","frame",(-.219,-.043,.054),(.396,.422,.032),steel)
shape("exit_shelf","frame",(.060,0,.054),(.077,.18,.032),steel)
shape("padded_roll","frame",(0,0,.045),(.064,.178,.064),linen,"cylinder")
for s in (-1,1):
    shape("pedestal"+str(s+1),"frame",(.046,s*.101,.033),(.099,.016,.064),green)
    shape("journal"+str(s+1),"frame",(0,s*.100,.045),(.030,.026,.030),ink,"cylinder")
    for j in range(3):
        shape(f"stripe{s+1}_{j}","frame",(-.035-.015*j,s*.095,.0703),(.006,.023,.0005),yellow,"guide_visual")
body("shoe","frame",(.060,0,.088),"lift","hinge",(0,1,0),(-9.5*pi/180,.72),1.15,0.,.055,(-.060,0,.008),(.0036,.0015,.0042))
# Three tangential strips approximate the short bearing arc of a curved pressure shoe.
for i,(x,z) in enumerate(((-.011,.0014),(0,.003),(.011,.0014))):
    shape("sole"+str(i),"shoe",(x-.060,0,z),(.011,.163,.0038),steel)
shape("shoe_cover","shoe",(-.060,0,.014),(.037,.175,.016),green)
for s in (-1,1):
    shape("rocker"+str(s+1),"shoe",(-.032,s*.085,.010),(.072,.010,.013),green)
shape("pivot","shoe",(0,0,0),(.016,.224,.016),ink,"cylinder")
shape("lever_stem","shoe",(-.016,.114,.058),(.042,.012,.013),steel)
shape("lever_riser","shoe",(0,.112,.030),(.011,.012,.067),steel)
shape("release_grip","shoe",(-.036,.120,.081),(.016,.042,.078),red)
# An elbow swivel and flexion joint carry an adult-size forearm. It cannot translate on a rail.
body("elbow","frame",(-.113,-.238,.093),"elbow_yaw","hinge",(0,0,1),(-.10,.89),.05,0.,.045)
body("arm","elbow",(0,0,0),"elbow_pitch","hinge",(1,0,0),(-.22,.25),.40,0.,.045,(0,.10,0),(.0029,.00030,.0030))
shape("sleeve_upper","arm",(0,.073,.003),(.069,.160,.051),blue,"ellipsoid")
shape("sleeve_lower","arm",(0,.160,0),(.055,.088,.044),blue,"ellipsoid")
shape("sleeve_fold","arm",(0,.162,-.002),(.050,.063,.039),blue)
shape("wrist_cuff","arm",(0,.202,0),(.046,.035,.042),blue,"ellipsoid")
# Padded proximal sleeve on the fixed elbow mount extends across the near table edge.
shape("upper_arm_sleeve","frame",(-.113,-.314,.097),(.076,.108,.065),blue,"ellipsoid")
shape("elbow_cuff","elbow",(0,-.012,0),(.030,.041,.030),blue,"ellipsoid")
shape("upper_arm_stripe","frame",(-.113,-.312,.128),(.052,.013,.002),white,"guide_visual")
shape("arm_band","arm",(0,.122,.023),(.049,.014,.004),white,"guide_visual")
body("hand","arm",(0,.211,-.002),"wrist_flex","hinge",(1,0,0),(-.30,.30),.12,.26,.023,(0,.055,0),(.00045,.00009,.00049))
shape("palm","hand",(0,.047,-.003),(.076,.099,.036),yellow,"ellipsoid")
for i,(x,l) in enumerate(((-.025,.065),(-.008,.078),(.010,.071),(.027,.054))):
    shape("digit"+str(i),"hand",(x,.086+l/2,-.007),(.016,l,.021),yellow,"ellipsoid")
    shape("seam"+str(i),"hand",(x,.089+l*.55,.003),(.011,.003,.001),ink,"guide_visual")
shape("thumb","hand",(.041,.037,.001),(.033,.062,.026),yellow,"ellipsoid")
body("gauge","arm",(.017,.204,-.013),"extension","slide",(1,0,0),(-.003,.036),.018,K_LOAD,.85,diag=(.000003,.000005,.000006))
shape("sewn_tab","gauge",(.004,0,0),(.009,.035,.004),blue)
shape("red_index","gauge",(0,.004,.031),(.004,.029,.005),red,"guide_visual")
for i in range(4):
    shape("scale"+str(i),"arm",(.017+i*.005,.221,.020),(.001,.006,.002),ink if i!=2 else red,"guide_visual")
# Sewn pleat represented by separately massive moving patches, short flexural seams and contacting faces.
for i in range(COUNT):
    name="fabric"+str(i)
    body(name,"gauge" if i==0 else "fabric"+str(i-1),(.008 if i==0 else PITCH,0,0),
         "seam"+str(i),"hinge",(0,0,1) if i%2 else (0,1,0),(-.68,.68),
         .003,.003,.00085,(PITCH/2,0,0),(.00000045,.00000012,.00000054))
    shape("weave"+str(i),name,(PITCH/2,0,0),(PITCH-.00025,.036,.003),blue)
    shape("stitch"+str(i),name,(.003,0,.00155),(.0014,.035,.0003),white,"guide_visual")

# A separate red annunciator rises after the calibrated wrist switch trips.
# Visual-only parts carry no collision and have no mechanical route to the sleeve.
body("flag","frame",(.076,-.101,.053),"flag_travel","slide",(0,0,1),(0.,.038),.015,250.,3.,diag=(.000002,.000002,.000002))
shape("flag_stem","flag",(0,0,.015),(.005,.006,.034),steel,"guide_visual")
shape("flag_red","flag",(0,0,.031),(.025,.009,.012),red,"guide_visual")
shape("flag_housing","frame",(.076,-.101,.077),(.030,.013,.035),ink,"guide_visual")
def collision_allowed(a,b):
    # Internal gauge/garment mounting and neighbouring sewn edges share solid boundaries.
    if "elbow" in (a,b) or {a,b} in ({"arm","hand"},{"gauge","hand"},{"hand","fabric0"},{"hand","fabric1"}):return False
    if "gauge" in (a,b) and ("arm" in (a,b) or "fabric0" in (a,b)):return False
    if a.startswith("fabric") and b.startswith("fabric"):return abs(int(a[6:])-int(b[6:]))>1
    if "arm" in (a,b) and any(n in ("fabric0","fabric1") for n in (a,b)):return False
    return True

MODES=("idle","release_only","closed_pull","partial_release","reference","slow_reference",
       "abandon_release","held_arm","hand_press","recover_after_tug","open_hover","park_closed","surface_touch")
# The short-stroke intervention has two independently testable outcomes, selected by measured load.
EXPECTED={"idle":0.,"release_only":0.,"closed_pull":0.,"reference":1.,"slow_reference":1.,
          "abandon_release":0.,"held_arm":.5,"hand_press":0.,"recover_after_tug":0.,
          "open_hover":.5,"park_closed":.5,"surface_touch":0.}
