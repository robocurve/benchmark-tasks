"""Generate a full-size custom carton net and check it against the simulator.

Offline documentation tool, never imported by the task/policy. Requires the
pinned runtime's numpy/scipy/matplotlib/shapely. Validation is geometric, not a
claim of physical fabrication, paper stiffness or adhesive strength.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from carton_probe import geometry as G, folding as F

ROOT = Path(__file__).resolve().parent
L = 35*np.sqrt(2)
T = 110+L


def design():
    # Vertex order corresponds exactly to geometry.FACETS, including which
    # coincident closed-state ridge vertex belongs to the moving front sheet.
    facets = {
        'carton_spout': [[140,110],[210,110],[175,T]],
        'carton_inner_minus': [[140,110],[140,T],[175,T]],
        'carton_inner_plus': [[210,110],[175,T],[210,T]],
        'carton_roof_minus': [[140,110],[105,T],[140,T]],
        'carton_roof_plus': [[210,110],[210,T],[245,T]],
        'carton_fin_minus': [[140,T],[105,T],[105,T+12],[140,T+12]],
        'carton_fin_plus': [[245,T],[210,T],[210,T+12],[245,T+12]],
        'carton_inner_fin_minus': [[140,T],[175,T],[175,T+24],[140,T+24]],
        'carton_inner_fin_plus': [[175,T],[210,T],[210,T+24],[175,T+24]],
    }
    panels = {n: np.array(v,float) for n,v in facets.items()}
    for i,n in enumerate(('rear_wall','minus_wall','front_wall','plus_wall')):
        panels[n] = np.array([[70*i,0],[70*(i+1),0],[70*(i+1),110],[70*i,110]],float)
    panels.update({
        'rear_gable': np.array([[0,110],[70,110],[35,145.]]),
        'fixed_roof_minus': np.array([[70,110],[140,110],[105,T],[70,T]]),
        'fixed_roof_plus': np.array([[210,110],[280,110],[280,T],[245,T]]),
        'fixed_fin_minus': np.array([[70,T],[105,T],[105,T+12],[70,T+12]]),
        'fixed_fin_plus': np.array([[245,T],[280,T],[280,T+12],[245,T+12]]),
        'floor': np.array([[0,-70],[70,-70],[70,0],[0,0]],float),
    })
    tabs = {
        'side_seam': np.array([[280,0],[292,4],[292,106],[280,110]],float),
        'floor_left': np.array([[0,-4],[-6,-10],[-6,-60],[0,-66]],float),
        'floor_right': np.array([[70,-66],[76,-60],[76,-10],[70,-4]],float),
        'floor_front': np.array([[4,-70],[10,-76],[60,-76],[66,-70]],float),
    }
    for name,a,b,out in [('rear_roof_left',(0,110),(35,145),(-1,1)),
                          ('rear_roof_right',(35,145),(70,110),(1,1))]:
        a,b=np.array(a,float),np.array(b,float)
        direction=(b-a)/np.linalg.norm(b-a)
        c,d=a+10*direction,b-10*direction
        offset=6*np.array(out)/np.sqrt(2)
        tabs[name]=np.array([c,d,d+offset,c+offset])
    # Cuts remove otherwise spurious constraints absent from the model. The
    # front's two 24 mm halves stay connected at their central crease.
    slits=[[[x,T],[x,T+12]] for x in (105,140,210,245)]
    creases=[]
    for x in (70,140,210,280):creases.append((f'wall {x}',[[x,0],[x,110]]))
    for x in (0,70,140,210):creases.append((f'shoulder {x}',[[x,110],[x+70,110]]))
    for x,y in ((140,175),(210,175),(140,105),(210,245)):
        creases.append((f'diagonal {x}-{y}',[[x,110],[y,T]]))
    for x in (140,210):creases.append((f'front corner {x}',[[x,110],[x,T]]))
    creases.extend([('inner fin',[[140,T],[210,T]]),('inner centre',[[175,T],[175,T+24]]),
                    ('minus outer fin',[[70,T],[140,T]]),('plus outer fin',[[210,T],[280,T]]),
                    ('floor hinge',[[0,0],[70,0]])])
    for name,v in tabs.items():
        if name!='side_seam':creases.append((name+' hinge',v[:2].tolist()))
    # The floor-side tabs' attachment edge is their first-to-last edge.
    creases=[(n, (tabs[n[:-6]][[0,3]].tolist() if n in
                  ('floor_left hinge','floor_right hinge','floor_front hinge') else e)) for n,e in creases]
    return panels,tabs,slits,creases


def validate(panels,tabs,slits):
    from shapely.geometry import Polygon,LineString
    from shapely import union_all
    worst=0.
    for name,vertices in G.FACETS.items():
        flat=np.c_[panels[name],np.zeros(len(vertices))]
        d0=np.linalg.norm(flat[:,None]-flat[None,:],axis=-1)
        d1=np.linalg.norm(vertices[:,None]-vertices[None,:],axis=-1)*1000
        worst=max(worst,float(np.abs(d0-d1).max()))
        assert np.allclose(d0,d1,atol=1e-9),name
    # Read the existing rigid-body surface constants without importing USD or
    # substituting a newly invented roof. Match every stationary net panel too.
    tree=ast.parse((ROOT/'carton_probe/geometry.py').read_text())
    node=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign) and
              any(isinstance(t,ast.Name) and t.id=='surfaces' for t in n.targets))
    body=eval(compile(ast.Expression(node.value),'<surface constants>','eval'),
              {'__builtins__':{}},{'A':G.A,'lo':-G.HEIGHT,'FIN':G.FIN})
    mapping={'front_wall':'front','rear_wall':'back','minus_wall':'right',
             'plus_wall':'left','floor':'bottom','rear_gable':'rear_gable',
             'fixed_roof_minus':'roof_right','fixed_roof_plus':'roof_left',
             'fixed_fin_minus':'rear_fin','fixed_fin_plus':'rear_fin'}
    import itertools
    for name,key in mapping.items():
        flat=np.c_[panels[name]/1000,np.zeros(len(panels[name]))]
        target=np.array(body[key]);matched=False
        for perm in itertools.permutations(range(len(target))):
            try:F.fit(flat,target[list(perm)]);matched=True;break
            except ValueError:pass
        assert matched,('Stationary panel mismatch',name)
    all_polys={n:Polygon(v) for n,v in {**panels,**tabs}.items()}
    names=list(all_polys)
    for i,n in enumerate(names):
        assert all_polys[n].is_valid and all_polys[n].area>0,n
        for m in names[:i]:assert all_polys[n].intersection(all_polys[m]).area<1e-7,(n,m)
    # Irrational endpoints create sub-ulp seams in GEOS. Snap the drawing union
    # only at 10^-8 mm; exact facet/hinge checks remain unsnapped.
    union=union_all(list(all_polys.values()),grid_size=1e-8)
    assert union.geom_type=='Polygon' and not union.interiors,'Net must be one connected sheet without holes'
    for slit in slits:assert union.buffer(1e-8).covers(LineString(slit))
    # A hinge is a shared material edge in the unfolded net. Check all twelve;
    # for body hinges identify the corresponding stationary sheet region.
    body_edges={'spout_base':[[140,110],[210,110]],
                'roof_minus':[[140,110],[105,T]],'roof_plus':[[210,110],[245,T]]}
    for name,b0,b1,p0,p1 in G.HINGES:
        edges=[]
        for body in (b0,b1):
            if body=='carton_body':edges.append(np.array(body_edges[name]));continue
            indices=[np.flatnonzero(np.linalg.norm(G.FACETS[body]-p,axis=1)<1e-10)[0] for p in (p0,p1)]
            edges.append(panels[body][indices])
        assert np.allclose(edges[0],edges[1],atol=1e-9),name
        for slit in slits:
            assert LineString(slit).intersection(LineString(edges[0])).length<1e-8,('Cut crosses crease',name)
    # Reconstruct folded start from the flat facets, independent of their 3D
    # initialization representation. Compare with the frozen engineering file.
    reference=json.loads((ROOT/'fold-reference.json').read_text())
    states=F.poses(120,0);initial_error=0.
    for name,vertices in G.FACETS.items():
        target=Rotation.from_quat(states[name]['quat_xyzw']).apply(vertices-G.CENTRES[name])+states[name]['center']
        flat=np.c_[panels[name]/1000,np.zeros(len(vertices))]
        c,r=F.fit(flat,target)
        folded=(flat-flat.mean(axis=0))@r.T+c
        expected=np.array(reference['facets'][name]['initial_vertices_mm'])/1000
        initial_error=max(initial_error,float(np.linalg.norm(folded-expected,axis=1).max()*1000))
    assert initial_error<1e-7
    area=sum(Polygon(panels[n]).area for n in G.FACETS)
    mass=area/1e6*350
    return union,dict(validation='DIGITAL GEOMETRY ONLY; no fabricated specimen tested',
        moving_facets=9,stationary_panels=10,checked_hinges=12,connected_sheet=True,overlapping_panel_area_mm2=0,
        max_facet_distance_error_mm=worst,max_initial_vertex_error_mm=initial_error,
        fold_sweep=F.check_path(),moving_sheet_area_mm2=area,moving_mass_g=mass,
        stationary_lumped_mass_g=520,total_simulated_mass_g=520+mass,
        net_bounds_mm=list(union.bounds),physical_fabrication_verified=False,
        geometry_sha256=hashlib.sha256((ROOT/'carton_probe/geometry.py').read_bytes()).hexdigest())


def draw(ax,panels,tabs,slits,creases,union,title):
    from matplotlib.patches import Polygon
    for n,v in panels.items():
        color='#fff0cf' if n.startswith('carton_') else '#edf2f4'
        ax.add_patch(Polygon(v,facecolor=color,edgecolor='none'))
    for n,v in tabs.items():
        ax.add_patch(Polygon(v,facecolor='#cde6d1',edgecolor='none',hatch='///'))
        c=v.mean(axis=0);ax.text(*c,'GLUE',fontsize=4,ha='center',va='center',rotation=90 if 'floor_' in n else 0)
    for x in (70,245):
        ax.add_patch(Polygon([[x,T],[x+35,T],[x+35,T+12],[x,T+12]],facecolor='#cde6d1',hatch='///',edgecolor='none'))
    x,y=union.exterior.xy;ax.plot(x,y,color='black',lw=.65)
    for _,e in creases:
        e=np.array(e);ax.plot(e[:,0],e[:,1],color='#176a94',lw=.5,ls=(0,(4,2)))
    # Closed-assembly fold directions. The subsequent spreading step and exact
    # start angles are in PHYSICAL_SETUP; these labels are preparation aids.
    for x,y,label,angle in [(70,55,'V90',90),(140,55,'V90',90),(210,55,'V90',90),
        (35,112,'FLAT',0),(90,112,'V45',0),(175,112,'V45',0),(260,112,'V45',0),
        (155,132,'M120',55),(195,132,'M120',-55),(121,132,'FLAT',-55),(228,132,'FLAT',55),
        (141,138,'M180',90),(209,138,'M180',90),(174,174,'M180',90),
        (155,T+2,'V45',0),(195,T+2,'V45',0),(123,T+2,'M45',0),(228,T+2,'M45',0),
        (35,2,'V90',0)]:
        ax.text(x,y,label,fontsize=4,color='#176a94',ha='center',rotation=angle)
    for e in slits:
        e=np.array(e);ax.plot(e[:,0],e[:,1],color='#b52727',lw=1.)
    labels=[(35,55,'REAR'),(105,55,'SIDE -'),(175,55,'FRONT'),(245,55,'SIDE +'),
            (35,-35,'FLOOR 70 x 70'),(175,126,'POURING TRIANGLE'),(175,T+18,'24 mm INNER FIN'),
            (87,T+6,'GLUE REAR FIN'),(263,T+6,'GLUE REAR FIN')]
    for x,y,s in labels:ax.text(x,y,s,ha='center',va='center',fontsize=5)
    ax.plot([167,175,183,167],[92,105,92,92],color='#81959b',lw=.7)
    ax.plot([20,120],[-87,-87],color='black',lw=.7)
    for x in (20,120):ax.plot([x,x],[-89,-85],color='black',lw=.7)
    ax.text(70,-92,'100 mm scale check; print ACTUAL SIZE',ha='center',fontsize=6)
    ax.text(140,191,title,ha='center',fontsize=9)
    ax.text(140,186,'Black: cut | Red: relief cut | Blue: crease | Green: glue | Pattern faces INSIDE',ha='center',fontsize=6)
    ax.text(140,-99,'Fold directions, glue mates, load and fresh-specimen procedure: PHYSICAL_SETUP.md',ha='center',fontsize=6)
    ax.set_aspect('equal');ax.axis('off')


def assembly_views(out):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig=plt.figure(figsize=(11.69,8.27))
    a=35;lo=-110
    fixed=[np.array(v,float) for v in [
        [[-a,-a,lo],[-a,a,lo],[-a,a,0],[-a,-a,0]],
        [[a,a,lo],[a,-a,lo],[a,-a,0],[a,a,0]],
        [[-a,a,lo],[a,a,lo],[a,a,0],[-a,a,0]],
        [[a,-a,lo],[-a,-a,lo],[-a,-a,0],[a,-a,0]],
        [[a,-a,0],[a,a,0],[a,0,a]],
        [[-a,a,0],[a,a,0],[a,0,a],[0,0,a]],
        [[a,-a,0],[-a,-a,0],[0,0,a],[a,0,a]],
        [[0,0,a],[a,0,a],[a,0,a+12],[0,0,a+12]]]]
    for i,(outer,beta,title) in enumerate([(0,0,'1. Assemble closed; front fins UNGLUED'),
                                        (120,0,'2. Spread outer folds: trial START'),
                                        (120,65,'3. Example formed mouth: outcome only')]):
        ax=fig.add_subplot(1,3,i+1,projection='3d')
        ax.add_collection3d(Poly3DCollection(fixed,facecolors='#c9e0e8',edgecolors='#50636b',linewidths=.4,alpha=.32))
        s=F.poses(outer,beta)
        for n,v in G.FACETS.items():
            world=(Rotation.from_quat(s[n]['quat_xyzw']).apply(v-G.CENTRES[n])+s[n]['center'])*1000
            ax.add_collection3d(Poly3DCollection([world],facecolors='#f5c978' if 'inner' in n or 'spout' in n else '#4288a5',edgecolors='#333333',linewidths=.6,alpha=.85))
        ax.set(xlim=(-65,55),ylim=(-55,55),zlim=(-110,85),xlabel='x mm',ylabel='y mm',zlabel='z mm')
        ax.set_box_aspect((120,110,195));ax.view_init(elev=23,azim=-145);ax.set_title(title,fontsize=9)
        ax.tick_params(labelsize=6)
    fig.suptitle('Assembly reference: calculated mid-surfaces, NOT photographs or physical validation',fontsize=12,y=.93)
    fig.text(.5,.12,'Front wall x = -35 mm; shoulder z = 0. Roof flat length 49.497 mm, rise 35 mm.\n'
             'Use the table and vertex checklist in PHYSICAL_SETUP.md. No setup gauge stays during a trial.\n'
             'Do not pre-open the pouring spout to state 3 when preparing a fresh specimen.',ha='center',fontsize=10)
    fig.subplots_adjust(top=.88,bottom=.19,left=.025,right=.975,wspace=.12)
    fig.savefig(out/'assembly-reference.pdf');fig.savefig(out/'assembly-reference.png',dpi=130);plt.close(fig)


def generate(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    panels,tabs,slits,creases=design();union,report=validate(panels,tabs,slits)
    out.mkdir(parents=True,exist_ok=True)
    # Full page axes: one coordinate unit is exactly one millimetre.
    fig=plt.figure(figsize=(420/25.4,297/25.4));ax=fig.add_axes([0,0,1,1])
    draw(ax,panels,tabs,slits,creases,union,'CUSTOM GABLE CARTON — A3 LANDSCAPE 1:1')
    ax.set_xlim(-65,355);ax.set_ylim(-102,195)
    fig.savefig(out/'carton-blank-A3.pdf');fig.savefig(out/'carton-blank-A3.svg')
    fig.savefig(out/'carton-blank-preview.png',dpi=110);plt.close(fig)
    with PdfPages(out/'carton-blank-A4-tiled.pdf') as pdf:
        for i,x0 in enumerate((-15,180)):
            fig=plt.figure(figsize=(210/25.4,297/25.4));ax=fig.add_axes([0,0,1,1])
            draw(ax,panels,tabs,slits,creases,union,f'A4 TILE {i+1}/2 — 15 mm overlap')
            for yy in (-80,180):ax.plot([185,190],[yy,yy],color='purple',lw=.8)
            ax.set_xlim(x0,x0+210);ax.set_ylim(-102,195);pdf.savefig(fig);plt.close(fig)
    data=dict(units='mm',sheet_view='pattern/interior side facing reader; v points to roof',
        panels={n:v.tolist() for n,v in panels.items()},glue_tabs={n:v.tolist() for n,v in tabs.items()},
        relief_cuts=slits,creases=dict(creases),validation=report)
    (out/'blank-geometry.json').write_text(json.dumps(data,indent=2)+'\n')
    (out/'blank-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    assembly_views(out)
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=ROOT/'fabrication')
    generate(p.parse_args().output)
