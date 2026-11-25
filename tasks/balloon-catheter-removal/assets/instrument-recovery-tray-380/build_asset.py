"""Original full-size stainless receiving tray and passive riser, CC0-1.0."""
from pathlib import Path
import json

root=Path(__file__).resolve().parent
text='''#usda 1.0
(defaultPrim="RecoveryTray"
 metersPerUnit=1
 upAxis="Z")
def Xform "RecoveryTray" {
 def Scope "Looks" {
  def Material "Steel" {
   token outputs:surface.connect=</RecoveryTray/Looks/Steel/Shader.outputs:surface>
   def Shader "Shader" {
    uniform token info:id="UsdPreviewSurface"
    color3f inputs:diffuseColor=(0.62,0.65,0.67)
    float inputs:metallic=0.8
    float inputs:roughness=0.28
    token outputs:surface
   }
  }
  def Material "Polypropylene" {
   token outputs:surface.connect=</RecoveryTray/Looks/Polypropylene/Shader.outputs:surface>
   def Shader "Shader" {
    uniform token info:id="UsdPreviewSurface"
    color3f inputs:diffuseColor=(0.77,0.79,0.78)
    float inputs:roughness=0.55
    token outputs:surface
   }
  }
 }
'''
parts=[('Riser',(0,0,.033),(.100,.140,.066),'Polypropylene'),
       ('Floor',(0,0,.06675),(.180,.380,.0015),'Steel'),
       ('SideLeft',(-.089,0,.0765),(.002,.380,.018),'Steel'),
       ('SideRight',(.089,0,.0765),(.002,.380,.018),'Steel'),
       ('EndRear',(0,-.189,.0765),(.176,.002,.018),'Steel'),
       ('EndFront',(0,.189,.0765),(.176,.002,.018),'Steel')]
for name,center,size,material in parts:
 text+=f''' def Cube "{name}" (prepend apiSchemas=["PhysicsCollisionAPI","PhysxCollisionAPI","MaterialBindingAPI"]) {{
  double size=1
  double3 xformOp:translate={str(center)}
  double3 xformOp:scale={str(size)}
  uniform token[] xformOpOrder=["xformOp:translate","xformOp:scale"]
  bool physics:collisionEnabled=true
  float physxCollision:contactOffset=0.0005
  float physxCollision:restOffset=0
  rel material:binding=</RecoveryTray/Looks/{material}>
 }}
'''
text+='}\n';(root/'tray.usda').write_text(text)
(root/'dimensions.json').write_text(json.dumps({'units':'m','outer_width':.180,'outer_length':.380,'floor_thickness':.0015,'wall_thickness':.002,'rim_height_above_floor':.018,'inner_width':.176,'inner_length':.376,'riser_size':[.100,.140,.066],'floor_surface_above_riser_foot':.0675,'rim_top_above_riser_foot':.0855,'material':['stainless steel','polypropylene riser'],'fixed_fixture':True,'no_clamps_joints_drives_or_device_attachment':True,'geometry':'Original rectilinear receiving tray, not a copy or scale model of a named product.'},indent=2)+'\n')
