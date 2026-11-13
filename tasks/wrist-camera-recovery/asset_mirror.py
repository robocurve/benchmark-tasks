"""Use NVIDIA's public production mirror for the same versioned Arena assets.
The staging 6.1 URLs return404; production6.1 hosts these exact named assets.
No mesh, scale or collider replacement.
"""
def install():
 import isaaclab_arena.assets.object_library as library
 audit={}
 for name in ['Apple01ObjaverseRobolab','Orange02FruitsVeggiesRobolab']:
  cls=getattr(library,name);before=cls.usd_path
  cls.usd_path=before.replace('omniverse-content-staging','omniverse-content-production')
  audit[name]={'original':before,'public_mirror':cls.usd_path}
 return audit
