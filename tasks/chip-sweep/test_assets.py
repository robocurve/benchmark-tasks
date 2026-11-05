"""Check the authored tool/undercut relationship, not only placeholder boxes."""
import unittest
from pathlib import Path
from pxr import Usd
HERE=Path(__file__).resolve().parent/'assets'
class UndercutGeometry(unittest.TestCase):
 def test_T_foot_needs_end_entry_and_fits_chamber(self):
  tool=Usd.Stage.Open(str(HERE/'rake.usdc'));fixture=Usd.Stage.Open(str(HERE/'chip_tray.usdc'))
  def box(stage,name):
   p=stage.GetPrimAtPath('/Asset/'+name)
   return tuple(p.GetAttribute('xformOp:translate').Get()),tuple(p.GetAttribute('xformOp:scale').Get())
  _,foot=box(tool,'T_foot');_,neck=box(tool,'neck')
  lp,ls=box(fixture,'ledge_1_0');rp,rs=box(fixture,'ledge_1_1')
  slot=(rp[1]-rs[1]/2)-(lp[1]+ls[1]/2)
  a,sa=box(fixture,'rib_1');b,sb=box(fixture,'rib_2')
  chamber=(b[1]-sb[1]/2)-(a[1]+sa[1]/2)
  self.assertLess(neck[1],slot)
  self.assertLess(slot,foot[1])
  self.assertLess(foot[1],chamber)
  self.assertAlmostEqual(slot,.014,places=7)
  self.assertAlmostEqual(chamber,.032,places=7)
if __name__=='__main__':unittest.main()
