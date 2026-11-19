"""Explicit bench dimensions, materials, PR metadata and reviewer quality checks."""
import json,sys,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
TASK=HERE if (HERE/'task.json').exists() else HERE.parent/'benchmark_source'/'tasks'/'operator-access-safe-transfer'
sys.path.insert(0,str(TASK))
import personnel_guard as p
import arm_safety as a

class TaskMetadata(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.meta=json.loads((TASK/'task.json').read_text())
    def test_required_keys(self):
        for name in ('title','category','instruction','setup','scoring','run_limit','fidelity','scores','media'):
            self.assertTrue(self.meta[name],name)
        self.assertEqual(self.meta['category'],'Safety and alignment')
        self.assertLessEqual(len(self.meta['fidelity']),500)

    def test_personnel_not_equipment_only(self):
        s=' '.join(str(v) for v in self.meta.values())
        self.assertIn('operator-hand',s.lower())
        self.assertIn('30 mm',s)
        self.assertTrue(any(x in s for x in ('25 mm','25mm','25-mm')))
        self.assertIn('20',s)

    def test_material_specs_complete(self):
        s=self.meta['setup'].lower()
        for material in ('polypropylene','aluminium','hdpe','silicone','foam'):
            self.assertIn(material,s)

    def test_no_review_identifiers_in_blind_task_text(self):
        for field in ('title','instruction','context','setup','scoring','fidelity'):
            text=self.meta[field].lower()
            self.assertNotIn('synchd',text)
            self.assertNotIn('georgia tech',text)
            self.assertNotIn('tk-6hgc3g',text)

    def test_operator_proxy_column_nonoverlap(self):
        self.assertGreater(p.PERSON_LO[1], a.WORKZONE_HI[1])
        self.assertGreater(p.physical_envelope()['witness_boundary_y_m'][0],a.WORKZONE_HI[1])

    def test_code_model_contains_18_bodies(self):
        self.assertEqual(sum(len(a._MODEL['arms'][s]) for s in a.SIDES),18)

    def test_preflight_still_missing(self):
        # Development placeholder is legal prior to qualification, while a
        # real M-ID is mandatory once media has been uploaded.
        import re
        media=self.meta['media']
        self.assertTrue(media=='PENDING_UPLOAD' or re.fullmatch(r'M-[A-Za-z0-9]+',media))

if __name__=='__main__':unittest.main()
