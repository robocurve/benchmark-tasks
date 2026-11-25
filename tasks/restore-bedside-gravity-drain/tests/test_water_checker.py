"""Exercise the worksheet verifier using temporary mathematical examples, not hardware data."""
import csv
import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location("water", Path(__file__).resolve().parents[1] / "physical/check_water.py")
water = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(water)


class Checks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.setup = self.root / "setup.json"
        self.sheet = self.root / "water.csv"
        self.setup.write_text(json.dumps(dict(fixture_id="synthetic-check", operator="test",
            source_scale_id="a", receiver_scale_id="b", water_density_g_ml=1,
            density_uncertainty_g_ml=.001, scale_resolution_g=.1, max_abs_check_weight_error_g=.05,
            max_repeat_spread_g=.1, survey_uncertainty_mm=.5, angle_uncertainty_deg=.5, timing_uncertainty_s=.03,
            restriction_length_mm=120, finished_mid_port_diameter_mm=.9)))
        record = self.root/"synthetic-record.txt"
        record.write_text("Synthetic numeric validation. No physical observations.")
        digest = hashlib.sha256(record.read_bytes()).hexdigest()
        self.rows=[]
        for case,(h,a) in water.CASES.items():
            delta=.01*max(0,h)*max(0,min(1,(a-10)/70))*15
            for rep in range(1,6):
                t=len(self.rows)*35
                self.rows.append([case,rep,500 if rep%2 else 330,h,h,a,t,t+15,t+30,
                                  600,600-delta,600-2*delta,20,20+delta,20+2*delta,0,
                                  record.name,digest])

        self.sweep = self.root/"sweep.csv"
        self.wet=[]
        for direction,points in (("up",range(0,91,5)),("down",range(90,-1,-5))):
            for angle in points:
                volume=water.model(50,angle,30)
                t=2000+len(self.wet)*35
                self.wet.append([direction,angle,angle,50,t,t+30,20,20+volume,record.name,digest])

    def check(self, include_sweep=False):
        with self.sheet.open("w",newline="") as f:
            writer=csv.writer(f)
            writer.writerow(water.FIELDS)
            writer.writerows(self.rows)
        if include_sweep:
            with self.sweep.open("w",newline="") as f:
                writer=csv.writer(f)
                writer.writerow(water.SWEEP_FIELDS)
                writer.writerows(self.wet)
        return water.observations(self.setup,self.sheet,self.sweep if include_sweep else None)

    def test_complete(self):
        self.assertEqual(self.check()["collector_totals_ml"]["low"],[51]*5)
    def test_full_sweep(self):
        self.assertEqual(len(self.check(True)["valve_sweep_ml"]["down"]),19)
    def test_sweep_absent(self):
        self.assertIn("unverified",self.check()["valve_sweep_ml"])
    def test_sweep_incomplete(self):
        self.wet.pop()
        with self.assertRaisesRegex(ValueError,"all 38"):
            self.check(True)
    def test_sweep_response(self):
        self.wet[9][7]+=3
        with self.assertRaisesRegex(ValueError,"response outside model"):
            self.check(True)
    def test_sweep_reversed(self):
        self.wet[9][7]=self.wet[8][7]-1
        with self.assertRaisesRegex(ValueError,"nonmonotonic"):
            self.check(True)
    def test_sweep_reused_35_point_record(self):
        self.wet[0][4:6]=[0,30]
        with self.assertRaisesRegex(ValueError,"reused clip"):
            self.check(True)
    def test_guard_band(self):
        self.rows[0][13]=20+6.1
        with self.assertRaisesRegex(ValueError,"inconclusive"):
            self.check()
    def test_partial(self):
        self.rows.pop()
        with self.assertRaisesRegex(ValueError,"five independent"):
            self.check()
    def test_fill(self):
        self.rows[1][2]=500
        with self.assertRaisesRegex(ValueError,"starting fill"):
            self.check()
    def test_timing(self):
        self.rows[0][7]=14
        with self.assertRaisesRegex(ValueError,"timing"):
            self.check()
    def test_balances(self):
        self.rows[0][14]+=2
        with self.assertRaisesRegex(ValueError,"disagreement"):
            self.check()
    def test_two_distinct_windows(self):
        self.rows[0][13]+=2
        with self.assertRaisesRegex(ValueError,"15 s delivery"):
            self.check()
    def test_returning_head(self):
        self.rows[0][3:5]=[49,51]
        with self.assertRaisesRegex(ValueError,"drift"):
            self.check()
    def test_duplicate_record(self):
        self.rows[1][6:9]=self.rows[0][6:9]
        with self.assertRaisesRegex(ValueError,"reused clip"):
            self.check()
    def test_missing_record(self):
        self.rows[0][-2]="missing"
        with self.assertRaisesRegex(ValueError,"must exist"):
            self.check()
    def test_modified_record(self):
        self.rows[0][-1]="..."
        with self.assertRaisesRegex(ValueError,"checksum"):
            self.check()
    def test_nonnumeric(self):
        self.rows[0][3]="nan"
        with self.assertRaisesRegex(ValueError,"finite"):
            self.check()
    def test_blank_inputs(self):
        p=Path(__file__).resolve().parents[1]/"physical/forms"
        with self.assertRaisesRegex(ValueError,"measured setup"):
            water.observations(p/"water_setup.json",p/"head_flow_calibration.csv")


if __name__ == "__main__":
    unittest.main()
