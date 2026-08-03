import importlib.util
import json
import random
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PROBE_PATH = ROOT / "tools" / "reachability_probe.py"


def load_probe_module():
    spec = importlib.util.spec_from_file_location("reachability_probe", PROBE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ReachabilityProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.probe = load_probe_module()

    def test_config_regions_replace_defaults(self):
        config = {
            "regions": {
                "custom_region": {
                    "sampler": "box",
                    "arm": "right",
                    "x": [-0.3, -0.2],
                    "y": [-0.2, -0.1],
                    "z": [1.1, 1.2],
                }
            },
            "checkers": [{"name": "geometry"}],
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "probe.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            loaded = self.probe.load_config(path)
        self.assertEqual(["custom_region"], list(loaded["regions"].keys()))

    def test_geometry_checker_runs_without_ros(self):
        cfg = self.probe.default_config()
        cfg["regions"] = {
            "fixed_right": {
                "sampler": "box",
                "arm": "right",
                "x": -0.225,
                "y": -0.155,
                "z": 1.10,
                "roll": 3.1415926536,
                "pitch": 0.0,
                "yaw": 0.0,
            }
        }
        sampler = self.probe.build_region_samplers(cfg, ["fixed_right"])[0]
        sample = sampler.sample(0, random.Random(1))
        result = self.probe.GeometryReachChecker(cfg).check(sample)

        self.assertIsInstance(result.ok, bool)
        self.assertIn("distance", result.details)
        self.assertEqual("right_base_link", result.details["base_link"])

    def test_sampling_enforces_arm_reach_boundary(self):
        cfg = self.probe.default_config()
        cfg["regions"] = {
            "wide_right": {
                "sampler": "box",
                "arm": "right",
                "x": [-1.2, -0.225],
                "y": -0.155,
                "z": 1.308,
                "roll": 3.1415926536,
                "pitch": 0.0,
                "yaw": 0.0,
            }
        }
        sampler = self.probe.build_region_samplers(cfg, ["wide_right"])[0]
        constraint = self.probe.ArmReachSamplingConstraint(cfg)
        sample = self.probe.sample_with_constraints(sampler, 0, random.Random(3), constraint, 1000)
        result = self.probe.GeometryReachChecker(cfg).check(sample)

        self.assertTrue(result.ok)
        self.assertTrue(sample.metadata["arm_reach_enforced"])
        self.assertLessEqual(sample.metadata["arm_reach_distance"], sample.metadata["arm_max_reach"])

    def test_sampling_rejects_region_outside_arm_reach(self):
        cfg = self.probe.default_config()
        cfg["regions"] = {
            "outside_right": {
                "sampler": "box",
                "arm": "right",
                "x": -10.0,
                "y": -0.155,
                "z": 1.308,
            }
        }
        sampler = self.probe.build_region_samplers(cfg, ["outside_right"])[0]
        constraint = self.probe.ArmReachSamplingConstraint(cfg)

        with self.assertRaises(RuntimeError):
            self.probe.sample_with_constraints(sampler, 0, random.Random(1), constraint, 3)

    def test_moveit_error_code_names(self):
        self.assertEqual("SUCCESS", self.probe.MOVEIT_ERROR_NAMES[1])
        self.assertEqual("FAILURE", self.probe.MOVEIT_ERROR_NAMES[99999])
        self.assertEqual("PLANNING_FAILED", self.probe.MOVEIT_ERROR_NAMES[-1])


if __name__ == "__main__":
    unittest.main()
