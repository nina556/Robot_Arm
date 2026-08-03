import ast
import json
import math
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def load_function(path, name, namespace=None):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name)
    module = ast.Module(body=[function], type_ignores=[])
    scope = dict(namespace or {})
    exec(compile(module, str(path), "exec"), scope)
    return scope[name]


class VlmDepthGeometryTests(unittest.TestCase):
    def test_camera_calibration_fits_rigid_transform_with_holdout_points(self):
        rotation_to_rpy = load_function(
            ROOT / "web_control" / "web_control" / "server.py",
            "rotation_matrix_to_rpy",
            {"math": math},
        )
        function = load_function(
            ROOT / "web_control" / "web_control" / "server.py",
            "fit_camera_calibration_samples",
            {
                "np": np,
                "rotation_matrix_to_rpy": rotation_to_rpy,
                "load_camera_extrinsic": lambda: {},
            },
        )
        angle = 0.08
        rotation = np.array([
            [math.cos(angle), -math.sin(angle), 0.0],
            [math.sin(angle), math.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ])
        translation = np.array([0.12, -0.28, 1.55])
        camera_points = np.array([
            [-0.15, -0.10, 0.80], [0.0, -0.12, 0.82], [0.14, -0.08, 0.84],
            [-0.12, 0.08, 0.86], [0.02, 0.10, 0.88], [0.13, 0.07, 0.90],
            [-0.08, 0.0, 0.94], [0.10, 0.02, 0.96],
        ])
        samples = []
        for index, camera in enumerate(camera_points):
            samples.append({
                "camera_xyz_m": camera.tolist(),
                "truth_xyz_m": (rotation @ camera + translation).tolist(),
                "surface": "table" if index < 3 else "plate" if index < 6 else "block",
                "validation": index >= 6,
            })
        result = function(samples, {"image_rotation_deg": 0})
        fitted = result["fitted_extrinsic"]
        np.testing.assert_allclose([fitted["x"], fitted["y"], fitted["z"]], translation, atol=1e-9)
        self.assertAlmostEqual(angle, fitted["yaw"], places=9)
        self.assertLess(result["validation_metrics"]["max_xy_error_mm"], 1e-6)
        self.assertLess(result["validation_metrics"]["max_z_error_mm"], 1e-6)

    def test_camera_calibration_diagnoses_consistent_translation(self):
        function = load_function(
            ROOT / "web_control" / "web_control" / "server.py",
            "analyze_camera_calibration_samples",
            {"np": np},
        )
        samples = []
        for radius, truth in zip((0.05, 0.12, 0.2), ((0.12, -0.72, 0.84), (0.05, -0.80, 0.84), (-0.1, -0.65, 0.84))):
            samples.append({
                "truth_xyz_m": truth,
                "estimated_world_xyz_m": [truth[0] - 0.012, truth[1] + 0.004, truth[2] - 0.003],
                "pixel_radius_norm": radius,
            })
        result = function(samples)
        self.assertEqual("translation", result["diagnosis"])
        self.assertTrue(result["can_apply_translation"])
        np.testing.assert_allclose(result["mean_offset_m"], [0.012, -0.004, 0.003], atol=1e-9)

    def test_camera_calibration_rejects_radially_growing_error_as_translation(self):
        function = load_function(
            ROOT / "web_control" / "web_control" / "server.py",
            "analyze_camera_calibration_samples",
            {"np": np},
        )
        samples = []
        for radius, error in ((0.05, 0.004), (0.15, 0.014), (0.3, 0.030)):
            samples.append({
                "truth_xyz_m": [0.0, 0.0, 0.84],
                "estimated_world_xyz_m": [-error, 0.0, 0.84],
                "pixel_radius_norm": radius,
            })
        result = function(samples)
        self.assertEqual("rotation_or_intrinsic", result["diagnosis"])
        self.assertFalse(result["can_apply_translation"])

    def test_camera_rpy_uses_standard_zyx_order(self):
        function = load_function(
            ROOT / "web_control" / "web_control" / "server.py",
            "camera_attitude_matrix",
            {"math": math, "np": np},
        )
        matrix = function({"roll": math.pi / 2.0, "pitch": 0.0, "yaw": 0.0})
        np.testing.assert_allclose(matrix @ [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], atol=1e-9)

        frontend = (ROOT / "web_control" / "frontend" / "src" / "robotScene.js").read_text(encoding="utf-8")
        self.assertIn("const cr = Math.cos(Number(pose.roll));", frontend)
        self.assertIn("const cp = Math.cos(Number(pose.pitch));", frontend)

    def test_camera_extrinsic_declares_standard_convention(self):
        config = json.loads((ROOT / "vision" / "camera_extrinsic.json").read_text(encoding="utf-8"))
        self.assertEqual("standard_zyx", config["rpy_convention"])
        self.assertEqual(0, config["image_rotation_deg"])
        function = load_function(
            ROOT / "web_control" / "web_control" / "server.py",
            "camera_attitude_matrix",
            {"math": math, "np": np},
        )
        optical_forward = function(config) @ [0.0, 0.0, 1.0]
        np.testing.assert_allclose(
            optical_forward,
            [0.0, -math.sqrt(0.5), -math.sqrt(0.5)],
            atol=0.01,
        )

    def test_image_rotation_rotates_ray_without_moving_depth_sample(self):
        function = load_function(
            ROOT / "web_control" / "web_control" / "server.py",
            "rotate_camera_xy",
        )
        self.assertEqual((-0.01, -0.20), function(0.01, 0.20, 180))
        self.assertEqual((-0.20, 0.01), function(0.01, 0.20, 90))

    def test_mujoco_vertical_fov_uses_image_height(self):
        bridge = (ROOT / "asm0003" / "scripts" / "mujoco_sim_bridge.py").read_text(encoding="utf-8")
        expected = "self._height / (2.0 * math.tan(fovy_rad / 2.0))"
        self.assertIn(expected, bridge)
        focal = 480 / (2.0 * math.tan(math.radians(60) / 2.0))
        self.assertAlmostEqual(415.692, focal, places=3)

    def test_vlm_grasp_failure_is_not_reported_as_http_200(self):
        server = (ROOT / "web_control" / "web_control" / "server.py").read_text(encoding="utf-8")
        self.assertIn('self._send(422, {"ok": False, "error": result["error"]', server)

    def test_vlm_pick_uses_reachable_grasp_orientation_candidates(self):
        server = (ROOT / "web_control" / "web_control" / "server.py").read_text(encoding="utf-8")
        self.assertIn('"grasp_orientation_mode": "z_parallel_grasp"', server)
        self.assertIn("descend_distance=0.1", server)


if __name__ == "__main__":
    unittest.main()
