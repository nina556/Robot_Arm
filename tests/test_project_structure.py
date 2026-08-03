import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FRONTEND = ROOT / "web_control" / "frontend"
URDF_ROOT = FRONTEND / "public" / "urdf"


class ProjectStructureTests(unittest.TestCase):
    def test_required_frontend_sources_exist(self):
        required = [
            FRONTEND / "index.html",
            FRONTEND / "src" / "main.js",
            FRONTEND / "src" / "controller.js",
            FRONTEND / "src" / "robotScene.js",
            FRONTEND / "src" / "api.js",
            FRONTEND / "src" / "styles.css",
            FRONTEND / "public" / "models" / "part.stl",
            URDF_ROOT / "unoarm.urdf",
        ]
        self.assertEqual([], [str(path) for path in required if not path.is_file()])

    def test_ui_preserves_control_surfaces(self):
        html = (FRONTEND / "index.html").read_text(encoding="utf-8")
        required_ids = {
            "robotScene",
            "planButton",
            "executeButton",
            "emergencyStop",
            "enableButton",
            "homeButton",
            "cameraFields",
            "jointConfigFields",
            "pointsList",
            "presetsList",
            "sequenceList",
            "logViewer",
            "omplPresetName",
            "saveOmplPresetButton",
            "deleteOmplPresetButton",
            "cameraCalibrationPoints",
            "runCameraCalibrationButton",
            "applyCameraTranslationButton",
        }
        ids = set(re.findall(r'id="([^"]+)"', html))
        self.assertEqual(set(), required_ids - ids)

    def test_real_urdf_meshes_are_packaged(self):
        urdf_path = URDF_ROOT / "unoarm.urdf"
        root = ET.parse(urdf_path).getroot()
        mesh_paths = {
            mesh.attrib["filename"]
            for mesh in root.findall(".//mesh")
            if "filename" in mesh.attrib
        }
        missing = [name for name in mesh_paths if not (URDF_ROOT / name).is_file()]
        self.assertEqual([], missing)

        movable = [
            joint.attrib["name"]
            for joint in root.findall("joint")
            if joint.attrib.get("type") in {"revolute", "continuous", "prismatic"}
        ]
        for side in ("Left", "Right"):
            for index in range(1, 8):
                self.assertIn(f"{side}_Joint{index}", movable)

    def test_legacy_link_simulator_is_not_in_new_frontend(self):
        sources = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (FRONTEND / "src").glob("*.js")
        )
        self.assertNotIn("drawArm3", sources)
        self.assertIn("URDFLoader", sources)

    def test_vision_target_uses_part_stl(self):
        scene = (FRONTEND / "src" / "robotScene.js").read_text(encoding="utf-8")
        self.assertIn("STLLoader", scene)
        self.assertIn("VISION_PART_MODEL_URL = '/models/part.stl'", scene)
        self.assertIn("ensureVisionPartMarker", scene)
        self.assertNotIn("new SphereGeometry(0.024", scene)

    def test_python_server_serves_built_frontend(self):
        server = (ROOT / "web_control" / "web_control" / "server.py").read_text(encoding="utf-8")
        self.assertIn("UNOARM_FRONTEND_DIST", server)
        self.assertIn('path.startswith(("/assets/", "/urdf/", "/models/"))', server)
        self.assertIn("clear_platform_obstacle", server)
        self.assertIn("/api/platform_obstacle/clear", server)
        self.assertIn("prepare_frontend_static_cache", server)
        self.assertIn("Content-Encoding", server)
        self.assertIn("ETag", server)
        self.assertIn("def _plan_home_zero", server)
        self.assertIn('mode="MoveIt planned home"', server)
        self.assertIn('config["ignore_workspace_bounds"] = True', server)
        self.assertIn('"workspace_bounds_ignored"', server)
        self.assertIn("sync_manual_collision_boxes_for_request", server)
        self.assertIn("manual_collision_boxes_from_body", server)
        self.assertIn("stale_ids = sorted(self._manual_collision_ids - desired_ids)", server)
        self.assertIn("manual collision boxes already absent", server)
        self.assertNotIn("for index in range(MANUAL_COLLISION_MAX_BOXES):", server)

    def test_joint_keyboard_control_is_wired_end_to_end(self):
        controller = (FRONTEND / "src" / "controller.js").read_text(encoding="utf-8")
        server = (ROOT / "web_control" / "web_control" / "server.py").read_text(encoding="utf-8")
        self.assertIn("JOINT_KEY_PAIRS", controller)
        self.assertIn("bindKeyboardControls", controller)
        self.assertIn("isEditableTarget", controller)
        self.assertIn("'/api/joint_jog'", controller)
        self.assertIn("def jog_joint(", server)
        self.assertIn('path == "/api/joint_jog"', server)
        self.assertIn("get_subscription_count", server)

    def test_mujoco_rgb_camera_is_wired_to_web(self):
        bridge = (ROOT / "asm0003" / "scripts" / "mujoco_sim_bridge.py").read_text(encoding="utf-8")
        scene_builder = (ROOT / "asm0003" / "scripts" / "build_mujoco_camera_scene.py").read_text(encoding="utf-8")
        robot_scene = (FRONTEND / "src" / "robotScene.js").read_text(encoding="utf-8")
        launcher = (ROOT / "start_mujoco_sim.sh").read_text(encoding="utf-8")
        server = (ROOT / "web_control" / "web_control" / "server.py").read_text(encoding="utf-8")
        html = (FRONTEND / "index.html").read_text(encoding="utf-8")
        controller = (FRONTEND / "src" / "controller.js").read_text(encoding="utf-8")
        verifier = (ROOT / "tools" / "verify_mujoco_rgbd.py").read_text(encoding="utf-8")

        self.assertIn("class MujocoCameraPublisher", bridge)
        self.assertIn('msg.encoding = "rgb8"', bridge)
        self.assertIn('depth_msg.encoding = "32FC1"', bridge)
        self.assertIn("renderer.enable_depth_rendering()", bridge)
        self.assertIn("renderer.disable_depth_rendering()", bridge)
        self.assertIn('name="mujoco_simulation_and_viewer"', bridge)
        self.assertIn("self._viewer_initialization_done.wait", bridge)
        self.assertIn("def stop_simulation(self):", bridge)
        self.assertIn("CameraInfo", bridge)
        self.assertIn("/mujoco_camera/depth/image_raw", bridge)
        self.assertIn("/mujoco_camera/color/camera_info", bridge)
        self.assertIn('default="vlm_camera_link"', bridge)
        self.assertIn("/mujoco_camera/color/image_raw", bridge)
        self.assertIn('camera=self._camera_name', bridge)
        self.assertIn('"name": "head_d455_rgb"', scene_builder)
        self.assertIn('"name": "overview_camera"', scene_builder)
        self.assertIn('"name": "work_table"', scene_builder)
        self.assertIn('"name": "grasp_object"', scene_builder)
        self.assertIn("MUJOCO_CAMERA_ENABLED", launcher)
        self.assertIn('path == "/api/mujoco_camera/status"', server)
        self.assertIn('path == "/mujoco_camera/snapshot.jpg"', server)
        self.assertIn('path == "/api/mujoco_depth/status"', server)
        self.assertIn('path == "/mujoco_depth/snapshot.jpg"', server)
        self.assertIn('path == "/api/mujoco/grasp_target"', server)
        self.assertIn('path == "/api/mujoco/visual_grasp_target"', server)
        self.assertIn("def mujoco_visual_grasp_target(self):", server)
        self.assertIn('visual grasp target unavailable', server)
        self.assertIn('path == "/api/mujoco/camera_calibration/sample"', server)
        self.assertIn('path == "/api/mujoco/calibration_fixtures"', server)
        self.assertIn("def set_mujoco_calibration_fixtures(self, body):", server)
        self.assertIn('path == "/api/mujoco/camera_calibration/analyze"', server)
        self.assertIn('path == "/api/mujoco/camera_calibration/fit"', server)
        self.assertIn("self._head_rgb_history = deque(maxlen=12)", server)
        self.assertIn("abs(rgb_stamp - depth_stamp) <= 1e-6", server)
        self.assertIn("now = time.monotonic_ns()", bridge)
        self.assertIn('f"/mujoco/{fixture}_visible"', bridge)
        self.assertIn("def _on_fixture_visibility", bridge)
        self.assertIn('/mujoco_camera/stream.mjpg', controller)
        self.assertIn('getMujocoVisualGraspTarget', controller)
        self.assertEqual(
            controller.count('const response = await getMujocoVisualGraspTarget();'),
            3,
        )
        self.assertIn('Date.now() + 12000', controller)
        self.assertIn('path == "/api/mujoco/scene"', server)
        self.assertIn("MUJOCO_GRASP_TCP_OFFSET", server)
        self.assertIn("_wait_for_sim_gripper", server)
        self.assertIn("SIM_GRIPPER_CONTACT_SETTLE_SEC", server)
        self.assertIn("pick: using live MuJoCo object position", server)
        self.assertIn('"object_attached": object_attached', server)
        self.assertIn('"gripper_width":', server)
        self.assertIn('id="simCameraImage"', html)
        self.assertIn('id="rightWristCameraImage"', html)
        self.assertIn('id="leftWristCameraImage"', html)
        self.assertIn('id="pickMujocoObjectButton"', html)
        self.assertIn('id="leftDropCylinderButton"', html)
        self.assertIn("智能抓取转子到圆柱中心", html)
        self.assertIn('id="reachabilityOverlayToggle"', html)
        self.assertIn('data-panel="camera-calibration"', html)
        self.assertIn('data-panel-content="camera-calibration"', html)
        self.assertIn('id="cameraCalibrationPlateToggle"', html)
        self.assertIn('id="cameraCalibrationBlockToggle"', html)
        self.assertLess(
            html.index('id="reachabilityOverlayToggle"'),
            html.index('<span class="eyebrow">BENCHMARK</span><h2>规划器跑分</h2>'),
        )
        self.assertIn("refreshMujocoCamera", controller)
        self.assertIn("refreshWristCameras", controller)
        self.assertIn("refreshMujocoScene", controller)
        self.assertIn("setMujocoScene(data)", robot_scene)
        self.assertIn("getMujocoGraspTarget", controller)
        self.assertIn("sampleMujocoCameraCalibration", controller)
        self.assertIn("runCameraCalibration", controller)
        self.assertIn("fitMujocoCameraCalibration", controller)
        self.assertIn("CAMERA_CALIBRATION_FRAMES = 5", controller)
        self.assertIn("pickMujocoObject", controller)
        self.assertIn("unoarm.reachabilityVisible", controller)
        self.assertIn("toggleReachabilityVisibility", controller)
        self.assertIn("setGraspPoseMode('z_parallel')", controller)
        self.assertIn("smart_search: true", controller)
        self.assertIn("grasp_modes: ['vertical_grasp', 'angled_grasp', 'side_grasp']", controller)
        self.assertIn("release_descend_height: 0.045", controller)
        self.assertIn('smart_search=parse_bool(body.get("smart_search"), False)', server)
        self.assertIn("search_offsets=body.get(\"search_offsets\")", server)
        self.assertIn("grasp_modes=body.get(\"grasp_modes\")", server)
        self.assertIn('"descend_to_release_distance"', server)
        self.assertIn('geom.set("condim", "4")', scene_builder)
        self.assertIn('"name": "calibration_block_geom"', scene_builder)
        self.assertIn('"object_motion": object_motion', bridge)
        self.assertIn('"name": "grasp_object_part_mesh"', scene_builder)
        self.assertIn('"mesh": "grasp_object_part_mesh"', scene_builder)
        self.assertIn("UNOARM_GRIPPER_CLOSED_POSITION", bridge)
        self.assertIn("def _grasp_debug_payload", bridge)
        self.assertIn('"contact_points": object_contacts', bridge)
        self.assertIn('"left_contact_geoms": sorted(set(left_geoms))', bridge)
        self.assertIn('"right_contact_geoms": sorted(set(right_geoms))', bridge)
        self.assertIn("def _attach_grasp_object", bridge)
        self.assertIn("def _detach_grasp_object", bridge)
        self.assertIn("def _sync_attached_object_pose", bridge)
        self.assertIn("self._grasp_weld_active = True", bridge)
        self.assertIn('"object_attached": bool(', bridge)
        self.assertIn('"name": f"{side.lower()}_grasp_weld"', scene_builder)
        self.assertIn('"contact_stopped": bool(left_geoms and right_geoms)', bridge)
        self.assertIn("synchronized_stamp", verifier)
        self.assertIn('depth["data_bytes"] == depth["width"] * depth["height"] * 4', verifier)

    def test_web_benchmark_feature_is_wired_end_to_end(self):
        html = (FRONTEND / "index.html").read_text(encoding="utf-8")
        controller = (FRONTEND / "src" / "controller.js").read_text(encoding="utf-8")
        scene = (FRONTEND / "src" / "robotScene.js").read_text(encoding="utf-8")
        api = (FRONTEND / "src" / "api.js").read_text(encoding="utf-8")
        server = (ROOT / "web_control" / "web_control" / "server.py").read_text(encoding="utf-8")

        required_ids = {
            "benchmarkState",
            "benchmarkCollisionBox",
            "benchmarkBoxCount",
            "benchmarkEdgeCount",
            "benchmarkRandomCount",
            "benchmarkPlanners",
            "generateBenchmarkButton",
            "runBenchmarkButton",
            "benchmarkPresetName",
            "saveBenchmarkPresetButton",
            "exportBenchmarkCsvButton",
            "benchmarkProgress",
            "benchmarkProgressBar",
            "benchmarkSummary",
            "benchmarkSamples",
        }
        ids = set(re.findall(r'id="([^"]+)"', html))
        self.assertEqual(set(), required_ids - ids)
        self.assertIn('data-panel="benchmark"', html)
        self.assertIn("generateBenchmarkSet", controller)
        self.assertIn("runBenchmarkSet", controller)
        self.assertIn("currentManualCollisionBoxes", controller)
        self.assertIn("collision_boxes: currentManualCollisionBoxes()", controller)
        self.assertIn("saveBenchmarkRuntimePreset", controller)
        self.assertIn("exportCurrentBenchmarkCsv", controller)
        self.assertIn("setBenchmarkSamples", scene)
        self.assertIn("generateBenchmarkSamples", api)
        self.assertIn("getBenchmarkProgress", api)
        self.assertIn("runBenchmark", api)
        self.assertIn("savePlanningPreset", api)
        self.assertIn("exportBenchmarkCsv", api)
        self.assertIn("/api/benchmark/generate", server)
        self.assertIn("/api/benchmark/progress", server)
        self.assertIn("/api/benchmark/run", server)
        self.assertIn("/api/planning_presets", server)
        self.assertIn("/api/benchmark/export_csv", server)
        self.assertIn("BENCHMARK_MAX_SAMPLES", server)
        self.assertIn("PLANNING_PRESETS_FILE", server)
        self.assertIn("save_benchmark_csv", server)
        self.assertIn("ik_scanned_count", server)
        self.assertIn("plan_scanned_count", server)
        self.assertIn('UNOARM_HORIZONTAL_GRASP_YAW_STEP_DEG", "15"', server)
        self.assertIn("_benchmark_grasp_plan", server)
        self.assertIn("summary = self._plan_single_pose(candidate)", server)
        self.assertIn('"scanned_count": index', server)
        self.assertIn("_set_benchmark_progress", server)
        self.assertIn("_benchmark_ik", server)
        self.assertIn("_benchmark_plan", server)


if __name__ == "__main__":
    unittest.main()
