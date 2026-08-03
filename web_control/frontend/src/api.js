export class ApiError extends Error {
  constructor(message, status = 0, payload = null) {
    super(message);
    this.name = 'ApiError';
    this.status = status;
    this.payload = payload;
  }
}

export async function apiRequest(path, body) {
  const response = await fetch(path, {
    method: body === undefined ? 'GET' : 'POST',
    headers: body === undefined ? {} : { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
    cache: body === undefined ? 'no-store' : 'default'
  });

  const contentType = response.headers.get('content-type') ?? '';
  const payload = contentType.includes('application/json')
    ? await response.json()
    : await response.text();

  if (!response.ok || payload?.ok === false) {
    const message = payload?.error || payload?.message || `HTTP ${response.status}`;
    throw new ApiError(message, response.status, payload);
  }

  return payload;
}

export function getStatus() {
  return apiRequest('/api/status');
}

export function getRobotState() {
  return apiRequest('/api/robot_state');
}

export function getLinks(arm) {
  return apiRequest(`/api/links?arm=${encodeURIComponent(arm)}`);
}

export function getGripper(arm) {
  return apiRequest(`/api/gripper?arm=${encodeURIComponent(arm)}`);
}

export function getVisionTarget() {
  return apiRequest('/api/vision_target');
}

export function getVisionStatus() {
  return apiRequest('/api/vision_status');
}

export function getMujocoCameraStatus() {
  return apiRequest('/api/mujoco_camera/status');
}

export function getRightWristCameraStatus() {
  return apiRequest('/api/right_wrist_camera/status');
}

export function getLeftWristCameraStatus() {
  return apiRequest('/api/left_wrist_camera/status');
}

export function getOverviewCameraStatus() {
  return apiRequest('/api/overview_camera/status');
}

export function getMujocoDepthStatus() {
  return apiRequest('/api/mujoco_depth/status');
}

export function moveObject(body) {
  return apiRequest('/api/mujoco/move_object', body);
}

export function setMujocoCalibrationFixtures(body) {
  return apiRequest('/api/mujoco/calibration_fixtures', body);
}

export function rebuildScene(body) {
  return apiRequest('/api/mujoco/rebuild_scene', body);
}

export function getMujocoGraspTarget() {
  return apiRequest('/api/mujoco/grasp_target');
}

export function getMujocoVisualGraspTarget() {
  return apiRequest('/api/mujoco/visual_grasp_target');
}

export function sampleMujocoCameraCalibration() {
  return apiRequest('/api/mujoco/camera_calibration/sample', {});
}

export function analyzeMujocoCameraCalibration(samples) {
  return apiRequest('/api/mujoco/camera_calibration/analyze', { samples });
}

export function fitMujocoCameraCalibration(samples, baseExtrinsic) {
  return apiRequest('/api/mujoco/camera_calibration/fit', {
    samples,
    base_extrinsic: baseExtrinsic
  });
}

export function getMujocoScene() {
  return apiRequest('/api/mujoco/scene');
}

export function runLeftSideDropRotorTask(body = {}) {
  return apiRequest('/api/mujoco/left_side_drop_rotor', body);
}

export function getCameraExtrinsic() {
  return apiRequest('/api/camera_extrinsic');
}

export function getJointConfig(arm) {
  return apiRequest(`/api/joint_config?arm=${encodeURIComponent(arm)}`);
}

export function getKinematics() {
  return apiRequest('/api/kinematics');
}

export function getOmplConfig() {
  return apiRequest('/api/ompl_config');
}

export function getPlanningPresets() {
  return apiRequest('/api/planning_presets');
}

export function savePlanningPreset(body) {
  return apiRequest('/api/planning_presets', body);
}

export function deletePlanningPreset(body) {
  return apiRequest('/api/planning_presets/delete', body);
}

export function getBenchmarkOptions(arm) {
  return apiRequest(`/api/benchmark/options?arm=${encodeURIComponent(arm)}`);
}

export function getBenchmarkProgress(arm) {
  return apiRequest(`/api/benchmark/progress?arm=${encodeURIComponent(arm)}`);
}

export function generateBenchmarkSamples(body) {
  return apiRequest('/api/benchmark/generate', body);
}

export function runBenchmark(body) {
  return apiRequest('/api/benchmark/run', body);
}

export function exportBenchmarkCsv(body) {
  return apiRequest('/api/benchmark/export_csv', body);
}

export function getPlatformObstacle() {
  return apiRequest('/api/platform_obstacle');
}

export function getManualCollisionBoxes() {
  return apiRequest('/api/manual_collision_boxes');
}

export function getWorkspaceBounds() {
  return apiRequest('/api/workspace_bounds');
}

export function saveWorkspaceBounds(body) {
  return apiRequest('/api/workspace_bounds', body);
}

export function getVlmConfig() {
  return apiRequest('/api/vlm_config');
}

export function saveVlmConfig(body) {
  return apiRequest('/api/vlm_config', body);
}

export function callVlm(body) {
  return apiRequest('/api/vlm_call', body);
}

export function vlmGrasp(body) {
  return apiRequest('/api/vlm_grasp', body);
}

export function getPosesLib() {
  return apiRequest('/api/poses_lib');
}

export function savePosesLib(body) {
  return apiRequest('/api/poses_lib', body);
}

export function deletePoseLib(body) {
  return apiRequest('/api/poses_lib/delete', body);
}

export function saveManualCollisionBoxes(body) {
  return apiRequest('/api/manual_collision_boxes', body);
}

export function applyManualCollisionBoxes(body) {
  return apiRequest('/api/manual_collision_boxes/apply', body);
}

export function clearManualCollisionBoxes() {
  return apiRequest('/api/manual_collision_boxes/clear', {});
}

export function getPoints() {
  return apiRequest('/api/points');
}

export function getPresets() {
  return apiRequest('/api/presets');
}

export function getLogs(limit = 200) {
  return apiRequest(`/api/logs?n=${limit}`);
}
