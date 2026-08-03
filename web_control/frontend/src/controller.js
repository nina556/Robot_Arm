import {
  apiRequest,
  applyManualCollisionBoxes,
  analyzeMujocoCameraCalibration,
  clearManualCollisionBoxes,
  deletePlanningPreset,
  exportBenchmarkCsv,
  fitMujocoCameraCalibration,
  getCameraExtrinsic,
  getBenchmarkOptions,
  getBenchmarkProgress,
  getGripper,
  getJointConfig,
  getKinematics,
  getLinks,
  getLogs,
  getManualCollisionBoxes,
  getMujocoCameraStatus,
  getRightWristCameraStatus,
  getLeftWristCameraStatus,
  getMujocoGraspTarget,
  getMujocoVisualGraspTarget,
  moveObject,
  rebuildScene,
  getMujocoScene,
  runLeftSideDropRotorTask,
  getOmplConfig,
  getPlatformObstacle,
  getPoints,
  getPresets,
  getRobotState,
  getStatus,
  getVisionStatus,
  getVisionTarget,
  getWorkspaceBounds,
  generateBenchmarkSamples,
  saveManualCollisionBoxes,
  savePlanningPreset,
  saveWorkspaceBounds,
  setMujocoCalibrationFixtures,
  runBenchmark,
  sampleMujocoCameraCalibration,
  getVlmConfig,
  saveVlmConfig,
  callVlm,
  vlmGrasp,
  getPosesLib,
  savePosesLib,
  deletePoseLib
} from './api.js';
import { RobotScene } from './robotScene.js';
import {
  byId,
  confirmAction,
  escapeHtml,
  formatNumber,
  refreshIcons,
  runCommand,
  setBadge,
  setStatus,
  showToast
} from './ui.js';

const POSE_FIELDS = [
  ['x', 'X', -1, 1, 0.001, 0.327],
  ['y', 'Y', -1, 1, 0.001, 0.018],
  ['z', 'Z', 0.1, 1.8, 0.001, 0.514]
];

const CAMERA_FIELDS = [
  ['x', 'X', -2, 2, 0.001, 0],
  ['y', 'Y', -2, 2, 0.001, 0],
  ['z', 'Z', 0, 3, 0.001, 1.4],
  ['roll', 'Roll', -3.142, 3.142, 0.001, 0],
  ['pitch', 'Pitch', -3.142, 3.142, 0.001, -1.571],
  ['yaw', 'Yaw', -3.142, 3.142, 0.001, 3.142],
  ['image_rotation_deg', 'Image Rot', 0, 270, 90, 180]
];

const JOINT_KEY_PAIRS = [
  ['1', 'q'],
  ['2', 'w'],
  ['3', 'e'],
  ['4', 'r'],
  ['5', 't'],
  ['6', 'y'],
  ['7', 'u']
];
const JOINT_JOG_STEP_RAD = 0.8 * Math.PI / 180;
const JOINT_JOG_REPEAT_MS = 180;
const GRIPPER_OPEN_POSITION = 0;
const GRIPPER_CLOSED_POSITION = -0.91;
const GRIPPER_COMMAND_DEBOUNCE_MS = 350;
const GRIPPER_STATUS_HOLD_MS = 900;
const GRIPPER_MIMIC_BY_ARM = {
  right: {
    Right_Gripper_Joint: 1,
    Right_Gripper_Left_Support_Joint: -1,
    Right_Gripper_Left_2_Joint: 1,
    Right_Gripper_Right_2_Joint: -1,
    Right_Gripper_Right_1_Joint: -1,
    Right_Gripper_Right_Support_Joint: -1
  },
  left: {
    Left_Gripper_Joint: 1,
    Left_Gripper_Left_Support_Joint: -1,
    Left_Gripper_Left_2_Joint: 1,
    Left_Gripper_Right_2_Joint: -1,
    Left_Gripper_Right_1_Joint: -1,
    Left_Gripper_Right_Support_Joint: -1
  }
};

const ACTION_LABELS = {
  approach: '接近',
  grasp: '抓取',
  lift: '抬起',
  move: '移动',
  release: '释放',
  close: '闭合',
  pick: '自动夹取',
  wait: '等待'
};
const DEFAULT_GRASP_POSE_MODE = 'side';
const GRASP_POSE_DEFAULTS = {
  vertical: { roll: 0, pitch: -Math.PI / 2, yaw: Math.PI, grasp_orientation_mode: 'vertical_grasp' },
  side: { roll: Math.PI / 2, pitch: 0, yaw: 0, grasp_orientation_mode: 'side_grasp' },
  z_parallel: { roll: 0, pitch: Math.PI / 6, yaw: 0, grasp_orientation_mode: 'angled_grasp' }
};
const DECISION_MODE_DETAILS = {
  vertical_grasp: {
    title: '垂直抓取',
    summary: 'IK 先验证垂直末端姿态，再按候选姿态逐个规划。',
    scanLabel: '垂直候选',
    candidateText: orientation => describeCandidateAngles(orientation)
  },
  side_grasp: {
    title: '侧面抓取',
    summary: 'IK 扫描侧向接近姿态，找到可规划候选后锁定。',
    scanLabel: '侧向候选',
    candidateText: orientation => describeCandidateAngles(orientation)
  },
  angled_grasp: {
    title: '斜向抓取',
    summary: 'IK 扫描斜向抓取姿态，兼顾目标方向和碰撞约束。',
    scanLabel: '斜向候选',
    candidateText: orientation => describeCandidateAngles(orientation)
  },
  z_parallel_grasp: {
    title: '斜向抓取',
    summary: 'IK 扫描斜向抓取姿态，兼顾目标方向和碰撞约束。',
    scanLabel: '斜向候选',
    candidateText: orientation => describeCandidateAngles(orientation)
  },
  flexible_grasp: {
    title: '灵活抓取',
    summary: 'IK 扫描圆柱接触方向，先确认预抓取，再确认抓取位姿。',
    scanLabel: '接触角',
    candidateText: orientation => Number.isFinite(Number(orientation?.theta_deg))
      ? `theta ${formatNumber(orientation.theta_deg, 1)}°`
      : describeCandidateAngles(orientation)
  }
};
const SMART_GRASP_MODE_LABELS = {
  vertical_grasp: '垂直抓取',
  angled_grasp: '斜向抓取',
  side_grasp: '侧面抓取'
};
const OMPL_CONFIG_STORAGE_KEY = 'unoarm.omplConfig';
const DEFAULT_OMPL_CONFIG = {
  planner_id: 'RRTConnectkConfigDefault',
  planning_time: 1.0,
  attempts: 1,
  ik_timeout: 0.6
};
const BENCHMARK_SOURCE_LABELS = {
  box_top: '箱上',
  edge: '边缘',
  random: '随机',
  custom: '自定义'
};

const CAMERA_CALIBRATION_FRAMES = 5;

function buildCameraCalibrationPoints(config = {}) {
  const plateX = Number(config.plate_x ?? -0.08);
  const plateY = Number(config.plate_y ?? -0.72);
  const blockX = Number(config.calibration_block_x ?? 0.13);
  const blockY = Number(config.calibration_block_y ?? -0.62);
  const make = (surface, label, spawnZ, validation, coordinates) => coordinates.map(([x, y], index) => ({
    id: `${surface}-${index + 1}`,
    surface,
    surfaceLabel: label,
    x,
    y,
    spawnZ,
    validation: validation.includes(index),
    sample: null
  }));
  return [
    ...make('table', '桌面', 0.875, [1, 6], [
      [-0.16, -0.84], [0.00, -0.84], [0.16, -0.84],
      [0.16, -0.78], [0.16, -0.72], [0.16, -0.69],
      [-0.16, -0.60], [0.02, -0.60], [0.05, -0.68]
    ]),
    ...make('plate', '盘内', 0.89, [4], [
      [plateX, plateY], [plateX - 0.025, plateY], [plateX + 0.025, plateY],
      [plateX, plateY - 0.025], [plateX, plateY + 0.025]
    ]),
    ...make('block', '标定块', 0.925, [1, 4], [
      [blockX, blockY], [blockX - 0.02, blockY], [blockX + 0.02, blockY],
      [blockX, blockY - 0.015], [blockX, blockY + 0.015]
    ])
  ];
}

const state = {
  arm: localStorage.getItem('unoarm.activeArm') === 'left' ? 'left' : 'right',
  activePanel: 'motion',
  linksByArm: { right: [], left: [] },
  jointState: null,
  jointConfig: null,
  jointLimitsByArm: { right: {}, left: {} },
  kinematics: null,
  ompl: {
    options: null,
    config: loadSavedOmplConfig(),
    selectedPresetId: null
  },
  benchmark: {
    options: null,
    defaultsApplied: false,
    selectedPlanners: [],
    samples: [],
    selectedSampleId: null,
    result: null,
    progress: null
  },
  visionTarget: null,
  cameraExtrinsic: null,
  cameraCalibration: {
    points: buildCameraCalibrationPoints(),
    analysis: null,
    fit: null,
    sceneReady: false,
    running: false
  },
  points: [],
  presets: [],
  posesLib: [],
  sequence: loadSavedSequence(),
  cloudVisible: localStorage.getItem('unoarm.cloudVisible') !== 'false',
  reachabilityVisible: localStorage.getItem('unoarm.reachabilityVisible') !== 'false',
  graspPoseMode: normalizeGraspPoseMode(localStorage.getItem('unoarm.graspPoseMode')),
  platformObstacleVisible: false,
  platformObstacleApplied: false,
  manualCollision: {
    boxes: [],
    applied: false
  },
  workspaceBoundsByArm: {
    right: {
      enabled: false,
      min: [-0.8, -0.65, 0.2],
      max: [0.35, 0.35, 1.55]
    },
    left: {
      enabled: false,
      min: [-0.35, -0.65, 0.2],
      max: [0.8, 0.35, 1.55]
    }
  },
  lastPlanByArm: { right: null, left: null },
  lastDecisionError: null,
  robotByArm: {
    right: { motors_enabled: null, estop_active: null },
    left: { motors_enabled: null, estop_active: null }
  },
  gripperByArm: {
    right: { state: 'open', arm: 'right', joint: 'Right_Gripper_Joint', position: GRIPPER_OPEN_POSITION, simulated: false },
    left: { state: 'open', arm: 'left', joint: 'Left_Gripper_Joint', position: GRIPPER_OPEN_POSITION, simulated: false }
  },
  gripperCommand: {
    right: { target: null, at: 0, inFlight: false },
    left: { target: null, at: 0, inFlight: false }
  },
  lastStatusAt: 0
};

let scene;
const heldJogKeys = new Set();
let jogInFlight = false;
let jogRepeatTimer = null;
let lastJogErrorAt = 0;
let benchmarkProgressTimer = null;
let scenePresets = {};

export function startApp() {
  buildPoseFields();
  buildCameraFields();
  bindRange('velocityScaling', 'velocityValue', value => Number(value).toFixed(2));
  bindRange('accelerationScaling', 'accelerationValue', value => Number(value).toFixed(2));
  bindNavigation();
  bindCommands();
  bindKeyboardControls();
  updateArmUi();
  updateGraspPoseModeUi();
  updateRobotCommandButtons({ anyConnected: false, anyEnabled: false, allEnabled: false, anyEstop: false });
  updateCloudButton();
  updateReachabilityVisibilityUi();
  updatePlatformButtons();
  updateManualCollisionButtons();
  renderDecisionPreview();
  renderSequence();
  renderBenchmarkQuickStats();
  renderBenchmarkSummary();
  renderBenchmarkSamples();
  renderBenchmarkProgress();
  renderCameraCalibration();
  refreshIcons();
  startScene();
  startClock();
  initialLoad();
  startPolling();
}

function startScene() {
  scene = new RobotScene(byId('robotScene'), {
    onLoadProgress: (loaded, total) => {
      byId('sceneLoadState').querySelector('span:last-child').textContent =
        `正在加载 UnoArm URDF ${loaded}/${total}`;
    },
    onLoaded: robot => {
      byId('sceneLoadState').classList.add('ready');
      byId('sceneLoadState').querySelector('span:last-child').textContent = '真实 URDF 已加载';
      byId('sceneModelState').textContent = `${Object.keys(robot.joints || {}).length} 个关节`;
      scene.applyJointState(state.jointState);
      applyCachedGrippersToScene();
      window.setTimeout(() => {
        byId('sceneLoadState').hidden = true;
      }, 1000);
    },
    onLoadError: error => {
      byId('sceneLoadState').querySelector('span:last-child').textContent = 'URDF 加载失败';
      byId('sceneModelState').textContent = '模型不可用';
      showToast(`URDF 加载失败：${error.message || error}`, 'bad', 6000);
    },
    onFps: fps => {
      byId('sceneFps').textContent = `${fps} FPS`;
    }
  });
  scene.setCloudVisible(state.cloudVisible);
  scene.setTargetPose(readPose());
}

async function initialLoad() {
  await Promise.allSettled([
    refreshStatus(),
    refreshRobotState(),
    refreshLinks(),
    refreshGripper(),
    loadJointLimits(),
    loadCameraExtrinsic(),
    refreshVision(),
    refreshMujocoCamera(),
    refreshWristCameras(),
    refreshMujocoScene(),
    loadPointLibrary(),
    loadPresetLibrary(),
    loadPlatformObstacle(false),
    loadManualCollisionBoxes(false),
    loadWorkspaceBounds(false),
    loadKinematics(),
    loadOmplConfig(),
    loadBenchmarkOptions(false)
  ]);
  loadSceneCloud();
}

function startPolling() {
  pollEvery(refreshStatus, 50);
  pollEvery(refreshLinks, 400);
  pollEvery(refreshRobotState, 600);
  pollEvery(refreshVision, 2000);
  pollEvery(refreshMujocoCamera, 1000);
  pollEvery(refreshWristCameras, 1000);
  window.setInterval(() => {
    if (state.activePanel === 'logs' && byId('logAutoRefresh').checked) loadLogs();
  }, 3000);
}

function pollEvery(fn, intervalMs) {
  let pending = false;
  window.setInterval(async () => {
    if (pending) return;
    pending = true;
    try {
      await fn();
    } finally {
      pending = false;
    }
  }, intervalMs);
}

function startClock() {
  const update = () => {
    byId('footerClock').textContent = new Date().toLocaleTimeString('zh-CN', { hour12: false });
  };
  update();
  window.setInterval(update, 1000);
}

function buildPoseFields() {
  const root = byId('poseFields');
  root.innerHTML = POSE_FIELDS.map(([id, label, min, max, step, value]) => `
    <label class="pose-field pose-slider-field">
      <span>${label} / m</span>
      <output id="pose_${id}_value">${formatNumber(value, 3)}</output>
      <input id="pose_${id}" type="range" min="${min}" max="${max}" step="${step}" value="${value}">
    </label>
  `).join('');
  for (const [id] of POSE_FIELDS) {
    byId(`pose_${id}`).addEventListener('input', () => {
      updatePoseOutput(id);
      scene?.setTargetPose(readPose());
    });
  }
}

function updatePoseOutput(id) {
  const output = byId(`pose_${id}_value`);
  if (output) output.textContent = formatNumber(Number(byId(`pose_${id}`).value), 3);
}

function buildCameraFields() {
  const root = byId('cameraFields');
  root.innerHTML = CAMERA_FIELDS.map(([id, label, min, max, step, value]) => `
    <label class="pose-field">
      <span>${label}</span>
      <input id="camera_${id}" type="number" min="${min}" max="${max}" step="${step}" value="${value}">
    </label>
  `).join('');
  for (const [id] of CAMERA_FIELDS) {
    byId(`camera_${id}`).addEventListener('input', () => {
      state.cameraExtrinsic = readCameraExtrinsic();
      scene?.setCameraExtrinsic(state.cameraExtrinsic);
      byId('extrinsicStatus').textContent = '外参已修改，尚未保存';
    });
  }
}

function bindRange(inputId, outputId, formatter) {
  const input = byId(inputId);
  const output = byId(outputId);
  if (!input || !output) return;
  const update = () => {
    output.textContent = formatter(input.value);
  };
  input.addEventListener('input', update);
  update();
}

function bindNavigation() {
  document.querySelectorAll('[data-panel]').forEach(button => {
    button.addEventListener('click', () => switchPanel(button.dataset.panel));
  });
  byId('toggleInspector').addEventListener('click', () => {
    byId('inspectorPanel').classList.toggle('open');
  });
  document.querySelectorAll('[data-arm]').forEach(button => {
    button.addEventListener('click', () => setArm(button.dataset.arm));
  });
  document.querySelectorAll('[data-grasp-mode]').forEach(button => {
    button.addEventListener('click', () => setGraspPoseMode(button.dataset.graspMode));
  });
  byId('fitViewButton').addEventListener('click', () => scene.fitView());
  byId('frontViewButton').addEventListener('click', () => scene.frontView());
  byId('resetViewButton').addEventListener('click', () => scene.resetView());
  document.querySelector('.scene-panel').addEventListener('click', event => {
    if (event.target.closest('button')) return;
    if (window.innerWidth <= 760) byId('controlPanel').classList.remove('open');
    if (window.innerWidth <= 1040) byId('inspectorPanel').classList.remove('open');
  });
}

function switchPanel(name) {
  state.activePanel = name;
  document.querySelectorAll('[data-panel]').forEach(button => {
    button.classList.toggle('active', button.dataset.panel === name);
  });
  document.querySelectorAll('[data-panel-content]').forEach(panel => {
    panel.classList.toggle('active', panel.dataset.panelContent === name);
  });
  if (window.innerWidth <= 760) byId('controlPanel').classList.add('open');
  if (name === 'logs') loadLogs();
  if (name === 'ompl') loadOmplConfig();
  if (name === 'benchmark') loadBenchmarkOptions();
  if (name === 'vlm') loadVlmConfig();
  if (name === 'poses') loadPosesLib();
  if (name === 'camera-calibration') {
    refreshCameraCalibrationImage();
    loadCameraCalibrationScene();
  }
  if (name === 'calibration') {
    loadJointConfig();
    loadKinematics();
  }
}

async function setArm(arm) {
  clearHeldJogKeys();
  state.arm = arm === 'left' ? 'left' : 'right';
  localStorage.setItem('unoarm.activeArm', state.arm);
  updateArmUi();
  await Promise.allSettled([refreshStatus(), refreshRobotState(), refreshLinks(), refreshGripper()]);
  if (state.activePanel === 'calibration') loadJointConfig();
  byId('footerMessage').textContent = `当前控制：${armLabel()}`;
}

function updateArmUi() {
  document.querySelectorAll('[data-arm]').forEach(button => {
    const active = button.dataset.arm === state.arm;
    button.classList.toggle('active', active);
    button.setAttribute('aria-selected', String(active));
  });
  byId('jointPanelTitle').textContent = `${armLabel()}关节`;
  renderWorkspaceBounds();
  scene?.setWorkspaceBounds(currentWorkspaceBounds());
  renderJointTelemetry();
  updateTcp();
  renderGripperStatus();
  applyCachedGrippersToScene();
  renderBenchmarkQuickStats();
  renderBenchmarkSamples();
  if (state.activePanel === 'benchmark') loadBenchmarkOptions(false);
}

function normalizeGraspPoseMode(mode) {
  return mode === 'vertical' || mode === 'side' || mode === 'z_parallel' ? mode : DEFAULT_GRASP_POSE_MODE;
}

function setGraspPoseMode(mode) {
  state.graspPoseMode = normalizeGraspPoseMode(mode);
  localStorage.setItem('unoarm.graspPoseMode', state.graspPoseMode);
  updateGraspPoseModeUi();
  renderDecisionPreview();
  scene?.setTargetPose(readPose());
  syncGraspReachabilityOverlay();
}

function syncGraspReachabilityOverlay() {
  const visible = state.reachabilityVisible && (state.arm === 'right' || state.arm === 'left');
  const mode = currentReachabilityMode();
  renderReachabilityOverlayState({ mode, arm: state.arm, loading: visible, visible });
  const request = scene?.setGraspReachabilityMode(state.graspPoseMode, { visible, arm: state.arm });
  if (request && typeof request.then === 'function') {
    request
      .then(entry => {
        if (!entry) {
          renderReachabilityOverlayState({ mode, arm: state.arm, visible: false });
          return;
        }
        if (currentReachabilityMode() === entry.data?.mode && state.arm === (entry.data?.arm || state.arm)) {
          renderReachabilityOverlayState({ mode: entry.data.mode, arm: entry.data.arm || state.arm, data: entry.data });
        }
      })
      .catch(error => {
        renderReachabilityOverlayState({ mode, arm: state.arm, error });
      });
  }
}

function updateReachabilityVisibilityUi() {
  const toggle = byId('reachabilityOverlayToggle');
  if (toggle) toggle.checked = state.reachabilityVisible;
  const label = byId('reachabilityOverlayToggleLabel');
  if (label) label.textContent = state.reachabilityVisible ? '显示' : '隐藏';
}

function toggleReachabilityVisibility(event) {
  state.reachabilityVisible = Boolean(event.currentTarget.checked);
  localStorage.setItem('unoarm.reachabilityVisible', String(state.reachabilityVisible));
  updateReachabilityVisibilityUi();
  syncGraspReachabilityOverlay();
}

function updateGraspPoseModeUi() {
  document.querySelectorAll('[data-grasp-mode]').forEach(button => {
    const active = button.dataset.graspMode === state.graspPoseMode;
    button.classList.toggle('active', active);
    button.setAttribute('aria-selected', String(active));
  });
}

function bindCommands() {
  const on = (id, eventName, handler) => byId(id)?.addEventListener(eventName, handler);
  on('syncTcpButton', 'click', syncTargetToTcp);
  on('planButton', 'click', planMotion);
  on('executeButton', 'click', executePlan);
  on('readVisionButton', 'click', () => readVisionTarget(true));
  on('pickMujocoObjectButton', 'click', pickMujocoObject);
  on('pickPlacePlateButton', 'click', pickPlacePlate);
  on('leftDropCylinderButton', 'click', leftSideDropRotorToCylinder);
  on('pickPlaceTableButton', 'click', pickPlaceTable);
  on('savePointQuickButton', 'click', savePoint);
  on('savePointButton', 'click', savePoint);
  on('savePresetButton', 'click', savePreset);
  on('emergencyStop', 'click', emergencyStop);
  on('enableButton', 'click', enableRobot);
  on('disableButton', 'click', disableRobot);
  on('resetEstopButton', 'click', resetEstop);
  on('homeButton', 'click', homeRobot);
  on('taskResetButton', 'click', resetTask);
  on('gripperOpenButton', 'click', () => runArmAction('gripper_open'));
  on('gripperCloseButton', 'click', () => runArmAction('gripper_close'));
  on('reloadExtrinsicButton', 'click', loadCameraExtrinsic);
  on('saveExtrinsicButton', 'click', saveCameraExtrinsic);
  on('reloadVlmButton', 'click', loadVlmConfig);
  on('saveVlmButton', 'click', saveVlmConfigAction);
  on('vlmConfirmButton', 'click', vlmConfirmAction);
  on("moveObjectButton", "click", moveObjectAction);
  on("rebuildSceneButton", "click", rebuildSceneAction);
  on("scenePreset", "change", applySelectedScenePreset);
  on("showPlate", "change", updatePlatePositionVisibility);
  on('reloadPosesButton', 'click', loadPosesLib);
  on('recordPoseButton', 'click', recordCurrentPose);
  on('addPoseButton', 'click', addPose);
  on('toggleCloudButton', 'click', toggleCloud);
  on('saveCloudButton', 'click', saveSceneCloud);
  on('rebuildCloudButton', 'click', rebuildCloudWithCurrentExtrinsic);
  on('loadPlatformButton', 'click', togglePlatformObstacle);
  on('applyPlatformButton', 'click', applyPlatformObstacle);
  on('addCollisionBoxButton', 'click', addManualCollisionBox);
  on('saveCollisionBoxesButton', 'click', saveManualCollisionConfig);
  on('applyCollisionBoxesButton', 'click', applyManualCollisionConfig);
  on('clearCollisionBoxesButton', 'click', clearManualCollisionConfig);
  on('manualCollisionFields', 'input', handleManualCollisionInput);
  on('manualCollisionFields', 'change', handleManualCollisionInput);
  on('manualCollisionFields', 'click', handleManualCollisionClick);
  on('workspaceBoundsFields', 'input', handleWorkspaceBoundsInput);
  on('workspaceBoundsFields', 'change', handleWorkspaceBoundsInput);
  on('saveWorkspaceBoundsButton', 'click', saveWorkspaceBoundsConfig);
  on('loadJointConfigButton', 'click', loadJointConfig);
  on('saveJointConfigButton', 'click', saveJointConfig);
  on('reloadKinematicsButton', 'click', loadKinematics);
  on('saveKinematicsButton', 'click', saveKinematics);
  on('reloadOmplConfigButton', 'click', loadOmplConfig);
  on('applyOmplPresetButton', 'click', applySelectedOmplPreset);
  on('saveOmplPresetButton', 'click', saveCurrentOmplPreset);
  on('deleteOmplPresetButton', 'click', deleteSelectedOmplPreset);
  on('omplPlannerSelect', 'change', handleOmplConfigInput);
  on('omplPlanningTime', 'input', handleOmplConfigInput);
  on('omplAttempts', 'input', handleOmplConfigInput);
  on('omplIkTimeout', 'input', handleOmplConfigInput);
  on('generateBenchmarkButton', 'click', generateBenchmarkSet);
  on('runBenchmarkButton', 'click', runBenchmarkSet);
  on('saveBenchmarkPresetButton', 'click', saveBenchmarkRuntimePreset);
  on('exportBenchmarkCsvButton', 'click', exportCurrentBenchmarkCsv);
  on('benchmarkSamples', 'click', handleBenchmarkSampleClick);
  on('benchmarkPlanners', 'change', handleBenchmarkPlannerChange);
  on('reachabilityOverlayToggle', 'change', toggleReachabilityVisibility);
  on('cameraCalibrationPlateToggle', 'change', updateCameraCalibrationFixtures);
  on('cameraCalibrationBlockToggle', 'change', updateCameraCalibrationFixtures);
  on('runCameraCalibrationButton', 'click', runCameraCalibration);
  on('clearCameraCalibrationButton', 'click', clearCameraCalibration);
  on('applyCameraTranslationButton', 'click', applyCameraTranslationCorrection);
  on('cameraCalibrationPoints', 'click', handleCameraCalibrationPointClick);
  on('cameraCalibrationPoints', 'change', handleCameraCalibrationPointChange);
  [
    'benchmarkCollisionBox',
    'benchmarkBoxCount',
    'benchmarkBoxOffsetCm',
    'benchmarkEdgeCount',
    'benchmarkEdgeDistanceCm',
    'benchmarkRandomCount',
    'benchmarkSeed'
  ].forEach(id => on(id, id === 'benchmarkCollisionBox' ? 'change' : 'input', handleBenchmarkSampleConfigInput));
  ['benchmarkPlanningTime', 'benchmarkAttempts', 'benchmarkIkTimeout'].forEach(id => {
    on(id, 'input', handleBenchmarkRuntimeInput);
  });
  on('addSequenceButton', 'click', addSequenceStep);
  on('clearSequenceButton', 'click', clearSequence);
  on('executeSequenceButton', 'click', executeSequence);
  on('refreshLogsButton', 'click', loadLogs);
  on('logLevelFilter', 'change', loadLogs);
  on('pointsList', 'click', handleLibraryClick);
  on('presetsList', 'click', handleLibraryClick);
  on('posesList', 'click', handlePosesClick);
  on('sequenceList', 'click', handleSequenceClick);
}

function bindKeyboardControls() {
  window.addEventListener('keydown', event => {
    if (isEditableTarget(event.target) || !byId('confirmDialog').hidden) return;
    const key = event.key.toLowerCase();

    if (key === 'tab' || key === '[' || key === ']') {
      event.preventDefault();
      if (event.repeat) return;
      const arm = key === '[' ? 'left' : key === ']' ? 'right' : state.arm === 'left' ? 'right' : 'left';
      setArm(arm);
      return;
    }

    const mapping = jointMappingForKey(key);
    if (!mapping) return;
    event.preventDefault();
    if (!canJogActiveArm()) {
      if (!event.repeat) showToast(`${armLabel()}尚未使能，不能使用关节键盘控制`, 'warn');
      return;
    }

    heldJogKeys.add(key);
    setJogKeyPressed(key, true);
    if (!event.repeat) requestJointJog(mapping.joint, mapping.direction);
    startJogRepeat();
  });

  window.addEventListener('keyup', event => {
    const key = event.key.toLowerCase();
    if (!jointMappingForKey(key)) return;
    heldJogKeys.delete(key);
    setJogKeyPressed(key, false);
    if (!heldJogKeys.size) stopJogRepeat();
  });
  window.addEventListener('blur', clearHeldJogKeys);

  byId('jointTelemetry').addEventListener('click', event => {
    const button = event.target.closest('[data-jog-joint]');
    if (!button || button.disabled) return;
    requestJointJog(Number(button.dataset.jogJoint), Number(button.dataset.jogDirection));
  });
}

function isEditableTarget(target) {
  return target instanceof HTMLElement &&
    (target.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName));
}

function jointMappingForKey(key) {
  for (let index = 0; index < JOINT_KEY_PAIRS.length; index += 1) {
    const [positive, negative] = JOINT_KEY_PAIRS[index];
    if (key === positive) return { joint: index + 1, direction: 1 };
    if (key === negative) return { joint: index + 1, direction: -1 };
  }
  return null;
}

function canJogActiveArm() {
  const robot = state.robotByArm[state.arm];
  return robot?.motors_enabled === true && robot?.estop_active !== true;
}

function setJogKeyPressed(key, pressed) {
  document.querySelectorAll(`[data-jog-key="${key}"]`).forEach(button => {
    button.classList.toggle('pressed', pressed);
  });
}

function startJogRepeat() {
  if (jogRepeatTimer) return;
  jogRepeatTimer = window.setInterval(() => {
    const key = heldJogKeys.values().next().value;
    const mapping = jointMappingForKey(key);
    if (mapping) requestJointJog(mapping.joint, mapping.direction);
  }, JOINT_JOG_REPEAT_MS);
}

function stopJogRepeat() {
  if (!jogRepeatTimer) return;
  window.clearInterval(jogRepeatTimer);
  jogRepeatTimer = null;
}

function clearHeldJogKeys() {
  heldJogKeys.clear();
  stopJogRepeat();
  document.querySelectorAll('[data-jog-key].pressed').forEach(button => button.classList.remove('pressed'));
}

async function requestJointJog(joint, direction) {
  if (jogInFlight || !canJogActiveArm()) return;
  jogInFlight = true;
  try {
    const result = await apiRequest('/api/joint_jog', {
      arm: state.arm,
      joint,
      delta_rad: direction * JOINT_JOG_STEP_RAD,
      duration_sec: 0.25
    });
    const degrees = Number(result.target_rad) * 180 / Math.PI;
    byId('footerMessage').textContent =
      `${armLabel()} J${joint} → ${Number.isFinite(degrees) ? `${degrees.toFixed(1)}°` : '已发送'}`;
  } catch (error) {
    clearHeldJogKeys();
    if (Date.now() - lastJogErrorAt > 1500) {
      showToast(`关节键盘控制失败：${error.message}`, 'bad', 5000);
      lastJogErrorAt = Date.now();
    }
  } finally {
    jogInFlight = false;
  }
}

function readPose() {
  const pose = Object.fromEntries(POSE_FIELDS.map(([id]) => [id, Number(byId(`pose_${id}`).value)]));
  const mode = normalizeGraspPoseMode(state.graspPoseMode);
  const orientation = GRASP_POSE_DEFAULTS[mode];
  const request = {
    ...pose,
    roll: orientation.roll,
    pitch: orientation.pitch,
    yaw: orientation.yaw,
    arm: state.arm,
    velocity_scaling: Number(byId('velocityScaling').value),
    acceleration_scaling: Number(byId('accelerationScaling').value),
    orientation_tolerance: 0.5,
    stay_near: false,
    avoid_platform: byId('avoidPlatform').checked,
    collision_boxes: currentManualCollisionBoxes(),
    grasp_orientation_mode: orientation.grasp_orientation_mode,
    ...readOmplRuntimeConfig()
  };
  return request;
}

function readGraspPose() {
  return readPose();
}

function numericValue(id, fallback) {
  const value = Number(byId(id)?.value);
  return Number.isFinite(value) ? value : fallback;
}

function vector3(values, label) {
  const vector = Array.isArray(values)
    ? values.slice(0, 3).map(value => Number(value))
    : ['x', 'y', 'z'].map(axis => Number(values?.[axis]));
  if (vector.length !== 3 || !vector.every(Number.isFinite)) {
    throw new Error(`${label}坐标无效`);
  }
  return vector;
}

function findCylinderDropTarget(sceneData) {
  const extras = Array.isArray(sceneData?.extras) ? sceneData.extras : [];
  const target = extras.find(item => item?.type === 'cylinder' || item?.name === 'cylinder')
    || extras.find(item => item?.type === 'slot' || item?.name === 'plate');
  if (!target) {
    throw new Error('场景中没有圆柱/半圆槽目标');
  }
  const center = vector3(target.position, `${target.name || target.type || '目标'}`);
  return {
    ...target,
    center,
    label: target.type === 'slot' || target.name === 'plate' ? '半圆槽' : '圆柱'
  };
}

function setPose(pose) {
  for (const [id] of POSE_FIELDS) {
    if (Number.isFinite(Number(pose?.[id]))) {
      byId(`pose_${id}`).value = pose[id];
      updatePoseOutput(id);
    }
  }
  scene?.setTargetPose(readPose());
}

function readCameraExtrinsic() {
  return Object.fromEntries(CAMERA_FIELDS.map(([id]) => [id, Number(byId(`camera_${id}`).value)]));
}

function setCameraExtrinsic(extrinsic) {
  for (const [id] of CAMERA_FIELDS) {
    if (Number.isFinite(Number(extrinsic?.[id]))) byId(`camera_${id}`).value = extrinsic[id];
  }
  state.cameraExtrinsic = readCameraExtrinsic();
  scene?.setCameraExtrinsic(state.cameraExtrinsic);
}

async function refreshStatus() {
  try {
    const data = await getStatus();
    state.lastStatusAt = Date.now();
    state.jointState = data.joint_state || null;
    state.lastPlanByArm = { ...state.lastPlanByArm, ...(data.last_plan || {}) };
    scene?.applyJointState(state.jointState);
    if (data.mujoco_scene) {
      scene?.setMujocoScene(data.mujoco_scene);
    }
    if (data.gripper) {
      for (const arm of ['right', 'left']) {
        if (data.gripper[arm]) {
          applyGripperState(arm, data.gripper[arm], { updateBadge: arm === state.arm, sceneFallback: false });
        }
      }
    }
    renderJointTelemetry();
    renderDecisionPreview();

    const plan = data.last_plan?.[state.arm];
    const planState = data.status?.[state.arm] || '就绪';
    const plannerText = plan?.ompl?.planner_id ? ` / ${plan.ompl.planner_id}` : '';
    setStatus(
      'statusPlan',
      '规划',
      plan ? `${plan.points ?? '--'} 点 / ${formatNumber(plan.duration, 2)} s${plannerText}` : planState,
      plan ? 'good' : 'neutral'
    );
    setStatus('statusConnection', '控制服务', '在线', 'good');
    if (!document.body.dataset.busy && byId('footerMessage').textContent.startsWith('等待')) {
      byId('footerMessage').textContent = 'R007 控制服务在线';
    }
  } catch {
    setStatus('statusConnection', '控制服务', '离线', 'bad');
    setStatus('statusPlan', '规划', '状态不可用', 'bad');
    if (!document.body.dataset.busy) byId('footerMessage').textContent = 'R007 控制服务离线';
  }
}

async function refreshRobotState() {
  try {
    const data = await getRobotState();
    let anyConnected = false;
    let anyEstop = false;
    let anyEnabled = false;
    let allEnabled = true;
    let allKnown = true;
    for (const arm of ['right', 'left']) {
      const armState = data.state?.[arm] || {};
      state.robotByArm[arm] = {
        motors_enabled: armState.motors_enabled,
        estop_active: armState.estop_active
      };
      const row = byId(`${arm}ArmState`);
      const enabled = armState.motors_enabled;
      const estop = armState.estop_active;
      anyConnected ||= enabled !== null && enabled !== undefined;
      anyEnabled ||= enabled === true;
      allEnabled &&= enabled === true;
      allKnown &&= enabled !== null && enabled !== undefined;
      anyEstop ||= estop === true;
      row.children[1].textContent = enabled == null ? '未连接' : enabled ? '已使能' : '未使能';
      row.children[2].textContent = estop == null ? '--' : estop ? '急停' : '正常';
      row.dataset.state = estop ? 'bad' : enabled ? 'good' : 'neutral';
    }
    allEnabled &&= allKnown;
    setBadge(
      'robotOverallState',
      anyEstop ? '急停' : anyConnected ? '在线' : '未连接',
      anyEstop ? 'bad' : anyConnected ? 'good' : 'bad'
    );
    updateRobotCommandButtons({ anyConnected, anyEnabled, allEnabled, anyEstop });
    renderJointTelemetry();
  } catch {
    state.robotByArm.right = { motors_enabled: null, estop_active: null };
    state.robotByArm.left = { motors_enabled: null, estop_active: null };
    setBadge('robotOverallState', '离线', 'bad');
    updateRobotCommandButtons({ anyConnected: false, anyEnabled: false, allEnabled: false, anyEstop: false });
    renderJointTelemetry();
  }
}

async function refreshLinks() {
  try {
    const [right, left] = await Promise.all([getLinks('right'), getLinks('left')]);
    state.linksByArm.right = right.links || [];
    state.linksByArm.left = left.links || [];
    updateTcp();
  } catch {
    setStatus('statusTcp', '末端位置', '读取失败', 'bad');
  }
}

function currentTcp() {
  const tcpName = state.arm === 'left' ? 'Left_Gripper_TCP' : 'Right_Gripper_TCP';
  return state.linksByArm[state.arm]?.find(link => link.name === tcpName && link.position);
}

function updateTcp() {
  const tcp = currentTcp();
  const xyz = tcp?.position;
  byId('tcpX').textContent = formatNumber(xyz?.[0]);
  byId('tcpY').textContent = formatNumber(xyz?.[1]);
  byId('tcpZ').textContent = formatNumber(xyz?.[2]);
  scene?.setTcpFrame(tcp);
  setStatus('statusTcp', '末端位置', xyz ? `${formatNumber(xyz[0], 2)}, ${formatNumber(xyz[1], 2)}, ${formatNumber(xyz[2], 2)}` : '等待数据', xyz ? 'good' : 'neutral');
}

function syncTargetToTcp() {
  const tcp = currentTcp();
  if (!tcp) {
    showToast('当前没有可用的末端位置数据', 'warn');
    return;
  }
  setPose({ ...readPose(), x: tcp.position[0], y: tcp.position[1], z: tcp.position[2] });
  showToast(`已同步 ${armLabel()}末端位置`);
}

function renderJointTelemetry() {
  const root = byId('jointTelemetry');
  const jointState = state.jointState;
  const prefix = state.arm === 'left' ? 'Left_Joint' : 'Right_Joint';
  const jogEnabled = canJogActiveArm();
  const rows = [];
  for (let index = 1; index <= 7; index += 1) {
    const stateIndex = jointState?.name?.indexOf(`${prefix}${index}`) ?? -1;
    const radians = stateIndex >= 0 ? Number(jointState.position[stateIndex]) : NaN;
    const degrees = radians * 180 / Math.PI;
    const width = Number.isFinite(degrees) ? Math.min(50, Math.abs(degrees) / 3.6) : 0;
    const transform = degrees < 0 ? 'translateX(-100%)' : 'none';
    const [positiveKey, negativeKey] = JOINT_KEY_PAIRS[index - 1];
    rows.push(`
      <div class="joint-telemetry-row">
        <b>J${index}</b>
        <div class="joint-meter"><span style="width:${width}%;transform:${transform}"></span></div>
        <output>${Number.isFinite(degrees) ? `${degrees.toFixed(1)}°` : '--'}</output>
        <div class="joint-jog-keys">
          <button type="button" data-jog-joint="${index}" data-jog-direction="1" data-jog-key="${positiveKey}"
            title="关节 ${index} 正向微动" ${jogEnabled ? '' : 'disabled'}>${positiveKey.toUpperCase()}</button>
          <button type="button" data-jog-joint="${index}" data-jog-direction="-1" data-jog-key="${negativeKey}"
            title="关节 ${index} 反向微动" ${jogEnabled ? '' : 'disabled'}>${negativeKey.toUpperCase()}</button>
        </div>
      </div>
    `);
  }
  root.innerHTML = rows.join('');
  heldJogKeys.forEach(key => setJogKeyPressed(key, true));
  byId('jointStamp').textContent = jointState ? '实时' : '--';
}

async function refreshGripper() {
  try {
    const data = await getGripper(state.arm);
    applyGripperState(state.arm, data, { sceneFallback: false });
  } catch {
    setBadge('gripperState', '未知', 'bad');
  }
}

function gripperJointName(arm) {
  return arm === 'left' ? 'Left_Gripper_Joint' : 'Right_Gripper_Joint';
}

function gripperPosition(data) {
  const position = Number(data?.position);
  if (Number.isFinite(position)) return position;
  return data?.state === 'closed' ? GRIPPER_CLOSED_POSITION : GRIPPER_OPEN_POSITION;
}

function gripperTargetPosition(data) {
  const position = Number(data?.target_position);
  if (Number.isFinite(position)) return position;
  const targetState = data?.target_state === 'closed' ? 'closed' : data?.state;
  return targetState === 'closed' ? GRIPPER_CLOSED_POSITION : GRIPPER_OPEN_POSITION;
}

function normalizeGripperState(arm, data = {}) {
  const normalizedArm = arm === 'left' ? 'left' : 'right';
  const stateName = data.state === 'closed' ? 'closed' : 'open';
  const targetState = data.target_state === 'closed' ? 'closed' : data.target_state === 'open' ? 'open' : stateName;
  const progress = Number(data.progress);
  return {
    ...data,
    arm: normalizedArm,
    state: stateName,
    target_state: targetState,
    joint: data.joint || gripperJointName(normalizedArm),
    position: gripperPosition(data),
    target_position: gripperTargetPosition(data),
    progress: Number.isFinite(progress) ? Math.max(0, Math.min(1, progress)) : undefined,
    moving: data.moving === true,
    simulated: data.simulated === true
  };
}

function applyGripperState(arm, data, options = {}) {
  const normalized = normalizeGripperState(arm, data);
  const command = state.gripperCommand[normalized.arm];
  if (
    command?.target
    && normalized.state !== command.target
    && Date.now() - command.at < GRIPPER_STATUS_HOLD_MS
  ) {
    normalized.state = command.target;
    normalized.target_state = command.target;
    normalized.target_position = command.target === 'closed' ? GRIPPER_CLOSED_POSITION : GRIPPER_OPEN_POSITION;
    normalized.moving = true;
  }
  state.gripperByArm[normalized.arm] = normalized;
  if (options.forceScene === true || (options.sceneFallback !== false && !jointStateHasGripper(normalized.arm))) {
    scene?.applyJointState(jointStateFromGripper(normalized.arm, normalized));
  }
  if (options.updateBadge !== false && normalized.arm === state.arm) {
    renderGripperStatus();
  }
}

function applyCachedGrippersToScene() {
  for (const arm of ['right', 'left']) {
    applyGripperState(arm, state.gripperByArm[arm], { updateBadge: arm === state.arm });
  }
}

function jointStateFromGripper(arm, data) {
  const mimic = GRIPPER_MIMIC_BY_ARM[arm === 'left' ? 'left' : 'right'];
  const base = gripperPosition(data);
  return {
    name: Object.keys(mimic),
    position: Object.values(mimic).map(multiplier => base * multiplier),
    velocity: Object.keys(mimic).map(() => 0)
  };
}

function jointStateHasGripper(arm) {
  const mainJoint = gripperJointName(arm);
  return Array.isArray(state.jointState?.name) && state.jointState.name.includes(mainJoint);
}

function renderGripperStatus() {
  const gripper = state.gripperByArm[state.arm];
  const targetState = gripper?.target_state || gripper?.state;
  const closed = targetState === 'closed';
  const moving = gripper?.moving === true;
  const progress = Number(gripper?.progress);
  const progressText = moving && Number.isFinite(progress) ? ` ${Math.round(progress * 100)}%` : '';
  const sim = gripper?.simulated === true ? ' · 仿真' : '';
  const text = moving ? `${closed ? '闭合中' : '张开中'}${progressText}` : `${closed ? '已闭合' : '已张开'}`;
  setBadge('gripperState', `${text}${sim}`, moving ? 'warn' : 'good');
}

async function refreshVision() {
  try {
    const [targetResult, statusResult] = await Promise.allSettled([getVisionTarget(), getVisionStatus()]);
    if (targetResult.status === 'fulfilled' && targetResult.value?.urdf_xyz_m) {
      state.visionTarget = targetResult.value;
      const xyz = state.visionTarget.urdf_xyz_m;
      scene?.setVisionTarget(state.visionTarget);
      byId('visionX').textContent = formatNumber(xyz[0]);
      byId('visionY').textContent = formatNumber(xyz[1]);
      byId('visionZ').textContent = formatNumber(xyz[2]);
      setStatus('statusVision', '视觉', `${formatNumber(xyz[0], 2)}, ${formatNumber(xyz[1], 2)}, ${formatNumber(xyz[2], 2)}`, 'good');
    } else {
      setStatus('statusVision', '视觉', '无目标', 'neutral');
    }
    if (statusResult.status === 'fulfilled') {
      const objects = statusResult.value.objects || statusResult.value.detections || [];
      scene?.setObjectMarkers(objects);
      setBadge('cloudState', '视觉在线', 'good');
    }
  } catch {
    setStatus('statusVision', '视觉', '离线', 'bad');
    setBadge('cloudState', '未连接', 'bad');
  }
}

async function refreshMujocoCamera() {
  await refreshCameraPanel({
    getStatus: getMujocoCameraStatus,
    imageId: 'simCameraImage',
    placeholderId: 'simCameraPlaceholder',
    stateId: 'simCameraState',
    infoId: 'simCameraInfo',
    snapshotPath: '/mujoco_camera/snapshot.jpg',
    streamPath: '/mujoco_camera/stream.mjpg',
    waitingText: '等待头顶 D455'
  });
}

async function refreshWristCameras() {
  await Promise.allSettled([
    refreshCameraPanel({
      getStatus: getRightWristCameraStatus,
      imageId: 'rightWristCameraImage',
      placeholderId: 'rightWristCameraPlaceholder',
      stateId: 'rightWristCameraState',
      infoId: 'rightWristCameraInfo',
      snapshotPath: '/right_wrist_camera/snapshot.jpg',
      streamPath: '/right_wrist_camera/stream.mjpg',
      waitingText: '等待右腕 RGB-D'
    }),
    refreshCameraPanel({
      getStatus: getLeftWristCameraStatus,
      imageId: 'leftWristCameraImage',
      placeholderId: 'leftWristCameraPlaceholder',
      stateId: 'leftWristCameraState',
      infoId: 'leftWristCameraInfo',
      snapshotPath: '/left_wrist_camera/snapshot.jpg',
      streamPath: '/left_wrist_camera/stream.mjpg',
      waitingText: '等待左腕 RGB-D'
    })
  ]);
}

async function refreshMujocoScene() {
  try {
    const data = await getMujocoScene();
    scene?.setMujocoScene(data);
    scenePresets = data.presets || scenePresets;
    if (data.config && !byId('scenePreset')?.dataset.initialized) {
      applySceneConfigToForm(data.config);
      byId('scenePreset').dataset.initialized = '1';
    }
  } catch {
    // The regular status indicators already report when the simulation is offline.
  }
}

function updatePlatePositionVisibility() {
  const visible = Boolean(byId('showPlate')?.checked);
  byId('platePositionFields').hidden = !visible;
}

function applySceneConfigToForm(config = {}) {
  const preset = byId('scenePreset');
  if (preset) preset.value = config.preset && preset.querySelector(`option[value="${config.preset}"]`) ? config.preset : 'custom';
  const objects = Array.isArray(config.objects) ? config.objects : [];
  byId('objX').value = config.object_x ?? 0;
  byId('objY').value = config.object_y ?? -0.73;
  byId('tableX').value = config.table_x ?? 0.40;
  byId('tableY').value = config.table_y ?? 0.30;
  byId('plateX').value = config.plate_x ?? -0.08;
  byId('plateY').value = config.plate_y ?? -0.72;
  byId('showPlate').checked = objects.includes('plate');
  byId('showCylinder').checked = objects.includes('cylinder');
  byId('scenePresetDescription').textContent = config.description || '自定义物体与桌面布局';
  updatePlatePositionVisibility();
}

function applySelectedScenePreset() {
  const id = byId('scenePreset').value;
  const config = scenePresets[id];
  if (config) applySceneConfigToForm({ ...config, preset: id });
  else byId('scenePresetDescription').textContent = '自定义物体与桌面布局';
}

async function refreshCameraPanel({
  getStatus, imageId, placeholderId, stateId, infoId, snapshotPath, streamPath, waitingText
}) {
  const image = byId(imageId);
  const placeholder = byId(placeholderId);
  try {
    const status = await getStatus();
    if (!status.ready) throw new Error(status.error || '等待摄像头图像');
    image.onload = () => {
      image.hidden = false;
      placeholder.hidden = true;
    };
    image.onerror = () => {
      image.hidden = true;
      placeholder.hidden = false;
    };
    if (streamPath) {
      if (image.dataset.streamPath !== streamPath) {
        image.dataset.streamPath = streamPath;
        image.src = streamPath;
      }
    } else {
      image.src = `${snapshotPath}?t=${Date.now()}`;
    }
    setBadge(stateId, '实时', 'good');
    byId(infoId).textContent =
      `${status.width} × ${status.height} · ${status.frame_id || status.topic}`;
    return status;
  } catch (error) {
    image.hidden = true;
    placeholder.hidden = false;
    setBadge(stateId, '未连接', 'bad');
    byId(infoId).textContent = error.message || waitingText;
    return null;
  }
}

async function readVisionTarget(applyToPose) {
  await runCommand('读取视觉目标', async () => {
    const target = await getVisionTarget();
    if (!target?.urdf_xyz_m) throw new Error(target?.error || '未找到有效视觉目标');
    state.visionTarget = target;
    scene.setVisionTarget(target);
    if (applyToPose) {
      const [x, y, z] = target.urdf_xyz_m;
      setPose({ ...readPose(), x, y, z });
    }
    return target;
  });
}

async function planMotion() {
  await runCommand(
    '运动规划',
    async () => {
      state.lastDecisionError = null;
      renderDecisionPreview({ pending: true });
      const result = await apiRequest('/api/plan', readPose());
      if (result.plan?.platform_obstacles || result.plan?.platform_obstacle) {
        scene.setPlatformObstacle(result.plan.platform_obstacles || result.plan.platform_obstacle);
      }
      updateManualCollisionFromSummary(result.plan?.manual_collision);
      if (result.plan) {
        state.lastPlanByArm[state.arm] = result.plan;
        renderDecisionPreview();
      }
      await refreshStatus();
      return result;
    },
    {
      onError: (error) => {
        const failure = error?.payload?.failure;
        state.lastDecisionError = failure || { title: error.message };
        renderDecisionPreview();
        const label = failure?.stage_label ? `${failure.stage_label}失败` : '规划失败';
        const detail = failure?.title || error.message;
        setStatus('statusPlan', label, detail, 'bad');
      }
    }
  );
}

function renderDecisionPreview(options = {}) {
  const pose = readPose();
  const plan = state.lastPlanByArm[state.arm];
  const orientation = plan?.grasp_orientation || null;
  const currentMode = pose.grasp_orientation_mode;
  const selected = Boolean(orientation && decisionModeMatches(orientation.mode, currentMode));
  const mode = selected ? orientation.mode : currentMode;
  const detail = DECISION_MODE_DETAILS[mode] || DECISION_MODE_DETAILS[pose.grasp_orientation_mode];
  const rejectedCount = Array.isArray(orientation?.rejected_candidates) ? orientation.rejected_candidates.length : 0;
  const candidateIndex = Number(orientation?.candidate_index);
  const activeIndex = selected && Number.isFinite(candidateIndex) ? Math.max(1, candidateIndex) : null;
  const totalScan = selected ? Math.max(activeIndex || 1, rejectedCount + 1) : 3;
  const steps = decisionCandidateSteps({
    detail,
    orientation,
    plan,
    pose,
    selected,
    pending: options.pending,
    activeIndex,
    totalScan
  });
  if (state.lastDecisionError) {
    steps.push({
      label: '失败',
      detail: state.lastDecisionError.stage_label || state.lastDecisionError.title || '未找到可行候选',
      state: 'failed'
    });
  }

  const html = `
    <div class="decision-title">
      <strong>${escapeHtml(detail.title)}</strong>
      <span>${escapeHtml(selected ? '已确定' : options.pending ? '规划中' : '预览')}</span>
    </div>
    <p>${escapeHtml(detail.summary)}</p>
    <div class="decision-chain">
      ${steps.map(step => `
        <div class="decision-step" data-state="${step.state}">
          <b>${escapeHtml(step.label)}</b>
          <span>${escapeHtml(step.detail)}</span>
        </div>
      `).join('')}
    </div>
    ${selected ? `
      <div class="decision-meta">
        <span>候选 ${activeIndex ?? '--'}</span>
        <span>拒绝 ${rejectedCount}</span>
        <span>${escapeHtml(plan?.ompl?.planner_id || 'IK')}</span>
      </div>
    ` : ''}
  `;
  for (const id of ['decisionPreview', 'activeDecisionPreview']) {
    const root = byId(id);
    if (root) root.innerHTML = html;
  }
  setBadge('decisionState', selected ? '已确定' : options.pending ? '规划中' : '预览', selected ? 'good' : options.pending ? 'warn' : 'neutral');
}

function decisionCandidateSteps({ detail, orientation, plan, pose, selected, pending, activeIndex, totalScan }) {
  const limit = Math.min(totalScan, 6);
  const steps = [];
  const isFallback = selected && String(plan?.mode || '').includes('direct IK fallback');
  for (let index = 1; index <= limit; index += 1) {
    const beforeSelected = selected && activeIndex && index < activeIndex;
    const isSelected = selected && activeIndex === index;
    const isPendingFirst = pending && index === 1;
    const ikState = isSelected ? 'done' : beforeSelected ? 'failed' : isPendingFirst ? 'active' : 'idle';
    const omplState = isSelected ? (isFallback ? 'failed' : 'done') : beforeSelected ? 'failed' : 'idle';
    const ikDetail = isSelected
      ? detail.candidateText(orientation)
      : `IK ${formatNumber(pose.ik_timeout, 2)}s`;
    const omplDetail = isSelected
      ? isFallback
        ? 'OMPL失败，使用IK直达'
        : `${plan?.ompl?.planner_id || 'OMPL'} ${formatNumber(plan?.ompl?.planning_time ?? pose.planning_time, 1)}s`
      : beforeSelected
        ? '未通过'
        : '等待IK通过';
    steps.push({
      label: `${detail.scanLabel}${index} IK`,
      detail: ikDetail,
      state: ikState
    });
    steps.push({
      label: `${detail.scanLabel}${index} OMPL`,
      detail: omplDetail,
      state: omplState
    });
  }
  steps.push({
    label: selected ? '确定' : pending ? '等待结果' : '待确定',
    detail: selected ? `${detail.title} · ${plan?.mode || '规划完成'}` : detail.title,
    state: selected ? 'done' : pending ? 'active' : 'idle'
  });
  return steps;
}

function describeCandidateAngles(orientation) {
  const parts = [];
  for (const key of ['roll', 'pitch', 'yaw']) {
    const value = Number(orientation?.[key]);
    if (Number.isFinite(value)) parts.push(`${key} ${formatNumber(value, 2)}`);
  }
  return parts.length ? parts.join(' / ') : '候选姿态';
}

function decisionModeMatches(planMode, currentMode) {
  if (planMode === currentMode) return true;
  return planMode === 'angled_grasp' && currentMode === 'z_parallel_grasp';
}

async function executePlan() {
  await runCommand(
    '执行规划',
    async () => {
      const result = await apiRequest('/api/execute', { arm: state.arm });
      await refreshStatus();
      return result;
    },
    { confirm: `即将让${armLabel()}执行最后一次规划，确认工作区安全后继续。`, confirmTitle: '确认执行轨迹' }
  );
}

async function runArmAction(action) {
  const pose = ['approach', 'grasp'].includes(action) ? readGraspPose() : readPose();
  const definitions = {
    approach: ['接近目标', '/api/approach', { pose, offset_z: numericValue('pickApproach', 0.1) }],
    lift: ['抬起机械臂', '/api/lift', { arm: state.arm, offset_z: numericValue('pickLift', 0.1) }],
    grasp: ['抓取目标', '/api/grasp', { pose }],
    place: ['放置目标', '/api/place', { pose }],
    gripper_open: ['张开夹爪', '/api/gripper', { arm: state.arm, action: 'open' }],
    gripper_close: ['闭合夹爪', '/api/gripper', { arm: state.arm, action: 'close' }]
  };
  const command = definitions[action];
  if (!command) return;
  if (action === 'gripper_open' || action === 'gripper_close') {
    const arm = state.arm;
    const previous = { ...(state.gripperByArm[arm] || {}) };
    const targetState = action === 'gripper_close' ? 'closed' : 'open';
    const commandState = state.gripperCommand[arm];
    const now = Date.now();
    if (
      commandState.inFlight
      || (commandState.target === targetState && now - commandState.at < GRIPPER_COMMAND_DEBOUNCE_MS)
    ) {
      return;
    }
    state.gripperCommand[arm] = { target: targetState, at: now, inFlight: true };
    applyGripperState(arm, {
      state: targetState,
      target_state: targetState,
      arm,
      joint: gripperJointName(arm),
      position: gripperPosition(previous),
      target_position: targetState === 'closed' ? GRIPPER_CLOSED_POSITION : GRIPPER_OPEN_POSITION,
      moving: gripperPosition(previous) !== (targetState === 'closed' ? GRIPPER_CLOSED_POSITION : GRIPPER_OPEN_POSITION),
      simulated: true
    }, { sceneFallback: false });
    byId('footerMessage').textContent = `${command[0]}已发送`;
    try {
      const result = await apiRequest(command[1], command[2]);
      if (result?.result) {
        applyGripperState(arm, result.result, { sceneFallback: false });
      }
    } catch (error) {
      applyGripperState(arm, previous, { sceneFallback: false });
      showToast(`${command[0]}失败：${error.message}`, 'bad', 5000);
    } finally {
      state.gripperCommand[arm] = { target: targetState, at: Date.now(), inFlight: false };
    }
    setTimeout(() => {
      refreshGripper();
      refreshStatus();
    }, 120);
    return;
  }
  const result = await runCommand(command[0], async () => {
    return apiRequest(command[1], command[2]);
  }, {
    confirm: `${command[0]}将驱动${armLabel()}，确认周围无人员或障碍物。`
  });
  if (action === 'gripper_open' || action === 'gripper_close') {
    applyGripperState(state.arm, result?.result || {}, { forceScene: true });
  }
  updateManualCollisionFromSummary(findManualCollisionSummary(result));
  refreshGripper();
  refreshStatus();
}

async function runAutoPick(options = {}) {
  const approachHeight = Number.isFinite(Number(options.approachHeight))
    ? Number(options.approachHeight)
    : numericValue('pickApproach', 0.1);
  const descendDistance = Number.isFinite(Number(options.descendDistance))
    ? Number(options.descendDistance)
    : numericValue('pickDescend', 0.05);
  const result = await runCommand(
    '自动夹取',
    async () => {
      return apiRequest('/api/pick', {
        pose: options.pose || readGraspPose(),
        approach_height: approachHeight,
        descend_distance: descendDistance,
        hold_seconds: numericValue('pickHold', 1.0),
        lift_height: numericValue('pickLift', 0.1)
      });
    },
    { confirm: `即将由${armLabel()}执行完整夹取流程，请确认目标位姿和现场安全。`, confirmTitle: '确认自动夹取' }
  );
  updateManualCollisionFromSummary(findManualCollisionSummary(result));
  refreshGripper();
}

function calibrationPointFromInputs(index) {
  const point = state.cameraCalibration.points[index];
  if (!point) throw new Error('标定点不存在');
  const x = Number(document.querySelector(`[data-calibration-x="${index}"]`)?.value ?? point.x);
  const y = Number(document.querySelector(`[data-calibration-y="${index}"]`)?.value ?? point.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) throw new Error('标定点坐标无效');
  point.x = x;
  point.y = y;
  return point;
}

async function loadCameraCalibrationScene() {
  try {
    const data = await getMujocoScene();
    const extras = new Set((data.extras || []).map(item => item.name));
    byId('cameraCalibrationPlateToggle').checked = extras.has('plate');
    byId('cameraCalibrationBlockToggle').checked = extras.has('calibration_block');
    state.cameraCalibration.sceneReady = extras.has('plate') && extras.has('calibration_block');
    if (!state.cameraCalibration.points.some(point => point.sample)) {
      state.cameraCalibration.points = buildCameraCalibrationPoints(data.config || {});
    }
    if (!state.cameraCalibration.sceneReady) {
      byId('cameraCalibrationStatus').textContent = '当前场景缺少盘子或 50 mm 标定块，请重建并重启仿真';
    }
    renderCameraCalibration();
  } catch (error) {
    state.cameraCalibration.sceneReady = false;
    byId('cameraCalibrationStatus').textContent = `场景读取失败：${error.message}`;
    renderCameraCalibration();
  }
}

async function updateCameraCalibrationFixtures() {
  const plateToggle = byId('cameraCalibrationPlateToggle');
  const blockToggle = byId('cameraCalibrationBlockToggle');
  const plate = Boolean(plateToggle.checked);
  const calibrationBlock = Boolean(blockToggle.checked);
  plateToggle.disabled = true;
  blockToggle.disabled = true;
  byId('cameraCalibrationStatus').textContent = '正在更新 MuJoCo 标定物...';
  try {
    await setMujocoCalibrationFixtures({
      plate,
      calibration_block: calibrationBlock
    });
    await Promise.all([refreshMujocoScene(), loadCameraCalibrationScene()]);
    const visible = [plate ? '盘子' : null, calibrationBlock ? '标定块' : null].filter(Boolean);
    byId('cameraCalibrationStatus').textContent = visible.length
      ? `MuJoCo 标定物已更新：显示${visible.join('、')}`
      : '盘子和标定块已从 MuJoCo 场景隐藏';
  } catch (error) {
    byId('cameraCalibrationStatus').textContent = `标定物更新失败：${error.message}`;
    showToast(`标定物显隐失败：${error.message}`, 'bad', 6000);
    await loadCameraCalibrationScene();
  } finally {
    plateToggle.disabled = false;
    blockToggle.disabled = false;
  }
}

function refreshCameraCalibrationImage() {
  const image = byId('cameraCalibrationImage');
  const streamPath = '/mujoco_camera/stream.mjpg';
  if (image && image.dataset.streamPath !== streamPath) {
    image.dataset.streamPath = streamPath;
    image.src = streamPath;
  }
}

function setCameraCalibrationBusy(running, status) {
  state.cameraCalibration.running = running;
  if (status) byId('cameraCalibrationStatus').textContent = status;
  renderCameraCalibration();
}

function renderCameraCalibration() {
  const calibration = state.cameraCalibration;
  const samples = calibration.points.filter(point => point.sample);
  const pointsRoot = byId('cameraCalibrationPoints');
  if (pointsRoot) {
    let previousSurface = null;
    pointsRoot.innerHTML = calibration.points.map((point, index) => {
      const sample = point.sample;
      const sampleState = sample
        ? `${formatNumber(sample.xy_error_mm, 1)} / ${formatNumber(sample.z_error_mm, 1)}`
        : '待采集';
      const heading = point.surface !== previousSurface
        ? `<div class="camera-calibration-group"><b>${escapeHtml(point.surfaceLabel)}</b><span>${point.surface === 'table' ? '9 点' : '5 点'}</span></div>`
        : '';
      previousSurface = point.surface;
      return `
        ${heading}
        <div class="camera-calibration-point" data-state="${sample ? (sample.xy_error_mm <= 10 ? 'good' : 'warn') : 'neutral'}">
          <span class="camera-calibration-point-index">${index + 1}</span>
          <label>X<input type="number" step="0.01" value="${formatNumber(point.x, 2)}" data-calibration-x="${index}" ${calibration.running ? 'disabled' : ''}></label>
          <label>Y<input type="number" step="0.01" value="${formatNumber(point.y, 2)}" data-calibration-y="${index}" ${calibration.running ? 'disabled' : ''}></label>
          <span class="camera-calibration-point-error" title="XY / Z 误差 (mm)">${escapeHtml(sampleState)}</span>
          <span class="camera-calibration-role">${point.validation ? '验证' : '拟合'}</span>
          <button class="icon-button" type="button" data-calibration-action="move" data-calibration-index="${index}" aria-label="移动定子到标定点 ${index + 1}" title="移动定子" ${calibration.running ? 'disabled' : ''}><i data-lucide="move-3d"></i></button>
          <button class="icon-button primary" type="button" data-calibration-action="capture" data-calibration-index="${index}" aria-label="采集标定点 ${index + 1}" title="采集 RGB-D 与真值" ${calibration.running ? 'disabled' : ''}><i data-lucide="crosshair"></i></button>
        </div>`;
    }).join('');
  }

  const resultsRoot = byId('cameraCalibrationResults');
  if (resultsRoot) {
    resultsRoot.innerHTML = samples.length ? `
      <div class="camera-calibration-result camera-calibration-result-head"><span>点位</span><span>真值 XYZ</span><span>视觉 XYZ</span><span>XY / Z</span></div>
      ${samples.map(point => {
        const sample = point.sample;
        const truth = sample.truth_xyz_m;
        const estimated = sample.estimated_world_xyz_m;
        return `<div class="camera-calibration-result">
          <span>P${calibration.points.indexOf(point) + 1}</span>
          <span>${truth.map(value => formatNumber(value, 3)).join(', ')}</span>
          <span>${estimated.map(value => formatNumber(value, 3)).join(', ')}</span>
          <b data-state="${sample.xy_error_mm <= 10 && sample.z_error_mm <= 10 ? 'good' : 'bad'}">${formatNumber(sample.xy_error_mm, 1)} / ${formatNumber(sample.z_error_mm, 1)}</b>
        </div>`;
      }).join('')}` : '<div class="empty-state">尚无采样结果</div>';
  }

  const analysis = calibration.analysis;
  const fit = calibration.fit;
  const validation = fit?.validation_metrics;
  const metrics = byId('cameraCalibrationMetrics');
  if (metrics) {
    metrics.innerHTML = analysis ? `
      <div><span>当前 RMS XY / Z</span><b>${formatNumber(analysis.rms_xy_error_mm, 1)} / ${formatNumber(analysis.rms_z_error_mm, 1)} mm</b></div>
      <div><span>当前最大 XY / Z</span><b>${formatNumber(analysis.max_xy_error_mm, 1)} / ${formatNumber(analysis.max_z_error_mm, 1)} mm</b></div>
      <div><span>拟合后验证 RMS</span><b>${validation ? `${formatNumber(validation.rms_xy_error_mm, 1)} / ${formatNumber(validation.rms_z_error_mm, 1)} mm` : '--'}</b></div>
      <div><span>拟合后验证最大</span><b>${validation ? `${formatNumber(validation.max_xy_error_mm, 1)} / ${formatNumber(validation.max_z_error_mm, 1)} mm` : '--'}</b></div>` : '';
  }

  const validationPassed = Boolean(
    validation
    && validation.count >= 5
    && validation.rms_xy_error_mm <= 10
    && validation.rms_z_error_mm <= 10
    && validation.max_xy_error_mm <= 15
    && validation.max_z_error_mm <= 15
  );
  let diagnosisState = 'neutral';
  let diagnosisText = `已采集 ${samples.length}/${calibration.points.length}，每点 ${CAMERA_CALIBRATION_FRAMES} 帧中位数`;
  if (!calibration.sceneReady) {
    diagnosisState = 'bad';
    diagnosisText = '场景缺少盘子或标定块，不能执行多高度拟合';
  } else if (fit && validationPassed) {
    diagnosisState = 'good';
    diagnosisText = `刚体外参验证通过：${fit.fit_count} 个拟合点 + ${fit.validation_count} 个独立验证点`;
  } else if (fit && validation) {
    diagnosisState = 'bad';
    diagnosisText = '刚体拟合后的验证误差仍超限，优先检查内参、深度尺度或目标中心提取';
  }
  const diagnosis = byId('cameraCalibrationDiagnosis');
  if (diagnosis) {
    diagnosis.dataset.state = diagnosisState;
    diagnosis.textContent = diagnosisText;
  }
  byId('cameraCalibrationCount').textContent = `${samples.length} / ${calibration.points.length}`;
  setBadge('cameraCalibrationState', calibration.running ? '采集中' : fit ? '已拟合' : samples.length ? '采样中' : '未采样', calibration.running ? 'warn' : fit ? 'good' : 'neutral');
  byId('runCameraCalibrationButton').disabled = calibration.running || !calibration.sceneReady;
  byId('clearCameraCalibrationButton').disabled = calibration.running || !samples.length;
  byId('applyCameraTranslationButton').disabled = calibration.running || !validationPassed;
  refreshIcons();
}

function calibrationObjectTiltRad(wxyz) {
  if (!Array.isArray(wxyz) || wxyz.length < 4) return Infinity;
  const qx = Number(wxyz[1]);
  const qy = Number(wxyz[2]);
  const zAlignment = Math.max(-1, Math.min(1, 1 - 2 * (qx * qx + qy * qy)));
  return Math.acos(zAlignment);
}

async function waitForCalibrationTruth(point, timeoutMs = 6500) {
  const deadline = Date.now() + timeoutMs;
  let stableCount = 0;
  const expectedZ = { table: 0.8435, plate: 0.8595, block: 0.8935 }[point.surface];
  while (Date.now() < deadline) {
    const response = await getMujocoGraspTarget();
    const center = response?.target?.object_center;
    const motion = response?.target?.object_motion || {};
    const atPosition = Math.abs(Number(center?.x) - point.x) <= 0.012
      && Math.abs(Number(center?.y) - point.y) <= 0.012
      && Math.abs(Number(center?.z) - expectedZ) <= 0.012;
    const stable = Number(motion.linear_speed_mps) <= 0.003
      && Number(motion.angular_speed_radps) <= 0.05
      && calibrationObjectTiltRad(response?.target?.orientation_wxyz) <= 0.12;
    stableCount = atPosition && stable ? stableCount + 1 : 0;
    if (stableCount >= 3) {
      return response;
    }
    await new Promise(resolve => window.setTimeout(resolve, 120));
  }
  throw new Error(`P${state.cameraCalibration.points.indexOf(point) + 1} 未在${point.surfaceLabel}稳定落定`);
}

async function moveCameraCalibrationPoint(index) {
  const point = calibrationPointFromInputs(index);
  await moveObject({ x: point.x, y: point.y, z: point.spawnZ });
  await waitForCalibrationTruth(point);
  refreshCameraCalibrationImage();
}

async function captureCameraCalibrationPoint(index) {
  const point = calibrationPointFromInputs(index);
  const frames = [];
  let previousStamp = null;
  let lastSampleError = null;
  const deadline = Date.now() + 12000;
  while (frames.length < CAMERA_CALIBRATION_FRAMES && Date.now() < deadline) {
    let cameraSample;
    try {
      cameraSample = await sampleMujocoCameraCalibration();
      lastSampleError = null;
    } catch (error) {
      lastSampleError = error;
      await new Promise(resolve => window.setTimeout(resolve, 120));
      continue;
    }
    if (cameraSample.rgb_stamp === previousStamp) {
      await new Promise(resolve => window.setTimeout(resolve, 80));
      continue;
    }
    const truthResponse = await getMujocoGraspTarget();
    const truth = truthResponse?.target?.object_center;
    if (![truth?.x, truth?.y, truth?.z].every(value => Number.isFinite(Number(value)))) {
      throw new Error('/api/mujoco/grasp_target 未返回有效真值');
    }
    frames.push({
      ...cameraSample,
      truth_xyz_m: [Number(truth.x), Number(truth.y), Number(truth.z)]
    });
    previousStamp = cameraSample.rgb_stamp;
  }
  if (frames.length < CAMERA_CALIBRATION_FRAMES) {
    const detail = lastSampleError?.message ? `：${lastSampleError.message}` : '';
    throw new Error(`等待 5 个独立且同步的 RGB-D 帧超时${detail}`);
  }
  const median = values => {
    const sorted = values.map(Number).sort((a, b) => a - b);
    return sorted[Math.floor(sorted.length / 2)];
  };
  const medianVector = (key) => [0, 1, 2].map(axis => median(frames.map(frame => frame[key][axis])));
  const truthXyz = medianVector('truth_xyz_m');
  const estimated = medianVector('estimated_world_xyz_m');
  const cameraXyz = medianVector('camera_xyz_m');
  const error = truthXyz.map((value, axis) => value - estimated[axis]);
  point.sample = {
    ...frames[Math.floor(frames.length / 2)],
    surface: point.surface,
    validation: point.validation,
    frame_count: frames.length,
    truth_xyz_m: truthXyz,
    camera_xyz_m: cameraXyz,
    estimated_world_xyz_m: estimated,
    error_m: error,
    xy_error_mm: Math.hypot(error[0], error[1]) * 1000,
    z_error_mm: Math.abs(error[2]) * 1000
  };
  const samples = state.cameraCalibration.points.filter(item => item.sample).map(item => item.sample);
  const result = await analyzeMujocoCameraCalibration(samples);
  state.cameraCalibration.analysis = result.analysis;
  const layers = new Set(samples.filter(sample => !sample.validation).map(sample => sample.surface));
  if (samples.filter(sample => !sample.validation).length >= 6 && layers.size >= 2) {
    try {
      const fitResult = await fitMujocoCameraCalibration(samples, state.cameraExtrinsic);
      state.cameraCalibration.fit = fitResult.fit;
    } catch {
      state.cameraCalibration.fit = null;
    }
  }
  refreshCameraCalibrationImage();
}

async function handleCameraCalibrationPointClick(event) {
  const button = event.target.closest('[data-calibration-action]');
  if (!button || state.cameraCalibration.running) return;
  const index = Number(button.dataset.calibrationIndex);
  const action = button.dataset.calibrationAction;
  try {
    setCameraCalibrationBusy(true, action === 'move' ? `正在移动定子到 P${index + 1}...` : `正在采集 P${index + 1}...`);
    if (action === 'move') await moveCameraCalibrationPoint(index);
    else await captureCameraCalibrationPoint(index);
    byId('cameraCalibrationStatus').textContent = action === 'move' ? `定子已移动到 P${index + 1}` : `P${index + 1} 已记录 ${CAMERA_CALIBRATION_FRAMES} 帧中位数`;
  } catch (error) {
    byId('cameraCalibrationStatus').textContent = `操作失败：${error.message}`;
    showToast(`相机标定失败：${error.message}`, 'bad', 6000);
  } finally {
    setCameraCalibrationBusy(false);
  }
}

function handleCameraCalibrationPointChange(event) {
  const index = Number(event.target.dataset.calibrationX ?? event.target.dataset.calibrationY);
  if (Number.isInteger(index)) calibrationPointFromInputs(index);
}

async function runCameraCalibration() {
  if (state.cameraCalibration.running) return;
  state.cameraCalibration.points.forEach(point => { point.sample = null; });
  state.cameraCalibration.analysis = null;
  state.cameraCalibration.fit = null;
  try {
    setCameraCalibrationBusy(true, '开始三点自动采集...');
    for (let index = 0; index < state.cameraCalibration.points.length; index += 1) {
      byId('cameraCalibrationStatus').textContent = `P${index + 1}/${state.cameraCalibration.points.length} ${state.cameraCalibration.points[index].surfaceLabel}：移动并等待稳定...`;
      await moveCameraCalibrationPoint(index);
      byId('cameraCalibrationStatus').textContent = `P${index + 1}/${state.cameraCalibration.points.length}：采集 ${CAMERA_CALIBRATION_FRAMES} 个 RGB-D 帧...`;
      await captureCameraCalibrationPoint(index);
      renderCameraCalibration();
    }
    byId('cameraCalibrationStatus').textContent = '19 点三高度采集完成，已生成刚体外参拟合与独立验证';
  } catch (error) {
    byId('cameraCalibrationStatus').textContent = `自动采集失败：${error.message}`;
    showToast(`自动标定失败：${error.message}`, 'bad', 6000);
  } finally {
    setCameraCalibrationBusy(false);
  }
}

function clearCameraCalibration() {
  state.cameraCalibration.points.forEach(point => { point.sample = null; });
  state.cameraCalibration.analysis = null;
  state.cameraCalibration.fit = null;
  byId('cameraCalibrationStatus').textContent = '采样已清空';
  renderCameraCalibration();
}

async function applyCameraTranslationCorrection() {
  const fit = state.cameraCalibration.fit;
  const fitted = fit?.fitted_extrinsic;
  if (!fitted) return;
  const accepted = await confirmAction(
    `将保存 ${fit.fit_count} 个拟合点求得的固定头部相机 XYZ + RPY，并以 ${fit.validation_count} 个独立点的误差作为验收结果。`,
    '应用刚体外参拟合',
    '保存外参'
  );
  if (!accepted) return;
  try {
    setCameraCalibrationBusy(true, '正在保存固定头部相机外参...');
    const result = await apiRequest('/api/camera_extrinsic', fitted);
    setCameraExtrinsic(result.extrinsic || fitted);
    state.cameraCalibration.points.forEach(point => { point.sample = null; });
    state.cameraCalibration.analysis = null;
    state.cameraCalibration.fit = null;
    byId('cameraCalibrationStatus').textContent = '刚体外参已保存，请重新执行 19 点采集验证';
    showToast('固定头部相机 XYZ + RPY 外参已保存', 'good', 5000);
  } catch (error) {
    byId('cameraCalibrationStatus').textContent = `保存失败：${error.message}`;
    showToast(`保存相机外参失败：${error.message}`, 'bad', 6000);
  } finally {
    setCameraCalibrationBusy(false);
  }
}

async function moveObjectAction() {
  const x = Number(byId("objX").value);
  const y = Number(byId("objY").value);
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    showToast("坐标无效");
    return;
  }
  await runCommand("移动方块", async () => moveObject({ x, y }));
  showToast("方块已移动");
}

async function rebuildSceneAction() {
  const plate = byId("showPlate").checked;
  const cylinder = byId("showCylinder").checked;
  const tableX = Number(byId("tableX").value);
  const tableY = Number(byId("tableY").value);
  const objectX = Number(byId("objX").value);
  const objectY = Number(byId("objY").value);
  const plateX = Number(byId("plateX").value);
  const plateY = Number(byId("plateY").value);
  const preset = byId("scenePreset").value;
  byId("sceneStatus").textContent = "重建中... (需要重启仿真生效)";
  const data = await runCommand("重建场景", async () => {
    return Promise.race([
      rebuildScene({
        preset, plate, cylinder, table_x: tableX, table_y: tableY,
        object_x: objectX, object_y: objectY, plate_x: plateX, plate_y: plateY
      }),
      new Promise((_, reject) => window.setTimeout(
        () => reject(new Error('场景重建超过 40 秒，请检查后端日志')),
        40000
      ))
    ]);
  }, {
    onError: error => {
      byId("sceneStatus").textContent = `重建失败：${error?.message || '未知错误'}`;
    }
  });
  if (!data) return;
  const objs = data.objects || "";
  byId("sceneStatus").textContent =
    "场景已保存并重建 (" + (objs || "基础场景") + "), 重启仿真后生效";
}

async function pickMujocoObject() {
  let target;
  try {
    const response = await getMujocoVisualGraspTarget();
    target = response?.target;
    if (![target?.x, target?.y, target?.z].every(value => Number.isFinite(Number(value)))) {
      throw new Error('头部 RGB-D 未返回有效的定子中心坐标');
    }
    setGraspPoseMode('z_parallel');
    setPose(target);
    const center = target.object_center || target;
    byId('footerMessage').textContent =
      `视觉定子中心：X ${formatNumber(center.x, 3)} / Y ${formatNumber(center.y, 3)} / Z ${formatNumber(center.z, 3)} m；已应用TCP偏移`;
  } catch (error) {
    showToast(`视觉定位失败：${error.message}`, 'bad', 5000);
    return;
  }
  // For the known object-center coordinate, descend the full approach height
  // so the existing pick sequence reaches the requested center before closing.
  await runAutoPick({
    approachHeight: 0.1,
    descendDistance: 0.1,
    pose: {
      ...readGraspPose(),
      planner_id: 'RRTConnectkConfigDefault',
      planning_time: 5.0,
      attempts: 4,
      ik_timeout: 1.5,
      mujoco_object_center: target.object_center || target,
      mujoco_nominal_tcp: { x: target.x, y: target.y, z: target.z }
    }
  });
}

async function pickPlacePlate() {
  const button = byId('pickPlacePlateButton');
  const buttonLabel = button?.querySelector('span');
  const shouldPlace = button?.dataset.stage === 'place';
  let target;
  try {
    const response = await getMujocoVisualGraspTarget();
    target = response?.target;
    if (![target?.x, target?.y, target?.z].every(value => Number.isFinite(Number(value)))) {
      throw new Error('头部 RGB-D 未返回有效的定子中心坐标');
    }
  } catch (error) {
    byId('pickPlaceStatus').textContent = `读取目标失败：${error.message}`;
    return;
  }
  if (!shouldPlace) {
    byId('pickPlaceStatus').textContent = '正在拿起定子...';
    await pickMujocoObject();
    const status = await apiRequest('/api/status');
    const attached = Boolean(status?.mujoco_scene?.object?.attached);
    if (attached) {
      if (button) button.dataset.stage = 'place';
      if (buttonLabel) buttonLabel.textContent = '放入槽中';
      byId('pickPlaceStatus').textContent = '定子已拿起；再次点击放入槽中';
    } else {
      byId('pickPlaceStatus').textContent = '定子未能稳定拿起，请重试';
    }
    return;
  }
  byId('pickPlaceStatus').textContent = '正在移动到槽并放置...';
  const result = await runCommand('放入槽中', () => apiRequest('/api/mujoco/pick_place_plate', {
    pose: {
      ...readGraspPose(),
      arm: state.arm,
      x: target.x, y: target.y, z: target.z,
      grasp_orientation_mode: 'angled_grasp',
      planner_id: 'RRTConnectkConfigDefault',
      planning_time: 5.0,
      attempts: 4,
      ik_timeout: 1.5,
      mujoco_object_center: target.object_center || target,
      mujoco_nominal_tcp: { x: target.x, y: target.y, z: target.z }
    },
    hover_height: 0.12
  }), {
    confirm: '机械臂将把已拿起的定子移动到槽上方并松爪。',
    confirmTitle: '确认放入槽中',
    onError: error => {
      byId('pickPlaceStatus').textContent = `任务失败：${error?.message || '未知错误'}`;
    }
  });
  if (!result) return;
  const completed = Boolean(result.result?.completed);
  byId('pickPlaceStatus').textContent = completed
    ? '任务完成：定子已释放到槽中'
    : `任务未完成：停在 ${result.result?.stage || '未知'} 阶段`;
  if (completed) {
    if (button) button.dataset.stage = 'pick';
    if (buttonLabel) buttonLabel.textContent = '拿起定子';
  }
  refreshStatus();
}

async function leftSideDropRotorToCylinder() {
  let target;
  let dropTarget;
  try {
    if (state.arm !== 'left') {
      await setArm('left');
    }
    const [targetResponse, sceneData] = await Promise.all([
      getMujocoGraspTarget(),
      getMujocoScene()
    ]);
    target = targetResponse?.target;
    if (![target?.x, target?.y, target?.z].every(value => Number.isFinite(Number(value)))) {
      throw new Error('仿真目标坐标无效');
    }
    dropTarget = findCylinderDropTarget(sceneData);
    setGraspPoseMode('z_parallel');
    setPose(target);
    const center = target.object_center || target;
    byId('footerMessage').textContent =
      `智能抓取目标：转子 (${formatNumber(center.x, 3)}, ${formatNumber(center.y, 3)}, ${formatNumber(center.z, 3)}) → ${dropTarget.label}中心 (${dropTarget.center.map(value => formatNumber(value, 3)).join(', ')})`;
  } catch (error) {
    byId('pickPlaceStatus').textContent = `准备失败：${error.message}`;
    showToast(`准备智能抓取任务失败：${error.message}`, 'bad', 5000);
    return;
  }

  byId('pickPlaceStatus').textContent = '等待确认：从转子上方下探搜索可解抓取点...';
  const response = await runCommand('智能抓取转子入圆柱', async () => {
    byId('pickPlaceStatus').textContent =
      `正在扫描垂直/斜向/侧面抓取，并移动到${dropTarget.label}中心上方...`;
    return runLeftSideDropRotorTask({
      smart_search: true,
      grasp_modes: ['vertical_grasp', 'angled_grasp', 'side_grasp'],
      search_offsets: [0.060, 0.045, 0.030, 0.015, 0.005, -0.005, -0.015, -0.030, -0.045],
      approach_height: 0.065,
      descend_distance: 0.065,
      hold_seconds: 0.45,
      lift_height: 0.055,
      fall_seconds: 1.8,
      drop_heights: [0.0, 0.015, 0.030, 0.050],
      release_descend_height: 0.045
    });
  }, {
    confirm: '将用左手从转子上方开始搜索可解点；若不可解会逐步下探，并在垂直、斜向、侧面三种抓取中选择可执行方案，保持转子与桌面水平后移动到圆柱/半圆槽中心上方开爪。',
    confirmTitle: '确认智能抓取投放',
    success: '智能抓取投放完成',
    onError: error => {
      byId('pickPlaceStatus').textContent = `任务失败：${error?.message || '未知错误'}`;
    }
  });

  const task = response?.result;
  if (!task) return;
  if (!task.completed) {
    byId('pickPlaceStatus').textContent = `任务未完成：${task.error || task.stage || '未知阶段'}`;
    refreshStatus();
    refreshGripper();
    return;
  }
  const searchStep = (Array.isArray(task.steps) ? task.steps : [])
    .find(step => step?.action === 'smart_grasp_search');
  const selectedMode = searchStep?.search?.mode;
  const selectedLabel = SMART_GRASP_MODE_LABELS[selectedMode] || '已选抓取';
  const probeOffset = Number(searchStep?.search?.probe_offset_m);
  const xyError = Number(task.xy_error_m);
  const levelError = Number(task.horizontal_transfer?.level_error_deg);
  const searchText = Number.isFinite(probeOffset)
    ? `${selectedLabel} / 下探偏移 ${formatNumber(probeOffset * 1000, 1)} mm`
    : selectedLabel;
  const levelText = Number.isFinite(levelError)
    ? `，水平误差 ${formatNumber(levelError, 2)}°`
    : '';
  byId('pickPlaceStatus').textContent = Number.isFinite(xyError)
    ? `任务完成：${searchText}${levelText}，最终 XY 误差 ${formatNumber(xyError * 1000, 1)} mm`
    : `任务完成：${searchText}${levelText}，转子已释放到目标中心上方`;
  refreshStatus();
  refreshGripper();
}

async function pickPlaceTable() {
  let target;
  try {
    const response = await getMujocoVisualGraspTarget();
    target = response?.target;
    if (![target?.x, target?.y, target?.z].every(value => Number.isFinite(Number(value)))) {
      throw new Error('未识别到红色方块的有效坐标');
    }
  } catch (error) {
    byId('pickPlaceStatus').textContent = `识别失败：${error.message}`;
    return;
  }
  byId('pickPlaceStatus').textContent = '正在执行：头部 RGB-D 定位 → 抓取 → 返回桌面原位 → 放置...';
  const result = await runCommand('抓取并放回桌面原位', () => apiRequest('/api/mujoco/pick_place_table', {
    pose: {
      ...readGraspPose(),
      arm: state.arm,
      x: target.x, y: target.y, z: target.z,
      grasp_orientation_mode: 'angled_grasp',
      planner_id: 'RRTConnectkConfigDefault',
      planning_time: 5.0,
      attempts: 4,
      ik_timeout: 1.5,
      mujoco_object_center: target.object_center || target,
      mujoco_nominal_tcp: { x: target.x, y: target.y, z: target.z }
    },
    hover_height: 0.12
  }), {
    confirm: '将识别并抓取红色方块，然后放回桌面初始位置。',
    confirmTitle: '确认执行放回原位任务',
    onError: error => {
      byId('pickPlaceStatus').textContent = `任务失败：${error?.message || '未知错误'}`;
    }
  });
  if (!result) return;
  const completed = Boolean(result.result?.completed);
  byId('pickPlaceStatus').textContent = completed
    ? '任务完成：红色方块已放回桌面原位'
    : `任务未完成：停在 ${result.result?.stage || '未知'} 阶段`;
  refreshStatus();
}

async function enableRobot() {
  await runCommand('双臂使能', () => apiRequest('/api/enable', { arm: 'both', enabled: true }), {
    confirm: '即将使能左右机械臂，确认急停可用且工作区安全。'
  });
  refreshRobotState();
}

async function disableRobot() {
  await runCommand('取消使能', () => apiRequest('/api/enable', { arm: 'both', enabled: false }));
  refreshRobotState();
}

async function resetEstop() {
  await runCommand('复位急停', () => apiRequest('/api/reset_estop', { arm: 'both' }), {
    confirm: '确认急停原因已经排除，然后复位左右臂急停状态。'
  });
  refreshRobotState();
}

async function homeRobot() {
  const result = await runCommand('双臂规划回零', () => apiRequest('/api/home_zero', {
    arm: 'both',
    avoid_platform: byId('avoidPlatform')?.checked !== false,
    collision_boxes: currentManualCollisionBoxes(),
    velocity_scaling: Number(byId('velocityScaling').value),
    acceleration_scaling: Number(byId('accelerationScaling').value),
    ...readOmplRuntimeConfig()
  }), {
    confirm: '将先用 MoveIt 规划避障路径，再按规划轨迹让左右臂依次回零。',
    confirmTitle: '确认双臂规划回零'
  });
  updateManualCollisionFromSummary(findManualCollisionSummary(result));
  refreshStatus();
}

async function resetTask() {
  const result = await runCommand(
    '任务归零',
    () => apiRequest('/api/mujoco/task_reset', {}),
    {
      confirm: '将张开双夹爪、把转子恢复到近左手起始位置，并让双臂归零。',
      confirmTitle: '确认任务归零'
    }
  );
  if (!result) return;
  await Promise.allSettled([refreshStatus(), refreshRobotState(), refreshGripper(), refreshMujocoScene()]);
  showToast('任务已恢复到初始状态', 'good', 5000);
}

async function emergencyStop() {
  if (!(await confirmAction('将立即向左右臂发送急停命令。', '紧急停止', '立即停止'))) return;
  try {
    await apiRequest('/api/estop', { arm: 'both', active: true });
    showToast('已发送双臂急停命令', 'warn', 5000);
    refreshRobotState();
  } catch (error) {
    showToast(`急停命令发送失败：${error.message}`, 'bad', 6000);
  }
}

async function loadCameraExtrinsic() {
  try {
    const data = await getCameraExtrinsic();
    setCameraExtrinsic(data.extrinsic || {});
    byId('extrinsicStatus').textContent = `已读取 ${data.path || 'camera_extrinsic.json'}`;
  } catch (error) {
    byId('extrinsicStatus').textContent = `读取失败：${error.message}`;
  }
}

async function saveCameraExtrinsic() {
  await runCommand('保存相机外参', async () => {
    const data = await apiRequest('/api/camera_extrinsic', readCameraExtrinsic());
    setCameraExtrinsic(data.extrinsic || readCameraExtrinsic());
    byId('extrinsicStatus').textContent = `已保存 ${data.path || 'camera_extrinsic.json'}`;
    return data;
  }, { confirm: '该配置将写入相机外参文件，并影响视觉目标与点云坐标。' });
}

async function loadVlmConfig() {
  try {
    const data = await getVlmConfig();
    const config = data.config || {};
    byId('vlmApiKey').value = config.api_key || '';
    byId('vlmBaseUrl').value = config.base_url || '';
    byId('vlmModel').value = config.model || '';
    byId('vlmPrompt').value = config.prompt || '';
    byId('vlmOffsetX').value = config.offset_x ?? 0;
    byId('vlmOffsetY').value = config.offset_y ?? 0;
    byId('vlmOffsetZ').value = config.offset_z ?? -0.1;
    byId('vlmStatus').textContent = `已读取 ${data.path || 'vlm_config.json'}`;
  } catch (error) {
    byId('vlmStatus').textContent = `读取失败：${error.message}`;
  }
}

function readVlmConfig() {
  return {
    api_key: byId('vlmApiKey').value.trim(),
    base_url: byId('vlmBaseUrl').value.trim(),
    model: byId('vlmModel').value.trim(),
    prompt: byId('vlmPrompt').value,
    offset_x: Number(byId('vlmOffsetX').value) || 0,
    offset_y: Number(byId('vlmOffsetY').value) || 0,
    offset_z: Number(byId('vlmOffsetZ').value) || 0,
  };
}

async function saveVlmConfigAction() {
  await runCommand('保存 VLM 配置', async () => {
    const data = await saveVlmConfig(readVlmConfig());
    const config = data.config || readVlmConfig();
    byId('vlmApiKey').value = config.api_key || '';
    byId('vlmBaseUrl').value = config.base_url || '';
    byId('vlmModel').value = config.model || '';
    byId('vlmPrompt').value = config.prompt || '';
    byId('vlmOffsetX').value = config.offset_x ?? 0;
    byId('vlmOffsetY').value = config.offset_y ?? 0;
    byId('vlmOffsetZ').value = config.offset_z ?? -0.1;
    byId('vlmStatus').textContent = `已保存 ${data.path || 'vlm_config.json'}`;
    return data;
  });
}

function showVlmStage(text, status) {
  const box = byId('vlmStageBox');
  const stageText = byId('vlmStageText');
  if (box) {
    box.style.display = 'flex';
    box.classList.remove('done', 'failed');
    if (status === 'done') box.classList.add('done');
    if (status === 'failed') box.classList.add('failed');
  }
  if (stageText) stageText.textContent = text;
}

function resetVlmOutput() {
  const box = byId('vlmOutputBox');
  if (box) box.style.display = 'none';
  for (const id of ['vlmOutVlm', 'vlmOutBbox', 'vlmOutPixel', 'vlmOutDepth', 'vlmOutWorld', 'vlmOutPick']) {
    const row = byId(id);
    if (row) row.style.display = 'none';
  }
}

function showVlmOutputRow(rowId, valueId, text) {
  const box = byId('vlmOutputBox');
  if (box) box.style.display = 'block';
  const row = byId(rowId);
  if (row) row.style.display = 'flex';
  const val = byId(valueId);
  if (val) val.textContent = text;
}

async function vlmConfirmAction() {
  const prompt = byId('vlmGraspPrompt').value.trim();
  if (!prompt) {
    byId('vlmGraspStatus').textContent = '请输入指令';
    return;
  }
  const btn = byId('vlmConfirmButton');
  if (btn) btn.disabled = true;
  resetVlmOutput();
  byId('vlmGraspStatus').textContent = '';
  showVlmStage('获取相机画面中...');

  try {
    const response = await fetch('/api/vlm_grasp_stream', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        prompt,
        execute: true,
        arm: state.arm,
        place_in_plate: /放(?:到|入|进)?[^，。]{0,8}(?:盘|plate)/i.test(prompt),
        place_on_table: /放(?:到|在)?[^，。]{0,8}(?:桌面|桌上|table)/i.test(prompt)
      }),
    });
    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';
    let finalError = null;

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;
      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n');
      buffer = lines.pop() || '';
      for (const line of lines) {
        if (!line.startsWith('data: ')) continue;
        const evt = JSON.parse(line.slice(6));
        const stage = evt.stage;
        if (evt.status) showVlmStage(evt.status);
        if (stage === 'vlm_done') {
          showVlmOutputRow('vlmOutVlm', 'vlmOutVlmVal', evt.label || '-');
          showVlmOutputRow('vlmOutBbox', 'vlmOutBboxVal',
            (evt.bbox_norm || evt.bbox || []).map(v => Number(v).toFixed(3)).join(', '));
        } else if (stage === 'depth_done') {
          const px = evt.pixel || {};
          showVlmOutputRow('vlmOutPixel', 'vlmOutPixelVal', `(${px.u}, ${px.v})`);
          const dp = evt.depth || {};
          showVlmOutputRow('vlmOutDepth', 'vlmOutDepthVal', `${Number(dp.value_m || 0).toFixed(3)} m`);
          const w = evt.world || {};
          showVlmOutputRow('vlmOutWorld', 'vlmOutWorldVal',
            `X:${w.x}  Y:${w.y}  Z:${w.z}`);
        } else if (stage === 'pick_done') {
          const pk = evt.pick || {};
          const ok = pk.completed === true;
          const detail = pk.error || pk.grasp_debug?.lift?.reason || pk.grasp_debug?.close?.reason;
          showVlmOutputRow('vlmOutPick', 'vlmOutPickVal', ok ? '成功' : (detail || '未完成'));
          showVlmStage(ok ? '抓取完成' : '抓取未完成', ok ? 'done' : 'failed');
        } else if (stage === 'place') {
          showVlmOutputRow('vlmOutPick', 'vlmOutPickVal', '抓取成功，正在放入盘子');
          showVlmStage(evt.status || '正在移动到盘子...');
        } else if (stage === 'place_done') {
          const placed = evt.place?.completed === true;
          const detail = evt.place?.error;
          showVlmOutputRow('vlmOutPick', 'vlmOutPickVal', placed ? '已放入盘子' : (detail || '放置未完成'));
          showVlmStage(placed ? '抓取并放置完成' : '放置未完成', placed ? 'done' : 'failed');
        } else if (stage === 'error') {
          finalError = evt.error;
          showVlmStage('执行失败', 'failed');
        } else if (stage === 'done') {
          if (!finalError && !byId('vlmOutPick') || (byId('vlmOutPick').style.display === 'none')) {
            if (!finalError) showVlmStage('定位完成', 'done');
          }
        }
      }
    }
    byId('vlmGraspStatus').textContent = finalError || '执行完成';
  } catch (error) {
    showVlmStage('执行失败', 'failed');
    byId('vlmGraspStatus').textContent = `错误: ${error.message}`;
  } finally {
    if (btn) btn.disabled = false;
  }
}

async function loadPosesLib() {
  try {
    const data = await getPosesLib();
    state.posesLib = (data.poses_lib && data.poses_lib.poses) || [];
    renderPosesList();
    byId('posesStatus').textContent = `已读取 ${data.path || 'poses.json'} (${state.posesLib.length} 个姿态)`;
  } catch (error) {
    byId('posesStatus').textContent = `读取失败：${error.message}`;
  }
}

function renderPosesList() {
  const list = byId('posesList');
  if (!state.posesLib || state.posesLib.length === 0) {
    list.innerHTML = '<div class="empty-hint">暂无姿态，点「记录当前位姿」添加</div>';
    return;
  }
  list.innerHTML = state.posesLib.map((pose, index) => `
    <div class="library-item">
      <div class="library-item-info">
        <span class="library-item-name">${escapeHtml(pose.name || pose.id || `姿态${index + 1}`)}</span>
        <span class="library-item-meta">${escapeHtml(pose.id || '')} · (${formatNumber(pose.x, 3)}, ${formatNumber(pose.y, 3)}, ${formatNumber(pose.z, 3)})</span>
      </div>
      <div class="library-item-actions">
        <button class="icon-button" data-pose-preview="${index}" type="button" title="预览(移动到此姿态)"><i data-lucide="play"></i></button>
        <button class="icon-button" data-pose-delete="${index}" type="button" title="删除"><i data-lucide="trash-2"></i></button>
      </div>
    </div>
  `).join('');
  refreshIcons();
}

async function recordCurrentPose() {
  const editor = byId('poseEditor');
  if (editor.style.display === 'none') {
    editor.style.display = 'flex';
  }
  byId('poseX').value = '读取中...';
  byId('poseY').value = '';
  byId('poseZ').value = '';
  try {
    const data = await getLinks(state.arm);
    const links = data.links || [];
    const tcp = links.find(l => l.name && l.name.includes('TCP'));
    if (!tcp || !tcp.position) {
      showToast('无法读取当前 TCP 坐标');
      return;
    }
    byId('poseX').value = Number(tcp.position[0]).toFixed(3);
    byId('poseY').value = Number(tcp.position[1]).toFixed(3);
    byId('poseZ').value = Number(tcp.position[2]).toFixed(3);
    byId('poseOrientationMode').value = state.graspPoseMode || 'vertical';
    showToast(`已读取实时坐标: (${Number(tcp.position[0]).toFixed(3)}, ${Number(tcp.position[1]).toFixed(3)}, ${Number(tcp.position[2]).toFixed(3)})`);
  } catch (error) {
    showToast(`读取坐标失败: ${error.message}`);
  }
}

async function addPose() {
  const entry = {
    id: byId('poseId').value.trim(),
    name: byId('poseNameInput').value.trim(),
    description: byId('poseDesc').value.trim(),
    x: Number(byId('poseX').value),
    y: Number(byId('poseY').value),
    z: Number(byId('poseZ').value),
    grasp_orientation_mode: byId('poseOrientationMode').value,
    arm: state.arm,
  };
  if (!entry.id) {
    showToast('请填写 ID (英文)');
    return;
  }
  if (!Number.isFinite(entry.x) || !Number.isFinite(entry.y) || !Number.isFinite(entry.z)) {
    showToast('坐标无效');
    return;
  }
  if (!state.posesLib) state.posesLib = [];
  const existing = state.posesLib.findIndex(p => p.id === entry.id);
  if (existing >= 0) {
    if (!(await confirmAction(`ID「${entry.id}」已存在，是否覆盖？`, '确认覆盖'))) return;
    state.posesLib[existing] = entry;
  } else {
    state.posesLib.push(entry);
  }
  await runCommand('保存姿态库', async () => {
    const data = await savePosesLib({ version: 1, poses: state.posesLib });
    state.posesLib = (data.poses_lib && data.poses_lib.poses) || [];
    renderPosesList();
    byId('posesStatus').textContent = `已保存 (${state.posesLib.length} 个姿态)`;
    return data;
  });
  byId('poseId').value = '';
  byId('poseNameInput').value = '';
  byId('poseDesc').value = '';
}

async function previewPose(index) {
  const pose = state.posesLib[index];
  if (!pose) return;
  setPose({ x: pose.x, y: pose.y, z: pose.z });
  if (pose.grasp_orientation_mode) {
    state.graspPoseMode = pose.grasp_orientation_mode;
  }
  await runCommand('预览姿态', async () => {
    await apiRequest('/api/plan', readPose());
    return apiRequest('/api/execute', { arm: pose.arm || 'right' });
  });
  showToast(`已移动到: ${pose.name || pose.id}`);
}

async function deletePose(index) {
  const pose = state.posesLib[index];
  if (!pose) return;
  if (!(await confirmAction(`删除姿态「${pose.name || pose.id}」？`, '确认删除'))) return;
  await runCommand('删除姿态', async () => {
    const data = await deletePoseLib({ id: pose.id, name: pose.name });
    state.posesLib = (data.poses_lib && data.poses_lib.poses) || [];
    renderPosesList();
    byId('posesStatus').textContent = `已删除 (${state.posesLib.length} 个姿态)`;
    return data;
  });
}

async function handlePosesClick(event) {
  const preview = event.target.closest('[data-pose-preview]')?.dataset.posePreview;
  const remove = event.target.closest('[data-pose-delete]')?.dataset.poseDelete;
  if (preview !== undefined) { await previewPose(Number(preview)); return; }
  if (remove !== undefined) { await deletePose(Number(remove)); return; }
}

async function loadSceneCloud() {
  try {
    const result = await scene.loadScenePointCloud();
    setBadge('cloudState', `${result.count} 点`, 'good');
  } catch {
    setBadge('cloudState', '暂无点云', 'neutral');
  }
}

function toggleCloud() {
  state.cloudVisible = !state.cloudVisible;
  localStorage.setItem('unoarm.cloudVisible', String(state.cloudVisible));
  scene.setCloudVisible(state.cloudVisible);
  updateCloudButton();
}

async function saveSceneCloud() {
  await runCommand('保存场景点云', async () => {
    const result = await apiRequest('/api/scene_pointcloud/save', {});
    await loadSceneCloud();
    return result;
  });
}

async function rebuildCloudWithCurrentExtrinsic() {
  await runCommand('按当前外参重建点云', async () => {
    const extrinsicResult = await apiRequest('/api/camera_extrinsic', readCameraExtrinsic());
    setCameraExtrinsic(extrinsicResult.extrinsic || readCameraExtrinsic());
    const cloudResult = await apiRequest('/api/scene_pointcloud/save', {});
    await loadSceneCloud();
    byId('extrinsicStatus').textContent = `已保存 ${extrinsicResult.path || 'camera_extrinsic.json'}`;
    return cloudResult;
  }, {
    confirm: '将保存当前相机外参，并让视觉服务按该外参重新保存静态场景点云。',
    confirmTitle: '确认重建点云'
  });
}

async function loadPlatformObstacle(showMessage) {
  try {
    const data = await getPlatformObstacle();
    const obstacle = data.platform_obstacle;
    if (!hasPlatformObstacle(obstacle)) {
      scene?.setPlatformObstacle(null);
      state.platformObstacleVisible = false;
      state.platformObstacleApplied = false;
      byId('platformState').textContent = '避障区未加载';
      updatePlatformButtons();
      if (showMessage) showToast('当前没有已应用的避障区', 'warn');
      return null;
    }

    scene?.setPlatformObstacle(obstacle);
    state.platformObstacleVisible = true;
    state.platformObstacleApplied = true;
    byId('platformState').textContent = '避障区已显示';
    updatePlatformButtons();
    if (showMessage) showToast('已显示平台避障区');
    return obstacle;
  } catch (error) {
    byId('platformState').textContent = `读取失败：${error.message}`;
    updatePlatformButtons();
    return null;
  }
}

async function togglePlatformObstacle() {
  if (state.platformObstacleVisible) {
    scene?.setPlatformObstacle(null);
    state.platformObstacleVisible = false;
    byId('platformState').textContent = state.platformObstacleApplied
      ? '避障区已隐藏，规划场景仍保留'
      : '避障区已隐藏';
    updatePlatformButtons();
    return;
  }
  await loadPlatformObstacle(true);
}

async function applyPlatformObstacle() {
  if (state.platformObstacleApplied) {
    await runCommand('取消平台避障区', async () => {
      const data = await apiRequest('/api/platform_obstacle/clear', {});
      scene?.setPlatformObstacle(null);
      state.platformObstacleVisible = false;
      state.platformObstacleApplied = false;
      byId('platformState').textContent = '避障区已从规划场景移除';
      updatePlatformButtons();
      return data;
    }, {
      confirm: '将从 MoveIt 规划场景移除当前平台避障区，后续规划不再使用该避障区。',
      confirmTitle: '确认取消避障区'
    });
    return;
  }

  await runCommand('应用平台避障区', async () => {
    const data = await apiRequest('/api/platform_obstacle/apply', {});
    scene.setPlatformObstacle(data.platform_obstacle);
    state.platformObstacleVisible = true;
    state.platformObstacleApplied = true;
    byId('platformState').textContent = '避障区已应用到规划场景';
    updatePlatformButtons();
    return data;
  });
}

async function loadWorkspaceBounds(showMessage = false) {
  try {
    const data = await getWorkspaceBounds();
    state.workspaceBoundsByArm = {
      right: normalizeWorkspaceBounds(data.right, 'right'),
      left: normalizeWorkspaceBounds(data.left, 'left')
    };
    renderWorkspaceBounds();
    scene?.setWorkspaceBounds(currentWorkspaceBounds());
    updateWorkspaceBoundsStatus('已读取');
    if (showMessage) showToast('已读取工作区边界');
  } catch (error) {
    byId('workspaceBoundsState').textContent = `读取失败：${error.message}`;
  }
}

function defaultWorkspaceBounds(arm = state.arm) {
  return arm === 'left'
    ? { enabled: false, min: [-0.35, -0.65, 0.2], max: [0.8, 0.35, 1.55] }
    : { enabled: false, min: [-0.8, -0.65, 0.2], max: [0.35, 0.35, 1.55] };
}

function currentWorkspaceBounds() {
  return normalizeWorkspaceBounds(state.workspaceBoundsByArm[state.arm], state.arm);
}

function normalizeWorkspaceBounds(bounds = {}, arm = state.arm) {
  const fallback = state.workspaceBoundsByArm?.[arm] || defaultWorkspaceBounds(arm);
  const vector = (key, fallbackValue) => {
    const source = Array.isArray(bounds[key]) ? bounds[key] : fallbackValue;
    return [0, 1, 2].map(axis => {
      const value = Number(source?.[axis]);
      return Number.isFinite(value) ? value : fallbackValue[axis];
    });
  };
  return {
    enabled: Boolean(bounds.enabled),
    min: vector('min', fallback.min),
    max: vector('max', fallback.max)
  };
}

function renderWorkspaceBounds() {
  const bounds = currentWorkspaceBounds();
  const root = byId('workspaceBoundsFields');
  if (!root) return;
  root.innerHTML = `
    <label class="workspace-enable"><input id="workspaceBoundsEnabled" type="checkbox" ${bounds.enabled ? 'checked' : ''}><span>启用${armLabel()}边界拒绝</span></label>
    <div class="compact-fields workspace-bounds-grid">
      ${workspaceBoundInputs('min', bounds.min, ['X min', 'Y min', 'Z min'])}
      ${workspaceBoundInputs('max', bounds.max, ['X max', 'Y max', 'Z max'])}
    </div>
  `;
}

function workspaceBoundInputs(key, values, labels) {
  return labels.map((label, axis) => `
    <label>${label}<input data-workspace-bound="${key}" data-workspace-axis="${axis}"
      type="number" step="0.001" value="${Number(values[axis] ?? 0)}" /><span>m</span></label>
  `).join('');
}

function readWorkspaceBoundsInputs() {
  const vector = key => [0, 1, 2].map(axis => {
    const value = Number(byId('workspaceBoundsFields')?.querySelector(`[data-workspace-bound="${key}"][data-workspace-axis="${axis}"]`)?.value);
    return Number.isFinite(value) ? value : 0;
  });
  return {
    enabled: byId('workspaceBoundsEnabled')?.checked === true,
    min: vector('min'),
    max: vector('max')
  };
}

function handleWorkspaceBoundsInput() {
  state.workspaceBoundsByArm[state.arm] = normalizeWorkspaceBounds(readWorkspaceBoundsInputs(), state.arm);
  scene?.setWorkspaceBounds(currentWorkspaceBounds());
  byId('workspaceBoundsState').textContent = `${armLabel()}工作区边界已修改，保存后生效`;
}

async function saveWorkspaceBoundsConfig() {
  state.workspaceBoundsByArm[state.arm] = normalizeWorkspaceBounds(readWorkspaceBoundsInputs(), state.arm);
  await runCommand('保存工作区边界', async () => {
    const data = await saveWorkspaceBounds({ arm: state.arm, ...currentWorkspaceBounds() });
    state.workspaceBoundsByArm = {
      right: normalizeWorkspaceBounds(data.right || state.workspaceBoundsByArm.right, 'right'),
      left: normalizeWorkspaceBounds(data.left || state.workspaceBoundsByArm.left, 'left')
    };
    renderWorkspaceBounds();
    scene?.setWorkspaceBounds(currentWorkspaceBounds());
    updateWorkspaceBoundsStatus('已保存');
    return data;
  });
}

function updateWorkspaceBoundsStatus(prefix = '') {
  const bounds = currentWorkspaceBounds();
  const text = bounds.enabled
    ? `${prefix ? `${prefix} · ` : ''}${armLabel()}工作区已启用`
    : `${prefix ? `${prefix} · ` : ''}${armLabel()}工作区未启用`;
  byId('workspaceBoundsState').textContent = text;
}

async function loadManualCollisionBoxes(showMessage = false) {
  try {
    const data = await getManualCollisionBoxes();
    state.manualCollision.boxes = data.boxes || [];
    state.manualCollision.applied = Boolean(data.summary?.applied && !data.summary?.cleared);
    renderManualCollisionBoxes();
    scene?.setManualCollisionBoxes(state.manualCollision.boxes);
    renderBenchmarkCollisionBoxOptions();
    updateManualCollisionButtons();
    byId('manualCollisionState').textContent = state.manualCollision.boxes.length
      ? `已读取 ${state.manualCollision.boxes.length} 个碰撞箱`
      : '尚未添加手动碰撞箱';
    if (showMessage) showToast('已读取手动碰撞箱');
  } catch (error) {
    byId('manualCollisionState').textContent = `读取失败：${error.message}`;
    updateManualCollisionButtons();
  }
}

function addManualCollisionBox() {
  const pose = readPose();
  const index = state.manualCollision.boxes.length + 1;
  state.manualCollision.boxes.push({
    id: `box_${index}`,
    name: `碰撞箱 ${index}`,
    enabled: true,
    center: [Number(pose.x), Number(pose.y), Number(pose.z)],
    dimensions: [0.18, 0.18, 0.18],
    rpy: [0, 0, 0]
  });
  state.manualCollision.applied = false;
  renderManualCollisionBoxes();
  scene?.setManualCollisionBoxes(state.manualCollision.boxes);
  renderBenchmarkCollisionBoxOptions();
  updateManualCollisionButtons();
  byId('manualCollisionState').textContent = '已添加碰撞箱，下一次规划/控制自动生效';
}

function renderManualCollisionBoxes() {
  const root = byId('manualCollisionFields');
  const boxes = state.manualCollision.boxes || [];
  if (!boxes.length) {
    renderEmpty('manualCollisionFields', '没有手动碰撞箱');
    renderBenchmarkCollisionBoxOptions();
    return;
  }
  root.innerHTML = boxes.map((box, index) => `
    <div class="collision-box-row" data-collision-index="${index}">
      <div class="collision-box-header">
        <input data-collision-field="name" type="text" value="${escapeHtml(box.name || `碰撞箱 ${index + 1}`)}" aria-label="碰撞箱名称">
        <label><input data-collision-field="enabled" type="checkbox" ${box.enabled === false ? '' : 'checked'}><span>启用</span></label>
        <button class="icon-button" type="button" data-collision-delete="${index}" aria-label="删除碰撞箱" title="删除碰撞箱">
          <i data-lucide="trash-2"></i>
        </button>
      </div>
      <div class="compact-fields collision-fields">
        ${collisionVectorInputs(index, box, 'center', ['X', 'Y', 'Z'], 'm', 0.001)}
        ${collisionVectorInputs(index, box, 'dimensions', ['长', '宽', '高'], 'm', 0.001)}
        ${collisionVectorInputs(index, box, 'rpy', ['Roll', 'Pitch', 'Yaw'], 'rad', 0.001)}
      </div>
    </div>
  `).join('');
  renderBenchmarkCollisionBoxOptions();
  refreshIcons();
}

function collisionVectorInputs(index, box, key, labels, unit, step) {
  const values = Array.isArray(box[key]) ? box[key] : [0, 0, 0];
  return labels.map((label, axis) => `
    <label>${label}<input data-collision-vector="${key}" data-collision-axis="${axis}" data-collision-index="${index}"
      type="number" step="${step}" value="${Number(values[axis] ?? 0)}" /><span>${unit}</span></label>
  `).join('');
}

function handleManualCollisionInput() {
  state.manualCollision.boxes = readManualCollisionInputs();
  state.manualCollision.applied = false;
  scene?.setManualCollisionBoxes(state.manualCollision.boxes);
  updateManualCollisionButtons();
  byId('manualCollisionState').textContent = '配置已修改，下一次规划/控制自动同步到 MoveIt';
}

function handleManualCollisionClick(event) {
  const button = event.target.closest('[data-collision-delete]');
  if (!button) return;
  state.manualCollision.boxes = readManualCollisionInputs();
  state.manualCollision.boxes.splice(Number(button.dataset.collisionDelete), 1);
  state.manualCollision.applied = false;
  renderManualCollisionBoxes();
  scene?.setManualCollisionBoxes(state.manualCollision.boxes);
  renderBenchmarkCollisionBoxOptions();
  updateManualCollisionButtons();
  byId('manualCollisionState').textContent = '已删除碰撞箱，下一次规划/控制自动更新 MoveIt';
}

function readManualCollisionInputs() {
  return Array.from(document.querySelectorAll('.collision-box-row')).map((row, fallbackIndex) => {
    const index = Number(row.dataset.collisionIndex);
    const previous = state.manualCollision.boxes[index] || {};
    const vector = key => [0, 1, 2].map(axis => {
      const input = row.querySelector(`[data-collision-vector="${key}"][data-collision-axis="${axis}"]`);
      const value = Number(input?.value);
      return Number.isFinite(value) ? value : 0;
    });
    return {
      id: previous.id || `box_${fallbackIndex + 1}`,
      name: row.querySelector('[data-collision-field="name"]')?.value?.trim() || `碰撞箱 ${fallbackIndex + 1}`,
      enabled: row.querySelector('[data-collision-field="enabled"]')?.checked !== false,
      center: vector('center'),
      dimensions: vector('dimensions').map(value => Math.max(0.005, Math.abs(value))),
      rpy: vector('rpy')
    };
  });
}

function currentManualCollisionBoxes() {
  if (document.querySelector('.collision-box-row')) {
    state.manualCollision.boxes = readManualCollisionInputs();
  }
  return state.manualCollision.boxes || [];
}

async function saveManualCollisionConfig() {
  state.manualCollision.boxes = readManualCollisionInputs();
  await runCommand('保存手动碰撞箱', async () => {
    const data = await saveManualCollisionBoxes({ boxes: state.manualCollision.boxes });
    state.manualCollision.boxes = data.boxes || [];
    state.manualCollision.applied = false;
    renderManualCollisionBoxes();
    scene?.setManualCollisionBoxes(state.manualCollision.boxes);
    renderBenchmarkCollisionBoxOptions();
    updateManualCollisionButtons();
    byId('manualCollisionState').textContent = `已保存 ${state.manualCollision.boxes.length} 个碰撞箱，下一次规划/控制自动生效`;
    return data;
  });
}

async function applyManualCollisionConfig() {
  state.manualCollision.boxes = readManualCollisionInputs();
  await runCommand('应用手动碰撞箱', async () => {
    const data = await applyManualCollisionBoxes({ boxes: state.manualCollision.boxes });
    const summary = data.manual_collision || {};
    state.manualCollision.boxes = summary.boxes || state.manualCollision.boxes;
    state.manualCollision.applied = Number(summary.count || 0) > 0;
    renderManualCollisionBoxes();
    scene?.setManualCollisionBoxes(state.manualCollision.boxes);
    renderBenchmarkCollisionBoxOptions();
    updateManualCollisionButtons();
    byId('manualCollisionState').textContent = `已应用 ${summary.count ?? 0} 个启用碰撞箱到 MoveIt`;
    return data;
  });
}

async function clearManualCollisionConfig() {
  await runCommand('清除手动碰撞箱', async () => {
    const data = await clearManualCollisionBoxes();
    const summary = data.manual_collision || {};
    state.manualCollision.boxes = summary.boxes || state.manualCollision.boxes;
    state.manualCollision.applied = false;
    renderManualCollisionBoxes();
    scene?.setManualCollisionBoxes(state.manualCollision.boxes);
    renderBenchmarkCollisionBoxOptions();
    updateManualCollisionButtons();
    byId('manualCollisionState').textContent = '已从 MoveIt 移除手动碰撞箱，网页配置仍保留';
    return data;
  }, {
    confirm: '将从 MoveIt PlanningScene 移除手动碰撞箱，但保留网页配置。',
    confirmTitle: '确认清除手动碰撞箱'
  });
}

function updateManualCollisionButtons() {
  const boxes = state.manualCollision.boxes || [];
  const enabledCount = boxes.filter(box => box.enabled !== false).length;
  byId('applyCollisionBoxesButton').querySelector('span').textContent =
    state.manualCollision.applied ? `已应用 ${enabledCount} 个` : '提前应用';
  setCommandButtonState(
    'applyCollisionBoxesButton',
    state.manualCollision.applied ? 'active-good' : enabledCount ? 'ready-warning' : 'inactive',
    state.manualCollision.applied
  );
  setCommandButtonState('clearCollisionBoxesButton', state.manualCollision.applied ? 'ready-warning' : 'inactive', false);
}

function updateManualCollisionFromSummary(summary) {
  if (!summary) return;
  state.manualCollision.boxes = summary.boxes || state.manualCollision.boxes;
  state.manualCollision.applied = Boolean(summary.applied && Number(summary.count || 0) > 0);
  renderManualCollisionBoxes();
  scene?.setManualCollisionBoxes(state.manualCollision.boxes);
  renderBenchmarkCollisionBoxOptions();
  updateManualCollisionButtons();
  byId('manualCollisionState').textContent = state.manualCollision.applied
    ? `已自动同步 ${summary.count ?? 0} 个碰撞箱到 MoveIt`
    : '当前没有启用的碰撞箱';
}

function findManualCollisionSummary(value) {
  if (!value || typeof value !== 'object') return null;
  if (value.manual_collision) return value.manual_collision;
  for (const key of ['plan', 'result', 'results', 'approach', 'descend', 'lift']) {
    const found = findManualCollisionSummary(value[key]);
    if (found) return found;
  }
  for (const child of Object.values(value)) {
    if (child && typeof child === 'object' && child !== value) {
      const found = findManualCollisionSummary(child);
      if (found) return found;
    }
  }
  return null;
}

function updateRobotCommandButtons({ anyConnected, anyEnabled, allEnabled, anyEstop }) {
  setCommandButtonState('enableButton', allEnabled ? 'active-good' : 'ready-good', allEnabled);
  byId('enableButton').querySelector('span').textContent = allEnabled ? '已全部使能' : '双臂使能';

  setCommandButtonState('disableButton', anyEnabled ? 'ready-danger' : 'inactive', false);
  byId('disableButton').querySelector('span').textContent = anyEnabled ? '取消使能' : '未使能';

  setCommandButtonState('resetEstopButton', anyEstop ? 'ready-warning' : 'inactive', anyEstop);
  byId('resetEstopButton').querySelector('span').textContent = anyEstop ? '复位急停' : '急停正常';

  setCommandButtonState('homeButton', anyConnected && allEnabled && !anyEstop ? 'ready-neutral' : 'inactive', false);
  byId('homeButton').querySelector('span').textContent = '双臂规划回零';
  setCommandButtonState('taskResetButton', anyConnected && allEnabled && !anyEstop ? 'ready-neutral' : 'inactive', false);
  byId('taskResetButton').querySelector('span').textContent = '任务归零';
}

function updateCloudButton() {
  const button = byId('toggleCloudButton');
  button.querySelector('span').textContent = state.cloudVisible ? '隐藏点云' : '显示点云';
  setCommandButtonState('toggleCloudButton', state.cloudVisible ? 'active-neutral' : 'inactive', state.cloudVisible);
}

function updatePlatformButtons() {
  const displayButton = byId('loadPlatformButton');
  displayButton.querySelector('span').textContent = state.platformObstacleVisible ? '隐藏避障区' : '显示避障区';
  setCommandButtonState('loadPlatformButton', state.platformObstacleVisible ? 'active-neutral' : 'ready-neutral', state.platformObstacleVisible);

  const applyButton = byId('applyPlatformButton');
  applyButton.querySelector('span').textContent = state.platformObstacleApplied ? '取消避障区' : '应用避障区';
  setCommandButtonState('applyPlatformButton', state.platformObstacleApplied ? 'active-warning' : 'ready-warning', state.platformObstacleApplied);
}

function setCommandButtonState(id, mode, pressed) {
  const button = byId(id);
  button.dataset.mode = mode;
  button.setAttribute('aria-pressed', String(Boolean(pressed)));
}

function hasPlatformObstacle(obstacle) {
  if (!obstacle) return false;
  if (Array.isArray(obstacle)) return obstacle.length > 0;
  if (Array.isArray(obstacle.platform_obstacles)) return obstacle.platform_obstacles.length > 0;
  return Boolean(obstacle.center || obstacle.dimensions);
}

async function loadJointConfig() {
  byId('jointConfigStatus').textContent = `正在读取${armLabel()}配置...`;
  try {
    const data = await getJointConfig(state.arm);
    state.jointConfig = data;
    applyJointConfigLimits(data);
    renderJointConfig(data);
    byId('jointConfigStatus').textContent = `已读取 ${data.files?.bridge || armLabel()}`;
  } catch (error) {
    byId('jointConfigStatus').textContent = `读取失败：${error.message}`;
  }
}

async function loadJointLimits() {
  const results = await Promise.allSettled([getJointConfig('right'), getJointConfig('left')]);
  results.forEach(result => {
    if (result.status === 'fulfilled') applyJointConfigLimits(result.value);
  });
}

function applyJointConfigLimits(config) {
  const arm = config?.arm === 'left' ? 'left' : 'right';
  const names = config?.joint_names || [];
  const limits = {};
  names.forEach((name, index) => {
    const item = config?.joint_limits_rad?.[index];
    limits[name] = item?.enabled && Number.isFinite(Number(item.lower)) && Number.isFinite(Number(item.upper))
      ? { lower: Number(item.lower), upper: Number(item.upper) }
      : { lower: -1000000, upper: 1000000 };
  });
  state.jointLimitsByArm[arm] = limits;
  scene?.setJointLimits({
    ...state.jointLimitsByArm.right,
    ...state.jointLimitsByArm.left
  });
}

function renderJointConfig(config) {
  const names = config.joint_names || [];
  const root = byId('jointConfigFields');
  root.innerHTML = `
    <div class="joint-config-header"><span>关节</span><span>零位</span><span>限位</span><span>下限</span><span>上限</span></div>
    ${names.map((name, index) => `
      <div class="joint-config-row">
        <b>J${index + 1}</b>
        <input id="cfg_offset_${index}" type="number" step="1" value="${Number(config.zero_offsets?.[index] || 0)}" aria-label="${escapeHtml(name)}零位">
        <input id="cfg_limit_${index}" type="checkbox" ${config.limit_enabled?.[index] ? 'checked' : ''} aria-label="${escapeHtml(name)}启用限位">
        <input id="cfg_min_${index}" type="number" step="1" value="${Number(config.raw_limit_a?.[index] || 0)}" aria-label="${escapeHtml(name)}下限">
        <input id="cfg_max_${index}" type="number" step="1" value="${Number(config.raw_limit_b?.[index] || 0)}" aria-label="${escapeHtml(name)}上限">
      </div>
    `).join('')}
  `;
}

async function saveJointConfig() {
  const names = state.jointConfig?.joint_names || [];
  if (!names.length) {
    showToast('请先读取关节配置', 'warn');
    return;
  }
  const body = { arm: state.arm, zero_offsets: [], limit_enabled: [], raw_limit_a: [], raw_limit_b: [] };
  names.forEach((_name, index) => {
    body.zero_offsets.push(Number.parseInt(byId(`cfg_offset_${index}`).value, 10));
    body.limit_enabled.push(byId(`cfg_limit_${index}`).checked);
    body.raw_limit_a.push(Number.parseInt(byId(`cfg_min_${index}`).value, 10));
    body.raw_limit_b.push(Number.parseInt(byId(`cfg_max_${index}`).value, 10));
  });
  await runCommand('保存关节配置', async () => {
    const result = await apiRequest('/api/joint_config', body);
    state.jointConfig = result;
    applyJointConfigLimits(result);
    renderJointConfig(result);
    return result;
  }, {
    confirm: `将覆盖${armLabel()}的编码器零位与原始限位配置。该操作会影响实际运动范围。`,
    confirmTitle: '确认写入关节配置'
  });
}

async function loadKinematics() {
  byId('kinematicsStatus').textContent = '正在读取求解器配置...';
  try {
    const data = await getKinematics();
    state.kinematics = data;
    renderKinematics(data);
  } catch (error) {
    byId('kinematicsStatus').textContent = `读取失败：${error.message}`;
  }
}

function solverLabel(info, options) {
  if (!info) return '--';
  const option = options.find(item => item.id === info.solver_id || item.plugin === info.solver_plugin);
  return option ? option.label : info.solver_plugin || info.solver_id || '--';
}

function renderKinematics(config) {
  const options = config.options || [];
  const select = byId('ikSolverSelect');
  const active = config.active_solver_id || 'kdl';
  const hasActive = options.some(option => option.id === active);
  select.innerHTML = [
    ...(!hasActive && active ? [{
      id: active,
      label: active === 'mixed' ? '左右臂配置不一致' : active,
      available: false,
      plugin: ''
    }] : []),
    ...options
  ].map(option => `
    <option value="${escapeHtml(option.id)}" ${option.available ? '' : 'disabled'} ${option.id === active ? 'selected' : ''}>
      ${escapeHtml(option.label)}${option.available ? '' : '（未安装）'}
    </option>
  `).join('');

  if (active !== 'mixed' && options.some(option => option.id === active && option.available)) {
    select.value = active;
  } else {
    const firstAvailable = options.find(option => option.available);
    if (firstAvailable) select.value = firstAvailable.id;
  }

  const right = config.groups?.arm;
  const left = config.groups?.left_arm;
  byId('ikRightSolver').textContent = `右臂 ${solverLabel(right, options)}`;
  byId('ikLeftSolver').textContent = `左臂 ${solverLabel(left, options)}`;
  byId('kinematicsStatus').textContent =
    `配置文件：${config.path || '--'}。保存后需重启 MoveIt 才会生效。`;
}

async function saveKinematics() {
  const solverId = byId('ikSolverSelect').value;
  if (!solverId) {
    showToast('请选择可用的 IK 求解器', 'warn');
    return;
  }
  const selected = state.kinematics?.options?.find(option => option.id === solverId);
  const label = selected?.label || solverId;
  await runCommand('保存 IK 求解器', async () => {
    const result = await apiRequest('/api/kinematics', { solver_id: solverId, target: 'both' });
    state.kinematics = result;
    renderKinematics(result);
    return result;
  }, {
    confirm: `将左右臂 MoveIt IK 求解器配置切换为 ${label}。已运行的 MoveIt 不会动态切换，保存后需要重启 MoveIt。`,
    confirmTitle: '确认切换 IK 求解器',
    success: `IK 求解器已写入：${label}，重启 MoveIt 后生效`
  });
}

function loadSavedOmplConfig() {
  try {
    return normalizeOmplRuntimeConfig(JSON.parse(localStorage.getItem(OMPL_CONFIG_STORAGE_KEY) || '{}'));
  } catch {
    return { ...DEFAULT_OMPL_CONFIG };
  }
}

function normalizeOmplRuntimeConfig(config = {}) {
  return {
    planner_id: String(config.planner_id || DEFAULT_OMPL_CONFIG.planner_id),
    planning_time: finiteUiNumber(config.planning_time, DEFAULT_OMPL_CONFIG.planning_time),
    attempts: Math.max(1, Math.round(finiteUiNumber(config.attempts, DEFAULT_OMPL_CONFIG.attempts))),
    ik_timeout: finiteUiNumber(config.ik_timeout, DEFAULT_OMPL_CONFIG.ik_timeout)
  };
}

function finiteUiNumber(value, fallback) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

async function loadOmplConfig() {
  byId('omplConfigStatus').textContent = '正在读取 OMPL 配置...';
  try {
    const data = await getOmplConfig();
    state.ompl.options = data;
    const recommended = data.recommended?.config || data.defaults || DEFAULT_OMPL_CONFIG;
    const plannerIds = new Set((data.planners || []).map(planner => planner.planner_id));
    const current = normalizeOmplRuntimeConfig(state.ompl.config);
    state.ompl.config = plannerIds.has(current.planner_id)
      ? current
      : normalizeOmplRuntimeConfig(recommended);
    saveOmplRuntimeConfig();
    renderOmplConfig();
  } catch (error) {
    byId('omplConfigStatus').textContent = `读取失败：${error.message}`;
    renderEmpty('omplPlannerDetails', 'OMPL 配置不可用');
  }
}

function renderOmplConfig() {
  const data = state.ompl.options || {};
  const planners = data.planners || [];
  const presets = data.presets || [];
  const presetSelect = byId('omplPresetSelect');
  presetSelect.innerHTML = presets.map(preset => `
    <option value="${escapeHtml(preset.id)}">
      ${escapeHtml(preset.custom ? `自定义 · ${preset.label}` : preset.label)}
    </option>
  `).join('');
  const presetIds = new Set(presets.map(preset => preset.id));
  if (state.ompl.selectedPresetId && presetIds.has(state.ompl.selectedPresetId)) {
    presetSelect.value = state.ompl.selectedPresetId;
  } else if (data.recommended?.id) {
    presetSelect.value = data.recommended.id;
  }

  const select = byId('omplPlannerSelect');
  select.innerHTML = planners.map(planner => `
    <option value="${escapeHtml(planner.planner_id)}">
      ${escapeHtml(planner.label || planner.planner_id)}
    </option>
  `).join('');
  if (planners.some(planner => planner.planner_id === state.ompl.config.planner_id)) {
    select.value = state.ompl.config.planner_id;
  }
  applyOmplFields(state.ompl.config);
  renderOmplRecommendation(data.recommended);
  renderOmplPlannerDetails();
  byId('omplConfigStatus').textContent =
    `运行时生效 · 配置文件：${data.path || '--'}`;
}

function applyOmplFields(config) {
  const normalized = normalizeOmplRuntimeConfig(config);
  byId('omplPlannerSelect').value = normalized.planner_id;
  byId('omplPlanningTime').value = normalized.planning_time;
  byId('omplAttempts').value = normalized.attempts;
  byId('omplIkTimeout').value = normalized.ik_timeout;
}

function readOmplRuntimeConfig() {
  const plannerSelect = byId('omplPlannerSelect');
  if (!plannerSelect) return { ...state.ompl.config };
  const config = normalizeOmplRuntimeConfig({
    planner_id: plannerSelect.value || state.ompl.config.planner_id,
    planning_time: byId('omplPlanningTime').value,
    attempts: byId('omplAttempts').value,
    ik_timeout: byId('omplIkTimeout').value
  });
  state.ompl.config = config;
  return config;
}

function saveOmplRuntimeConfig() {
  localStorage.setItem(OMPL_CONFIG_STORAGE_KEY, JSON.stringify(state.ompl.config));
}

function handleOmplConfigInput() {
  state.ompl.config = readOmplRuntimeConfig();
  saveOmplRuntimeConfig();
  renderOmplPlannerDetails();
  const planner = omplPlannerOption(state.ompl.config.planner_id);
  byId('omplConfigStatus').textContent =
    `下一次规划使用 ${planner?.label || state.ompl.config.planner_id} · ${state.ompl.config.planning_time}s / ${state.ompl.config.attempts} 次`;
}

function applySelectedOmplPreset() {
  const presetId = byId('omplPresetSelect').value;
  const preset = (state.ompl.options?.presets || []).find(item => item.id === presetId)
    || state.ompl.options?.recommended;
  state.ompl.selectedPresetId = preset?.id || null;
  state.ompl.config = normalizeOmplRuntimeConfig(preset?.config || DEFAULT_OMPL_CONFIG);
  applyOmplFields(state.ompl.config);
  saveOmplRuntimeConfig();
  renderOmplPlannerDetails();
  showToast(`已应用 OMPL 配置：${preset?.label || '推荐配置'}`);
  byId('omplConfigStatus').textContent = `已应用 ${preset?.label || '推荐配置'}，下一次规划生效`;
}

async function saveCurrentOmplPreset() {
  const name = normalizedPresetName('omplPresetName');
  const config = readOmplRuntimeConfig();
  const data = await runCommand(
    '保存规划预设',
    () => savePlanningPreset({ name, source: 'ompl', config }),
    { quiet: true, success: '规划预设已保存' }
  );
  if (!data) return;
  const saved = findSavedPreset(data.presets, name, config);
  state.ompl.selectedPresetId = saved?.id || null;
  await loadOmplConfig();
  byId('omplConfigStatus').textContent = `已保存到预设库：${saved?.label || name}`;
  showToast(`已保存规划预设：${saved?.label || name}`);
}

async function deleteSelectedOmplPreset() {
  const presetId = byId('omplPresetSelect').value;
  const preset = (state.ompl.options?.presets || []).find(item => item.id === presetId);
  if (!preset?.custom) {
    showToast('只能删除自定义预设', 'warn');
    return;
  }
  if (!(await confirmAction(`删除自定义预设「${preset.label}」？`, '删除规划预设', '删除'))) return;
  const data = await runCommand(
    '删除规划预设',
    () => deletePlanningPreset({ id: preset.id }),
    { quiet: true, success: '规划预设已删除' }
  );
  if (!data) return;
  state.ompl.selectedPresetId = null;
  await loadOmplConfig();
  showToast(`已删除规划预设：${preset.label}`);
}

function normalizedPresetName(inputId) {
  const input = byId(inputId);
  const name = String(input?.value || '').trim();
  if (name) return name;
  const fallback = `规划预设 ${new Date().toLocaleString('zh-CN', { hour12: false })}`;
  if (input) input.value = fallback;
  return fallback;
}

function findSavedPreset(presets = [], name, config) {
  const sameName = presets.find(preset => preset.name === name || preset.label === name);
  if (sameName) return sameName;
  return presets.find(preset => {
    const presetConfig = preset.config || {};
    return presetConfig.planner_id === config.planner_id
      && Number(presetConfig.planning_time) === Number(config.planning_time)
      && Number(presetConfig.attempts) === Number(config.attempts)
      && Number(presetConfig.ik_timeout) === Number(config.ik_timeout);
  }) || null;
}

function omplPlannerOption(plannerId) {
  return (state.ompl.options?.planners || []).find(planner => planner.planner_id === plannerId) || null;
}

function renderOmplRecommendation(recommended) {
  const root = byId('omplRecommended');
  const config = recommended?.config || DEFAULT_OMPL_CONFIG;
  const planner = omplPlannerOption(config.planner_id);
  root.innerHTML = `
    <div class="ompl-recommendation-main">
      <strong>${escapeHtml(recommended?.label || '推荐配置')}</strong>
      <span>${escapeHtml(planner?.label || config.planner_id)}</span>
    </div>
    <div class="ompl-config-chips">
      <span>规划 ${formatNumber(config.planning_time, 1)}s</span>
      <span>尝试 ${Number(config.attempts)} 次</span>
      <span>IK ${formatNumber(config.ik_timeout, 1)}s</span>
    </div>
    <p>${escapeHtml(recommended?.description || '用于快速调试的默认配置。')}</p>
  `;
}

function renderOmplPlannerDetails() {
  const planner = omplPlannerOption(state.ompl.config.planner_id);
  const root = byId('omplPlannerDetails');
  if (!planner) {
    renderEmpty('omplPlannerDetails', '请选择规划器');
    return;
  }
  const configRows = Object.entries(planner.config || {});
  root.innerHTML = `
    <div class="ompl-detail-row"><b>planner_id</b><span>${escapeHtml(planner.planner_id)}</span></div>
    <div class="ompl-detail-row"><b>type</b><span>${escapeHtml(planner.type || '--')}</span></div>
    <div class="ompl-detail-row"><b>groups</b><span>${escapeHtml((planner.groups || []).join(', ') || '--')}</span></div>
    ${configRows.map(([key, value]) => `
      <div class="ompl-detail-row"><b>${escapeHtml(key)}</b><span>${escapeHtml(String(value))}</span></div>
    `).join('')}
  `;
}

async function loadBenchmarkOptions(showMessage = false) {
  const status = byId('benchmarkStatus');
  if (status && state.activePanel === 'benchmark') status.textContent = '正在读取跑分配置...';
  try {
    const data = await getBenchmarkOptions(state.arm);
    state.benchmark.options = data;
    applyBenchmarkDefaults(data);
    renderBenchmarkCollisionBoxOptions();
    renderBenchmarkPlanners();
    renderBenchmarkQuickStats();
    renderBenchmarkSummary();
    renderBenchmarkSamples();
    if (status && showMessage) status.textContent = '跑分配置已读取';
  } catch (error) {
    if (status) status.textContent = `跑分配置读取失败：${error.message}`;
    renderEmpty('benchmarkPlanners', '规划器配置不可用');
  }
}

function applyBenchmarkDefaults(data = {}) {
  const maxSamples = Number(data.max_samples || 120);
  for (const id of ['benchmarkBoxCount', 'benchmarkEdgeCount', 'benchmarkRandomCount']) {
    const input = byId(id);
    if (input && Number.isFinite(maxSamples)) input.max = String(maxSamples);
  }
  if (state.benchmark.defaultsApplied) return;
  const defaults = data.defaults || {};
  setInputValueIfFinite('benchmarkBoxCount', defaults.box_top_count);
  setInputValueIfFinite('benchmarkBoxOffsetCm', defaults.box_top_offset_cm);
  setInputValueIfFinite('benchmarkEdgeCount', defaults.edge_count);
  setInputValueIfFinite('benchmarkEdgeDistanceCm', defaults.edge_distance_cm);
  setInputValueIfFinite('benchmarkRandomCount', defaults.random_count);
  setInputValueIfFinite('benchmarkPlanningTime', defaults.planning_time);
  setInputValueIfFinite('benchmarkAttempts', defaults.attempts);
  setInputValueIfFinite('benchmarkIkTimeout', defaults.ik_timeout);
  state.benchmark.defaultsApplied = true;
}

function setInputValueIfFinite(id, value) {
  const input = byId(id);
  const number = Number(value);
  if (input && Number.isFinite(number)) input.value = String(value);
}

function renderBenchmarkCollisionBoxOptions() {
  const select = byId('benchmarkCollisionBox');
  if (!select) return;
  const previous = select.value;
  const manualBoxes = (state.manualCollision.boxes || []).filter(box => box.enabled !== false);
  const optionBoxes = Array.isArray(state.benchmark.options?.collision_boxes)
    ? state.benchmark.options.collision_boxes
    : [];
  const boxes = manualBoxes.length ? manualBoxes : optionBoxes;
  if (!boxes.length) {
    select.innerHTML = '<option value="">没有启用的碰撞箱</option>';
    select.disabled = true;
    const boxCount = byId('benchmarkBoxCount');
    if (boxCount) boxCount.value = '0';
    return;
  }
  select.disabled = false;
  select.innerHTML = boxes.map((box, index) => {
    const value = box.id || box.name || `box_${index + 1}`;
    const label = box.name || box.id || `碰撞箱 ${index + 1}`;
    return `<option value="${escapeHtml(value)}">${escapeHtml(label)}</option>`;
  }).join('');
  const values = new Set(boxes.map((box, index) => String(box.id || box.name || `box_${index + 1}`)));
  select.value = values.has(previous) ? previous : select.options[0]?.value || '';
}

function renderBenchmarkPlanners() {
  const root = byId('benchmarkPlanners');
  if (!root) return;
  const planners = state.benchmark.options?.planners || state.ompl.options?.planners || [];
  if (!planners.length) {
    renderEmpty('benchmarkPlanners', '未读取到适用于当前手臂的规划器');
    return;
  }
  let selected = new Set(state.benchmark.selectedPlanners);
  if (!selected.size) selected = new Set([state.ompl.config.planner_id]);
  const validSelected = planners.filter(planner => selected.has(planner.planner_id)).map(planner => planner.planner_id);
  state.benchmark.selectedPlanners = validSelected.length ? validSelected : [planners[0].planner_id];
  const selectedSet = new Set(state.benchmark.selectedPlanners);
  root.innerHTML = planners.map(planner => `
    <label class="planner-option">
      <input type="checkbox" value="${escapeHtml(planner.planner_id)}" ${selectedSet.has(planner.planner_id) ? 'checked' : ''}>
      <span>
        <b>${escapeHtml(planner.label || planner.planner_id)}</b>
        <small>${escapeHtml(planner.planner_id)}</small>
      </span>
    </label>
  `).join('');
}

function selectedBenchmarkPlannerIds() {
  const root = byId('benchmarkPlanners');
  const inputs = Array.from(root?.querySelectorAll('input[type="checkbox"]') || []);
  if (inputs.length) {
    state.benchmark.selectedPlanners = inputs.filter(input => input.checked).map(input => input.value);
    return [...state.benchmark.selectedPlanners];
  }
  if (state.benchmark.selectedPlanners.length) return [...state.benchmark.selectedPlanners];
  const fallback = state.ompl.config?.planner_id || state.benchmark.options?.planners?.[0]?.planner_id;
  return fallback ? [fallback] : [];
}

function benchmarkPlannerCount() {
  return selectedBenchmarkPlannerIds().length || state.benchmark.result?.planners?.length || 0;
}

function benchmarkRequestBase() {
  const pose = readPose();
  if (document.querySelector('.collision-box-row')) {
    state.manualCollision.boxes = readManualCollisionInputs();
  }
  const seedText = byId('benchmarkSeed')?.value;
  const seed = Number(seedText);
  const request = {
    ...pose,
    avoid_collisions: byId('avoidPlatform')?.checked !== false,
    box_top_count: nonNegativeUiInt('benchmarkBoxCount', 0),
    box_top_offset_cm: finiteUiNumber(byId('benchmarkBoxOffsetCm')?.value, 5),
    edge_count: nonNegativeUiInt('benchmarkEdgeCount', 0),
    edge_distance_cm: finiteUiNumber(byId('benchmarkEdgeDistanceCm')?.value, 5),
    random_count: nonNegativeUiInt('benchmarkRandomCount', 0),
    planning_time: finiteUiNumber(byId('benchmarkPlanningTime')?.value, DEFAULT_OMPL_CONFIG.planning_time),
    attempts: Math.max(1, Math.round(finiteUiNumber(byId('benchmarkAttempts')?.value, DEFAULT_OMPL_CONFIG.attempts))),
    ik_timeout: finiteUiNumber(byId('benchmarkIkTimeout')?.value, DEFAULT_OMPL_CONFIG.ik_timeout),
    collision_boxes: state.manualCollision.boxes || [],
    planners: selectedBenchmarkPlannerIds()
  };
  const collisionBox = byId('benchmarkCollisionBox');
  if (collisionBox?.disabled) request.box_top_count = 0;
  else if (collisionBox?.value) request.collision_box_id = collisionBox.value;
  if (seedText !== '' && Number.isFinite(seed)) request.seed = Math.max(1, Math.round(seed));
  return request;
}

function benchmarkPresetConfigFromRequest(request) {
  const selected = selectedBenchmarkPlannerIds();
  let plannerId = selected[0] || state.ompl.config?.planner_id || DEFAULT_OMPL_CONFIG.planner_id;
  const summaryRows = Object.values(state.benchmark.result?.summary || {})
    .filter(row => !selected.length || selected.includes(row.planner_id));
  if (summaryRows.length) {
    summaryRows.sort((a, b) => {
      const rateDelta = Number(b.success_rate || 0) - Number(a.success_rate || 0);
      if (rateDelta) return rateDelta;
      return Number(a.mean_elapsed_ms || Infinity) - Number(b.mean_elapsed_ms || Infinity);
    });
    plannerId = summaryRows[0].planner_id || plannerId;
  }
  return normalizeOmplRuntimeConfig({
    planner_id: plannerId,
    planning_time: request.planning_time,
    attempts: request.attempts,
    ik_timeout: request.ik_timeout
  });
}

async function saveBenchmarkRuntimePreset() {
  const name = normalizedPresetName('benchmarkPresetName');
  const request = benchmarkRequestBase();
  const config = benchmarkPresetConfigFromRequest(request);
  const data = await runCommand(
    '保存跑分配置',
    () => savePlanningPreset({ name, source: 'benchmark', config }),
    { quiet: true, success: '跑分配置已保存' }
  );
  if (!data) return;
  const saved = findSavedPreset(data.presets, name, config);
  state.ompl.config = config;
  state.ompl.selectedPresetId = saved?.id || null;
  saveOmplRuntimeConfig();
  await loadOmplConfig();
  byId('benchmarkStatus').textContent = `已保存到规划预设库：${saved?.label || name}`;
  showToast(`已保存跑分配置：${saved?.label || name}`);
}

async function exportCurrentBenchmarkCsv() {
  if (!state.benchmark.result) {
    byId('benchmarkStatus').textContent = '没有可导出的跑分结果';
    showToast('请先完成一次跑分', 'warn');
    return;
  }
  const result = { arm: state.arm, ...state.benchmark.result };
  const data = await runCommand(
    '导出跑分 CSV',
    () => exportBenchmarkCsv({ result }),
    { quiet: true, success: '跑分 CSV 已导出' }
  );
  if (!data) return;
  downloadBenchmarkCsv(data);
  byId('benchmarkStatus').textContent = `CSV 已保存：${data.path}`;
  showToast('跑分 CSV 已导出');
}

function downloadBenchmarkCsv(data) {
  if (!data?.csv) return;
  const blob = new Blob([data.csv], { type: 'text/csv;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = (data.path || 'unoarm_benchmark.csv').split('/').pop() || 'unoarm_benchmark.csv';
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

function nonNegativeUiInt(id, fallback) {
  return Math.max(0, Math.round(finiteUiNumber(byId(id)?.value, fallback)));
}

function requestedBenchmarkPointCount() {
  return nonNegativeUiInt('benchmarkBoxCount', 0)
    + nonNegativeUiInt('benchmarkEdgeCount', 0)
    + nonNegativeUiInt('benchmarkRandomCount', 0);
}

async function generateBenchmarkSet() {
  if (!state.benchmark.options) await loadBenchmarkOptions(false);
  const request = benchmarkRequestBase();
  const data = await runCommand(
    '生成跑分测试点',
    () => generateBenchmarkSamples(request),
    { quiet: true, success: '测试点已生成' }
  );
  if (!data) return;
  state.benchmark.options = { ...state.benchmark.options, ...data };
  state.benchmark.samples = data.samples || [];
  state.benchmark.selectedSampleId = null;
  state.benchmark.result = null;
  state.benchmark.progress = null;
  if (data.seed && byId('benchmarkSeed')) byId('benchmarkSeed').value = data.seed;
  scene?.setBenchmarkSamples(state.benchmark.samples);
  renderBenchmarkCollisionBoxOptions();
  renderBenchmarkPlanners();
  renderBenchmarkQuickStats();
  renderBenchmarkSummary();
  renderBenchmarkSamples();
  renderBenchmarkProgress();
  byId('benchmarkStatus').textContent = `已生成 ${state.benchmark.samples.length} 个测试点，种子 ${data.seed ?? '--'}`;
  showToast(`已生成 ${state.benchmark.samples.length} 个跑分测试点`);
}

async function runBenchmarkSet() {
  if (!state.benchmark.options) await loadBenchmarkOptions(false);
  const request = benchmarkRequestBase();
  if (!request.planners.length) {
    byId('benchmarkStatus').textContent = '至少选择一个规划器';
    showToast('至少选择一个规划器', 'warn');
    return;
  }
  if (state.benchmark.samples.length) request.samples = state.benchmark.samples;
  startBenchmarkProgressPolling(request);
  const data = await runCommand(
    '规划器跑分',
    () => runBenchmark(request),
    { quiet: true, success: '跑分完成' }
  );
  await pollBenchmarkProgress();
  stopBenchmarkProgressPolling();
  if (!data) return;
  state.benchmark.result = data;
  state.benchmark.samples = data.samples || state.benchmark.samples;
  state.benchmark.selectedSampleId = null;
  scene?.setBenchmarkSamples(state.benchmark.samples);
  renderBenchmarkQuickStats();
  renderBenchmarkSummary();
  renderBenchmarkSamples();
  const total = Number(data.total_points || state.benchmark.samples.length);
  const valid = Number(data.valid_points || 0);
  byId('benchmarkStatus').textContent =
    `跑分完成：IK 有效 ${valid}/${total}，耗时 ${formatNumber(Number(data.elapsed_ms || 0) / 1000, 2)} s`;
  showToast(`跑分完成：有效点 ${valid}/${total}`);
  renderBenchmarkProgress();
  refreshStatus();
}

function startBenchmarkProgressPolling(request = {}) {
  stopBenchmarkProgressPolling();
  const totalPoints = state.benchmark.samples.length || (
    Number(request.box_top_count || 0) + Number(request.edge_count || 0) + Number(request.random_count || 0)
  );
  const plannerCount = selectedBenchmarkPlannerIds().length;
  state.benchmark.progress = {
    running: true,
    stage: 'starting',
    label: '等待 MoveIt 服务',
    percent: 0,
    total_points: totalPoints,
    completed_steps: 0,
    total_steps: totalPoints + totalPoints * plannerCount,
    ik_done: 0,
    ik_ok: 0,
    plan_done: 0,
    plan_total: totalPoints * plannerCount,
    elapsed_ms: 0,
    eta_sec: null
  };
  renderBenchmarkProgress();
  benchmarkProgressTimer = window.setInterval(pollBenchmarkProgress, 500);
  pollBenchmarkProgress();
}

function stopBenchmarkProgressPolling() {
  if (!benchmarkProgressTimer) return;
  window.clearInterval(benchmarkProgressTimer);
  benchmarkProgressTimer = null;
}

async function pollBenchmarkProgress() {
  try {
    const progress = await getBenchmarkProgress(state.arm);
    state.benchmark.progress = progress;
    renderBenchmarkProgress();
  } catch {
    // Progress polling is best-effort; the main benchmark request reports errors.
  }
}

function renderBenchmarkProgress() {
  const root = byId('benchmarkProgress');
  if (!root) return;
  const progress = state.benchmark.progress;
  if (!progress) {
    root.hidden = true;
    return;
  }
  root.hidden = false;
  const percent = Math.max(0, Math.min(100, Number(progress.percent || 0)));
  byId('benchmarkProgressStage').textContent = benchmarkProgressLabel(progress);
  byId('benchmarkProgressPercent').textContent = `${formatNumber(percent, 1)}%`;
  byId('benchmarkProgressBar').style.width = `${percent}%`;
  byId('benchmarkProgressMeta').textContent = benchmarkProgressMeta(progress);
  root.dataset.state = progress.stage === 'failed' || progress.stage === 'blocked'
    ? 'bad'
    : progress.stage === 'done'
      ? 'good'
      : progress.running
        ? 'running'
        : 'neutral';
  if (progress.running) {
    setBadge('benchmarkState', '跑分中', 'warn');
    byId('benchmarkStatus').textContent = benchmarkProgressMeta(progress);
  } else if (progress.stage === 'failed' || progress.stage === 'blocked') {
    setBadge('benchmarkState', '跑分失败', 'bad');
    byId('benchmarkStatus').textContent = progress.error || progress.label || '跑分失败';
  }
}

function benchmarkProgressLabel(progress) {
  if (progress.error && (progress.stage === 'failed' || progress.stage === 'blocked')) {
    return progress.label || '跑分失败';
  }
  return progress.label || (
    progress.stage === 'ik'
      ? 'IK 检查'
      : progress.stage === 'planning'
        ? '规划中'
        : progress.stage === 'done'
          ? '跑分完成'
          : '等待跑分'
  );
}

function benchmarkProgressMeta(progress) {
  const parts = [];
  const totalPoints = Number(progress.total_points || 0);
  const ikDone = Number(progress.ik_done || 0);
  const ikOk = Number(progress.ik_ok || 0);
  if (totalPoints) parts.push(`IK ${ikDone}/${totalPoints}，有效 ${ikOk}`);
  const planTotal = Number(progress.plan_total || 0);
  if (planTotal) parts.push(`规划 ${Number(progress.plan_done || 0)}/${planTotal}`);
  if (progress.current_sample_id !== null && progress.current_sample_id !== undefined) {
    parts.push(`点 #${progress.current_sample_id}`);
  }
  if (progress.current_planner_label || progress.current_planner_id) {
    parts.push(progress.current_planner_label || progress.current_planner_id);
  }
  if (Number(progress.elapsed_ms) > 0) {
    parts.push(`已用 ${formatDurationSec(Number(progress.elapsed_ms) / 1000)}`);
  }
  if (progress.running && Number(progress.eta_sec) > 0) {
    parts.push(`预计剩余 ${formatDurationSec(progress.eta_sec)}`);
  }
  if (progress.error) parts.push(progress.error);
  return parts.join(' · ') || '等待跑分';
}

function formatDurationSec(value) {
  const seconds = Number(value);
  if (!Number.isFinite(seconds)) return '--';
  if (seconds < 10) return `${formatNumber(seconds, 1)}s`;
  if (seconds < 90) return `${Math.round(seconds)}s`;
  return `${Math.floor(seconds / 60)}m${Math.round(seconds % 60)}s`;
}

function handleBenchmarkSampleConfigInput() {
  if (document.body.dataset.busy) return;
  state.benchmark.samples = [];
  state.benchmark.result = null;
  state.benchmark.selectedSampleId = null;
  state.benchmark.progress = null;
  scene?.setBenchmarkSamples([]);
  renderBenchmarkQuickStats();
  renderBenchmarkSummary();
  renderBenchmarkSamples();
  renderBenchmarkProgress();
  byId('benchmarkStatus').textContent = '点集参数已修改，重新生成后生效';
}

function handleBenchmarkRuntimeInput() {
  if (state.benchmark.result) {
    state.benchmark.result = null;
    state.benchmark.progress = null;
    renderBenchmarkSummary();
    renderBenchmarkSamples();
    renderBenchmarkProgress();
  }
  renderBenchmarkQuickStats();
  byId('benchmarkStatus').textContent = state.benchmark.samples.length
    ? '规划参数已修改，重新跑分后生效'
    : '等待生成测试点';
}

function handleBenchmarkPlannerChange() {
  selectedBenchmarkPlannerIds();
  if (state.benchmark.result) state.benchmark.result = null;
  state.benchmark.progress = null;
  renderBenchmarkQuickStats();
  renderBenchmarkSummary();
  renderBenchmarkSamples();
  renderBenchmarkProgress();
  byId('benchmarkStatus').textContent = state.benchmark.samples.length
    ? '规划器选择已修改，重新跑分后生效'
    : '等待生成测试点';
}

function currentBenchmarkSamples() {
  return state.benchmark.result?.samples || state.benchmark.samples || [];
}

function renderBenchmarkQuickStats() {
  const root = byId('benchmarkQuickStats');
  if (!root) return;
  const samples = currentBenchmarkSamples();
  const result = state.benchmark.result;
  const requested = requestedBenchmarkPointCount();
  const total = Number(result?.total_points ?? samples.length);
  const valid = Number(result?.valid_points ?? samples.filter(sample => sample.ik?.ok).length);
  const planners = benchmarkPlannerCount();
  root.innerHTML = [
    ['请求点', requested],
    ['已生成', samples.length || 0],
    ['IK 有效', result ? `${valid}/${total}` : '--'],
    ['规划器', planners]
  ].map(([label, value]) => `
    <div class="benchmark-metric"><span>${escapeHtml(label)}</span><b>${escapeHtml(value)}</b></div>
  `).join('');

  if (result) {
    setBadge('benchmarkState', valid ? '结果已生成' : '无有效点', valid ? 'good' : 'bad');
  } else if (samples.length) {
    setBadge('benchmarkState', `已生成 ${samples.length}`, 'good');
  } else {
    setBadge('benchmarkState', '未生成', 'neutral');
  }
}

function renderBenchmarkSummary() {
  const root = byId('benchmarkSummary');
  if (!root) return;
  const result = state.benchmark.result;
  const rows = Object.values(result?.summary || {});
  if (!rows.length) {
    renderEmpty('benchmarkSummary', '跑分后生成规划器成功率结果');
    return;
  }
  const order = new Map((result.planners || []).map((planner, index) => [planner.planner_id, index]));
  rows.sort((a, b) => (order.get(a.planner_id) ?? 999) - (order.get(b.planner_id) ?? 999));
  root.innerHTML = rows.map(row => {
    const rate = Math.max(0, Math.min(1, Number(row.success_rate || 0)));
    const percent = `${Math.round(rate * 100)}%`;
    return `
      <div class="benchmark-score-row">
        <div>
          <strong>${escapeHtml(row.label || row.planner_id)}</strong>
          <small>${escapeHtml(row.planner_id)}</small>
        </div>
        <div class="benchmark-score-meter"><span style="width:${rate * 100}%"></span></div>
        <b>${percent}</b>
        <small>${Number(row.planned_ok || 0)}/${Number(row.valid_points || 0)} · ${formatNumber(row.mean_elapsed_ms, 1)} ms</small>
      </div>
    `;
  }).join('');
}

function renderBenchmarkSamples() {
  const root = byId('benchmarkSamples');
  if (!root) return;
  const samples = currentBenchmarkSamples();
  const result = state.benchmark.result;
  const total = Number(result?.total_points ?? samples.length);
  const valid = Number(result?.valid_points ?? samples.filter(sample => sample.ik?.ok).length);
  const badge = byId('benchmarkSetBadge');
  if (badge) badge.textContent = `${valid}/${total || 0}`;
  scene?.setBenchmarkSamples(samples, state.benchmark.selectedSampleId);
  if (!samples.length) {
    renderEmpty('benchmarkSamples', '暂无测试点');
    return;
  }
  const planRowsBySample = benchmarkPlanRowsBySample(result);
  const plannerCount = result?.planners?.length || benchmarkPlannerCount();
  root.innerHTML = samples.map(sample => {
    const point = [sample.x, sample.y, sample.z].map(value => formatNumber(value, 3)).join(', ');
    const plans = planRowsBySample.get(Number(sample.id)) || [];
    const planOk = plans.filter(row => row.ok).length;
    const ikKnown = Boolean(sample.ik);
    const ikOk = sample.ik?.ok === true;
    const active = String(sample.id) === String(state.benchmark.selectedSampleId);
    const planText = ikKnown && !ikOk ? '未计入' : result ? `${planOk}/${plannerCount}` : '待跑分';
    const scanText = benchmarkIkScanText(sample.ik);
    const planScanText = benchmarkPlanScanText(plans, plannerCount);
    return `
      <div class="benchmark-sample ${active ? 'active' : ''}" data-benchmark-sample="${escapeHtml(sample.id)}">
        <div>
          <strong>#${escapeHtml(sample.id)} ${escapeHtml(BENCHMARK_SOURCE_LABELS[sample.source] || sample.source || '测试点')}</strong>
          <small>X/Y/Z ${escapeHtml(point)}${scanText ? ` · ${escapeHtml(scanText)}` : ''}${planScanText ? ` · ${escapeHtml(planScanText)}` : ''}</small>
        </div>
        <div class="benchmark-sample-state">
          <span data-state="${ikKnown ? ikOk ? 'good' : 'bad' : 'neutral'}">${ikKnown ? ikOk ? 'IK 通过' : 'IK 失败' : '未跑 IK'}</span>
          <span data-state="${result ? planOk ? 'good' : 'bad' : 'neutral'}">规划 ${escapeHtml(planText)}</span>
        </div>
      </div>
    `;
  }).join('');
}

function benchmarkIkScanText(ik) {
  if (!ik) return '';
  const scanned = Number(ik.scanned_count ?? ik.candidate_index);
  const total = Number(ik.candidate_count);
  const mode = ik.mode ? `${ik.mode} ` : '';
  if (Number.isFinite(scanned) && Number.isFinite(total) && total > 1) {
    return `姿态 ${mode}${scanned}/${total}`;
  }
  if (Number.isFinite(scanned) && scanned > 1) {
    return `姿态 ${mode}第 ${scanned} 个`;
  }
  return mode ? `姿态 ${mode}` : '';
}

function benchmarkPlanScanText(plans, plannerCount) {
  if (!plans?.length || plannerCount !== 1) return '';
  const row = plans[0];
  const scanned = Number(row.scanned_count ?? row.candidate_index);
  const total = Number(row.candidate_count);
  if (Number.isFinite(scanned) && Number.isFinite(total) && total > 1) {
    return `规划姿态 ${scanned}/${total}`;
  }
  return '';
}

function benchmarkPlanRowsBySample(result) {
  const rowsBySample = new Map();
  for (const rows of Object.values(result?.results || {})) {
    for (const row of rows || []) {
      const key = Number(row.sample_id);
      if (!rowsBySample.has(key)) rowsBySample.set(key, []);
      rowsBySample.get(key).push(row);
    }
  }
  return rowsBySample;
}

function handleBenchmarkSampleClick(event) {
  const row = event.target.closest('[data-benchmark-sample]');
  if (!row) return;
  const sample = currentBenchmarkSamples().find(item => String(item.id) === String(row.dataset.benchmarkSample));
  if (!sample) return;
  state.benchmark.selectedSampleId = sample.id;
  setPose(sample);
  scene?.setBenchmarkSamples(currentBenchmarkSamples(), state.benchmark.selectedSampleId);
  renderBenchmarkSamples();
  byId('footerMessage').textContent =
    `已选中跑分点 #${sample.id}：${formatNumber(sample.x, 3)}, ${formatNumber(sample.y, 3)}, ${formatNumber(sample.z, 3)}`;
}

async function loadPointLibrary() {
  try {
    const data = await getPoints();
    state.points = data.points || [];
    renderLibrary('pointsList', state.points, 'point');
  } catch (error) {
    renderEmpty('pointsList', error.message);
  }
}

async function loadPresetLibrary() {
  try {
    const data = await getPresets();
    state.presets = data.presets || [];
    renderLibrary('presetsList', state.presets, 'preset');
  } catch (error) {
    renderEmpty('presetsList', error.message);
  }
}

function renderLibrary(rootId, items, type) {
  const root = byId(rootId);
  if (!items.length) {
    renderEmpty(rootId, type === 'point' ? '暂无点位' : '暂无预设');
    return;
  }
  root.innerHTML = items.map((item, index) => `
    <div class="list-item">
      <div>
        <strong>${escapeHtml(item.name || `${type === 'point' ? '点位' : '预设'} ${index + 1}`)}</strong>
        <small>${armLabel(item.arm)} · X ${formatNumber(item.x)} · Y ${formatNumber(item.y)} · Z ${formatNumber(item.z)}</small>
      </div>
      <div class="item-actions">
        <button class="icon-button" type="button" data-library-use="${type}:${index}" aria-label="载入" title="载入"><i data-lucide="corner-down-left"></i></button>
        <button class="icon-button" type="button" data-library-delete="${type}:${index}" aria-label="删除" title="删除"><i data-lucide="trash-2"></i></button>
      </div>
    </div>
  `).join('');
  refreshIcons(root);
}

function renderEmpty(rootId, text) {
  byId(rootId).innerHTML = `<p class="empty-state">${escapeHtml(text)}</p>`;
}

async function savePoint() {
  const name = byId('pointName').value.trim() || new Date().toLocaleTimeString('zh-CN', { hour12: false });
  await runCommand('保存点位', () => apiRequest('/api/points', { ...readPose(), name }));
  byId('pointName').value = '';
  loadPointLibrary();
}

async function savePreset() {
  const name = byId('presetName').value.trim() || new Date().toLocaleTimeString('zh-CN', { hour12: false });
  await runCommand('保存抓取预设', () => apiRequest('/api/presets', {
    ...readGraspPose(),
    name,
    pick_approach: numericValue('pickApproach', 0.1),
    pick_descend: numericValue('pickDescend', 0.05),
    pick_hold: numericValue('pickHold', 1.0),
    pick_lift: numericValue('pickLift', 0.1)
  }));
  byId('presetName').value = '';
  loadPresetLibrary();
}

async function handleLibraryClick(event) {
  const use = event.target.closest('[data-library-use]')?.dataset.libraryUse;
  const remove = event.target.closest('[data-library-delete]')?.dataset.libraryDelete;
  if (!use && !remove) return;
  const [type, indexText] = (use || remove).split(':');
  const index = Number(indexText);
  const collection = type === 'point' ? state.points : state.presets;
  if (use) {
    const item = collection[index];
    if (!item) return;
    if (item.arm) await setArm(item.arm);
    setPose(item);
    if (type === 'preset') applyPresetOptions(item);
    showToast(`已载入${type === 'point' ? '点位' : '预设'}：${item.name || index + 1}`);
    return;
  }
  if (!(await confirmAction(`删除“${collection[index]?.name || index + 1}”？`, '确认删除'))) return;
  await apiRequest(`/api/${type === 'point' ? 'points' : 'presets'}/delete`, { index });
  if (type === 'point') loadPointLibrary();
  else loadPresetLibrary();
}

function applyPresetOptions(item) {
  const mappings = [
    ['pickApproach', 'pick_approach'],
    ['pickDescend', 'pick_descend'],
    ['pickHold', 'pick_hold'],
    ['pickLift', 'pick_lift']
  ];
  for (const [id, key] of mappings) {
    const node = byId(id);
    if (node && Number.isFinite(Number(item[key]))) node.value = item[key];
  }
}

function addSequenceStep() {
  const type = byId('sequenceType').value;
  let step = { type, arm: state.arm };
  if (['approach', 'grasp', 'pick'].includes(type)) step.pose = readGraspPose();
  if (type === 'move') step.pose = readPose();
  if (type === 'approach') step.offset_z = numericValue('pickApproach', 0.1);
  if (type === 'lift') step.offset_z = numericValue('pickLift', 0.1);
  if (type === 'pick') {
    step = {
      ...step,
      approach_height: numericValue('pickApproach', 0.1),
      descend_distance: numericValue('pickDescend', 0.05),
      hold_seconds: numericValue('pickHold', 1.0),
      lift_height: numericValue('pickLift', 0.1)
    };
  }
  if (type === 'wait') step.seconds = 1;
  state.sequence.push(step);
  persistSequence();
  renderSequence();
}

function renderSequence() {
  const root = byId('sequenceList');
  if (!state.sequence.length) {
    renderEmpty('sequenceList', '暂无动作步骤');
    return;
  }
  root.innerHTML = state.sequence.map((step, index) => `
    <div class="list-item">
      <div><strong>${index + 1}. ${ACTION_LABELS[step.type] || step.type}</strong><small>${armLabel(step.arm)}${step.seconds ? ` · ${step.seconds} s` : ''}</small></div>
      <div class="item-actions">
        <button class="icon-button" type="button" data-sequence-up="${index}" aria-label="上移" title="上移"><i data-lucide="arrow-up"></i></button>
        <button class="icon-button" type="button" data-sequence-delete="${index}" aria-label="删除" title="删除"><i data-lucide="trash-2"></i></button>
      </div>
    </div>
  `).join('');
  refreshIcons(root);
}

function handleSequenceClick(event) {
  const up = event.target.closest('[data-sequence-up]')?.dataset.sequenceUp;
  const remove = event.target.closest('[data-sequence-delete]')?.dataset.sequenceDelete;
  if (up !== undefined) {
    const index = Number(up);
    if (index > 0) [state.sequence[index - 1], state.sequence[index]] = [state.sequence[index], state.sequence[index - 1]];
  } else if (remove !== undefined) {
    state.sequence.splice(Number(remove), 1);
  } else {
    return;
  }
  persistSequence();
  renderSequence();
}

function clearSequence() {
  state.sequence = [];
  persistSequence();
  renderSequence();
}

function persistSequence() {
  localStorage.setItem('unoarm.sequence', JSON.stringify(state.sequence));
}

async function executeSequence() {
  if (!state.sequence.length) {
    showToast('请先添加动作步骤', 'warn');
    return;
  }
  await runCommand('执行动作序列', () => apiRequest('/api/sequence', { steps: state.sequence }), {
    confirm: `即将连续执行 ${state.sequence.length} 个动作步骤，请确认双臂工作区安全。`,
    confirmTitle: '确认执行动作序列'
  });
}

async function loadLogs() {
  try {
    const data = await getLogs(200);
    const level = byId('logLevelFilter').value;
    const lines = (data.logs || []).map(log => typeof log === 'string' ? log : JSON.stringify(log));
    const filtered = level === 'all' ? lines : lines.filter(line => line.includes(level));
    byId('logViewer').textContent = filtered.join('\n') || '暂无日志';
  } catch (error) {
    byId('logViewer').textContent = `日志读取失败：${error.message}`;
  }
}

function armLabel(arm = state.arm) {
  return arm === 'left' ? '左臂' : '右臂';
}

function loadSavedSequence() {
  try {
    const saved = JSON.parse(localStorage.getItem('unoarm.sequence') || '[]');
    return Array.isArray(saved) ? saved : [];
  } catch {
    return [];
  }
}
