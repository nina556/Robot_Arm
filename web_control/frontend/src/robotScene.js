import {
  ACESFilmicToneMapping,
  AmbientLight,
  ArrowHelper,
  Box3,
  BoxGeometry,
  BufferAttribute,
  BufferGeometry,
  Color,
  CylinderGeometry,
  DirectionalLight,
  DoubleSide,
  EdgesGeometry,
  Group,
  LineBasicMaterial,
  LineSegments,
  LoadingManager,
  Matrix4,
  Mesh,
  MeshPhysicalMaterial,
  MeshStandardMaterial,
  PCFSoftShadowMap,
  PerspectiveCamera,
  PlaneGeometry,
  Points,
  PointsMaterial,
  Quaternion,
  RingGeometry,
  Scene,
  SRGBColorSpace,
  SphereGeometry,
  Vector3,
  WebGLRenderer
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import URDFLoader from 'urdf-loader';

const MODEL_URL = '/urdf/unoarm.urdf';
const VISION_PART_MODEL_URL = '/models/part.stl';
const VISION_PART_SCALE = 0.001;
// ROS world Z at the wheel contact plane. The viewer displays that plane as Y=0.
const WHEEL_GROUND_WORLD_Z = -0.082951078;
const WORLD_TO_DISPLAY_Z_OFFSET = -WHEEL_GROUND_WORLD_Z;
const JOINT_RENDER_RESPONSE_SEC = 0.035;
const OBJECT_RENDER_RESPONSE_SEC = JOINT_RENDER_RESPONSE_SEC;
const DETACHED_OBJECT_JUMP_M = 0.12;
const DETACHED_OBJECT_CONFIRM_SAMPLES = 3;

export class RobotScene {
  constructor(canvas, callbacks = {}) {
    this.canvas = canvas;
    this.callbacks = callbacks;
    this.scene = new Scene();
    this.scene.background = new Color(0x182327);
    this.camera = new PerspectiveCamera(42, 1, 0.01, 100);
    this.renderer = new WebGLRenderer({ canvas, antialias: true, alpha: false });
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = PCFSoftShadowMap;
    this.renderer.toneMapping = ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.18;
    this.renderer.outputColorSpace = SRGBColorSpace;

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.07;
    this.controls.minDistance = 0.7;
    this.controls.maxDistance = 8;
    this.controls.target.set(0, 0.85 + WORLD_TO_DISPLAY_Z_OFFSET, 0);

    this.robot = null;
    this.robotBounds = null;
    this.jointLimits = new Map();
    this.jointTargets = new Map();
    this.jointDisplayValues = new Map();
    this.targetGroup = new Group();
    this.tcpFrameGroup = new Group();
    this.benchmarkGroup = new Group();
    this.visionGroup = new Group();
    this.visionPartGeometry = null;
    this.visionPartMarker = null;
    this.visionPartPoint = null;
    this.visionPartLoadPromise = null;
    this.mujocoExtraGeometryPromises = {};
    this.cameraMarkerGroup = new Group();
    this.platformGroup = new Group();
    this.workspaceBoundsGroup = new Group();
    this.manualCollisionGroup = new Group();
    this.objectMarkerGroup = new Group();
    this.mujocoSceneGroup = new Group();
    this.mujocoTable = null;
    this.mujocoObject = null;
    this.mujocoObjectState = null;
    this.mujocoObjectTargetPosition = null;
    this.mujocoObjectTargetQuaternion = null;
    this.mujocoObjectPendingJump = null;
    this.cloudPoints = null;
    this.cameraExtrinsic = null;
    this.cloudVisible = true;
    this.frameCounter = 0;
    this.frameWindowStart = performance.now();
    this.lastAnimationTime = this.frameWindowStart;
    this.lastWidth = 0;
    this.lastHeight = 0;

    this.scene.add(
      this.targetGroup,
      this.tcpFrameGroup,
      this.benchmarkGroup,
      this.visionGroup,
      this.cameraMarkerGroup,
      this.platformGroup,
      this.workspaceBoundsGroup,
      this.manualCollisionGroup,
      this.objectMarkerGroup,
      this.mujocoSceneGroup
    );

    this.addLighting();
    this.addGround();
    this.setDefaultView();
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas.parentElement);
    this.loadRobot();
    this.loadVisionPartModel();
    this.animate();
  }

  addLighting() {
    const key = new DirectionalLight(0xffffff, 1.45);
    key.position.set(3.5, 5.5, 4);
    key.castShadow = true;
    key.shadow.mapSize.setScalar(2048);
    key.shadow.camera.near = 0.1;
    key.shadow.camera.far = 15;
    key.shadow.camera.left = -3;
    key.shadow.camera.right = 3;
    key.shadow.camera.top = 3;
    key.shadow.camera.bottom = -3;
    this.scene.add(key);

    const fill = new DirectionalLight(0xb9dded, 0.62);
    fill.position.set(-4, 3, -2.5);
    this.scene.add(fill);

    const rim = new DirectionalLight(0xffddb0, 0.34);
    rim.position.set(0, 2, -4);
    this.scene.add(rim);

    this.scene.add(new AmbientLight(0xffffff, 0.5));
  }

  addGround() {
    const ground = new Mesh(
      new PlaneGeometry(12, 12),
      new MeshPhysicalMaterial({
        color: 0x4b5857,
        roughness: 0.82,
        metalness: 0.08,
        clearcoat: 0.08,
        side: DoubleSide
      })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.002;
    ground.receiveShadow = true;
    this.scene.add(ground);

    const grid = this.createGrid();
    grid.position.y = 0.001;
    this.scene.add(grid);
  }

  createGrid() {
    const root = new Group();
    const size = 8;
    const divisions = 40;
    const step = size / divisions;
    const half = size / 2;
    const minorMaterial = new LineBasicMaterial({
      color: 0x344347,
      transparent: true,
      opacity: 0.68
    });
    const majorMaterial = new LineBasicMaterial({
      color: 0x526266,
      transparent: true,
      opacity: 0.72
    });

    for (let index = 0; index <= divisions; index += 1) {
      const position = -half + index * step;
      const geometryA = new BufferGeometry().setFromPoints([
        new Vector3(-half, 0, position),
        new Vector3(half, 0, position)
      ]);
      const geometryB = new BufferGeometry().setFromPoints([
        new Vector3(position, 0, -half),
        new Vector3(position, 0, half)
      ]);
      const material = index % 5 === 0 ? majorMaterial : minorMaterial;
      root.add(new LineSegments(geometryA, material), new LineSegments(geometryB, material));
    }

    return root;
  }

  loadRobot() {
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);
    let loadedRobot = null;

    manager.onProgress = (_url, loaded, total) => {
      this.callbacks.onLoadProgress?.(loaded, total);
    };

    manager.onLoad = () => {
      if (!loadedRobot) {
        return;
      }
      this.robot = loadedRobot;
      this.scene.add(loadedRobot);
      loadedRobot.updateMatrixWorld(true);
      this.robotBounds = new Box3().setFromObject(loadedRobot);
      this.fitView();
      this.callbacks.onLoaded?.(loadedRobot);
    };

    loader.load(
      MODEL_URL,
      robot => {
        loadedRobot = robot;
        robot.name = 'UnoArmURDF';
        robot.rotation.set(0, 0, 0);
        robot.scale.setScalar(1);
        robot.position.set(0, WORLD_TO_DISPLAY_Z_OFFSET, 0);
        robot.traverse(child => {
          child.castShadow = true;
          child.receiveShadow = true;
          if (child.material) {
            child.material.side = DoubleSide;
            if ('roughness' in child.material) {
              child.material.roughness = 0.58;
            }
            child.material.needsUpdate = true;
          }
        });
      },
      undefined,
      error => {
        console.error('Failed to load UnoArm URDF', error);
        this.callbacks.onLoadError?.(error);
      }
    );
  }

  applyJointState(jointState) {
    if (!this.robot || !jointState?.name || !jointState?.position) {
      return 0;
    }

    let applied = 0;
    jointState.name.forEach((name, index) => {
      const joint = this.robot.joints?.[name];
      let value = Number(jointState.position[index]);
      if (!joint || !Number.isFinite(value)) {
        return;
      }
      const limit = this.jointLimits.get(name);
      if (limit) {
        if (joint.limit) {
          joint.limit.lower = limit.lower;
          joint.limit.upper = limit.upper;
        }
        value = Math.min(limit.upper, Math.max(limit.lower, value));
      }
      this.jointTargets.set(name, value);
      if (!this.jointDisplayValues.has(name)) {
        this.jointDisplayValues.set(name, value);
        joint.setJointValue(value);
      }
      applied += 1;
    });
    return applied;
  }

  updateRenderedRobot(deltaSec) {
    if (!this.robot || this.jointTargets.size === 0) {
      return;
    }
    const alpha = 1 - Math.exp(-Math.max(0, deltaSec) / JOINT_RENDER_RESPONSE_SEC);
    let changed = false;
    for (const [name, target] of this.jointTargets) {
      const joint = this.robot.joints?.[name];
      if (!joint) continue;
      const current = this.jointDisplayValues.get(name) ?? target;
      let delta = target - current;
      if (joint.jointType === 'continuous') {
        delta = Math.atan2(Math.sin(delta), Math.cos(delta));
      }
      const next = Math.abs(delta) < 1e-5 ? target : current + delta * alpha;
      if (Math.abs(next - current) > 1e-7) changed = true;
      this.jointDisplayValues.set(name, next);
      joint.setJointValue(next);
    }
    if (changed) {
      this.robot.updateMatrixWorld(true);
    }
  }

  setJointLimits(limitsByName = {}) {
    this.jointLimits = new Map(Object.entries(limitsByName));
  }

  loadVisionPartModel() {
    if (this.visionPartLoadPromise) {
      return this.visionPartLoadPromise;
    }

    const loader = new STLLoader();
    this.visionPartLoadPromise = new Promise(resolve => {
      loader.load(
        VISION_PART_MODEL_URL,
        geometry => {
          geometry.computeVertexNormals();
          geometry.computeBoundingBox();
          const center = new Vector3();
          geometry.boundingBox?.getCenter(center);
          geometry.translate(-center.x, -center.y, -center.z);
          geometry.computeBoundingBox();
          this.visionPartGeometry = geometry;
          const marker = this.ensureVisionPartMarker();
          this.ensureMujocoObject();
          if (marker && this.visionPartPoint) {
            marker.position.copy(this.visionPartPoint);
            marker.visible = true;
          }
          resolve(geometry);
        },
        undefined,
        error => {
          this.callbacks.onStatus?.(`零件模型加载失败: ${error?.message || error}`);
          resolve(null);
        }
      );
    });

    return this.visionPartLoadPromise;
  }

  ensureVisionPartMarker() {
    if (!this.visionPartGeometry) {
      return null;
    }
    if (this.visionPartMarker) {
      return this.visionPartMarker;
    }

    const marker = new Group();
    const model = new Group();
    const part = new Mesh(
      this.visionPartGeometry,
      new MeshStandardMaterial({
        color: 0xf0b64f,
        emissive: 0x3f2b0c,
        roughness: 0.42,
        metalness: 0.08
      })
    );
    part.castShadow = true;
    part.receiveShadow = true;

    const outline = new LineSegments(
      new EdgesGeometry(this.visionPartGeometry, 35),
      new LineBasicMaterial({
        color: 0xffd98b,
        transparent: true,
        opacity: 0.55
      })
    );

    model.add(part, outline);
    model.scale.setScalar(VISION_PART_SCALE);
    // The STL follows the usual CAD/ROS Z-up convention; Three.js displays Y-up.
    model.rotation.x = -Math.PI / 2;
    marker.add(model);
    marker.visible = Boolean(this.visionPartPoint);
    this.visionPartMarker = marker;
    this.visionGroup.add(marker);
    return marker;
  }

  setTargetPose(pose) {
    clearGroup(this.targetGroup);
    if (!pose || !isFinitePoint([pose.x, pose.y, pose.z])) {
      return;
    }

    const position = rosToThree([pose.x, pose.y, pose.z]);
    const ring = new Mesh(
      new RingGeometry(0.035, 0.043, 40),
      new MeshStandardMaterial({
        color: 0x38d2bd,
        emissive: 0x0b4a42,
        side: DoubleSide
      })
    );
    ring.position.copy(position);
    ring.quaternion.setFromUnitVectors(new Vector3(0, 0, 1), new Vector3(0, 1, 0));
    this.targetGroup.add(ring);

    const orientation = rpyDirection(pose.roll, pose.pitch, pose.yaw);
    const direction = rosDirectionToThree(orientation).normalize();
    this.targetGroup.add(new ArrowHelper(direction, position, 0.16, 0x38d2bd, 0.035, 0.02));
  }

  setBenchmarkSamples(samples = [], selectedId = null) {
    clearGroup(this.benchmarkGroup);
    for (const sample of (samples || []).slice(0, 120)) {
      if (!isFinitePoint([sample.x, sample.y, sample.z])) {
        continue;
      }
      const selected = String(sample.id) === String(selectedId);
      const failedIk = sample.ik && sample.ik.ok !== true;
      const color = selected
        ? 0x43c486
        : failedIk
          ? 0xee6262
          : benchmarkSampleColor(sample.source);
      const position = rosToThree([sample.x, sample.y, sample.z]);
      const marker = new Mesh(
        new SphereGeometry(selected ? 0.023 : 0.016, 16, 10),
        new MeshStandardMaterial({
          color,
          emissive: color,
          emissiveIntensity: selected ? 0.28 : 0.12,
          transparent: true,
          opacity: failedIk ? 0.68 : 0.92,
          depthWrite: false
        })
      );
      marker.position.copy(position);
      this.benchmarkGroup.add(marker);

      const direction = rosDirectionToThree(rpyDirection(sample.roll, sample.pitch, sample.yaw)).normalize();
      this.benchmarkGroup.add(new ArrowHelper(
        direction,
        position,
        selected ? 0.11 : 0.07,
        color,
        selected ? 0.026 : 0.018,
        selected ? 0.014 : 0.009
      ));
    }
  }

  setTcpFrame(link) {
    clearGroup(this.tcpFrameGroup);
    if (!link || !isFinitePoint(link.position) || !isFiniteQuaternion(link.orientation)) {
      return;
    }

    const origin = rosToThree(link.position);
    const axes = [
      { axis: [1, 0, 0], color: 0xff4d4d, length: 0.12 },
      { axis: [0, 1, 0], color: 0x43c486, length: 0.105 },
      { axis: [0, 0, 1], color: 0x4da8ff, length: 0.095 }
    ];
    for (const item of axes) {
      const direction = rosDirectionToThree(rotateVectorByQuat(item.axis, link.orientation)).normalize();
      this.tcpFrameGroup.add(new ArrowHelper(direction, origin, item.length, item.color, 0.024, 0.012));
    }
  }

  setVisionTarget(target) {
    const xyz = target?.urdf_xyz_m;
    if (!isFinitePoint(xyz)) {
      this.visionPartPoint = null;
      if (this.visionPartMarker) {
        this.visionPartMarker.visible = false;
      }
      return;
    }

    this.visionPartPoint = rosToThree(xyz);
    const marker = this.ensureVisionPartMarker();
    if (marker) {
      marker.position.copy(this.visionPartPoint);
      marker.visible = true;
    } else {
      this.loadVisionPartModel();
    }
  }

  setObjectMarkers(objects = []) {
    clearGroup(this.objectMarkerGroup);
    for (const object of objects.slice(0, 16)) {
      const xyz = object.center_camera_xyz_m;
      if (!isFinitePoint(xyz) || !this.cameraExtrinsic) {
        continue;
      }
      const point = rosToThree(cameraPointToRos(xyz, this.cameraExtrinsic));
      const marker = new Mesh(
        new SphereGeometry(0.014, 16, 10),
        new MeshStandardMaterial({ color: 0x67c7ed, emissive: 0x173c4b })
      );
      marker.position.copy(point);
      this.objectMarkerGroup.add(marker);
    }
  }

  setMujocoScene(data) {
    const table = data?.table;
    const object = data?.object;
    this.mujocoObjectState = object || null;
    if (isFinitePoint(table?.center) && isFinitePoint(table?.size)) {
      const displaySize = [
        Number(table.size[0]),
        Number(table.size[2]),
        Number(table.size[1])
      ];
      if (!this.mujocoTable) {
        this.mujocoTable = new Mesh(
          new BoxGeometry(...displaySize),
          new MeshStandardMaterial({
            color: table.color || 0x252d2f,
            roughness: 0.9,
            metalness: 0.04
          })
        );
        this.mujocoTable.userData.sceneSize = displaySize;
        this.mujocoSceneGroup.add(this.mujocoTable);
      } else if (!sameSize(this.mujocoTable.userData.sceneSize, displaySize)) {
        this.mujocoTable.geometry.dispose();
        this.mujocoTable.geometry = new BoxGeometry(...displaySize);
        this.mujocoTable.userData.sceneSize = displaySize;
      }
      this.mujocoTable.receiveShadow = true;
      this.mujocoTable.castShadow = true;
      this.mujocoTable.position.copy(rosToThree(table.center));
    }
    this.ensureMujocoObject();
    if (isFinitePoint(object?.position)) {
      const nextPosition = rosToThree(object.position);
      const attached = Boolean(object?.attached);
      const currentTarget = this.mujocoObjectTargetPosition;
      let acceptSample = true;
      const isDetachedJump = (
        !attached
        && currentTarget
        && currentTarget.distanceTo(nextPosition) > DETACHED_OBJECT_JUMP_M
      );
      if (isDetachedJump) {
        const pending = this.mujocoObjectPendingJump;
        if (pending && pending.position.distanceTo(nextPosition) < 0.02) {
          pending.samples += 1;
        } else {
          this.mujocoObjectPendingJump = { position: nextPosition.clone(), samples: 1 };
        }
        if (this.mujocoObjectPendingJump.samples >= DETACHED_OBJECT_CONFIRM_SAMPLES) {
          this.mujocoObjectTargetPosition = nextPosition;
          this.mujocoObjectPendingJump = null;
        } else {
          acceptSample = false;
        }
      } else {
        this.mujocoObjectTargetPosition = nextPosition;
        this.mujocoObjectPendingJump = null;
      }
      if (acceptSample && !currentTarget && this.mujocoObject) {
        this.mujocoObject.position.copy(nextPosition);
      }
      const wxyz = object.orientation_wxyz;
      if (acceptSample && isFiniteQuaternion(wxyz)) {
        const hadTargetQuaternion = Boolean(this.mujocoObjectTargetQuaternion);
        const xyzw = [wxyz[1], wxyz[2], wxyz[3], wxyz[0]];
        const xAxis = rosDirectionToThree(rotateVectorByQuat([1, 0, 0], xyzw)).normalize();
        const yAxis = rosDirectionToThree(rotateVectorByQuat([0, 0, 1], xyzw)).normalize();
        const zAxis = rosDirectionToThree(rotateVectorByQuat([0, 1, 0], xyzw)).multiplyScalar(-1).normalize();
        this.mujocoObjectTargetQuaternion = new Quaternion().setFromRotationMatrix(
          new Matrix4().makeBasis(xAxis, yAxis, zAxis)
        );
        if (this.mujocoObject && !hadTargetQuaternion) {
          this.mujocoObject.quaternion.copy(this.mujocoObjectTargetQuaternion);
        }
      }
    }

    // 渲染额外物体 (定子槽/圆柱)
    const extras = data?.extras || [];
    if (!this.mujocoExtras) this.mujocoExtras = {};
    for (const ext of extras) {
      if (!ext.name || !isFinitePoint(ext.position) || !ext.size) continue;
      const isSlot = ext.type === 'slot' && ext.model_url;
      const isBox = ext.type === 'box';
      const displaySize = isBox
        ? ext.size.slice(0, 3).map(Number)
        : [Number(ext.size[0]), Number(ext.size[1])];
      const makeGeometry = () => isBox
        ? new BoxGeometry(displaySize[0], displaySize[2], displaySize[1])
        : new CylinderGeometry(displaySize[0], displaySize[0], displaySize[1], 24);
      if (!this.mujocoExtras[ext.name] && isSlot) {
        const placeholder = new Group();
        placeholder.userData.sceneSize = displaySize;
        this.mujocoSceneGroup.add(placeholder);
        this.mujocoExtras[ext.name] = placeholder;
        new STLLoader().load(ext.model_url, geometry => {
          geometry.computeVertexNormals();
          geometry.computeBoundingBox();
          const center = new Vector3();
          geometry.boundingBox?.getCenter(center);
          geometry.translate(-center.x, -center.y, -center.z);
          const mesh = new Mesh(
            geometry,
            new MeshStandardMaterial({
              color: ext.color || '#b8bdc7',
              roughness: 0.55,
              metalness: 0.18,
            })
          );
          mesh.scale.setScalar(Number(ext.model_scale) || 0.001);
          // 槽口朝向桌面右侧的机械臂。
          mesh.rotation.y = Math.PI;
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          placeholder.add(mesh);
        }, undefined, error => {
          this.callbacks.onStatus?.(`槽模型加载失败: ${error?.message || error}`);
        });
      } else if (!this.mujocoExtras[ext.name]) {
        const mesh = new Mesh(
          makeGeometry(),
          new MeshStandardMaterial({
            color: ext.color || "#cccccc",
            roughness: 0.6,
            metalness: 0.05,
          })
        );
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh.userData.sceneSize = displaySize;
        this.mujocoSceneGroup.add(mesh);
        this.mujocoExtras[ext.name] = mesh;
      } else if (!isSlot && !sameSize(this.mujocoExtras[ext.name].userData.sceneSize, displaySize)) {
        this.mujocoExtras[ext.name].geometry.dispose();
        this.mujocoExtras[ext.name].geometry = makeGeometry();
        this.mujocoExtras[ext.name].userData.sceneSize = displaySize;
      }
      this.mujocoExtras[ext.name].position.copy(rosToThree(ext.position));
    }
    // 移除不再存在的 extras
    const validNames = new Set(extras.map(e => e.name));
    for (const [name, mesh] of Object.entries(this.mujocoExtras)) {
      if (!validNames.has(name)) {
        this.mujocoSceneGroup.remove(mesh);
        mesh.traverse?.(child => {
          child.geometry?.dispose?.();
          child.material?.dispose?.();
        });
        delete this.mujocoExtras[name];
      }
    }
  }

  ensureMujocoObject() {
    if (this.mujocoObject || !this.visionPartGeometry) {
      return this.mujocoObject;
    }
    const object = this.mujocoObjectState || {};
    const group = new Group();
    const part = new Mesh(
      this.visionPartGeometry,
      new MeshStandardMaterial({
        color: object.color || 0xc62828,
        roughness: 0.46,
        metalness: 0.48
      })
    );
    part.scale.setScalar(VISION_PART_SCALE);
    part.rotation.x = -Math.PI / 2;
    part.castShadow = true;
    part.receiveShadow = true;
    group.add(part);
    this.mujocoObject = group;
    this.mujocoSceneGroup.add(group);
    if (this.mujocoObjectTargetPosition) {
      group.position.copy(this.mujocoObjectTargetPosition);
    }
    if (this.mujocoObjectTargetQuaternion) {
      group.quaternion.copy(this.mujocoObjectTargetQuaternion);
    }
    return group;
  }

  updateRenderedMujocoObject(deltaSec) {
    if (!this.mujocoObject) return;
    const alpha = 1 - Math.exp(-Math.max(0, deltaSec) / OBJECT_RENDER_RESPONSE_SEC);
    if (this.mujocoObjectTargetPosition) {
      this.mujocoObject.position.lerp(this.mujocoObjectTargetPosition, alpha);
    }
    if (this.mujocoObjectTargetQuaternion) {
      this.mujocoObject.quaternion.slerp(this.mujocoObjectTargetQuaternion, alpha);
    }
  }

  setCameraExtrinsic(extrinsic) {
    this.cameraExtrinsic = extrinsic ? { ...extrinsic } : null;
    clearGroup(this.cameraMarkerGroup);
    if (!extrinsic || !isFinitePoint([extrinsic.x, extrinsic.y, extrinsic.z])) {
      return;
    }

    const origin = rosToThree([extrinsic.x, extrinsic.y, extrinsic.z]);
    const camX = cameraAxisToThree(extrinsic, [1, 0, 0]);
    const camY = cameraAxisToThree(extrinsic, [0, 1, 0]);
    const camZ = cameraAxisToThree(extrinsic, [0, 0, 1]);
    const basis = new Matrix4().makeBasis(camX, camY, camZ);
    const marker = new Group();
    marker.position.copy(origin);
    marker.quaternion.setFromRotationMatrix(basis);

    const body = new Mesh(
      new BoxGeometry(0.124, 0.029, 0.026),
      new MeshStandardMaterial({
        color: 0x9aa8ad,
        roughness: 0.52,
        metalness: 0.16,
        transparent: true,
        opacity: 0.45
      })
    );
    marker.add(body);
    this.cameraMarkerGroup.add(marker);
    this.cameraMarkerGroup.add(new ArrowHelper(camZ, origin, 0.18, 0x67c7ed, 0.03, 0.018));
  }

  setPlatformObstacle(obstacle) {
    clearGroup(this.platformGroup);
    const source = obstacle?.platform_obstacle ?? obstacle;
    const entries = Array.isArray(source)
      ? source
      : Array.isArray(source?.platform_obstacles)
        ? source.platform_obstacles
        : source
          ? [source]
          : [];

    for (const entry of entries) {
      this.addCollisionBox(this.platformGroup, entry, {
        fill: 0xe35f63,
        edge: 0xee777a,
        opacity: 0.14
      });
    }
  }

  setManualCollisionBoxes(boxes = []) {
    clearGroup(this.manualCollisionGroup);
    for (const entry of boxes || []) {
      this.addCollisionBox(this.manualCollisionGroup, entry, {
        fill: entry.enabled === false ? 0x7b8589 : 0xe8b04d,
        edge: entry.enabled === false ? 0x9aa8ad : 0xffc75f,
        opacity: entry.enabled === false ? 0.08 : 0.18
      });
    }
  }

  setWorkspaceBounds(bounds) {
    clearGroup(this.workspaceBoundsGroup);
    if (!bounds?.enabled || !isFinitePoint(bounds.min) || !isFinitePoint(bounds.max)) {
      return;
    }
    const min = bounds.min.map(Number);
    const max = bounds.max.map(Number);
    const dimensions = max.map((value, index) => value - min[index]);
    if (dimensions.some(value => !Number.isFinite(value) || value <= 0)) {
      return;
    }
    const center = min.map((value, index) => (value + max[index]) / 2);
    this.addCollisionBox(this.workspaceBoundsGroup, {
      center,
      dimensions,
      rpy: [0, 0, 0]
    }, {
      fill: 0x4da8da,
      edge: 0x67c7ed,
      opacity: 0.06
    });
  }

  addCollisionBox(group, entry, palette) {
    if (!isFinitePoint(entry?.center) || !isFinitePoint(entry?.dimensions)) {
      return;
    }
    const [sx, sy, sz] = entry.dimensions.map(Number);
    const geometry = new BoxGeometry(sx, sz, sy);
    const fill = new Mesh(
      geometry,
      new MeshStandardMaterial({
        color: palette.fill,
        transparent: true,
        opacity: palette.opacity,
        depthWrite: false
      })
    );
    applyRosBoxPose(fill, entry);
    const edges = new LineSegments(
      new EdgesGeometry(geometry),
      new LineBasicMaterial({ color: palette.edge, transparent: true, opacity: 0.9 })
    );
    applyRosBoxPose(edges, entry);
    group.add(fill, edges);
  }

  async loadScenePointCloud() {
    if (!this.cloudVisible || !this.cameraExtrinsic) {
      return null;
    }

    const response = await fetch('/api/scene_pointcloud.bin', { cache: 'no-store' });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    const buffer = await response.arrayBuffer();
    if (buffer.byteLength < 8) {
      throw new Error('点云数据为空');
    }

    const view = new DataView(buffer);
    const sequence = view.getUint32(0, true);
    const count = view.getUint32(4, true);
    const requiredBytes = 8 + count * 6 * 4;
    if (!count || buffer.byteLength < requiredBytes) {
      throw new Error('点云数据格式无效');
    }

    const floats = new Float32Array(buffer, 8, count * 6);
    const positions = new Float32Array(count * 3);
    const colors = new Float32Array(count * 3);

    for (let index = 0; index < count; index += 1) {
      const sourceOffset = index * 6;
      const targetOffset = index * 3;
      const ros = cameraPointToRos(
        [
          floats[sourceOffset],
          floats[sourceOffset + 1],
          floats[sourceOffset + 2]
        ],
        this.cameraExtrinsic
      );
      const point = rosToThree(ros);
      positions[targetOffset] = point.x;
      positions[targetOffset + 1] = point.y;
      positions[targetOffset + 2] = point.z;
      colors[targetOffset] = floats[sourceOffset + 3];
      colors[targetOffset + 1] = floats[sourceOffset + 4];
      colors[targetOffset + 2] = floats[sourceOffset + 5];
    }

    const geometry = new BufferGeometry();
    geometry.setAttribute('position', new BufferAttribute(positions, 3));
    geometry.setAttribute('color', new BufferAttribute(colors, 3));
    const material = new PointsMaterial({
      size: 0.005,
      vertexColors: true,
      transparent: true,
      opacity: 0.82,
      depthWrite: false
    });
    const points = new Points(geometry, material);
    points.frustumCulled = false;

    if (this.cloudPoints) {
      this.scene.remove(this.cloudPoints);
      disposeObject(this.cloudPoints);
    }
    this.cloudPoints = points;
    this.scene.add(points);

    return { sequence, count };
  }

  setCloudVisible(visible) {
    this.cloudVisible = Boolean(visible);
    if (this.cloudPoints) {
      this.cloudPoints.visible = this.cloudVisible;
    }
  }

  setDefaultView() {
    this.camera.position.set(2.8, 1.65, 3.7);
    this.controls.target.set(0, 0.85 + WORLD_TO_DISPLAY_Z_OFFSET, 0);
    this.controls.update();
  }

  frontView() {
    const target = this.controls.target.clone();
    this.camera.position.set(target.x, target.y + 0.15, target.z + 3.9);
    this.camera.up.set(0, 1, 0);
    this.controls.update();
  }

  fitView() {
    if (!this.robot) {
      this.setDefaultView();
      return;
    }

    this.robot.updateMatrixWorld(true);
    const box = new Box3().setFromObject(this.robot);
    const center = box.getCenter(new Vector3());
    const size = box.getSize(new Vector3());
    const maxDimension = Math.max(size.x, size.y, size.z, 1);
    const distance = maxDimension / (2 * Math.tan((this.camera.fov * Math.PI) / 360));
    this.controls.target.copy(center);
    this.camera.position.set(center.x + distance * 0.62, center.y + distance * 0.18, center.z + distance * 1.08);
    this.camera.near = Math.max(0.01, distance / 100);
    this.camera.far = Math.max(20, distance * 10);
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  resetView() {
    this.setDefaultView();
  }

  resize() {
    const parent = this.canvas.parentElement;
    if (!parent) {
      return;
    }
    const width = Math.max(1, parent.clientWidth);
    const height = Math.max(1, parent.clientHeight);
    if (width === this.lastWidth && height === this.lastHeight) {
      return;
    }
    this.lastWidth = width;
    this.lastHeight = height;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  animate = () => {
    this.animationFrame = requestAnimationFrame(this.animate);
    const now = performance.now();
    const deltaSec = Math.min(0.1, Math.max(0, (now - this.lastAnimationTime) / 1000));
    this.lastAnimationTime = now;
    this.updateRenderedRobot(deltaSec);
    this.updateRenderedMujocoObject(deltaSec);
    this.resize();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);

    this.frameCounter += 1;
    if (now - this.frameWindowStart >= 1000) {
      const fps = Math.round((this.frameCounter * 1000) / (now - this.frameWindowStart));
      this.callbacks.onFps?.(fps);
      this.frameCounter = 0;
      this.frameWindowStart = now;
    }
  };

  dispose() {
    cancelAnimationFrame(this.animationFrame);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.renderer.dispose();
  }
}

export function rosToThree(point) {
  return new Vector3(
    Number(point[0]),
    Number(point[2]) + WORLD_TO_DISPLAY_Z_OFFSET,
    -Number(point[1])
  );
}

function rosDirectionToThree(direction) {
  return new Vector3(
    Number(direction[0]),
    Number(direction[2]),
    -Number(direction[1])
  );
}

function applyRosBoxPose(object, entry) {
  object.position.copy(rosToThree(entry.center));
  object.quaternion.setFromRotationMatrix(rosBoxRotationToThree(entry.rpy || [0, 0, 0]));
}

function rosBoxRotationToThree(rpy) {
  const matrix = rpyMatrix(Number(rpy[0] || 0), Number(rpy[1] || 0), Number(rpy[2] || 0));
  const rosX = [matrix[0][0], matrix[1][0], matrix[2][0]];
  const rosY = [matrix[0][1], matrix[1][1], matrix[2][1]];
  const rosZ = [matrix[0][2], matrix[1][2], matrix[2][2]];
  const xAxis = rosDirectionToThree(rosX).normalize();
  const yAxis = rosDirectionToThree(rosZ).normalize();
  const zAxis = rosDirectionToThree(rosY).multiplyScalar(-1).normalize();
  return new Matrix4().makeBasis(xAxis, yAxis, zAxis);
}

function rpyMatrix(roll, pitch, yaw) {
  const sr = Math.sin(roll);
  const cr = Math.cos(roll);
  const sp = Math.sin(pitch);
  const cp = Math.cos(pitch);
  const sy = Math.sin(yaw);
  const cy = Math.cos(yaw);
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr]
  ];
}

function rpyDirection(roll = 0, pitch = 0, yaw = 0) {
  const cr = Math.cos(Number(roll));
  const sr = Math.sin(Number(roll));
  const cp = Math.cos(Number(pitch));
  const sp = Math.sin(Number(pitch));
  const cy = Math.cos(Number(yaw));
  const sy = Math.sin(Number(yaw));
  return [cy * cp, sy * cp, -sp];
}

function benchmarkSampleColor(source) {
  if (source === 'box_top') return 0x4da8da;
  if (source === 'edge') return 0xe8b04d;
  if (source === 'random') return 0xd7e1e4;
  return 0x9aa8ad;
}

function cameraRpyMatrix(pose) {
  const cr = Math.cos(Number(pose.roll));
  const sr = Math.sin(Number(pose.roll));
  const cp = Math.cos(Number(pose.pitch));
  const sp = Math.sin(Number(pose.pitch));
  const cy = Math.cos(Number(pose.yaw));
  const sy = Math.sin(Number(pose.yaw));
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr]
  ];
}

function cameraAxisToThree(pose, axis) {
  const matrix = cameraRpyMatrix(pose);
  const rosAxis = [
    matrix[0][0] * axis[0] + matrix[0][1] * axis[1] + matrix[0][2] * axis[2],
    matrix[1][0] * axis[0] + matrix[1][1] * axis[1] + matrix[1][2] * axis[2],
    matrix[2][0] * axis[0] + matrix[2][1] * axis[1] + matrix[2][2] * axis[2]
  ];
  return rosDirectionToThree(rosAxis).normalize();
}

function cameraPointToRos(point, extrinsic) {
  const matrix = cameraRpyMatrix(extrinsic);
  return [
    Number(extrinsic.x) + matrix[0][0] * point[0] + matrix[0][1] * point[1] + matrix[0][2] * point[2],
    Number(extrinsic.y) + matrix[1][0] * point[0] + matrix[1][1] * point[1] + matrix[1][2] * point[2],
    Number(extrinsic.z) + matrix[2][0] * point[0] + matrix[2][1] * point[1] + matrix[2][2] * point[2]
  ];
}

function isFinitePoint(point) {
  return Array.isArray(point) && point.length === 3 && point.every(value => Number.isFinite(Number(value)));
}

function isFiniteQuaternion(quat) {
  return Array.isArray(quat) && quat.length === 4 && quat.every(value => Number.isFinite(Number(value)));
}

function sameSize(previous, next) {
  return Array.isArray(previous)
    && previous.length === next.length
    && previous.every((value, index) => Math.abs(Number(value) - Number(next[index])) < 1e-9);
}

function rotateVectorByQuat(vector, quat) {
  const [vx, vy, vz] = vector.map(Number);
  const [qx, qy, qz, qw] = quat.map(Number);
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);
  return [
    vx + qw * tx + (qy * tz - qz * ty),
    vy + qw * ty + (qz * tx - qx * tz),
    vz + qw * tz + (qx * ty - qy * tx)
  ];
}

function clearGroup(group) {
  while (group.children.length) {
    const child = group.children.pop();
    disposeObject(child);
  }
}

function disposeObject(object) {
  object?.traverse?.(child => {
    child.geometry?.dispose?.();
    if (Array.isArray(child.material)) {
      child.material.forEach(material => material?.dispose?.());
    } else {
      child.material?.dispose?.();
    }
  });
}
