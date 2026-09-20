// robopy VR teleoperation page.
//
// Everything kinematic happens on the server. This page:
//   * opens /ws/teleop and streams the headset + controller poses (WebXR,
//     reference space `local-floor`), receiving joint state back;
//   * opens /ws/camera and shows the newest JPEG on a plane in front of the
//     operator (head-locked by default);
//   * draws the robot "twin" from the poses the server sends, so the operator
//     can see the machine's configuration next to the camera image.
//
// Controls (Meta Quest Touch; the clutch button comes from the server's hello,
// default A / X):
//   A / X (thumb)           clutch: while held, that arm follows the controller
//   trigger                 gripper (only when its travel is configured)
//   both thumbstick clicks  re-centre (headset forward = robot front)
//   B / Y                   start / stop recording on the server
//   page checkboxes         robot twin, its mirror, "image follows head"
//
// The mirror: the twin is drawn again, reflected in a vertical plane a
// chosen distance in front of the robot's head, so the operator -- who stands
// inside the twin -- sees the machine face them as in a mirror, left on the
// left.
import * as THREE from 'three';
import { STLLoader } from 'three/addons/STLLoader.js';
import { OrbitControls } from 'three/addons/OrbitControls.js';

const $ = (sel) => document.querySelector(sel);
const DEG = 180 / Math.PI;

// --------------------------------------------------------------- state
const state = {
  hello: null,           // server greeting
  model: null,           // model description (from hello)
  meshes: [],            // THREE.Object3D per geometry, in the robot group
  lastState: null,       // last {type:"state"} message
  lastPoses: null,       // last geometry poses received
  ws: null, camWs: null,
  xrSession: null,
  preview: false,        // desktop preview: head follows the window camera
  sendHz: 60, lastSendMs: 0,
  sent: 0, received: 0, lastRttMs: null,
  controllers: {},       // index -> {handedness, grip, source}
  buttonsPrev: { left: {}, right: {} },
  cameraLocked: true,
  clutchButton: 'a',     // 'a' (A/X), 'grip' or 'stick'; from hello
  mirrorOn: true, mirrorDistance: 1.5,
  recording: null,       // server's recorder state (from hello / state)
  frames: 0, lastFrameBytes: 0, lastFrameMs: null, camConnected: false,
  connected: false,
};
window.__robopy_vr = state;

// --------------------------------------------------------------- scene
const viewport = $('#viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local-floor');
viewport.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171c);
const camera = new THREE.PerspectiveCamera(70, 1, 0.01, 60);
camera.position.set(0.0, 1.5, 1.2);
scene.add(camera);                            // so head-locked children render
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 1.0, -0.6);
controls.update();

scene.add(new THREE.HemisphereLight(0xdfe6f0, 0x2a2f38, 1.0));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(2, 4, 1.5);
scene.add(sun);
const floor = new THREE.GridHelper(6, 12, 0x3a4150, 0x2a2f38);
scene.add(floor);

// The robot is described in its base frame (Z up, X forward); WebXR is Y up,
// -Z forward. XR = R^T * robot with R = XR_TO_ROBOT from xr_math.py.
const XR_FROM_ROBOT = new THREE.Matrix4().set(
  0, -1, 0, 0,
  0, 0, 1, 0,
  -1, 0, 0, 0,
  0, 0, 0, 1,
);
const robotGroup = new THREE.Group();     // twin placement (offset in robot frame)
robotGroup.matrixAutoUpdate = false;
scene.add(robotGroup);
const tcpFrames = {};                     // side -> AxesHelper inside robotGroup
const mirrorGroup = new THREE.Group();    // the twin reflected in a plane ahead of it
mirrorGroup.matrixAutoUpdate = false;
scene.add(mirrorGroup);
const mirrorMeshes = [];                  // same indices as state.meshes

// Camera image: a plane 1.5 m ahead, sized by the horizontal FOV.
const IMAGE_DISTANCE = 1.5;
const imageTexture = new THREE.Texture();
imageTexture.colorSpace = THREE.SRGBColorSpace;
const imageMaterial = new THREE.MeshBasicMaterial({ map: imageTexture, color: 0x666666, side: THREE.DoubleSide });
const imagePlane = new THREE.Mesh(new THREE.PlaneGeometry(1, 0.75), imageMaterial);
imagePlane.position.set(0, 0, -IMAGE_DISTANCE);
camera.add(imagePlane);
let imageAspect = 4 / 3;

// HUD: a canvas texture under the image.
const hudCanvas = document.createElement('canvas');
hudCanvas.width = 1024; hudCanvas.height = 256;
const hudCtx = hudCanvas.getContext('2d');
const hudTexture = new THREE.CanvasTexture(hudCanvas);
const hudPlane = new THREE.Mesh(
  new THREE.PlaneGeometry(1.2, 0.3),
  new THREE.MeshBasicMaterial({ map: hudTexture, transparent: true, side: THREE.DoubleSide }),
);
hudPlane.position.set(0, -0.62, -IMAGE_DISTANCE);
camera.add(hudPlane);

function sizeImagePlane(fovDeg) {
  const width = 2 * IMAGE_DISTANCE * Math.tan((fovDeg / 2) / DEG);
  imagePlane.geometry.dispose();
  imagePlane.geometry = new THREE.PlaneGeometry(width, width / imageAspect);
  hudPlane.position.y = -(width / imageAspect) / 2 - 0.2;
}

function setImageLocked(locked) {
  state.cameraLocked = locked;
  $('#cam-lock').checked = locked;
  if (locked) {
    if (imagePlane.parent !== camera) {
      scene.remove(imagePlane); scene.remove(hudPlane);
      imagePlane.position.set(0, 0, -IMAGE_DISTANCE); imagePlane.quaternion.identity();
      hudPlane.position.set(0, hudPlane.position.y, -IMAGE_DISTANCE); hudPlane.quaternion.identity();
      camera.add(imagePlane); camera.add(hudPlane);
    }
  } else if (imagePlane.parent === camera) {
    // Freeze where it is now, in world coordinates.
    for (const obj of [imagePlane, hudPlane]) {
      obj.updateWorldMatrix(true, false);
      const m = obj.matrixWorld.clone();
      camera.remove(obj);
      m.decompose(obj.position, obj.quaternion, obj.scale);
      scene.add(obj);
    }
  }
}

function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(1, h);
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

// --------------------------------------------------------------- robot twin
function placeObject(obj, pose) {
  obj.position.set(pose.p[0], pose.p[1], pose.p[2]);
  obj.quaternion.set(pose.q[0], pose.q[1], pose.q[2], pose.q[3]);
}

function applyPoses(poses) {
  state.lastPoses = poses;
  poses.forEach((pose, i) => {
    const obj = state.meshes[i]; if (obj) placeObject(obj, pose);
    const m = mirrorMeshes[i]; if (m) placeObject(m, pose);
  });
}

// Reflection of the twin in the vertical plane x = d (robot axes), d being the
// head anchor's x plus the chosen distance: x' = 2d - x.  A negative-scale
// matrix flips the winding; three.js renders it correctly.
function updateMirror() {
  const anchor = (state.hello && state.hello.robot_anchor_m) || [0, 0, 0];
  const d = anchor[0] + state.mirrorDistance;
  const reflect = new THREE.Matrix4().makeTranslation(2 * d, 0, 0).multiply(new THREE.Matrix4().makeScale(-1, 1, 1));
  mirrorGroup.matrix.copy(robotGroup.matrix).multiply(reflect);
  mirrorGroup.visible = state.mirrorOn;
}

function setTwinOffset(offset) {
  const t = new THREE.Matrix4().makeTranslation(offset[0], offset[1], offset[2]);
  robotGroup.matrix.copy(XR_FROM_ROBOT).multiply(t);
  updateMirror();
}

// Server-computed placement: the robot base at `p` (robot axes from the WebXR
// floor origin) turned by `yaw` about +Z, so the twin's head sits where the
// operator's head was at re-centring and faces their forward.
function setTwinPose(twin) {
  if (!twin) return;
  const t = new THREE.Matrix4().makeTranslation(twin.p[0], twin.p[1], twin.p[2]);
  const r = new THREE.Matrix4().makeRotationZ(twin.yaw);
  robotGroup.matrix.copy(XR_FROM_ROBOT).multiply(t).multiply(r);
  updateMirror();
}

async function loadMeshes(model) {
  const loader = new STLLoader();
  const overlay = $('#overlay');
  let done = 0;
  const total = model.geometries.filter((g) => g.shape.type === 'mesh').length;
  const material = new THREE.MeshStandardMaterial({ color: 0xb8bec8, roughness: 0.6, metalness: 0.15 });
  const attach = (i, obj) => {
    state.meshes[i] = obj;
    const twin = obj.clone();
    mirrorMeshes[i] = twin;
    const pose = state.lastPoses && state.lastPoses[i];
    if (pose) { placeObject(obj, pose); placeObject(twin, pose); }
    robotGroup.add(obj);
    mirrorGroup.add(twin);
  };
  const jobs = model.geometries.map((g, i) => new Promise((resolve) => {
    if (g.shape.type !== 'mesh') {
      let geometry;
      const s = g.shape;
      if (s.type === 'cylinder') geometry = new THREE.CylinderGeometry(s.radius, s.radius, s.length, 32);
      else if (s.type === 'box') geometry = new THREE.BoxGeometry(...s.size);
      else if (s.type === 'sphere') geometry = new THREE.SphereGeometry(s.radius, 24, 16);
      const holder = new THREE.Group();
      if (geometry) {
        const mesh = new THREE.Mesh(geometry, material);
        if (s.type === 'cylinder') mesh.rotation.x = Math.PI / 2;
        holder.add(mesh);
      }
      attach(i, holder); resolve(); return;
    }
    loader.load(g.url, (geometry) => {
      geometry.computeVertexNormals();
      const mesh = new THREE.Mesh(geometry, material);
      mesh.scale.set(...g.scale);
      attach(i, mesh);
      done += 1; overlay.textContent = `loading meshes… ${done}/${total}`;
      resolve();
    }, undefined, () => { attach(i, new THREE.Group()); resolve(); });
  }));
  await Promise.all(jobs);
  if (state.lastPoses) applyPoses(state.lastPoses);
  for (const side of Object.keys(model.tcp_frames || {})) {
    const axes = new THREE.AxesHelper(0.08);
    tcpFrames[side] = axes; robotGroup.add(axes);
  }
  overlay.hidden = true;
}

// --------------------------------------------------------------- teleop socket
function wsUrl(path) {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  return `${proto}://${location.host}${path}`;
}

// The server drops an operator whose socket is silent for teleop_timeout_s
// (5 s by default).  Poses only flow while presenting or previewing, so the
// page keeps the socket alive with a ping every second the rest of the time
// (before Enter VR, while the headset prompts, while the operator reads the
// page) and reconnects on its own if the connection is lost anyway.
const KEEPALIVE_MS = 1000;
const RECONNECT_MS = 1500;
let keepalive = null;

function connectTeleop() {
  const ws = new WebSocket(wsUrl('/ws/teleop'));
  state.ws = ws;
  ws.onopen = () => {
    state.connected = true;
    ws.send(JSON.stringify({ type: 'hello', want_poses: true }));
    setStatus('connected; waiting for hello…');
    if (keepalive) clearInterval(keepalive);
    keepalive = setInterval(() => {
      if (ws.readyState === WebSocket.OPEN && performance.now() - state.lastSendMs > KEEPALIVE_MS) {
        ws.send(JSON.stringify({ type: 'ping' }));
      }
    }, KEEPALIVE_MS);
  };
  ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch (e) { return; }
    state.received += 1;
    if (msg.type === 'hello') onHello(msg);
    else if (msg.type === 'state') onState(msg);
    else if (msg.type === 'recording') { state.recording = msg.recording; renderStatus(); }
    else if (msg.type === 'pong') { /* keepalive answered */ }
    else if (msg.type === 'error') { setStatus(`server error: ${msg.message}`, 'bad'); console.warn(msg.message); }
  };
  ws.onclose = () => {
    state.connected = false;
    if (keepalive) { clearInterval(keepalive); keepalive = null; }
    setStatus('teleop socket closed — the arms hold; reconnecting…', 'bad');
    setTimeout(connectTeleop, RECONNECT_MS);
  };
  ws.onerror = () => setStatus('teleop socket error', 'bad');
}

async function onHello(msg) {
  const first = state.hello === null;
  state.hello = msg;
  state.model = msg.model;
  $('#robot-name').textContent = msg.model ? msg.model.robot : '(no model)';
  const badge = $('#backend-badge');
  badge.textContent = msg.backend === 'hardware' ? 'HARDWARE · the robot will move' : 'SIMULATION · no hardware';
  badge.className = `badge ${msg.backend === 'hardware' ? 'hw' : 'sim'}`;
  $('#head-on').checked = msg.head_enabled;
  $('#arms-on').checked = msg.arms_enabled;
  if (msg.arms && msg.arms.left) $('#pos-scale').value = msg.arms.left.position_scale.toFixed(2);
  sizeImagePlane(msg.camera_fov_deg || 69);
  state.clutchButton = msg.clutch_button || 'a';
  state.recording = msg.recording || null;
  imagePlane.visible = msg.camera_available !== false;
  $('#record').disabled = !msg.recording;
  if (msg.twin_offset_m) setTwinOffset(msg.twin_offset_m);
  else if (msg.twin) setTwinPose(msg.twin);
  else setTwinOffset([0, 0, 1]);   // until the first re-centre places it
  if (first && msg.model) await loadMeshes(msg.model);
  if (first) { connectCamera(); }
  renderStatus();
}

function onState(msg) {
  state.lastState = msg;
  if (msg.t != null) state.lastRttMs = performance.now() - msg.t;
  if (msg.twin) setTwinPose(msg.twin);
  if (msg.recording !== undefined) state.recording = msg.recording;
  if (msg.geometries) applyPoses(msg.geometries);
  if (msg.tcp) for (const [side, pose] of Object.entries(msg.tcp)) { const f = tcpFrames[side]; if (f) placeObject(f, pose); }
  renderStatus();
}

function toggleRecording() { send({ type: 'record', action: 'toggle' }); }

function recordingLine() {
  const r = state.recording;
  if (!r) return 'recording  off (server started without a recording directory)';
  const mmss = (s) => `${Math.floor(s / 60).toString().padStart(2, '0')}:${Math.floor(s % 60).toString().padStart(2, '0')}`;
  if (r.active) return `recording  ● REC ${mmss(r.seconds)}  (${r.frames} frames, view ${state.operatorFrames || 0}; B / Y or the button stops)`;
  const rd = r.render;
  if (rd && rd.status === 'rendering') return `recording  rendering video… ${Math.round(rd.progress * 100)}%`;
  if (rd && rd.status === 'done') return `recording  saved ${rd.outputs.map((p) => p.split('/').pop()).join(', ')} in ${r.directory}`;
  if (rd && rd.status === 'failed') return `recording  render FAILED: ${rd.error}  (log kept: ${r.last_recording})`;
  if (r.last_recording) return `recording  saved ${r.last_recording}`;
  return 'recording  idle (B / Y or the button starts)';
}

function send(obj) {
  if (!state.ws || state.ws.readyState !== WebSocket.OPEN) return false;
  state.ws.send(JSON.stringify(obj));
  state.sent += 1;
  state.lastSendMs = performance.now();
  return true;
}
state.send = send;   // for tests: window.__robopy_vr.send({...})

// --------------------------------------------------------------- camera socket
function connectCamera() {
  const ws = new WebSocket(wsUrl('/ws/camera'));
  ws.binaryType = 'blob';
  state.camWs = ws;
  ws.onopen = () => { state.camConnected = true; };
  ws.onmessage = async (ev) => {
    if (typeof ev.data === 'string') {
      try { const m = JSON.parse(ev.data); if (m.type === 'camera' && m.fov_deg) sizeImagePlane(m.fov_deg); } catch (e) { /* ignore */ }
      return;
    }
    try {
      const bitmap = await createImageBitmap(ev.data);
      const aspect = bitmap.width / bitmap.height;
      if (Math.abs(aspect - imageAspect) > 1e-3) { imageAspect = aspect; sizeImagePlane((state.hello && state.hello.camera_fov_deg) || 69); }
      if (imageTexture.image && imageTexture.image.close) imageTexture.image.close();
      imageTexture.image = bitmap;
      imageTexture.needsUpdate = true;
      imageMaterial.color.set(0xffffff);
      state.frames += 1; state.lastFrameBytes = ev.data.size; state.lastFrameMs = performance.now();
    } catch (e) { console.warn('frame decode failed', e); }
  };
  // A dropped camera socket is reconnected on its own; the picture greys
  // out meanwhile so a stale frame is not mistaken for a live one.
  ws.onclose = () => {
    state.camConnected = false;
    imageMaterial.color.set(0x444444);
    setTimeout(() => { if (state.camWs === ws) connectCamera(); }, RECONNECT_MS);
  };
  ws.onerror = () => {};
}

// --------------------------------------------------------------- WebXR
const xrSupportEl = $('#xr-support');
const enterButton = $('#enter-vr');
if (navigator.xr) {
  navigator.xr.isSessionSupported('immersive-vr').then((ok) => {
    enterButton.disabled = !ok;
    xrSupportEl.textContent = ok ? 'WebXR available' : 'no immersive-vr support on this device';
  }).catch(() => { xrSupportEl.textContent = 'WebXR check failed'; });
} else {
  xrSupportEl.textContent = window.isSecureContext
    ? 'navigator.xr missing (open this page in the headset browser)'
    : 'not a secure context: WebXR needs https:// or localhost';
}

enterButton.addEventListener('click', async () => {
  try {
    const session = await navigator.xr.requestSession('immersive-vr', { optionalFeatures: ['local-floor', 'bounded-floor', 'hand-tracking'] });
    state.xrSession = session;
    state.preview = false;
    session.addEventListener('end', () => { state.xrSession = null; setStatus('XR session ended; the arms hold.', 'warn'); });
    await renderer.xr.setSession(session);
  } catch (err) {
    setStatus(`could not start XR: ${err.message}`, 'bad');
  }
});

for (let i = 0; i < 2; i += 1) {
  const grip = renderer.xr.getControllerGrip(i);
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, 0.10), new THREE.MeshStandardMaterial({ color: 0xff9f43 }));
  body.position.z = 0.03;
  grip.add(body);
  const ray = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -0.3)]),
    new THREE.LineBasicMaterial({ color: 0x4cc2ff }),
  );
  renderer.xr.getController(i).add(ray);
  scene.add(grip); scene.add(renderer.xr.getController(i));
  grip.addEventListener('connected', (e) => { state.controllers[i] = { handedness: e.data.handedness, source: e.data }; });
  grip.addEventListener('disconnected', () => { delete state.controllers[i]; });
}

function xrPose(transform) {
  const p = transform.position, q = transform.orientation;
  return { p: [p.x, p.y, p.z], q: [q.x, q.y, q.z, q.w] };
}

function controllerEntry(frame, refSpace, source) {
  if (!source.gripSpace) return null;
  const pose = frame.getPose(source.gripSpace, refSpace);
  if (!pose) return null;
  const gp = source.gamepad;
  const b = (k) => (gp && gp.buttons[k] ? gp.buttons[k] : null);
  const entry = xrPose(pose.transform);
  entry.trigger = b(0) ? b(0).value : 0;
  // xr-standard gamepad: 0 trigger, 1 squeeze (grip), 3 thumbstick click,
  // 4 A/X, 5 B/Y.
  entry.buttons = { a: !!(b(4) && b(4).pressed), b: !!(b(5) && b(5).pressed), stick: !!(b(3) && b(3).pressed), grip: !!(b(1) && b(1).pressed) };
  entry.clutch = !!entry.buttons[state.clutchButton];
  entry.axes = gp ? [gp.axes[2] || 0, gp.axes[3] || 0] : [0, 0];
  return entry;
}

function handleButtons(side, entry) {
  if (!entry) return;
  const prev = state.buttonsPrev[side];
  const now = entry.buttons;
  if (now.b && !prev.b) toggleRecording();
  state.buttonsPrev[side] = { ...now };
}

function collectAndSend(frame, timeMs) {
  const refSpace = renderer.xr.getReferenceSpace();
  if (!refSpace) return;
  if (timeMs - state.lastSendMs < 1000 / state.sendHz) return;
  state.lastSendMs = timeMs;
  const viewer = frame.getViewerPose(refSpace);
  const msg = { type: 'pose', t: performance.now(), head: viewer ? xrPose(viewer.transform) : null, left: null, right: null };
  const session = renderer.xr.getSession();
  for (const source of session.inputSources) {
    if (source.handedness !== 'left' && source.handedness !== 'right') continue;
    const entry = controllerEntry(frame, refSpace, source);
    msg[source.handedness] = entry;
    handleButtons(source.handedness, entry);
  }
  // Re-centre: both thumbsticks clicked.
  const l = msg.left && msg.left.buttons.stick, r = msg.right && msg.right.buttons.stick;
  if (l && r && !state.recenterHeld) { state.recenterHeld = true; send({ type: 'recenter' }); }
  if (!(l && r)) state.recenterHeld = false;
  send(msg);
}

// Desktop preview: the window camera's orientation drives the head.
function previewPose(timeMs) {
  if (timeMs - state.lastSendMs < 1000 / 30) return;
  state.lastSendMs = timeMs;
  camera.updateMatrixWorld();
  const q = new THREE.Quaternion(); const p = new THREE.Vector3();
  camera.getWorldQuaternion(q); camera.getWorldPosition(p);
  send({ type: 'pose', t: performance.now(), head: { p: [p.x, p.y, p.z], q: [q.x, q.y, q.z, q.w] }, left: null, right: null });
}

// --------------------------------------------------------------- operator view capture
// While the server records, the scene is drawn again from the operator's
// current viewpoint into an offscreen target, encoded as JPEG and sent over
// the teleop socket, so the recording also holds what the headset showed
// (twin, mirror, camera image, HUD).  Ten frames a second is plenty.
const OPERATOR_VIEW_MS = 100;
const OPERATOR_VIEW_W = 640, OPERATOR_VIEW_H = 480;
const spectatorTarget = new THREE.WebGLRenderTarget(OPERATOR_VIEW_W, OPERATOR_VIEW_H);
const spectatorCamera = new THREE.PerspectiveCamera(70, OPERATOR_VIEW_W / OPERATOR_VIEW_H, 0.01, 60);
const spectatorPixels = new Uint8Array(OPERATOR_VIEW_W * OPERATOR_VIEW_H * 4);
const spectatorCanvas = document.createElement('canvas');
spectatorCanvas.width = OPERATOR_VIEW_W; spectatorCanvas.height = OPERATOR_VIEW_H;
const spectatorCtx = spectatorCanvas.getContext('2d');
const spectatorImage = spectatorCtx.createImageData(OPERATOR_VIEW_W, OPERATOR_VIEW_H);
let lastOperatorViewMs = 0;
let operatorViewBusy = false;
state.operatorFrames = 0;

function captureOperatorView(timeMs) {
  if (!(state.recording && state.recording.active)) return;
  if (operatorViewBusy || timeMs - lastOperatorViewMs < OPERATOR_VIEW_MS) return;
  if (!state.ws || state.ws.readyState !== WebSocket.OPEN) return;
  lastOperatorViewMs = timeMs;
  // The operator's eye: `camera` carries the headset pose while presenting
  // (three.js writes the XR pose into it) and the window's camera otherwise.
  camera.updateMatrixWorld();
  camera.matrixWorld.decompose(spectatorCamera.position, spectatorCamera.quaternion, spectatorCamera.scale);
  spectatorCamera.updateMatrixWorld();
  const xrWas = renderer.xr.enabled;
  renderer.xr.enabled = false;
  renderer.setRenderTarget(spectatorTarget);
  renderer.render(scene, spectatorCamera);
  renderer.readRenderTargetPixels(spectatorTarget, 0, 0, OPERATOR_VIEW_W, OPERATOR_VIEW_H, spectatorPixels);
  renderer.setRenderTarget(null);
  renderer.xr.enabled = xrWas;
  // GL rows are bottom-up; flip into the image.
  const row = OPERATOR_VIEW_W * 4;
  for (let y = 0; y < OPERATOR_VIEW_H; y++) {
    spectatorImage.data.set(spectatorPixels.subarray((OPERATOR_VIEW_H - 1 - y) * row, (OPERATOR_VIEW_H - y) * row), y * row);
  }
  spectatorCtx.putImageData(spectatorImage, 0, 0);
  operatorViewBusy = true;
  spectatorCanvas.toBlob((blob) => {
    operatorViewBusy = false;
    if (blob && state.ws && state.ws.readyState === WebSocket.OPEN && state.recording && state.recording.active) {
      state.ws.send(blob);
      state.operatorFrames += 1;
    }
  }, 'image/jpeg', 0.7);
}

renderer.setAnimationLoop((timeMs, frame) => {
  if (frame && renderer.xr.isPresenting) collectAndSend(frame, timeMs);
  else if (state.preview) previewPose(timeMs);
  if (!renderer.xr.isPresenting) controls.update();
  renderer.render(scene, camera);
  captureOperatorView(timeMs);
});

// --------------------------------------------------------------- UI
function setStatus(text, kind) {
  const el = $('#stats');
  el.textContent = text;
  el.className = `stats ${kind || ''}`;
}

function renderStatus() {
  const s = state.lastState;
  const lines = [];
  const h = state.hello;
  if (h) {
    lines.push(`backend   ${h.backend}${h.backend === 'hardware' ? '   <-- THE ROBOT WILL MOVE' : ''}`);
    if (h.head) {
      const m = h.head.mapping;
      lines.push(`head      yaw ${m.yaw_joint} (sign ${m.yaw_sign > 0 ? '+' : '-'}, neutral ${(m.yaw_neutral_rad * DEG).toFixed(1)}°)  pitch ${m.pitch_joint} (sign ${m.pitch_sign > 0 ? '+' : '-'}, neutral ${(m.pitch_neutral_rad * DEG).toFixed(1)}°)  forward from ${m.forward_source}`);
      for (const n of m.notes) lines.push(`          note: ${n}`);
    } else lines.push('head      off (no tracker)');
    if (h.arms) {
      lines.push(`arms      ${h.arms.left.mapping} mapping  clutch ${h.clutch_button === 'a' ? 'A/X' : h.clutch_button}  scale ${h.arms.left.position_scale}  orientation ${h.arms.left.orientation_enabled ? 'on' : 'off'}  grippers L:${h.arms.left.gripper_available ? 'on' : 'unmeasured'} R:${h.arms.right.gripper_available ? 'on' : 'unmeasured'}`);
    } else lines.push('arms      off (no teleop)');
  }
  lines.push(recordingLine());
  const recBtn = $('#record');
  const active = !!(state.recording && state.recording.active);
  recBtn.textContent = active ? '\u25a0 Stop' : '\u25cf Record';
  recBtn.classList.toggle('on', active);
  if (s) {
    const hd = s.head || {};
    lines.push(`operator  ${s.operator.recentred ? 'recentred' : 'NOT recentred'}  head ${s.head_enabled ? 'on' : 'off'}  arms ${s.arms_enabled ? 'on' : 'off'}`);
    if (hd.tracking) {
      const t = hd.targets_rad || {};
      lines.push(`headset   yaw ${(hd.yaw_input_rad * DEG).toFixed(1)}°  pitch ${(hd.pitch_input_rad * DEG).toFixed(1)}°   ->  ${Object.entries(t).map(([k, v]) => `${k}=${(v * DEG).toFixed(1)}°`).join('  ')}${hd.at_limit && hd.at_limit.length ? '  AT LIMIT ' + hd.at_limit.join(',') : ''}`);
    } else lines.push('headset   not tracking');
    for (const side of ['left', 'right']) {
      const a = (s.arms || {})[side];
      if (!a) continue;
      const hint = { a: side === 'left' ? 'hold X' : 'hold A', grip: 'squeeze grip', stick: 'click stick' }[state.clutchButton] || 'hold the clutch';
      lines.push(`${side.padEnd(9)} ${a.tracked ? (a.clutched ? 'CLUTCHED - following' : `idle (${hint} to drive)`) : 'controller not tracked'}${a.gripper_rad != null ? `  gripper ${(a.gripper_rad * DEG).toFixed(0)}°` : ''}`);
    }
    if (s.ik) {
      const e = s.ik.errors || {};
      const fmt = (v) => (v == null ? '—' : `${(v * 1000).toFixed(1)}mm`);
      lines.push(`ik        ${s.ik.status || '—'}  L ${fmt(e.left_position_m)}  R ${fmt(e.right_position_m)}  ${s.ik.compute_ms ? s.ik.compute_ms.toFixed(1) + 'ms' : ''}${s.ik.message && !s.ik.commandable ? '  ' + s.ik.message : ''}`);
    }
    for (const w of s.warnings || []) lines.push(`warning   ${w}`);
  }
  const age = state.lastFrameMs == null ? null : (performance.now() - state.lastFrameMs) / 1000;
  const cam = !state.camConnected ? 'camera socket down, reconnecting'
    : age == null ? 'camera waiting for the first frame'
    : age > 3 ? `camera STALLED ${age.toFixed(0)} s (${state.frames} frames)`
    : `camera ${state.frames} frames, last ${age.toFixed(1)} s ago (${(state.lastFrameBytes / 1024).toFixed(0)} kB)`;
  lines.push(`link      sent ${state.sent}  recv ${state.received}  rtt ${state.lastRttMs == null ? '—' : state.lastRttMs.toFixed(0) + 'ms'}  ${cam}`);
  const el = $('#vr-status');
  el.textContent = lines.join('\n');
  drawHud(lines.slice(0, 6));
  const rtt = state.lastRttMs == null ? '' : `  rtt ${state.lastRttMs.toFixed(0)} ms`;
  setStatus(`${state.connected ? 'live' : 'offline'}${rtt}`, state.connected ? 'ok' : 'bad');
}

function drawHud(lines) {
  hudCtx.clearRect(0, 0, hudCanvas.width, hudCanvas.height);
  hudCtx.fillStyle = 'rgba(20,23,28,0.75)';
  hudCtx.fillRect(0, 0, hudCanvas.width, hudCanvas.height);
  hudCtx.fillStyle = '#e6e9ef';
  hudCtx.font = '28px ui-monospace, monospace';
  lines.forEach((line, i) => hudCtx.fillText(line.slice(0, 70), 16, 40 + i * 36));
  hudTexture.needsUpdate = true;
}

$('#preview').addEventListener('click', () => {
  state.preview = !state.preview;
  $('#preview').textContent = state.preview ? 'Stop preview' : 'Desktop preview';
  setStatus(state.preview ? 'desktop preview: orbit the view to move the head' : 'preview stopped', 'warn');
});
$('#recenter').addEventListener('click', () => send({ type: 'recenter' }));
$('#head-on').addEventListener('change', (e) => send({ type: 'set', head_enabled: e.target.checked }));
$('#arms-on').addEventListener('change', (e) => send({ type: 'set', arms_enabled: e.target.checked }));
$('#twin-on').addEventListener('change', (e) => { robotGroup.visible = e.target.checked; });
$('#mirror-on').addEventListener('change', (e) => { state.mirrorOn = e.target.checked; updateMirror(); });
$('#mirror-dist').addEventListener('change', (e) => {
  const v = parseFloat(e.target.value);
  if (Number.isFinite(v) && v > 0) { state.mirrorDistance = v; updateMirror(); }
});
$('#record').addEventListener('click', toggleRecording);
$('#cam-lock').addEventListener('change', (e) => setImageLocked(e.target.checked));
$('#pos-scale').addEventListener('change', (e) => {
  const v = parseFloat(e.target.value);
  if (Number.isFinite(v) && v > 0) send({ type: 'set', position_scale: v });
});

setInterval(renderStatus, 500);
connectTeleop();
