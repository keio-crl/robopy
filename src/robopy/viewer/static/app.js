// robopy viewer -- joint-space and end-effector control of the model, no hardware.
//
// The page never computes kinematics itself: every pose comes from the server,
// which runs the same WholeBodyModel / DualArmIK the controller uses.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';
import { STLLoader } from 'three/addons/STLLoader.js';

const DEG = 180 / Math.PI;
const $ = (sel) => document.querySelector(sel);

// ------------------------------------------------------------------ state
const state = {
  model: null,                 // /api/model description
  joints: {},                  // {name: rad}
  unitDeg: true,
  meshes: [],                  // THREE.Object3D per geometry
  tcpFrames: {},               // side -> AxesHelper (current)
  targetFrames: {},            // side -> AxesHelper (target)
  handles: {},                 // side -> draggable sphere sitting on the target
  ee: {                        // side -> {enabled, target:{p:[3], q:[4]} | null, current:{p,q}|null}
    left: { enabled: true, target: null, current: null },
    right: { enabled: false, target: null, current: null },
  },
  arm: 'left',                 // what the solver drives: left | right | both (single arm by default)
  fkInFlight: false,
  fkDirty: false,
  ikInFlight: false,
  ikPending: null,             // options of a solve asked for while one was in flight
  ikSeq: 0,                    // sequence of the last /api/ik request sent
  appliedSeq: -1,              // sequence of the newest trajectory the page accepted
  playback: null,              // the trajectory being played (see startPlayback)
  lastIk: null,                // the last accepted /api/ik reply, for the status block
  manualSinceSolve: false,     // a hand edit of the joints already reset the session
  trail: { left: [], right: [] }, // TCP positions of the played trajectories, newest last
  groundZ: 0,                  // where the grid sits: the bottom of the robot's base
  lastFkMs: null,
  lastPoses: null,             // the most recent /api/fk (or IK) reply, re-applied to late meshes
  playbackExpect: null,        // {t, tcp} the display expects for the FK request in flight
};
const STREAM_BUDGET_S = 0.6;   // trajectory duration asked for per request during a drag
const FULL_BUDGET_S = 2.0;     // ... for a jog, a release, the move button and continuations
const TRAIL_POINTS = 600;      // per side; older points fall off
// Exposed read-only for debugging and the browser regression test.
window.__robopy_state = state;

// ------------------------------------------------------------------ three.js scene
const viewport = $('#viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
viewport.appendChild(renderer.domElement);   // sized by resize(), below

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171c);
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
camera.up.set(0, 0, 1);                       // URDF is Z-up
camera.position.set(1.2, -1.2, 0.9);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0, 0.3);

scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 0.9));
const key = new THREE.DirectionalLight(0xffffff, 1.1); key.position.set(2, -2, 3); scene.add(key);
const fill = new THREE.DirectionalLight(0xffffff, 0.4); fill.position.set(-2, 2, 1); scene.add(fill);

const grid = new THREE.GridHelper(2, 20, 0x3a4250, 0x262c36);
grid.rotation.x = Math.PI / 2;                // GridHelper is XZ; we want XY
scene.add(grid);                              // z is set by placeGround() once the base is drawn
const worldAxes = new THREE.AxesHelper(0.15);
scene.add(worldAxes);
const frameGroup = new THREE.Group();          // TCP + target triads (toggleable)
scene.add(frameGroup);
const handleGroup = new THREE.Group();         // the grab handles; always shown, they are the affordance
scene.add(handleGroup);
const pathGroup = new THREE.Group();           // TCP trails + the moving reference markers
scene.add(pathGroup);

function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  if (!w || !h) return;                       // a collapsed viewport has no aspect ratio
  // The canvas must be told its CSS size, not only its backing-store size: with
  // the size left to the width/height attributes the element lays out at one
  // layout pixel per device pixel, so on a HiDPI screen -- or at any browser
  // zoom other than 100%, which moves devicePixelRatio too -- the canvas came
  // out devicePixelRatio times too large, overflowed #viewport and covered the
  // control panel next to it. The ratio is re-read here because zooming
  // changes it while the page is open.
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport);
resize();

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

// The grid is the floor the robot stands on, so it belongs at the bottom of
// the base -- the geometry no joint moves -- and not at the model origin: in
// the Rakuda export that origin sits about 26 cm above the floor, which put
// the grid through the middle of the torso. A model whose base draws nothing
// (or whose meshes failed) keeps the grid on the root frame, z = 0.
function placeGround() {
  const box = new THREE.Box3();
  let found = false;
  (state.model?.geometries || []).forEach((g, i) => {
    const obj = state.meshes[i];
    if (!g.static || !obj) return;
    box.expandByObject(obj);
    found = true;
  });
  state.groundZ = found && isFinite(box.min.z) ? box.min.z : 0;
  grid.position.z = state.groundZ;
}

function fitCamera() {
  const box = new THREE.Box3();
  for (const m of state.meshes) if (m.visible) box.expandByObject(m);
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length();
  controls.target.copy(center);
  const dir = camera.position.clone().sub(center).normalize();
  if (!isFinite(dir.length()) || dir.length() === 0) dir.set(1, -1, 0.6).normalize();
  camera.position.copy(center).add(dir.multiplyScalar(size * 1.2));
  camera.near = size / 100; camera.far = size * 20; camera.updateProjectionMatrix();
}
function lookFrom(dx, dy, dz) {
  const box = new THREE.Box3();
  for (const m of state.meshes) box.expandByObject(m);
  const center = box.isEmpty() ? new THREE.Vector3(0, 0, 0.3) : box.getCenter(new THREE.Vector3());
  const size = box.isEmpty() ? 1.5 : box.getSize(new THREE.Vector3()).length();
  controls.target.copy(center);
  camera.position.set(center.x + dx * size * 1.3, center.y + dy * size * 1.3, center.z + dz * size * 1.3 + 1e-3);
}
$('#view-fit').onclick = fitCamera;
$('#view-front').onclick = () => lookFrom(1, 0, 0.15);
$('#view-side').onclick = () => lookFrom(0, 1, 0.15);
$('#view-top').onclick = () => lookFrom(0, 0, 1);
$('#show-frames').onchange = (e) => { frameGroup.visible = e.target.checked; };
$('#show-path').onchange = (e) => { pathGroup.visible = e.target.checked; };

// ------------------------------------------------------------------ API
async function api(path, body) {
  const res = await fetch(path, body === undefined
    ? {}
    : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const data = await res.json();
  if (!res.ok) throw new Error(data.error || `${res.status} ${res.statusText}`);
  return data;
}

// Coalescing FK: at most one request in flight; a change during a request
// triggers exactly one more when it returns.
async function requestFK() {
  if (state.fkInFlight) { state.fkDirty = true; return; }
  state.fkInFlight = true;
  try {
    const sent = { ...state.joints };
    const expect = state.playbackExpect;
    const poses = await api('/api/fk', { joints: sent });
    applyPoses(poses, sent);
    if (expect) checkDisplayedPath(poses, expect);
  } catch (err) {
    setStats(`fk error: ${err.message}`);
  } finally {
    state.fkInFlight = false;
    if (state.fkDirty) { state.fkDirty = false; requestFK(); }
  }
}

function placeObject(obj, pose) {
  obj.position.set(pose.p[0], pose.p[1], pose.p[2]);
  obj.quaternion.set(pose.q[0], pose.q[1], pose.q[2], pose.q[3]);
}

function applyPoses(poses, sent) {
  // Meshes are downloaded asynchronously and may arrive AFTER this reply; the
  // reply is kept so loadMeshes() can place them when they show up. Without
  // that, every late mesh sat at the identity pose (its link frame at the world
  // origin) and the robot appeared to fall apart on slow connections.
  state.lastPoses = poses;
  poses.geometries.forEach((pose, i) => {
    const obj = state.meshes[i];
    if (obj) placeObject(obj, pose);
  });
  for (const [side, pose] of Object.entries(poses.tcp || {})) {
    // While a trajectory plays, the readouts follow its samples (playbackTick);
    // an FK reply from a few frames ago must not drag them back.
    if (state.playback) continue;
    state.ee[side].current = pose;
    const triad = state.tcpFrames[side];
    if (triad) {
      triad.position.set(...pose.p);
      triad.quaternion.set(...pose.q);
    }
  }
  if (poses.joints) {
    // The server echoes the configuration it actually used (continuous joints
    // come back wrapped into (-pi, pi]). Apply that echo only to joints the
    // user has not touched since the request was sent: a slider moved while a
    // request was in flight must not be snapped back by the stale reply.
    for (const [name, value] of Object.entries(poses.joints)) {
      if (!sent || state.joints[name] === sent[name]) state.joints[name] = value;
    }
    refreshJointInputs();
  }
  // An /api/ik reply carries poses but no FK timing; keep the last reading.
  if (typeof poses.timing_ms === 'number') {
    state.lastFkMs = poses.timing_ms;
    setStats(`fk ${poses.timing_ms.toFixed(2)} ms`);
  }
  renderEEPanel();
}
function setStats(text) { $('#stats').textContent = text; }

// ------------------------------------------------------------------ meshes
function materialFor(rgba) {
  const [r, g, b, a] = rgba;
  return new THREE.MeshStandardMaterial({
    color: new THREE.Color(r, g, b), transparent: a < 1, opacity: a, roughness: 0.6, metalness: 0.15,
  });
}

// URDF primitives: a cylinder's axis is Z, three.js's CylinderGeometry is Y,
// so it is wrapped in a group carrying the fixed 90-degree correction.
function primitiveObject(g) {
  const s = g.shape;
  let geometry, mesh;
  if (s.type === 'cylinder') {
    geometry = new THREE.CylinderGeometry(s.radius, s.radius, s.length, 48);
    mesh = new THREE.Mesh(geometry, materialFor(g.rgba));
    mesh.rotation.x = Math.PI / 2;
  } else if (s.type === 'box') {
    geometry = new THREE.BoxGeometry(...s.size);
    mesh = new THREE.Mesh(geometry, materialFor(g.rgba));
  } else if (s.type === 'sphere') {
    geometry = new THREE.SphereGeometry(s.radius, 32, 24);
    mesh = new THREE.Mesh(geometry, materialFor(g.rgba));
  } else {
    return new THREE.Group();
  }
  const holder = new THREE.Group();
  holder.add(mesh);
  return holder;
}

async function loadMeshes(model) {
  const loader = new STLLoader();
  let failed = 0, done = 0;
  const total = model.geometries.filter((g) => g.shape.type === 'mesh').length;
  const overlay = $('#overlay');
  // Register an object for geometry i and put it where the latest FK reply
  // says, if one has arrived already (see applyPoses).
  const attach = (i, obj) => {
    state.meshes[i] = obj;
    const pose = state.lastPoses && state.lastPoses.geometries[i];
    if (pose) placeObject(obj, pose);
    scene.add(obj);
  };
  const jobs = model.geometries.map((g, i) => new Promise((resolve) => {
    if (g.shape.type !== 'mesh') {
      const obj = primitiveObject(g);
      obj.name = g.id;
      attach(i, obj);
      resolve();
      return;
    }
    loader.load(g.url, (geometry) => {
      geometry.computeVertexNormals();
      const mesh = new THREE.Mesh(geometry, materialFor(g.rgba));
      mesh.scale.set(...g.scale);
      mesh.name = g.id;
      attach(i, mesh);
      done += 1; overlay.textContent = `loading meshes… ${done}/${total}`;
      resolve();
    }, undefined, (err) => { failed += 1; console.warn('mesh failed', g.url, err); attach(i, new THREE.Group()); resolve(); });
  }));
  await Promise.all(jobs);
  // Belt and braces: whatever order the downloads and the FK reply finished in,
  // every object now carries the latest configuration.
  if (state.lastPoses) applyPoses(state.lastPoses, null);
  overlay.hidden = true;
  if (failed) setStats(`${failed} mesh(es) failed to load`);
}

function makeTriad(size, opacity) {
  const axes = new THREE.AxesHelper(size);
  axes.material.transparent = opacity < 1;
  axes.material.opacity = opacity;
  axes.material.depthTest = false;
  axes.renderOrder = 10;
  return axes;
}

// ------------------------------------------------------------------ joints panel
function jointValueDisplay(rad) { return state.unitDeg ? (rad * DEG).toFixed(2) : rad.toFixed(4); }
function jointStepRad() {
  const step = parseFloat($('#joint-step').value) || 0;
  return state.unitDeg ? step / DEG : step;
}

// Where a slider's range came from. Every consumer -- these sliders, the
// solver, the machine adapter -- reads the same resolved range, so the slider
// range IS the solver's range. The source says what decided it and whether it
// is validated for the machine.
const LIMIT_NOTE = {
  urdf: 'range declared by the URDF',
  override: 'recorded override of the URDF range (see the reason in the Info tab)',
  soft: 'soft limit narrowing the model\'s range',
  display: 'display range only: no finite limit is known; the solver will not drive this joint',
  unbounded: 'no limit known',
};
const LIMIT_TAG = { override: ' (override)', display: ' (display)' };

function limitText(j) {
  const tag = LIMIT_TAG[j.limit_source] || '';
  const prov = j.limit_source === 'soft' && !j.validated ? ' (provisional)' : '';
  return `${fmtLimit(j.lower)} … ${fmtLimit(j.upper)}${tag}${prov}`;
}
function limitTitle(j) {
  let note = LIMIT_NOTE[j.limit_source] || LIMIT_NOTE.urdf;
  note += j.validated ? '; validated for the machine' : '; NOT validated for the machine (simulation only)';
  if (j.urdf_lower != null && j.limit_source !== 'urdf') {
    note += `. URDF: ${fmtLimit(j.urdf_lower)} … ${fmtLimit(j.urdf_upper)}.`;
  }
  for (const n of j.limit_notes || []) note += ` ${n}.`;
  return note;
}

function buildJointsPanel(model) {
  const groups = { torso: [], left: [], right: [], head: [] };
  for (const j of model.joints) (groups[j.group] || groups.torso).push(j);
  const root = $('#joint-groups');
  root.innerHTML = '';
  const titles = { torso: 'Torso', left: 'Left arm', right: 'Right arm', head: 'Head (never driven by the arm IK)' };
  for (const [group, joints] of Object.entries(groups)) {
    if (!joints.length) continue;
    const box = document.createElement('div');
    box.className = `group ${group}`;
    box.innerHTML = `<h3>${titles[group]}</h3>`;
    for (const j of joints) {
      state.joints[j.name] = state.joints[j.name] ?? 0;
      const row = document.createElement('div');
      row.className = 'joint';
      row.dataset.joint = j.name;
      row.innerHTML = `
        <div class="name"><span>${j.name}${j.continuous ? ' <span class="dim" title="continuous joint">∞</span>' : ''}</span>
          <span class="lim ${j.limit_is_display_only ? 'display-only' : ''}" title="${limitTitle(j)}">
            ${limitText(j)}</span></div>
        <!-- step="any": a stepped range snaps to multiples of (max - min) from
             its minimum, and an asymmetric range's grid misses zero, so the
             home pose sat a few thousandths of a degree off. -->
        <input type="range" min="${j.lower}" max="${j.upper}" step="any" value="0">
        <input type="text" class="num" value="0">
        <button class="jog" data-dir="-1">−</button>
        <button class="jog" data-dir="1">+</button>`;
      const slider = row.querySelector('input[type=range]');
      const text = row.querySelector('input.num');
      slider.addEventListener('input', () => setJoint(j.name, parseFloat(slider.value)));
      text.addEventListener('change', () => {
        const v = parseFloat(text.value);
        if (!isFinite(v)) { refreshJointInputs(); return; }
        setJoint(j.name, state.unitDeg ? v / DEG : v);
      });
      row.querySelectorAll('button.jog').forEach((b) => b.addEventListener('click', () => {
        setJoint(j.name, state.joints[j.name] + parseInt(b.dataset.dir, 10) * jointStepRad());
      }));
      box.appendChild(row);
    }
    root.appendChild(box);
  }
  refreshJointInputs();
}
function fmtLimit(rad) { return state.unitDeg ? `${(rad * DEG).toFixed(0)}°` : rad.toFixed(2); }

function setJoint(name, rad) {
  const j = state.model.joints.find((x) => x.name === name);
  if (j) rad = Math.min(j.upper, Math.max(j.lower, rad));
  manualPoseEdit();
  state.joints[name] = rad;
  refreshJointInputs();
  requestFK();
}

// A joint edited by hand is one of the explicit session resets: whatever was
// playing stops (the pose is now yours, not the trajectory's), and the next
// move first resets the solver at the pose it starts from -- once per burst
// of edits, not once per slider event, and anchored where the burst ended.
function manualPoseEdit() {
  stopPlayback();
  state.manualSinceSolve = true;
}

function setAllJoints(values) {
  manualPoseEdit();
  for (const k of Object.keys(state.joints)) state.joints[k] = 0;
  for (const [k, v] of Object.entries(values)) if (k in state.joints) state.joints[k] = v;
  refreshJointInputs();
  requestFK();
}

function refreshJointInputs() {
  for (const row of document.querySelectorAll('.joint')) {
    const name = row.dataset.joint;
    const v = state.joints[name] ?? 0;
    const slider = row.querySelector('input[type=range]');
    const text = row.querySelector('input.num');
    if (document.activeElement !== slider) slider.value = v;
    if (document.activeElement !== text) text.value = jointValueDisplay(v);
    const j = state.model.joints.find((x) => x.name === name);
    const lim = row.querySelector('.lim');
    lim.firstChild.textContent = limitText(j);
    lim.title = limitTitle(j);
  }
}

document.querySelectorAll('input[name=unit]').forEach((r) => r.addEventListener('change', () => {
  const wasDeg = state.unitDeg;
  state.unitDeg = r.value === 'deg';
  const step = $('#joint-step');
  if (wasDeg !== state.unitDeg) step.value = state.unitDeg ? (parseFloat(step.value) * DEG).toFixed(2) : (parseFloat(step.value) / DEG).toFixed(4);
  $('#joint-step-unit').textContent = state.unitDeg ? 'deg' : 'rad';
  refreshJointInputs();
}));
$('#joints-zero').onclick = () => setAllJoints({});
$('#joints-home').onclick = () => {
  const home = state.model?.home_positions_rad || {};
  if (Object.keys(home).length) setAllJoints(home);
};
$('#joints-copy').onclick = async () => {
  const text = JSON.stringify(state.joints, null, 2);
  try { await navigator.clipboard.writeText(text); setStats('copied joint angles (rad) to clipboard'); }
  catch { window.prompt('Joint angles (radians):', text); }
};

// ------------------------------------------------------------------ end-effector panel
function quatToRpy(q) {            // URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll)  == three.js order 'ZYX'
  const e = new THREE.Euler().setFromQuaternion(new THREE.Quaternion(q[0], q[1], q[2], q[3]), 'ZYX');
  return [e.x, e.y, e.z];
}
function rpyToQuat(r, p, y) {
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(r, p, y, 'ZYX'));
  return [q.x, q.y, q.z, q.w];
}

// Range of one target slider, in the units the page shows (mm, deg). Position
// spans the reach bound the server reports -- the arm unfolded from its
// shoulder -- so the whole of what the hand could possibly reach is one drag
// wide. Without a solver there is no bound; +/-1 m keeps the slider usable.
function targetRange(side, key) {
  if (key !== 'x' && key !== 'y' && key !== 'z') return { min: -180, max: 180, step: 0.5 };
  const reach = state.model.ik && state.model.ik.workspace && state.model.ik.workspace[side];
  const centre = reach ? reach.center[{ x: 0, y: 1, z: 2 }[key]] * 1000 : 0;
  const radius = reach ? reach.radius * 1000 : 1000;
  return { min: Math.round(centre - radius), max: Math.round(centre + radius), step: 1 };
}

function buildEEPanel(model) {
  const root = $('#ee-sides');
  root.innerHTML = '';
  for (const side of ['left', 'right']) {
    if (!model.tcp_frames[side]) continue;
    const box = document.createElement('div');
    box.className = `side ${side}`;
    box.dataset.side = side;
    box.innerHTML = `
      <header><span>${side} TCP <span class="dim">(${model.tcp_frames[side]})</span></span>
        <span class="role" data-role></span>
        <button class="ee-capture" title="Set the target to the current pose">capture</button></header>
      <div class="pose">
        <span class="h"></span><span class="h">current</span><span class="h">target</span>
        ${['x', 'y', 'z', 'roll', 'pitch', 'yaw'].map((k) => {
          const r = targetRange(side, k);
          return `
          <span class="h">${k}</span><span class="cur" data-cur="${k}">—</span>
          <span class="tgt"><input type="text" class="num" data-tgt="${k}"><button class="jog" data-jog="${k}" data-dir="-1">−</button><button class="jog" data-jog="${k}" data-dir="1">+</button></span>
          <input type="range" class="bar" data-bar="${k}" min="${r.min}" max="${r.max}" step="${r.step}" value="0" title="${k} target: ${r.min} … ${r.max} ${k.length === 1 ? 'mm' : 'deg'}">`;
        }).join('')}
      </div>`;
    box.querySelector('.ee-capture').addEventListener('click', () => { captureTarget(side); });
    box.querySelectorAll('.tgt input').forEach((inp) => inp.addEventListener('change', () => { readTargetInputs(side); maybeLiveSolve(); }));
    box.querySelectorAll('button.jog').forEach((b) => b.addEventListener('click', () => {
      jogTarget(side, b.dataset.jog, parseInt(b.dataset.dir, 10));
    }));
    // Dragging a bar is a stream of small moves: each one re-targets the
    // motion with a short duration budget, and the release asks once more
    // with the full budget so the motion that stays is the whole one.
    box.querySelectorAll('input.bar').forEach((bar) => {
      const set = () => setTargetComponent(side, bar.dataset.bar, parseFloat(bar.value));
      bar.addEventListener('input', () => { set(); maybeLiveSolve({ budget: STREAM_BUDGET_S }); });
      bar.addEventListener('change', () => { set(); maybeLiveSolve(); });
    });
    root.appendChild(box);
  }
  applyArmSelection(state.arm, { reset: false });
}

// The arm selector is the one place that decides which hands the solver
// drives: the boxes, the handles and the targets sent all follow it.
function applyArmSelection(arm, { reset = true } = {}) {
  state.arm = arm;
  for (const side of ['left', 'right']) {
    const enabled = arm === 'both' || arm === side;
    const was = state.ee[side].enabled;
    state.ee[side].enabled = enabled;
    const box = document.querySelector(`.side[data-side=${side}]`);
    if (box) {
      box.classList.toggle('idle', !enabled);
      const role = box.querySelector('[data-role]');
      role.textContent = enabled ? 'driven' : 'idle';
      role.className = `role ${enabled ? 'driven' : ''}`;
      box.querySelectorAll('.tgt input, .tgt button, input.bar').forEach((el) => { el.disabled = !enabled; });
    }
    // A hand that was idle may have moved with the torso: start from where
    // it is rather than pulling it back to a stale world target.
    if (enabled && !was) captureTarget(side);
    // Takes the triad *and* the drag handle with it: a hand nothing tracks
    // must not be left with a grabbable target in the scene.
    updateTargetFrame(side);
  }
  const sel = $('#arm-select');
  if (sel.value !== arm) sel.value = arm;
  if (reset) sessionReset(`arm: ${arm}`);
  renderSession();
}

function captureTarget(side) {
  const cur = state.ee[side].current;
  if (!cur) return;
  state.ee[side].target = { p: [...cur.p], q: [...cur.q] };
  writeTargetInputs(side);
  updateTargetFrame(side);
}
function ensureTarget(side) { if (!state.ee[side].target) captureTarget(side); }

function writeTargetInputs(side) {
  const t = state.ee[side].target; if (!t) return;
  const box = document.querySelector(`.side[data-side=${side}]`);
  const rpy = quatToRpy(t.q);
  const vals = { x: t.p[0] * 1000, y: t.p[1] * 1000, z: t.p[2] * 1000, roll: rpy[0] * DEG, pitch: rpy[1] * DEG, yaw: rpy[2] * DEG };
  for (const [k, v] of Object.entries(vals)) {
    const inp = box.querySelector(`input[data-tgt=${k}]`);
    if (document.activeElement !== inp) inp.value = v.toFixed(k.length === 1 ? 1 : 2);
    const bar = box.querySelector(`input[data-bar=${k}]`);
    if (bar && document.activeElement !== bar) bar.value = v;   // the bar clamps a target outside its range
  }
}

// One component of a target, given in the units the page shows (mm, deg).
function setTargetComponent(side, key, value) {
  if (!isFinite(value)) return;
  ensureTarget(side);
  const t = state.ee[side].target;
  const idx = { x: 0, y: 1, z: 2 }[key];
  if (idx !== undefined) {
    t.p[idx] = value / 1000;
  } else {
    const rpy = quatToRpy(t.q);
    rpy[{ roll: 0, pitch: 1, yaw: 2 }[key]] = value / DEG;
    t.q = rpyToQuat(...rpy);
  }
  writeTargetInputs(side);
  updateTargetFrame(side);
}
function readTargetInputs(side) {
  ensureTarget(side);
  const box = document.querySelector(`.side[data-side=${side}]`);
  const g = (k) => parseFloat(box.querySelector(`input[data-tgt=${k}]`).value);
  const p = [g('x') / 1000, g('y') / 1000, g('z') / 1000];
  const q = rpyToQuat(g('roll') / DEG, g('pitch') / DEG, g('yaw') / DEG);
  if (p.every(isFinite) && q.every(isFinite)) state.ee[side].target = { p, q };
  writeTargetInputs(side);
  updateTargetFrame(side);
}
function jogTarget(side, key, dir) {
  ensureTarget(side);
  const t = state.ee[side].target;
  const dpos = (parseFloat($('#ee-step-mm').value) || 0) / 1000;
  const drot = (parseFloat($('#ee-step-deg').value) || 0) / DEG;
  const idx = { x: 0, y: 1, z: 2 }[key];
  if (idx !== undefined) {
    t.p[idx] += dir * dpos;
  } else {
    const rpy = quatToRpy(t.q);
    rpy[{ roll: 0, pitch: 1, yaw: 2 }[key]] += dir * drot;
    t.q = rpyToQuat(...rpy);
  }
  writeTargetInputs(side);
  updateTargetFrame(side);
  maybeLiveSolve();
}
function updateTargetFrame(side) {
  const t = state.ee[side].target, triad = state.targetFrames[side];
  const handle = state.handles[side];
  if (handle && t) {
    handle.position.set(...t.p);
    handle.visible = state.ee[side].enabled;
  }
  if (!t || !triad) return;
  triad.position.set(...t.p);
  triad.quaternion.set(...t.q);
  triad.visible = state.ee[side].enabled;
}

function renderEEPanel() {
  for (const side of ['left', 'right']) {
    const box = document.querySelector(`.side[data-side=${side}]`);
    const cur = state.ee[side].current;
    if (!box || !cur) continue;
    const rpy = quatToRpy(cur.q);
    const vals = { x: cur.p[0] * 1000, y: cur.p[1] * 1000, z: cur.p[2] * 1000, roll: rpy[0] * DEG, pitch: rpy[1] * DEG, yaw: rpy[2] * DEG };
    for (const [k, v] of Object.entries(vals)) box.querySelector(`[data-cur=${k}]`).textContent = v.toFixed(k.length === 1 ? 1 : 2);
    if (!state.ee[side].target) { state.ee[side].target = { p: [...cur.p], q: [...cur.q] }; writeTargetInputs(side); updateTargetFrame(side); }
  }
}

// ------------------------------------------------------------------ modes
function buildModeControls(model) {
  const ik = model.ik;
  const sel = $('#ori-select');
  sel.innerHTML = '';
  if (!ik) return;
  for (const mode of ik.orientation_modes) {
    const opt = document.createElement('option');
    opt.value = mode; opt.textContent = mode;
    sel.appendChild(opt);
  }
  sel.value = ik.orientation_mode;
  $('#ori-weight').value = ik.orientation_weight;
  if (ik.approach_axis_tcp) {
    $('#ori-select-wrap').title += ` Approach axis in the TCP frame: [${ik.approach_axis_tcp.map((v) => v.toFixed(2)).join(', ')}].`;
  }
  syncWeightInput();
  $('#ik-collision').hidden = ik.collision_modelled;
  $('#ik-collision').textContent = 'Self-collision is NOT evaluated: no collision pairs are registered on this model, so the solver avoids nothing (loading a collision URDF alone registers none).';
}
function syncWeightInput() {
  $('#ori-weight').disabled = $('#ori-select').value === 'position_only';
}
$('#ori-select').addEventListener('change', () => { syncWeightInput(); sessionReset('orientation mode'); maybeLiveSolve(); });
$('#ori-weight').addEventListener('change', () => maybeLiveSolve());
$('#torso-policy').addEventListener('change', (e) => {
  $('#torso-velocity-wrap').hidden = e.target.value !== 'manual';
  sessionReset('torso policy');
  maybeLiveSolve();
});
$('#inactive-arm-policy').addEventListener('change', () => { sessionReset('inactive arm policy'); maybeLiveSolve(); });
$('#arm-select').addEventListener('change', (e) => applyArmSelection(e.target.value));
$('#ee-capture-all').onclick = () => { captureTarget('left'); captureTarget('right'); setIKStatus('targets set to the current poses', 'ok'); };
$('#ik-solve').onclick = () => solveIK();
$('#ik-reset').onclick = () => { stopPlayback(); sessionReset('reset button'); };
function maybeLiveSolve(opts) { if ($('#ik-live').checked) solveIK(opts); }

function setIKStatus(text, kind) {
  const el = $('#ik-status');
  el.textContent = text;
  el.className = `status ${kind || ''}`;
}

// The session line: always says which hands are driven and under what modes,
// so a reply is never read against the wrong assumptions.
function renderSession() {
  const ik = state.model?.ik;
  if (!ik) { $('#ik-session').textContent = 'no solver'; return; }
  const driven = ['left', 'right'].filter((s) => state.ee[s].enabled);
  const idle = ['left', 'right'].filter((s) => !state.ee[s].enabled);
  const inactive = $('#inactive-arm-policy').value === 'hold_joints' ? 'follow torso, keep joints' : 'hold TCP in world';
  const ori = $('#ori-select').value || ik.orientation_mode;
  const weight = ori === 'position_only' ? '' : ` ×${parseFloat($('#ori-weight').value) || 0}`;
  const traj = ik.trajectory;
  const lines = [
    `driving: ${driven.join(' + ') || 'nothing'}${idle.length ? `   idle: ${idle.join(' + ')} (${inactive})` : ''}`,
    `torso: ${$('#torso-policy').value}   orientation: ${ori}${weight}   priority: ${ik.priority_mode}   collision: ${ik.collision_modelled ? 'evaluated' : 'NOT evaluated'}`,
    `profile ${traj.profile}: v ≤ ${traj.max_linear_velocity_m_s} m/s, a ≤ ${traj.max_linear_acceleration_m_s2} m/s², ω ≤ ${traj.max_angular_velocity_rad_s} rad/s, dt ${traj.sample_period_s} s`,
  ];
  $('#ik-session').textContent = lines.join('\n');
}

// ------------------------------------------------------------------ the solver session
// Explicit resets only: start-up, a mode switch, a manual pose change, the
// button. A continuous operation keeps its velocity history and its posture
// reference, or the acceleration bound and the posture objective mean nothing.
async function sessionReset(reason) {
  if (!state.model?.ik) return;
  clearTrail();
  try {
    await api('/api/ik/reset', { joints: state.joints });
    console.debug('session reset:', reason);
  } catch (err) {
    setIKStatus(`reset failed: ${err.message}`, 'bad');
  }
}

// Every request carries a sequence number; the reply echoes it, and a reply
// older than one already accepted is dropped, so a slow answer can never
// overwrite a newer target or a hand edit made meanwhile.
// opts.budget: duration (seconds of motion) the request may plan.
// opts.continuation: the follow-on request for a truncated trajectory.
// Coalescing, like requestFK: one request in flight, and a solve asked for
// meanwhile runs exactly once more afterwards -- from whatever the targets are
// by then, so a fast drag never queues a backlog of stale poses.
async function solveIK(opts = {}) {
  if (!state.model?.ik) return;
  if (state.ikInFlight) { if (!opts.continuation) state.ikPending = opts; return; }
  const { budget = FULL_BUDGET_S, continuation = false } = opts;
  const targets = {};
  for (const side of ['left', 'right']) {
    if (state.ee[side].enabled && state.ee[side].target) targets[side] = state.ee[side].target;
  }
  if (!Object.keys(targets).length) { setIKStatus('no arm selected; nothing to move', 'warn'); return; }
  state.ikInFlight = true;
  if (state.manualSinceSolve) {
    state.manualSinceSolve = false;
    await sessionReset('manual pose edit');
  }
  const seq = ++state.ikSeq;
  const sentAt = performance.now();
  // A move asked for while a trajectory plays continues it: the reference
  // resumes from its pose and velocity at this instant and the solver from
  // the joint velocity there, so the new goal bends the motion.
  let resume = null;
  let joints = { ...state.joints };
  const playing = state.playback;
  if (playing) {
    const t = continuation ? playing.duration : playbackTime(playing);
    resume = { seq: playing.seq, t };
    if (continuation) joints = { ...playing.samples[playing.samples.length - 1].joints };
    else joints = { ...playbackJoints(playing, t) };
  }
  if (!continuation) setIKStatus(playing ? 're-targeting…' : 'planning…', 'busy');
  try {
    const body = {
      mode: 'trajectory', seq, joints, targets, resume,
      torso_policy: $('#torso-policy').value,
      inactive_arm_policy: $('#inactive-arm-policy').value,
      torso_velocity_rad_s: parseFloat($('#torso-velocity').value) || 0,
      orientation_mode: $('#ori-select').value || null,
      orientation_weight: $('#ori-select').value === 'position_only' ? null : (parseFloat($('#ori-weight').value) || 0),
      max_duration_s: budget,
    };
    const res = await api('/api/ik', body);
    if (res.seq < state.appliedSeq || (res.seq !== state.ikSeq && state.ikPending)) {
      // Stale: a newer request has been accepted, or one is waiting to go
      // out with newer targets. Its own reply will carry the motion.
      console.debug('dropped stale trajectory reply', res.seq);
    } else if (state.manualSinceSolve && !continuation) {
      // The joints were edited by hand while this was in flight: the pose
      // is the user's now, the plan started from one that no longer holds.
      console.debug('dropped trajectory reply after a manual edit', res.seq);
    } else {
      state.appliedSeq = res.seq;
      state.lastIk = res;
      if (res.commandable && res.samples && res.samples.length > 1) {
        // A re-target keeps the time base of the motion it bends (the hand
        // kept moving while this was planned, and the plan started from that
        // instant); a move from rest starts now, so nothing is skipped.
        const t0 = playing ? sentAt : performance.now();
        startPlayback(res, t0, { append: continuation && playing && state.playback === playing });
      } else {
        stopPlayback();
      }
      renderStatus(res);
    }
  } catch (err) {
    setIKStatus(`error: ${err.message}`, 'bad');
  } finally {
    state.ikInFlight = false;
    const pending = state.ikPending;
    state.ikPending = null;
    if (pending) solveIK(pending);
  }
}

function fmtErr(m, r) {
  if (m == null) return '—';
  return `${(m * 1000).toFixed(2)} mm${r == null ? '' : ` / ${(r * DEG).toFixed(2)}°`}`;
}

const STATUS_KIND = {
  converged: 'ok', tracking: 'busy',
  locally_stalled: 'warn', limits_blocked: 'warn', collision_blocked: 'warn',
};
const STATUS_HINT = {
  locally_stalled: 'No constraint is active and the residual stopped improving: the local method cannot progress from this configuration. That does not prove the goal unreachable -- try from another posture, another orientation mode, or with the torso free.',
  limits_blocked: 'A joint limit is binding (listed above). Whether an orientation can be held while translating depends on the whole arm\'s configuration and its limits, not on the wrist alone; position_only or axis_aligned asks for less.',
  collision_blocked: 'A collision pair is at its safety distance; the solver will not push through it.',
  stale: 'The request was not usable (the input was old or inconsistent). The pose is unchanged.',
  solver_error: 'The QP failed; the pose is unchanged.',
  infeasible: 'No step satisfies every constraint from here. The pose is unchanged; check the limits listed above.',
  limit_violation: 'A joint is already outside its resolved range. Bring it back inside (Joints tab) and move again.',
  collision_at_start: 'The starting pose is already in self-collision. Move a joint by hand first.',
};

function renderStatus(res) {
  const e = res.errors || {};
  const driven = res.enabled || [];
  const sideLine = (side) => {
    if (driven.includes(side)) {
      const goal = res.goal_error_m?.[side];
      const ang = res.goal_orientation_error_rad?.[side];
      const oriShown = res.orientation_mode === 'position_only' ? null : ang;
      return `${side.padEnd(5)}: goal ${fmtErr(goal, oriShown)}${res.orientation_mode === 'position_only' && ang != null ? `   (orientation ${(ang * DEG).toFixed(1)}° not requested)` : ''}`;
    }
    if (res.inactive_arm_policy === 'hold_joints') return `${side.padEnd(5)}: idle, follows torso; arm joints held`;
    return `${side.padEnd(5)}: idle, hold residual ${fmtErr(e[`${side}_hold_m`], null)}`;
  };
  const head = res.truncated
    ? `${res.status.toUpperCase()} · ${res.duration_s.toFixed(2)} s planned, still moving (continuing)`
    : `${res.status.toUpperCase()} · ${res.duration_s.toFixed(2)} s · ${res.n_samples} samples`;
  const dev = state.playback?.maxDeviationM;
  const lines = [
    `${head} · ${res.timing_ms.toFixed(0)} ms compute`,
    sideLine('left'),
    sideLine('right'),
    `torso ω ${res.torso_velocity_rad_s.toFixed(3)} rad/s`
      + (res.min_singular_value != null ? `   σmin ${res.min_singular_value.toFixed(3)}` : '')
      + (res.active_limits.length ? `   limits: ${res.active_limits.join(', ')}` : '')
      + (dev ? `   display path deviation ${(dev * 1000).toFixed(2)} mm` : ''),
    res.message ? `\n${res.message}` : '',
    STATUS_HINT[res.status] ? `\n${STATUS_HINT[res.status]}` : '',
  ];
  const kind = res.commandable ? (STATUS_KIND[res.status] || 'warn') : 'bad';
  setIKStatus(lines.filter((l) => l !== '').join('\n'), kind);
  renderSession();
}

// ------------------------------------------------------------------ timed playback
// The server returns the trajectory as time-stamped samples; the page plays
// them at their own time stamps. Between two samples the joint angles are
// interpolated linearly -- exact for revolute joints, the model's integrate()
// along a constant velocity -- and never compressed into a fixed tween.
//
// The playback clock is the trajectory's sample period, on its own timer:
// the drawing rate is the renderer's business (a slow machine draws fewer
// frames of the same motion), the control rate is not.
function playbackTime(pb) { return Math.max(0, (performance.now() - pb.t0) / 1000); }
function playbackPeriodMs() {
  const period = state.model?.ik?.trajectory?.sample_period_s;
  return Math.max(5, Math.round((period || 0.02) * 1000));
}

// Index of the sample interval holding t, cached so a tick is O(1).
function playbackIndex(pb, t) {
  const s = pb.samples;
  let i = Math.min(pb.cursor || 0, s.length - 2);
  while (i > 0 && s[i].t > t) i -= 1;
  while (i < s.length - 2 && s[i + 1].t <= t) i += 1;
  pb.cursor = i;
  return i;
}
function lerpJoints(a, b, f) {
  const out = {};
  for (const k of Object.keys(a)) out[k] = a[k] + ((b[k] ?? a[k]) - a[k]) * f;
  return out;
}
function lerpP(a, b, f) { return [0, 1, 2].map((i) => a[i] + (b[i] - a[i]) * f); }
function playbackJoints(pb, t) {
  const s = pb.samples;
  if (t >= pb.duration) return s[s.length - 1].joints;
  const i = playbackIndex(pb, t);
  const a = s[i], b = s[i + 1];
  const f = b.t > a.t ? Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t))) : 1;
  return lerpJoints(a.joints, b.joints, f);
}
// Poses at t from the samples' 'tcp' or 'reference' entries: {side: {p, q}}.
// The position is interpolated; the orientation is the nearer sample's.
function playbackPoses(pb, t, key) {
  const s = pb.samples;
  const out = {};
  if (t >= pb.duration) {
    for (const [side, pose] of Object.entries(s[s.length - 1][key] || {})) out[side] = { p: [...pose.p], q: [...pose.q] };
    return out;
  }
  const i = playbackIndex(pb, t);
  const a = s[i], b = s[i + 1];
  const f = b.t > a.t ? Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t))) : 1;
  for (const [side, pose] of Object.entries(a[key] || {})) {
    const later = b[key] && b[key][side];
    out[side] = later
      ? { p: lerpP(pose.p, later.p, f), q: [...(f < 0.5 ? pose.q : later.q)] }
      : { p: [...pose.p], q: [...pose.q] };
  }
  return out;
}

function startPlayback(res, t0, { append = false } = {}) {
  const pb = {
    seq: res.seq, samples: res.samples, duration: res.duration_s, truncated: res.truncated,
    t0, cursor: 0, continuationAsked: false, maxDeviationM: state.playback?.maxDeviationM || 0,
    computeMs: res.timing_ms,
  };
  if (append && state.playback) {
    // The continuation of the trajectory still playing: queue it to start
    // when that one ends, so the motion is one piece.
    state.playback.next = pb;
    pb.t0 = null;
    extendTrail(res);
    return;
  }
  state.playback = pb;
  extendTrail(res);
  if (!playbackLoop.timer) playbackLoop.timer = setInterval(playbackTick, playbackPeriodMs());
}

function stopPlayback() {
  state.playback = null;
  state.playbackExpect = null;
  endPlaybackLoop();
  for (const marker of Object.values(referenceMarkers)) marker.visible = false;
}

const playbackLoop = { timer: null };
function endPlaybackLoop() {
  if (playbackLoop.timer) clearInterval(playbackLoop.timer);
  playbackLoop.timer = null;
}
function playbackTick() {
  const pb = state.playback;
  if (!pb) { endPlaybackLoop(); return; }
  const t = playbackTime(pb);
  const done = t >= pb.duration;
  const joints = playbackJoints(pb, t);
  for (const [k, v] of Object.entries(joints)) state.joints[k] = v;
  refreshJointInputs();
  const refs = playbackPoses(pb, t, 'reference');
  for (const [side, pose] of Object.entries(refs)) {
    const marker = referenceMarkers[side];
    if (marker) { marker.position.set(...pose.p); marker.visible = state.ee[side].enabled; }
  }
  // The TCP readouts and triads follow the playback clock from the samples
  // themselves; the meshes follow as fast as the FK replies land.
  const tcp = playbackPoses(pb, t, 'tcp');
  for (const [side, pose] of Object.entries(tcp)) {
    if (!state.ee[side]) continue;
    state.ee[side].current = pose;
    const triad = state.tcpFrames[side];
    if (triad) { triad.position.set(...pose.p); triad.quaternion.set(...pose.q); }
  }
  renderEEPanel();
  state.playbackExpect = { t, tcp };
  requestFK();
  // The follow-on of a truncated run is asked for early enough that it can
  // be back before this one ends (its compute time is known from this one).
  if (pb.truncated && !pb.continuationAsked && !pb.next) {
    const lead = Math.min(1.5, (pb.computeMs * 1.5 + 150) / 1000);
    if (pb.duration - t <= lead) { pb.continuationAsked = true; solveIK({ continuation: true, budget: FULL_BUDGET_S }); }
  }
  if (done) {
    if (pb.next) {
      // Seamless when the continuation arrived in time; when it came late the
      // hand held its last pose, so the follow-on starts now rather than
      // skipping the motion it missed.
      const next = pb.next;
      next.t0 = Math.max(pb.t0 + pb.duration * 1000, performance.now() - 16);
      state.playback = next;
    } else if (!pb.truncated) {
      // Finished. (A truncated run holds its last pose until the
      // continuation arrives and starts on its own.)
      state.playback = null;
      state.playbackExpect = null;
      endPlaybackLoop();
    }
  }
}

// The displayed path is the FK of interpolated joints; the samples' TCPs are
// what the solver saw. The largest gap between the two is the display error
// of the interpolation, reported in the status block.
function checkDisplayedPath(poses, expect) {
  const pb = state.playback;
  if (!pb || !poses.tcp) return;
  for (const [side, pose] of Object.entries(expect.tcp)) {
    const shown = poses.tcp[side];
    if (!shown || !state.ee[side].enabled) continue;
    const p = pose.p;
    const d = Math.hypot(shown.p[0] - p[0], shown.p[1] - p[1], shown.p[2] - p[2]);
    if (d > pb.maxDeviationM) pb.maxDeviationM = d;
  }
}

// ------------------------------------------------------------------ trail + reference markers
const trailLines = {};
const referenceMarkers = {};
function makeTrail(side) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(TRAIL_POINTS * 3), 3));
  geometry.setDrawRange(0, 0);
  const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({
    color: side === 'left' ? 0x6fa8ff : 0xff9c6b, transparent: true, opacity: 0.85, depthTest: false,
  }));
  line.renderOrder = 9;
  line.frustumCulled = false;
  pathGroup.add(line);
  return line;
}
function makeReferenceMarker() {
  const mesh = new THREE.Mesh(
    new THREE.OctahedronGeometry(0.009),
    new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false, transparent: true, opacity: 0.9 }),
  );
  mesh.renderOrder = 12;
  mesh.visible = false;
  pathGroup.add(mesh);
  return mesh;
}
function extendTrail(res) {
  for (const side of res.enabled || []) {
    const pts = state.trail[side];
    for (const s of res.samples) if (s.tcp && s.tcp[side]) pts.push(s.tcp[side].p);
    if (pts.length > TRAIL_POINTS) pts.splice(0, pts.length - TRAIL_POINTS);
    drawTrail(side);
  }
}
function drawTrail(side) {
  const line = trailLines[side];
  if (!line) return;
  const pts = state.trail[side];
  const attr = line.geometry.getAttribute('position');
  pts.forEach((p, i) => attr.setXYZ(i, p[0], p[1], p[2]));
  attr.needsUpdate = true;
  line.geometry.setDrawRange(0, pts.length);
}
function clearTrail() {
  for (const side of ['left', 'right']) { state.trail[side] = []; drawTrail(side); }
}

// ------------------------------------------------------------------ dragging a hand in the 3D view
// Grabbing the hand is the same edit as typing into the target boxes: the
// handle sits on the target pose, and dragging it writes that target and asks
// the solver for a pose that reaches it. Motion follows the pointer on the
// plane through the target that faces the camera; orbit the view to move along
// the remaining axis, or grab with Shift held to keep x and y and slide along
// world Z. Only the position is dragged -- the orientation stays whatever the
// target already had, which is what the boxes and the bars edit.
const raycaster = new THREE.Raycaster();
const pointerNdc = new THREE.Vector2();
const dragPlane = new THREE.Plane();
const drag = { side: null, pointerId: null, vertical: false, offset: new THREE.Vector3(), origin: new THREE.Vector3() };

function makeHandle(side) {
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(0.018, 24, 16),
    new THREE.MeshStandardMaterial({
      color: side === 'left' ? 0x6fa8ff : 0xff9c6b,
      roughness: 0.35,
      transparent: true,
      opacity: 0.6,
      // Drawn over everything: a handle swallowed by the gripper mesh is one
      // nobody can find, and the triads are drawn this way too.
      depthTest: false,
      depthWrite: false,
    }),
  );
  mesh.userData.side = side;
  mesh.renderOrder = 11;
  mesh.visible = false;
  handleGroup.add(mesh);
  return mesh;
}

function pointerRay(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  pointerNdc.set(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1,
  );
  raycaster.setFromCamera(pointerNdc, camera);
  return raycaster;
}
function handleUnder(event) {
  const hits = pointerRay(event).intersectObjects(handleGroup.children, false);
  const hit = hits.find((h) => h.object.visible);
  return hit ? hit.object.userData.side : null;
}
function planePoint(event, out) {
  return pointerRay(event).ray.intersectPlane(dragPlane, out) ? out : null;
}

// Client coordinates of a handle, for debugging and the browser test: the same
// projection the renderer uses, so the test clicks where the user would.
state.handleScreen = (side) => {
  const handle = state.handles[side];
  if (!handle || !handle.visible) return null;
  const rect = renderer.domElement.getBoundingClientRect();
  const ndc = handle.position.clone().project(camera);
  return {
    x: rect.left + ((ndc.x + 1) / 2) * rect.width,
    y: rect.top + ((-ndc.y + 1) / 2) * rect.height,
  };
};

function startDrag(event) {
  if (event.button !== undefined && event.button !== 0) return;
  if (!state.model?.ik) return;
  const side = handleUnder(event);
  if (side === null || !state.ee[side].enabled) return;
  ensureTarget(side);
  const target = state.ee[side].target;
  if (!target) return;

  // Stop the event before OrbitControls sees it, otherwise the same press
  // starts an orbit underneath the drag.
  event.stopPropagation();
  event.preventDefault();
  controls.enabled = false;

  const origin = new THREE.Vector3(...target.p);
  const forward = camera.getWorldDirection(new THREE.Vector3()).negate();
  drag.vertical = !!event.shiftKey;      // fixed for the whole drag: a plane swapped mid-drag jumps
  let normal = forward;
  if (drag.vertical) {
    // A plane holding the world Z axis and facing the camera as squarely as it
    // can; looking straight down there is no such plane, so keep the free one.
    const flat = new THREE.Vector3(forward.x, forward.y, 0);
    if (flat.lengthSq() > 1e-6) normal = flat.normalize();
  }
  dragPlane.setFromNormalAndCoplanarPoint(normal, origin);
  const grabbed = planePoint(event, new THREE.Vector3());
  drag.offset.copy(grabbed ? grabbed.sub(origin) : new THREE.Vector3());
  drag.side = side;
  drag.pointerId = event.pointerId;
  drag.origin = origin;
  renderer.domElement.setPointerCapture?.(event.pointerId);
  renderer.domElement.style.cursor = 'grabbing';
  showTab('ee');                          // so the numbers being edited are in view
}

function moveDrag(event) {
  if (drag.side === null) {
    if (event.target === renderer.domElement) {
      const side = handleUnder(event);
      renderer.domElement.style.cursor = side === null ? '' : 'grab';
    }
    return;
  }
  if (drag.pointerId !== null && event.pointerId !== drag.pointerId) return;
  const point = planePoint(event, new THREE.Vector3());
  if (!point) return;
  point.sub(drag.offset);
  if (drag.vertical) { point.x = drag.origin.x; point.y = drag.origin.y; }
  state.ee[drag.side].target.p = [point.x, point.y, point.z];
  writeTargetInputs(drag.side);
  updateTargetFrame(drag.side);
  maybeLiveSolve({ budget: STREAM_BUDGET_S });
}

function endDrag(event) {
  if (drag.side === null) return;
  if (drag.pointerId !== null && event && event.pointerId !== drag.pointerId) return;
  if (drag.pointerId !== null) renderer.domElement.releasePointerCapture?.(drag.pointerId);
  drag.side = null;
  drag.pointerId = null;
  controls.enabled = true;
  renderer.domElement.style.cursor = '';
  // Once on release with the full budget, whatever "move on every jog" says:
  // letting go of a hand somewhere is a request to put it there.
  solveIK();
}

// Capture on the container: at the canvas itself OrbitControls' own listener
// runs first, and stopping propagation there would be too late.
viewport.addEventListener('pointerdown', startDrag, true);
window.addEventListener('pointermove', moveDrag);
window.addEventListener('pointerup', endDrag);
window.addEventListener('pointercancel', endDrag);

// ------------------------------------------------------------------ info + tabs
function buildInfo(model) {
  const dl = $('#info-list');
  const rows = [
    ['URDF', model.urdf], ['nq / nv', `${model.nq} / ${model.nv}`], ['joints', `${model.joints.length} movable`],
    ['shapes', `${model.geometries.length} (${model.geometries.filter((g) => g.shape.type === 'mesh').length} meshes, drawn from <${model.geometry_source}>)`],
    ['joint range', `one resolved range per joint, read by the sliders, the solver and the machine: `
      + `${(model.limits?.sources?.override || []).length} override(s), `
      + `${(model.limits?.sources?.soft || []).length} soft limit(s), `
      + `${model.joints.filter((j) => !j.validated).length} joint(s) NOT validated for the machine`],
    ['limit overrides', (model.limits?.sources?.override || []).length
      ? (model.limits.sources.override.map((n) => { const j = model.limits.joints.find((x) => x.joint === n); return `${n}: ${j.override.reason || '(no reason recorded)'}`; }).join('; '))
      : 'none'],
    ['home pose', Object.keys(model.home_positions_rad || {}).length ? 'configured and checked (Joints: home)' : 'none configured'],
    ['TCP offsets', model.tcp_validated ? 'measured' : 'NOT measured: the end-effector pose is a placeholder frame'],
    ['TCP frames', JSON.stringify(model.tcp_frames)],
    ['IK', model.ik ? `groups: ${JSON.stringify(model.ik.groups)}` : 'not available'],
    ['IK modes', model.ik ? `priority ${model.ik.priority_mode}; orientation ${model.ik.orientation_modes.join(' | ')} (default ${model.ik.orientation_mode})`
      + (model.ik.approach_axis_tcp ? `; approach axis (TCP) [${model.ik.approach_axis_tcp.join(', ')}]` : '; no approach axis stated') : '—'],
    ['collision', model.ik ? (model.ik.collision_modelled ? 'self-collision pairs registered and evaluated' : 'NOT evaluated: no collision pairs registered') : '—'],
    ['trajectory', model.ik ? `${model.ik.trajectory.profile} profile: v ≤ ${model.ik.trajectory.max_linear_velocity_m_s} m/s, a ≤ ${model.ik.trajectory.max_linear_acceleration_m_s2} m/s², ω ≤ ${model.ik.trajectory.max_angular_velocity_rad_s} rad/s, α ≤ ${model.ik.trajectory.max_angular_acceleration_rad_s2} rad/s², sample ${model.ik.trajectory.sample_period_s} s` : '—'],
    ['IK config', model.ik ? JSON.stringify(model.ik.config) : '—'],
  ];
  dl.innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
  $('#info-warnings').innerHTML = model.warnings.map((w) => `<li>${w}</li>`).join('') || '<li class="dim">none</li>';
}
function showTab(name) {
  document.querySelectorAll('.tab').forEach((x) => x.classList.toggle('active', x.dataset.tab === name));
  document.querySelectorAll('.tab-body').forEach((b) => b.classList.toggle('active', b.id === `tab-${name}`));
}
document.querySelectorAll('.tab').forEach((t) => t.addEventListener('click', () => showTab(t.dataset.tab)));

// ------------------------------------------------------------------ boot
(async function boot() {
  try {
    const model = await api('/api/model');
    state.model = model;
    $('#robot-name').textContent = model.robot;
    for (const side of Object.keys(model.tcp_frames)) {
      state.tcpFrames[side] = makeTriad(0.06, 1); frameGroup.add(state.tcpFrames[side]);
      state.targetFrames[side] = makeTriad(0.09, 0.55); frameGroup.add(state.targetFrames[side]);
      // No solver, no drag: moving a target nothing follows would be a lie.
      if (model.ik) {
        state.handles[side] = makeHandle(side);
        trailLines[side] = makeTrail(side);
        referenceMarkers[side] = makeReferenceMarker();
      }
    }
    buildJointsPanel(model);
    buildModeControls(model);
    buildEEPanel(model);
    buildInfo(model);
    const home = model.home_positions_rad || {};
    if (Object.keys(home).length) {
      $('#joints-home').disabled = false;
      $('#joints-home').title = 'Move every joint to the configured home pose (control.model.home_positions_rad)';
    }
    if (!model.ik) { $('#ik-unavailable').hidden = false; $('#ik-unavailable').textContent = 'The solver is not available for this model (see Info). Joint-space control still works.'; }
    else if (model.ik.geometric_study_only) { $('#ik-study').hidden = false; $('#ik-study').textContent = `Continuous joint(s) ${model.ik.groups.unbounded_continuous.join(', ')} have no soft limit: the solver runs as a geometric study only.`; }
    await Promise.all([loadMeshes(model), requestFK()]);
    placeGround();
    fitCamera();
    renderSession();
    // Start-up is one of the explicit resets: the session begins here.
    if (model.ik) sessionReset('start-up');
  } catch (err) {
    $('#overlay').textContent = `failed to load: ${err.message}`;
    console.error(err);
  }
})();
