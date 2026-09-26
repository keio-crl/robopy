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
    right: { enabled: true, target: null, current: null },
  },
  fkInFlight: false,
  fkDirty: false,
  ikInFlight: false,
  ikPending: null,             // options of a solve asked for while one was in flight
  groundZ: 0,                  // where the grid sits: the bottom of the robot's base
  lastFkMs: null,
  lastPoses: null,             // the most recent /api/fk (or IK) reply, re-applied to late meshes
};
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
    const poses = await api('/api/fk', { joints: sent });
    applyPoses(poses, sent);
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
  state.joints[name] = rad;
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
$('#joints-zero').onclick = () => { for (const k of Object.keys(state.joints)) state.joints[k] = 0; refreshJointInputs(); requestFK(); };
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
        <label><input type="checkbox" class="ee-enable" checked> track TCP</label>
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
    box.querySelector('.ee-enable').addEventListener('change', (e) => {
      state.ee[side].enabled = e.target.checked;
      // The inactive TCP may have moved with the torso. Start from its
      // current pose rather than pulling it back to a stale world target.
      if (e.target.checked) captureTarget(side);
      box.querySelectorAll('.tgt input, .tgt button, input.bar').forEach((el) => { el.disabled = !e.target.checked; });
      // Takes the triad *and* the drag handle with it: a hand nothing tracks
      // must not be left with a grabbable target in the scene.
      updateTargetFrame(side);
      maybeLiveSolve();
    });
    box.querySelector('.ee-capture').addEventListener('click', () => { captureTarget(side); });
    box.querySelectorAll('.tgt input').forEach((inp) => inp.addEventListener('change', () => { readTargetInputs(side); maybeLiveSolve(); }));
    box.querySelectorAll('button.jog').forEach((b) => b.addEventListener('click', () => {
      jogTarget(side, b.dataset.jog, parseInt(b.dataset.dir, 10));
    }));
    // Dragging a bar is a stream of small moves: solve each one straight into
    // the scene (no tween, a shorter iteration budget), then once more on
    // release with the full budget so the pose that stays is the solved one.
    box.querySelectorAll('input.bar').forEach((bar) => {
      const set = () => setTargetComponent(side, bar.dataset.bar, parseFloat(bar.value));
      bar.addEventListener('input', () => { set(); maybeLiveSolve({ animate: false, iterations: 150 }); });
      bar.addEventListener('change', () => { set(); maybeLiveSolve({ animate: false }); });
    });
    root.appendChild(box);
  }
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

$('#ori-mode').addEventListener('change', () => maybeLiveSolve());
$('#torso-policy').addEventListener('change', (e) => {
  $('#torso-velocity-wrap').hidden = e.target.value !== 'manual';
  maybeLiveSolve();
});
$('#ee-capture-all').onclick = () => { captureTarget('left'); captureTarget('right'); setIKStatus('targets set to the current poses', 'ok'); };
$('#inactive-arm-policy').addEventListener('change', () => maybeLiveSolve());
$('#ik-solve').onclick = () => solveIK();
function maybeLiveSolve(opts) { if ($('#ik-live').checked) solveIK(opts); }

function setIKStatus(text, kind) {
  const el = $('#ik-status');
  el.textContent = text;
  el.className = `status ${kind || ''}`;
}

// opts.animate: tween to the solved pose (a jog) or show it at once (a drag).
// opts.iterations: solver budget for this request.
// Coalescing, like requestFK: one request in flight, and a solve asked for
// meanwhile runs exactly once more afterwards -- from whatever the targets are
// by then, so a fast drag never queues a backlog of stale poses.
async function solveIK(opts = {}) {
  if (!state.model?.ik) return;
  if (state.ikInFlight) { state.ikPending = opts; return; }
  const { animate = true, iterations = 300 } = opts;
  const targets = {};
  for (const side of ['left', 'right']) {
    if (state.ee[side].enabled && state.ee[side].target) targets[side] = state.ee[side].target;
  }
  if (!Object.keys(targets).length) { setIKStatus('both hands are disabled; nothing to solve', 'warn'); return; }
  state.ikInFlight = true;
  setIKStatus('solving…');
  try {
    const body = {
      joints: state.joints, targets,
      torso_policy: $('#torso-policy').value,
      inactive_arm_policy: $('#inactive-arm-policy').value,
      torso_velocity_rad_s: parseFloat($('#torso-velocity').value) || 0,
      orientation_weight: parseFloat($('#ori-mode').value),
      iterations, dt: 0.02,
    };
    const res = await api('/api/ik', body);
    const e = res.errors;
    const fmt = (m, r) => (m == null ? '—' : `${(m * 1000).toFixed(2)} mm / ${r == null ? '—' : (r * DEG).toFixed(2) + '°'}`);
    const lines = [
      `${res.status.toUpperCase()}${res.stalled ? ' (stalled)' : ''}  ${res.iterations} iter  ${res.timing_ms.toFixed(0)} ms`,
      `left : ${targets.left ? fmt(e.left_position_m, e.left_orientation_rad) : (res.inactive_arm_policy === 'hold_joints' ? 'follows torso; arm joints held' : `hold residual ${fmt(e.left_hold_m, null)}`)}`,
      `right: ${targets.right ? fmt(e.right_position_m, e.right_orientation_rad) : (res.inactive_arm_policy === 'hold_joints' ? 'follows torso; arm joints held' : `hold residual ${fmt(e.right_hold_m, null)}`)}`,
      `torso ω ${res.torso_velocity_rad_s.toFixed(3)} rad/s${res.active_limits.length ? `   limits: ${res.active_limits.join(', ')}` : ''}`,
      res.message ? `\n${res.message}` : '',
      res.stalled && res.status !== 'converged'
        ? '\nNo exact solution from here: the residual stopped changing, so this is the closest pose found.'
          + (res.active_limits.some((l) => l.includes(':at_') || l.endsWith(':lower') || l.endsWith(':upper'))
            ? ' A joint is on its limit (see limits above).'
            : ' The target is probably outside the reachable workspace from this pose -- at q = 0 the arm is fully extended, so bend the elbow first (each elbow bends one way only; see its slider range).')
          + ($('#torso-policy').value === 'fixed' ? ' The torso is fixed, so only this arm\'s six joints may move; torso: optimize lets the shared torso turn, which is what reaches further (the other arm keeps its joint angles and is carried with it).' : '')
          + (res.orientation_weight > 0 ? ' A two-axis wrist also cannot keep the full orientation while translating; orientation: free jogs position only.' : '')
        : '',
    ];
    if (res.commandable) {
      setIKStatus(lines.join('\n'), res.status === 'converged' ? 'ok' : 'warn');
      if (animate) {
        await animateTo(res.joints, 350);
      } else {
        // The reply already carries the poses for the solved configuration, so
        // a drag costs one request per step and no extra FK round trip.
        Object.assign(state.joints, res.joints);
        refreshJointInputs();
        applyPoses(res.poses, null);
      }
    } else {
      setIKStatus(lines.join('\n'), 'bad');
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

// Tween the displayed configuration to the solved one so the motion is
// visible; each frame is an FK request, which is cheap on localhost.
async function animateTo(target, durationMs) {
  const start = { ...state.joints };
  const t0 = performance.now();
  return new Promise((resolve) => {
    const step = async () => {
      const a = Math.min(1, (performance.now() - t0) / durationMs);
      const s = a * a * (3 - 2 * a);
      for (const k of Object.keys(target)) state.joints[k] = start[k] + (target[k] - start[k]) * s;
      refreshJointInputs();
      if (!state.fkInFlight) {
        state.fkInFlight = true;
        const sent = { ...state.joints };
        try { applyPoses(await api('/api/fk', { joints: sent }), sent); } catch (e) { console.warn(e); }
        state.fkInFlight = false;
      }
      if (a < 1) requestAnimationFrame(step); else { Object.assign(state.joints, target); refreshJointInputs(); await requestFK(); resolve(); }
    };
    step();
  });
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
  maybeLiveSolve({ animate: false, iterations: 150 });
}

function endDrag(event) {
  if (drag.side === null) return;
  if (drag.pointerId !== null && event && event.pointerId !== drag.pointerId) return;
  if (drag.pointerId !== null) renderer.domElement.releasePointerCapture?.(drag.pointerId);
  drag.side = null;
  drag.pointerId = null;
  controls.enabled = true;
  renderer.domElement.style.cursor = '';
  // Once on release with the full budget, whatever "solve on every jog" says:
  // letting go of a hand somewhere is a request to put it there.
  solveIK({ animate: false });
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
      if (model.ik) state.handles[side] = makeHandle(side);
    }
    buildJointsPanel(model);
    buildEEPanel(model);
    buildInfo(model);
    if (!model.ik) { $('#ik-unavailable').hidden = false; $('#ik-unavailable').textContent = 'The solver is not available for this model (see Info). Joint-space control still works.'; }
    else if (model.ik.geometric_study_only) { $('#ik-study').hidden = false; $('#ik-study').textContent = `Continuous joint(s) ${model.ik.groups.unbounded_continuous.join(', ')} have no soft limit: the solver runs as a geometric study only.`; }
    await Promise.all([loadMeshes(model), requestFK()]);
    placeGround();
    fitCamera();
  } catch (err) {
    $('#overlay').textContent = `failed to load: ${err.message}`;
    console.error(err);
  }
})();
