// Brickfall: mixed-reality brick smashing for Meta Quest 3 (WebXR, hand tracking only).
// Rendering with three.js; all physics comes from ./physics.js.

import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { World, Body, Vec3, DYNAMIC, KINEMATIC, STATIC, angularVelocityBetween } from './physics.js';

// ---------------------------------------------------------------------------
// Settings (sizes in meters, masses in kilograms)
// ---------------------------------------------------------------------------

const BRICK = { x: 0.10, y: 0.05, z: 0.05, mass: 0.15, friction: 0.6, restitution: 0.1, gap: 0.003 };
const BRICK_COLORS = ['#d8453b', '#f2b33d', '#2f80cf', '#3d9a4f', '#ee7d2b', '#ece5d8'];
const BALL_COLORS = ['#ff5d73', '#ffd166', '#4cc9f0', '#80ed99', '#c77dff', '#ff9f1c'];
const BALL_TYPES = [
  { name: 'Normal', radius: 0.045, mass: 0.3, restitution: 0.45 },
  { name: 'Big', radius: 0.08, mass: 0.6, restitution: 0.5 },
  { name: 'Heavy', radius: 0.05, mass: 2.5, restitution: 0.15 },
  { name: 'Bouncy', radius: 0.035, mass: 0.08, restitution: 0.85 },
];
const GRAVITIES = [{ name: 'Earth', g: 9.81 }, { name: 'Moon', g: 1.62 }, { name: 'Zero', g: 0 }];
const PILES = ['Wall', 'Pyramid', 'Tower', 'Castle'];
const MAX_BALLS = 24;
const GRAB_RANGE = 0.045;        // how close (to the surface) a hand must be to grab something
const THROW_BOOST = 1.15;        // hand tracking smooths motion a little; give throws a bit back
const MAX_THROW_SPEED = 25;
const GROUP_OBJECTS = 1, GROUP_HANDS = 2;

const settings = loadSettings({ pile: 0, ball: 0, gravity: 0, slowmo: false, handPush: true, showRoom: false });

function loadSettings(defaults) {
  try { return { ...defaults, ...JSON.parse(localStorage.getItem('brickfall') || '{}') }; } catch { return { ...defaults }; }
}
function saveSettings() {
  try { localStorage.setItem('brickfall', JSON.stringify(settings)); } catch { /* storage unavailable */ }
}

const UP = new THREE.Vector3(0, 1, 0);
const _v1 = new THREE.Vector3(), _v2 = new THREE.Vector3(), _v3 = new THREE.Vector3();
const _q1 = new THREE.Quaternion(), _q2 = new THREE.Quaternion();
const _m4 = new THREE.Matrix4(), _s1 = new THREE.Vector3(1, 1, 1), _c1 = new THREE.Color();

// ---------------------------------------------------------------------------
// Renderer, scene, lights
// ---------------------------------------------------------------------------

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local-floor');
renderer.xr.setFoveation(0.5);
document.getElementById('app').appendChild(renderer.domElement);

const scene = new THREE.Scene();
const DESKTOP_BG = new THREE.Color('#1d2126');
const DESKTOP_FOG = new THREE.Fog(DESKTOP_BG, 7, 14);
scene.background = DESKTOP_BG;
scene.fog = DESKTOP_FOG;

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.02, 60);
const DESKTOP_VIEW = { pos: new THREE.Vector3(0.3, 1.1, -0.25), target: new THREE.Vector3(-0.3, 0.25, -1.6) };
camera.position.copy(DESKTOP_VIEW.pos);

scene.add(new THREE.HemisphereLight(0xfffaf0, 0x6b6157, 1.5));
const sun = new THREE.DirectionalLight(0xffffff, 2.0);
sun.castShadow = true;
sun.shadow.mapSize.set(1024, 1024);
Object.assign(sun.shadow.camera, { left: -2, right: 2, top: 2, bottom: -2, near: 0.5, far: 10 });
sun.shadow.bias = -0.0005;
sun.shadow.normalBias = 0.01;
scene.add(sun, sun.target);
function aimSun(center) {
  sun.target.position.copy(center);
  sun.position.copy(center).add(_v1.set(1.0, 4.0, 1.4));
}

// Desktop preview floor; in mixed reality only a shadow catcher is drawn over the real floor.
const deskFloor = new THREE.Mesh(new THREE.PlaneGeometry(30, 30), new THREE.MeshStandardMaterial({ color: '#2b3036', roughness: 1 }));
deskFloor.rotation.x = -Math.PI / 2;
deskFloor.receiveShadow = true;
const grid = new THREE.GridHelper(30, 60, '#4a525c', '#363c44');
const desktopOnly = new THREE.Group();
desktopOnly.add(deskFloor, grid);
scene.add(desktopOnly);

const arFloor = new THREE.Mesh(new THREE.PlaneGeometry(30, 30), new THREE.ShadowMaterial({ opacity: 0.32 }));
arFloor.rotation.x = -Math.PI / 2;
arFloor.receiveShadow = true;
arFloor.visible = false;
scene.add(arFloor);

// Writes depth only: hides virtual objects behind real things (your hands, tables, walls).
const occluderMat = new THREE.MeshBasicMaterial({ colorWrite: false });

// ---------------------------------------------------------------------------
// Physics world and objects
// ---------------------------------------------------------------------------

const world = new World();
world.gravity.set(0, -GRAVITIES[settings.gravity].g, 0);
const floorBody = world.addBody(new Body({ shape: 'plane', normal: [0, 1, 0], position: [0, 0, 0], friction: 0.6, restitution: 0.3 }));
let floorY = 0;
function setFloorY(y) {
  if (Math.abs(y - floorY) < 0.002) return;
  floorY = y;
  floorBody.setPose(new Vec3(0, y, 0));
  deskFloor.position.y = y;
  grid.position.y = y + 0.001;
  arFloor.position.y = y + 0.001;
  world.wakeAll();
}
grid.position.y = 0.001;

const bricks = [];
const balls = [];
let ballColorIndex = 0;

const brickMesh = new THREE.InstancedMesh(
  new RoundedBoxGeometry(BRICK.x, BRICK.y, BRICK.z, 2, 0.0045),
  new THREE.MeshStandardMaterial({ roughness: 0.55, metalness: 0 }),
  500);
const ballMesh = new THREE.InstancedMesh(
  new THREE.SphereGeometry(1, 28, 18),
  new THREE.MeshStandardMaterial({ roughness: 0.35, metalness: 0 }),
  MAX_BALLS + 8);
for (const mesh of [brickMesh, ballMesh]) {
  mesh.castShadow = mesh.receiveShadow = true;
  mesh.frustumCulled = false;
  mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  mesh.setColorAt(0, _c1.set('#ffffff'));
  mesh.count = 0;
  scene.add(mesh);
}

world.pairFilter = (a, b) => {
  const ha = a.userData && a.userData.hand, hb = b.userData && b.userData.hand;
  if (!ha && !hb) return true;
  const hand = ha || hb, other = ha ? b : a;
  if (!settings.handPush) return false;
  const ud = other.userData;
  if (ud && ud.heldBy) return false;
  if (ud && ud.thrownBy === hand && performance.now() < ud.thrownUntil) return false;
  return other !== hand.hover;
};

world.onImpact = (a, b, point, speed) => {
  const handHit = (a.userData && a.userData.hand) || (b.userData && b.userData.hand);
  if (handHit && speed < 0.6) return;
  const kind = (a.userData && a.userData.kind === 'ball') || (b.userData && b.userData.kind === 'ball') ? 'ball' : 'brick';
  audio.impact(point, speed, kind);
};

function addBrick(pos, quat, color) {
  const b = new Body({
    shape: 'box', halfExtents: [BRICK.x / 2, BRICK.y / 2, BRICK.z / 2], mass: BRICK.mass,
    friction: BRICK.friction, restitution: BRICK.restitution, position: pos, quaternion: quat,
    linearDamping: 0.02, angularDamping: 0.08, collisionGroup: GROUP_OBJECTS,
    userData: { kind: 'brick', color: new THREE.Color(color), heldBy: null },
  });
  world.addBody(b);
  bricks.push(b);
  return b;
}

function addBall(pos, typeIndex = settings.ball) {
  const t = BALL_TYPES[typeIndex];
  const b = new Body({
    shape: 'sphere', radius: t.radius, mass: t.mass, friction: 0.5, restitution: t.restitution,
    rollingResistance: 0.04, position: pos, linearDamping: 0.01, angularDamping: 0.05,
    collisionGroup: GROUP_OBJECTS,
    userData: { kind: 'ball', color: new THREE.Color(BALL_COLORS[ballColorIndex++ % BALL_COLORS.length]), heldBy: null },
  });
  world.addBody(b);
  balls.push(b);
  while (balls.length > MAX_BALLS) {
    const oldest = balls.find((x) => !x.userData.heldBy);
    if (!oldest) break;
    removeBody(oldest);
  }
  return b;
}

function removeBody(b) {
  if (b.userData.heldBy) release(b.userData.heldBy, false);
  world.removeBody(b);
  const list = b.userData.kind === 'ball' ? balls : bricks;
  const i = list.indexOf(b);
  if (i >= 0) list.splice(i, 1);
  for (const h of hands) if (h.hover === b) h.hover = null;
}

function clearBalls() { for (const b of [...balls]) removeBody(b); }

// ---------------------------------------------------------------------------
// Brick piles
// ---------------------------------------------------------------------------

// Local layout: x to the right, y up from the surface, z toward the player.
function pileLayout(type) {
  const L = BRICK.x, H = BRICK.y, D = BRICK.z, g = BRICK.gap, out = [];
  const add = (x, y, z, rotY = 0) => out.push({ x, y, z, rotY });
  const course = (r) => H / 2 + r * (H + 0.0005);
  if (type === 'Wall') {
    const cols = 8, rows = 8;
    for (let r = 0; r < rows; r++) {
      const n = r % 2 ? cols - 1 : cols;    // running bond
      for (let c = 0; c < n; c++) add((c - (n - 1) / 2) * (L + g), course(r), 0);
    }
  } else if (type === 'Pyramid') {
    const base = 9;
    for (let r = 0; r < base; r++) {
      const n = base - r;
      for (let c = 0; c < n; c++) add((c - (n - 1) / 2) * (L + g), course(r), 0);
    }
  } else if (type === 'Tower') {
    const layers = 14;
    for (let r = 0; r < layers; r++) {
      for (let k = 0; k < 2; k++) {
        const off = (k - 0.5) * (D + g);
        if (r % 2 === 0) add(0, course(r), off, 0); else add(off, course(r), 0, Math.PI / 2);
      }
    }
  } else {
    // Castle: a hollow square with interlocking corners
    const courses = 6, nFull = 3, nIn = 2;
    const wallOff = (nIn * L + (nIn - 1) * g) / 2 + g + D / 2;
    for (let r = 0; r < courses; r++) {
      const swap = r % 2 === 1;
      for (const side of [-1, 1]) {
        for (let c = 0; c < nFull; c++) {
          const t = (c - (nFull - 1) / 2) * (L + g);
          if (swap) add(side * wallOff, course(r), t, Math.PI / 2); else add(t, course(r), side * wallOff, 0);
        }
        for (let c = 0; c < nIn; c++) {
          const t = (c - (nIn - 1) / 2) * (L + g);
          if (swap) add(t, course(r), side * wallOff, 0); else add(side * wallOff, course(r), t, Math.PI / 2);
        }
      }
    }
  }
  return out;
}

function pileSize(type) {
  const box = new THREE.Box3();
  for (const e of pileLayout(type)) {
    const hx = e.rotY ? BRICK.z / 2 : BRICK.x / 2, hz = e.rotY ? BRICK.x / 2 : BRICK.z / 2;
    box.expandByPoint(_v1.set(e.x - hx, e.y - BRICK.y / 2, e.z - hz));
    box.expandByPoint(_v1.set(e.x + hx, e.y + BRICK.y / 2, e.z + hz));
  }
  return box.getSize(new THREE.Vector3());
}

const pile = { origin: new THREE.Vector3(0, 0, -1.6), yaw: 0 };

function buildPile(origin = pile.origin, yaw = pile.yaw) {
  for (const b of [...bricks]) removeBody(b);
  pile.origin.copy(origin);
  pile.yaw = yaw;
  const qYaw = new THREE.Quaternion().setFromAxisAngle(UP, yaw);
  for (const e of pileLayout(PILES[settings.pile])) {
    const p = new THREE.Vector3(e.x, e.y, e.z).applyQuaternion(qYaw).add(pile.origin);
    const q = new THREE.Quaternion().setFromAxisAngle(UP, yaw + e.rotY);
    addBrick(p, q, BRICK_COLORS[Math.floor(Math.random() * BRICK_COLORS.length)]);
  }
  aimSun(pile.origin);
}

function yawFacing(point, target) {
  return Math.atan2(target.x - point.x, target.z - point.z);
}

// ---------------------------------------------------------------------------
// Sound: short filtered-noise clicks, panned to where the impact happened
// ---------------------------------------------------------------------------

const audio = {
  ctx: null, noise: null, master: null, times: [],
  init() {
    if (this.ctx) { this.ctx.resume(); return; }
    const AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) return;
    this.ctx = new AC();
    const len = Math.floor(this.ctx.sampleRate * 0.25);
    this.noise = this.ctx.createBuffer(1, len, this.ctx.sampleRate);
    const data = this.noise.getChannelData(0);
    for (let i = 0; i < len; i++) data[i] = Math.random() * 2 - 1;
    this.master = this.ctx.createGain();
    this.master.gain.value = 0.9;
    this.master.connect(this.ctx.destination);
  },
  ready() { return this.ctx && this.ctx.state === 'running'; },
  impact(pos, speed, kind) {
    if (!this.ready()) return;
    const t = this.ctx.currentTime;
    this.times = this.times.filter((x) => t - x < 0.06);
    if (this.times.length >= 6) return;
    const vol = Math.pow(Math.min(1, speed / 4), 1.3) * (kind === 'ball' ? 0.9 : 0.6);
    if (vol < 0.015) return;
    this.times.push(t);
    const ctx = this.ctx;
    const src = ctx.createBufferSource();
    src.buffer = this.noise;
    src.playbackRate.value = 0.8 + Math.random() * 0.4;
    const filt = ctx.createBiquadFilter();
    filt.type = 'bandpass';
    filt.frequency.value = kind === 'ball' ? 230 + Math.random() * 90 : 1200 + Math.random() * 1000;
    filt.Q.value = kind === 'ball' ? 1.4 : 6;
    const gain = ctx.createGain();
    const decay = kind === 'ball' ? 0.12 : 0.07;
    gain.gain.setValueAtTime(vol * (kind === 'ball' ? 2.5 : 4), t);
    gain.gain.exponentialRampToValueAtTime(0.0005, t + decay);
    const pan = ctx.createPanner();
    pan.panningModel = 'equalpower';
    pan.distanceModel = 'inverse';
    pan.refDistance = 0.6;
    setAudioPos(pan, pos.x, pos.y, pos.z);
    src.connect(filt).connect(gain).connect(pan).connect(this.master);
    src.start(t);
    src.stop(t + decay + 0.05);
  },
  tick(freq = 1700) {
    if (!this.ready()) return;
    const ctx = this.ctx, t = ctx.currentTime;
    const osc = ctx.createOscillator();
    osc.frequency.value = freq;
    const gain = ctx.createGain();
    gain.gain.setValueAtTime(0.12, t);
    gain.gain.exponentialRampToValueAtTime(0.0005, t + 0.05);
    osc.connect(gain).connect(this.master);
    osc.start(t);
    osc.stop(t + 0.06);
  },
  listen(pos, quat) {
    if (!this.ready()) return;
    const l = this.ctx.listener;
    const f = _v2.set(0, 0, -1).applyQuaternion(quat), u = _v3.set(0, 1, 0).applyQuaternion(quat);
    if (l.positionX) {
      l.positionX.value = pos.x; l.positionY.value = pos.y; l.positionZ.value = pos.z;
      l.forwardX.value = f.x; l.forwardY.value = f.y; l.forwardZ.value = f.z;
      l.upX.value = u.x; l.upY.value = u.y; l.upZ.value = u.z;
    } else {
      l.setPosition(pos.x, pos.y, pos.z);
      l.setOrientation(f.x, f.y, f.z, u.x, u.y, u.z);
    }
  },
};
function setAudioPos(node, x, y, z) {
  if (node.positionX) { node.positionX.value = x; node.positionY.value = y; node.positionZ.value = z; } else node.setPosition(x, y, z);
}

// ---------------------------------------------------------------------------
// Hands
// ---------------------------------------------------------------------------

const CHAINS = [
  ['wrist', 'thumb-metacarpal', 'thumb-phalanx-proximal', 'thumb-phalanx-distal', 'thumb-tip'],
  ...['index', 'middle', 'ring', 'pinky'].map((f) => ['wrist', `${f}-finger-metacarpal`, `${f}-finger-phalanx-proximal`,
    `${f}-finger-phalanx-intermediate`, `${f}-finger-phalanx-distal`, `${f}-finger-tip`]),
];
const JOINT_NAMES = ['wrist', ...CHAINS.flatMap((c) => c.slice(1))];
const BONES = CHAINS.flatMap((c) => c.slice(1).map((name, i) => [c[i], name]));
BONES.push(
  ['index-finger-phalanx-proximal', 'middle-finger-phalanx-proximal'],
  ['middle-finger-phalanx-proximal', 'ring-finger-phalanx-proximal'],
  ['ring-finger-phalanx-proximal', 'pinky-finger-phalanx-proximal'],
  ['thumb-phalanx-proximal', 'index-finger-phalanx-proximal'],
);
const PALM_FILL = ['index', 'middle', 'ring', 'pinky'].flatMap((f) => [0.35, 0.7].map((t) => [`${f}-finger-metacarpal`, `${f}-finger-phalanx-proximal`, t]));
const COLLIDERS = [
  ['thumb-tip', 0.010], ['index-finger-tip', 0.009], ['middle-finger-tip', 0.009], ['ring-finger-tip', 0.009],
  ['pinky-finger-tip', 0.008], ['thumb-phalanx-distal', 0.011], ['index-finger-phalanx-intermediate', 0.010],
  ['middle-finger-phalanx-intermediate', 0.010], ['ring-finger-phalanx-intermediate', 0.010],
  ['pinky-finger-phalanx-intermediate', 0.009], ['palm', 0.032],
];

class Hand {
  constructor(index) {
    this.xr = renderer.xr.getHand(index);
    scene.add(this.xr);
    this.handedness = null;
    this.xr.addEventListener('connected', (e) => { if (e.data && e.data.hand) this.handedness = e.data.handedness; });
    this.xr.addEventListener('disconnected', () => { this.handedness = null; });
    this.tracked = false;
    this.lostTime = 0;
    this.p = {};
    this.r = {};
    for (const n of JOINT_NAMES) { this.p[n] = new THREE.Vector3(); this.r[n] = 0.008; }
    this.wristQuat = new THREE.Quaternion();
    this.palm = new THREE.Vector3();
    this.palmNormal = new THREE.Vector3();
    this.fingerDir = new THREE.Vector3();
    this.pinchMid = new THREE.Vector3();
    this.palmGrasp = new THREE.Vector3();
    this.pinching = false; this.fisting = false; this.grabbing = false;
    this.grabRise = false; this.grabFall = false; this.open = false;
    this.suppress = false;   // ignore the current grab until the hand opens again
    this.held = null;
    this.hover = null;

    this.colliders = COLLIDERS.map(([joint, radius]) => {
      const b = new Body({ shape: 'sphere', radius, type: KINEMATIC, position: [0, -50, 0], collisionGroup: GROUP_HANDS, collisionMask: GROUP_OBJECTS, userData: { hand: this } });
      b.joint = joint;
      b.active = false;
      world.addBody(b);
      return b;
    });

    this.occSpheres = new THREE.InstancedMesh(new THREE.SphereGeometry(1, 12, 8), occluderMat, JOINT_NAMES.length + PALM_FILL.length);
    this.occBones = new THREE.InstancedMesh(new THREE.CylinderGeometry(1, 1, 1, 10, 1, true), occluderMat, BONES.length);
    for (const m of [this.occSpheres, this.occBones]) {
      m.renderOrder = -1;
      m.frustumCulled = false;
      m.visible = false;
      m.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
      scene.add(m);
    }
  }

  get isRight() { return this.handedness === 'right'; }

  update(realDt, simDt) {
    const J = this.xr.joints;
    const need = ['wrist', 'thumb-tip', 'index-finger-tip', 'middle-finger-phalanx-proximal', 'pinky-finger-phalanx-proximal'];
    const ok = this.handedness && this.xr.visible && need.every((n) => J[n] && J[n].visible);
    if (!ok) {
      this.lostTime += realDt;
      if (this.tracked) {
        this.tracked = false;
        for (const c of this.colliders) { c.active = false; c.collisionMask = 0; c.setPose(new Vec3(0, -50, 0)); }
        this.occSpheres.visible = this.occBones.visible = false;
      }
      this.grabRise = this.grabFall = false;
      return;
    }
    const firstFrame = !this.tracked;
    this.tracked = true;
    this.lostTime = 0;
    const P = this.p;
    for (const n of JOINT_NAMES) {
      const j = J[n];
      if (j && j.visible) { P[n].copy(j.position); this.r[n] = j.jointRadius || this.r[n]; }
    }
    this.wristQuat.copy(J.wrist.quaternion);

    this.palm.addVectors(P['middle-finger-metacarpal'], P['middle-finger-phalanx-proximal']).multiplyScalar(0.5);
    _v1.subVectors(P['index-finger-phalanx-proximal'], P.wrist);
    _v2.subVectors(P['pinky-finger-phalanx-proximal'], P.wrist);
    this.palmNormal.crossVectors(_v1, _v2).normalize();
    if (!this.isRight) this.palmNormal.negate();
    this.fingerDir.subVectors(P['middle-finger-phalanx-proximal'], P.wrist).normalize();
    this.pinchMid.addVectors(P['thumb-tip'], P['index-finger-tip']).multiplyScalar(0.5);
    this.palmGrasp.copy(this.palm).addScaledVector(this.palmNormal, 0.04);

    const pinchDist = P['thumb-tip'].distanceTo(P['index-finger-tip']);
    const curl = (P['index-finger-tip'].distanceTo(this.palm) + P['middle-finger-tip'].distanceTo(this.palm) + P['ring-finger-tip'].distanceTo(this.palm)) / 3;
    this.pinching = this.pinching ? pinchDist < 0.04 : pinchDist < 0.022;
    this.fisting = this.fisting ? curl < 0.085 : curl < 0.065;
    this.open = curl > 0.08 && pinchDist > 0.05;
    const grabbing = this.pinching || this.fisting;
    this.grabRise = grabbing && !this.grabbing;
    this.grabFall = !grabbing && this.grabbing;
    this.grabbing = grabbing;

    // physics colliders follow the joints
    for (const c of this.colliders) {
      const target = c.joint === 'palm' ? _v1.copy(this.palm).addScaledVector(this.palmNormal, -0.005) : P[c.joint];
      c.collisionMask = settings.handPush ? GROUP_OBJECTS : 0;
      if (firstFrame || !c.active || c.position.distanceTo(target) > 0.25) {
        c.setPose(target);   // teleport: no velocity, no whack
        c.active = true;
      } else {
        c.moveKinematic(target, null, simDt);
      }
    }
    this.updateOccluder();
  }

  updateOccluder() {
    const P = this.p;
    let i = 0;
    for (const n of JOINT_NAMES) {
      const r = this.r[n] * 1.15;
      this.occSpheres.setMatrixAt(i++, _m4.compose(P[n], _q1.identity(), _v1.set(r, r, r)));
    }
    for (const [a, b, t] of PALM_FILL) {
      _v2.lerpVectors(P[a], P[b], t);
      this.occSpheres.setMatrixAt(i++, _m4.compose(_v2, _q1.identity(), _v1.set(0.016, 0.016, 0.016)));
    }
    this.occSpheres.instanceMatrix.needsUpdate = true;
    BONES.forEach(([a, b], k) => {
      _v2.subVectors(P[b], P[a]);
      const len = _v2.length();
      _q1.setFromUnitVectors(UP, _v2.multiplyScalar(1 / Math.max(len, 1e-6)));
      const r = Math.min(this.r[a], this.r[b]) * 1.1;
      _v3.addVectors(P[a], P[b]).multiplyScalar(0.5);
      this.occBones.setMatrixAt(k, _m4.compose(_v3, _q1, _v1.set(r, len, r)));
    });
    this.occBones.instanceMatrix.needsUpdate = true;
    this.occSpheres.visible = this.occBones.visible = true;
  }
}

const hands = [new Hand(0), new Hand(1)];

function surfaceDistance(b, p) {
  if (b.shape === 'sphere') return p.distanceTo(b.position) - b.radius;
  const R = b.R, h = b.halfExtents;
  const dx = p.x - b.position.x, dy = p.y - b.position.y, dz = p.z - b.position.z;
  const lx = R[0] * dx + R[3] * dy + R[6] * dz;
  const ly = R[1] * dx + R[4] * dy + R[7] * dz;
  const lz = R[2] * dx + R[5] * dy + R[8] * dz;
  const ox = Math.max(Math.abs(lx) - h.x, 0), oy = Math.max(Math.abs(ly) - h.y, 0), oz = Math.max(Math.abs(lz) - h.z, 0);
  return Math.sqrt(ox * ox + oy * oy + oz * oz);
}

function findGrabTarget(hand) {
  let best = null, bestD = GRAB_RANGE;
  for (const list of [balls, bricks]) {
    for (const b of list) {
      if (b.userData.heldBy) continue;
      if (hand.palm.distanceToSquared(b.position) > 0.09) continue;
      const d = Math.min(surfaceDistance(b, hand.pinchMid), surfaceDistance(b, hand.palmGrasp));
      if (d < bestD) { bestD = d; best = b; }
    }
  }
  return best;
}

function grab(hand, body) {
  if (body.userData.heldBy) release(body.userData.heldBy, false);
  body.setType(KINEMATIC);
  body.userData.heldBy = hand;
  const inv = _q1.copy(hand.wristQuat).invert();
  hand.held = {
    body,
    offPos: new THREE.Vector3().copy(body.position).sub(hand.p.wrist).applyQuaternion(inv),
    offQuat: new THREE.Quaternion().copy(inv).multiply(_q2.copy(body.quaternion)),
    hist: [],
  };
  hand.hover = null;
  audio.tick(1400);
}

function updateHeld(hand, simDt, now) {
  const h = hand.held;
  const tp = _v1.copy(h.offPos).applyQuaternion(hand.wristQuat).add(hand.p.wrist);
  const tq = _q1.copy(hand.wristQuat).multiply(h.offQuat);
  h.body.moveKinematic(tp, tq, simDt);
  h.hist.push({ t: now, x: tp.x, y: tp.y, z: tp.z, q: tq.clone() });
  while (h.hist.length > 2 && now - h.hist[0].t > 0.25) h.hist.shift();
}

// Peak velocity over the last ~120 ms: tracking reports the hand opening late,
// when the hand is already slowing down.
function estimateThrow(hist, now) {
  const v = new THREE.Vector3(), w = new THREE.Vector3();
  if (hist.length < 4) return { v, w };
  let best = -1;
  for (let i = hist.length - 1; i >= 3 && now - hist[i].t < 0.12; i--) {
    const a = hist[i - 3], b = hist[i], dt = b.t - a.t;
    if (dt < 1e-3) continue;
    const vx = (b.x - a.x) / dt, vy = (b.y - a.y) / dt, vz = (b.z - a.z) / dt;
    const s2 = vx * vx + vy * vy + vz * vz;
    if (s2 > best) { best = s2; v.set(vx, vy, vz); }
  }
  const a = hist[hist.length - 3], b = hist[hist.length - 1];
  if (b.t - a.t > 1e-3) angularVelocityBetween(a.q, b.q, b.t - a.t, w);
  v.multiplyScalar(THROW_BOOST);
  if (v.length() > MAX_THROW_SPEED) v.setLength(MAX_THROW_SPEED);
  if (w.length() > 40) w.setLength(40);
  return { v, w };
}

function release(hand, throwIt, est) {
  const h = hand.held;
  if (!h) return;
  hand.held = null;
  const b = h.body;
  b.userData.heldBy = null;
  b.userData.thrownBy = hand;               // fingers opening shouldn't bat the ball
  b.userData.thrownUntil = performance.now() + 300;
  if (!b.world) return;
  b.setType(DYNAMIC);
  if (throwIt) {
    est = est || estimateThrow(h.hist, performance.now() / 1000);
    b.velocity.copy(est.v);
    b.angularVelocity.copy(est.w);
  } else {
    b.velocity.set(0, 0, 0);
    b.angularVelocity.set(0, 0, 0);
  }
  b.wake();
}

function ballSpawnPoint(hand, radius) {
  if (hand.fisting) return new THREE.Vector3().copy(hand.palm).addScaledVector(hand.palmNormal, radius + 0.012).addScaledVector(hand.fingerDir, 0.02);
  return new THREE.Vector3().copy(hand.pinchMid).addScaledVector(hand.fingerDir, radius * 0.7).addScaledVector(hand.palmNormal, radius * 0.3);
}

function handleHand(hand, simDt, now) {
  if (!hand.tracked) {
    hand.hover = null;
    if (hand.held) {
      // Fast throws often outrun hand tracking: throw with the last known motion.
      const est = estimateThrow(hand.held.hist, now);
      if (est.v.length() > 1.2 || hand.lostTime > 0.35) release(hand, true, est);
    }
    return;
  }
  const menuHand = menu.hand === hand;
  const poking = menu.isNear(hand);
  const busy = menuHand || poking;
  hand.hover = (!hand.held && !busy && !placement.active) ? findGrabTarget(hand) : null;

  if (hand.suppress) {
    if (!hand.grabbing) hand.suppress = false;
  } else if (hand.grabRise && !hand.held && !busy) {
    if (placement.active) {
      if (placement.hand === hand) placement.confirm();
      hand.suppress = true;
    } else if (hand.hover) {
      grab(hand, hand.hover);
    } else {
      const ball = addBall(ballSpawnPoint(hand, BALL_TYPES[settings.ball].radius));
      grab(hand, ball);
    }
  }
  if (hand.held && hand.grabFall) release(hand, true);
  if (hand.held) updateHeld(hand, simDt, now);
}

// ---------------------------------------------------------------------------
// Text panels (canvas textures)
// ---------------------------------------------------------------------------

function makeLabel(w, h, pxW, pxH) {
  const canvas = document.createElement('canvas');
  canvas.width = pxW; canvas.height = pxH;
  const ctx = canvas.getContext('2d');
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
  mesh.renderOrder = 10;
  return { canvas, ctx, tex, mesh, text: null };
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

const FONT = '"Figtree", "Segoe UI", system-ui, sans-serif';

// ---------------------------------------------------------------------------
// Wrist menu: turn a palm toward your face, poke buttons with the other index finger
// ---------------------------------------------------------------------------

const menuDefs = [
  { label: () => 'Rebuild', run: () => buildPile() },
  { label: () => (placement.active ? 'Cancel move' : 'Move pile'), run: () => placement.toggle() },
  { label: () => `Pile: ${PILES[settings.pile]}`, run: () => { settings.pile = (settings.pile + 1) % PILES.length; buildPile(); } },
  { label: () => 'Clear balls', run: () => clearBalls() },
  { label: () => `Ball: ${BALL_TYPES[settings.ball].name}`, run: () => { settings.ball = (settings.ball + 1) % BALL_TYPES.length; } },
  { label: () => `Gravity: ${GRAVITIES[settings.gravity].name}`, run: () => cycleGravity() },
  { label: () => `Slow-mo: ${settings.slowmo ? 'On' : 'Off'}`, run: () => { settings.slowmo = !settings.slowmo; } },
  { label: () => `Hand push: ${settings.handPush ? 'On' : 'Off'}`, run: () => { settings.handPush = !settings.handPush; } },
  { label: () => (room.hasPlanes || !room.canCapture() ? `Room: ${settings.showRoom ? 'Shown' : 'Hidden'}` : 'Scan room'), run: () => room.buttonPressed() },
  { label: () => 'Exit MR', run: () => { const s = renderer.xr.getSession(); if (s) s.end(); } },
];

function cycleGravity() {
  settings.gravity = (settings.gravity + 1) % GRAVITIES.length;
  world.gravity.set(0, -GRAVITIES[settings.gravity].g, 0);
  world.wakeAll();
}

class Menu {
  constructor(defs) {
    this.group = new THREE.Group();
    this.group.visible = false;
    scene.add(this.group);
    this.hand = null;
    this.showT = 0; this.hideT = 0; this.snap = false; this.lastPress = 0; this.statsT = 0;
    const BW = 0.07, BH = 0.03, GAP = 0.006, PAD = 0.01, TITLE = 0.026;
    const rows = Math.ceil(defs.length / 2);
    const pw = 2 * BW + GAP + 2 * PAD, ph = rows * BH + (rows - 1) * GAP + 2 * PAD + TITLE;
    const bg = new THREE.Mesh(new THREE.PlaneGeometry(pw, ph), new THREE.MeshBasicMaterial({ color: '#15181c', transparent: true, opacity: 0.9, depthWrite: false }));
    bg.renderOrder = 9;
    this.group.add(bg);
    this.title = makeLabel(pw - 2 * PAD, TITLE * 0.8, 512, 64);
    this.title.mesh.position.set(0, ph / 2 - PAD - TITLE * 0.4, 0.001);
    this.group.add(this.title.mesh);
    const top = ph / 2 - PAD - TITLE - BH / 2;
    this.buttons = defs.map((def, i) => {
      const lbl = makeLabel(BW, BH, 256, 110);
      const x = (i % 2 - 0.5) * (BW + GAP), y = top - Math.floor(i / 2) * (BH + GAP);
      lbl.mesh.position.set(x, y, 0.002);
      this.group.add(lbl.mesh);
      return { def, ...lbl, x, y, w: BW, h: BH, armed: true, hot: false, down: false, state: '' };
    });
    this.refresh();
  }

  drawButton(b) {
    const text = b.def.label();
    const state = `${text}|${b.hot}|${b.down}`;
    if (state === b.state) return;
    b.state = state;
    const { ctx, canvas } = b;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    roundRect(ctx, 4, 4, canvas.width - 8, canvas.height - 8, 22);
    ctx.fillStyle = b.down ? '#f2b33d' : b.hot ? '#3a424c' : '#262b32';
    ctx.fill();
    ctx.fillStyle = b.down ? '#241a05' : '#eceae4';
    ctx.font = `600 40px ${FONT}`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, canvas.width / 2, canvas.height / 2 + 2, canvas.width - 24);
    b.tex.needsUpdate = true;
  }

  drawTitle() {
    const t = this.title, ctx = t.ctx, c = t.canvas;
    const text = `${bricks.length} bricks · ${balls.length} balls · ${stats.stepMs.toFixed(1)} ms`;
    if (text === t.text) return;
    t.text = text;
    ctx.clearRect(0, 0, c.width, c.height);
    ctx.fillStyle = '#f2b33d';
    ctx.font = `800 40px "Bricolage Grotesque", ${FONT}`;
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'left';
    ctx.fillText('Brickfall', 6, c.height / 2);
    ctx.fillStyle = '#a3a8ae';
    ctx.font = `500 26px ${FONT}`;
    ctx.textAlign = 'right';
    ctx.fillText(text, c.width - 6, c.height / 2);
    t.tex.needsUpdate = true;
  }

  refresh() { for (const b of this.buttons) this.drawButton(b); this.drawTitle(); }

  other(hand) { return hands.find((h) => h !== hand && h.tracked) || null; }

  isNear(hand) {
    if (!this.group.visible || hand === this.hand || !hand.tracked) return false;
    return hand.p['index-finger-tip'].distanceTo(this.group.position) < 0.16;
  }

  facing(hand, head) {
    _v1.subVectors(head.pos, hand.palm).normalize();
    return hand.palmNormal.dot(_v1);
  }

  update(dt, now, head) {
    // who owns the menu?
    let want = null;
    const owner = this.hand;
    if (owner && owner.tracked && !owner.held && this.facing(owner, head) > 0.45) want = owner;
    else {
      for (const h of hands) {
        if (h.tracked && !h.held && h.open && this.facing(h, head) > 0.75 && (!want || h.handedness === 'left')) want = h;
      }
    }
    if (want) {
      this.hideT = 0;
      if (want !== owner) {
        this.showT += dt;
        if (this.showT > 0.12) { this.hand = want; this.showT = 0; this.group.visible = true; this.snap = true; this.refresh(); }
      }
    } else {
      this.showT = 0;
      if (owner) { this.hideT += dt; if (this.hideT > 0.25) { this.hand = null; this.group.visible = false; } }
    }
    if (!this.hand) return;

    const h = this.hand, poker = this.other(h);
    const tip = poker ? poker.p['index-finger-tip'] : null;
    const near = tip && tip.distanceTo(this.group.position) < 0.14;
    if (this.snap || !near) {
      // float just above the palm, facing your eyes; holds still while you aim at it
      const target = _v2.copy(h.palm).addScaledVector(h.palmNormal, 0.05).addScaledVector(UP, 0.1);
      if (this.snap) this.group.position.copy(target);
      else this.group.position.lerp(target, 1 - Math.exp(-dt * 18));
      this.group.lookAt(head.pos);
      this.snap = false;
    }
    this.group.updateMatrixWorld();

    for (const b of this.buttons) { b.hot = false; b.down = false; }
    if (tip) {
      const local = this.group.worldToLocal(_v3.copy(tip));
      for (const b of this.buttons) {
        const inside = Math.abs(local.x - b.x) < b.w / 2 + 0.003 && Math.abs(local.y - b.y) < b.h / 2 + 0.003;
        if (inside && local.z < 0.04) b.hot = true;
        if (inside && local.z < 0.006 && local.z > -0.05) {
          b.down = true;
          if (b.armed && now - this.lastPress > 0.3) {
            b.armed = false;
            this.lastPress = now;
            audio.tick();
            b.def.run();
            saveSettings();
          }
        } else if (!inside || local.z > 0.012) {
          b.armed = true;
        }
      }
    }
    for (const b of this.buttons) b.mesh.position.z = b.down ? -0.002 : 0.002;
    this.statsT += dt;
    if (this.statsT > 0.5) { this.statsT = 0; this.drawTitle(); }
    for (const b of this.buttons) this.drawButton(b);
  }
}

// Short messages that float below your line of sight.
class Toast {
  constructor() {
    this.label = makeLabel(0.36, 0.07, 720, 140);
    this.label.mesh.renderOrder = 11;
    this.label.mesh.visible = false;
    scene.add(this.label.mesh);
    this.until = 0;
  }
  show(text, seconds = 4) {
    const { ctx, canvas, tex } = this.label;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    roundRect(ctx, 4, 4, canvas.width - 8, canvas.height - 8, 30);
    ctx.fillStyle = 'rgba(21,24,28,0.9)';
    ctx.fill();
    ctx.fillStyle = '#eceae4';
    ctx.font = `500 34px ${FONT}`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const words = text.split(' '), lines = [''];
    for (const w of words) {
      const test = lines[lines.length - 1] ? `${lines[lines.length - 1]} ${w}` : w;
      if (ctx.measureText(test).width > canvas.width - 60 && lines[lines.length - 1]) lines.push(w); else lines[lines.length - 1] = test;
    }
    lines.slice(0, 3).forEach((l, i, arr) => ctx.fillText(l, canvas.width / 2, canvas.height / 2 + (i - (arr.length - 1) / 2) * 40));
    tex.needsUpdate = true;
    this.until = performance.now() / 1000 + seconds;
    this.snap = true;
    this.label.mesh.visible = renderer.xr.isPresenting;
  }
  update(dt, now, head) {
    const m = this.label.mesh;
    if (!renderer.xr.isPresenting || now > this.until) { m.visible = false; return; }
    m.visible = true;
    const fwd = _v1.set(0, 0, -1).applyQuaternion(head.quat);
    fwd.y = 0;
    if (fwd.lengthSq() < 1e-4) fwd.set(0, 0, -1);
    fwd.normalize();
    const target = _v2.copy(head.pos).addScaledVector(fwd, 0.7).addScaledVector(UP, -0.2);
    if (this.snap) m.position.copy(target); else m.position.lerp(target, 1 - Math.exp(-dt * 4));
    m.lookAt(head.pos);
    this.snap = false;
  }
}

// ---------------------------------------------------------------------------
// Moving the pile: point with a hand ray, pinch to drop it there
// ---------------------------------------------------------------------------

const placement = {
  active: false,
  hand: null,
  valid: false,
  point: new THREE.Vector3(),
  ghost: new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)), new THREE.LineBasicMaterial({ color: '#f2b33d' })),
  ray: new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 0, -1)]), new THREE.LineBasicMaterial({ color: '#f2b33d', transparent: true, opacity: 0.6 })),
  init() {
    this.ghost.visible = this.ray.visible = false;
    this.ray.frustumCulled = false;
    scene.add(this.ghost, this.ray);
  },
  toggle() {
    this.active = !this.active;
    if (this.active) toast.show('Point at the floor or a table, then pinch to put the pile there.');
  },
  update(head) {
    this.ghost.visible = this.ray.visible = false;
    this.valid = false;
    if (!this.active) return;
    const candidates = hands.filter((h) => h.tracked && h !== menu.hand);
    const hand = candidates.find((h) => h.isRight) || candidates[0];
    this.hand = hand || null;
    if (!hand) return;
    const origin = hand.p['index-finger-phalanx-proximal'];
    const right = _v1.set(1, 0, 0).applyQuaternion(head.quat);
    const shoulder = _v2.copy(head.pos).addScaledVector(right, hand.isRight ? 0.17 : -0.17).addScaledVector(UP, -0.22);
    const dir = _v3.subVectors(origin, shoulder).normalize();
    const hit = world.raycast(origin, dir, 8, (b) => b.type === STATIC);
    const end = hit ? hit.point : _v2.copy(origin).addScaledVector(dir, 3);
    const pos = this.ray.geometry.attributes.position;
    pos.setXYZ(0, origin.x, origin.y, origin.z);
    pos.setXYZ(1, end.x, end.y, end.z);
    pos.needsUpdate = true;
    this.ray.visible = true;
    if (hit && hit.normal.y > 0.7) {
      this.valid = true;
      this.point.set(hit.point.x, hit.point.y, hit.point.z);
      const size = pileSize(PILES[settings.pile]);
      this.ghost.scale.copy(size);
      this.ghost.position.copy(this.point).addScaledVector(UP, size.y / 2);
      this.ghost.rotation.set(0, yawFacing(this.point, head.pos), 0);
      this.ghost.visible = true;
    }
  },
  confirm() {
    if (!this.valid) { toast.show('Aim at the floor or a tabletop.', 2.5); return; }
    buildPile(this.point, yawFacing(this.point, head.pos));
    this.active = false;
    menu.refresh();
  },
};

// ---------------------------------------------------------------------------
// The room: Space Setup planes become static colliders, occluders and shadow catchers
// ---------------------------------------------------------------------------

const ROOM_COLORS = { floor: '#3d9a4f', wall: '#2f80cf', ceiling: '#7a7f86', table: '#f2b33d', couch: '#ee7d2b' };

const room = {
  entries: new Map(),
  group: new THREE.Group(),
  hasPlanes: false,
  checkT: 0,
  startT: 0,
  warned: false,
  init() { scene.add(this.group); },
  canCapture() {
    const s = renderer.xr.getSession();
    return !!(s && typeof s.initiateRoomCapture === 'function' && !this.captured);
  },
  buttonPressed() {
    if (!this.hasPlanes && this.canCapture()) {
      this.captured = true;
      renderer.xr.getSession().initiateRoomCapture().catch(() => toast.show('Room scan is not available here.'));
      return;
    }
    settings.showRoom = !settings.showRoom;
    for (const e of this.entries.values()) for (const m of e.debug) m.visible = settings.showRoom;
  },
  reset() {
    for (const e of this.entries.values()) this.remove(e);
    this.entries.clear();
    this.hasPlanes = false;
    this.warned = false;
    this.captured = false;
    this.startT = performance.now() / 1000;
  },
  remove(e) {
    if (e.body) world.removeBody(e.body);
    this.group.remove(e.node);
    e.node.traverse((o) => { if (o.geometry) o.geometry.dispose(); });
  },
  update(frame, refSpace, now) {
    if (!frame || !refSpace) return;
    let planes = null;
    try { planes = frame.detectedPlanes; } catch { planes = null; }
    if (!planes || planes.size === 0) {
      if (!this.warned && now - this.startT > 4) {
        this.warned = true;
        toast.show('No room scan found. Only the floor is solid. Run Space Setup in Quest settings to add walls and tables.', 7);
      }
      return;
    }
    this.checkT -= 1;
    if (this.checkT > 0) return;
    this.checkT = 30;
    let changed = false;
    for (const plane of planes) {
      const pose = frame.getPose(plane.planeSpace, refSpace);
      if (!pose) continue;
      const e = this.entries.get(plane);
      const p = pose.transform.position;
      if (e && e.changed === plane.lastChangedTime && Math.abs(e.pos.x - p.x) + Math.abs(e.pos.y - p.y) + Math.abs(e.pos.z - p.z) < 0.01) continue;
      if (e) this.remove(e);
      const ne = this.build(plane, pose);
      if (ne) this.entries.set(plane, ne); else this.entries.delete(plane);
      changed = true;
    }
    for (const [plane, e] of this.entries) {
      if (!planes.has(plane)) { this.remove(e); this.entries.delete(plane); changed = true; }
    }
    if (changed) {
      if (!this.hasPlanes) toast.show('Room loaded: walls, tables and floor are solid.', 3);
      this.hasPlanes = this.entries.size > 0;
      let fy = Infinity;
      for (const e of this.entries.values()) if (e.isFloor) fy = Math.min(fy, e.pos.y);
      if (fy < Infinity) setFloorY(fy);
      world.wakeAll();
      menu.refresh();
    }
  },
  build(plane, pose) {
    const poly = plane.polygon;
    if (!poly || poly.length < 3) return null;
    const label = String(plane.semanticLabel || plane.orientation || '').toLowerCase();
    const t = pose.transform;
    const pos = new THREE.Vector3(t.position.x, t.position.y, t.position.z);
    const quat = new THREE.Quaternion(t.orientation.x, t.orientation.y, t.orientation.z, t.orientation.w);
    const normal = new THREE.Vector3(0, 1, 0).applyQuaternion(quat);
    const vertical = plane.orientation === 'vertical' || Math.abs(normal.y) < 0.5;
    const facingUp = normal.y > 0.9;
    const isFloor = label === 'floor' || (facingUp && pos.y < 0.2 && label !== 'table');
    let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
    for (const p of poly) { minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x); minZ = Math.min(minZ, p.z); maxZ = Math.max(maxZ, p.z); }
    const e = { pos, label, isFloor, changed: plane.lastChangedTime, body: null, debug: [], node: new THREE.Group() };

    if (!isFloor) {
      // A slab behind the surface, thick enough that fast balls can't pass through.
      const T = vertical || normal.y < -0.5 ? 0.3 : 0.15;
      const pad = vertical ? 0.05 : 0;
      const center = new THREE.Vector3((minX + maxX) / 2, -T / 2, (minZ + maxZ) / 2).applyQuaternion(quat).add(pos);
      e.body = world.addBody(new Body({
        shape: 'box', type: STATIC, halfExtents: [(maxX - minX) / 2 + pad, T / 2, (maxZ - minZ) / 2 + pad],
        position: center, quaternion: quat, friction: 0.6, restitution: 0.3, userData: { kind: 'room' },
      }));
    }

    const pts2 = poly.map((p) => new THREE.Vector2(p.x, p.z));
    const tris = THREE.ShapeUtils.triangulateShape(pts2, []);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(poly.flatMap((p) => [p.x, 0, p.z]), 3));
    geo.setIndex(tris.flat());
    geo.computeVertexNormals();
    e.node.position.copy(pos);
    e.node.quaternion.copy(quat);
    if (!isFloor) {
      const occ = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ colorWrite: false, side: THREE.DoubleSide }));
      occ.renderOrder = -1;
      e.node.add(occ);
      if (facingUp) {
        const shadow = new THREE.Mesh(geo, new THREE.ShadowMaterial({ opacity: 0.32, side: THREE.DoubleSide }));
        shadow.position.y = 0.002;
        shadow.receiveShadow = true;
        e.node.add(shadow);
      }
    }
    const color = ROOM_COLORS[label] || (vertical ? ROOM_COLORS.wall : '#c77dff');
    const fill = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false }));
    fill.position.y = 0.003;
    const outline = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(poly.map((p) => new THREE.Vector3(p.x, 0.004, p.z))), new THREE.LineBasicMaterial({ color }));
    for (const m of [fill, outline]) { m.visible = settings.showRoom; e.debug.push(m); e.node.add(m); }
    this.group.add(e.node);
    return e;
  },
};

// ---------------------------------------------------------------------------
// Frame loop
// ---------------------------------------------------------------------------

const head = { pos: new THREE.Vector3(0, 1.6, 0), quat: new THREE.Quaternion(), valid: false };
const stats = { stepMs: 0 };
const menu = new Menu(menuDefs);
const toast = new Toast();
placement.init();
room.init();
let lastTime = performance.now();
let dtSmooth = 1 / 72;
let initialPileAt = 0;   // when (seconds) to place the first pile after entering MR
let noHandsSince = 0, noHandsWarnedAt = 0;
const highlight = new THREE.Color('#ffffff');

function syncVisuals() {
  let i = 0;
  for (const b of bricks) {
    brickMesh.setMatrixAt(i, _m4.compose(b.position, _q1.copy(b.quaternion), _s1.set(1, 1, 1)));
    _c1.copy(b.userData.color);
    if (hands[0].hover === b || hands[1].hover === b) _c1.lerp(highlight, 0.45);
    brickMesh.setColorAt(i, _c1);
    i++;
  }
  brickMesh.count = i;
  i = 0;
  for (const b of balls) {
    ballMesh.setMatrixAt(i, _m4.compose(b.position, _q1.copy(b.quaternion), _s1.set(b.radius, b.radius, b.radius)));
    _c1.copy(b.userData.color);
    if (hands[0].hover === b || hands[1].hover === b) _c1.lerp(highlight, 0.45);
    ballMesh.setColorAt(i, _c1);
    i++;
  }
  ballMesh.count = i;
  for (const m of [brickMesh, ballMesh]) {
    m.instanceMatrix.needsUpdate = true;
    if (m.instanceColor) m.instanceColor.needsUpdate = true;
  }
}

function cleanup() {
  for (const list of [bricks, balls]) {
    for (let i = list.length - 1; i >= 0; i--) {
      const b = list[i];
      if (b.position.y < floorY - 3 || Math.abs(b.position.x) > 40 || Math.abs(b.position.z) > 40 || b.position.y > 40) removeBody(b);
    }
  }
}

// Fewer substeps when a big collapse makes the solver expensive.
function adaptSubsteps() {
  stats.stepMs += (world.stepTimeMs - stats.stepMs) * 0.1;
  if (world.substeps === 6 && stats.stepMs > 4.5) world.substeps = 4;
  else if (world.substeps === 4 && stats.stepMs < 2.0) world.substeps = 6;
}

renderer.setAnimationLoop((time, frame) => {
  const nowMs = performance.now();
  const rawDt = Math.min(Math.max((nowMs - lastTime) / 1000, 1 / 144), 1 / 30);
  lastTime = nowMs;
  dtSmooth += (rawDt - dtSmooth) * 0.1;
  const realDt = dtSmooth;
  const simDt = Math.min(realDt, 1 / 45) * (settings.slowmo ? 0.25 : 1);
  const now = nowMs / 1000;
  const xr = renderer.xr.isPresenting;

  if (xr && frame) {
    const ref = renderer.xr.getReferenceSpace();
    const pose = frame.getViewerPose(ref);
    if (pose) {
      const t = pose.transform;
      head.pos.set(t.position.x, t.position.y, t.position.z);
      head.quat.set(t.orientation.x, t.orientation.y, t.orientation.z, t.orientation.w);
      head.valid = true;
    }
    room.update(frame, ref, now);
    for (const h of hands) h.update(realDt, simDt);
    if (hands.some((h) => h.tracked)) noHandsSince = now;
    else if (now - noHandsSince > 3 && now - noHandsWarnedAt > 15) {
      noHandsWarnedAt = now;
      toast.show('No hands found. Put the controllers down and hold your hands up in front of you.', 5);
    }
    menu.update(realDt, now, head);
    for (const h of hands) handleHand(h, simDt, now);
    placement.update(head);
    toast.update(realDt, now, head);
    if (initialPileAt && now > initialPileAt && head.valid) {
      initialPileAt = 0;
      const fwd = _v1.set(0, 0, -1).applyQuaternion(head.quat);
      fwd.y = 0;
      if (fwd.lengthSq() < 1e-4) fwd.set(0, 0, -1);
      fwd.normalize();
      const origin = new THREE.Vector3(head.pos.x, floorY, head.pos.z).addScaledVector(fwd, 1.4);
      buildPile(origin, yawFacing(origin, head.pos));
      toast.show('Pinch or make a fist to get a ball. Palm toward your face opens the menu.', 6);
    }
  } else {
    controls.update();
    head.pos.copy(camera.position);
    head.quat.copy(camera.quaternion);
  }

  world.step(simDt);
  adaptSubsteps();
  cleanup();
  syncVisuals();
  audio.listen(head.pos, head.quat);
  renderer.render(scene, camera);
});

// ---------------------------------------------------------------------------
// Entering and leaving mixed reality
// ---------------------------------------------------------------------------

const overlay = document.getElementById('overlay');
const enterBtn = document.getElementById('enter');
const statusEl = document.getElementById('status');

function setStatus(text, warn = false) {
  statusEl.textContent = text;
  statusEl.classList.toggle('warn', warn);
}

async function checkSupport() {
  if (!window.isSecureContext) {
    enterBtn.textContent = 'Needs HTTPS';
    setStatus('WebXR only works on https:// pages (or localhost). Host these files on an HTTPS server.', true);
    return;
  }
  if (!navigator.xr) {
    enterBtn.textContent = 'Mixed reality unavailable';
    setStatus('Open this page in the Meta Quest Browser to play in your room. The preview below works with a mouse.');
    return;
  }
  let ok = false;
  try { ok = await navigator.xr.isSessionSupported('immersive-ar'); } catch { ok = false; }
  if (ok) {
    enterBtn.disabled = false;
    enterBtn.textContent = 'Enter mixed reality';
    setStatus('Put the controllers down: this runs on hand tracking.');
  } else {
    enterBtn.textContent = 'Mixed reality unavailable';
    setStatus('This browser has WebXR but not passthrough (immersive-ar). Use the Meta Quest Browser on Quest 3.');
  }
}

async function enterMR() {
  audio.init();
  try {
    const session = await navigator.xr.requestSession('immersive-ar', {
      requiredFeatures: ['local-floor'],
      optionalFeatures: ['hand-tracking', 'plane-detection'],
    });
    session.addEventListener('end', onSessionEnd);
    await renderer.xr.setSession(session);
    try {
      if (session.updateTargetFrameRate && session.supportedFrameRates && Array.from(session.supportedFrameRates).includes(90)) {
        await session.updateTargetFrameRate(90);
      }
    } catch { /* keep the default rate */ }
    scene.background = null;
    scene.fog = null;
    desktopOnly.visible = false;
    arFloor.visible = true;
    overlay.hidden = true;
    controls.enabled = false;
    room.reset();
    clearBalls();
    initialPileAt = performance.now() / 1000 + 1.5;
    noHandsSince = performance.now() / 1000;
  } catch (err) {
    setStatus(`Could not start mixed reality: ${err && err.message ? err.message : err}`, true);
  }
}

function onSessionEnd() {
  room.reset();
  for (const h of hands) { if (h.held) release(h, false); h.hover = null; }
  menu.hand = null;
  menu.group.visible = false;
  placement.active = false;
  scene.background = DESKTOP_BG;
  scene.fog = DESKTOP_FOG;
  desktopOnly.visible = true;
  camera.position.copy(DESKTOP_VIEW.pos);
  controls.target.copy(DESKTOP_VIEW.target);
  arFloor.visible = false;
  overlay.hidden = false;
  controls.enabled = true;
  setFloorY(0);
  clearBalls();
  buildPile(new THREE.Vector3(0, 0, -1.6), 0);
}

enterBtn.addEventListener('click', enterMR);
checkSupport();

// ---------------------------------------------------------------------------
// Desktop preview: orbit, click to throw, keyboard shortcuts
// ---------------------------------------------------------------------------

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.copy(DESKTOP_VIEW.target);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * 0.49;
controls.update();

let downAt = null;
renderer.domElement.addEventListener('pointerdown', (e) => { downAt = { x: e.clientX, y: e.clientY }; audio.init(); });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downAt || renderer.xr.isPresenting || e.button !== 0) return;
  const moved = Math.hypot(e.clientX - downAt.x, e.clientY - downAt.y);
  downAt = null;
  if (moved > 6) return;
  const ndc = new THREE.Vector2((e.clientX / window.innerWidth) * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1);
  const ray = new THREE.Raycaster();
  ray.setFromCamera(ndc, camera);
  const dir = ray.ray.direction;
  const ball = addBall(new THREE.Vector3().copy(camera.position).addScaledVector(dir, 0.3));
  ball.velocity.set(dir.x * 9, dir.y * 9 + 1.0, dir.z * 9);
});

window.addEventListener('keydown', (e) => {
  if (renderer.xr.isPresenting || e.metaKey || e.ctrlKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (k === 'r') buildPile();
  else if (k === 'p') { settings.pile = (settings.pile + 1) % PILES.length; buildPile(); }
  else if (k === 'b') settings.ball = (settings.ball + 1) % BALL_TYPES.length;
  else if (k === 'g') cycleGravity();
  else if (k === 's') settings.slowmo = !settings.slowmo;
  else if (k === 'c') clearBalls();
  else return;
  saveSettings();
  setStatus(`Pile: ${PILES[settings.pile]} · Ball: ${BALL_TYPES[settings.ball].name} · Gravity: ${GRAVITIES[settings.gravity].name} · Slow-mo: ${settings.slowmo ? 'on' : 'off'}`);
});

window.addEventListener('resize', () => {
  if (renderer.xr.isPresenting) return;
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

buildPile(new THREE.Vector3(0, 0, -1.6), 0);

// For automated tests and the browser console.
window.brickfall = { world, bricks, balls, hands, settings, buildPile, addBall, menu, room, placement };
