// AUTOBOT live sensor viewer.
//
// Reads the firmware's 100 Hz "@I,..." serial stream over USB (Web
// Serial, Chrome / Edge), and shows:
//   - a 3D robot that follows the MPU6050 attitude,
//   - the HC-SR04 beam and its echo, in 3D,
//   - echo points left behind in world space, so spinning the robot
//     in place sweeps out the room (2D radar + 3D),
//   - gyro / motor bars and the raw serial console.
//
// The newest frame is drawn on every animation frame. Nothing is
// interpolated or smoothed here, so the only delay is the 10 ms
// stream period plus one display frame.

import * as THREE from "three";
import { OrbitControls } from "./vendor/OrbitControls.js";

const BAUD = 921600;
const FRAME_FIELDS = 22;

// Distance thresholds from the firmware (MANUAL_GUARD_CM, OBSTACLE_CM,
// SLOWDOWN_CM), used to colour the beam.
const GUARD_CM = 20;
const OBSTACLE_CM = 35;
const SLOWDOWN_CM = 70;
const US_MAX_CM = 400;
const US_HALF_ANGLE = THREE.MathUtils.degToRad(15);

// Robot geometry, cm, robot frame: X forward, Y left, Z up, origin at
// the wheel axle centre.
const WHEEL_R = 3.3;
const SENSOR_POS = new THREE.Vector3(12.2, 0, 3.4);

const ECHO_MAX = 4000;
const ECHO_LIFE_S = 45;
const TREND_S = 15;

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------
// Settings (per browser)
// ---------------------------------------------------------------

const settings = { mountFwd: "+x", mountFace: "up", showAxes: true, view: "orbit" };

try {
  Object.assign(settings, JSON.parse(localStorage.getItem("autobot-viewer") || "{}"));
} catch {}

function saveSettings() {
  try {
    localStorage.setItem("autobot-viewer", JSON.stringify(settings));
  } catch {}
}

// ---------------------------------------------------------------
// Live state
// ---------------------------------------------------------------

const live = {
  source: null,          // "serial" | "demo" | null
  frame: null,           // newest parsed frame
  lastFrameAt: 0,
  frameTimes: [],        // arrival times, last 1 s, for the Hz readout
  usTimes: [],
  lastUsSeq: -1,
  badLines: 0,
};

// Sensor -> robot mount rotation, and the heading tare.
const mountQ = new THREE.Quaternion();
const mountInvQ = new THREE.Quaternion();
const tareQ = new THREE.Quaternion();
const sensorQ = new THREE.Quaternion();   // from the firmware
const robotQ = new THREE.Quaternion();    // shown

function updateMount() {
  const f = {
    "+x": new THREE.Vector3(1, 0, 0),
    "-x": new THREE.Vector3(-1, 0, 0),
    "+y": new THREE.Vector3(0, 1, 0),
    "-y": new THREE.Vector3(0, -1, 0),
  }[settings.mountFwd];

  // The UI says where the sensor's X axis points on the robot. Turn
  // that into "which sensor axis is robot-forward".
  const up = new THREE.Vector3(0, 0, settings.mountFace === "up" ? 1 : -1);

  // Sensor axes in robot coordinates: X_s = f, Z_s = up.
  const xs = f.clone();
  const zs = up.clone();
  const ys = new THREE.Vector3().crossVectors(zs, xs);
  const sensorToRobot = new THREE.Matrix4().makeBasis(xs, ys, zs);

  // robot = sensorToRobot * sensor, so robot->world = sensor->world * robotToSensor.
  mountInvQ.setFromRotationMatrix(sensorToRobot);
  mountQ.copy(mountInvQ).invert();

  imuAxes.quaternion.copy(mountInvQ);
}

// ---------------------------------------------------------------
// Three.js scene. Everything lives in `world`, which is Z-up in cm.
// ---------------------------------------------------------------

const container = $("three");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.setClearColor(0x07090d);
container.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x07090d, 1600, 3200);

const camera = new THREE.PerspectiveCamera(45, 1, 1, 5000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.12;
controls.target.set(0, 4, 0);

const world = new THREE.Group();
world.rotation.x = -Math.PI / 2;   // Z-up inside, Y-up three.js outside
scene.add(world);

scene.add(new THREE.HemisphereLight(0xbfd8ff, 0x1a1208, 0.9));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(80, 160, 60);
sun.castShadow = true;
sun.shadow.mapSize.set(1024, 1024);
sun.shadow.camera.left = -40;
sun.shadow.camera.right = 40;
sun.shadow.camera.top = 40;
sun.shadow.camera.bottom = -40;
sun.shadow.camera.near = 50;
sun.shadow.camera.far = 400;
sun.shadow.radius = 4;
scene.add(sun);

// Floor: shadow catcher, 50 cm grid, range rings every metre.
{
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(2000, 2000),
    new THREE.ShadowMaterial({ opacity: 0.45 })
  );
  floor.receiveShadow = true;
  world.add(floor);

  const grid = new THREE.GridHelper(1000, 20, 0x1d2733, 0x141b24);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = 0.02;
  world.add(grid);

  const ringMat = new THREE.LineBasicMaterial({ color: 0x2a3a4c, transparent: true, opacity: 0.8 });
  for (const r of [100, 200, 300, 400]) {
    const pts = [];
    for (let i = 0; i <= 128; i++) {
      const a = (i / 128) * Math.PI * 2;
      pts.push(new THREE.Vector3(Math.cos(a) * r, Math.sin(a) * r, 0.05));
    }
    world.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), ringMat));
    world.add(makeTextSprite(`${r / 100} m`, new THREE.Vector3(r + 4, 0, 0.1), "#3d5166", 11));
  }
}

function makeTextSprite(text, pos, color, size) {
  const c = document.createElement("canvas");
  const ctx = c.getContext("2d");
  c.width = 128;
  c.height = 48;
  ctx.font = "600 26px ui-monospace, Menlo, monospace";
  ctx.fillStyle = color;
  ctx.textBaseline = "middle";
  ctx.fillText(text, 4, 24);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthWrite: false, transparent: true }));
  s.position.copy(pos);
  s.scale.set(size * 2.67, size, 1);
  s.center.set(0, 0.5);
  return s;
}

// ---------------------------------------------------------------
// Robot model (robot frame, cm)
// ---------------------------------------------------------------

const robot = new THREE.Group();
robot.position.z = WHEEL_R;
world.add(robot);

const imuAxes = new THREE.Group();

{
  const mat = (color, extra = {}) =>
    new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.1, ...extra });

  const add = (geo, material, x, y, z, parent = robot) => {
    const m = new THREE.Mesh(geo, material);
    m.position.set(x, y, z);
    m.castShadow = true;
    m.receiveShadow = true;
    parent.add(m);
    return m;
  };

  // Two acrylic decks with standoffs.
  const acrylic = mat(0x1c2733, { roughness: 0.25, metalness: 0.0, transparent: true, opacity: 0.92 });
  add(roundedPlate(23, 15, 0.4, 2.5), acrylic, 0, 0, 0.3);
  add(roundedPlate(21, 14, 0.4, 2.5), acrylic, 0.5, 0, 4.2);
  const brass = mat(0xb08d48, { metalness: 0.8, roughness: 0.35 });
  for (const [x, y] of [[9, 5.5], [9, -5.5], [-8, 5.5], [-8, -5.5]]) {
    add(new THREE.CylinderGeometry(0.35, 0.35, 3.6, 8).rotateX(Math.PI / 2), brass, x, y, 2.3);
  }

  // Yellow TT gear motors and wheels.
  const yellow = mat(0xf2c230);
  const tyre = mat(0x111214, { roughness: 0.9 });
  const hub = mat(0xe8e8e8, { roughness: 0.4 });
  for (const s of [1, -1]) {
    add(new THREE.BoxGeometry(7, 2.2, 1.9), yellow, -2.5, s * 5.6, -1.1);
    const wheel = new THREE.Group();
    wheel.position.set(-2.5, s * 8.9, 0);
    robot.add(wheel);
    add(new THREE.CylinderGeometry(WHEEL_R, WHEEL_R, 2.6, 32), tyre, 0, 0, 0, wheel);
    add(new THREE.CylinderGeometry(1.6, 1.6, 2.7, 16), hub, 0, 0, 0, wheel);
    for (let i = 0; i < 6; i++) {
      const tread = add(new THREE.BoxGeometry(0.5, 2.65, 0.3), mat(0x26282c), 0, 0, 0, wheel);
      const a = (i / 6) * Math.PI * 2;
      tread.position.set(Math.cos(a) * (WHEEL_R - 0.1), 0, Math.sin(a) * (WHEEL_R - 0.1));
      tread.rotation.y = -a;
    }
  }

  // Front caster.
  add(new THREE.CylinderGeometry(0.9, 0.9, 1.2, 12).rotateX(Math.PI / 2), mat(0x8a96a3, { metalness: 0.7 }), 8.5, 0, -0.5);
  add(new THREE.SphereGeometry(1.0, 16, 12), mat(0xd0d6dc, { metalness: 0.9, roughness: 0.2 }), 8.5, 0, -2.3);

  // Battery pack (lower deck, rear).
  add(new THREE.BoxGeometry(6, 6, 1.9), mat(0x15171b), -6.5, 0, 1.5);

  // L298N: red board + black heatsink.
  add(new THREE.BoxGeometry(4.3, 4.3, 0.2), mat(0xb3262e), -5.5, 0, 4.6);
  add(new THREE.BoxGeometry(2.4, 4.0, 2.2), mat(0x0e0f12, { metalness: 0.6 }), -5.5, 0, 5.8);

  // ESP32 (Wemos D1 R32) board.
  add(new THREE.BoxGeometry(6.8, 5.3, 0.2), mat(0x0d2a52), 2.5, 0, 4.6);
  add(new THREE.BoxGeometry(2.5, 1.8, 0.35), mat(0x9aa4ad, { metalness: 0.8 }), 4.5, 1.2, 4.85);

  // MPU6050 breakout + its axes.
  add(new THREE.BoxGeometry(2.1, 1.6, 0.15), mat(0x2b3fb6), -2.2, 0, 4.75);
  add(new THREE.BoxGeometry(0.45, 0.45, 0.12), mat(0x111111), -2.2, 0, 4.88);
  imuAxes.position.set(-2.2, 0, 5.0);
  robot.add(imuAxes);
  const axisDefs = [
    [new THREE.Vector3(1, 0, 0), 0xff5a5f, "X"],
    [new THREE.Vector3(0, 1, 0), 0x4cd964, "Y"],
    [new THREE.Vector3(0, 0, 1), 0x4aa3ff, "Z"],
  ];
  for (const [dir, color, name] of axisDefs) {
    const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(), 6, color, 1.4, 0.8);
    arrow.line.material.depthTest = false;
    arrow.cone.material.depthTest = false;
    arrow.renderOrder = 10;
    imuAxes.add(arrow);
    const label = makeTextSprite(name, dir.clone().multiplyScalar(7.2), new THREE.Color(color).getStyle(), 2.2);
    label.material.depthTest = false;
    label.renderOrder = 11;
    imuAxes.add(label);
  }

  // HC-SR04 on the front edge: PCB, two transducers.
  const pcb = mat(0x1a4f8a);
  add(new THREE.BoxGeometry(0.2, 4.5, 2.1), pcb, 11.6, 0, SENSOR_POS.z);
  const can = mat(0xc9ced3, { metalness: 0.85, roughness: 0.3 });
  const mesh = mat(0x1a1c1f, { roughness: 1 });
  for (const y of [1.3, -1.3]) {
    add(new THREE.CylinderGeometry(0.8, 0.8, 1.2, 24).rotateZ(Math.PI / 2), can, 12.3, y, SENSOR_POS.z);
    add(new THREE.CylinderGeometry(0.62, 0.62, 0.05, 24).rotateZ(Math.PI / 2), mesh, 12.92, y, SENSOR_POS.z);
  }

  // Forward arrow painted on the top deck.
  const arrowShape = new THREE.Shape();
  arrowShape.moveTo(9.6, 0);
  arrowShape.lineTo(7.4, 1.6);
  arrowShape.lineTo(7.4, 0.6);
  arrowShape.lineTo(6.0, 0.6);
  arrowShape.lineTo(6.0, -0.6);
  arrowShape.lineTo(7.4, -0.6);
  arrowShape.lineTo(7.4, -1.6);
  const fwd = new THREE.Mesh(new THREE.ShapeGeometry(arrowShape), new THREE.MeshBasicMaterial({ color: 0x39c6ff }));
  fwd.position.z = 4.42;
  robot.add(fwd);
}

function roundedPlate(w, h, t, r) {
  const s = new THREE.Shape();
  const x = -w / 2, y = -h / 2;
  s.moveTo(x + r, y);
  s.lineTo(x + w - r, y);
  s.quadraticCurveTo(x + w, y, x + w, y + r);
  s.lineTo(x + w, y + h - r);
  s.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
  s.lineTo(x + r, y + h);
  s.quadraticCurveTo(x, y + h, x, y + h - r);
  s.lineTo(x, y + r);
  s.quadraticCurveTo(x, y, x + r, y);
  const g = new THREE.ExtrudeGeometry(s, { depth: t, bevelEnabled: false });
  g.translate(0, 0, -t / 2);
  return g;
}

// Heading line on the floor (follows yaw only).
const headingLine = new THREE.Line(
  new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(14, 0, 0.1), new THREE.Vector3(60, 0, 0.1)]),
  new THREE.LineDashedMaterial({ color: 0x39c6ff, dashSize: 3, gapSize: 2, transparent: true, opacity: 0.6 })
);
headingLine.computeLineDistances();
world.add(headingLine);

// ---------------------------------------------------------------
// Ultrasonic beam (child of the robot, so it tilts with it)
// ---------------------------------------------------------------

const beam = new THREE.Group();
beam.position.copy(SENSOR_POS);
robot.add(beam);

const coneGeo = new THREE.ConeGeometry(Math.tan(US_HALF_ANGLE), 1, 48, 1, true);
coneGeo.translate(0, -0.5, 0);
coneGeo.rotateZ(Math.PI / 2);   // apex at origin, opening along +X

const beamMat = new THREE.MeshBasicMaterial({
  color: 0xffb23e,
  transparent: true,
  opacity: 0.16,
  side: THREE.DoubleSide,
  depthWrite: false,
  blending: THREE.AdditiveBlending,
});
const beamCone = new THREE.Mesh(coneGeo, beamMat);
beam.add(beamCone);

const beamEdgeMat = new THREE.LineBasicMaterial({ color: 0xffb23e, transparent: true, opacity: 0.55 });
const beamEdges = new THREE.LineSegments(new THREE.EdgesGeometry(coneGeo, 10), beamEdgeMat);
beam.add(beamEdges);

const axisMat = new THREE.LineDashedMaterial({ color: 0xffb23e, dashSize: 0.03, gapSize: 0.02, transparent: true, opacity: 0.7 });
const beamAxis = new THREE.Line(
  new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(1, 0, 0)]),
  axisMat
);
beamAxis.computeLineDistances();
beam.add(beamAxis);

const hitGeo = new THREE.CircleGeometry(Math.tan(US_HALF_ANGLE), 48).rotateY(Math.PI / 2);
const hitMat = new THREE.MeshBasicMaterial({
  color: 0xffb23e,
  transparent: true,
  opacity: 0.12,
  side: THREE.DoubleSide,
  depthWrite: false,
  blending: THREE.AdditiveBlending,
});
const hitDisc = new THREE.Mesh(hitGeo, hitMat);
beam.add(hitDisc);

const pulseMat = hitMat.clone();

// Bright rim where the beam meets the obstacle.
const rimMat = new THREE.LineBasicMaterial({ color: 0xffb23e });
const rim = new THREE.LineLoop(
  new THREE.BufferGeometry().setFromPoints(
    Array.from({ length: 64 }, (_, i) => {
      const a = (i / 64) * Math.PI * 2;
      const r = Math.tan(US_HALF_ANGLE);
      return new THREE.Vector3(0, Math.cos(a) * r, Math.sin(a) * r);
    })
  ),
  rimMat
);
beam.add(rim);
const pulseRing = new THREE.Mesh(
  new THREE.RingGeometry(Math.tan(US_HALF_ANGLE) * 0.92, Math.tan(US_HALF_ANGLE), 48).rotateY(Math.PI / 2),
  pulseMat
);
beam.add(pulseRing);
let pulseStart = -1;

function distColor(cm) {
  if (cm <= GUARD_CM) return new THREE.Color(0xff3b4a);
  if (cm <= OBSTACLE_CM) return new THREE.Color(0xff6a3d);
  if (cm <= SLOWDOWN_CM) return new THREE.Color(0xffb23e);
  return new THREE.Color(0x3ddc84).lerp(new THREE.Color(0x39c6ff), Math.min(1, (cm - SLOWDOWN_CM) / 250));
}

// ---------------------------------------------------------------
// Echo points (world frame, fade with age)
// ---------------------------------------------------------------

const echoPos = new Float32Array(ECHO_MAX * 3);
const echoCol = new Float32Array(ECHO_MAX * 3);
const echoBase = new Float32Array(ECHO_MAX * 3);
const echoTime = new Float64Array(ECHO_MAX).fill(-1e9);
const echoXY = [];   // for the 2D radar: {x, y, t, cm}
let echoHead = 0;
let echoCount = 0;

const echoGeo = new THREE.BufferGeometry();
echoGeo.setAttribute("position", new THREE.BufferAttribute(echoPos, 3));
echoGeo.setAttribute("color", new THREE.BufferAttribute(echoCol, 3));
echoGeo.setDrawRange(0, 0);

const dotTex = (() => {
  const c = document.createElement("canvas");
  c.width = c.height = 64;
  const g = c.getContext("2d");
  const grad = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  grad.addColorStop(0, "rgba(255,255,255,1)");
  grad.addColorStop(0.35, "rgba(255,255,255,0.9)");
  grad.addColorStop(1, "rgba(255,255,255,0)");
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  return new THREE.CanvasTexture(c);
})();

const echoPoints = new THREE.Points(
  echoGeo,
  new THREE.PointsMaterial({
    size: 5,
    map: dotTex,
    vertexColors: true,
    transparent: true,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
  })
);
echoPoints.frustumCulled = false;
world.add(echoPoints);

function addEcho(p, cm, now) {
  const i = echoHead;
  echoPos.set([p.x, p.y, p.z], i * 3);
  const c = distColor(cm);
  echoBase.set([c.r, c.g, c.b], i * 3);
  echoTime[i] = now;
  echoHead = (echoHead + 1) % ECHO_MAX;
  echoCount = Math.min(echoCount + 1, ECHO_MAX);
  echoGeo.setDrawRange(0, echoCount);
  echoGeo.attributes.position.needsUpdate = true;

  echoXY.push({ x: p.x, y: p.y, t: now, cm });
  if (echoXY.length > ECHO_MAX) echoXY.shift();
}

function clearEchoes() {
  echoTime.fill(-1e9);
  echoCol.fill(0);
  echoCount = 0;
  echoHead = 0;
  echoGeo.setDrawRange(0, 0);
  echoGeo.attributes.color.needsUpdate = true;
  echoXY.length = 0;
}

function fadeEchoes(now) {
  for (let i = 0; i < echoCount; i++) {
    const age = (now - echoTime[i]) / 1000;
    const k = Math.max(0, 1 - age / ECHO_LIFE_S);
    const b = 0.15 + 0.85 * k * k;
    const live = age < 0.25 ? 1.6 : 1;
    echoCol[i * 3] = echoBase[i * 3] * b * live;
    echoCol[i * 3 + 1] = echoBase[i * 3 + 1] * b * live;
    echoCol[i * 3 + 2] = echoBase[i * 3 + 2] * b * live;
  }
  echoGeo.attributes.color.needsUpdate = true;
}

// ---------------------------------------------------------------
// Frame handling
// ---------------------------------------------------------------

const trend = [];   // {t, cm} (cm = -1 for no echo)

function parseFrame(line) {
  const f = line.split(",");
  if (f.length !== FRAME_FIELDS) return null;
  const n = (i) => Number(f[i]);
  const q = [n(2), n(3), n(4), n(5)];
  if (q.some((v) => !Number.isFinite(v))) return null;
  return {
    ms: n(1),
    q,
    yawFw: n(6),
    gyro: [n(7), n(8), n(9)],
    acc: [n(10), n(11), n(12)],
    dist: n(13),
    usSeq: n(14),
    usStatus: f[15],
    left: n(16),
    right: n(17),
    imuOk: f[18] === "1",
    cal: f[19],
    mode: f[20],
    dropped: n(21),
  };
}

const tmpV = new THREE.Vector3();

function onFrame(fr) {
  const now = performance.now();
  live.frame = fr;
  live.lastFrameAt = now;
  live.frameTimes.push(now);

  // Firmware: (w, x, y, z), sensor -> world, Z up.
  sensorQ.set(fr.q[1], fr.q[2], fr.q[3], fr.q[0]).normalize();
  robotQ.copy(tareQ).multiply(sensorQ).multiply(mountQ);

  if (fr.usSeq !== live.lastUsSeq) {
    const isNew = live.lastUsSeq >= 0;
    live.lastUsSeq = fr.usSeq;
    if (isNew) {
      live.usTimes.push(now);
      trend.push({ t: now, cm: fr.dist });
      if (fr.dist > 0) {
        // Echo position in world: sensor position + range along its axis.
        tmpV.set(1, 0, 0).applyQuaternion(robotQ).multiplyScalar(fr.dist);
        const p = SENSOR_POS.clone().applyQuaternion(robotQ).add(robot.position).add(tmpV);
        addEcho(p, fr.dist, now);
        pulseStart = now;
      }
    }
  }
}

function handleLine(line) {
  if (line.startsWith("@I,")) {
    const fr = parseFrame(line);
    if (fr) onFrame(fr);
    else live.badLines++;
    return;
  }
  if (line.trim()) consoleLog(line);
}

// ---------------------------------------------------------------
// Serial console
// ---------------------------------------------------------------

const consoleLines = [];
let consoleDirty = false;

function consoleLog(line) {
  consoleLines.push(line);
  if (consoleLines.length > 400) consoleLines.splice(0, consoleLines.length - 400);
  consoleDirty = true;
}

setInterval(() => {
  if (!consoleDirty) return;
  consoleDirty = false;
  const el = $("console");
  const atBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 20;
  // Reset reasons, safety stops and link problems stand out.
  const loud = /^\[(BOOT|SAFETY|NET)\]|WARNING|ERROR|BROWNOUT|froze|Guru Meditation|panic/i;
  const esc = (t) => t.replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" })[c]);
  el.innerHTML = consoleLines
    .map((l) => (loud.test(l) ? `<span class="loud">${esc(l)}</span>` : esc(l)))
    .join("\n");
  if (atBottom) el.scrollTop = el.scrollHeight;
}, 150);

// ---------------------------------------------------------------
// Web Serial
// ---------------------------------------------------------------

const serial = {
  port: null,
  reader: null,
  writer: null,
  keepalive: null,
  wanted: false,   // reopen when the same board is plugged back in
};

async function serialSend(text) {
  if (!serial.writer) return;
  try {
    await serial.writer.write(new TextEncoder().encode(text + "\n"));
  } catch {}
}

async function connectSerial() {
  if (!("serial" in navigator)) {
    alert("This browser has no Web Serial. Open the viewer in Chrome or Edge.");
    return;
  }
  let port;
  try {
    port = await navigator.serial.requestPort();
  } catch {
    return;   // picker cancelled
  }
  await openPort(port);
}

async function openPort(port) {
  await stopSource();
  try {
    await port.open({ baudRate: BAUD, bufferSize: 65536 });
  } catch (e) {
    setLink("err", `open failed: ${e.message}`);
    consoleLog(`[viewer] Could not open the port: ${e.message}. Is the Arduino Serial Monitor still open?`);
    return;
  }

  serial.port = port;
  serial.wanted = true;
  serial.writer = port.writable.getWriter();
  live.source = "serial";
  resetLive();
  setLink("stale", "connecting…");
  setButtons();
  consoleLog(`[viewer] Port open at ${BAUD} baud. Requesting stream…`);

  serialSend("@stream 1");
  serial.keepalive = setInterval(() => serialSend("@stream 1"), 1000);

  readLoop(port);
}

async function readLoop(port) {
  const decoder = new TextDecoder();
  let buf = "";
  try {
    serial.reader = port.readable.getReader();
    while (true) {
      const { value, done } = await serial.reader.read();
      if (done) break;
      buf += decoder.decode(value, { stream: true });
      let nl;
      while ((nl = buf.indexOf("\n")) >= 0) {
        handleLine(buf.slice(0, nl).replace(/\r$/, ""));
        buf = buf.slice(nl + 1);
      }
      if (buf.length > 8192) buf = "";
    }
  } catch (e) {
    consoleLog(`[viewer] Serial read stopped: ${e.message}`);
  } finally {
    try { serial.reader?.releaseLock(); } catch {}
    serial.reader = null;
  }
  if (live.source === "serial" && serial.port === port) {
    const wanted = serial.wanted;
    await stopSource();
    serial.wanted = wanted;
    serial.lastPort = port;
    setLink("err", "USB disconnected");
  }
}

async function closeSerial() {
  clearInterval(serial.keepalive);
  serial.keepalive = null;
  const port = serial.port;
  serial.port = null;
  if (serial.writer) {
    try {
      await serial.writer.write(new TextEncoder().encode("@stream 0\n"));
    } catch {}
    try { serial.writer.releaseLock(); } catch {}
    serial.writer = null;
  }
  if (serial.reader) {
    try { await serial.reader.cancel(); } catch {}
  }
  if (port) {
    try { await port.close(); } catch {}
  }
}

if ("serial" in navigator) {
  // Same board plugged back in: carry on without the picker.
  navigator.serial.addEventListener("connect", (e) => {
    if (serial.wanted && !live.source && e.target === serial.lastPort) {
      consoleLog("[viewer] Board is back - reconnecting.");
      setTimeout(() => openPort(e.target), 300);
    }
  });
}

// ---------------------------------------------------------------
// Demo source: a robot spinning and wobbling in a virtual room,
// fed through the same text parser as the real stream.
// ---------------------------------------------------------------

const demo = { timer: null, t0: 0, usSeq: 0, lastUs: 0 };

const ROOM = { x0: -170, x1: 300, y0: -140, y1: 220 };
const BOXES = [
  { x0: 120, x1: 180, y0: 120, y1: 220 },   // cabinet
  { x0: -170, x1: -90, y0: -140, y1: -60 }, // couch corner
  { x0: 60, x1: 90, y0: -40, y1: -10 },     // chair leg
];

function rayBox(ox, oy, dx, dy, b, inside) {
  // Slab test; returns the distance to the first hit or Infinity.
  const ix = 1 / (dx || 1e-9), iy = 1 / (dy || 1e-9);
  const tx1 = (b.x0 - ox) * ix, tx2 = (b.x1 - ox) * ix;
  const ty1 = (b.y0 - oy) * iy, ty2 = (b.y1 - oy) * iy;
  const tmin = Math.max(Math.min(tx1, tx2), Math.min(ty1, ty2));
  const tmax = Math.min(Math.max(tx1, tx2), Math.max(ty1, ty2));
  if (tmax < Math.max(tmin, 0)) return Infinity;
  return inside ? tmax : tmin > 0 ? tmin : Infinity;
}

function demoRange(ox, oy, yaw) {
  // Several rays across the cone; the nearest hit wins, like an HC-SR04.
  let best = Infinity;
  for (const off of [-0.18, -0.09, 0, 0.09, 0.18]) {
    const dx = Math.cos(yaw + off), dy = Math.sin(yaw + off);
    best = Math.min(best, rayBox(ox, oy, dx, dy, ROOM, true));
    for (const b of BOXES) best = Math.min(best, rayBox(ox, oy, dx, dy, b, false));
  }
  return best;
}

function startDemo() {
  stopSource().then(() => {
    live.source = "demo";
    resetLive();
    demo.t0 = performance.now();
    demo.lastUs = 0;
    setButtons();
    consoleLog("[viewer] Demo: simulated robot in a 4.7 x 3.6 m room.");
    const q = new THREE.Quaternion();
    const e = new THREE.Euler();
    demo.timer = setInterval(() => {
      const t = (performance.now() - demo.t0) / 1000;
      // Spin, pause, spin back; a little pitch / roll wobble.
      const yaw = 1.1 * Math.sin(t * 0.35) * Math.PI + 0.4 * Math.sin(t * 0.9);
      const yawRate = (1.1 * Math.PI * 0.35 * Math.cos(t * 0.35) + 0.36 * Math.cos(t * 0.9)) * 180 / Math.PI;
      const pitch = 0.05 * Math.sin(t * 2.1) + (Math.sin(t * 0.21) > 0.93 ? 0.35 * Math.sin(t * 3) : 0);
      const roll = 0.04 * Math.sin(t * 1.7 + 1);
      e.set(roll, pitch, yaw, "ZYX");
      q.setFromEuler(e);
      // Back out the mount so the Setup selectors behave as on the real robot.
      q.multiply(mountInvQ);

      if (t - demo.lastUs >= 0.06) {
        demo.lastUs = t;
        demo.usSeq++;
        const sx = Math.cos(yaw) * SENSOR_POS.x, sy = Math.sin(yaw) * SENSOR_POS.x;
        demo.dist = demoRange(sx, sy, yaw) + (Math.random() - 0.5) * 2;
        demo.status = demo.dist > US_MAX_CM ? "no_echo" : "ok";
        if (Math.random() < 0.04) demo.status = "no_echo";
        if (demo.status !== "ok") demo.dist = -1;
      }

      const drive = Math.round(Math.max(-1, Math.min(1, yawRate / 120)) * 140);
      const line = [
        "@I", Math.round(t * 1000),
        q.w.toFixed(5), q.x.toFixed(5), q.y.toFixed(5), q.z.toFixed(5),
        (yaw * 180 / Math.PI).toFixed(2),
        (Math.cos(t * 1.7 + 1) * 3.9).toFixed(1), (Math.cos(t * 2.1) * 6).toFixed(1), yawRate.toFixed(1),
        (-Math.sin(pitch)).toFixed(3), (Math.sin(roll) * Math.cos(pitch)).toFixed(3), (Math.cos(roll) * Math.cos(pitch)).toFixed(3),
        (demo.dist ?? -1).toFixed(1), demo.usSeq, demo.status ?? "none",
        -drive, drive, 1, "ok", "manual", 0,
      ].join(",");
      handleLine(line);
    }, 10);
  });
}

async function stopSource() {
  if (demo.timer) {
    clearInterval(demo.timer);
    demo.timer = null;
  }
  if (live.source === "serial" || serial.port) await closeSerial();
  serial.wanted = false;
  live.source = null;
  live.frame = null;
  setButtons();
}

function resetLive() {
  live.frame = null;
  live.lastFrameAt = 0;
  live.frameTimes.length = 0;
  live.usTimes.length = 0;
  live.lastUsSeq = -1;
  trend.length = 0;
  tareQ.identity();
  clearEchoes();
}

// ---------------------------------------------------------------
// UI wiring
// ---------------------------------------------------------------

function setLink(kind, text) {
  $("linkDot").className = `dot ${kind}`;
  $("linkText").textContent = text;
}

function setButtons() {
  const on = !!live.source;
  $("disconnectBtn").disabled = !on;
  $("calBtn").disabled = live.source !== "serial";
  $("emptyState").classList.toggle("hide", on);
  if (!on) setLink("", "offline");
}

$("connectBtn").onclick = connectSerial;
$("demoBtn").onclick = startDemo;
$("disconnectBtn").onclick = () => stopSource();
$("clearBtn").onclick = clearEchoes;
$("calBtn").onclick = () => {
  serialSend("@cal");
  consoleLog("[viewer] Sent @cal - keep the robot still for ~2 s.");
};
$("zeroBtn").onclick = () => {
  // Remove the current heading, keep tilt. Old echoes no longer line up.
  const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(robotQ);
  const yaw = Math.atan2(fwd.y, fwd.x);
  tareQ.premultiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -yaw));
  robotQ.premultiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -yaw));
  clearEchoes();
};

$("mountFwd").value = settings.mountFwd;
$("mountFace").value = settings.mountFace;
$("showAxes").checked = settings.showAxes;
$("mountFwd").onchange = (e) => { settings.mountFwd = e.target.value; saveSettings(); updateMount(); clearEchoes(); };
$("mountFace").onchange = (e) => { settings.mountFace = e.target.value; saveSettings(); updateMount(); clearEchoes(); };
$("showAxes").onchange = (e) => { settings.showAxes = e.target.checked; saveSettings(); imuAxes.visible = settings.showAxes; };
imuAxes.visible = settings.showAxes;

for (const b of $("viewSeg").querySelectorAll("button")) {
  b.onclick = () => setView(b.dataset.view);
}

function setView(v) {
  settings.view = v;
  saveSettings();
  for (const b of $("viewSeg").querySelectorAll("button")) b.classList.toggle("on", b.dataset.view === v);
  controls.enabled = v !== "chase";
  if (v === "orbit") {
    camera.position.set(-120, 95, 115);
    controls.target.set(25, 4, 0);
  } else if (v === "top") {
    // Slightly behind on -X so the tare heading points up, as on the radar.
    camera.position.set(-0.01, 1050, 0);
    controls.target.set(0, 0, 0);
  }
  controls.update();
}

// Bars
function makeBars(el, defs) {
  const out = {};
  for (const [key, label, color] of defs) {
    const row = document.createElement("div");
    row.className = "bar";
    row.innerHTML = `<span class="lab">${label}</span><div class="track"><div class="fill" style="background:${color}"></div></div><span class="val">—</span>`;
    el.appendChild(row);
    out[key] = { fill: row.querySelector(".fill"), val: row.querySelector(".val") };
  }
  return out;
}

const gyroBars = makeBars($("gyroBars"), [
  ["x", "ωx", "var(--x)"],
  ["y", "ωy", "var(--y)"],
  ["z", "ωz", "var(--z)"],
]);
const motorBars = makeBars($("motorBars"), [
  ["l", "L", "var(--imu)"],
  ["r", "R", "var(--imu)"],
]);

function setBar(bar, v, max, text) {
  const k = Math.max(-1, Math.min(1, v / max));
  bar.fill.style.left = k >= 0 ? "50%" : `${50 + k * 50}%`;
  bar.fill.style.width = `${Math.abs(k) * 50}%`;
  bar.val.textContent = text;
}

// ---------------------------------------------------------------
// 2D radar (world frame, heading-up at tare) and range trend
// ---------------------------------------------------------------

const radar = $("radar");
const trendCv = $("trend");

function fitCanvas(cv) {
  const dpr = Math.min(window.devicePixelRatio, 2);
  const w = cv.clientWidth;
  const h = Number(cv.getAttribute("height"));
  if (cv.width !== Math.round(w * dpr)) {
    cv.width = Math.round(w * dpr);
    cv.height = Math.round(h * dpr);
    cv.style.height = `${h}px`;
  }
  const ctx = cv.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

function cssColor(c) {
  return `rgb(${Math.round(c.r * 255)},${Math.round(c.g * 255)},${Math.round(c.b * 255)})`;
}

function drawRadar(now, fr) {
  const { ctx, w, h } = fitCanvas(radar);
  ctx.clearRect(0, 0, w, h);
  const cx = w / 2, cy = h / 2;
  const scale = (Math.min(w, h) / 2 - 8) / US_MAX_CM;
  // World +X (heading at tare) is up, world +Y (left) is left.
  const sx = (x, y) => cx - y * scale;
  const sy = (x, y) => cy - x * scale;

  ctx.lineWidth = 1;
  ctx.font = "10px ui-monospace, Menlo, monospace";
  for (const r of [100, 200, 300, 400]) {
    ctx.strokeStyle = r === 400 ? "#26323f" : "#1a232e";
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = "#3d5166";
    ctx.fillText(`${r / 100}m`, cx + 3, cy - r * scale + 11);
  }
  ctx.strokeStyle = "#1a232e";
  ctx.beginPath();
  ctx.moveTo(cx, 4); ctx.lineTo(cx, h - 4);
  ctx.moveTo(cx - h / 2 + 4, cy); ctx.lineTo(cx + h / 2 - 4, cy);
  ctx.stroke();

  // Echo points.
  for (const e of echoXY) {
    const age = (now - e.t) / 1000;
    if (age > ECHO_LIFE_S) continue;
    const k = 1 - age / ECHO_LIFE_S;
    ctx.globalAlpha = 0.15 + 0.85 * k * k;
    ctx.fillStyle = cssColor(distColor(e.cm));
    ctx.fillRect(sx(e.x, e.y) - 1.5, sy(e.x, e.y) - 1.5, 3, 3);
  }
  ctx.globalAlpha = 1;

  if (!fr) return;

  // Current beam wedge (horizontal projection of the sensor axis).
  const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(robotQ);
  const yaw = Math.atan2(fwd.y, fwd.x);
  const len = fr.dist > 0 ? fr.dist : US_MAX_CM;
  const col = fr.dist > 0 ? cssColor(distColor(fr.dist)) : "#56616d";
  // Screen angle of world direction (x, y): screen = (-y, -x).
  const ang = (a) => Math.atan2(-Math.cos(a), -Math.sin(a));
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.arc(cx, cy, len * scale, ang(yaw + US_HALF_ANGLE), ang(yaw - US_HALF_ANGLE));
  ctx.closePath();
  ctx.fillStyle = col;
  ctx.globalAlpha = fr.dist > 0 ? 0.22 : 0.08;
  ctx.fill();
  ctx.globalAlpha = 1;
  if (fr.dist > 0) {
    ctx.strokeStyle = col;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, len * scale, ang(yaw + US_HALF_ANGLE), ang(yaw - US_HALF_ANGLE));
    ctx.stroke();
  }

  // Robot glyph.
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-yaw);
  ctx.fillStyle = "#39c6ff";
  ctx.beginPath();
  ctx.moveTo(0, -9);
  ctx.lineTo(6, 6);
  ctx.lineTo(0, 3);
  ctx.lineTo(-6, 6);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function drawTrend(now) {
  const { ctx, w, h } = fitCanvas(trendCv);
  ctx.clearRect(0, 0, w, h);
  const pad = 4;
  const ymax = US_MAX_CM;
  const X = (t) => w - pad - ((now - t) / 1000 / TREND_S) * (w - pad * 2);
  // sqrt scale: room for the near ranges that matter for obstacles.
  const Y = (cm) => h - pad - Math.sqrt(Math.min(cm, ymax) / ymax) * (h - pad * 2);

  ctx.font = "10px ui-monospace, Menlo, monospace";
  for (const [cm, color, label] of [[OBSTACLE_CM, "#5a2a2e", "35"], [SLOWDOWN_CM, "#4a3a1e", "70"], [200, "#1a232e", "200"], [400, "#1a232e", "400"]]) {
    ctx.strokeStyle = color;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(pad, Y(cm));
    ctx.lineTo(w - pad, Y(cm));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = "#3d5166";
    ctx.fillText(label, pad + 2, Y(cm) - 3);
  }

  while (trend.length && now - trend[0].t > TREND_S * 1000) trend.shift();

  ctx.lineWidth = 1.5;
  let prev = null;
  for (const p of trend) {
    if (p.cm <= 0) {
      ctx.fillStyle = "#56616d";
      ctx.fillRect(X(p.t) - 1, h - pad - 3, 2, 3);
      prev = null;
      continue;
    }
    if (prev) {
      ctx.strokeStyle = cssColor(distColor(p.cm));
      ctx.beginPath();
      ctx.moveTo(X(prev.t), Y(prev.cm));
      ctx.lineTo(X(p.t), Y(p.cm));
      ctx.stroke();
    }
    prev = p;
  }
}

// ---------------------------------------------------------------
// Render loop
// ---------------------------------------------------------------

const euler = new THREE.Euler();
const labelPos = new THREE.Vector3();
const chaseOffset = new THREE.Vector3();
const chaseAim = new THREE.Vector3();

function fmtDeg(v) {
  const s = `${v < 0 ? "-" : ""}${Math.abs(v).toFixed(1)}`;
  return `${s.padStart(6, "\u2007")}°`;
}

function resize() {
  const w = container.clientWidth, h = container.clientHeight;
  renderer.setSize(w, h, false);
  renderer.domElement.style.width = `${w}px`;
  renderer.domElement.style.height = `${h}px`;
  camera.aspect = w / Math.max(1, h);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(container);
resize();

let lastUi = 0;

function tick() {
  requestAnimationFrame(tick);
  const now = performance.now();
  const fr = live.frame;

  while (live.frameTimes.length && now - live.frameTimes[0] > 1000) live.frameTimes.shift();
  while (live.usTimes.length && now - live.usTimes[0] > 1000) live.usTimes.shift();

  // Robot pose: newest frame, no smoothing.
  robot.quaternion.copy(robotQ);
  euler.setFromQuaternion(robotQ, "ZYX");
  headingLine.rotation.z = euler.z;

  // Beam.
  if (fr) {
    beam.visible = true;
    const valid = fr.dist > 0;
    const len = valid ? fr.dist : US_MAX_CM;
    const col = valid ? distColor(fr.dist) : new THREE.Color(0x56616d);
    beamCone.scale.setScalar(len);
    beamEdges.scale.setScalar(len);
    beamAxis.scale.setScalar(len);
    hitDisc.visible = valid;
    hitDisc.position.x = len;
    hitDisc.scale.setScalar(len);
    rim.visible = valid;
    rim.position.x = len;
    rim.scale.setScalar(len);
    rimMat.color.copy(col);
    beamMat.color.copy(col);
    beamMat.opacity = valid ? 0.16 : 0.05;
    beamEdgeMat.color.copy(col);
    beamEdgeMat.opacity = valid ? 0.55 : 0.18;
    axisMat.color.copy(col);
    hitMat.color.copy(col);

    const p = pulseStart > 0 ? (now - pulseStart) / 350 : 1;
    pulseRing.visible = valid && p < 1;
    if (pulseRing.visible) {
      pulseRing.position.x = len;
      pulseRing.scale.setScalar(len * (1 + p * 0.6));
      pulseMat.color.copy(col);
      pulseMat.opacity = 0.7 * (1 - p);
    }

    // Distance tag at the echo (or at max range for no echo).
    labelPos.set(len, 0, 0);
    beam.localToWorld(labelPos);
    labelPos.project(camera);
    const label = $("rangeLabel");
    if (labelPos.z < 1) {
      label.style.display = "block";
      label.style.left = `${(labelPos.x * 0.5 + 0.5) * container.clientWidth}px`;
      label.style.top = `${(-labelPos.y * 0.5 + 0.5) * container.clientHeight}px`;
      label.style.color = cssColor(col);
      label.textContent = valid ? `${fr.dist.toFixed(1)} cm` : fr.usStatus.replaceAll("_", " ");
    } else {
      label.style.display = "none";
    }
  } else {
    beam.visible = false;
    $("rangeLabel").style.display = "none";
  }

  fadeEchoes(now);

  // Chase camera: behind and above the robot's heading.
  if (settings.view === "chase") {
    const zAxis = new THREE.Vector3(0, 0, 1);
    chaseOffset.set(-95, 0, 50).applyAxisAngle(zAxis, euler.z);
    chaseAim.set(45, 0, 0).applyAxisAngle(zAxis, euler.z);
    // world is Z-up; convert to scene (x, z, -y).
    camera.position.set(chaseOffset.x, chaseOffset.z, -chaseOffset.y);
    controls.target.set(chaseAim.x, 4, -chaseAim.y);
    camera.lookAt(controls.target);
  } else {
    controls.update();
  }

  renderer.render(scene, camera);

  drawRadar(now, fr);
  drawTrend(now);

  if (now - lastUi > 50) {
    lastUi = now;
    updatePanel(now, fr);
  }
}

function updatePanel(now, fr) {
  const age = live.lastFrameAt ? now - live.lastFrameAt : Infinity;
  $("rateText").textContent = live.source ? `${live.frameTimes.length} Hz` : "— Hz";
  $("ageText").textContent = live.source && isFinite(age) ? `${Math.round(age)} ms` : "—";

  if (live.source) {
    const name = live.source === "demo" ? "demo" : "USB";
    if (!fr) setLink("stale", live.source === "serial" ? "waiting for stream…" : name);
    else if (age > 300) setLink("stale", `${name} · stale`);
    else setLink("live", `${name} · live`);
  }

  $("calBanner").classList.toggle("show", !!fr && fr.cal === "running");
  $("echoCount").textContent = echoXY.length;
  $("usRate").textContent = live.source ? `${live.usTimes.length}` : "—";

  if (!fr) return;

  $("hudYaw").textContent = fmtDeg(THREE.MathUtils.radToDeg(euler.z));
  // Nose up = positive pitch; right side down = positive roll.
  $("hudPitch").textContent = fmtDeg(-THREE.MathUtils.radToDeg(euler.y));
  $("hudRoll").textContent = fmtDeg(THREE.MathUtils.radToDeg(euler.x));

  const valid = fr.dist > 0;
  $("distBig").innerHTML = valid ? `${fr.dist.toFixed(1)}<small>cm</small>` : `—<small>cm</small>`;
  $("distBig").style.color = valid ? cssColor(distColor(fr.dist)) : "var(--muted)";
  const chip = $("usChip");
  chip.textContent = fr.usStatus.replaceAll("_", " ");
  chip.className = "chip " + (fr.usStatus === "ok" ? (fr.dist <= OBSTACLE_CM ? "bad" : "ok") : fr.usStatus === "no_echo" ? "warn" : "bad");

  setBar(gyroBars.x, fr.gyro[0], 360, `${fr.gyro[0].toFixed(1)}°/s`);
  setBar(gyroBars.y, fr.gyro[1], 360, `${fr.gyro[1].toFixed(1)}°/s`);
  setBar(gyroBars.z, fr.gyro[2], 360, `${fr.gyro[2].toFixed(1)}°/s`);
  setBar(motorBars.l, fr.left, 255, `${fr.left}`);
  setBar(motorBars.r, fr.right, 255, `${fr.right}`);

  const a = Math.hypot(...fr.acc);
  $("accMag").textContent = `${a.toFixed(2)} g`;
  $("calState").textContent = fr.cal;
  $("imuState").textContent = fr.imuOk ? "ok" : "FAULT";
  $("imuState").style.color = fr.imuOk ? "" : "var(--bad)";
  $("modeText").textContent = fr.mode;
  $("dropText").textContent = `${fr.dropped}`;
}

updateMount();
setView(settings.view);
setButtons();
tick();

// viewer/index.html?demo starts the simulated robot straight away.
const params = new URLSearchParams(location.search);
if (params.has("view")) setView(params.get("view"));
if (params.has("demo")) startDemo();
