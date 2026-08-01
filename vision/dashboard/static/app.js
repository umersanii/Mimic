const FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"];
const FINGER_LABELS = { thumb: "T", index: "I", middle: "M", ring: "R", pinky: "P" };

const videoEl = document.getElementById("video");
const simVideoEl = document.getElementById("sim-video");
const fpsPill = document.getElementById("fps-pill");
const serialPill = document.getElementById("serial-pill");
const gazeboPill = document.getElementById("gazebo-pill");

const SIM_CAMERA_PORT = 8766;

// Static per-sensor metadata mirrored from sim/worlds/hand_world.sdf - these cameras are
// all <static>true</static> single-link models, so their pose/fov never change at
// runtime (and can't be usefully teleported live - see CLAUDE.md's set_pose note). Kept
// here as plain data for display only; if a pose is retuned in the SDF, update it here
// too, there's no live source of truth the browser can query for this.
const CAMERA_INFO = {
  overview_front: { pose: [0, 0.85, 0.78], rpy: [0, 0.12, -1.5708], fov: 1.05, topic: "overview_camera/image" },
  left_front: { pose: [-0.2, 0.7, 0.75], rpy: [0, 0.12, -1.5708], fov: 1.0, topic: "left_hand_camera/image" },
  right_front: { pose: [0.2, 0.7, 0.75], rpy: [0, 0.12, -1.5708], fov: 1.0, topic: "right_hand_camera/image" },
  overview_back: { pose: [0, -0.85, 0.78], rpy: [0, 0.12, 1.5708], fov: 1.05, topic: "back_overview_camera/image" },
  left_back: { pose: [-0.2, -0.7, 0.75], rpy: [0, 0.12, 1.5708], fov: 1.0, topic: "back_left_hand_camera/image" },
  right_back: { pose: [0.2, -0.7, 0.75], rpy: [0, 0.12, 1.5708], fov: 1.0, topic: "back_right_hand_camera/image" },
};
const cameraInfoGridEl = document.getElementById("camera-info-grid");

function statCard(label, value) {
  return `<div class="camera-info-stat">
    <div class="camera-info-stat-label">${label}</div>
    <div class="camera-info-stat-value">${value}</div>
  </div>`;
}

function updateCameraInfo(camera, side) {
  const info = CAMERA_INFO[`${camera}_${side}`];
  if (!info || !cameraInfoGridEl) return;
  const [x, y, z] = info.pose;
  const [r, p, yw] = info.rpy;
  cameraInfoGridEl.innerHTML = [
    statCard("CAMERA", `${camera.toUpperCase()} / ${side.toUpperCase()}`),
    statCard("POSITION (m)", `x=${x} y=${y} z=${z}`),
    statCard("ORIENTATION (rad)", `roll=${r} pitch=${p} yaw=${yw}`),
    statCard("HORIZONTAL FOV (rad)", info.fov),
    statCard("RESOLUTION", "640 &times; 480 @ 30Hz"),
    statCard("TOPIC", info.topic),
  ].join("");
}

// Live angle = curl fraction (0..1, already streamed per-frame in msg.hands.<L|R>.curls)
// times JOINT_MAX_RAD - the exact scaling sim/bridge/gz_hand_bridge.py applies before
// publishing each driver joint's cmd_pos, so this is the real commanded angle, not an
// approximation.
const JOINT_MAX_RAD = 1.5708;

// Static driver-joint mount pose (xyz, rpy) per hand/finger, transcribed from
// sim/models/hand/generate_hand.py's finger_chain() (flip=-1 for L, +1 for R) - these are
// fixed joint origins baked into hand.urdf, not something the browser can query live, so
// kept here as plain data same as CAMERA_INFO above. Position is the joint origin
// relative to hand_link, not a live fingertip position - forward kinematics through the
// curled chain isn't computed anywhere in this project yet.
const FINGER_JOINT_POSE = {
  L: {
    thumb: { xyz: [0.0, -0.029, -0.0577], rpy: [-0.1, 0.0, 0.0] },
    index: { xyz: [-0.0015, -0.0342, -0.119], rpy: [-0.1, 0.0, 0.0] },
    middle: { xyz: [-0.00175, -0.007, -0.12325], rpy: [0.0, 0.0, 0.0] },
    ring: { xyz: [0.0, 0.00705, -0.0794], rpy: [-0.7, 0.0, 0.0] },
    pinky: { xyz: [0.0, 0.027, -0.0555], rpy: [-0.7, 0.0, 0.0] },
  },
  R: {
    thumb: { xyz: [0.0, 0.029, -0.0577], rpy: [0.1, 0.0, 0.0] },
    index: { xyz: [-0.0015, 0.0342, -0.119], rpy: [0.1, 0.0, 0.0] },
    middle: { xyz: [-0.00175, 0.007, -0.12325], rpy: [0.0, 0.0, 0.0] },
    ring: { xyz: [0.0, -0.00705, -0.0794], rpy: [0.7, 0.0, 0.0] },
    pinky: { xyz: [0.0, -0.027, -0.0555], rpy: [0.7, 0.0, 0.0] },
  },
};

const fingerTables = {
  L: document.getElementById("finger-table-L"),
  R: document.getElementById("finger-table-R"),
};

function buildFingerTable(hand) {
  const table = fingerTables[hand];
  const rows = FINGER_ORDER.map((finger) => {
    const { xyz, rpy } = FINGER_JOINT_POSE[hand][finger];
    return `<tr data-finger="${finger}">
      <td class="finger-name">${finger.toUpperCase()}</td>
      <td class="finger-curl">—</td>
      <td class="finger-angle">—</td>
      <td>${xyz.join(", ")}</td>
      <td>${rpy.join(", ")}</td>
    </tr>`;
  }).join("");
  table.innerHTML = `<thead><tr>
      <th>Finger</th><th>Curl</th><th>Angle (rad)</th>
      <th>Joint pos (m)</th><th>Joint rpy (rad)</th>
    </tr></thead><tbody>${rows}</tbody>`;
}
buildFingerTable("L");
buildFingerTable("R");

function updateFingerTable(hand, curls, tracked) {
  const table = fingerTables[hand];
  table.classList.toggle("not-tracked", !tracked);
  for (const finger of FINGER_ORDER) {
    const row = table.querySelector(`tr[data-finger="${finger}"]`);
    const t = tracked ? Math.max(0, Math.min(1, curls[finger] ?? 0)) : null;
    row.querySelector(".finger-curl").textContent = t === null ? "—" : t.toFixed(2);
    row.querySelector(".finger-angle").textContent = t === null ? "—" : (t * JOINT_MAX_RAD).toFixed(3);
  }
}

function setVideoOffline(imgEl, offline) {
  imgEl.closest(".video-wrap").classList.toggle("offline", offline);
}
setVideoOffline(videoEl, true);
setVideoOffline(simVideoEl, true);

const handPanels = {
  L: document.querySelector('.hand-panel[data-hand="L"]'),
  R: document.querySelector('.hand-panel[data-hand="R"]'),
};

function buildGauges(panel) {
  const container = panel.querySelector(".gauges");
  for (const finger of FINGER_ORDER) {
    const gauge = document.createElement("div");
    gauge.className = "gauge";
    gauge.dataset.finger = finger;
    gauge.innerHTML = `
      <div class="gauge-track"><div class="gauge-fill"></div></div>
      <div class="gauge-label">${FINGER_LABELS[finger]}</div>
    `;
    container.appendChild(gauge);
  }
}
buildGauges(handPanels.L);
buildGauges(handPanels.R);

function setPill(el, state, labels) {
  el.classList.remove("ok", "bad", "warn");
  const cls = { connected: "ok", disconnected: "bad", disabled: "warn" }[state] || "warn";
  el.classList.add(cls);
  el.textContent = labels[state] ?? labels.disabled;
}

function updateHand(label, data) {
  const panel = handPanels[label];
  const tracked = !!(data && data.tracked);
  panel.classList.toggle("not-tracked", !tracked);
  panel.querySelector(".hand-count").textContent = tracked
    ? `${data.count}/5`
    : "NOT TRACKED";

  const curls = (data && data.curls) || {};
  for (const finger of FINGER_ORDER) {
    const fill = panel.querySelector(`.gauge[data-finger="${finger}"] .gauge-fill`);
    const t = tracked ? Math.max(0, Math.min(1, curls[finger] ?? 0)) : 0;
    fill.style.height = `${t * 100}%`;
  }
  updateFingerTable(label, curls, tracked);
}

function handleMessage(msg) {
  videoEl.src = msg.image;
  setVideoOffline(videoEl, false);

  const fps = Math.round(msg.fps ?? 0);
  fpsPill.textContent = `FPS ${fps}`;
  fpsPill.classList.remove("ok", "warn", "bad");
  fpsPill.classList.add(fps >= 20 ? "ok" : fps >= 10 ? "warn" : "bad");

  setPill(serialPill, msg.serial, {
    connected: "SERIAL LINKED",
    disconnected: "SERIAL LOST",
    disabled: "NO SERIAL",
  });
  setPill(gazeboPill, msg.gazebo, {
    connected: "GAZEBO LINKED",
    disconnected: "GAZEBO LOST",
    disabled: "NO GAZEBO",
  });

  const hands = msg.hands || {};
  updateHand("L", hands.L);
  updateHand("R", hands.R);
}

// Stored (rather than a local inside connect()) so the refresh button can force-close
// and reopen it, same reasoning as simWs below: a dead server-side link (e.g. the
// gz_hand_bridge docker-exec pipe hand_tracker.py holds open dying) doesn't always fire
// a clean onclose on this socket, so the plain 1s auto-retry loop can sit stuck on a
// half-open connection indefinitely without a manual kick.
let mainWs = null;

function connect() {
  if (mainWs) {
    mainWs.onclose = null; // being replaced, not lost - don't double-reconnect
    mainWs.close();
  }
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${window.location.host}/ws`);
  mainWs = ws;
  ws.onmessage = (event) => handleMessage(JSON.parse(event.data));
  ws.onclose = () => {
    setVideoOffline(videoEl, true);
    setTimeout(connect, 1000);
  };
  ws.onerror = () => ws.close();
}
connect();

// Sim camera bridge is a separate server/process (sim/bridge/gz_camera_dashboard.py,
// runs inside the Gazebo container) - independent connection, independent reconnect
// loop, so a sim feed that isn't running yet doesn't block the webcam panel.
//
// gz_camera_dashboard.py multiplexes 6 cameras (overview/left/right x front/back) as
// separate WebSocket paths (/ws/<camera>/<side>), each backed by its own gz-transport
// subscription - switching either the camera or the side here just reconnects to a
// different path, it doesn't filter a shared stream.
let simWs = null;
let activeCamera = "overview";
let activeSide = "front";

function connectSim(camera = activeCamera, side = activeSide) {
  activeCamera = camera;
  activeSide = side;
  updateCameraInfo(camera, side);
  if (simWs) {
    simWs.onclose = null; // this connection is being replaced, not lost - don't reconnect it
    simWs.close();
  }
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${window.location.hostname}:${SIM_CAMERA_PORT}/ws/${camera}/${side}`);
  ws.binaryType = "blob";
  simWs = ws;
  // Raw JPEG bytes over a binary frame now (gz_camera_dashboard.py dropped base64-in-JSON
  // to cut encode CPU + payload size) - turn each frame into an object URL and revoke the
  // previous one so blobs don't pile up.
  let lastObjectUrl = null;
  ws.onmessage = (event) => {
    const objectUrl = URL.createObjectURL(event.data);
    simVideoEl.src = objectUrl;
    if (lastObjectUrl) URL.revokeObjectURL(lastObjectUrl);
    lastObjectUrl = objectUrl;
    setVideoOffline(simVideoEl, false);
  };
  ws.onclose = () => {
    setVideoOffline(simVideoEl, true);
    setTimeout(() => connectSim(activeCamera, activeSide), 1000);
  };
  ws.onerror = () => ws.close();
}
connectSim();

const cameraSwitcher = document.getElementById("camera-switcher");
cameraSwitcher.addEventListener("click", (event) => {
  const btn = event.target.closest(".camera-btn");
  if (!btn || btn.classList.contains("active")) return;
  cameraSwitcher.querySelector(".camera-btn.active")?.classList.remove("active");
  btn.classList.add("active");
  connectSim(btn.dataset.camera, activeSide);
});

const sideSwitcher = document.getElementById("side-switcher");
sideSwitcher.addEventListener("click", (event) => {
  const btn = event.target.closest(".side-btn");
  if (!btn || btn.classList.contains("active")) return;
  sideSwitcher.querySelector(".side-btn.active")?.classList.remove("active");
  btn.classList.add("active");
  connectSim(activeCamera, btn.dataset.side);
});

// Manual reconnect button: forces fresh WebSockets on BOTH links, not just the sim
// camera feed - the GAZEBO status pill comes from mainWs (the /ws connection to
// hand_tracker.py's own dashboard server), which can just as easily get stuck on a
// dead-but-not-closed socket as the camera link can (e.g. gz_hand_bridge's docker-exec
// pipe died server-side without the browser's socket seeing a clean close). Previously
// this button only re-kicked connectSim(), so a lost Gazebo link had no manual recovery
// path short of a full page reload.
const simRefreshBtn = document.getElementById("sim-refresh-btn");
simRefreshBtn.addEventListener("click", () => {
  connect();
  connectSim(activeCamera, activeSide);
});
