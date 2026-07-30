const FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"];
const FINGER_LABELS = { thumb: "T", index: "I", middle: "M", ring: "R", pinky: "P" };

const videoEl = document.getElementById("video");
const simVideoEl = document.getElementById("sim-video");
const fpsPill = document.getElementById("fps-pill");
const serialPill = document.getElementById("serial-pill");
const gazeboPill = document.getElementById("gazebo-pill");

const SIM_CAMERA_PORT = 8766;

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

function connect() {
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${window.location.host}/ws`);
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
function connectSim() {
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${window.location.hostname}:${SIM_CAMERA_PORT}/ws`);
  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    simVideoEl.src = msg.image;
    setVideoOffline(simVideoEl, false);
  };
  ws.onclose = () => {
    setVideoOffline(simVideoEl, true);
    setTimeout(connectSim, 1000);
  };
  ws.onerror = () => ws.close();
}
connectSim();
