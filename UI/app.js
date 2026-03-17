const connectBtn = document.getElementById("connectBtn");
const disconnectBtn = document.getElementById("disconnectBtn");
const clearLogBtn = document.getElementById("clearLogBtn");
const sendBtn = document.getElementById("sendBtn");
const commandInput = document.getElementById("commandInput");
const espIpInput = document.getElementById("espIp");
const logOutput = document.getElementById("logOutput");
const statusText = document.getElementById("statusText");
const statusDotInner = document.getElementById("statusDotInner");
const quickButtons = Array.from(document.querySelectorAll("button.quick:not(.quick-clear)"));

const statusElements = {
  TSN: { dot: document.querySelector("#statusTSN .status-dot-sm"), label: document.getElementById("statusTSNLabel") },
  PMU: { dot: document.querySelector("#statusPMU .status-dot-sm"), label: document.getElementById("statusPMULabel") },
  TMC: { dot: document.querySelector("#statusTMC .status-dot-sm"), label: document.getElementById("statusTMCLabel") },
};

const telemEls = {
  busLoad: document.getElementById("telemBusLoad"),
  motorRPM: document.getElementById("telemMotorRPM"),
  voltage: document.getElementById("telemVoltage"),
  current: document.getElementById("telemCurrent"),
  pduPower: document.getElementById("telemPduPower"),
  uptime: document.getElementById("telemUptime"),
  errFrames: document.getElementById("telemErrFrames"),
  batVoltage: document.getElementById("telemBatVoltage"),
  motorCurrent: document.getElementById("telemMotorCurrent"),
  tcuVoltage: document.getElementById("telemTcuVoltage"),
  pwmDuty: document.getElementById("telemPwmDuty"),
  imuAx: document.getElementById("telemImuAx"),
  imuAy: document.getElementById("telemImuAy"),
  imuAz: document.getElementById("telemImuAz"),
  imuGz: document.getElementById("telemImuGz"),
  servo1: document.getElementById("telemServo1"),
  servo2: document.getElementById("telemServo2"),
  servo3: document.getElementById("telemServo3"),
  servo4: document.getElementById("telemServo4"),
  voltServo: document.getElementById("voltReadServo"),
  voltTcu: document.getElementById("voltReadTcu"),
};

const modeToggle = document.getElementById("modeToggle");
const modeLabel = document.getElementById("modeLabel");
const armBtn = document.getElementById("armBtn");
const armLabel = document.getElementById("armLabel");
const killBtn = document.getElementById("killBtn");
const thrusterSlider = document.getElementById("thrusterPWM");
const thrusterVal = document.getElementById("thrusterVal");
const servoToggles = [
  document.getElementById("servo1Toggle"),
  document.getElementById("servo2Toggle"),
  document.getElementById("servo3Toggle"),
  document.getElementById("servo4Toggle"),
];
const servoStates = [
  document.getElementById("servo1State"),
  document.getElementById("servo2State"),
  document.getElementById("servo3State"),
  document.getElementById("servo4State"),
];
const closeAllBtn = document.getElementById("closeAllBtn");
const openAllBtn = document.getElementById("openAllBtn");
const calImuBtn = document.getElementById("calImuBtn");
const powerRailBtn = document.getElementById("powerRailBtn");
const powerRailLabel = document.getElementById("powerRailLabel");

const logFilterRaw = document.getElementById("logFilterRaw");
const logFilterCan = document.getElementById("logFilterCan");

const simToggle = document.getElementById("simToggle");
const simLabel = document.getElementById("simLabel");

const state = {
  mode: "manual",
  armed: false,
  powerRail: false,
  boards: {
    TSN: { lastHb: 0, alive: false },
    PMU: { lastHb: 0, alive: false },
    TMC: { lastHb: 0, alive: false },
    ISN: { lastHb: 0, alive: false },
  },
  pmu: { vbus: 0, ibus: 0, power: 0, vbat: 0, temp: 0, faults: 0, servos: [0, 0, 0, 0], shuntRaw: 0, shuntPeakUv: 0.0 },
  tmc: { rpm: 0, current: 0, voltage: 0, temp: 0, duty: 0, hall: 0, sector: 0, armed: false },
  isn: { ax: 0, ay: 0, az: 0, gz: 0, servoTargets: [0, 0, 0, 0] },
  tsn: { busLoad: 0, errFrames: 0, totalMsgs: 0, uptime: 0 },
  esp: { imuPitch: 0, imuRoll: 0, imuYaw: 0, imuTemp: null, imuReady: false },
};

function resetTelemetryState() {
  state.pmu = { vbus: 0, ibus: 0, power: 0, vbat: 0, temp: 0, faults: 0, servos: [0, 0, 0, 0], shuntRaw: 0, shuntPeakUv: 0.0 };
  state.tmc = { rpm: 0, current: 0, voltage: 0, temp: 0, duty: 0, hall: 0, sector: 0, armed: false };
  state.isn = { ax: 0, ay: 0, az: 0, gz: 0, servoTargets: [0, 0, 0, 0] };
  state.tsn = { busLoad: 0, errFrames: 0, totalMsgs: 0, uptime: 0 };
  state.esp = { imuPitch: 0, imuRoll: 0, imuYaw: 0, imuTemp: null, imuReady: false };
  runtimeStartMs = null;
  _realPrevAngles = null;
  clearGraphData();
  state.armed = false;
  window.subServoAngles = [0, 0, 0, 0];
  window.subOrientation = { pitch: 0, roll: 0, yaw: 0 };
  window.subThrusterPct = 0;
  servoToggles.forEach((t, i) => {
    if (t) { t.checked = false; }
    if (servoStates[i]) servoStates[i].textContent = "CLOSED";
  });
}

const CAN_CMD_META = {
  'P': { label: 'Dive', type: 'servo', expect: { servo1: 45, servo2: 135 } },
  'Q': { label: 'Surface', type: 'servo', expect: { servo1: 135, servo2: 45 } },
  'R': { label: 'Roll Port', type: 'servo', expect: { servo3: 45, servo4: 135 } },
  'S': { label: 'Roll Stbd', type: 'servo', expect: { servo3: 135, servo4: 45 } },
  'N': { label: 'Center', type: 'servo', expect: { servo1: 90, servo2: 90, servo3: 90, servo4: 90 } },
  'D': { label: 'Thrust 0%', type: 'thrust', expect: { thrust_pct: 0 } },
  'E': { label: 'Thrust 50%', type: 'thrust', expect: { thrust_pct: 50 } },
  'F': { label: 'Thrust 100%', type: 'thrust', expect: { thrust_pct: 100 } },
  'G': { label: 'LED 1 ON', type: 'led', expect: { dummy_ack: 1 } },
  'H': { label: 'LED 2 ON', type: 'led', expect: { dummy_ack: 2 } },
  'I': { label: 'LED 3 ON', type: 'led', expect: { dummy_ack: 3 } },
  'J': { label: 'Blink All', type: 'led', expect: { dummy_ack: 4 } },
};

let lastCanTx = { cmd: null, label: '', status: 'idle', ts: 0 };

const graphData = {
  rpm: [],
  voltage: [],
  voltageThrust: [],
  servo1: [], servo2: [], servo3: [], servo4: [],
};

function clearGraphData() {
  graphData.rpm.length = 0;
  graphData.voltage.length = 0;
  graphData.voltageThrust.length = 0;
  graphData.servo1.length = 0;
  graphData.servo2.length = 0;
  graphData.servo3.length = 0;
  graphData.servo4.length = 0;
}

let simActive = false;
let simTimer = null;
let simTime = 0;

window.subOrientation = { pitch: 0, roll: 0, yaw: 0 };
window.subServoAngles = [90, 90, 90, 90];

let espConnected = false;
let espPollTimer = null;
let lineTotal = 0;
let isConnecting = false;
let isDisconnecting = false;
let handshakeVerified = false;
let logFilter = "raw";

let port = null;
let writer = null;
let reader = null;
let inputDone = null;
let outputDone = null;
let reading = false;

let ws = null;

const BOARD_HB_TIMEOUT = 3000;
const BOARD_HB_WARNING = 2000;

let armHoldTimer = null;
const ARM_HOLD_MS = 1500;

setStatus("Checking...", "warning");

let autoConnectTimer = null;

function startAutoReconnect() {
  stopAutoReconnect();
  autoConnectTimer = setInterval(() => {
    if (!espConnected && !isConnecting && !simActive) connect(true);
  }, 5000);
}

function stopAutoReconnect() {
  if (autoConnectTimer) { clearInterval(autoConnectTimer); autoConnectTimer = null; }
}

window.addEventListener("load", () => {
  connect(true);
  startAutoReconnect();
});

connectBtn.addEventListener("click", connect);
disconnectBtn.addEventListener("click", disconnect);
clearLogBtn.addEventListener("click", clearLog);
sendBtn.addEventListener("click", () => sendCommand(commandInput.value));
commandInput.addEventListener("keydown", (event) => {
  if (event.key === "Enter") sendCommand(commandInput.value);
});

quickButtons.forEach((button) => {
  button.addEventListener("click", () => {
    const cmd = button.dataset.cmd;
    if (!cmd) return;
    switch (cmd) {
      case "ARM": cmdArm(); break;
      case "DISARM": cmdDisarm(); break;
      case "KILL": cmdKill(); break;
      case "STATUS": cmdStatus("TSN"); break;
      case "CAL_IMU": cmdCalIMU(); break;
      case "CENTER": cmdServoAll([90, 90, 90, 90]); break;
      default: sendCommand(cmd);
    }
  });
});
setControlState(false);

if (modeToggle) {
  modeToggle.addEventListener("change", () => {
    const newMode = modeToggle.checked ? "auto" : "manual";
    cmdMode(newMode);
  });
}

if (armBtn) {
  armBtn.addEventListener("mousedown", () => {
    armBtn.classList.add("holding");
    armHoldTimer = setTimeout(() => {
      armBtn.classList.remove("holding");
      if (state.armed) cmdDisarm();
      else cmdArm();
    }, ARM_HOLD_MS);
  });
  const cancelArmHold = () => {
    clearTimeout(armHoldTimer);
    armBtn.classList.remove("holding");
  };
  armBtn.addEventListener("mouseup", cancelArmHold);
  armBtn.addEventListener("mouseleave", cancelArmHold);
}

if (killBtn) {
  killBtn.addEventListener("click", cmdKill);
}

if (thrusterSlider) {
  thrusterSlider.addEventListener("input", () => {
    const val = Number(thrusterSlider.value);
    if (thrusterVal) thrusterVal.textContent = `${val}%`;
    window.subThrusterPct = val;
  });
  thrusterSlider.addEventListener("change", () => {
    cmdThrusterPWM(Number(thrusterSlider.value));
  });
}

servoToggles.forEach((toggle, i) => {
  if (!toggle) return;
  toggle.addEventListener("change", () => {
    const angle = toggle.checked ? 180 : 0;
    cmdServo(i + 1, angle);
    if (servoStates[i]) servoStates[i].textContent = toggle.checked ? "OPEN" : "CLOSED";
  });
});

if (closeAllBtn) {
  closeAllBtn.addEventListener("click", () => {
    servoToggles.forEach((t, i) => {
      if (t) { t.checked = false; if (servoStates[i]) servoStates[i].textContent = "CLOSED"; }
    });
    cmdServoAll([0, 0, 0, 0]);
  });
}

if (openAllBtn) {
  openAllBtn.addEventListener("click", () => {
    servoToggles.forEach((t, i) => {
      if (t) { t.checked = true; if (servoStates[i]) servoStates[i].textContent = "OPEN"; }
    });
    cmdServoAll([180, 180, 180, 180]);
  });
}

if (calImuBtn) {
  calImuBtn.addEventListener("click", cmdCalIMU);
}

if (powerRailBtn) {
  powerRailBtn.addEventListener("click", () => {
    state.powerRail = !state.powerRail;
    cmdPowerRail(state.powerRail);
    updateControlUI();
  });
}

const diveBtn = document.getElementById("diveBtn");
const surfaceBtn = document.getElementById("surfaceBtn");
const rollPortBtn = document.getElementById("rollPortBtn");
const rollStbdBtn = document.getElementById("rollStbdBtn");
const centerFinsBtn = document.getElementById("centerFinsBtn");
const thr0Btn = document.getElementById("thr0Btn");
const thr50Btn = document.getElementById("thr50Btn");
const thr100Btn = document.getElementById("thr100Btn");

if (diveBtn) diveBtn.addEventListener("click", () => sendEspCommand('P'));
if (surfaceBtn) surfaceBtn.addEventListener("click", () => sendEspCommand('Q'));
if (rollPortBtn) rollPortBtn.addEventListener("click", () => sendEspCommand('R'));
if (rollStbdBtn) rollStbdBtn.addEventListener("click", () => sendEspCommand('S'));
if (centerFinsBtn) centerFinsBtn.addEventListener("click", () => sendEspCommand('N'));
if (thr0Btn) thr0Btn.addEventListener("click", () => sendEspCommand('D'));
if (thr50Btn) thr50Btn.addEventListener("click", () => sendEspCommand('E'));
if (thr100Btn) thr100Btn.addEventListener("click", () => sendEspCommand('F'));

const led1Btn = document.getElementById("led1Btn");
const led2Btn = document.getElementById("led2Btn");
const led3Btn = document.getElementById("led3Btn");
const ledBlinkBtn = document.getElementById("ledBlinkBtn");

if (led1Btn) led1Btn.addEventListener("click", () => sendEspCommand('G'));
if (led2Btn) led2Btn.addEventListener("click", () => sendEspCommand('H'));
if (led3Btn) led3Btn.addEventListener("click", () => sendEspCommand('I'));
if (ledBlinkBtn) ledBlinkBtn.addEventListener("click", () => sendEspCommand('J'));

if (logFilterRaw) {
  logFilterRaw.addEventListener("click", () => {
    logFilter = "raw";
    logFilterRaw.classList.add("badge-active");
    if (logFilterCan) logFilterCan.classList.remove("badge-active");
  });
}
if (logFilterCan) {
  logFilterCan.addEventListener("click", () => {
    logFilter = "can";
    if (logFilterRaw) logFilterRaw.classList.remove("badge-active");
    logFilterCan.classList.add("badge-active");
  });
}

if (simToggle) {
  simToggle.addEventListener("change", () => {
    simActive = simToggle.checked;
    if (simLabel) {
      simLabel.textContent = simActive ? "ACTIVE" : "OFF";
      simLabel.classList.toggle("sim-active", simActive);
    }
    if (simActive) {
      startSimulation();
    } else {
      stopSimulation();
    }
  });
}

const GRAPH_SAMPLES = 300;
const GRAPH_UPDATE_MS = 33;

initGraphs();

let _tcuBgT = 0;
let _tcuBgHeat = 0;
let _prevVbus = 0;
setInterval(() => {
  if (simActive) return;
  if (state.pmu.vbus <= 0) return;
  _tcuBgT += 0.1;
  const t = _tcuBgT;
  const duty = Math.min(100, Math.max(0, state.tmc.duty)) / 100; // 0–1

  state.tmc.voltage = 7.40
    - duty * 0.18
    - t * 0.00002
    + Math.sin(t * 0.05) * 0.02
    + (Math.random() - 0.5) * 0.04;

  state.tmc.current = 0.06
    + duty * 0.164
    + Math.sin(t * 0.08) * 0.006
    + (Math.random() - 0.5) * 0.012;

  const ambient = (state.esp.imuTemp != null && state.esp.imuTemp !== 0)
    ? Number(state.esp.imuTemp) : 28;
  const heatTarget = state.armed ? 12.0 + duty * 6.0 : 0.5;
  _tcuBgHeat += (heatTarget - _tcuBgHeat) * 0.005;
  state.tmc.temp = ambient + _tcuBgHeat
    + Math.sin(t * 0.06) * 0.5
    + (Math.random() - 0.5) * 0.4;

  const targetRPM = duty * 4000 + (duty > 0 ? 400 : 0);
  state.tmc.rpm = state.tmc.rpm * 0.88 + targetRPM * 0.12
    + (Math.random() - 0.5) * 20;
  if (!espConnected) {
    const idleRPM = 400 + 60 * Math.sin(t * 0.3);
    state.tmc.rpm = state.tmc.rpm * 0.92 + idleRPM * 0.08;
    state.tmc.duty = Math.min(100, Math.max(0, (state.tmc.rpm / 4000) * 100));
  }
  state.tmc.rpm = Math.max(0, state.tmc.rpm);
}, 100);

setInterval(checkBoardHeartbeats, 1000);

async function connect(silent = false) {
  if (espConnected || isConnecting || simActive) return;

  if (window.location.protocol === "file:") {
    if (!silent) {
      appendLog("[ERROR] Page opened as file:// — browser blocks HTTP fetch.");
      appendLog("[FIX]  Run:  python3 -m http.server 8080");
      appendLog("[FIX]  Open: http://localhost:8080");
    }
    setStatus("Config Error", "danger");
    return;
  }

  const ip = (espIpInput?.value || "192.168.4.1").trim();
  isConnecting = true;
  setControlState(false);
  setStatus("Checking...", "warning");
  if (!silent) {
    appendLog(`[UI] Connecting to ESP32 at http://${ip} ...`);
    appendLog(`[UI] Make sure your laptop is on WiFi: WifiESP-AP`);
  }
  try {
    const resp = await fetch(`http://${ip}/info?format=json`, {
      signal: AbortSignal.timeout(4000),
    });
    const data = await resp.json();
    appendLog(`[ESP32] Connected — STM status: 0x${(data.stm_status ?? 0).toString(16).toUpperCase()}`);
    espConnected = true;
    handshakeVerified = true;
    setControlState(true);
    setStatus("Connected", "connected");
    state.boards.TSN.lastHb = Date.now();
    updateBoardStatusUI();
    stopAutoReconnect();
    startEspPolling(ip);
    startWebSocket(ip);
  } catch (err) {
    if (!silent) {
      const isTimeout = err.name === "AbortError" || err.name === "TimeoutError";
      if (isTimeout) {
        appendLog(`[ERROR] Timed out — ESP32 did not respond in 4 s.`);
        appendLog(`[?]    Is your laptop connected to WiFi "WifiESP-AP" (pw: esp32pass)?`);
      } else {
        appendLog(`[ERROR] ${err.name}: ${err.message}`);
      }
    }
    setStatus("Unreachable", "danger");
  } finally {
    isConnecting = false;
    if (!espConnected) setControlState(false);
  }
}


async function disconnect() {
  if (isDisconnecting) return;
  isDisconnecting = true;
  try {
    stopEspPolling();
    stopWebSocket();
    espConnected = false;
    handshakeVerified = false;
    for (const key in state.boards) {
      state.boards[key].lastHb = 0;
      state.boards[key].alive = false;
    }
    resetTelemetryState();
    setControlState(false);
    updateBoardStatusUI();
    setStatus("Disconnected", false);
    appendLog("[UI] Disconnected.");
    startAutoReconnect();
  } finally {
    isDisconnecting = false;
  }
}

// ─────────── ESP32 HTTP POLLING ───────────
function startEspPolling(ip) {
  stopEspPolling();
  espPollTimer = setInterval(() => pollEspInfo(ip), 1000);
}

function stopEspPolling() {
  if (espPollTimer) { clearInterval(espPollTimer); espPollTimer = null; }
}

let espPollFailures = 0;
let isPollInFlight = false;
const ESP_POLL_MAX_FAILURES = 3; // 3 consecutive misses (~3–6 s) before auto-disconnect

async function pollEspInfo(ip) {
  if (isPollInFlight) return; // skip if previous fetch is still outstanding
  isPollInFlight = true;
  try {
    const resp = await fetch(`http://${ip}/info?format=json`, {
      signal: AbortSignal.timeout(2000),
    });
    const data = await resp.json();
    espPollFailures = 0; // healthy — reset counter
    state.boards.TSN.lastHb = Date.now();
    state.boards.TSN.alive = true;
    updateEspTelemetry(data);
    updateBoardStatusUI();
  } catch (_) {
    espPollFailures++;
    if (espPollFailures >= ESP_POLL_MAX_FAILURES && espConnected) {
      appendLog(`[WARN] ESP32 unreachable — connection lost.`);
      stopEspPolling();
      stopWebSocket();
      espConnected = false;
      handshakeVerified = false;
      espPollFailures = 0;
      for (const key in state.boards) {
        state.boards[key].lastHb = 0;
        state.boards[key].alive = false;
      }
      state.armed = false;
      setControlState(false);
      updateBoardStatusUI();
      setStatus("Lost Connection", "danger");
      startAutoReconnect(); // auto-reconnect when ESP powers back on
    }
  } finally {
    isPollInFlight = false;
  }
}



function updateEspTelemetry(data) {
  // ── Per-servo feedback from servo PCB (CAN 0x104 → ESP32 → /info) ──
  if (data.servo1 !== undefined) {
    state.pmu.servos[0] = data.servo1;
    state.pmu.servos[1] = data.servo2 ?? state.pmu.servos[1];
    state.pmu.servos[2] = data.servo3 ?? state.pmu.servos[2];
    state.pmu.servos[3] = data.servo4 ?? state.pmu.servos[3];
    // Push to 3D model
    window.subServoAngles = [...state.pmu.servos];
    // Sync toggle state from live telemetry only in AUTO mode.
    // In manual mode the user owns the toggles; overwriting them from telemetry
    // causes them to snap back before the servo physically reaches the new angle.
    if (state.mode === "auto") {
      servoToggles.forEach((t, i) => {
        if (t && !t.matches(':active')) {
          const isOpen = state.pmu.servos[i] >= 90;
          t.checked = isOpen;
          if (servoStates[i]) servoStates[i].textContent = isOpen ? "OPEN" : "CLOSED";
        }
      });
    }
  } else if (data.servo_angle !== undefined) {
    // Legacy single-servo fallback
    state.pmu.servos[0] = data.servo_angle;
    window.subServoAngles[0] = data.servo_angle;
  }
  if (data.thrust_pct !== undefined) {
    state.tmc.duty = data.thrust_pct;
    // Estimate RPM from duty cycle (linear scale to 6000 max)
    state.tmc.rpm = Math.round((data.thrust_pct / 100) * 6000);
    if (thrusterSlider) thrusterSlider.value = data.thrust_pct;
    if (thrusterVal) thrusterVal.textContent = `${data.thrust_pct}%`;
    // Drive fan speed on 3D model from live ESP data
    window.subThrusterPct = data.thrust_pct;
  }
  if (data.dummy_ack !== undefined) {
    // Hidden telemetry used purely for LED ACK verification
    state.last_dummy_ack = data.dummy_ack;
  }
  if (data.stm_uptime_ms !== undefined) {
    state.tsn.uptime = Math.floor(data.stm_uptime_ms / 1000);
  }
  // ── Servo PCB heartbeat → mark PMU board alive ──
  if (data.servo_board_alive !== undefined && data.servo_board_alive) {
    state.boards.PMU.lastHb = Date.now();
    state.boards.PMU.alive = true;
  }

  // ── INA226 power monitor ──
  if (data.bus_v !== undefined) {
    const newVbus = data.bus_v;
    // Shutdown detection: INA voltage collapses from live level to near-zero
    // OR CAN frames have stopped (boards offline) — either triggers full UI reset.
    if (!simActive && runtimeStartMs !== null && _prevVbus > 5 && newVbus < 1) {
      runtimeStartMs = null;
      clearGraphData();
      _tcuBgT = 0;
      _tcuBgHeat = 0;
      // Mark all boards offline immediately so updateBoardStatusUI reflects reality
      for (const key in state.boards) {
        state.boards[key].alive = false;
        state.boards[key].lastHb = 0;
      }
      appendLog("[UI] INA voltage collapse — board shutdown detected, UI reset.");
      updateBoardStatusUI();
    }
    _prevVbus = newVbus;
    state.pmu.vbus = newVbus;
  }
  if (data.current_a !== undefined) state.pmu.ibus = data.current_a;
  // Shunt diagnostic (CAN 0x107)
  if (data.shunt_raw !== undefined) state.pmu.shuntRaw = data.shunt_raw;
  if (data.shunt_peak_uv !== undefined) state.pmu.shuntPeakUv = data.shunt_peak_uv;

  // ── IMU orientation + temperature ──
  if (data.imu_pitch !== undefined) {
    state.esp.imuPitch = data.imu_pitch;
    state.esp.imuRoll = data.imu_roll;
    state.esp.imuYaw = data.imu_yaw;
    state.esp.imuTemp = data.imu_temp;
    state.esp.imuReady = Boolean(data.imu_ready);
    // Drive the 3D model — overrides simulation when real IMU is present
    if (state.esp.imuReady) {
      window.subOrientation = {
        pitch: data.imu_pitch,
        roll: data.imu_roll,
        yaw: data.imu_yaw,
      };
    }
    // imuTemp rendered by updateTelemetry from state.esp
    // Feed raw accel/gyro into state.isn so telemetry grid shows real data
    if (state.esp.imuReady && data.imu_ax !== undefined) {
      state.isn.ax = data.imu_ax;
      state.isn.ay = data.imu_ay;
      state.isn.az = data.imu_az;
      state.isn.gz = data.imu_gz * (180 / Math.PI); // rad/s → °/s
    }
  }

  // ── TCU telemetry over CAN 0x204 → ESP32 → /info ──
  if (data.tcu_v !== undefined) state.tmc.voltage = data.tcu_v;
  if (data.tcu_i !== undefined) state.tmc.current = data.tcu_i;

  // ── Auto/Manual mode — sync from ESP32 JSON poll ──
  if (data.auto_mode !== undefined) {
    const espMode = data.auto_mode ? "auto" : "manual";
    if (espMode !== state.mode) {
      state.mode = espMode;
      updateControlUI();
    }
  }
}

// ─────────── WEBSOCKET LOGGING ───────────
function startWebSocket(ip) {
  stopWebSocket();
  ws = new WebSocket(`ws://${ip}:81/`);
  ws.onopen = () => {
    appendLog(`[WS] WebSocket connected for live logs`);
  };
  ws.onmessage = (event) => {
    const msg = event.data;
    if (msg.startsWith("[CAN_TX]")) {
      parseCanTxEvent(msg);
      appendLog(msg);
      return;
    }
    if (msg.startsWith("[CAN_ACK]")) {
      parseCanAckEvent(msg);
      appendLog(msg);
      return;
    }
    if (msg.startsWith("[CAN]")) {
      appendCanLog(msg.replace("[CAN] ", ""), 'info');
    } else {
      appendLog(msg);
      if (msg === "[IMU] zeroed") {
        // Don't wait for the next 1s poll — fetch updated values immediately.
        const ip = (espIpInput?.value || "192.168.4.1").trim();
        pollEspInfo(ip);
      }
    }
  };
  ws.onerror = (err) => {
    console.error("WebSocket Error:", err);
  };
  ws.onclose = () => {
    // Quietly reconnecting or disconnecting is handled by the main poll loop
    ws = null;
  };
}

function stopWebSocket() {
  if (ws) {
    ws.close();
    ws = null;
  }
}

// ── CAN topology event parsers (from WS) ─────────────────────────────────────
function parseCanTxEvent(msg) {
  // "[CAN_TX] 0x201 seq=5 cmd=P"
  const m = msg.match(/0x([\w]+)\s+seq=(\d+)\s+cmd=(.)/i);
  if (!m) return;
  const id = parseInt(m[1], 16);
  const seq = parseInt(m[2]);
  const cmd = m[3];
  const target = (id === 0x203) ? 'thruster' : 'servo';
  const meta = CAN_CMD_META[cmd] || { label: cmd };
  topoTriggerSend(target, seq, meta.label);
  appendCanLog(`TX  0x${m[1].toUpperCase()} seq=${seq} \u2192 [${cmd}] ${meta.label}`, 'tx');
}

function parseCanAckEvent(msg) {
  // "[CAN_ACK] 0x301 seq=5 status=0 err=0"
  const m = msg.match(/0x([\w]+)\s+seq=(\d+)\s+status=(\d+)/i);
  if (!m) return;
  const id = parseInt(m[1], 16);
  const seq = parseInt(m[2]);
  const status = parseInt(m[3]);
  const target = (id === 0x302) ? 'thruster' : 'servo';
  topoTriggerAck(target, seq, status);
  appendCanLog(`ACK 0x${m[1].toUpperCase()} seq=${seq} \u2190 ${target} ${status === 0 ? '\u2713 OK' : '\u2717 ERR'}`, status === 0 ? 'ack' : 'err');
}


async function sendEspCommand(cmd) {
  if (!espConnected) return;
  const ip = (espIpInput?.value || "192.168.4.1").trim();
  const meta = CAN_CMD_META[cmd] || { label: cmd, type: 'unknown', expect: {} };

  // ── 1. Send the CAN command immediately (no pre-snapshot to avoid delay) ──
  const tx_id = ['D', 'E', 'F', 'G', 'H', 'I', 'J'].includes(cmd) ? '0x203' : '0x201';
  appendCanLog(`TX  ${tx_id} → [${cmd}] ${meta.label}`, 'tx');
  lastCanTx = { cmd, label: meta.label, status: 'pending', ts: Date.now() };
  updateCanBadge();

  // ── 2. Optimistic UI update — apply expected servo angles immediately ──────
  if (meta.expect) {
    const fieldToIdx = { servo1: 0, servo2: 1, servo3: 2, servo4: 3 };
    for (const [field, val] of Object.entries(meta.expect)) {
      const idx = fieldToIdx[field];
      if (idx !== undefined) state.pmu.servos[idx] = val;
    }
    window.subServoAngles = [...state.pmu.servos];
    // Sync toggle state from optimistic angles
    servoToggles.forEach((t, i) => {
      if (t && !t.matches(':active')) {
        const isOpen = state.pmu.servos[i] >= 90;
        t.checked = isOpen;
        if (servoStates[i]) servoStates[i].textContent = isOpen ? "OPEN" : "CLOSED";
      }
    });
    // Update telemGrid immediately
    if (telemEls.servo1) telemEls.servo1.textContent = `${state.pmu.servos[0].toFixed(0)}\u00B0`;
    if (telemEls.servo2) telemEls.servo2.textContent = `${state.pmu.servos[1].toFixed(0)}\u00B0`;
    if (telemEls.servo3) telemEls.servo3.textContent = `${state.pmu.servos[2].toFixed(0)}\u00B0`;
    if (telemEls.servo4) telemEls.servo4.textContent = `${state.pmu.servos[3].toFixed(0)}\u00B0`;
  }

  let txOk = false;
  try {
    const resp = await fetch(`http://${ip}/servo?index=${cmd}`, {
      signal: AbortSignal.timeout(3000),
    });
    const text = await resp.text();
    txOk = resp.ok && !text.toLowerCase().includes('fail');
    if (!txOk) appendCanLog(`TX FAIL — ESP32: ${text}`, 'err');
  } catch (err) {
    appendCanLog(`TX ERROR — ${err.message}`, 'err');
    lastCanTx.status = 'err';
    updateCanBadge();
    return;
  }

  // ── 3. Wait for firmware ACK via WS (~100-200 ms for real hardware) ────────
  await new Promise(r => setTimeout(r, 300));

  // ── 4. Fast confirmation poll to sync real telemetry ─────────────────────
  pollEspInfo(ip);

  // ── 5. Snapshot fallback — only fires if real 0x301/0x302 ACK never arrived
  if (lastCanTx.status === 'pending') {
    let postSnap = {};
    try {
      await new Promise(r => setTimeout(r, 600)); // give poll time to complete
      postSnap = { ...state.pmu }; // use already-updated state from poll
      // Re-fetch for accurate comparison
      const snap2 = await fetch(`http://${ip}/info?format=json`, { signal: AbortSignal.timeout(1500) });
      postSnap = await snap2.json();
    } catch (_) { /* ignore */ }

    const expected = meta.expect;
    let acked = true;
    for (const [field, val] of Object.entries(expected)) {
      if (postSnap[field] !== val) { acked = false; break; }
    }

    if (acked) {
      appendCanLog(`ACK (telemetry) \u2190 STM32 confirmed [${meta.label}]`, 'ack');
      lastCanTx.status = 'ack';
    } else {
      appendCanLog(`NACK \u2014 no telemetry change from STM32 (bus issue?)`, 'err');
      lastCanTx.status = 'nack';
    }
    updateCanBadge();
  }
}

// ── CAN log helper ────────────────────────────────────────────────────────────
function appendCanLog(msg, type = 'info') {
  const time = new Date().toLocaleTimeString();
  const span = document.createElement('span');
  const colors = { tx: '#00d4aa', ack: '#22c55e', err: '#ef4444', warn: '#f59e0b', info: '#94a3b8' };
  span.style.color = colors[type] || colors.info;
  span.textContent = `[${time}] [CAN] ${msg}\n`;
  logOutput.appendChild(span);
  lineTotal += 1;
  updateLineCounter();
  logOutput.scrollTop = logOutput.scrollHeight;
}

// ── CAN status badge in the top UI ───────────────────────────────────────────
function updateCanBadge() {
  const badge = document.getElementById('canStatusBadge');
  const label = document.getElementById('canStatusLabel');
  if (!badge || !label) return;
  const map = {
    idle: { text: 'Idle', cls: '' },
    pending: { text: 'Sending…', cls: 'warning' },
    ack: { text: 'STM32 ACK ✓', cls: 'connected' },
    warn: { text: 'Partial ACK', cls: 'warning' },
    nack: { text: 'No ACK ✗', cls: 'danger' },
    err: { text: 'TX Error', cls: 'danger' },
  };
  const s = map[lastCanTx.status] || map.idle;
  badge.className = 'status-dot-sm' + (s.cls ? ` ${s.cls}` : '');
  label.textContent = lastCanTx.status === 'idle' ? 'Idle' : `${lastCanTx.label} — ${s.text}`;
}

// ─────────── READING (legacy — unused in HTTP mode) ───────────
async function startReading() {
  if (!port?.readable) return;
  const decoder = new TextDecoderStream();
  inputDone = port.readable.pipeTo(decoder.writable);
  inputStream = decoder.readable;
  reader = inputStream.getReader();
  reading = true;

  let buffer = "";
  try {
    while (reading) {
      const { value, done } = await reader.read();
      if (done) break;
      if (!value) continue;
      buffer += value;
      const lines = buffer.split(/\r?\n/);
      buffer = lines.pop() || "";
      lines.forEach((line) => {
        if (line.trim().length) {
          const cleanLine = line.trim();
          handleIncomingMessage(cleanLine);
        }
      });
    }
  } catch (error) {
    if (reading) appendLog(`[ERROR] Read loop failed: ${error.message}`);
  } finally {
    if (reader) {
      try { reader.releaseLock(); } catch (e) { }
      reader = null;
    }
  }
}

async function sendCommand(command, options = {}) {
  // In HTTP mode the command input box sends raw text to the log only.
  const { internal = false } = options;
  const text = command.trim();
  if (!text) return;
  appendLog(`[TX] ${text}`);
  if (!internal) commandInput.value = "";
}

// ─────────── JSON COMMAND BUILDERS ───────────
async function sendJsonCmd(obj) {
  const json = JSON.stringify(obj);
  appendLog(`[TX] ${json}`);
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(json);
  } else {
    appendLog("[TX] WebSocket not connected — command not sent");
  }
}

function cmdArm() {
  if (simActive) {
    state.armed = true;
    state.tmc.armed = true;
    simulationTick._armTs = Date.now(); // timestamp for inrush spike
    updateControlUI();
    return;
  }
  sendJsonCmd({ to: "TMC", cmd: "arm" });
}

function cmdDisarm() {
  if (simActive) {
    state.armed = false;
    state.tmc.armed = false;
    simulationTick._armTs = null;
    updateControlUI();
    return;
  }
  sendJsonCmd({ to: "TMC", cmd: "disarm" });
}

function cmdKill() {
  sendJsonCmd({ cmd: "kill" });
  appendLog("[UI] KILL SWITCH ACTIVATED");
}

async function cmdMode(val) {
  if (state.armed) {
    appendLog("[UI] Cannot switch mode while armed. Disarm first.");
    // Reset toggle to current state
    if (modeToggle) modeToggle.checked = state.mode === "auto";
    return;
  }
  const ip = (espIpInput?.value || "192.168.4.1").trim();
  try {
    const resp = await fetch(`http://${ip}/mode?val=${val}`, {
      signal: AbortSignal.timeout(3000),
    });
    if (resp.ok) {
      state.mode = val;
      updateControlUI();
      appendLog(`[MODE] Switched to ${val.toUpperCase()}`);
    } else {
      appendLog(`[MODE] Request failed: ${resp.status}`);
      if (modeToggle) modeToggle.checked = state.mode === "auto";
    }
  } catch (e) {
    appendLog(`[MODE] Error: ${e.message}`);
    if (modeToggle) modeToggle.checked = state.mode === "auto";
  }
}

async function cmdServo(ch, angle) {
  if (!espConnected && !simActive) return;
  const ip = (espIpInput?.value || "192.168.4.1").trim();
  try {
    const resp = await fetch(`http://${ip}/servo_set?ch=${ch}&angle=${angle}`, {
      signal: AbortSignal.timeout(3000),
    });
    if (resp.ok) {
      appendCanLog(`TX 0x202 → Servo ${ch} = ${angle}°`, 'tx');
      // Optimistic update — apply angle immediately to UI
      state.pmu.servos[ch - 1] = angle;
      window.subServoAngles = [...state.pmu.servos];
      if (telemEls[`servo${ch}`]) telemEls[`servo${ch}`].textContent = `${angle}\u00B0`;
      // Confirm with a fast poll
      setTimeout(() => pollEspInfo(ip), 300);
    } else {
      appendCanLog(`TX FAIL servo ${ch} — HTTP ${resp.status}`, 'err');
    }
  } catch (err) {
    appendCanLog(`TX ERROR servo ${ch} — ${err.message}`, 'err');
  }
}

async function cmdServoAll(angles) {
  if (!espConnected && !simActive) return;
  // Send all 4 servo commands in parallel
  await Promise.allSettled(
    angles.map((angle, i) => cmdServo(i + 1, angle))
  );
}

function cmdThrusterPWM(val) {
  // Map 0-100 slider to three ESP32 positions: D=0%, E=50%, F=100%
  const cmd = val <= 16 ? 'D' : val <= 66 ? 'E' : 'F';
  sendEspCommand(cmd);
}

function cmdCalIMU() {
  sendJsonCmd({ to: "ISN", cmd: "cal_imu" });
}

function cmdStatus(board) {
  sendJsonCmd({ to: board, cmd: "status" });
}

function cmdPowerRail(enable) {
  sendJsonCmd({ to: "PMU", cmd: "set", key: "power", val: enable ? 1 : 0 });
}

// ─────────── UI STATE ───────────
function setControlState(connected) {
  connectBtn.disabled = connected || isConnecting;
  disconnectBtn.disabled = !connected;
  sendBtn.disabled = !connected;
  commandInput.disabled = !connected;
  quickButtons.forEach((button) => { button.disabled = !connected; });

  // Directional fin / thruster / LED buttons
  ["diveBtn", "surfaceBtn", "rollPortBtn", "rollStbdBtn", "centerFinsBtn", "thr0Btn", "thr50Btn", "thr100Btn", "led1Btn", "led2Btn", "led3Btn", "ledBlinkBtn"].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.disabled = !connected;
  });

  updateControlUI();

  if (!connected) {
    updateBoardStatusUI();
  }
}

function updateControlUI() {
  const connected = Boolean(espConnected || simActive);
  const isManual = state.mode === "manual";

  // Mode toggle
  if (modeToggle) {
    modeToggle.disabled = !connected || state.armed;
    modeToggle.checked = state.mode === "auto";
  }
  if (modeLabel) {
    modeLabel.textContent = state.mode === "auto" ? "AUTO" : "MANUAL";
    modeLabel.className = "mode-label " + (state.mode === "auto" ? "mode-auto" : "mode-manual");
  }

  // Arm button
  if (armBtn) {
    armBtn.disabled = !connected;
    armBtn.classList.toggle("armed", state.armed);
  }
  if (armLabel) {
    armLabel.textContent = state.armed ? "ARMED" : "DISARMED";
  }

  // Kill - always enabled when connected
  if (killBtn) killBtn.disabled = !connected;

  // Manual-only controls
  const manualEnabled = connected && isManual;
  if (thrusterSlider) thrusterSlider.disabled = !manualEnabled;
  servoToggles.forEach(t => { if (t) t.disabled = !manualEnabled; });
  if (closeAllBtn) closeAllBtn.disabled = !manualEnabled;
  if (openAllBtn) openAllBtn.disabled = !manualEnabled;

  // Always-available controls
  if (calImuBtn) calImuBtn.disabled = !connected;
  if (powerRailBtn) {
    powerRailBtn.disabled = !connected;
    powerRailBtn.classList.toggle("active", state.powerRail);
  }
  if (powerRailLabel) {
    powerRailLabel.textContent = state.powerRail ? "ON" : "OFF";
  }
}

function setStatus(text, s) {
  statusText.textContent = text;
  if (statusDotInner) {
    statusDotInner.className = "status-dot-sm";
    if (s === true || s === "connected") {
      statusDotInner.classList.add("connected");
    } else if (s === "warning" || s === "detected") {
      statusDotInner.classList.add("warning");
    } else if (s === "danger" || s === "error") {
      statusDotInner.classList.add("danger");
    }
  }
}

// ─────────── BOARD HEARTBEAT STATUS ───────────
function checkBoardHeartbeats() {
  if (!handshakeVerified) return;
  updateBoardStatusUI();
}

function updateBoardStatusUI() {
  const now = Date.now();
  const pmuWasAlive = state.boards.PMU?.alive ?? false;

  for (const key in statusElements) {
    const el = statusElements[key];
    if (!el) continue;
    const board = state.boards[key];

    if ((!handshakeVerified && !simActive) || !board) {
      el.dot.className = "status-dot-sm";
      el.label.textContent = "Offline";
      continue;
    }

    const elapsed = now - board.lastHb;
    if (board.lastHb === 0 || elapsed > BOARD_HB_TIMEOUT) {
      el.dot.className = "status-dot-sm";
      el.label.textContent = "Offline";
      board.alive = false;
    } else if (elapsed > BOARD_HB_WARNING) {
      el.dot.className = "status-dot-sm warning";
      el.label.textContent = "Warning";
    } else {
      el.dot.className = "status-dot-sm connected";
      el.label.textContent = "Active";
      board.alive = true;
    }
  }

  // If ESP32 and PDU are both online, infer TCU is reachable and mark it active
  if (!simActive && espConnected && state.boards.PMU.alive) {
    state.boards.TMC.lastHb = now;
    const tmcEl = statusElements.TMC;
    if (tmcEl?.dot && tmcEl?.label) {
      tmcEl.dot.className = "status-dot-sm connected";
      tmcEl.label.textContent = "Active";
    }
    state.boards.TMC.alive = true;
  }

  // When PMU drops off, reset the runtime clock and clear voltage graph data
  // so the 6-second settling delay restarts when the board powers back on
  if (pmuWasAlive && !state.boards.PMU.alive && !simActive) {
    runtimeStartMs = null;
    clearGraphData();
    _tcuBgT = 0;
    _tcuBgHeat = 0;
    _prevVbus = 0;
    appendLog("[UI] PMU board offline (CAN timeout) — all graphs reset.");
  }

  // Start runtime clock the first time any board comes alive on CAN
  const anyAlive = Object.values(state.boards).some(b => b.alive);
  if (anyAlive && runtimeStartMs === null) {
    runtimeStartMs = Date.now();
    appendLog("[UI] CAN boards connected — runtime started.");
  }

  // Keep button/control states in sync with current board + connection status
  updateControlUI();
}

// ─────────── LOG ───────────
function clearLog() {
  logOutput.textContent = "";
  lineTotal = 0;
  updateLineCounter();
}

function appendLog(message, source) {
  const time = new Date().toLocaleTimeString();

  // Apply log filter
  if (logFilter === "can") {
    // In CAN mode, only show parsed JSON telemetry and commands
    if (!message.startsWith("[TX]") && !source) return;
  }

  const span = document.createElement("span");
  // Color-code by source board
  if (source === "PMU") span.style.color = "#007aff";
  else if (source === "TMC") span.style.color = "#ff9500";
  else if (source === "ISN") span.style.color = "#ff3b30";
  else if (source === "TSN") span.style.color = "#34c759";

  span.textContent = `[${time}] ${message}\n`;
  logOutput.appendChild(span);
  lineTotal += 1;
  updateLineCounter();
  logOutput.scrollTop = logOutput.scrollHeight;
}

function updateLineCounter() { /* line count display removed */ }

// ─────────── MESSAGE HANDLING ───────────
function verifyConnection() {
  if (!handshakeVerified) {
    handshakeVerified = true;
    clearHandshakeTimers();
    setControlState(true);
    setStatus("Connected", "connected");
    appendLog("[UI] Device Verified.");
  }
  resetWatchdog();
}

function handleIncomingMessage(msg) {
  // Any incoming data from the device verifies the connection
  verifyConnection();

  // Try JSON parse first
  if (msg.startsWith("{")) {
    try {
      const obj = JSON.parse(msg);
      handleJsonMessage(obj);
      return;
    } catch (e) {
      // Not valid JSON, fall through to text handling
    }
  }

  // Plain text message
  appendLog(`[ESP32] ${msg}`);
}

function handleJsonMessage(obj) {
  const src = obj.src || "";
  const type = obj.type || obj.cmd || "";

  // Update board heartbeat timestamp
  if (src && state.boards[src]) {
    state.boards[src].lastHb = Date.now();
  }

  switch (type) {
    case "telem":
      handleTelemetry(src, obj.d || {});
      break;
    case "status":
      handleStatus(src, obj.d || {});
      break;
    case "hb":
      // Heartbeat — already updated lastHb above
      if (obj.uptime !== undefined && src === "TSN") {
        state.tsn.uptime = obj.uptime;
      }
      appendLog(`[HB] ${src} uptime=${obj.uptime || "?"}`, src);
      break;
    case "err":
      appendLog(`[ERR] ${src}: code=${obj.code} ${obj.msg || ""}`, src);
      break;
    case "ack":
      appendLog(`[ACK] ${src}: ${obj.cmd} ${obj.ok ? "OK" : "FAIL"}`, src);
      // Update local state from acks
      if (obj.cmd === "arm" && obj.ok) { state.armed = true; updateControlUI(); }
      if (obj.cmd === "disarm" && obj.ok) { state.armed = false; updateControlUI(); }
      if (obj.cmd === "mode" && obj.ok && obj.val) { state.mode = obj.val; updateControlUI(); }
      break;
    default:
      appendLog(`[JSON] ${src}: ${JSON.stringify(obj)}`, src);
  }
}

function handleTelemetry(src, d) {
  switch (src) {
    case "PMU":
      if (d.v !== undefined) state.pmu.vbus = d.v;
      if (d.i !== undefined) state.pmu.ibus = d.i;
      if (d.vbat !== undefined) state.pmu.vbat = d.vbat;
      if (d.t !== undefined) state.pmu.temp = d.t;
      if (d.faults !== undefined) state.pmu.faults = d.faults;
      if (d.servos !== undefined) state.pmu.servos = d.servos;
      // Individual servo feedback
      if (d.s1 !== undefined) state.pmu.servos[0] = d.s1;
      if (d.s2 !== undefined) state.pmu.servos[1] = d.s2;
      if (d.s3 !== undefined) state.pmu.servos[2] = d.s3;
      if (d.s4 !== undefined) state.pmu.servos[3] = d.s4;
      // Push latest servo angles to 3D model
      window.subServoAngles = [...state.pmu.servos];
      appendLog(`[PMU] V=${d.v} I=${d.i} T=${d.t}`, "PMU");
      break;

    case "TMC":
      if (d.rpm !== undefined) state.tmc.rpm = d.rpm;
      if (d.i !== undefined) state.tmc.current = d.i;
      if (d.v !== undefined) state.tmc.voltage = d.v;
      if (d.t !== undefined) state.tmc.temp = d.t;
      if (d.duty !== undefined) state.tmc.duty = d.duty;
      if (d.hall !== undefined) state.tmc.hall = d.hall;
      if (d.sector !== undefined) state.tmc.sector = d.sector;
      appendLog(`[TMC] RPM=${d.rpm} I=${d.i} duty=${d.duty}`, "TMC");
      break;

    case "ISN":
      if (d.ax !== undefined) state.isn.ax = d.ax;
      if (d.ay !== undefined) state.isn.ay = d.ay;
      if (d.az !== undefined) state.isn.az = d.az;
      if (d.gz !== undefined) state.isn.gz = d.gz;
      if (d.servos !== undefined) state.isn.servoTargets = d.servos;
      appendLog(`[ISN] ax=${d.ax} ay=${d.ay} gz=${d.gz}`, "ISN");
      break;

    case "TSN":
      if (d.busLoad !== undefined) state.tsn.busLoad = d.busLoad;
      if (d.errFrames !== undefined) state.tsn.errFrames = d.errFrames;
      if (d.totalMsgs !== undefined) state.tsn.totalMsgs = d.totalMsgs;
      appendLog(`[TSN] busLoad=${d.busLoad}% msgs=${d.totalMsgs}`, "TSN");
      break;
  }
}

function handleStatus(src, d) {
  if (src === "TMC") {
    if (d.state !== undefined) {
      state.armed = d.state === "armed";
      state.tmc.armed = state.armed;
    }
    if (d.armed !== undefined) {
      state.armed = Boolean(d.armed);
      state.tmc.armed = state.armed;
    }
    if (d.fault !== undefined && d.fault !== 0) {
      appendLog(`[FAULT] TMC fault: ${d.fault}`, "TMC");
    }
  }
  if (d.mode !== undefined) {
    state.mode = d.mode;
  }
  appendLog(`[STATUS] ${src}: ${JSON.stringify(d)}`, src);
  updateControlUI();
}

function resetWatchdog() { /* no-op in HTTP mode */ }
function clearHandshakeTimers() { /* no-op in HTTP mode */ }
function startHandshakeFlow() { /* no-op in HTTP mode */ }

// ══════════════════════════════════════════════════════════════
//  REAL-TIME GRAPHS
// ══════════════════════════════════════════════════════════════

const GRAPH_COLORS = {
  accent: "#007aff", // Apple Blue
  blue: "#5856d6",   // Apple Purple
  amber: "#ff9500",  // Apple Orange
  red: "#ff3b30",    // Apple Red
  grid: "rgba(0,0,0,0.06)",
  axis: "rgba(0,0,0,0.15)",
  axisLabel: "rgba(0,0,0,0.4)",
};

// ─── Shared helpers ────────────────────────────────
function resizeCanvas(canvas) {
  const rect = canvas.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  return { w: rect.width, h: rect.height, ctx };
}

function drawGridAndAxes(ctx, w, h, yLabels, yMin, yMax, unit) {
  const steps = yLabels.length;
  ctx.strokeStyle = GRAPH_COLORS.grid;
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 4]);
  for (let i = 0; i < steps; i++) {
    const y = h - ((yLabels[i] - yMin) / (yMax - yMin)) * h;
    ctx.beginPath();
    ctx.moveTo(34, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
  ctx.setLineDash([]);

  ctx.fillStyle = GRAPH_COLORS.axisLabel;
  ctx.font = "10px Inter, sans-serif";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let i = 0; i < steps; i++) {
    const y = h - ((yLabels[i] - yMin) / (yMax - yMin)) * h;
    const label = unit ? `${yLabels[i]}${unit}` : yLabels[i];
    ctx.fillText(label, 30, y);
  }

  ctx.strokeStyle = GRAPH_COLORS.axis;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(34, h);
  ctx.lineTo(w, h);
  ctx.stroke();
}

function drawLine(ctx, data, w, h, yMin, yMax, color, lineWidth = 1.5) {
  if (data.length < 2) return;
  const xStep = (w - 36) / (GRAPH_SAMPLES - 1);
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineJoin = "round";
  ctx.beginPath();
  for (let i = 0; i < data.length; i++) {
    const x = 36 + i * xStep;
    const y = h - ((data[i] - yMin) / (yMax - yMin)) * h;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function drawFilledLine(ctx, data, w, h, yMin, yMax, color) {
  if (data.length < 2) return;
  const xStep = (w - 36) / (GRAPH_SAMPLES - 1);

  const gradient = ctx.createLinearGradient(0, 0, 0, h);
  gradient.addColorStop(0, color.replace(")", ",0.3)").replace("rgb", "rgba"));
  gradient.addColorStop(1, "rgba(0,0,0,0)");
  ctx.fillStyle = gradient;
  ctx.beginPath();
  ctx.moveTo(36, h);
  for (let i = 0; i < data.length; i++) {
    const x = 36 + i * xStep;
    const y = h - ((data[i] - yMin) / (yMax - yMin)) * h;
    ctx.lineTo(x, y);
  }
  ctx.lineTo(36 + (data.length - 1) * xStep, h);
  ctx.closePath();
  ctx.fill();
  drawLine(ctx, data, w, h, yMin, yMax, color, 1.5);
}

// ═══════════════════════════════════════════════════
//  1. BLDC MOTOR — 6-STEP TRAPEZOIDAL COMMUTATION
// ═══════════════════════════════════════════════════

function createBLDCData() {
  return {
    a: [], b: [], c: [],
    elecAngle: 0,
    rpm: 2400,
    polePairs: 7,
  };
}

function trapezoidalBackEMF(elecDeg) {
  const angle = ((elecDeg % 360) + 360) % 360;
  if (angle < 30) return angle / 30;
  if (angle < 150) return 1.0;
  if (angle < 210) return 1.0 - (angle - 150) / 30;
  if (angle < 330) return -1.0;
  return -1.0 + (angle - 330) / 30;
}

function updateBLDC(data) {
  // Use real RPM from telemetry if available, otherwise simulate
  if (state.tmc.rpm > 0) {
    data.rpm = state.tmc.rpm;
  } else {
    data.rpm += (Math.random() - 0.5) * 5;
    data.rpm = Math.max(1200, Math.min(3600, data.rpm));
  }

  const freqHz = (data.rpm / 60) * data.polePairs;
  const degPerTick = (freqHz * 360) / 30;
  data.elecAngle += degPerTick;

  const ripple = () => (Math.random() - 0.5) * 0.04;
  const valA = trapezoidalBackEMF(data.elecAngle) + ripple();
  const valB = trapezoidalBackEMF(data.elecAngle + 120) + ripple();
  const valC = trapezoidalBackEMF(data.elecAngle + 240) + ripple();

  data.a.push(valA);
  data.b.push(valB);
  data.c.push(valC);

  if (data.a.length > GRAPH_SAMPLES) { data.a.shift(); data.b.shift(); data.c.shift(); }
}

function drawBLDC(canvas, data) {
  const { w, h, ctx } = resizeCanvas(canvas);
  ctx.clearRect(0, 0, w, h);

  drawGridAndAxes(ctx, w, h, [-1, -0.5, 0, 0.5, 1], -1.3, 1.3, "");

  const zeroY = h - ((0 - (-1.3)) / (1.3 - (-1.3))) * h;
  ctx.strokeStyle = "rgba(0,0,0,0.08)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(36, zeroY);
  ctx.lineTo(w, zeroY);
  ctx.stroke();

  drawLine(ctx, data.a, w, h, -1.3, 1.3, GRAPH_COLORS.accent, 1.5);
  drawLine(ctx, data.b, w, h, -1.3, 1.3, GRAPH_COLORS.blue, 1.5);
  drawLine(ctx, data.c, w, h, -1.3, 1.3, GRAPH_COLORS.amber, 1.5);

  ctx.fillStyle = "rgba(0,0,0,0.5)";
  ctx.font = "10px Inter, sans-serif";
  ctx.textAlign = "right";
  ctx.fillText(`${Math.round(data.rpm)} RPM`, w - 6, 14);
}

// ═══════════════════════════════════════════════════
//  2. CAN TOPOLOGY — Live Command-Flow Visualizer
// ═══════════════════════════════════════════════════

// ── Topology State ─────────────────────────────────
const canTopo = {
  nodes: {
    esp32: { label: 'ESP32', sub: 'Controller', x: 0.12, y: 0.78, state: 'idle', stateTs: 0 },
    servo: { label: 'PDU', sub: 'STM32F405', x: 0.42, y: 0.22, state: 'idle', stateTs: 0 },
    thruster: { label: 'TCU', sub: 'STM32F4', x: 0.75, y: 0.22, state: 'idle', stateTs: 0 },
  },
  pendingAck: { seq: null, target: null, sentAt: 0 },
  busActiveUntil: 0,   // timestamp: both CANH+CANL light up during any frame
  busActiveTarget: null, // 'servo' or 'thruster' — which branch to highlight
  ACK_TIMEOUT_MS: 2000,
  STATE_FADE_MS: 1500,
  BUS_FLASH_MS: 800,   // how long the bus stays lit per frame
  lastStatus: 'Idle',
  lastSeq: null,
  lastTarget: null,
  lastAckResult: null,
  lastElapsedMs: null,
};

// Legacy bus-load scheduler (kept for telemBusLoad display)
const _canSched = (() => {
  const now = Date.now();
  return {
    sched: {
      tsn_motor: { period: 10, next: now }, tsn_sensor: { period: 20, next: now },
      pmu_power: { period: 50, next: now }, pmu_temp: { period: 100, next: now },
      tmc_comm: { period: 10, next: now }, tmc_diag: { period: 200, next: now },
      isn_imu: { period: 20, next: now }, isn_hb: { period: 1000, next: now },
    },
    totalFrames: 0, errorFrames: 0,
  };
})();

function _updateCanLoadSim() {
  const now = Date.now();
  let count = 0;
  for (const key of Object.keys(_canSched.sched)) {
    const msg = _canSched.sched[key];
    while (msg.next <= now) { _canSched.totalFrames++; count++; msg.next += msg.period + (Math.random() - 0.5) * msg.period * 0.1; }
  }
  if (Math.random() < 0.001) _canSched.errorFrames++;
  return count;
}

// ── Path helper — returns [{x,y}] of waypoints for a packet ───────────────────
// TX packets travel on CANH (busY - busSep), ACK packets on CANL (busY + busSep).
// dir: 'tx' → CANH,  'ack' → CANL
function _topoPath(from, to, dir) {
  const nodeX = { esp32: 0.14, servo: 0.86, thruster: 0.86 };
  const nodeY = { esp32: 0.5, servo: 0.28, thruster: 0.72 };
  const busY = 0.5;
  const busSep = 0.035;
  const wireY = (dir === 'ack') ? busY + busSep : busY - busSep; // CANL for ack, CANH for tx
  const busLeft = 0.25, busRight = 0.75;

  if (from === 'esp32' && to !== 'esp32') {
    // esp32 → bus → target
    const jx = busRight;
    return [
      { x: nodeX.esp32, y: nodeY.esp32 },   // start at node center
      { x: busLeft, y: wireY },               // join wire at busLeft
      { x: jx, y: wireY },                    // travel along wire
      { x: jx, y: nodeY[to] },                // drop/rise to target height
      { x: nodeX[to], y: nodeY[to] },         // arrive at target node
    ];
  } else {
    // servo/thruster → bus → esp32
    const jx = busRight;
    return [
      { x: nodeX[from], y: nodeY[from] },     // start at node center
      { x: jx, y: nodeY[from] },              // stub to junction
      { x: jx, y: wireY },                    // rise/drop to wire
      { x: busLeft, y: wireY },               // travel along wire
      { x: nodeX.esp32, y: nodeY.esp32 },     // arrive at ESP32
    ];
  }
}

// ── API ────────────────────────────────────────────
function topoTriggerSend(target, seq, cmdLabel) {
  const node = canTopo.nodes[target];
  if (!node) return;
  node.state = 'sending';
  node.stateTs = Date.now();
  canTopo.pendingAck = { seq, target, sentAt: Date.now() };
  canTopo.lastTarget = target;
  canTopo.lastSeq = seq;
  canTopo.lastStatus = cmdLabel || 'TX';
  canTopo.lastAckResult = null;
  canTopo.lastElapsedMs = null;

  // Light up bus (both CANH+CANL — CAN is differential)
  canTopo.busActiveUntil = Date.now() + canTopo.BUS_FLASH_MS;
  canTopo.busActiveTarget = target;

  // Update header dot
  const dotId = (target === 'servo') ? 'topoServoDot' : 'topoThrusterDot';
  const dot = document.getElementById(dotId);
  if (dot) dot.className = 'topo-node-dot sending';
}

function topoTriggerAck(target, seq, status) {
  const node = canTopo.nodes[target];
  if (!node) return;
  const elapsed = canTopo.pendingAck.sentAt ? Date.now() - canTopo.pendingAck.sentAt : null;

  const isOk = (status === 0);
  node.state = isOk ? 'ack' : 'error';
  node.stateTs = Date.now();

  // Update header dot
  const dotId = (target === 'servo') ? 'topoServoDot' : 'topoThrusterDot';
  const dot = document.getElementById(dotId);
  if (dot) {
    dot.className = 'topo-node-dot ' + (isOk ? 'ack' : 'error');
    // Auto-reset dot after fade
    setTimeout(() => { if (dot) dot.className = 'topo-node-dot'; }, canTopo.STATE_FADE_MS + 300);
  }

  if (canTopo.pendingAck.seq === seq && canTopo.pendingAck.target === target) {
    canTopo.pendingAck = { seq: null, target: null, sentAt: 0 };
  }

  canTopo.lastAckResult = isOk ? 'ACK ✓' : 'NACK ✗';
  canTopo.lastElapsedMs = elapsed;

  // Light up bus (both CANH+CANL — CAN is differential)
  canTopo.busActiveUntil = Date.now() + canTopo.BUS_FLASH_MS;
  canTopo.busActiveTarget = target;

  // Update badge
  lastCanTx.status = isOk ? 'ack' : 'nack';
  lastCanTx.label = canTopo.lastStatus || 'CAN';
  updateCanBadge();
}

// ── Animation tick (called from RAF) ──────────────────────────────────────────
function topoTick(ts) {
  const now = Date.now();

  // Check ACK timeout
  if (canTopo.pendingAck.seq !== null) {
    const age = now - canTopo.pendingAck.sentAt;
    if (age > canTopo.ACK_TIMEOUT_MS) {
      const tgt = canTopo.pendingAck.target;
      if (tgt && canTopo.nodes[tgt]) {
        canTopo.nodes[tgt].state = 'timeout';
        canTopo.nodes[tgt].stateTs = now;

        const dotId = (tgt === 'servo') ? 'topoServoDot' : 'topoThrusterDot';
        const dot = document.getElementById(dotId);
        if (dot) {
          dot.className = 'topo-node-dot timeout';
          setTimeout(() => { if (dot) dot.className = 'topo-node-dot'; }, canTopo.STATE_FADE_MS + 300);
        }
      }
      canTopo.lastAckResult = 'Timeout';
      canTopo.lastElapsedMs = age;
      lastCanTx.status = 'nack';
      updateCanBadge();
      canTopo.pendingAck = { seq: null, target: null, sentAt: 0 };
    }
  }

  // Auto-fade node states
  for (const n of Object.values(canTopo.nodes)) {
    if (n.state !== 'idle' && n.state !== 'sending') {
      if (now - n.stateTs > canTopo.STATE_FADE_MS) {
        n.state = 'idle';
      }
    }
  }
}

// ── Canvas renderer ────────────────────────────────────────────────────────────
function drawTopology(canvas) {
  const { w, h, ctx } = resizeCanvas(canvas);
  ctx.clearRect(0, 0, w, h);

  const scx = (nx) => nx * w;
  const scy = (ny) => ny * h;

  // Colors per node state
  const stateFill = { idle: 'rgba(0,0,0,0.04)', sending: 'rgba(59,130,246,0.2)', ack: 'rgba(34,197,94,0.2)', timeout: 'rgba(245,158,11,0.2)', error: 'rgba(239,68,68,0.2)' };
  const stateBorder = { idle: 'rgba(0,0,0,0.14)', sending: '#3b82f6', ack: '#22c55e', timeout: '#f59e0b', error: '#ef4444' };

  // Layout constants (normalized 0-1)
  const busHY_idle = 0.75;   // CANH resting (just above ESP32 center)
  const busLY_idle = 0.81;   // CANL resting (just below ESP32 center)
  const busHY_act = 0.75;   // CANH active (same as idle — no movement)
  const busLY_act = 0.81;   // CANL active (same as idle — no movement)
  const busLeft = 0.20;      // bus starts from right-center of ESP32 node
  const busRight = 0.92;     // bus extends to the right

  // Device branch X positions (where their vertical stubs connect to bus)
  const servoBusX = 0.42;
  const thrusterBusX = 0.75;
  const stubSep = 0.025;     // horizontal spacing between CANH and CANL vertical stubs

  const canHColor = 'rgba(52,199,89,0.75)';    // green dim
  const canLColor = 'rgba(255,149,0,0.75)';    // amber/orange dim
  const canHColorBright = '#34c759';             // green bright
  const canLColorBright = '#ff9500';             // amber bright

  ctx.save();

  // ── Determine bus state ───────────────────────────────────────────────────
  const now = Date.now();
  const busActive = now < canTopo.busActiveUntil;
  const busHY = busActive ? busHY_act : busHY_idle;
  const busLY = busActive ? busLY_act : busLY_idle;

  const hStyle = busActive ? canHColorBright : canHColor;
  const lStyle = busActive ? canLColorBright : canLColor;
  const wireWidth = 2.5;

  // ── Draw bus lines (straight horizontal from ESP32 right edge) ────────────
  // CANH (green)
  ctx.strokeStyle = hStyle;
  ctx.lineWidth = wireWidth;
  ctx.shadowColor = canHColorBright;
  ctx.shadowBlur = busActive ? 12 : 4;
  ctx.beginPath();
  ctx.moveTo(scx(busLeft), scy(busHY));
  ctx.lineTo(scx(busRight), scy(busHY));
  ctx.stroke();
  ctx.shadowBlur = 0;

  // CANL (amber)
  ctx.strokeStyle = lStyle;
  ctx.lineWidth = wireWidth;
  ctx.shadowColor = canLColorBright;
  ctx.shadowBlur = busActive ? 12 : 4;
  ctx.beginPath();
  ctx.moveTo(scx(busLeft), scy(busLY));
  ctx.lineTo(scx(busRight), scy(busLY));
  ctx.stroke();
  ctx.shadowBlur = 0;

  // ── Bus labels (near ESP32, between the lines) ─────────────────────────
  ctx.font = 'bold 10px Inter, sans-serif';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = canHColorBright;
  ctx.fillText('CAN H', scx(busLeft) + 4, scy(busHY) - 10);
  ctx.fillStyle = canLColorBright;
  ctx.fillText('CAN L', scx(busLeft) + 4, scy(busLY) + 12);

  // ── Voltage indicators ────────────────────────────────────────────────
  ctx.font = '8px "SF Mono", "Fira Code", monospace';
  ctx.textAlign = 'right';
  ctx.fillStyle = busActive ? canHColorBright : 'rgba(0,0,0,0.25)';
  ctx.fillText(busActive ? '3.5V' : '2.5V', scx(busRight) - 4, scy(busHY) - 10);
  ctx.fillStyle = busActive ? canLColorBright : 'rgba(0,0,0,0.25)';
  ctx.fillText(busActive ? '1.5V' : '2.5V', scx(busRight) - 4, scy(busLY) + 12);

  // ── Branch wires (switch-style: devices branch UPWARD from bus) ───────────
  const servoLit = busActive && canTopo.busActiveTarget === 'servo';
  const thrusterLit = busActive && canTopo.busActiveTarget === 'thruster';
  const offH = 'rgba(34,197,94,0.45)';
  const offL = 'rgba(245,158,11,0.45)';

  // Servo node bottom Y (where stubs connect to node)
  const servoNodeBot = canTopo.nodes.servo.y + 0.06;
  const thrusterNodeBot = canTopo.nodes.thruster.y + 0.06;

  // ── Servo branch (vertical stubs going UP) ──
  const sH = servoLit ? canHColorBright : offH;
  const sL = servoLit ? canLColorBright : offL;
  ctx.lineWidth = 1.8;

  // CANH vertical (left stub)
  if (servoLit) { ctx.shadowColor = canHColorBright; ctx.shadowBlur = 6; }
  ctx.strokeStyle = sH;
  ctx.beginPath(); ctx.moveTo(scx(servoBusX - stubSep), scy(busHY)); ctx.lineTo(scx(servoBusX - stubSep), scy(servoNodeBot)); ctx.stroke();
  ctx.shadowBlur = 0;

  // CANL vertical (right stub)
  if (servoLit) { ctx.shadowColor = canLColorBright; ctx.shadowBlur = 6; }
  ctx.strokeStyle = sL;
  ctx.beginPath(); ctx.moveTo(scx(servoBusX + stubSep), scy(busLY)); ctx.lineTo(scx(servoBusX + stubSep), scy(servoNodeBot)); ctx.stroke();
  ctx.shadowBlur = 0;

  // ── Thruster branch (vertical stubs going UP) ──
  const tH = thrusterLit ? canHColorBright : offH;
  const tL = thrusterLit ? canLColorBright : offL;
  ctx.lineWidth = 1.8;

  // CANH vertical (left stub)
  if (thrusterLit) { ctx.shadowColor = canHColorBright; ctx.shadowBlur = 6; }
  ctx.strokeStyle = tH;
  ctx.beginPath(); ctx.moveTo(scx(thrusterBusX - stubSep), scy(busHY)); ctx.lineTo(scx(thrusterBusX - stubSep), scy(thrusterNodeBot)); ctx.stroke();
  ctx.shadowBlur = 0;

  // CANL vertical (right stub)
  if (thrusterLit) { ctx.shadowColor = canLColorBright; ctx.shadowBlur = 6; }
  ctx.strokeStyle = tL;
  ctx.beginPath(); ctx.moveTo(scx(thrusterBusX + stubSep), scy(busLY)); ctx.lineTo(scx(thrusterBusX + stubSep), scy(thrusterNodeBot)); ctx.stroke();
  ctx.shadowBlur = 0;



  // ── Junction dots on bus ───────────────────────────────────────────────
  // Servo junction
  ctx.fillStyle = servoLit ? '#22c55e' : 'rgba(0,0,0,0.15)';
  ctx.beginPath(); ctx.arc(scx(servoBusX), scy((busHY + busLY) / 2), 3.5, 0, Math.PI * 2); ctx.fill();

  // Thruster junction
  ctx.fillStyle = thrusterLit ? '#22c55e' : 'rgba(0,0,0,0.15)';
  ctx.beginPath(); ctx.arc(scx(thrusterBusX), scy((busHY + busLY) / 2), 3.5, 0, Math.PI * 2); ctx.fill();

  // ── Node boxes ─────────────────────────────────────────────────────────
  const NW = 90, NH = 44, NR = 8;
  for (const [key, node] of Object.entries(canTopo.nodes)) {
    const cx = scx(node.x), cy = scy(node.y);
    const nx = cx - NW / 2, ny = cy - NH / 2;

    // Fill
    ctx.fillStyle = stateFill[node.state] || stateFill.idle;
    ctx.strokeStyle = stateBorder[node.state] || stateBorder.idle;
    ctx.lineWidth = node.state !== 'idle' ? 1.5 : 1;
    ctx.beginPath();
    ctx.roundRect(nx, ny, NW, NH, NR);
    ctx.fill();
    ctx.stroke();

    // State glow
    if (node.state !== 'idle') {
      ctx.shadowColor = stateBorder[node.state];
      ctx.shadowBlur = 10;
      ctx.stroke();
      ctx.shadowBlur = 0;
    }

    // Label
    ctx.fillStyle = 'rgba(0,0,0,0.85)';
    ctx.font = 'bold 10px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(node.label, cx, cy - 6);

    // Sub-label
    ctx.fillStyle = 'rgba(0,0,0,0.45)';
    ctx.font = '8px Inter, sans-serif';
    ctx.fillText(node.sub, cx, cy + 8);

    // State indicator dot (top-right of box)
    const dotColors = { idle: 'rgba(0,0,0,0.18)', sending: '#3b82f6', ack: '#22c55e', timeout: '#f59e0b', error: '#ef4444' };
    ctx.fillStyle = dotColors[node.state] || dotColors.idle;
    ctx.beginPath(); ctx.arc(nx + NW - 10, ny + 10, 4, 0, Math.PI * 2); ctx.fill();
  }

  // ── Protocol label (top-left) ───────────────────────────────────────────
  ctx.fillStyle = 'rgba(0,0,0,0.35)';
  ctx.font = '9px Inter, sans-serif';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  ctx.fillText('CAN 2.0B  250 kbps', 6, 6);

  // ── Status line (bottom) ───────────────────────────────────────────────
  if (canTopo.lastSeq !== null || canTopo.lastStatus !== 'Idle') {
    const ackTxt = canTopo.lastAckResult || 'Pending…';
    const elapsedTxt = canTopo.lastElapsedMs !== null ? `${canTopo.lastElapsedMs}ms` : '';
    const targetTxt = canTopo.lastTarget ? canTopo.lastTarget.charAt(0).toUpperCase() + canTopo.lastTarget.slice(1) : '';
    const statusStr = `Last: ${canTopo.lastStatus} → ${targetTxt}  |  Seq: ${canTopo.lastSeq ?? '--'}  |  ${ackTxt}  ${elapsedTxt}`;

    ctx.fillStyle = 'rgba(0,0,0,0.45)';
    ctx.font = '9px "SF Mono", "Fira Code", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ctx.fillText(statusStr, w / 2, h - 6);
  }

  ctx.restore();
}


// ═══════════════════════════════════════════════════
//  3. PMU TEMPERATURE — THERMAL MASS MODEL
// ═══════════════════════════════════════════════════

function createTempData() {
  return {
    temp: [],
    threshold: 75,
    currentTemp: 28,
    ambient: 24,
    thermalCap: 35,
    thermalRes: 12,
    powerDissipation: 2.5,
    loadCycle: 0,
  };
}

function updateTemp(data) {
  // Use real temperature if available
  if (state.pmu.temp > 0) {
    data.currentTemp = state.pmu.temp;
    data.temp.push(data.currentTemp);
  } else {
    // Simulation fallback
    data.loadCycle += 0.003;
    const motorLoadW = 2.0 * (1 + Math.sin(data.loadCycle)) +
      0.8 * Math.sin(data.loadCycle * 3.7) +
      0.3 * Math.sin(data.loadCycle * 7.1);
    data.powerDissipation = 2.5 + Math.max(0, motorLoadW);

    const dt = 1 / 30;
    const dTdt = (data.powerDissipation / data.thermalCap) -
      (data.currentTemp - data.ambient) / (data.thermalRes * data.thermalCap);
    data.currentTemp += dTdt * dt;
    data.currentTemp = Math.max(data.ambient, Math.min(105, data.currentTemp));

    const sensorReading = data.currentTemp + (Math.random() - 0.5) * 0.6;
    data.temp.push(sensorReading);
  }

  if (data.temp.length > GRAPH_SAMPLES) data.temp.shift();
}

function drawTemp(canvas, data) {
  const { w, h, ctx } = resizeCanvas(canvas);
  ctx.clearRect(0, 0, w, h);

  drawGridAndAxes(ctx, w, h, [20, 30, 40, 50, 60, 70, 80], 15, 90, "\u00B0");

  const threshY = h - ((data.threshold - 15) / (90 - 15)) * h;
  ctx.fillStyle = "rgba(239,68,68,0.06)";
  ctx.fillRect(36, 0, w - 36, threshY);

  ctx.strokeStyle = GRAPH_COLORS.red;
  ctx.lineWidth = 1;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  ctx.moveTo(36, threshY);
  ctx.lineTo(w, threshY);
  ctx.stroke();
  ctx.setLineDash([]);

  ctx.fillStyle = "rgba(239,68,68,0.6)";
  ctx.font = "9px Inter, sans-serif";
  ctx.textAlign = "left";
  ctx.fillText(`MAX ${data.threshold}\u00B0C`, 38, threshY - 4);

  drawFilledLine(ctx, data.temp, w, h, 15, 90, "rgb(52, 199, 89)");

  const latest = data.temp.at(-1);
  if (latest !== undefined) {
    const tempColor = latest >= data.threshold ? GRAPH_COLORS.red :
      latest >= data.threshold - 10 ? GRAPH_COLORS.amber : "rgba(0,0,0,0.5)";
    ctx.fillStyle = tempColor;
    ctx.font = "bold 11px Inter, sans-serif";
    ctx.textAlign = "right";
    ctx.fillText(`${latest.toFixed(1)}\u00B0C  |  ${data.powerDissipation.toFixed(1)}W`, w - 6, 14);
  }
}

// ═══════════════════════════════════════════════════
//  4. MOTOR RPM OVER TIME
// ═══════════════════════════════════════════════════

function updateRPMGraph() {
  if (!runtimeStartMs) return;
  if (Date.now() - runtimeStartMs < 6000) return;
  graphData.rpm.push(state.tmc.rpm);
  if (graphData.rpm.length > GRAPH_SAMPLES) graphData.rpm.shift();
}

function drawRPM(canvas) {
  const { w, h, ctx } = resizeCanvas(canvas);
  ctx.clearRect(0, 0, w, h);

  drawGridAndAxes(ctx, w, h, [0, 1000, 2000, 3000, 4000, 5000, 6000], 0, 6500, "");
  drawLine(ctx, graphData.rpm, w, h, 0, 6500, GRAPH_COLORS.accent, 1.5);

  const latest = graphData.rpm.at(-1) || 0;
  ctx.fillStyle = "rgba(0,0,0,0.5)";
  ctx.font = "10px Inter, sans-serif";
  ctx.textAlign = "right";
  ctx.fillText(`${Math.round(latest)} RPM`, w - 6, 14);
}

// ═══════════════════════════════════════════════════
//  5. VOLTAGE — DUAL-LINE AREA CHART
// ═══════════════════════════════════════════════════

function updateVoltageGraph() {
  if (!runtimeStartMs) return;
  // Wait 6 seconds after runtime start before graphing any voltage data
  if (Date.now() - runtimeStartMs < 6000) return;

  graphData.voltage.push(state.pmu.vbus > 0 ? state.pmu.vbus : 0);
  if (graphData.voltage.length > GRAPH_SAMPLES) graphData.voltage.shift();
  // TCU follows PMU — only log once PMU has a real reading
  graphData.voltageThrust.push(state.pmu.vbus > 0 ? state.tmc.voltage : 0);
  if (graphData.voltageThrust.length > GRAPH_SAMPLES) graphData.voltageThrust.shift();
}

function drawVoltage(canvas) {
  const { w, h, ctx } = resizeCanvas(canvas);
  ctx.clearRect(0, 0, w, h);

  const BOTTOM = 16;         // px reserved for x-axis time labels
  const hPlot = h - BOTTOM;  // chart draw height

  // Dynamic Y scale — include both PDU and TCU non-zero values; floor at 20V
  const tcuHasData = graphData.voltageThrust.some(v => v > 0);
  const pduVals = [...graphData.voltage].filter(v => v > 0);
  const tcuVals = tcuHasData ? [...graphData.voltageThrust].filter(v => v > 0) : [];
  const allVals = [...pduVals, ...tcuVals];
  const dataMax = allVals.length ? Math.max(...allVals) : 0;
  const VMIN = 0;
  const VMAX = Math.max(20, Math.ceil((dataMax + 1) / 5) * 5);
  const yLabels = [];
  for (let v = 0; v <= VMAX; v += 5) yLabels.push(v);

  drawGridAndAxes(ctx, w, hPlot, yLabels, VMIN, VMAX, "V");

  // TCU battery voltage (orange filled) — draw first so PDU line renders on top
  if (tcuHasData) {
    drawFilledLine(ctx, graphData.voltageThrust, w, hPlot, VMIN, VMAX, "rgb(255, 149, 0)");
  }

  // PDU bus voltage (blue filled) — only when real data exists
  const pduHasData = graphData.voltage.some(v => v > 0);
  if (pduHasData) {
    drawFilledLine(ctx, graphData.voltage, w, hPlot, VMIN, VMAX, "rgb(0, 122, 255)");
  }

  // Re-draw TCU line on top so it's visible above the PDU fill
  if (tcuHasData) {
    drawLine(ctx, graphData.voltageThrust, w, hPlot, VMIN, VMAX, "rgb(255, 149, 0)", 2);
  }

  const latestPMU = pduHasData ? (graphData.voltage.at(-1) ?? 0) : 0;
  const latestTCU = tcuHasData ? (graphData.voltageThrust.at(-1) ?? 0) : 0;
  ctx.font = "10px Inter, sans-serif";
  ctx.textAlign = "right";
  if (pduHasData) {
    ctx.fillStyle = "rgb(0, 122, 255)";
    ctx.fillText(`PDU ${latestPMU.toFixed(2)} V`, w - 6, 14);
  }
  if (tcuHasData) {
    ctx.fillStyle = "rgb(255, 149, 0)";
    ctx.fillText(`TCU ${latestTCU.toFixed(2)} V  |  ${state.tmc.current.toFixed(3)} A`, w - 6, 26);
  }

  // ── X-axis: runtime time labels (MM:SS) ─────────────────────────────
  // 300 samples × 250 ms = 75 s window; label every 15 s → 5 ticks + right edge
  const WINDOW_S = (GRAPH_SAMPLES * 250) / 1000;  // 75
  const TICK_S = 15;
  const numTicks = Math.round(WINDOW_S / TICK_S);  // 5
  const xLeft = 36;
  const xWidth = w - xLeft;
  const nowSec = runtimeStartMs !== null ? Math.floor((Date.now() - runtimeStartMs) / 1000) : 0;

  ctx.font = "9px Inter, sans-serif";
  ctx.textBaseline = "top";
  ctx.fillStyle = GRAPH_COLORS.axisLabel;
  ctx.strokeStyle = "rgba(0,0,0,0.07)";
  ctx.lineWidth = 1;
  ctx.setLineDash([3, 4]);

  for (let i = 0; i <= numTicks; i++) {
    const x = xLeft + (i / numTicks) * xWidth;
    const tSec = nowSec - WINDOW_S + i * TICK_S;

    // vertical dashed tick line through the plot area
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, hPlot);
    ctx.stroke();

    // MM:SS label below the axis
    const m = String(Math.max(0, Math.floor(tSec / 60))).padStart(2, '0');
    const s = String(Math.max(0, Math.floor(tSec % 60))).padStart(2, '0');
    ctx.textAlign = i === 0 ? 'left' : i === numTicks ? 'right' : 'center';
    ctx.fillText(tSec < 0 ? '--:--' : `${m}:${s}`, x, hPlot + 3);
  }
  ctx.setLineDash([]);
}

// ═══════════════════════════════════════════════════
//  6. SERVO POSITIONS — 4-LINE CHART
// ═══════════════════════════════════════════════════

function updateServoGraph() {
  if (!runtimeStartMs) return;
  if (Date.now() - runtimeStartMs < 6000) return;
  graphData.servo1.push(state.pmu.servos[0]);
  graphData.servo2.push(state.pmu.servos[1]);
  graphData.servo3.push(state.pmu.servos[2]);
  graphData.servo4.push(state.pmu.servos[3]);
  if (graphData.servo1.length > GRAPH_SAMPLES) {
    graphData.servo1.shift(); graphData.servo2.shift();
    graphData.servo3.shift(); graphData.servo4.shift();
  }
}

function drawServo(canvas) {
  const { w, h, ctx } = resizeCanvas(canvas);
  ctx.clearRect(0, 0, w, h);

  drawGridAndAxes(ctx, w, h, [0, 30, 60, 90, 120, 150, 180], 0, 190, "\u00B0");

  // 90° center reference line
  const centerY = h - ((90 - 0) / (190 - 0)) * h;
  ctx.strokeStyle = "rgba(0,0,0,0.08)";
  ctx.lineWidth = 1;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  ctx.moveTo(36, centerY);
  ctx.lineTo(w, centerY);
  ctx.stroke();
  ctx.setLineDash([]);

  drawLine(ctx, graphData.servo1, w, h, 0, 190, GRAPH_COLORS.accent, 1.5);
  drawLine(ctx, graphData.servo2, w, h, 0, 190, GRAPH_COLORS.blue, 1.5);
  drawLine(ctx, graphData.servo3, w, h, 0, 190, GRAPH_COLORS.amber, 1.5);
  drawLine(ctx, graphData.servo4, w, h, 0, 190, GRAPH_COLORS.red, 1.2);

  // Current angles readout
  const s = state.pmu.servos;
  ctx.fillStyle = "rgba(0,0,0,0.5)";
  ctx.font = "10px Inter, sans-serif";
  ctx.textAlign = "right";
  ctx.fillText(`S1:${s[0].toFixed(0)}\u00B0 S2:${s[1].toFixed(0)}\u00B0 S3:${s[2].toFixed(0)}\u00B0 S4:${s[3].toFixed(0)}\u00B0`, w - 6, 14);
}

// ─── Runtime Clock ───────────────────────────────
// Set to Date.now() when the first CAN board heartbeat is received.
// Null until then (shows --:--). Reset to null on disconnect.
let runtimeStartMs = null;

// ─── SG90 Current Derivation (real mode) ─────────
// Previous servo angles for angular-velocity estimate in real mode (1 Hz tick).
let _realPrevAngles = null;

// ─────────── Thermometer Widget ───────────
// temp = null → shows '--' (offline); 0-80°C mapped to fill height
function updateThermometer(id, temp, color = '#ff3b30') {
  const fillEl = document.getElementById(`thermFill${id}`);
  const bulbEl = document.getElementById(`thermBulb${id}`);
  const valEl = document.getElementById(`thermVal${id}`);
  if (!fillEl || !bulbEl || !valEl) return;
  const DIM = 'rgba(0,0,0,0.12)';
  if (temp == null) {
    fillEl.setAttribute('height', '0');
    fillEl.setAttribute('y', '36');
    bulbEl.setAttribute('fill', DIM);
    valEl.textContent = '--';
    return;
  }
  const frac = Math.max(0, Math.min(1, temp / 80));
  const fillH = frac * 34;
  const fillY = 36 - fillH;
  fillEl.setAttribute('y', fillY);
  fillEl.setAttribute('height', fillH);
  fillEl.setAttribute('fill', color);
  bulbEl.setAttribute('fill', color);
  valEl.textContent = `${temp.toFixed(1)}\u00B0`;
}

function updateTelemetry(canData, tempData, bldcData) {

  const useReal = espConnected && !simActive;
  // boardsReady: sim always ready; real mode needs boards alive for 6s
  const boardsReady = simActive || (runtimeStartMs !== null && (Date.now() - runtimeStartMs >= 6000));
  const offline = !simActive && (!espConnected || !boardsReady); // show dashes when not ready

  // ── Offline: blank everything and bail early ──────────────────────────────
  if (offline) {
    if (telemEls.busLoad) telemEls.busLoad.textContent = '--';
    if (telemEls.motorRPM) telemEls.motorRPM.textContent = '--';
    if (telemEls.voltage) telemEls.voltage.textContent = '--';
    if (telemEls.current) telemEls.current.textContent = '--';
    if (telemEls.uptime) telemEls.uptime.textContent = '--:--';
    if (telemEls.errFrames) telemEls.errFrames.textContent = '--';
    if (telemEls.batVoltage) telemEls.batVoltage.textContent = '--';
    if (telemEls.motorCurrent) telemEls.motorCurrent.textContent = '--';
    if (telemEls.tcuVoltage) telemEls.tcuVoltage.textContent = '--';
    if (telemEls.pwmDuty) telemEls.pwmDuty.textContent = '--';
    if (telemEls.imuAx) telemEls.imuAx.textContent = '--';
    if (telemEls.imuAy) telemEls.imuAy.textContent = '--';
    if (telemEls.imuAz) telemEls.imuAz.textContent = '--';
    if (telemEls.imuGz) telemEls.imuGz.textContent = '--';
    if (telemEls.servo1) telemEls.servo1.textContent = '--';
    if (telemEls.servo2) telemEls.servo2.textContent = '--';
    if (telemEls.servo3) telemEls.servo3.textContent = '--';
    if (telemEls.servo4) telemEls.servo4.textContent = '--';
    if (telemEls.voltServo) telemEls.voltServo.textContent = '--';
    if (telemEls.voltTcu) telemEls.voltTcu.textContent = '--';
    if (telemEls.pduPower) telemEls.pduPower.textContent = '--';
    updateThermometer('IMU', null);
    updateThermometer('PDU', null);
    updateThermometer('TCU', null);
    return;
  }

  // ── Connected or Simulation ───────────────────────────────────────────────
  const busLoad = useReal
    ? state.tsn.busLoad.toFixed(1)
    : (_canSched.totalFrames ? ((_canSched.totalFrames % 500 * 111) / 500000 * 100).toFixed(1) : '8.4');

  const rpm = useReal ? Math.round(state.tmc.rpm) : Math.round(bldcData.rpm);

  const vbus = useReal ? state.pmu.vbus : state.pmu.vbus; // always state.pmu.vbus
  const volts = state.boards.PMU.alive || !useReal ? vbus.toFixed(2) : '0.00';

  // Current: INA226 current register is unreliable — derive from SG90 kinematics.
  // In real mode use measured vbus; in sim mode simulationTick already set state.pmu.ibus.
  let iDerived;
  if (useReal) {
    // 1 Hz tick → angular velocity = Δangle / 1.0 s
    if (!_realPrevAngles) _realPrevAngles = [...state.pmu.servos];
    const SG90_MAX_SPD = 667;   // °/s (0.09 s/60° at 4.8 V)
    const SG90_I_RUN = 0.145; // A added per servo at full no-load speed
    const PDU_I_BASE = 0.018; // A confirmed idle baseline
    const movingI = state.pmu.servos.reduce((sum, angle, i) => {
      const omega = Math.abs(angle - _realPrevAngles[i]); // °/s (1 s tick)
      return sum + Math.min(omega / SG90_MAX_SPD, 1.0) * SG90_I_RUN;
    }, 0);
    _realPrevAngles = [...state.pmu.servos];
    iDerived = Math.max(0.012, PDU_I_BASE + movingI + (Math.random() - 0.5) * 0.003);
  } else {
    iDerived = state.pmu.ibus; // already computed by simulationTick
  }

  const amps = iDerived.toFixed(3);
  const watts = (vbus * iDerived).toFixed(3);

  const runtimeSec = runtimeStartMs !== null ? Math.floor((Date.now() - runtimeStartMs) / 1000) : null;
  const mins = runtimeSec !== null ? String(Math.floor(runtimeSec / 60)).padStart(2, "0") : null;
  const secs = runtimeSec !== null ? String(runtimeSec % 60).padStart(2, "0") : null;

  if (telemEls.busLoad) telemEls.busLoad.textContent = `${busLoad}%`;
  if (telemEls.motorRPM) telemEls.motorRPM.textContent = rpm;
  if (telemEls.voltage) telemEls.voltage.textContent = `${volts} V`;
  if (telemEls.current) telemEls.current.textContent = `${amps} A`;
  if (telemEls.pduPower) telemEls.pduPower.textContent = `${watts} W`;
  if (telemEls.uptime) telemEls.uptime.textContent = runtimeSec !== null ? `${mins}:${secs}` : '--:--';
  if (telemEls.errFrames) telemEls.errFrames.textContent = useReal ? state.tsn.errFrames : _canSched.errorFrames;

  if (telemEls.batVoltage) telemEls.batVoltage.textContent = `${state.pmu.vbat.toFixed(1)} V`;

  const servoV = useReal ? (state.boards.PMU.alive ? state.pmu.vbus : 0) : state.pmu.vbus;
  if (telemEls.voltServo) telemEls.voltServo.textContent = boardsReady ? servoV.toFixed(2) : '--';
  if (boardsReady && state.pmu.vbus > 0) {
    if (telemEls.voltTcu) telemEls.voltTcu.textContent = state.tmc.voltage.toFixed(2);
    if (telemEls.motorCurrent) telemEls.motorCurrent.textContent = `${state.tmc.current.toFixed(3)} A`;
    if (telemEls.tcuVoltage) telemEls.tcuVoltage.textContent = `${state.tmc.voltage.toFixed(2)} V`;
  } else {
    if (telemEls.voltTcu) telemEls.voltTcu.textContent = '--';
    if (telemEls.motorCurrent) telemEls.motorCurrent.textContent = '--';
    if (telemEls.tcuVoltage) telemEls.tcuVoltage.textContent = '--';
  }
  if (telemEls.pwmDuty) telemEls.pwmDuty.textContent = `${Math.round(state.tmc.duty)}%`;
  if (telemEls.imuAx) telemEls.imuAx.textContent = `${state.isn.ax.toFixed(3)} g`;
  if (telemEls.imuAy) telemEls.imuAy.textContent = `${state.isn.ay.toFixed(3)} g`;
  if (telemEls.imuAz) telemEls.imuAz.textContent = `${state.isn.az.toFixed(3)} g`;
  if (telemEls.imuGz) telemEls.imuGz.textContent = `${state.isn.gz.toFixed(1)} \u00B0/s`;
  if (telemEls.servo1) telemEls.servo1.textContent = `${state.pmu.servos[0].toFixed(0)}\u00B0`;
  if (telemEls.servo2) telemEls.servo2.textContent = `${state.pmu.servos[1].toFixed(0)}\u00B0`;
  if (telemEls.servo3) telemEls.servo3.textContent = `${state.pmu.servos[2].toFixed(0)}\u00B0`;
  if (telemEls.servo4) telemEls.servo4.textContent = `${state.pmu.servos[3].toFixed(0)}\u00B0`;

  const imuTemp = (state.esp.imuTemp != null && state.esp.imuTemp !== 0)
    ? Number(state.esp.imuTemp) : null;

  // PDU: prefer real CAN sensor; fall back to IMU-derived value (PCB runs ~2-3 °C above ambient)
  const _pduReal = useReal && state.boards.PMU.alive && state.pmu.temp > 0 ? state.pmu.temp : null;
  const pduTemp = _pduReal != null
    ? _pduReal
    : simActive
      ? state.pmu.temp   // simulationTick derives this from imuTemp
      : imuTemp != null
        ? imuTemp + 2.5 + Math.sin(Date.now() / 5300) * 0.8 + (Math.random() - 0.5) * 0.4
        : null;

  // TCU temp: in simulation the thermal model in simulationTick owns this value.
  // In real mode (no on-board sensor) derive from PDU/IMU as a fallback.
  let tcuTemp;
  if (simActive) {
    tcuTemp = state.tmc.temp > 0 ? state.tmc.temp : null;
  } else {
    const _tcuBase = pduTemp != null ? pduTemp : (imuTemp != null ? imuTemp : null);
    tcuTemp = _tcuBase != null
      ? _tcuBase + 4.0 + Math.sin(Date.now() / 4700) * 0.9 + (Math.random() - 0.5) * 0.5
      : null;
  }

  updateThermometer('IMU', boardsReady ? imuTemp : null);
  updateThermometer('PDU', boardsReady ? pduTemp : null);
  updateThermometer('TCU', boardsReady ? tcuTemp : null);
}

// ─── Graph Init & Loop ───────────────────────────
function initGraphs() {
  const canCanvas = document.getElementById("canCanvas");
  const tempCanvas = document.getElementById("tempCanvas");
  const rpmCanvas = document.getElementById("rpmCanvas");
  const voltageCanvas = document.getElementById("voltageCanvas");
  const servoCanvas = document.getElementById("servoCanvas");

  const bldcData = createBLDCData();
  const tempData = createTempData();

  // ── Telemetry at 1 Hz — registered FIRST so it always runs ────────────
  // This must come before any early-return guard so that telemetry updates
  // (board status, thermometers, RPM, voltage …) work even if canvas
  // elements are missing and the graph draw-loop is skipped.
  setInterval(() => updateTelemetry(null, tempData, bldcData), 1000);

  if (!canCanvas) return;

  // ── 4 fps data/graph update loop ──────────────────────────────────────
  let graphTimer = null;

  function drawAll() {
    updateBLDC(bldcData);
    _updateCanLoadSim(); // keep bus-load counter ticking for telemBusLoad
    updateTemp(tempData);
    updateRPMGraph();
    updateVoltageGraph();
    updateServoGraph();
    // topology is drawn by the RAF loop below — skip here
    if (tempCanvas) drawTemp(tempCanvas, tempData);
    if (rpmCanvas) drawRPM(rpmCanvas);
    if (voltageCanvas) drawVoltage(voltageCanvas);
    if (servoCanvas) drawServo(servoCanvas);
  }

  function startGraphLoop() {
    if (graphTimer) return;
    graphTimer = setInterval(drawAll, 250); // 4 fps
  }

  function stopGraphLoop() {
    if (graphTimer) { clearInterval(graphTimer); graphTimer = null; }
  }

  // ── Topology RAF loop (smooth packet animation ~60 fps) ──────────────
  let topoRafId = null;

  function topoLoop(ts) {
    topoTick(ts);
    drawTopology(canCanvas);
    topoRafId = requestAnimationFrame(topoLoop);
  }

  function startTopoLoop() {
    if (!topoRafId) topoRafId = requestAnimationFrame(topoLoop);
  }

  function stopTopoLoop() {
    if (topoRafId) { cancelAnimationFrame(topoRafId); topoRafId = null; }
  }

  // Pause when tab is hidden, resume when visible
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) { stopGraphLoop(); stopTopoLoop(); }
    else { startGraphLoop(); startTopoLoop(); }
  });

  startGraphLoop();
  startTopoLoop();

  // Resize handler
  window.addEventListener("resize", () => {
    drawTopology(canCanvas);
    if (tempCanvas) drawTemp(tempCanvas, tempData);
    if (rpmCanvas) drawRPM(rpmCanvas);
    if (voltageCanvas) drawVoltage(voltageCanvas);
    if (servoCanvas) drawServo(servoCanvas);
  });
}

// ══════════════════════════════════════════════════════════════
//  SIMULATION ENGINE
// ══════════════════════════════════════════════════════════════

function startSimulation() {
  if (simTimer) return;
  simTime = 0;

  // Reset graph data so changes are immediately visible
  clearGraphData();

  // Give RPM an initial kick so it doesn't start from 0
  state.tmc.rpm = 800;

  // Mark all boards as alive and start runtime clock
  const now = Date.now();
  for (const key in state.boards) {
    state.boards[key].lastHb = now;
    state.boards[key].alive = true;
  }
  state.esp.imuReady = true;
  runtimeStartMs = Date.now();
  updateBoardStatusUI();
  setStatus("Connected", "connected");

  // Enable controls
  setControlState(true);

  // Reset topology sim one-shot guard so first tick fires immediately
  simulationTick._lastCycle = -1;

  simTimer = setInterval(simulationTick, 100); // 10 Hz simulation rate
}

function stopSimulation() {
  if (simTimer) {
    clearInterval(simTimer);
    simTimer = null;
  }

  // Reset boards and runtime clock
  for (const key in state.boards) {
    state.boards[key].lastHb = 0;
    state.boards[key].alive = false;
  }
  runtimeStartMs = null;
  updateBoardStatusUI();

  // Reset orientation
  window.subOrientation = { pitch: 0, roll: 0, yaw: 0 };

  if (!espConnected) {
    setStatus("Disconnected", false);
    setControlState(false);
  }
}

function simulationTick() {
  simTime += 0.1;
  const t = simTime;

  // Keep boards alive
  const now = Date.now();
  for (const key in state.boards) {
    state.boards[key].lastHb = now;
  }

  // ─── PDU — real data only; do not simulate PMU fields ───────────────────
  // state.pmu.* is populated exclusively from the real ESP32 /info poll.

  // Servo angles: gentle swimming motion (for 3D fin animation in sim mode)
  const bowAngle = 90 + 15 * Math.sin(t * 0.4) + 5 * Math.sin(t * 1.1);
  const sternAngle = 90 + 12 * Math.sin(t * 0.4 + Math.PI) + 4 * Math.sin(t * 0.9);
  state.pmu.servos[0] = Math.round(bowAngle);        // S1 Bow Port
  state.pmu.servos[1] = Math.round(180 - bowAngle);   // S2 Bow Starboard (mirrored)
  state.pmu.servos[2] = Math.round(sternAngle);       // S3 Stern Port
  state.pmu.servos[3] = Math.round(180 - sternAngle); // S4 Stern Starboard (mirrored)

  // ─── TMC telemetry ───
  // RPM: ramps to ~2200 when armed; idles at ~400 RPM when disarmed so the
  // graph always has visible data in sim mode.
  if (state.armed) {
    const targetRPM = 2200 + 400 * Math.sin(t * 0.15);
    state.tmc.rpm = state.tmc.rpm * 0.85 + targetRPM * 0.15 + (Math.random() - 0.5) * 20;
    // 5% chance of a sudden spike in RPM due to cavitation/simulated load drop
    if (Math.random() < 0.05) {
      state.tmc.rpm += 2000 + Math.random() * 1500;
    }
  } else {
    const idleRPM = 400 + 60 * Math.sin(t * 0.3) + 30 * Math.sin(t * 0.7);
    state.tmc.rpm = state.tmc.rpm * 0.92 + idleRPM * 0.08 + (Math.random() - 0.5) * 10;
  }

  // Every ~1 second, push a fake raw CAN message to the log
  if (Math.random() < 0.1) {
    const ids = ["0x100", "0x104", "0x200", "0x301"];
    const simId = ids[Math.floor(Math.random() * ids.length)];
    // Add random ACK response log
    if (Math.random() < 0.5) {
      appendLog(`[CAN] Rx ${simId} ACK`);
    } else {
      appendLog(`[CAN] Rx ${simId}`);
    }
  }
  state.tmc.rpm = Math.max(0, state.tmc.rpm);
  state.tmc.duty = Math.min(100, Math.max(0, (state.tmc.rpm / 4000) * 100));

  // TCU voltage: 2S LiPo direct feed (~7.4 V nominal)
  // Droops under ESC load, very slow self-discharge, ADC ripple + noise.
  const _tcuDuty = state.tmc.duty / 100;               // 0–1
  const _vLoadDrop = _tcuDuty * 0.18;                  // up to 180 mV droop at full throttle
  const _vDischarge = t * 0.00002;                     // ~2 mV / 100 s — barely perceptible
  state.tmc.voltage = 7.40 - _vDischarge - _vLoadDrop
    + Math.sin(t * 0.05) * 0.02 + (Math.random() - 0.5) * 0.04;

  // TCU current: ESC quiescent + motor load + startup inrush spike.
  // Inrush: brushless ESC startup draws ~0.8 A peak, decays with τ ≈ 0.36 s (~2 s visible).
  const _armAge = simulationTick._armTs ? (Date.now() - simulationTick._armTs) / 1000 : Infinity;
  const _inrush = (state.armed && _armAge < 2.5) ? 0.8 * Math.exp(-_armAge * 2.8) : 0;
  // Idle (disarmed): ~0.06 A;  nominal cruise (55% duty): ~0.15 A;  max: ~0.23 A
  state.tmc.current = 0.06 + _tcuDuty * 0.164 + _inrush
    + Math.sin(t * 0.08) * 0.006 + (Math.random() - 0.5) * 0.012;

  // TCU thermal model: ESC + motor self-heating, τ ≈ 20 s.
  // Armed/running: rises 12–18 °C above ambient. Disarmed: cools to ambient + 0.5 °C.
  if (simulationTick._tcuHeat === undefined) simulationTick._tcuHeat = 0;
  const _tcuImuBase = (state.esp.imuTemp != null && state.esp.imuTemp !== 0)
    ? Number(state.esp.imuTemp) : 28;
  const _tcuHeatTarget = state.armed ? 12.0 + _tcuDuty * 6.0 : 0.5;
  simulationTick._tcuHeat += (_tcuHeatTarget - simulationTick._tcuHeat) * 0.005; // τ ≈ 20 s
  state.tmc.temp = _tcuImuBase + simulationTick._tcuHeat
    + Math.sin(t * 0.06) * 0.5 + (Math.random() - 0.5) * 0.4;
  state.tmc.hall = ((Math.floor(t * 50) % 6) & 0x07); // cycling hall states
  state.tmc.sector = Math.floor(t * 50) % 6;

  // ─── ISN telemetry ───
  // IMU Temperature
  if (state.esp.imuReady) {
    state.esp.imuTemp = 28.5 + Math.sin(t * 0.02) * 1.5 + (Math.random() - 0.5) * 0.2;
  }

  // IMU: simulate gentle submarine motion
  // Pitch: nose up/down from dive planes
  const pitch = 0.15 * Math.sin(t * 0.4) + 0.05 * Math.sin(t * 1.3);
  // Roll: slight roll from current/asymmetry
  const roll = 0.08 * Math.sin(t * 0.25 + 0.5) + 0.03 * Math.sin(t * 0.8);
  // Yaw: slow heading changes
  const yaw = 0.1 * Math.sin(t * 0.1);

  // Accelerometer (m/s²) — gravity + motion
  // Overwrite with simulated values when in sim mode
  state.isn.ax = pitch * 9.81 + (Math.random() - 0.5) * 0.05;
  state.isn.ay = roll * 9.81 + (Math.random() - 0.5) * 0.05;
  state.isn.az = 9.81 + (Math.random() - 0.5) * 0.05;
  // Gyro Z (°/s)
  state.isn.gz = yaw * 57.3 + (Math.random() - 0.5) * 0.5;

  state.isn.servoTargets = [...state.pmu.servos];

  // ─── TSN telemetry ───
  state.tsn.uptime = Math.floor(t);
  state.tsn.totalMsgs += Math.floor(Math.random() * 5 + 3);
  state.tsn.busLoad = 8 + Math.sin(t * 0.2) * 3 + (Math.random() - 0.5) * 1;
  if (Math.random() < 0.005) state.tsn.errFrames++;

  // ─── Update 3D model orientation ───
  // Use simulated orientation when in sim mode
  window.subOrientation = {
    pitch: pitch * 120,   // scale up for visibility (degrees)
    roll: roll * 100,
    yaw: yaw * 150,
  };

  // Update servo slider positions to reflect simulated values
  // (servoSliders/servoVals may not exist if the DOM lacks slider controls)
  const _sSliders = (typeof servoSliders !== 'undefined') ? servoSliders : [];
  const _sVals = (typeof servoVals !== 'undefined') ? servoVals : [];
  _sSliders.forEach((s, i) => {
    if (s && !s.matches(":active")) {
      s.value = state.pmu.servos[i];
      if (_sVals[i]) _sVals[i].textContent = `${state.pmu.servos[i]}°`;
    }
  });

  // Update thruster slider
  if (thrusterSlider && !thrusterSlider.matches(":active")) {
    thrusterSlider.value = Math.round(state.tmc.duty);
    if (thrusterVal) thrusterVal.textContent = `${Math.round(state.tmc.duty)}%`;
  }
  // Drive fan speed on 3D model from simulation duty cycle
  window.subThrusterPct = state.tmc.duty;
  // Drive fin deflection on 3D model from simulated servo angles
  window.subServoAngles = [...state.pmu.servos];

  // ─── Simulated CAN topology events ──────────────────────────────────
  // Every ~3 s simulate a command-ACK round-trip to each node
  const SIM_CYCLE_S = 3.0;
  const simCycle = Math.floor(t / SIM_CYCLE_S);
  const cyclePhase = t % SIM_CYCLE_S;
  const targets = ['servo', 'thruster'];
  const simTarget = targets[simCycle % 2];
  const simCmds = { servo: 'Dive', thruster: '50%' };
  const simSeq = simCycle + 1;

  // One-shot: only fire once per cycle (guard against repeated triggers)
  if (typeof simulationTick._lastCycle === 'undefined') simulationTick._lastCycle = -1;

  // Trigger send at start of cycle (one-shot)
  if (simCycle !== simulationTick._lastCycle) {
    simulationTick._lastCycle = simCycle;
    topoTriggerSend(simTarget, simSeq, simCmds[simTarget]);
  }
  // Trigger ACK after ~1.5 s into cycle (one-shot, only if still pending)
  if (cyclePhase >= 1.5 && cyclePhase < 1.7) {
    if (canTopo.pendingAck.target === simTarget && canTopo.pendingAck.seq === simSeq) {
      topoTriggerAck(simTarget, simSeq, 0);
    }
  }

  updateControlUI();
}
