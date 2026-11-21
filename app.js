/* ==================== 1. APPLICATION STATE ==================== */
const DEFAULT_OBSTACLES = [
  { id: 1, x: 0.0, y: -2.40, r: 0.30, label: 'Камень' },
  { id: 2, x: -2.4, y: -3.20, r: 0.42, label: 'Бордюр' },
  { id: 3, x: 2.6, y: 1.80, r: 0.35, label: 'Столб' }
];

const DEFAULT_PEDESTRIANS = [
  {
    id: 'ped_1',
    x: -2.8,
    y: -1.25,
    r: 0.26,
    speed: 0.85,
    headingRad: Math.PI / 2,
    label: 'Пешеход',
    waypoints: [
      { x: -3.2, y: -1.25 },
      { x: 3.2, y: -1.25 }
    ],
    wpIdx: 1,
    knockedTimer: 0,
    knockVx: 0,
    knockVy: 0,
    spinAngle: 0
  },
  {
    id: 'ped_2',
    x: 3.4,
    y: 0.4,
    r: 0.26,
    speed: 0.72,
    headingRad: Math.PI,
    label: 'Пешеход',
    waypoints: [
      { x: 1.6, y: -2.9 },
      { x: 3.5, y: -2.9 },
      { x: 3.5, y: 2.6 },
      { x: 1.4, y: 2.6 }
    ],
    wpIdx: 2,
    knockedTimer: 0,
    knockVx: 0,
    knockVy: 0,
    spinAngle: 0
  }
];

const state = {
  mode: 'comfort', // 'comfort' | 'street' | 'park'
  headlight: false,
  estop: false,
  soundEnabled: true,

  // Secret sandbox override mode
  pedestrianOverride: false,
  pedestriansHitCount: 0,

  // Joystick normalized (-1..1)
  joyX: 0,
  joyY: 0,

  // Sensors & 2D Kinematics (coordinates in real meters)
  speedKmh: 0,
  headingDeg: 0,
  distanceMeters: 0,
  posX: 0,
  posY: 0,
  trackPoints: [{ x: 0, y: 0 }],
  geofenceRadiusM: 50,
  mapViewRadiusM: 8,
  geofenceAlerted: false,

  // Fixed 2D World Obstacles & Walking Pedestrians
  obstacles: JSON.parse(JSON.stringify(DEFAULT_OBSTACLES)),
  pedestrians: JSON.parse(JSON.stringify(DEFAULT_PEDESTRIANS)),

  batteryPct: 86,
  frontDistCm: 145,
  rearDistCm: 350,
  frontTargetLabel: 'Препятствие',
  tiltDeg: 2,

  // Automation thresholds
  brakeThreshCm: 30,
  batThreshPct: 20,
  tiltThreshDeg: 12,
  rolloverDeg: 22,

  // Stats & History
  autoBrakesCount: 0,
  chartHistory: [],
  dbRecords: [],
  lastAutoBrakeAlert: 0,
  lastTiltAlert: 0,
  lastLowBatAlert: 0,
  lastRolloverAlert: 0,

  // Cloud settings
  cloudProvider: 'local',
  cloudUrl: '',
  cloudKey: ''
};

const MODE_LIMITS = { comfort: 3.0, street: 8.0, park: 0.0 };
const MODE_NAMES = { comfort: 'Комфорт', street: 'Улица', park: 'Парковка' };

/* ==================== 2. 2D PEDESTRIANS & ULTRASONIC RAYCASTING ==================== */
function updatePedestrians(dt) {
  for (const ped of state.pedestrians) {
    // Handle knocked-down slide & recovery animation
    if (ped.knockedTimer > 0) {
      ped.knockedTimer = Math.max(0, ped.knockedTimer - dt);
      ped.x += ped.knockVx * dt;
      ped.y += ped.knockVy * dt;
      ped.knockVx *= Math.pow(0.08, dt);
      ped.knockVy *= Math.pow(0.08, dt);
      ped.spinAngle += 10 * dt;
      continue;
    }

    const target = ped.waypoints[ped.wpIdx];
    const toTargetX = target.x - ped.x;
    const toTargetY = target.y - ped.y;
    const distToWp = Math.hypot(toTargetX, toTargetY);

    if (distToWp < 0.25) {
      ped.wpIdx = (ped.wpIdx + 1) % ped.waypoints.length;
      continue;
    }

    let dirX = toTargetX / distToWp;
    let dirY = toTargetY / distToWp;

    // Smooth steering around static obstacles
    for (const obs of state.obstacles) {
      const dx = ped.x - obs.x;
      const dy = ped.y - obs.y;
      const d = Math.hypot(dx, dy);
      const safeDist = obs.r + ped.r + 0.55;
      if (d > 0.001 && d < safeDist) {
        const push = (safeDist - d) / safeDist;
        dirX += (dx / d) * push * 2.2;
        dirY += (dy / d) * push * 2.2;
      }
    }

    // Separate slightly from other pedestrians
    for (const other of state.pedestrians) {
      if (other.id === ped.id) continue;
      const dx = ped.x - other.x;
      const dy = ped.y - other.y;
      const d = Math.hypot(dx, dy);
      if (d > 0.001 && d < 0.75) {
        dirX += (dx / d) * 1.4;
        dirY += (dy / d) * 1.4;
      }
    }

    // Yield to wheelchair only when safety mode is active
    const wdx = ped.x - state.posX;
    const wdy = ped.y - state.posY;
    const distToChair = Math.hypot(wdx, wdy);
    let speedFactor = 1.0;

    if (!state.pedestrianOverride) {
      if (distToChair < 0.72) {
        speedFactor = 0;
      } else if (distToChair < 1.15) {
        speedFactor = 0.45;
        dirX += (wdx / distToChair) * 1.2;
        dirY += (wdy / distToChair) * 1.2;
      }
    }

    const norm = Math.hypot(dirX, dirY);
    if (norm > 0.001 && speedFactor > 0) {
      dirX /= norm;
      dirY /= norm;
      const nextX = ped.x + dirX * ped.speed * speedFactor * dt;
      const nextY = ped.y + dirY * ped.speed * speedFactor * dt;

      const hitsStatic = state.obstacles.some(o => Math.hypot(nextX - o.x, nextY - o.y) < (o.r + ped.r + 0.05));
      const hitsChair = !state.pedestrianOverride && Math.hypot(nextX - state.posX, nextY - state.posY) < (0.35 + ped.r + 0.05);

      if (!hitsStatic && !hitsChair) {
        ped.x = nextX;
        ped.y = nextY;
        ped.headingRad = Math.atan2(dirX, -dirY);
      }
    }
  }
}

function rayCircleDistance(ox, oy, dx, dy, cx, cy, r) {
  const ocx = cx - ox;
  const ocy = cy - oy;
  const distSq = ocx * ocx + ocy * ocy;
  if (distSq <= r * r) return 0;

  const tca = ocx * dx + ocy * dy;
  if (tca < 0) return Infinity;

  const d2 = distSq - tca * tca;
  if (d2 > r * r) return Infinity;

  const thc = Math.sqrt(r * r - d2);
  const t = tca - thc;
  return t >= 0 ? t : Infinity;
}

function measureUltrasonicCone(sensorX, sensorY, baseRad, maxRangeM = 3.5) {
  const spreadAngles = [-0.26, -0.17, -0.08, 0, 0.08, 0.17, 0.26];
  let minHitM = maxRangeM;
  let hitLabel = 'Препятствие';
  const activePeds = state.pedestrians.filter(p => p.knockedTimer <= 0);
  const allTargets = [...state.obstacles, ...activePeds];

  for (const offset of spreadAngles) {
    const angle = baseRad + offset;
    const dx = Math.sin(angle);
    const dy = -Math.cos(angle);

    for (const obj of allTargets) {
      const hit = rayCircleDistance(sensorX, sensorY, dx, dy, obj.x, obj.y, obj.r);
      if (hit < minHitM) {
        minHitM = hit;
        hitLabel = obj.label || 'Препятствие';
      }
    }
  }
  return {
    distCm: Math.max(10, Math.round(minHitM * 100)),
    label: hitLabel
  };
}

function updateSensorsFrom2DWorld() {
  const rad = (state.headingDeg * Math.PI) / 180;
  const fwdX = Math.sin(rad);
  const fwdY = -Math.cos(rad);
  const sensorOffsetM = 0.35;

  const frontSensorX = state.posX + fwdX * sensorOffsetM;
  const frontSensorY = state.posY + fwdY * sensorOffsetM;
  const frontRes = measureUltrasonicCone(frontSensorX, frontSensorY, rad, 3.5);
  state.frontDistCm = frontRes.distCm;
  state.frontTargetLabel = frontRes.label;

  const rearSensorX = state.posX - fwdX * sensorOffsetM;
  const rearSensorY = state.posY - fwdY * sensorOffsetM;
  const rearRes = measureUltrasonicCone(rearSensorX, rearSensorY, rad + Math.PI, 3.5);
  state.rearDistCm = rearRes.distCm;
}

function placeObstacleRelative(distCm, isFront, label) {
  const rad = (state.headingDeg * Math.PI) / 180;
  const dir = isFront ? 1 : -1;
  const obsRadius = 0.35;
  const centerDistM = 0.35 + (distCm / 100) + obsRadius;
  const ox = +(state.posX + Math.sin(rad) * centerDistM * dir).toFixed(2);
  const oy = +(state.posY - Math.cos(rad) * centerDistM * dir).toFixed(2);
  const tagId = isFront ? 'sim_front' : 'sim_rear';

  state.obstacles = state.obstacles.filter(o => o.id !== tagId);
  if (distCm < 345) {
    state.obstacles.push({ id: tagId, x: ox, y: oy, r: obsRadius, label });
  }
  updateSensorsFrom2DWorld();
  syncSimSlidersUI();
}

/* ==================== 3. AUDIO ALERT HELPER ==================== */
let audioCtx = null;
function playBeep(freq = 520, duration = 0.14, type = 'triangle') {
  if (!state.soundEnabled) return;
  try {
    if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    osc.type = type;
    osc.frequency.value = freq;
    gain.gain.setValueAtTime(0.08, audioCtx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + duration);
    osc.connect(gain);
    gain.connect(audioCtx.destination);
    osc.start();
    osc.stop(audioCtx.currentTime + duration);
  } catch (e) {}
}

/* ==================== 4. NOTIFICATIONS & LOGGING ==================== */
function showToast(msg, level = 'info') {
  const container = document.getElementById('toastContainer');
  const el = document.createElement('div');
  el.className = `toast ${level}`;
  el.textContent = msg;
  container.appendChild(el);
  setTimeout(() => {
    el.style.opacity = '0';
    setTimeout(() => el.remove(), 220);
  }, 3400);
}

function addLog(msg, level = 'info') {
  const box = document.getElementById('consoleLog');
  const now = new Date().toLocaleTimeString('ru-RU');
  const line = document.createElement('div');
  line.className = `log-line ${level}`;
  line.textContent = `[${now}] ${msg}`;
  box.prepend(line);
  if (box.children.length > 45) box.lastChild.remove();
}

/* ==================== 5. NAVIGATION, THEME & SOUND ==================== */
function switchTab(tabId) {
  document.querySelectorAll('.tab-page').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.nav-btn').forEach(b => {
    b.classList.toggle('active', b.dataset.tab === tabId);
  });
  document.getElementById(`tab-${tabId}`).classList.add('active');
  if (tabId === 'analytics') drawChart();
  if (tabId === 'cockpit') drawTrajectory();
}

document.querySelectorAll('.nav-btn').forEach(btn => {
  btn.addEventListener('click', () => switchTab(btn.dataset.tab));
});

document.getElementById('themeToggle').addEventListener('click', (e) => {
  const btn = e.currentTarget;
  const isLight = document.body.classList.toggle('light-theme');
  btn.querySelector('span').textContent = isLight ? 'Ночь' : 'День';
  drawTrajectory();
  if (document.getElementById('tab-analytics').classList.contains('active')) drawChart();
});

document.getElementById('soundToggle').addEventListener('click', (e) => {
  const btn = e.currentTarget;
  state.soundEnabled = !state.soundEnabled;
  btn.querySelector('span').textContent = state.soundEnabled ? 'Звук' : 'Выкл';
  btn.classList.toggle('is-muted', !state.soundEnabled);
});

document.getElementById('btnClearLog').addEventListener('click', () => {
  document.getElementById('consoleLog').innerHTML = '';
});

document.getElementById('btnMapZoom').addEventListener('click', (e) => {
  state.mapViewRadiusM = state.mapViewRadiusM === 8 ? 50 : 8;
  e.target.textContent = `${state.mapViewRadiusM} м`;
  drawTrajectory();
});

document.getElementById('btnResetTrack').addEventListener('click', () => {
  state.posX = 0;
  state.posY = 0;
  state.headingDeg = 0;
  state.speedKmh = 0;
  state.trackPoints = [{ x: 0, y: 0 }];
  state.obstacles = JSON.parse(JSON.stringify(DEFAULT_OBSTACLES));
  state.pedestrians = JSON.parse(JSON.stringify(DEFAULT_PEDESTRIANS));
  state.pedestriansHitCount = 0;
  state.geofenceAlerted = false;
  updateSensorsFrom2DWorld();
  syncSimSlidersUI();
  drawTrajectory();
  showToast('Позиция, маршрут, препятствия и пешеходы сброшены', 'info');
});

// Secret Pedestrian Collision Override Button
const secretBtn = document.getElementById('btnSecretPedMode');
secretBtn.addEventListener('mouseenter', () => {
  if (!state.pedestrianOverride) secretBtn.style.opacity = '0.85';
});
secretBtn.addEventListener('mouseleave', () => {
  if (!state.pedestrianOverride) secretBtn.style.opacity = '0.22';
});
secretBtn.addEventListener('click', () => {
  state.pedestrianOverride = !state.pedestrianOverride;
  if (state.pedestrianOverride) {
    secretBtn.style.opacity = '1';
    secretBtn.style.background = 'rgba(255, 59, 48, 0.22)';
    secretBtn.style.borderColor = 'var(--crimson)';
    secretBtn.style.color = '#FCA5A5';
    secretBtn.textContent = '☠️ ON';
    playBeep(300, 0.2, 'sawtooth');
    showToast('☠️ Секретный режим ВКЛ: защита от наезда на пешеходов отключена!', 'crit');
    addLog('[SECRET] Активирован режим отключения защиты от наезда на пешеходов', 'crit');
  } else {
    secretBtn.style.opacity = '0.22';
    secretBtn.style.background = 'transparent';
    secretBtn.style.borderColor = 'transparent';
    secretBtn.style.color = 'var(--text-muted)';
    secretBtn.textContent = '☠️';
    showToast('🛡️ Защита от наезда на пешеходов восстановлена', 'info');
    addLog('[SECRET] Защита от наезда на пешеходов снова включена', 'info');
  }
});

// Interactive obstacle placement/removal on 2D Trajectory Map
document.getElementById('trajectoryCanvas').addEventListener('click', (e) => {
  const canvas = e.currentTarget;
  const rect = canvas.getBoundingClientRect();
  const cx = rect.width / 2;
  const cy = rect.height / 2;
  const viewPx = Math.min(cx, cy) - 14;
  const scale = viewPx / state.mapViewRadiusM;

  const wx = (e.offsetX - cx) / scale;
  const wy = (e.offsetY - cy) / scale;

  if (Math.hypot(wx - state.posX, wy - state.posY) < 0.65) {
    showToast('Нельзя поставить препятствие под саму коляску', 'warn');
    return;
  }

  const existingIdx = state.obstacles.findIndex(o => Math.hypot(o.x - wx, o.y - wy) <= o.r + 0.35);
  if (existingIdx !== -1) {
    const removed = state.obstacles.splice(existingIdx, 1)[0];
    updateSensorsFrom2DWorld();
    syncSimSlidersUI();
    drawTrajectory();
    showToast(`Убрано препятствие: ${removed.label}`, 'info');
    return;
  }

  state.obstacles.push({
    id: Date.now(),
    x: +wx.toFixed(2),
    y: +wy.toFixed(2),
    r: 0.32,
    label: 'Препятствие'
  });
  updateSensorsFrom2DWorld();
  syncSimSlidersUI();
  drawTrajectory();
  showToast(`Поставлено препятствие (${wx.toFixed(1)} м, ${wy.toFixed(1)} м)`, 'info');
});

/* ==================== 6. INTERACTIVE WIRING DIAGRAM ==================== */
const PINS = [
  { id: 'esp_3v3', board: 'esp32', label: '3V3', x: 320, y: 120, type: 'VCC', side: 'left' },
  { id: 'esp_gnd1', board: 'esp32', label: 'GND', x: 320, y: 155, type: 'GND', side: 'left' },
  { id: 'esp_g4', board: 'esp32', label: 'GPIO 4', x: 320, y: 195, type: 'ECHO', side: 'left' },
  { id: 'esp_g5', board: 'esp32', label: 'GPIO 5', x: 320, y: 230, type: 'TRIG', side: 'left' },
  { id: 'esp_gnd2', board: 'esp32', label: 'GND', x: 320, y: 315, type: 'GND', side: 'left' },

  { id: 'esp_5v', board: 'esp32', label: '5V', x: 500, y: 120, type: 'VCC', side: 'right' },
  { id: 'esp_gnd3', board: 'esp32', label: 'GND', x: 500, y: 155, type: 'GND', side: 'right' },
  { id: 'esp_g21', board: 'esp32', label: 'GPIO 21', x: 500, y: 195, type: 'SDA', side: 'right' },
  { id: 'esp_g22', board: 'esp32', label: 'GPIO 22', x: 500, y: 230, type: 'SCL', side: 'right' },
  { id: 'esp_g26', board: 'esp32', label: 'GPIO 26', x: 500, y: 270, type: 'IN1', side: 'right' },
  { id: 'esp_g27', board: 'esp32', label: 'GPIO 27', x: 500, y: 310, type: 'IN2', side: 'right' },
  { id: 'esp_gnd4', board: 'esp32', label: 'GND', x: 500, y: 350, type: 'GND', side: 'right' },

  { id: 'son_vcc', board: 'sonar', label: 'VCC', x: 205, y: 85, type: 'VCC', side: 'right' },
  { id: 'son_trig', board: 'sonar', label: 'TRIG', x: 205, y: 115, type: 'TRIG', side: 'right' },
  { id: 'son_echo', board: 'sonar', label: 'ECHO', x: 205, y: 145, type: 'ECHO', side: 'right' },
  { id: 'son_gnd', board: 'sonar', label: 'GND', x: 205, y: 175, type: 'GND', side: 'right' },

  { id: 'bat_vcc', board: 'battery', label: 'VCC', x: 205, y: 300, type: 'VCC', side: 'right' },
  { id: 'bat_gnd', board: 'battery', label: 'GND', x: 205, y: 330, type: 'GND', side: 'right' },
  { id: 'bat_sda', board: 'battery', label: 'SDA', x: 205, y: 360, type: 'SDA', side: 'right' },
  { id: 'bat_scl', board: 'battery', label: 'SCL', x: 205, y: 390, type: 'SCL', side: 'right' },

  { id: 'tlt_vcc', board: 'tilt', label: 'VCC', x: 615, y: 85, type: 'VCC', side: 'left' },
  { id: 'tlt_gnd', board: 'tilt', label: 'GND', x: 615, y: 115, type: 'GND', side: 'left' },
  { id: 'tlt_sda', board: 'tilt', label: 'SDA', x: 615, y: 145, type: 'SDA', side: 'left' },
  { id: 'tlt_scl', board: 'tilt', label: 'SCL', x: 615, y: 175, type: 'SCL', side: 'left' },

  { id: 'mot_5v', board: 'motors', label: '5V', x: 615, y: 300, type: 'VCC', side: 'left' },
  { id: 'mot_gnd', board: 'motors', label: 'GND', x: 615, y: 330, type: 'GND', side: 'left' },
  { id: 'mot_in1', board: 'motors', label: 'IN1', x: 615, y: 360, type: 'IN1', side: 'left' },
  { id: 'mot_in2', board: 'motors', label: 'IN2', x: 615, y: 390, type: 'IN2', side: 'left' }
];

const DEFAULT_WIRES = [
  { from: 'son_vcc', to: 'esp_3v3', color: '#FF3B30' },
  { from: 'son_trig', to: 'esp_g5', color: '#38BDF8' },
  { from: 'son_echo', to: 'esp_g4', color: '#00E599' },
  { from: 'son_gnd', to: 'esp_gnd1', color: '#71717A' },

  { from: 'bat_vcc', to: 'esp_3v3', color: '#FF3B30' },
  { from: 'bat_gnd', to: 'esp_gnd2', color: '#71717A' },
  { from: 'bat_sda', to: 'esp_g21', color: '#FF9F1C' },
  { from: 'bat_scl', to: 'esp_g22', color: '#A855F7' },

  { from: 'tlt_vcc', to: 'esp_5v', color: '#FF3B30' },
  { from: 'tlt_gnd', to: 'esp_gnd3', color: '#71717A' },
  { from: 'tlt_sda', to: 'esp_g21', color: '#FF9F1C' },
  { from: 'tlt_scl', to: 'esp_g22', color: '#A855F7' },

  { from: 'mot_5v', to: 'esp_5v', color: '#FF3B30' },
  { from: 'mot_gnd', to: 'esp_gnd4', color: '#71717A' },
  { from: 'mot_in1', to: 'esp_g26', color: '#38BDF8' },
  { from: 'mot_in2', to: 'esp_g27', color: '#00E599' }
];

const MODULE_INFO = {
  esp32: {
    title: 'ESP32 DevKit V1 (Контроллер)',
    desc: 'Главный микроконтроллер с Wi-Fi. Считывает данные по шине I2C (GPIO 21/22), измеряет импульсы ультразвукового сонара и формирует ШИМ-сигналы для драйвера двигателей.'
  },
  sonar: {
    title: 'HC-SR04 (Ультразвуковой сонар)',
    desc: 'Измеряет расстояние до статических препятствий и пешеходов (10–350 см). При сокращении дистанции ниже порога запускает алгоритм автоторможения.'
  },
  battery: {
    title: 'INA219 (Датчик тока и напряжения АКБ)',
    desc: 'Подключается по цифровой шине I2C. Контролирует напряжение 24В батареи, ток нагрузки моторов и остаточный заряд в процентах.'
  },
  tilt: {
    title: 'MPU6050 (Гироскоп / Акселерометр)',
    desc: 'Определяет угол наклона коляски при заезде на пандус или критическое опрокидывание по шине I2C.'
  },
  motors: {
    title: 'L298N (Двухканальный драйвер моторов)',
    desc: 'Силовой мост управления левым и правым мотор-колесами. Без подключения IN1/IN2 и питания движение коляски заблокировано.'
  }
};

let wires = JSON.parse(JSON.stringify(DEFAULT_WIRES));
let selectedPinId = null;

function getPinById(id) {
  return PINS.find(p => p.id === id);
}

function isModuleConnected(modName) {
  const modPins = PINS.filter(p => p.board === modName);
  return modPins.every(mp => wires.some(w => w.from === mp.id || w.to === mp.id));
}

function renderWiring() {
  const pinsLayer = document.getElementById('pinsLayer');
  const wiresLayer = document.getElementById('wiresLayer');
  const pulsesLayer = document.getElementById('pulsesLayer');
  pinsLayer.innerHTML = '';
  wiresLayer.innerHTML = '';
  pulsesLayer.innerHTML = '';

  wires.forEach((w, idx) => {
    const p1 = getPinById(w.from);
    const p2 = getPinById(w.to);
    if (!p1 || !p2) return;
    const dx = Math.abs(p2.x - p1.x) * 0.5;
    const c1x = p1.x < p2.x ? p1.x + dx : p1.x - dx;
    const c2x = p1.x < p2.x ? p2.x - dx : p2.x + dx;
    const d = `M ${p1.x} ${p1.y} C ${c1x} ${p1.y}, ${c2x} ${p2.y}, ${p2.x} ${p2.y}`;

    const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
    path.setAttribute('d', d);
    path.setAttribute('stroke', w.color);
    path.setAttribute('class', 'wire-path');
    path.addEventListener('click', () => {
      wires.splice(idx, 1);
      renderWiring();
      updateModuleStatuses();
      showToast('Провод отсоединён', 'warn');
    });
    wiresLayer.appendChild(path);

    if (w.color !== '#71717A' && w.color !== '#FF3B30') {
      const pulse = document.createElementNS('http://www.w3.org/2000/svg', 'path');
      pulse.setAttribute('d', d);
      pulse.setAttribute('class', 'wire-pulse');
      pulsesLayer.appendChild(pulse);
    }
  });

  PINS.forEach(p => {
    const isConnected = wires.some(w => w.from === p.id || w.to === p.id);
    const isSelected = selectedPinId === p.id;

    const txt = document.createElementNS('http://www.w3.org/2000/svg', 'text');
    txt.setAttribute('x', p.side === 'left' ? p.x + 12 : p.x - 12);
    txt.setAttribute('y', p.y + 4);
    txt.setAttribute('text-anchor', p.side === 'left' ? 'start' : 'end');
    txt.setAttribute('fill', isConnected ? '#FAFAFA' : '#A1A1AA');
    txt.setAttribute('font-size', '11');
    txt.setAttribute('font-family', 'monospace');
    txt.textContent = p.label;
    pinsLayer.appendChild(txt);

    const hit = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
    hit.setAttribute('cx', p.x);
    hit.setAttribute('cy', p.y);
    hit.setAttribute('r', '12');
    hit.setAttribute('fill', 'transparent');
    hit.setAttribute('class', 'pin-hit');
    hit.addEventListener('click', () => handlePinClick(p));
    pinsLayer.appendChild(hit);

    const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
    circle.setAttribute('cx', p.x);
    circle.setAttribute('cy', p.y);
    circle.setAttribute('r', isSelected ? '7.5' : '5.5');
    circle.setAttribute('fill', isSelected ? '#38BDF8' : (isConnected ? '#00E599' : '#09090B'));
    circle.setAttribute('stroke', isSelected ? '#FFFFFF' : '#A1A1AA');
    circle.setAttribute('stroke-width', '1.5');
    circle.setAttribute('class', 'pin-circle');
    circle.style.pointerEvents = 'none';
    pinsLayer.appendChild(circle);
  });

  document.getElementById('wireCountBadge').textContent = `${wires.length}/16 пинов`;
}

function handlePinClick(pin) {
  if (!selectedPinId) {
    selectedPinId = pin.id;
    renderWiring();
    return;
  }
  if (selectedPinId === pin.id) {
    selectedPinId = null;
    renderWiring();
    return;
  }

  const p1 = getPinById(selectedPinId);
  const p2 = pin;
  selectedPinId = null;

  if (p1.board === p2.board || (p1.board !== 'esp32' && p2.board !== 'esp32')) {
    showToast('Соединяйте пин периферийного модуля с пином ESP32', 'warn');
    renderWiring();
    return;
  }

  if ((p1.type === 'VCC' && p2.type === 'GND') || (p1.type === 'GND' && p2.type === 'VCC')) {
    showToast('⚠️ КОРОТКОЕ ЗАМЫКАНИЕ! Цепь питания (VCC → GND) заблокирована', 'crit');
    playBeep(220, 0.25);
    addLog('Ошибка сборки: попытка замыкания VCC на GND', 'crit');
    renderWiring();
    return;
  }

  if (p1.type !== p2.type) {
    showToast(`Неверный пин: ${p1.label} (${p1.type}) несовместим с ${p2.label} (${p2.type})`, 'warn');
    renderWiring();
    return;
  }

  const modPin = p1.board === 'esp32' ? p2 : p1;
  const espPin = p1.board === 'esp32' ? p1 : p2;

  wires = wires.filter(w => w.from !== modPin.id && w.to !== modPin.id);

  const colorMap = { VCC: '#FF3B30', GND: '#71717A', SDA: '#FF9F1C', SCL: '#A855F7', TRIG: '#38BDF8', ECHO: '#00E599', IN1: '#38BDF8', IN2: '#00E599' };
  wires.push({ from: modPin.id, to: espPin.id, color: colorMap[modPin.type] || '#38BDF8' });

  renderWiring();
  updateModuleStatuses();
  showToast(`Подключено: ${modPin.label} → ${espPin.label}`, 'info');
}

document.querySelectorAll('.board-unit').forEach(el => {
  el.addEventListener('click', () => {
    const mod = el.dataset.module;
    const info = MODULE_INFO[mod];
    if (!info) return;
    document.getElementById('inspTitle').textContent = info.title;
    document.getElementById('inspDesc').textContent = info.desc;
    const list = document.getElementById('inspPins');
    list.innerHTML = '';
    PINS.filter(p => p.board === mod).forEach(p => {
      const w = wires.find(item => item.from === p.id || item.to === p.id);
      const target = w ? getPinById(w.from === p.id ? w.to : w.from) : null;
      const row = document.createElement('div');
      row.className = 'pin-row';
      row.innerHTML = `<span>${p.label}</span><span style="color:${target ? '#00E599' : '#FF3B30'}">${target ? '→ ' + target.label : 'Не подключен'}</span>`;
      list.appendChild(row);
    });
  });
});

document.getElementById('btnAutoWire').addEventListener('click', () => {
  wires = JSON.parse(JSON.stringify(DEFAULT_WIRES));
  selectedPinId = null;
  renderWiring();
  updateModuleStatuses();
  showToast('Все 4 модуля подключены к ESP32', 'info');
  addLog('Схема подключения собрана автоматически (16/16 пинов)', 'info');
});

document.getElementById('btnClearWires').addEventListener('click', () => {
  wires = [];
  selectedPinId = null;
  renderWiring();
  updateModuleStatuses();
  showToast('Провода сброшены — проверьте статус датчиков на пульте', 'warn');
  addLog('Все соединения на плате ESP32 разорваны', 'warn');
});

function updateModuleStatuses() {
  const sonarOk = isModuleConnected('sonar');
  const batOk = isModuleConnected('battery');
  const tiltOk = isModuleConnected('tilt');
  const motOk = isModuleConnected('motors');
  const activeCount = [sonarOk, batOk, tiltOk, motOk].filter(Boolean).length;

  document.getElementById('sysStatusText').textContent = `${activeCount}/4 OK`;
  document.getElementById('sysDot').classList.toggle('warn', activeCount < 4);

  document.getElementById('cardSonar').classList.toggle('offline', !sonarOk);
  document.getElementById('cardBattery').classList.toggle('offline', !batOk);
  document.getElementById('cardTilt').classList.toggle('offline', !tiltOk);
  document.getElementById('cardMotors').classList.toggle('offline', !motOk);

  document.getElementById('badgeMotors').textContent = motOk ? 'L298N: АКТИВЕН' : 'НЕТ СИГНАЛА';
  document.getElementById('badgeMotors').className = motOk ? 'badge' : 'badge err';
}

/* ==================== 7. VIRTUAL JOYSTICK & KEYBOARD ==================== */
const joyBase = document.getElementById('joystickBase');
const joyKnob = document.getElementById('joystickKnob');
let joyActive = false;

function updateJoystickFromEvent(clientX, clientY) {
  const rect = joyBase.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const maxR = rect.width / 2 - 26;

  let dx = clientX - cx;
  let dy = clientY - cy;
  const dist = Math.hypot(dx, dy);
  if (dist > maxR) {
    dx = (dx / dist) * maxR;
    dy = (dy / dist) * maxR;
  }

  joyKnob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
  state.joyX = +(dx / maxR).toFixed(2);
  state.joyY = +(-dy / maxR).toFixed(2);
}

function resetJoystick() {
  joyActive = false;
  state.joyX = 0;
  state.joyY = 0;
  joyKnob.style.transform = 'translate(-50%, -50%)';
}

joyBase.addEventListener('pointerdown', (e) => {
  joyActive = true;
  joyBase.setPointerCapture(e.pointerId);
  updateJoystickFromEvent(e.clientX, e.clientY);
});

joyBase.addEventListener('pointermove', (e) => {
  if (!joyActive) return;
  updateJoystickFromEvent(e.clientX, e.clientY);
});

joyBase.addEventListener('pointerup', resetJoystick);
joyBase.addEventListener('pointercancel', resetJoystick);

const keys = {};
window.addEventListener('keydown', (e) => {
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
  keys[e.code] = true;
  syncKeyboardToJoy();
});
window.addEventListener('keyup', (e) => {
  keys[e.code] = false;
  syncKeyboardToJoy();
});

function syncKeyboardToJoy() {
  if (joyActive) return;
  let x = 0, y = 0;
  if (keys['KeyW'] || keys['ArrowUp']) y += 1;
  if (keys['KeyS'] || keys['ArrowDown']) y -= 1;
  if (keys['KeyD'] || keys['ArrowRight']) x += 1;
  if (keys['KeyA'] || keys['ArrowLeft']) x -= 1;
  state.joyX = x;
  state.joyY = y;
  const maxR = 47;
  joyKnob.style.transform = `translate(calc(-50% + ${x * maxR}px), calc(-50% + ${-y * maxR}px))`;
}

/* ==================== 8. MODES, ACTUATORS & SIMULATOR SLIDERS ==================== */
function setMode(newMode) {
  state.mode = newMode;
  document.querySelectorAll('.mode-btn').forEach(b => {
    b.classList.toggle('active', b.dataset.mode === newMode);
  });
  addLog(`Профиль скорости: ${MODE_NAMES[newMode]}`, 'info');
}

document.querySelectorAll('.mode-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    if (state.estop) {
      showToast('Сначала снимите аварийный СТОП', 'warn');
      return;
    }
    setMode(btn.dataset.mode);
  });
});

document.getElementById('btnHeadlight').addEventListener('click', (e) => {
  state.headlight = !state.headlight;
  e.target.textContent = state.headlight ? '🔦 Фара: ВКЛ' : '🔦 Фара: ВЫКЛ';
  e.target.classList.toggle('on', state.headlight);
  document.getElementById('headlightCone').setAttribute('opacity', state.headlight ? '1' : '0');
  document.getElementById('headlightLed').setAttribute('fill', state.headlight ? '#FF9F1C' : '#52525B');
  addLog(`LED-фара ${state.headlight ? 'включена' : 'выключена'}`, 'info');
});

document.getElementById('btnHorn').addEventListener('click', () => {
  playBeep(440, 0.22, 'sawtooth');
  showToast('📢 Звуковой сигнал подан', 'info');
  addLog('Активирован звуковой сигнал (клаксон)', 'info');
});

document.getElementById('btnEstop').addEventListener('click', (e) => {
  state.estop = !state.estop;
  e.target.classList.toggle('active', state.estop);
  e.target.textContent = state.estop ? '🛑 СТОП АКТИВЕН' : '🛑 СТОП';
  if (state.estop) {
    state.speedKmh = 0;
    playBeep(320, 0.25);
    showToast('Аварийный тормоз заблокирован вручную!', 'crit');
    addLog('Активирован аварийный ручной тормоз (E-STOP)', 'crit');
  } else {
    addLog('Аварийный тормоз снят', 'info');
  }
});

function triggerSosAlert(reason = 'Ручной вызов SOS пользователем') {
  playBeep(780, 0.35, 'square');
  showToast(`🆘 SOS ТРЕВОГА: ${reason}`, 'crit');
  addLog(`[SOS] Экстренный вызов опекуну! Причина: ${reason}`, 'crit');
  saveRecordToCloud('SOS_ТРЕВОГА');
}

document.getElementById('btnSos').addEventListener('click', () => {
  triggerSosAlert('Нажата кнопка SOS на пульте');
});

document.getElementById('inpBrakeThresh').addEventListener('input', (e) => {
  state.brakeThreshCm = +e.target.value;
  document.getElementById('valBrakeThresh').textContent = `${state.brakeThreshCm} см`;
  document.getElementById('lblBrakeThresh').textContent = `${state.brakeThreshCm} см`;
});

document.getElementById('inpBatThresh').addEventListener('input', (e) => {
  state.batThreshPct = +e.target.value;
  document.getElementById('valBatThresh').textContent = `${state.batThreshPct}%`;
  document.getElementById('lblBatThresh').textContent = `${state.batThreshPct}%`;
});

document.getElementById('inpTiltThresh').addEventListener('input', (e) => {
  state.tiltThreshDeg = +e.target.value;
  document.getElementById('valTiltThresh').textContent = `${state.tiltThreshDeg}°`;
  document.getElementById('lblTiltThresh').textContent = `${state.tiltThreshDeg}°`;
});

function syncSimSlidersUI() {
  const activeEl = document.activeElement;
  if (activeEl?.id !== 'simFront') {
    document.getElementById('simFront').value = Math.round(state.frontDistCm);
  }
  document.getElementById('simFrontVal').textContent = `${Math.round(state.frontDistCm)} см`;

  if (activeEl?.id !== 'simRear') {
    document.getElementById('simRear').value = Math.round(state.rearDistCm);
  }
  document.getElementById('simRearVal').textContent = `${Math.round(state.rearDistCm)} см`;

  document.getElementById('simTilt').value = state.tiltDeg;
  document.getElementById('simTiltVal').textContent = `${state.tiltDeg}°`;
  document.getElementById('simBat').value = Math.round(state.batteryPct);
  document.getElementById('simBatVal').textContent = `${Math.round(state.batteryPct)}%`;
}

document.getElementById('simFront').addEventListener('input', (e) => {
  placeObstacleRelative(+e.target.value, true, 'Стена');
});
document.getElementById('simRear').addEventListener('input', (e) => {
  placeObstacleRelative(+e.target.value, false, 'Барьер');
});
document.getElementById('simTilt').addEventListener('input', (e) => {
  state.tiltDeg = +e.target.value;
  syncSimSlidersUI();
});
document.getElementById('simBat').addEventListener('input', (e) => {
  state.batteryPct = +e.target.value;
  syncSimSlidersUI();
});

document.getElementById('scenWall').addEventListener('click', () => {
  placeObstacleRelative(22, true, 'Стена');
  showToast('Сценарий: препятствие в 22 см спереди!', 'crit');
});
document.getElementById('scenRamp').addEventListener('click', () => {
  state.tiltDeg = 16;
  syncSimSlidersUI();
  showToast('Сценарий: заезд на крутой пандус (16°)', 'warn');
});
document.getElementById('scenLowBat').addEventListener('click', () => {
  state.batteryPct = 15;
  syncSimSlidersUI();
  showToast('Сценарий: критический заряд АКБ (15%)', 'warn');
});
document.getElementById('scenRollover').addEventListener('click', () => {
  state.tiltDeg = 24;
  syncSimSlidersUI();
});

/* ==================== 9. 2D PHYSICS, AUTOMATION & ANIMATION LOOP ==================== */
let treadOffset = 0;

function stepSimulation(dt) {
  const sonarOk = isModuleConnected('sonar');
  const batOk = isModuleConnected('battery');
  const tiltOk = isModuleConnected('tilt');
  const motOk = isModuleConnected('motors');

  // Move pedestrians and update true 2D ultrasonic raycasts
  updatePedestrians(dt);
  updateSensorsFrom2DWorld();
  syncSimSlidersUI();

  const now = Date.now();
  let maxSpeed = MODE_LIMITS[state.mode];

  // RULE 2: Low battery -> switch to Comfort mode
  if (batOk && state.batteryPct < state.batThreshPct && state.mode === 'street') {
    setMode('comfort');
    maxSpeed = MODE_LIMITS.comfort;
    if (now - state.lastLowBatAlert > 6000) {
      state.lastLowBatAlert = now;
      playBeep(440, 0.2);
      showToast(`АКБ ${Math.round(state.batteryPct)}% < ${state.batThreshPct}%: включен эко-режим Комфорт`, 'warn');
      addLog(`[AUTO] Низкий заряд (${Math.round(state.batteryPct)}%) → автопереход в Комфорт`, 'warn');
    }
  }

  // RULE 3 & 4: Steep ramp tilt limit & Rollover SOS
  if (tiltOk && state.tiltDeg >= state.rolloverDeg) {
    maxSpeed = 0;
    if (now - state.lastRolloverAlert > 6000) {
      state.lastRolloverAlert = now;
      triggerSosAlert(`Критический крен ${state.tiltDeg}° (угроза опрокидывания)`);
    }
  } else if (tiltOk && state.tiltDeg > state.tiltThreshDeg) {
    maxSpeed = Math.min(maxSpeed, 2.0);
    if (now - state.lastTiltAlert > 6000) {
      state.lastTiltAlert = now;
      playBeep(480, 0.18);
      showToast(`Опасный уклон ${state.tiltDeg}° > ${state.tiltThreshDeg}°: скорость ограничена`, 'warn');
      addLog(`[AUTO] Уклон ${state.tiltDeg}° превысил порог → лимит 2.0 км/ч`, 'warn');
    }
  }

  let desiredSpeed = (motOk && !state.estop) ? state.joyY * maxSpeed : 0;

  // RULE 1: Anti-collision auto-brake (bypassed for pedestrians when secret mode is active)
  let autoBrakeTriggered = false;
  const bypassPedestrianBrake = state.pedestrianOverride && state.frontTargetLabel === 'Пешеход';
  if (sonarOk && state.frontDistCm <= state.brakeThreshCm && desiredSpeed > 0 && !bypassPedestrianBrake) {
    desiredSpeed = 0;
    autoBrakeTriggered = true;
    if (now - state.lastAutoBrakeAlert > 3500) {
      state.lastAutoBrakeAlert = now;
      state.autoBrakesCount++;
      playBeep(680, 0.25);
      const targetName = state.frontTargetLabel;
      showToast(`🛑 АВТОТОРМОЗ! ${targetName} спереди: ${Math.round(state.frontDistCm)} см`, 'crit');
      addLog(`[AUTO] Экстренное торможение! ${targetName} (${Math.round(state.frontDistCm)} см < ${state.brakeThreshCm} см)`, 'crit');
      saveRecordToCloud('АВТОТОРМОЗ');
    }
  }

  state.speedKmh += (desiredSpeed - state.speedKmh) * Math.min(1, dt * 8);
  if (Math.abs(state.speedKmh) < 0.04) state.speedKmh = 0;

  // Differential steering rotation
  if (motOk && !state.estop && Math.abs(state.joyX) > 0.05) {
    state.headingDeg = (state.headingDeg + state.joyX * 105 * dt + 360) % 360;
    updateSensorsFrom2DWorld();
  }

  // 2D Position Integration with physical collision check
  const signedSpeedMs = state.speedKmh / 3.6;
  if (Math.abs(signedSpeedMs) > 0.005) {
    const rad = (state.headingDeg * Math.PI) / 180;
    const nextX = state.posX + Math.sin(rad) * signedSpeedMs * dt;
    const nextY = state.posY - Math.cos(rad) * signedSpeedMs * dt;

    const collidesStatic = state.obstacles.some(o => Math.hypot(nextX - o.x, nextY - o.y) < (0.35 + o.r));
    const collidesPed = !state.pedestrianOverride && state.pedestrians.some(p => p.knockedTimer <= 0 && Math.hypot(nextX - p.x, nextY - p.y) < (0.35 + p.r));

    if (!collidesStatic && !collidesPed) {
      const stepDist = Math.hypot(nextX - state.posX, nextY - state.posY);
      state.posX = nextX;
      state.posY = nextY;
      state.distanceMeters += stepDist;

      const lastPt = state.trackPoints[state.trackPoints.length - 1];
      if (Math.hypot(state.posX - lastPt.x, state.posY - lastPt.y) > 0.12) {
        state.trackPoints.push({ x: state.posX, y: state.posY });
        if (state.trackPoints.length > 350) state.trackPoints.shift();
      }

      if (batOk) {
        state.batteryPct = Math.max(1, state.batteryPct - stepDist * 0.04);
      }
    } else {
      state.speedKmh = 0;
    }

    // Check pedestrian hit when secret override mode is ON
    if (state.pedestrianOverride) {
      for (const ped of state.pedestrians) {
        if (ped.knockedTimer <= 0 && Math.hypot(state.posX - ped.x, state.posY - ped.y) < (0.35 + ped.r + 0.08)) {
          ped.knockedTimer = 1.8;
          ped.spinAngle = 0;
          const pushAngle = Math.atan2(ped.x - state.posX, ped.y - state.posY);
          const impactSpeed = Math.max(2.2, Math.abs(state.speedKmh) * 0.9);
          ped.knockVx = Math.sin(pushAngle) * impactSpeed;
          ped.knockVy = Math.cos(pushAngle) * impactSpeed;
          state.pedestriansHitCount++;
          playBeep(190, 0.28, 'sawtooth');
          showToast(`💥 Пешеход сбит! Сбито: ${state.pedestriansHitCount}`, 'crit');
          addLog(`[OVERRIDE] Столкновение с пешеходом! Счётчик: ${state.pedestriansHitCount}`, 'crit');
          saveRecordToCloud('НАЕЗД_ТЕСТ');
        }
      }
    }

    updateSensorsFrom2DWorld();

    const distFromHome = Math.hypot(state.posX, state.posY);
    if (distFromHome > state.geofenceRadiusM && !state.geofenceAlerted) {
      state.geofenceAlerted = true;
      playBeep(560, 0.25);
      showToast(`📍 Выход за пределы безопасной геозоны (${Math.round(distFromHome)} м)!`, 'warn');
      addLog(`[GEO] Коляска покинула безопасный радиус ${state.geofenceRadiusM} м`, 'warn');
      saveRecordToCloud('ВЫХОД_ГЕОЗОНЫ');
    }
  }

  // Update Wheelchair SVG Animation & Radar Obstacle Blip
  const steerAngle = state.joyX * 28;
  document.getElementById('casterLeft').setAttribute('transform', `translate(86, 82) rotate(${steerAngle})`);
  document.getElementById('casterRight').setAttribute('transform', `translate(154, 82) rotate(${steerAngle})`);

  treadOffset = (treadOffset - state.speedKmh * dt * 18) % 20;
  if (treadOffset < 0) treadOffset += 20;
  [-20, 0, 20].forEach((base, i) => {
    const y = base + treadOffset - 10;
    document.getElementById(`treadL${i + 1}`).setAttribute('y1', y);
    document.getElementById(`treadL${i + 1}`).setAttribute('y2', y);
    document.getElementById(`treadR${i + 1}`).setAttribute('y1', y);
    document.getElementById(`treadR${i + 1}`).setAttribute('y2', y);
  });

  const sonarColor = !sonarOk ? '#52525B' : (state.frontDistCm <= state.brakeThreshCm ? '#FF3B30' : (state.frontDistCm < 80 ? '#FF9F1C' : '#00E599'));
  ['sonarFront1', 'sonarFront2', 'sonarFront3'].forEach(id => {
    document.getElementById(id).setAttribute('stroke', sonarColor);
  });

  const blip = document.getElementById('radarObstacleGroup');
  if (sonarOk && state.frontDistCm < 180) {
    const norm = Math.max(0, Math.min(1, (state.frontDistCm - 15) / 165));
    const blipY = 52 - norm * 42;
    const blipColor = state.frontTargetLabel === 'Пешеход' && state.frontDistCm > state.brakeThreshCm ? '#A855F7' : sonarColor;
    blip.setAttribute('transform', `translate(120, ${blipY.toFixed(1)})`);
    blip.setAttribute('opacity', '0.95');
    blip.querySelector('ellipse').setAttribute('stroke', blipColor);
    blip.querySelector('circle').setAttribute('fill', blipColor);
  } else {
    blip.setAttribute('opacity', '0');
  }

  const motionBadge = document.getElementById('motionBadge');
  if (!motOk) {
    motionBadge.textContent = 'НЕТ СИГНАЛА МОТОРОВ';
    motionBadge.className = 'badge err';
  } else if (state.estop || autoBrakeTriggered) {
    motionBadge.textContent = 'ТОРМОЗ АКТИВЕН';
    motionBadge.className = 'badge err';
  } else if (Math.abs(state.speedKmh) > 0.1) {
    motionBadge.textContent = state.speedKmh > 0 ? 'ДВИЖЕНИЕ ВПЕРЕД' : 'ЗАДНИЙ ХОД';
    motionBadge.className = 'badge';
  } else {
    motionBadge.textContent = 'ОЖИДАНИЕ';
    motionBadge.className = 'badge';
  }

  document.getElementById('valSpeed').innerHTML = `${Math.abs(state.speedKmh).toFixed(1)} <small style="font-size:14px;">км/ч</small>`;
  document.getElementById('subDirection').textContent = `Курс: ${Math.round(state.headingDeg)}° • Путь: ${Math.round(state.distanceMeters)} м`;
  document.getElementById('barSpeed').style.width = `${Math.min(100, (Math.abs(state.speedKmh) / 8) * 100)}%`;

  if (batOk) {
    const pct = Math.round(state.batteryPct);
    const volt = (21.6 + (pct / 100) * 3.6).toFixed(1);
    const amps = (0.8 + Math.abs(state.speedKmh) * 1.4).toFixed(1);
    const rangeKm = ((pct / 100) * 20).toFixed(1);
    document.getElementById('valBattery').innerHTML = `${pct} <small style="font-size:14px;">%</small>`;
    document.getElementById('valBattery').style.color = pct < state.batThreshPct ? 'var(--crimson)' : 'var(--emerald)';
    document.getElementById('badgeBattery').textContent = `${volt} В`;
    document.getElementById('badgeBattery').className = pct < state.batThreshPct ? 'badge err' : 'badge';
    document.getElementById('subCurrent').textContent = `${amps} А • Запас: ~${rangeKm} км`;
    document.getElementById('barBattery').style.width = `${pct}%`;
    document.getElementById('barBattery').style.background = pct < state.batThreshPct ? 'var(--crimson)' : 'var(--emerald)';
  } else {
    document.getElementById('valBattery').innerHTML = `-- <small style="font-size:14px;">%</small>`;
    document.getElementById('badgeBattery').textContent = 'НЕТ СИГНАЛА';
    document.getElementById('badgeBattery').className = 'badge err';
    document.getElementById('subCurrent').textContent = 'Проверьте шину I2C (INA219)';
  }

  if (sonarOk) {
    const f = Math.round(state.frontDistCm);
    const r = Math.round(state.rearDistCm);
    const frontTag = f < 340 ? ` (${state.frontTargetLabel})` : '';
    document.getElementById('radarFrontLabel').textContent = `Спереди: ${f} см${frontTag}`;
    document.getElementById('radarRearLabel').textContent = `Сзади: ${r} см`;
    document.getElementById('valSonar').innerHTML = `${f} <small style="font-size:14px;">см</small>`;
    document.getElementById('valSonar').style.color = sonarColor;
    document.getElementById('badgeSonar').textContent = f <= state.brakeThreshCm ? 'ОПАСНОСТЬ' : (f < 80 ? 'ВНИМАНИЕ' : 'БЕЗОПАСНО');
    document.getElementById('badgeSonar').className = f <= state.brakeThreshCm ? 'badge err' : (f < 80 ? 'badge warn' : 'badge');
    document.getElementById('subSonarRear').textContent = `Задний датчик: ${r} см`;
    document.getElementById('barSonar').style.width = `${Math.min(100, (f / 350) * 100)}%`;
    document.getElementById('barSonar').style.background = sonarColor;
  } else {
    document.getElementById('radarFrontLabel').textContent = 'Сонар: НЕТ СИГНАЛА';
    document.getElementById('valSonar').innerHTML = `-- <small style="font-size:14px;">см</small>`;
    document.getElementById('badgeSonar').textContent = 'НЕТ СИГНАЛА';
    document.getElementById('badgeSonar').className = 'badge err';
    document.getElementById('subSonarRear').textContent = 'Проверьте пины HC-SR04';
  }

  if (tiltOk) {
    document.getElementById('valTilt').textContent = `${state.tiltDeg.toFixed(1)}°`;
    const rollover = state.tiltDeg >= state.rolloverDeg;
    const danger = state.tiltDeg > state.tiltThreshDeg;
    document.getElementById('badgeTilt').textContent = rollover ? 'ОПРОКИДЫВАНИЕ' : (danger ? 'КРУТОЙ УКЛОН' : 'НОРМА');
    document.getElementById('badgeTilt').className = rollover ? 'badge err' : (danger ? 'badge warn' : 'badge');
    document.getElementById('subTilt').textContent = rollover ? 'Отправлен сигнал SOS!' : (danger ? 'Скорость ограничена автоматикой' : 'Уклон пандуса в норме');
    document.getElementById('barTilt').style.width = `${Math.min(100, (state.tiltDeg / 28) * 100)}%`;
  } else {
    document.getElementById('valTilt').textContent = `--°`;
    document.getElementById('badgeTilt').textContent = 'НЕТ СИГНАЛА';
    document.getElementById('badgeTilt').className = 'badge err';
    document.getElementById('subTilt').textContent = 'Проверьте пины MPU6050';
  }

  document.getElementById('statDist').textContent = `${Math.round(state.distanceMeters)} м`;
  document.getElementById('statBrakes').textContent = state.autoBrakesCount;

  if (document.getElementById('tab-cockpit').classList.contains('active')) {
    drawTrajectory();
  }
}

/* ==================== 10. 2D OBSTACLE, PEDESTRIAN & TRAJECTORY MAP ==================== */
function drawTrajectory() {
  const canvas = document.getElementById('trajectoryCanvas');
  if (!canvas) return;
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  if (rect.width === 0) return;
  if (canvas.width !== Math.round(rect.width * dpr)) {
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.save();
  ctx.scale(dpr, dpr);

  const w = rect.width;
  const h = rect.height;
  const cx = w / 2;
  const cy = h / 2;
  ctx.clearRect(0, 0, w, h);

  const viewPx = Math.min(cx, cy) - 14;
  const scale = viewPx / state.mapViewRadiusM;

  // Metric grid
  const stepM = state.mapViewRadiusM <= 10 ? 1 : 10;
  ctx.strokeStyle = 'rgba(142, 146, 155, 0.1)';
  ctx.lineWidth = 1;
  for (let m = -state.mapViewRadiusM; m <= state.mapViewRadiusM; m += stepM) {
    const gx = cx + m * scale;
    const gy = cy + m * scale;
    ctx.beginPath();
    ctx.moveTo(gx, 0);
    ctx.lineTo(gx, h);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, gy);
    ctx.lineTo(w, gy);
    ctx.stroke();
  }

  // Geofence circle (50m)
  const fencePx = state.geofenceRadiusM * scale;
  ctx.beginPath();
  ctx.arc(cx, cy, fencePx, 0, Math.PI * 2);
  ctx.strokeStyle = state.geofenceAlerted ? 'rgba(255, 59, 48, 0.55)' : 'rgba(0, 229, 153, 0.25)';
  ctx.setLineDash([4, 4]);
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.setLineDash([]);

  // Home base dot
  ctx.fillStyle = '#8E929B';
  ctx.beginPath();
  ctx.arc(cx, cy, 3, 0, Math.PI * 2);
  ctx.fill();

  // Fixed 2D World Obstacles
  for (const obs of state.obstacles) {
    const ox = cx + obs.x * scale;
    const oy = cy + obs.y * scale;
    const or = Math.max(4, obs.r * scale);

    ctx.fillStyle = 'rgba(255, 159, 28, 0.22)';
    ctx.strokeStyle = '#FF9F1C';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(ox, oy, or, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    if (state.mapViewRadiusM <= 12) {
      ctx.font = '10px monospace';
      ctx.fillStyle = '#F4F4F5';
      ctx.textAlign = 'center';
      ctx.fillText(obs.label, ox, oy - or - 3);
    }
  }

  // Walking Pedestrians
  for (const ped of state.pedestrians) {
    const px = cx + ped.x * scale;
    const py = cy + ped.y * scale;
    const pr = Math.max(4, ped.r * scale);
    const isKnocked = ped.knockedTimer > 0;

    ctx.fillStyle = isKnocked ? 'rgba(255, 59, 48, 0.35)' : 'rgba(168, 85, 247, 0.26)';
    ctx.strokeStyle = isKnocked ? '#FF3B30' : '#A855F7';
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.arc(px, py, pr, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    if (isKnocked) {
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(ped.spinAngle);
      ctx.strokeStyle = '#FCA5A5';
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(-4, -4);
      ctx.lineTo(4, 4);
      ctx.moveTo(4, -4);
      ctx.lineTo(-4, 4);
      ctx.stroke();
      ctx.restore();
    } else {
      ctx.fillStyle = '#E9D5FF';
      ctx.beginPath();
      ctx.arc(px, py, 2.2, 0, Math.PI * 2);
      ctx.fill();

      ctx.strokeStyle = '#E9D5FF';
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px + Math.sin(ped.headingRad) * (pr + 4), py - Math.cos(ped.headingRad) * (pr + 4));
      ctx.stroke();
    }

    if (state.mapViewRadiusM <= 12) {
      ctx.font = '10px monospace';
      ctx.fillStyle = isKnocked ? '#FCA5A5' : '#D8B4FE';
      ctx.textAlign = 'center';
      ctx.fillText(isKnocked ? 'Сбит!' : ped.label, px, py - pr - 3);
    }
  }
  ctx.textAlign = 'left';

  // Path trail
  if (state.trackPoints.length > 1) {
    ctx.beginPath();
    ctx.strokeStyle = '#38BDF8';
    ctx.lineWidth = 2;
    state.trackPoints.forEach((pt, idx) => {
      const px = cx + pt.x * scale;
      const py = cy + pt.y * scale;
      if (idx === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }

  // Wheelchair position & live Front Ultrasonic Beam Cone
  const curX = cx + state.posX * scale;
  const curY = cy + state.posY * scale;
  const rad = (state.headingDeg * Math.PI) / 180;

  if (isModuleConnected('sonar')) {
    const beamLenPx = Math.min(3.5, state.frontDistCm / 100) * scale;
    const coneColor = state.frontDistCm <= state.brakeThreshCm
      ? 'rgba(255, 59, 48, 0.28)'
      : (state.frontDistCm < 80 ? 'rgba(255, 159, 28, 0.22)' : 'rgba(0, 229, 153, 0.14)');
    ctx.save();
    ctx.translate(curX, curY);
    ctx.rotate(rad);
    ctx.fillStyle = coneColor;
    ctx.beginPath();
    ctx.moveTo(0, -4);
    ctx.arc(0, -4, beamLenPx, -Math.PI / 2 - 0.26, -Math.PI / 2 + 0.26);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  // Wheelchair arrow marker
  ctx.save();
  ctx.translate(curX, curY);
  ctx.rotate(rad);
  ctx.fillStyle = state.pedestrianOverride ? '#FF9F1C' : (state.geofenceAlerted ? '#FF3B30' : '#00E599');
  ctx.beginPath();
  ctx.moveTo(0, -8);
  ctx.lineTo(5.5, 5.5);
  ctx.lineTo(0, 2.5);
  ctx.lineTo(-5.5, 5.5);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  // HUD readout
  const distHome = Math.hypot(state.posX, state.posY).toFixed(1);
  ctx.font = '10px monospace';
  ctx.fillStyle = state.pedestrianOverride ? '#FCA5A5' : '#8E929B';
  const hudText = state.pedestrianOverride
    ? `От базы: ${distHome} м • Сбито: ${state.pedestriansHitCount} (☠️ ON)`
    : `От базы: ${distHome} м • Клик: +/- препятствие`;
  ctx.fillText(hudText, 8, h - 8);
  ctx.restore();
}

let lastFrame = performance.now();
function animLoop(now) {
  const dt = Math.min(0.1, (now - lastFrame) / 1000);
  lastFrame = now;
  stepSimulation(dt);
  requestAnimationFrame(animLoop);
}
requestAnimationFrame(animLoop);

/* ==================== 11. CHART, JSON, CSV & CLOUD DB ==================== */
function buildTelemetryPayload(eventLabel = 'Телеметрия') {
  return {
    device_id: 'esp32-wheelchair-01',
    timestamp: new Date().toISOString(),
    mode: state.mode,
    headlight: state.headlight,
    speed_kmh: +Math.abs(state.speedKmh).toFixed(2),
    battery_pct: Math.round(state.batteryPct),
    front_obstacle_cm: Math.round(state.frontDistCm),
    tilt_deg: +state.tiltDeg.toFixed(1),
    event: eventLabel
  };
}

function drawChart() {
  const canvas = document.getElementById('telemetryChart');
  if (!canvas) return;
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  if (rect.width === 0) return;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  const ctx = canvas.getContext('2d');
  ctx.scale(dpr, dpr);

  const w = rect.width;
  const h = rect.height;
  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = 'rgba(142, 146, 155, 0.2)';
  ctx.lineWidth = 1;
  for (let i = 1; i <= 4; i++) {
    const y = (h / 5) * i;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  const pts = state.chartHistory;
  if (pts.length < 2) return;

  ctx.strokeStyle = '#00E599';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  pts.forEach((p, idx) => {
    const x = (idx / (pts.length - 1)) * w;
    const y = h - 20 - (p.speed / 8) * (h - 40);
    if (idx === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  ctx.strokeStyle = '#FF9F1C';
  ctx.lineWidth = 2;
  ctx.beginPath();
  pts.forEach((p, idx) => {
    const x = (idx / (pts.length - 1)) * w;
    const y = h - 20 - (p.front / 350) * (h - 40);
    if (idx === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  ctx.font = '11px monospace';
  ctx.fillStyle = '#00E599';
  ctx.fillText('● Скорость (км/ч)', 10, 16);
  ctx.fillStyle = '#FF9F1C';
  ctx.fillText('● Дистанция сонара (см)', 140, 16);
}

function renderDbTable() {
  const tbody = document.getElementById('dbTableBody');
  tbody.innerHTML = '';
  state.dbRecords.slice(0, 12).forEach(rec => {
    const tr = document.createElement('tr');
    const time = new Date(rec.timestamp).toLocaleTimeString('ru-RU');
    const isCrit = rec.event === 'АВТОТОРМОЗ' || rec.event === 'SOS_ТРЕВОГА' || rec.event === 'НАЕЗД_ТЕСТ';
    tr.innerHTML = `
      <td>${time}</td>
      <td>${MODE_NAMES[rec.mode] || rec.mode}</td>
      <td>${rec.speed_kmh} км/ч</td>
      <td>${rec.battery_pct}%</td>
      <td>${rec.front_obstacle_cm} см</td>
      <td style="color:${isCrit ? '#FF3B30' : '#00E599'}">${rec.event}</td>
    `;
    tbody.appendChild(tr);
  });
  document.getElementById('statRecords').textContent = state.dbRecords.length;
}

async function saveRecordToCloud(eventLabel = 'Телеметрия') {
  const payload = buildTelemetryPayload(eventLabel);
  state.dbRecords.unshift(payload);
  if (state.dbRecords.length > 50) state.dbRecords.pop();
  localStorage.setItem('sw_cloud_records', JSON.stringify(state.dbRecords));
  renderDbTable();

  if (state.cloudProvider === 'supabase' && state.cloudUrl && state.cloudKey) {
    try {
      await fetch(`${state.cloudUrl.replace(/\/$/, '')}/rest/v1/telemetry_logs`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'apikey': state.cloudKey,
          'Authorization': `Bearer ${state.cloudKey}`,
          'Prefer': 'return=minimal'
        },
        body: JSON.stringify(payload)
      });
    } catch (e) {}
  } else if (state.cloudProvider === 'firebase' && state.cloudUrl) {
    try {
      await fetch(`${state.cloudUrl.replace(/\/$/, '')}/telemetry_logs.json`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
    } catch (e) {}
  }
}

setInterval(() => {
  const payload = buildTelemetryPayload();
  document.getElementById('liveJsonView').textContent = JSON.stringify(payload, null, 2);

  state.chartHistory.push({ speed: payload.speed_kmh, front: payload.front_obstacle_cm });
  if (state.chartHistory.length > 30) state.chartHistory.shift();
  if (document.getElementById('tab-analytics').classList.contains('active')) drawChart();

  if (Math.abs(state.speedKmh) > 0.2) {
    saveRecordToCloud('Движение');
  }
}, 1500);

document.getElementById('cloudProvider').addEventListener('change', (e) => {
  state.cloudProvider = e.target.value;
  const urlGrp = document.getElementById('cloudUrlGroup');
  const keyGrp = document.getElementById('cloudKeyGroup');
  if (state.cloudProvider === 'local') {
    urlGrp.style.display = 'none';
    keyGrp.style.display = 'none';
    document.getElementById('statCloudMode').textContent = 'Local Cloud';
  } else if (state.cloudProvider === 'supabase') {
    urlGrp.style.display = 'flex';
    keyGrp.style.display = 'flex';
    document.getElementById('cloudUrlLabel').textContent = 'Supabase Project URL';
    document.getElementById('statCloudMode').textContent = 'Supabase API';
  } else {
    urlGrp.style.display = 'flex';
    keyGrp.style.display = 'none';
    document.getElementById('cloudUrlLabel').textContent = 'Firebase Realtime DB URL';
    document.getElementById('statCloudMode').textContent = 'Firebase RTDB';
  }
});

document.getElementById('btnSaveCloud').addEventListener('click', () => {
  state.cloudUrl = document.getElementById('cloudUrl').value.trim();
  state.cloudKey = document.getElementById('cloudKey').value.trim();
  saveRecordToCloud('Ручная синхронизация');
  showToast('Данные сохранены и отправлены в облако', 'info');
  addLog(`Запись отправлена в облачную БД (${state.cloudProvider})`, 'info');
});

document.getElementById('btnExportCsv').addEventListener('click', () => {
  const headers = ['timestamp', 'mode', 'speed_kmh', 'battery_pct', 'front_obstacle_cm', 'tilt_deg', 'event'];
  const rows = state.dbRecords.map(r => headers.map(h => JSON.stringify(r[h] ?? '')).join(';'));
  const csv = '\uFEFF' + [headers.join(';'), ...rows].join('\r\n');
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = 'smart_wheelchair_telemetry.csv';
  a.click();
  URL.revokeObjectURL(url);
  showToast('Файл smart_wheelchair_telemetry.csv скачан', 'info');
});

document.getElementById('btnClearDb').addEventListener('click', () => {
  state.dbRecords = [];
  localStorage.removeItem('sw_cloud_records');
  renderDbTable();
  showToast('История БД очищена', 'warn');
});

document.getElementById('btnCopyJson').addEventListener('click', () => {
  navigator.clipboard.writeText(document.getElementById('liveJsonView').textContent);
  showToast('JSON-пакет скопирован в буфер обмена', 'info');
});

/* ==================== 12. INITIALIZATION ==================== */
try {
  const saved = JSON.parse(localStorage.getItem('sw_cloud_records') || '[]');
  if (Array.isArray(saved)) state.dbRecords = saved;
} catch (e) {}

updateSensorsFrom2DWorld();
syncSimSlidersUI();
renderWiring();
updateModuleStatuses();
drawTrajectory();
saveRecordToCloud('Запуск системы');
addLog('Бортовой контроллер ESP32 запущен. Все 4 датчика онлайн.', 'info');