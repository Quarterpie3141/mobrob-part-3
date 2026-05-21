const socket = io();

const statusEl = document.getElementById('connection-status');
const logEl = document.getElementById('status-log');
const canvas = document.getElementById('minimap');
const ctx = canvas.getContext('2d');
const scrub      = document.getElementById('replay-scrub');
const posLabel   = document.getElementById('replay-pos');
const toggleBtn  = document.getElementById('replay-toggle');s
const playBtn    = document.getElementById('replay-play');
const speedSel   = document.getElementById('replay-speed');

let robotPose = { x: 0, y: 0, theta: 0 };
let currentGoal = null;
let pois = [];                // Array of { x, y, phi }
let classifiedPois = [];      // Array of { x, y, phi, label }  -- letters
let objectPois = [];          // Array of { x, y, phi, label }  -- objects
const poiImages = {};         // letter label -> base64 jpeg
const objectImages = {};      // object label -> base64 jpeg
let costmap = null;


// replayability stuff
const history = [];                 // { t, type, data }
const MAX_HISTORY = 50000;
let recording = true;
let replayMode = false;
let replayIndex = 0;                // current event index in history
let replayTimer = null;
let replaySpeed = 1.0;

const live = {
  robotPose: { x: 0, y: 0, theta: 0 },
  currentGoal: null,
  pois: [],
  classifiedPois: [],
  objectPois: [],
  trail: [],
};

const SCALE = 20;
const trail = [];
const MAX_TRAIL = 200;

let currentPhase = 1;
let isPaused = false;
let availableWaypoints = [];
let selectedSequence = [];

// ---------- Socket Events ----------
socket.on('connect', () => {
  statusEl.textContent = 'Connected';
  statusEl.className = 'connected';
  addLog('Connected to ROS 2 server', 'success');
});

socket.on('costmap', (data) => {
  costmap = data;
  const cells = new Int8Array(data.data);
  buildCostmapImage(cells, data.width, data.height);
  drawMinimap();
});

socket.on('disconnect', () => {
  statusEl.textContent = 'Disconnected';
  statusEl.className = 'disconnected';
  addLog('Disconnected from server', 'error');
});

socket.on('status_log', (data) => addLog(data.message, data.level));

socket.on('state_sync', (data) => {
  currentPhase = data.phase;
  isPaused = data.paused;
  availableWaypoints = data.waypoints || [];
  selectedSequence = data.sequence || [];
  setPhase(currentPhase, false);
  updatePauseButton();
  renderWaypoints();
});

socket.on('pause_state', (data) => {
  isPaused = data.paused;
  updatePauseButton();
});

socket.on('nav2_goal', (data) => {
  record('nav2_goal', data);
  if (replayMode) return;
  currentGoal = data;
  drawMinimap();
});

socket.on('poi_update', (data) => {
  try {
    const parsed = JSON.parse(data.poi.replace(/'/g, '"'));
    record('poi_update', parsed);
    if (replayMode) return;
    pois = parsed;
    addLog(`POIs updated: ${pois.length} object(s) detected`, 'info');
    drawMinimap();
  } catch (e) { console.error('Failed to parse POIs:', e); }
});

socket.on('robot_pose', (data) => {
  record('robot_pose', data);
  if (replayMode) return;          // ignore live updates while replay
  applyRobotPose(data);
});

socket.on('classified_poi_update', (data) => {
  try {
    const parsed = JSON.parse(data.classified_poi.replace(/'/g, '"'));
    record('classified_poi_update', parsed);
    if (replayMode) return;
    classifiedPois = parsed;
    addLog(`Classified POIs updated: ${classifiedPois.length} object(s)`, 'info');
    drawMinimap();
  } catch (e) { console.error('Failed to parse classified POIs:', e); }
});
socket.on('poi_image', (data) => {
  poiImages[data.label] = data.image;
});

// Object detections
socket.on('object_poi_update', (data) => {
  try {
    const parsed = JSON.parse(data.object_poi.replace(/'/g, '"'));
    record('object_poi_update', parsed);
    if (replayMode) return;
    objectPois = parsed;
    addLog(`Object detections updated: ${objectPois.length}`, 'info');
    drawMinimap();
  } catch (e) { console.error('Failed to parse object POIs:', e); }
});

socket.on('object_image', (data) => {
  objectImages[data.label] = data.image;
});

//  Phase Switch 
document.querySelectorAll('.phase-btn').forEach((btn) => {
  btn.addEventListener('click', () => {
    const phase = parseInt(btn.dataset.phase, 10);
    setPhase(phase, true);
  });
});

function setPhase(phase, emit) {
  currentPhase = phase;
  document.querySelectorAll('.phase-btn').forEach((b) => {
    b.classList.toggle('active', parseInt(b.dataset.phase, 10) === phase);
  });
  document.getElementById('phase-1-controls').classList.toggle('active', phase === 1);
  document.getElementById('phase-2-controls').classList.toggle('active', phase === 2);

  if (emit) {
    socket.emit('set_phase', { phase });
    addLog(`Switched to Phase ${phase}`, 'info');
  }
}

//  Pause 
const pauseBtn = document.getElementById('pause-btn');
pauseBtn.addEventListener('click', () => {
  socket.emit('toggle_pause');
});

// replayability 
function record(type, data) {
  if (!recording || replayMode) return;
  history.push({ t: Date.now(), type, data });
  if (history.length > MAX_HISTORY) history.shift();
  updateScrubberMax();
}

function applyRobotPose(data) {
  robotPose = data;
  document.getElementById('pose-x').textContent = data.x.toFixed(2);
  document.getElementById('pose-y').textContent = data.y.toFixed(2);
  document.getElementById('pose-theta').textContent = data.theta.toFixed(2);
  trail.push({ x: data.x, y: data.y });
  if (trail.length > MAX_TRAIL) trail.shift();
  drawMinimap();
}

function rebuildStateUpTo(idx) {
  // Reset
  robotPose = { x: 0, y: 0, theta: 0 };
  currentGoal = null;
  pois = [];
  classifiedPois = [];
  objectPois = [];
  trail.length = 0;

  for (let i = 0; i <= idx && i < history.length; i++) {
    const ev = history[i];
    switch (ev.type) {
      case 'robot_pose':
        robotPose = ev.data;
        trail.push({ x: ev.data.x, y: ev.data.y });
        if (trail.length > MAX_TRAIL) trail.shift();
        break;
      case 'nav2_goal':           currentGoal = ev.data; break;
      case 'poi_update':          pois = ev.data; break;
      case 'classified_poi_update': classifiedPois = ev.data; break;
      case 'object_poi_update':   objectPois = ev.data; break;
    }
  }

  // Reflect in UI
  document.getElementById('pose-x').textContent = robotPose.x.toFixed(2);
  document.getElementById('pose-y').textContent = robotPose.y.toFixed(2);
  document.getElementById('pose-theta').textContent = robotPose.theta.toFixed(2);
  drawMinimap();
}

const scrub      = document.getElementById('replay-scrub');
const posLabel   = document.getElementById('replay-pos');
const toggleBtn  = document.getElementById('replay-toggle');
const playBtn    = document.getElementById('replay-play');
const speedSel   = document.getElementById('replay-speed');

function updateScrubberMax() {
  scrub.max = Math.max(0, history.length - 1);
  if (!replayMode) {
    posLabel.textContent = `live (${history.length})`;
  }
}

toggleBtn.addEventListener('click', () => {
  if (!replayMode) enterReplay(); else exitReplay();
});

function enterReplay() {
  if (history.length === 0) {
    addLog('No history to replay yet', 'warning');
    return;
  }
  // Snapshot live state
  live.robotPose = { ...robotPose };
  live.currentGoal = currentGoal ? { ...currentGoal } : null;
  live.pois = pois.slice();
  live.classifiedPois = classifiedPois.slice();
  live.objectPois = objectPois.slice();
  live.trail = trail.slice();

  replayMode = true;
  replayIndex = history.length - 1;
  scrub.value = replayIndex;
  toggleBtn.textContent = '⏏ EXIT REPLAY';
  toggleBtn.classList.add('paused');
  rebuildStateUpTo(replayIndex);
  posLabel.textContent = `${replayIndex} / ${history.length - 1}`;
  addLog('Entered replay mode', 'info');
}

function exitReplay() {
  stopPlayback();
  replayMode = false;
  toggleBtn.textContent = '⏮ REPLAY';
  toggleBtn.classList.remove('paused');

  // Restore live state
  robotPose = live.robotPose;
  currentGoal = live.currentGoal;
  pois = live.pois;
  classifiedPois = live.classifiedPois;
  objectPois = live.objectPois;
  trail.length = 0;
  live.trail.forEach(p => trail.push(p));

  document.getElementById('pose-x').textContent = robotPose.x.toFixed(2);
  document.getElementById('pose-y').textContent = robotPose.y.toFixed(2);
  document.getElementById('pose-theta').textContent = robotPose.theta.toFixed(2);
  drawMinimap();
  updateScrubberMax();
  addLog('Resumed live mode', 'info');
}

scrub.addEventListener('input', () => {
  if (!replayMode) return;
  replayIndex = parseInt(scrub.value, 10);
  rebuildStateUpTo(replayIndex);
  posLabel.textContent = `${replayIndex} / ${history.length - 1}`;
});

speedSel.addEventListener('change', () => {
  replaySpeed = parseFloat(speedSel.value);
  if (replayTimer) { stopPlayback(); startPlayback(); }
});

playBtn.addEventListener('click', () => {
  if (!replayMode) return;
  if (replayTimer) stopPlayback();
  else startPlayback();
});

function startPlayback() {
  if (replayIndex >= history.length - 1) replayIndex = 0;
  playBtn.textContent = '⏸';
  // Use real timestamps between events for natural pacing
  const tick = () => {
    if (replayIndex >= history.length - 1) { stopPlayback(); return; }
    const dt = history[replayIndex + 1].t - history[replayIndex].t;
    replayIndex++;
    scrub.value = replayIndex;
    posLabel.textContent = `${replayIndex} / ${history.length - 1}`;
    rebuildStateUpTo(replayIndex);
    replayTimer = setTimeout(tick, Math.max(10, dt / replaySpeed));
  };
  tick();
}

function stopPlayback() {
  if (replayTimer) { clearTimeout(replayTimer); replayTimer = null; }
  playBtn.textContent = '▶';
}

//  Hover / Click on minimap POIs 
function poiToCanvas(poi) {
  const { resolution, origin_x, origin_y, width: cw, height: ch } = costmap;
  const ppx = ((poi.x - origin_x) / resolution / cw) * canvas.width;
  const ppy = canvas.height - ((poi.y - origin_y) / resolution / ch) * canvas.height;
  return { ppx, ppy };
}

canvas.addEventListener('mousemove', (e) => {
  if (!costmap) { canvas.style.cursor = 'default'; return; }
  const rect = canvas.getBoundingClientRect();
  const mx = (e.clientX - rect.left) * (canvas.width / rect.width);
  const my = (e.clientY - rect.top) * (canvas.height / rect.height);

  const hit = (poi) => {
    const { ppx, ppy } = poiToCanvas(poi);
    return Math.hypot(mx - ppx, my - ppy) < 12;
  };

  const hover = classifiedPois.some(hit) || objectPois.some(hit);
  canvas.style.cursor = hover ? 'pointer' : 'default';
});

canvas.addEventListener('click', (e) => {
  if (!costmap) return;
  const rect = canvas.getBoundingClientRect();
  const clickX = (e.clientX - rect.left) * (canvas.width / rect.width);
  const clickY = (e.clientY - rect.top) * (canvas.height / rect.height);

  const hit = (poi) => {
    const { ppx, ppy } = poiToCanvas(poi);
    return Math.hypot(clickX - ppx, clickY - ppy) < 12;
  };

  for (const poi of classifiedPois) {
    if (hit(poi)) { showPoiPopup(poi, 'letter'); return; }
  }
  for (const poi of objectPois) {
    if (hit(poi)) { showPoiPopup(poi, 'object'); return; }
  }
});

function showPoiPopup(poi, kind) {
  const modal  = document.getElementById('poi-modal');
  const img    = document.getElementById('modal-image');
  const label  = document.getElementById('modal-label');
  const coords = document.getElementById('modal-coords');

  const prefix = kind === 'object' ? 'OBJECT' : 'LETTER';
  label.textContent = `${prefix}: ${poi.label}`;
  coords.textContent = `x: ${poi.x.toFixed(2)}  y: ${poi.y.toFixed(2)}  φ: ${poi.phi.toFixed(2)}`;

  const b64 = kind === 'object' ? objectImages[poi.label] : poiImages[poi.label];
  img.src = b64 ? `data:image/jpeg;base64,${b64}` : '';
  img.style.display = b64 ? 'block' : 'none';
  modal.classList.remove('hidden');
}

document.getElementById('modal-close').addEventListener('click', () => {
  document.getElementById('poi-modal').classList.add('hidden');
});

function updatePauseButton() {
  if (isPaused) {
    pauseBtn.textContent = '▶ RESUME';
    pauseBtn.classList.add('paused');
  } else {
    pauseBtn.textContent = '⏸ PAUSE';
    pauseBtn.classList.remove('paused');
  }
}

// ---------- Waypoints ----------
function renderWaypoints() {
  const availEl = document.getElementById('available-waypoints');
  availEl.innerHTML = '';
  availableWaypoints.forEach((wp) => {
    const chip = document.createElement('div');
    chip.className = 'wp-chip';
    chip.textContent = wp;
    if (selectedSequence.includes(wp)) chip.classList.add('used');
    chip.addEventListener('click', () => {
      if (!selectedSequence.includes(wp)) {
        selectedSequence.push(wp);
        renderWaypoints();
      }
    });
    availEl.appendChild(chip);
  });

  const selEl = document.getElementById('selected-waypoints');
  selEl.innerHTML = '';
  if (selectedSequence.length === 0) {
    const hint = document.createElement('p');
    hint.className = 'hint';
    hint.textContent = 'No waypoints selected. Click letters on the left.';
    selEl.appendChild(hint);
    return;
  }
  selectedSequence.forEach((wp, idx) => {
    const row = document.createElement('div');
    row.className = 'wp-row';
    row.innerHTML = `
      <span class="order">${idx + 1}.</span>
      <span class="name">${wp}</span>
      <button class="up" title="Move up">▲</button>
      <button class="down" title="Move down">▼</button>
      <button class="remove" title="Remove">✕</button>
    `;
    row.querySelector('.up').addEventListener('click', () => moveItem(idx, -1));
    row.querySelector('.down').addEventListener('click', () => moveItem(idx, 1));
    row.querySelector('.remove').addEventListener('click', () => {
      selectedSequence.splice(idx, 1);
      renderWaypoints();
    });
    selEl.appendChild(row);
  });
}

function moveItem(idx, dir) {
  const newIdx = idx + dir;
  if (newIdx < 0 || newIdx >= selectedSequence.length) return;
  [selectedSequence[idx], selectedSequence[newIdx]] =
    [selectedSequence[newIdx], selectedSequence[idx]];
  renderWaypoints();
}

document.getElementById('send-waypoints').addEventListener('click', () => {
  if (selectedSequence.length === 0) {
    addLog('Select at least one waypoint first', 'warning');
    return;
  }

  // Map waypoint names -> [x, y, phi] from classified POIs
  const coords = [];
  const missing = [];
  for (const wp of selectedSequence) {
    const poi = classifiedPois.find(p => p.label === wp);
    if (poi) {
      coords.push([poi.x, poi.y, poi.phi]);
    } else {
      missing.push(wp);
    }
  }

  if (missing.length > 0) {
    addLog(`No classified POI found for: ${missing.join(', ')}`, 'error');
    return;
  }

  socket.emit('send_waypoints', {
    sequence: selectedSequence,
    coordinates: coords,
  });
});

document.getElementById('clear-waypoints').addEventListener('click', () => {
  selectedSequence = [];
  renderWaypoints();
  socket.emit('clear_waypoints');
});

// ---------- Log ----------
document.getElementById('clear-log').addEventListener('click', () => {
  logEl.innerHTML = '';
});

function addLog(message, level = 'info') {
  const entry = document.createElement('div');
  entry.className = `log-entry ${level}`;
  const time = new Date().toLocaleTimeString();
  entry.innerHTML = `<span class="log-time">[${time}]</span>${message}`;
  logEl.appendChild(entry);
  logEl.scrollTop = logEl.scrollHeight;
}

// ---------- Minimap ----------
function buildCostmapImage(cells, w, h) {
  const offscreen = document.createElement('canvas');
  offscreen.width = w;
  offscreen.height = h;
  const octx = offscreen.getContext('2d');
  const img = octx.createImageData(w, h);

  for (let i = 0; i < cells.length; i++) {
    const v = cells[i];
    let r, g, b, a;
    if (v < 0) {
      r = g = b = 40; a = 255;
    } else if (v === 0) {
      r = g = b = 0; a = 255;
    } else if (v >= 100) {
      r = g = b = 255; a = 255;
    } else {
      const c = Math.round((v / 100) * 255);
      r = g = b = c; a = 255;
    }

    const px = i % w;
    const py = h - 1 - Math.floor(i / w);
    const idx = (py * w + px) * 4;
    img.data[idx]     = r;
    img.data[idx + 1] = g;
    img.data[idx + 2] = b;
    img.data[idx + 3] = a;
  }

  octx.putImageData(img, 0, 0);
  costmapImage = offscreen;
}

function drawTriangle(ppx, ppy, color, label) {
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(ppx, ppy - 6);
  ctx.lineTo(ppx - 5, ppy + 4);
  ctx.lineTo(ppx + 5, ppy + 4);
  ctx.closePath();
  ctx.fill();

  ctx.fillStyle = color;
  ctx.font = 'bold 10px monospace';
  ctx.textAlign = 'left';
  ctx.fillText(label, ppx + 10, ppy + 4);
}

function drawMinimap() {
  const w = canvas.width, h = canvas.height;
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, w, h);

  if (costmap && costmapImage) {
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(costmapImage, 0, 0, w, h);

    const { resolution, origin_x, origin_y, width: cw, height: ch } = costmap;

    // render raw pois (blue circles)
    if (pois.length > 0) {
      pois.forEach((poi, idx) => {
        const pCellX = (poi.x - origin_x) / resolution;
        const pCellY = (poi.y - origin_y) / resolution;
        const ppx = (pCellX / cw) * w;
        const ppy = h - (pCellY / ch) * h;

        ctx.strokeStyle = '#5E81E0';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(ppx, ppy, 8, 0, Math.PI * 2);
        ctx.stroke();

        ctx.fillStyle = '#D25C76';
        ctx.beginPath();
        ctx.arc(ppx, ppy, 1.5, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#D25C76';
        ctx.font = 'bold 10px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`P${idx + 1}`, ppx + 10, ppy + 4);
      });
    }

    // object detections - red triangles
    objectPois.forEach((poi) => {
      const pCellX = (poi.x - origin_x) / resolution;
      const pCellY = (poi.y - origin_y) / resolution;
      const ppx = (pCellX / cw) * w;
      const ppy = h - (pCellY / ch) * h;
      drawTriangle(ppx, ppy, '#ff6b6b', poi.label);
    });

    // letter detections - green triangles
    classifiedPois.forEach((poi) => {
      const pCellX = (poi.x - origin_x) / resolution;
      const pCellY = (poi.y - origin_y) / resolution;
      const ppx = (pCellX / cw) * w;
      const ppy = h - (pCellY / ch) * h;
      drawTriangle(ppx, ppy, '#83ff83', poi.label);
    });

    // current goal
    if (currentGoal) {
      const gCellX = (currentGoal.x - origin_x) / resolution;
      const gCellY = (currentGoal.y - origin_y) / resolution;
      const gx = (gCellX / cw) * w;
      const gy = h - (gCellY / ch) * h;

      ctx.strokeStyle = '#F6D768';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(gx - 14, gy); ctx.lineTo(gx + 14, gy);
      ctx.moveTo(gx, gy - 14); ctx.lineTo(gx, gy + 14);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.strokeStyle = '#D25C76';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(gx, gy);
      ctx.lineTo(gx + Math.cos(currentGoal.phi) * 18,
                 gy - Math.sin(currentGoal.phi) * 18);
      ctx.stroke();

      ctx.strokeRect(gx - 6, gy - 6, 12, 12);

      ctx.fillStyle = '#F6D768';
      ctx.font = 'bold 11px monospace';
      ctx.textAlign = 'left';
      ctx.fillText('GOAL', gx + 10, gy - 10);
    }

    // robot pose
    const cellX = (robotPose.x - origin_x) / resolution;
    const cellY = (robotPose.y - origin_y) / resolution;
    const px = (cellX / cw) * w;
    const py = h - (cellY / ch) * h;

    ctx.strokeStyle = '#D25C76';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(px + Math.cos(robotPose.theta) * 15,
               py - Math.sin(robotPose.theta) * 15);
    ctx.stroke();

    ctx.fillStyle = '#5E81E0';
    ctx.fillRect(px - 4, py - 4, 8, 8);
    ctx.strokeStyle = '#000';
    ctx.lineWidth = 1;
    ctx.strokeRect(px - 4, py - 4, 8, 8);

    // line from robot to goal
    if (currentGoal) {
      const gCellX = (currentGoal.x - origin_x) / resolution;
      const gCellY = (currentGoal.y - origin_y) / resolution;
      const gx = (gCellX / cw) * w;
      const gy = h - (gCellY / ch) * h;

      ctx.strokeStyle = '#5b81e8b6';
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(gx, gy);
      ctx.setLineDash([]);
      ctx.stroke();
    }

  } else {
    ctx.fillStyle = '#342C3F';
    ctx.font = '14px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('// AWAITING COSTMAP', w / 2, h / 2);
  }
}

drawMinimap();
renderWaypoints();