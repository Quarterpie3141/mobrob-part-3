const socket = io();

const statusEl = document.getElementById('connection-status');
const logEl = document.getElementById('status-log');
const canvas = document.getElementById('minimap');
const ctx = canvas.getContext('2d');

let robotPose = { x: 0, y: 0, theta: 0 };
const trail = [];
const MAX_TRAIL = 200;
const SCALE = 20;

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
  // 'data' field arrives as ArrayBuffer because we sent raw bytes
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

socket.on('robot_pose', (data) => {
  robotPose = data;
  document.getElementById('pose-x').textContent = data.x.toFixed(2);
  document.getElementById('pose-y').textContent = data.y.toFixed(2);
  document.getElementById('pose-theta').textContent = data.theta.toFixed(2);
  trail.push({ x: data.x, y: data.y });
  if (trail.length > MAX_TRAIL) trail.shift();
  drawMinimap();
});

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

// ---------- Phase Switch ----------
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

// ---------- Pause ----------
const pauseBtn = document.getElementById('pause-btn');
pauseBtn.addEventListener('click', () => {
  socket.emit('toggle_pause');
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
  // Available
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

  // Selected (ordered)
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
  socket.emit('send_waypoints', { sequence: selectedSequence });
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
  // Pre-render the costmap into an offscreen canvas for fast blitting
  const offscreen = document.createElement('canvas');
  offscreen.width = w;
  offscreen.height = h;
  const octx = offscreen.getContext('2d');
  const img = octx.createImageData(w, h);

  for (let i = 0; i < cells.length; i++) {
    const v = cells[i];
    let r, g, b, a;
    if (v < 0) {
      // Unknown - dark gray
      r = g = b = 40; a = 255;
    } else if (v === 0) {
      // Free - black
      r = g = b = 0; a = 255;
    } else if (v >= 100) {
      // Occupied - pure white
      r = g = b = 255; a = 255;
    } else {
      // Cost gradient - black to white
      const c = Math.round((v / 100) * 255);
      r = g = b = c; a = 255;
    }

    // OccupancyGrid is stored row-major, bottom-up in ROS convention
    // We need to flip vertically when drawing
    const px = i % w;
    const py = h - 1 - Math.floor(i / w);  // flip Y
    const idx = (py * w + px) * 4;
    img.data[idx]     = r;
    img.data[idx + 1] = g;
    img.data[idx + 2] = b;
    img.data[idx + 3] = a;
  }

  octx.putImageData(img, 0, 0);
  costmapImage = offscreen;
}

function drawMinimap() {
  const w = canvas.width, h = canvas.height;
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, w, h);

  if (costmap && costmapImage) {
    // Scale the costmap to fit the canvas
    ctx.imageSmoothingEnabled = false;  // pixelated look
    ctx.drawImage(costmapImage, 0, 0, w, h);

    // Convert robot world coords to costmap pixel coords
    const { resolution, origin_x, origin_y, width: cw, height: ch } = costmap;
    const cellX = (robotPose.x - origin_x) / resolution;
    const cellY = (robotPose.y - origin_y) / resolution;

    // Then to canvas coords (with vertical flip)
    const px = (cellX / cw) * w;
    const py = h - (cellY / ch) * h;

    // Robot - bright marker
    ctx.fillStyle = '#fff';
    ctx.fillRect(px - 4, py - 4, 8, 8);
    ctx.strokeStyle = '#000';
    ctx.lineWidth = 1;
    ctx.strokeRect(px - 4, py - 4, 8, 8);

    // Heading arrow
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(px + Math.cos(robotPose.theta) * 15,
               py - Math.sin(robotPose.theta) * 15);
    ctx.stroke();
  } else {
    // No costmap yet - show waiting message
    ctx.fillStyle = '#666';
    ctx.font = '14px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('// AWAITING COSTMAP', w / 2, h / 2);
  }
}
drawMinimap();
renderWaypoints();