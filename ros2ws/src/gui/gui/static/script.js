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
function drawMinimap() {
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = '#313244';
  ctx.lineWidth = 1;
  for (let i = 0; i <= w; i += SCALE) {
    ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, h); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(w, i); ctx.stroke();
  }
  ctx.strokeStyle = '#585b70';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(w/2, 0); ctx.lineTo(w/2, h); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, h/2); ctx.lineTo(w, h/2); ctx.stroke();

  if (trail.length > 1) {
    ctx.strokeStyle = '#89b4fa';
    ctx.lineWidth = 2;
    ctx.beginPath();
    trail.forEach((p, i) => {
      const px = w/2 + p.x * SCALE;
      const py = h/2 - p.y * SCALE;
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }

  const rx = w/2 + robotPose.x * SCALE;
  const ry = h/2 - robotPose.y * SCALE;
  ctx.fillStyle = '#a6e3a1';
  ctx.beginPath();
  ctx.arc(rx, ry, 8, 0, 2 * Math.PI);
  ctx.fill();
}

drawMinimap();
renderWaypoints();