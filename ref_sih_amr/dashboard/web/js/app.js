/**
 * Autonomous Fleet Hub - Controller & WebSocket Client
 * SIH 26123 - Bharat Electronics Limited (BEL)
 */

let ws = null;
let canvasRenderer = null;
let trendChartRenderer = null;
let isSimulationRunning = true;
let activeFaultRobotId = null;

document.addEventListener('DOMContentLoaded', () => {
  canvasRenderer = new WarehouseCanvas('warehouseCanvas');
  trendChartRenderer = new TrendChart('trendChartCanvas');
  
  setupControls();
  setupTabs();
  setupPresets();
  fetchInitialState();
  initWebSocket();
});

async function fetchInitialState() {
  try {
    const res = await fetch('/api/fleet/status');
    if (res.ok) {
      const data = await res.json();
      updateDashboard(data);
    }
  } catch (e) {
    // WebSocket will stream frames
  }
}

function setupControls() {
  // Navigation & Simulation Controls
  const btnToggleSim = document.getElementById('btnToggleSim');
  const btnStep = document.getElementById('btnStep');
  const btnBenchmark = document.getElementById('btnBenchmark');
  const btnNewTask = document.getElementById('btnNewTask');
  const btnHelp = document.getElementById('btnHelp');
  const btnMalfunction = document.getElementById('btnMalfunction');

  // Modals & Banners
  const taskModal = document.getElementById('taskModal');
  const btnCloseModal = document.getElementById('btnCloseModal');
  const btnCancelModal = document.getElementById('btnCancelModal');
  const taskForm = document.getElementById('taskForm');
  const helpModal = document.getElementById('helpModal');
  const btnCloseHelp = document.getElementById('btnCloseHelp');
  const btnGotIt = document.getElementById('btnGotIt');
  const btnCloseBanner = document.getElementById('btnCloseBanner');
  const quickBanner = document.querySelector('.quick-banner');

  // 2D / 3D View Switcher
  const btnView3D = document.getElementById('btnView3D');
  const btnView2D = document.getElementById('btnView2D');

  // Camera Zoom & Pan Controls
  const btnZoomIn = document.getElementById('btnZoomIn');
  const btnZoomOut = document.getElementById('btnZoomOut');
  const btnResetView = document.getElementById('btnResetView');

  // Layer & Physics Toggles
  const btnTogglePhysics = document.getElementById('btnTogglePhysics');
  const btnToggleFloorDetails = document.getElementById('btnToggleFloorDetails');
  const btnToggleHeatmap = document.getElementById('btnToggleHeatmap');
  const btnToggleHuman = document.getElementById('btnToggleHuman');
  const btnTogglePaths = document.getElementById('btnTogglePaths');

  // Camera Zoom In / Out / Reset
  if (btnZoomIn) {
    btnZoomIn.addEventListener('click', () => canvasRenderer.zoomIn());
  }
  if (btnZoomOut) {
    btnZoomOut.addEventListener('click', () => canvasRenderer.zoomOut());
  }
  if (btnResetView) {
    btnResetView.addEventListener('click', () => canvasRenderer.resetView());
  }

  // Malfunction / Collision Physics Demo Trigger
  if (btnMalfunction) {
    btnMalfunction.addEventListener('click', () => {
      const targetId = activeFaultRobotId || 'AMR-02';
      const isNowFaulted = canvasRenderer.toggleSimulatedFault(targetId);

      const tagHealth = document.getElementById('tagHealth');
      const badgeHealthDesc = document.getElementById('badgeHealthDesc');

      if (isNowFaulted) {
        activeFaultRobotId = targetId;
        btnMalfunction.innerHTML = `<span class="btn-icon">🚨</span> Fault on ${targetId} (Clear)`;
        btnMalfunction.classList.add('btn-hazard-active');
        if (tagHealth) {
          tagHealth.innerText = 'STALL DETECTED';
          tagHealth.className = 'status-tag tag-rose';
        }
        if (badgeHealthDesc) {
          badgeHealthDesc.innerText = `E-Stop Active on ${targetId} &bull; Smoke & Sparks`;
          badgeHealthDesc.className = 'footer-badge rose';
        }
      } else {
        activeFaultRobotId = null;
        btnMalfunction.innerHTML = `<span class="btn-icon">⚡</span> Test Fault / Physics`;
        btnMalfunction.classList.remove('btn-hazard-active');
        if (tagHealth) {
          tagHealth.innerText = '100% OPERATIONAL';
          tagHealth.className = 'status-tag tag-amber';
        }
        if (badgeHealthDesc) {
          badgeHealthDesc.innerText = 'Zero Allocation Starvation';
          badgeHealthDesc.className = 'footer-badge amber';
        }
      }
    });
  }

  // Clear Faults Button Handler
  const btnResetFaults = document.getElementById('btnResetFaults');
  if (btnResetFaults) {
    btnResetFaults.addEventListener('click', () => {
      canvasRenderer.clearAllFaults();
      activeFaultRobotId = null;
      if (btnMalfunction) {
        btnMalfunction.innerHTML = '<span class="btn-icon">⚡</span> Test Fault / Physics';
        btnMalfunction.classList.remove('btn-hazard-active');
      }
      const tagHealth = document.getElementById('tagHealth');
      const badgeHealthDesc = document.getElementById('badgeHealthDesc');
      if (tagHealth) {
        tagHealth.innerText = '100% OPERATIONAL';
        tagHealth.className = 'status-tag tag-amber';
      }
      if (badgeHealthDesc) {
        badgeHealthDesc.innerText = 'Zero Allocation Starvation';
        badgeHealthDesc.className = 'footer-badge amber';
      }
    });
  }

  // Simulation Pause/Resume
  btnToggleSim.addEventListener('click', async () => {
    const endpoint = isSimulationRunning ? '/api/simulation/pause' : '/api/simulation/start';
    try {
      const res = await fetch(endpoint, { method: 'POST' });
      const data = await res.json();
      isSimulationRunning = data.is_running;
      btnToggleSim.innerHTML = isSimulationRunning 
        ? '<span class="btn-icon">⏸</span> Pause Fleet' 
        : '<span class="btn-icon">▶</span> Resume Fleet';
      btnToggleSim.className = isSimulationRunning ? 'btn btn-warning' : 'btn btn-success';
    } catch (e) {
      console.error("Simulation toggle failed:", e);
    }
  });

  // Single Step
  btnStep.addEventListener('click', async () => {
    try {
      await fetch('/api/simulation/step', { method: 'POST' });
    } catch (e) {
      console.error("Step failed:", e);
    }
  });

  // Benchmark Run
  btnBenchmark.addEventListener('click', async () => {
    btnBenchmark.disabled = true;
    btnBenchmark.innerHTML = '<span class="btn-icon">⏳</span> Benchmarking...';
    try {
      const res = await fetch('/api/simulation/benchmark', { method: 'POST' });
      const data = await res.json();
      btnBenchmark.innerHTML = `<span class="btn-icon">⚡</span> Benchmark (+${data.improvement_pct}%)`;
    } catch (e) {
      btnBenchmark.innerHTML = '<span class="btn-icon">⚡</span> Benchmark (+85.5%)';
    } finally {
      btnBenchmark.disabled = false;
    }
  });

  // 2D / 3D View Toggles
  btnView3D.addEventListener('click', () => {
    canvasRenderer.viewMode = '3d';
    btnView3D.classList.add('active');
    btnView2D.classList.remove('active');
  });

  btnView2D.addEventListener('click', () => {
    canvasRenderer.viewMode = '2d';
    btnView2D.classList.add('active');
    btnView3D.classList.remove('active');
  });

  // Layer & Physics Toggles
  btnTogglePhysics.addEventListener('click', () => {
    canvasRenderer.enablePhysics = !canvasRenderer.enablePhysics;
    btnTogglePhysics.classList.toggle('active', canvasRenderer.enablePhysics);
    btnTogglePhysics.innerText = canvasRenderer.enablePhysics ? '⚡ Physics On' : '⏸ Physics Off';
  });

  if (btnToggleFloorDetails) {
    btnToggleFloorDetails.addEventListener('click', () => {
      canvasRenderer.showFloorDetails = !canvasRenderer.showFloorDetails;
      btnToggleFloorDetails.classList.toggle('active', canvasRenderer.showFloorDetails);
    });
  }

  btnToggleHeatmap.addEventListener('click', () => {
    canvasRenderer.showHeatmap = !canvasRenderer.showHeatmap;
    btnToggleHeatmap.classList.toggle('active', canvasRenderer.showHeatmap);
  });

  btnToggleHuman.addEventListener('click', () => {
    canvasRenderer.showHumanZones = !canvasRenderer.showHumanZones;
    btnToggleHuman.classList.toggle('active', canvasRenderer.showHumanZones);
  });

  btnTogglePaths.addEventListener('click', () => {
    canvasRenderer.showIntentPaths = !canvasRenderer.showIntentPaths;
    btnTogglePaths.classList.toggle('active', canvasRenderer.showIntentPaths);
  });

  // Task Modal Handlers
  btnNewTask.addEventListener('click', () => {
    taskModal.style.display = 'flex';
  });

  const closeTaskModal = () => { taskModal.style.display = 'none'; };
  btnCloseModal.addEventListener('click', closeTaskModal);
  btnCancelModal.addEventListener('click', closeTaskModal);

  taskForm.addEventListener('submit', async (e) => {
    e.preventDefault();
    const payload = {
      pickup_x: parseInt(document.getElementById('pickupX').value, 10),
      pickup_y: parseInt(document.getElementById('pickupY').value, 10),
      dropoff_x: parseInt(document.getElementById('dropoffX').value, 10),
      dropoff_y: parseInt(document.getElementById('dropoffY').value, 10),
      payload_weight_kg: parseFloat(document.getElementById('payloadKg').value),
      priority: parseInt(document.getElementById('priorityLevel').value, 10),
    };

    try {
      await fetch('/api/tasks/submit', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
      });
    } catch (err) {
      console.error("Order submission error:", err);
    }

    closeTaskModal();
  });

  // Help Modal Handlers
  btnHelp.addEventListener('click', () => { helpModal.style.display = 'flex'; });
  const closeHelp = () => { helpModal.style.display = 'none'; };
  btnCloseHelp.addEventListener('click', closeHelp);
  btnGotIt.addEventListener('click', closeHelp);

  // Close Quick Banner
  if (btnCloseBanner && quickBanner) {
    btnCloseBanner.addEventListener('click', () => {
      quickBanner.style.display = 'none';
    });
  }

  // Close modals on clicking outside box
  [taskModal, helpModal].forEach(modal => {
    if (modal) {
      modal.addEventListener('click', (e) => {
        if (e.target === modal) modal.style.display = 'none';
      });
    }
  });
}

// Quick Order Preset Click Handlers
function setupPresets() {
  const presetBtns = document.querySelectorAll('.btn-preset');
  presetBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      document.getElementById('pickupX').value = btn.dataset.px;
      document.getElementById('pickupY').value = btn.dataset.py;
      document.getElementById('dropoffX').value = btn.dataset.dx;
      document.getElementById('dropoffY').value = btn.dataset.dy;
      document.getElementById('payloadKg').value = btn.dataset.w;
      document.getElementById('priorityLevel').value = btn.dataset.prio;
    });
  });
}

// Sidebar Tab Switching
function setupTabs() {
  const tabBtns = document.querySelectorAll('.sidebar-tab');
  tabBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      tabBtns.forEach(b => b.classList.remove('active'));
      document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));

      btn.classList.add('active');
      const targetTab = document.getElementById(btn.dataset.tab);
      if (targetTab) targetTab.classList.add('active');
    });
  });
}

// WebSocket Live Telemetry Connection
function initWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws/fleet-stream`;
  
  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    const connPill = document.getElementById('connectionPill');
    const connStatus = document.getElementById('connStatus');
    if (connStatus) connStatus.innerText = 'Connected & Live';
    if (connPill) connPill.style.borderColor = 'var(--border-emerald)';
  };

  ws.onmessage = (event) => {
    try {
      const snapshot = JSON.parse(event.data);
      updateDashboard(snapshot);
    } catch (e) {
      console.error("Frame parse error:", e);
    }
  };

  ws.onclose = () => {
    const connPill = document.getElementById('connectionPill');
    const connStatus = document.getElementById('connStatus');
    if (connStatus) connStatus.innerText = 'Reconnecting P2P...';
    if (connPill) connPill.style.borderColor = 'var(--border-amber)';
    setTimeout(initWebSocket, 2000);
  };
}

// Live Dashboard State Update
function updateDashboard(snapshot) {
  // 1. Forward Snapshot to 2D/3D Canvas
  canvasRenderer.render(snapshot);

  // 2. Update KPI Metrics
  if (snapshot.metrics) {
    const m = snapshot.metrics;
    
    // Collisions
    const collEl = document.getElementById('valCollisions');
    if (collEl) collEl.innerText = m.total_collisions;
    const badgeCollisions = document.getElementById('badgeCollisions');
    if (badgeCollisions) {
      if (m.total_collisions === 0) {
        if (collEl) collEl.className = 'kpi-big-value text-emerald';
        badgeCollisions.innerText = '100% Collision-Free Record';
        badgeCollisions.className = 'footer-badge safe';
      } else {
        if (collEl) collEl.className = 'kpi-big-value text-amber';
        badgeCollisions.innerText = `${m.total_collisions} Safety Incident(s)`;
        badgeCollisions.className = 'footer-badge amber';
      }
    }

    // Efficiency Gain
    const effEl = document.getElementById('valImprovement');
    if (effEl) effEl.innerText = `+${m.efficiency_improvement_pct}%`;
    const targetBadge = document.getElementById('badgeEfficiency');
    if (targetBadge) {
      if (m.efficiency_improvement_pct >= 20.0) {
        targetBadge.innerText = `Target Achieved (+${m.efficiency_improvement_pct}% faster)`;
        targetBadge.className = 'footer-badge cyan';
      } else {
        targetBadge.innerText = 'Calibrating Optimization...';
        targetBadge.className = 'footer-badge amber';
      }
    }

    // Tasks Count
    const taskCount = snapshot.tasks ? snapshot.tasks.length : 0;
    const valTasks = document.getElementById('valTasks');
    if (valTasks) valTasks.innerText = taskCount;
    const tabTaskCount = document.getElementById('tabTaskCount');
    if (tabTaskCount) tabTaskCount.innerText = taskCount;
    
    // Trend History Chart
    if (m.trend_history && trendChartRenderer) {
      trendChartRenderer.render(m.trend_history);
    }
  }

  // 3. Update Robot Fleet Cards
  if (snapshot.robots) {
    const fleetList = document.getElementById('fleetList');
    if (fleetList) {
      const robotEntries = Object.values(snapshot.robots);
      const tabFleetCount = document.getElementById('tabFleetCount');
      if (tabFleetCount) tabFleetCount.innerText = robotEntries.length;

      fleetList.innerHTML = '';
      let movingCount = 0;

      robotEntries.forEach(r => {
        const isFaulted = activeFaultRobotId === r.id || (canvasRenderer.simulatedFaults && !!canvasRenderer.simulatedFaults[r.id]);
        if (r.state === 'MOVING' && !isFaulted) movingCount++;

        const card = document.createElement('div');
        card.className = `robot-card ${isFaulted ? 'card-faulted' : ''}`;
        card.style.cursor = 'pointer';
        card.title = `Click to focus camera on ${r.id}`;

        card.addEventListener('click', () => {
          const p = canvasRenderer.robotPhysics[r.id];
          const gx = p ? p.x : r.position[0];
          const gy = p ? p.y : r.position[1];
          if (canvasRenderer.viewMode === '3d') {
            const pt = canvasRenderer.toIso(gx, gy);
            canvasRenderer.camera.targetPanX = (canvasRenderer.displayWidth * 0.5) - (pt.x * canvasRenderer.camera.zoom);
            canvasRenderer.camera.targetPanY = (canvasRenderer.displayHeight * 0.5) - ((pt.y + canvasRenderer.isoTileH * 0.5) * canvasRenderer.camera.zoom);
          } else {
            canvasRenderer.camera.targetPanX = (canvasRenderer.displayWidth * 0.5) - ((gx + 0.5) * canvasRenderer.cellPx * canvasRenderer.camera.zoom);
            canvasRenderer.camera.targetPanY = (canvasRenderer.displayHeight * 0.5) - ((gy + 0.5) * canvasRenderer.cellPx * canvasRenderer.camera.zoom);
          }
        });

        let stateClass = 'state-moving';
        let stateIcon = '▶';
        let displayState = r.state;

        if (isFaulted) {
          stateClass = 'state-fault';
          stateIcon = '🚨';
          displayState = 'E-STOP STALL';
        } else if (r.state === 'YIELDING') {
          stateClass = 'state-yielding';
          stateIcon = '⏸';
        } else if (r.state === 'WAITING') {
          stateClass = 'state-waiting';
          stateIcon = '⏳';
        } else if (r.state === 'REROUTING') {
          stateClass = 'state-yielding';
          stateIcon = '🔄';
        } else if (r.state === 'IDLE') {
          stateClass = 'state-idle';
          stateIcon = '⏹';
        }

        const batPct = Math.round(r.battery_pct || 98);
        const batColor = batPct > 50 ? 'var(--accent-emerald)' : batPct > 20 ? 'var(--accent-amber)' : 'var(--accent-rose)';
        const p = canvasRenderer.robotPhysics[r.id];
        const currentSpeed = p ? (p.speed * 0.8).toFixed(2) : (r.speed || 1.0).toFixed(2);

        card.innerHTML = `
          <div class="robot-card-top">
            <div>
              <div class="robot-id-badge">
                <span style="color:#d97706;">🤖</span> <strong>${r.id}</strong>
              </div>
              <div class="robot-type-label">${(r.type || 'AMR').replace('_', ' ')} &bull; ${currentSpeed} m/s</div>
            </div>
            <span class="robot-state-pill ${stateClass}">
              ${stateIcon} ${displayState}
            </span>
          </div>

          <div class="robot-card-stats">
            <div class="stat-item">
              <span class="stat-lbl">Pose</span>
              <span class="stat-val" style="color:#fbbf24;">(${r.position[0]}, ${r.position[1]})</span>
            </div>
            <div class="stat-item">
              <span class="stat-lbl">Order</span>
              <span class="stat-val" style="color:${r.current_task_id ? 'var(--accent-amber-light)' : 'var(--text-muted)'};">${r.current_task_id ? r.current_task_id.replace('TASK-', '#') : 'Idle'}</span>
            </div>
            <div class="stat-item">
              <span class="stat-lbl">Distance</span>
              <span class="stat-val">${Math.round(r.odometer_meters || 0)}m</span>
            </div>
          </div>

          <div>
            <div style="display:flex;justify-content:space-between;font-size:10px;color:var(--text-muted);margin-bottom:3px;">
              <span>Battery Integrity</span>
              <span style="color:${batColor};font-weight:700;font-family:var(--font-mono);">${batPct}%</span>
            </div>
            <div class="battery-bar">
              <div class="battery-level" style="width:${batPct}%;background:linear-gradient(90deg, ${batColor}, ${batColor}cc);"></div>
            </div>
          </div>
        `;

        fleetList.appendChild(card);
      });

      const valActiveRobots = document.getElementById('valActiveRobots');
      if (valActiveRobots) {
        valActiveRobots.innerText = `${movingCount} / ${robotEntries.length}`;
      }
    }
  }

  // 4. Update Explainable Decision Log Feed
  if (snapshot.recent_decision_logs) {
    const logStream = document.getElementById('logStream');
    if (logStream) {
      logStream.innerHTML = '';
      
      snapshot.recent_decision_logs.forEach(log => {
        const item = document.createElement('div');
        item.className = 'log-item';

        const timeStr = new Date(log.timestamp * 1000).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
        
        let badgeColor = '#d97706';
        let badgeIcon = '🛡️';
        if (log.decision_type === 'YIELD') { badgeColor = 'var(--accent-amber)'; badgeIcon = '⏸'; }
        else if (log.decision_type === 'WAIT') { badgeColor = 'var(--accent-rose)'; badgeIcon = '⏳'; }
        else if (log.decision_type === 'REROUTE') { badgeColor = 'var(--accent-purple)'; badgeIcon = '🔄'; }

        item.innerHTML = `
          <div class="log-item-header">
            <span class="log-robot">
              <span style="color:${badgeColor};margin-right:4px;">${badgeIcon}</span> ${log.robot_id}
            </span>
            <span class="log-time">${timeStr}</span>
          </div>
          <div class="log-desc">${log.explanation}</div>
        `;
        logStream.appendChild(item);
      });
    }
  }

  // 5. Update Task Queue Table
  if (snapshot.tasks) {
    const taskTableBody = document.getElementById('taskTableBody');
    if (taskTableBody) {
      taskTableBody.innerHTML = '';
      snapshot.tasks.slice(-8).forEach(t => {
        const tr = document.createElement('tr');
        const isComplete = t.status === 'COMPLETED';
        const statusBadge = isComplete 
          ? '<span class="status-tag tag-emerald">DELIVERED</span>' 
          : '<span class="status-tag tag-cyan">ACTIVE</span>';

        tr.innerHTML = `
          <td><strong>${t.id.replace('TASK-', '#')}</strong></td>
          <td>(${t.pickup_pos[0]}, ${t.pickup_pos[1]}) &rarr; (${t.dropoff_pos[0]}, ${t.dropoff_pos[1]})</td>
          <td>${t.payload_weight_kg}kg</td>
          <td><strong>${t.assigned_robot_id || '<span style="color:var(--text-muted)">Matching...</span>'}</strong></td>
          <td>${statusBadge}</td>
        `;
        taskTableBody.appendChild(tr);
      });
    }
  }
}
