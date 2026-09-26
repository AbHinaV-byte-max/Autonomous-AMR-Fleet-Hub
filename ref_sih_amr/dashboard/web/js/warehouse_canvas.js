/**
 * Warehouse Canvas Engine - Industrial-Grade Semi-Realistic 3D Digital Twin
 * Inspired by modern factory simulators (Satisfactory, Siemens Digital Twin, Amazon Kiva).
 *
 * Features:
 *  - Matte industrial polished concrete floor with specular sheen, expansion joints,
 *    and yellow/black chevron safety hazard perimeter lines.
 *  - Non-intrusive coordinate grid projection with corner crosshairs.
 *  - Solid structured industrial pallet racks: teardrop steel uprights, safety corner
 *    guards, load beams, diagonal trusses, wooden timber pallets, and textured cargo.
 *  - Realistic low-profile AMR drive units: rugged chamfered chassis in safety orange/graphite,
 *    semi-transparent top shield revealing drive wheels, suspension linkages, cooling louvers,
 *    and glowing status board; 360° LiDAR puck with dynamic ground scan fan; stereo depth cameras.
 *  - Visual physics: dynamic chassis pitch/roll inertia, recoil bounce on collisions/faults,
 *    spark bursts with gravity bounce, billowing smoke particulate, blinking red ground hazard halo,
 *    and dual-track rubber skid marks baked onto the concrete floor.
 *
 * SIH 26123 - Bharat Electronics Limited (BEL)
 */

class WarehouseCanvas {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.viewportEl = document.getElementById('canvasViewport') || this.canvas.parentElement;
    
    // View mode: '3d' (isometric) or '2d' (top-down)
    this.viewMode = '3d';
    this.enablePhysics = true;
    this.showFloorDetails = true; // Floor navigation fiducials & safety guidelines
    this.showHeatmap = true;
    this.showHumanZones = true;
    this.showIntentPaths = true;
    
    // Grid metrics (warehouse coordinates)
    this.gridWidth = 24;
    this.gridHeight = 18;
    this.cellPx = 34;
    this.origin2DX = 0;
    this.origin2DY = 0;

    // Isometric projection metrics
    this.isoTileW = 48;
    this.isoTileH = 24;
    this.isoOriginX = 0;
    this.isoOriginY = 0;

    // Interactive Camera (Pan & Zoom)
    this.camera = {
      zoom: 1.0,
      targetZoom: 1.0,
      panX: 0,
      panY: 0,
      targetPanX: 0,
      targetPanY: 0,
      isDragging: false,
      dragStartX: 0,
      dragStartY: 0,
      panStartX: 0,
      panStartY: 0,
      minZoom: 0.55,
      maxZoom: 3.0,
    };

    // Snapshot buffer
    this.latestSnapshot = null;

    // Physics state cache for each AMR (keyed by robot_id)
    this.robotPhysics = {};

    // Dynamic Physics & Visual Effects
    this.skidMarks = [];         // Deceleration & near-miss tire skid marks on floor
    this.faultParticles = [];     // Smoke puffs & electrical sparks for malfunctions
    this.collisionRipples = [];   // Ground impact shockwaves
    this.simulatedFaults = {};    // Injected malfunction states { robotId: { type, since, recoil } }

    // Ambient dust motes
    this.dustParticles = [];
    this.initDustParticles(36);

    // Mouse & Hover tracking
    this.mouse = { x: -1, y: -1, worldX: 0, worldY: 0, hoveredRobot: null };
    this.tooltipEl = document.getElementById('robotTooltip');
    this.ttHeaderEl = document.getElementById('ttHeader');
    this.ttBodyEl = document.getElementById('ttBody');

    // Setup viewport & interaction events
    this.resize();
    window.addEventListener('resize', () => this.resize());
    this.setupInteractions();

    // Start 60 FPS continuous animation loop
    this.lastTimestamp = performance.now();
    this.animate = this.animate.bind(this);
    requestAnimationFrame(this.animate);
  }

  initDustParticles(count) {
    this.dustParticles = [];
    for (let i = 0; i < count; i++) {
      this.dustParticles.push({
        x: Math.random() * (this.displayWidth || 1000),
        y: Math.random() * (this.displayHeight || 700),
        vx: (Math.random() - 0.5) * 0.22,
        vy: -0.1 - Math.random() * 0.18,
        radius: 0.7 + Math.random() * 1.3,
        alpha: 0.08 + Math.random() * 0.18,
      });
    }
  }

  resize() {
    const rect = this.viewportEl.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = rect.width * dpr;
    this.canvas.height = rect.height * dpr;
    this.ctx.scale(dpr, dpr);
    this.displayWidth = rect.width;
    this.displayHeight = rect.height;

    // Recompute 2D metrics with generous comfortable padding
    this.cellPx = Math.min(
      (this.displayWidth - 80) / this.gridWidth,
      (this.displayHeight - 80) / this.gridHeight
    );
    this.origin2DX = Math.round((this.displayWidth - this.gridWidth * this.cellPx) / 2);
    this.origin2DY = Math.round((this.displayHeight - this.gridHeight * this.cellPx) / 2);

    // Recompute 3D isometric base metrics
    const desiredIsoW = (this.gridWidth + this.gridHeight) * 0.52;
    this.isoTileW = Math.min(56, Math.max(34, (this.displayWidth - 80) / desiredIsoW));
    this.isoTileH = this.isoTileW * 0.5;

    this.isoOriginX = this.displayWidth / 2;
    this.isoOriginY = Math.max(70, (this.displayHeight - (this.gridWidth + this.gridHeight) * (this.isoTileH * 0.5)) / 2 + 15);
  }

  setupInteractions() {
    const el = this.canvas;

    // Mouse Move & Hover
    el.addEventListener('mousemove', (e) => {
      const rect = el.getBoundingClientRect();
      const rawX = e.clientX - rect.left;
      const rawY = e.clientY - rect.top;

      if (this.camera.isDragging) {
        const dx = rawX - this.camera.dragStartX;
        const dy = rawY - this.camera.dragStartY;
        this.camera.targetPanX = this.camera.panStartX + dx;
        this.camera.targetPanY = this.camera.panStartY + dy;
      } else {
        this.mouse.x = rawX;
        this.mouse.y = rawY;
        this.updateHover();
      }
    });

    // Mouse Down (Pan Start)
    el.addEventListener('mousedown', (e) => {
      if (e.button === 0) { // Left click
        this.camera.isDragging = true;
        this.camera.dragStartX = e.clientX - el.getBoundingClientRect().left;
        this.camera.dragStartY = e.clientY - el.getBoundingClientRect().top;
        this.camera.panStartX = this.camera.targetPanX;
        this.camera.panStartY = this.camera.targetPanY;
      }
    });

    // Mouse Up (Pan End)
    window.addEventListener('mouseup', () => {
      this.camera.isDragging = false;
    });

    // Mouse Leave
    el.addEventListener('mouseleave', () => {
      this.mouse.x = -1;
      this.mouse.y = -1;
      this.mouse.hoveredRobot = null;
      if (this.tooltipEl) this.tooltipEl.style.display = 'none';
    });

    // Mouse Wheel (Zoom smoothly towards mouse anchor)
    el.addEventListener('wheel', (e) => {
      e.preventDefault();
      const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
      const newZoom = Math.min(this.camera.maxZoom, Math.max(this.camera.minZoom, this.camera.targetZoom * zoomFactor));

      const rect = el.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      // Adjust pan to zoom into cursor point
      const scaleChange = newZoom - this.camera.targetZoom;
      this.camera.targetPanX -= (mouseX - this.camera.targetPanX) * (scaleChange / this.camera.targetZoom);
      this.camera.targetPanY -= (mouseY - this.camera.targetPanY) * (scaleChange / this.camera.targetZoom);

      this.camera.targetZoom = newZoom;
    }, { passive: false });

    // Touch support for tablets/laptops
    let touchStartDist = 0;
    el.addEventListener('touchstart', (e) => {
      if (e.touches.length === 1) {
        this.camera.isDragging = true;
        this.camera.dragStartX = e.touches[0].clientX;
        this.camera.dragStartY = e.touches[0].clientY;
        this.camera.panStartX = this.camera.targetPanX;
        this.camera.panStartY = this.camera.targetPanY;
      } else if (e.touches.length === 2) {
        touchStartDist = Math.hypot(
          e.touches[0].clientX - e.touches[1].clientX,
          e.touches[0].clientY - e.touches[1].clientY
        );
      }
    });

    el.addEventListener('touchmove', (e) => {
      if (e.touches.length === 1 && this.camera.isDragging) {
        const dx = e.touches[0].clientX - this.camera.dragStartX;
        const dy = e.touches[0].clientY - this.camera.dragStartY;
        this.camera.targetPanX = this.camera.panStartX + dx;
        this.camera.targetPanY = this.camera.panStartY + dy;
      } else if (e.touches.length === 2 && touchStartDist > 0) {
        const dist = Math.hypot(
          e.touches[0].clientX - e.touches[1].clientX,
          e.touches[0].clientY - e.touches[1].clientY
        );
        const factor = dist / touchStartDist;
        this.camera.targetZoom = Math.min(this.camera.maxZoom, Math.max(this.camera.minZoom, this.camera.targetZoom * factor));
        touchStartDist = dist;
      }
    });

    el.addEventListener('touchend', () => {
      this.camera.isDragging = false;
      touchStartDist = 0;
    });
  }

  zoomIn() {
    this.camera.targetZoom = Math.min(this.camera.maxZoom, this.camera.targetZoom * 1.25);
  }

  zoomOut() {
    this.camera.targetZoom = Math.max(this.camera.minZoom, this.camera.targetZoom / 1.25);
  }

  resetView() {
    this.camera.targetZoom = 1.0;
    this.camera.targetPanX = 0;
    this.camera.targetPanY = 0;
  }

  // Coordinate transforms (Model coordinates to Screen coordinates)
  toIso(gx, gy) {
    return {
      x: this.isoOriginX + (gx - gy) * (this.isoTileW * 0.5),
      y: this.isoOriginY + (gx + gy) * (this.isoTileH * 0.5),
    };
  }

  screenToWorldIso(screenX, screenY) {
    const unX = (screenX - this.camera.panX) / this.camera.zoom;
    const unY = (screenY - this.camera.panY) / this.camera.zoom;
    const relX = unX - this.isoOriginX;
    const relY = unY - this.isoOriginY;
    const halfW = this.isoTileW * 0.5;
    const halfH = this.isoTileH * 0.5;

    const gx = (relY / halfH + relX / halfW) * 0.5;
    const gy = (relY / halfH - relX / halfW) * 0.5;
    return { gx, gy };
  }

  // Hover detection with camera transformation awareness
  updateHover() {
    if (!this.latestSnapshot || !this.latestSnapshot.robots) return;

    let closest = null;
    let minDist = 34 * this.camera.zoom;

    Object.values(this.latestSnapshot.robots).forEach(r => {
      const p = this.robotPhysics[r.id];
      const gx = p ? p.x : r.position[0];
      const gy = p ? p.y : r.position[1];

      let rawX, rawY;
      if (this.viewMode === '3d') {
        const pt = this.toIso(gx, gy);
        rawX = pt.x;
        rawY = pt.y + this.isoTileH * 0.5 + (p ? p.suspension : 0) - 10;
      } else {
        const ox = this.origin2DX || 0;
        const oy = this.origin2DY || 0;
        rawX = ox + (gx + 0.5) * this.cellPx;
        rawY = oy + (gy + 0.5) * this.cellPx;
      }

      const screenX = rawX * this.camera.zoom + this.camera.panX;
      const screenY = rawY * this.camera.zoom + this.camera.panY;

      const d = Math.hypot(this.mouse.x - screenX, this.mouse.y - screenY);
      if (d < minDist) {
        minDist = d;
        closest = { robot: r, screenX, screenY, physics: p };
      }
    });

    this.mouse.hoveredRobot = closest ? closest.robot : null;

    if (closest && this.tooltipEl) {
      const r = closest.robot;
      const p = closest.physics;
      const isFaulted = !!this.simulatedFaults[r.id];

      this.tooltipEl.style.display = 'block';
      this.tooltipEl.style.left = `${Math.min(this.displayWidth - 230, Math.max(12, closest.screenX + 16))}px`;
      this.tooltipEl.style.top = `${Math.min(this.displayHeight - 140, Math.max(12, closest.screenY - 35))}px`;

      this.ttHeaderEl.innerHTML = `
        <div style="display:flex;align-items:center;justify-content:space-between;gap:8px;">
          <span>🤖 <strong>${r.id}</strong></span>
          <span style="font-size:10px;color:var(--text-muted);font-weight:600;">${(r.type || 'AMR').replace('_', ' ')}</span>
        </div>
      `;

      let stateText = r.state;
      let stateColor = '#fbbf24';
      if (isFaulted) { stateText = '⚠️ MOTOR STALL (E-STOP)'; stateColor = 'var(--accent-rose)'; }
      else if (r.state === 'YIELDING') { stateText = '⏸ YIELDING (Right of Way)'; stateColor = 'var(--accent-amber-light)'; }
      else if (r.state === 'WAITING') { stateText = '⏳ WAITING (Corridor)'; stateColor = 'var(--accent-rose)'; }
      else if (r.state === 'MOVING') { stateText = '▶ CRUISING (Active)'; stateColor = 'var(--accent-emerald-light)'; }

      const speedMs = p ? (p.speed * 0.8).toFixed(2) : '0.00';
      const battery = Math.round(r.battery_pct || 98);

      this.ttBodyEl.innerHTML = `
        <div style="color:${stateColor};font-weight:700;font-size:11px;margin-bottom:5px;">${stateText}</div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:4px;font-size:11px;margin-bottom:4px;">
          <div>Speed: <strong style="color:var(--text-main);">${speedMs} m/s</strong></div>
          <div>Battery: <strong style="color:${battery > 40 ? 'var(--accent-emerald-light)' : 'var(--accent-amber-light)'};">${battery}%</strong></div>
          <div>Grid Pose: <strong style="color:var(--text-main);">(${r.position[0]}, ${r.position[1]})</strong></div>
          <div>Odometer: <strong style="color:var(--text-main);">${Math.round(r.odometer_meters || 0)}m</strong></div>
        </div>
        ${r.current_task_id ? `<div style="color:#fbbf24;font-size:10.5px;padding-top:3px;border-top:1px solid var(--border-subtle);">Payload Order: <strong>${r.current_task_id.replace('TASK-', '#')}</strong></div>` : '<div style="color:var(--text-muted);font-size:10px;padding-top:3px;border-top:1px solid var(--border-subtle);">Turntable: Empty Pallet Deck</div>'}
        ${isFaulted ? '<div style="color:#fca5a5;font-weight:700;font-size:10.5px;margin-top:4px;padding:3px 6px;background:rgba(239,68,68,0.2);border-radius:4px;border:1px solid var(--border-hazard);">🚨 Recoil Shudder &amp; Interlock Active</div>' : ''}
      `;
    } else if (this.tooltipEl) {
      this.tooltipEl.style.display = 'none';
    }
  }

  // Simulated Fault Injection API (Collision Recoil, Sparks, Smoke & Skid Marks)
  toggleSimulatedFault(robotId) {
    if (this.simulatedFaults[robotId]) {
      delete this.simulatedFaults[robotId];
      return false; // Fault cleared
    } else {
      this.simulatedFaults[robotId] = {
        type: 'MOTOR_STALL',
        since: performance.now(),
      };

      const p = this.robotPhysics[robotId];
      if (p) {
        // 1. Dynamic Micro-Rebound Recoil: violent reverse jerk with damped harmonic spring
        const impactAngle = p.angle || 0;
        p.recoilVx = -Math.cos(impactAngle) * 1.8;
        p.recoilVy = -Math.sin(impactAngle) * 1.8;
        p.recoilDecay = 1.0;

        // 2. High-Density Impact Spark Burst (18 sparks)
        for (let i = 0; i < 18; i++) {
          this.spawnSparkParticle(p.x, p.y);
        }

        // 3. Dense Billowing Smoke Particulate (7 puffs)
        for (let i = 0; i < 7; i++) {
          this.spawnSmokeParticle(p.x, p.y);
        }

        // 4. Heavy Rubber Brake Skid Marks baked into the floor tile
        this.addSkidMark(p.x, p.y, p.angle, 1.0);

        // 5. Ground Impact Shockwave Ring
        this.addCollisionRipple(p.x, p.y);
      }
      return true; // Fault activated
    }
  }

  clearAllFaults() {
    this.simulatedFaults = {};
  }

  addSkidMark(gx, gy, angle, intensity = 0.8) {
    this.skidMarks.push({
      gx,
      gy,
      angle,
      alpha: intensity,
      life: 1.0,
      decayRate: 0.04, // Fades gracefully over ~25s
    });
    if (this.skidMarks.length > 60) this.skidMarks.shift();
  }

  addCollisionRipple(gx, gy) {
    this.collisionRipples.push({
      gx,
      gy,
      radius: 0.1,
      maxRadius: 2.6,
      alpha: 0.9,
      speed: 3.8,
    });
  }

  // External snapshot receiver from WebSocket
  render(snapshot) {
    if (!snapshot) return;
    this.latestSnapshot = snapshot;

    if (snapshot.warehouse) {
      this.gridWidth = snapshot.warehouse.width || 24;
      this.gridHeight = snapshot.warehouse.height || 18;
    }

    // Sync robot physics cache
    if (snapshot.robots) {
      Object.values(snapshot.robots).forEach(r => {
        const [tx, ty] = r.position;
        if (!this.robotPhysics[r.id]) {
          this.robotPhysics[r.id] = {
            x: tx,
            y: ty,
            targetX: tx,
            targetY: ty,
            vx: 0,
            vy: 0,
            speed: 0,
            lastSpeed: 0,
            angle: 0,
            pitch: 0,
            roll: 0,
            suspension: 0,
            cargoSettle: 0,
            recoilX: 0,
            recoilY: 0,
            recoilVx: 0,
            recoilVy: 0,
            recoilDecay: 0,
            lidarAngle: Math.random() * Math.PI * 2,
            skidCooldown: 0,
            fanRotation: 0,
          };
        } else {
          this.robotPhysics[r.id].targetX = tx;
          this.robotPhysics[r.id].targetY = ty;
        }
      });
    }
  }

  // Continuous 60 FPS Engine Loop
  animate(now) {
    const dt = Math.min(0.1, (now - this.lastTimestamp) / 1000.0);
    this.lastTimestamp = now;

    // Smooth camera pan & zoom interpolation
    this.camera.zoom += (this.camera.targetZoom - this.camera.zoom) * Math.min(1.0, 10.0 * dt);
    this.camera.panX += (this.camera.targetPanX - this.camera.panX) * Math.min(1.0, 12.0 * dt);
    this.camera.panY += (this.camera.targetPanY - this.camera.panY) * Math.min(1.0, 12.0 * dt);

    // Update physical dynamics
    this.updatePhysics(dt, now * 0.001);

    // Render frame
    this.drawFrame(now * 0.001);

    requestAnimationFrame(this.animate);
  }

  updatePhysics(dt, timeSec) {
    const springRate = this.enablePhysics ? 5.8 : 14.0;

    // 1. Robot Motion, Acceleration & Suspension Dynamics
    Object.keys(this.robotPhysics).forEach(rId => {
      const p = this.robotPhysics[rId];
      const isFaulted = !!this.simulatedFaults[rId];

      // Internal cooling fan spin
      p.fanRotation = (p.fanRotation + dt * 18.0) % (Math.PI * 2);

      // Handle Recoil Dynamics (damped spring return)
      if (p.recoilDecay > 0) {
        p.recoilX += p.recoilVx * dt;
        p.recoilY += p.recoilVy * dt;
        p.recoilVx += (-p.recoilX * 28.0 - p.recoilVx * 8.5) * dt;
        p.recoilVy += (-p.recoilY * 28.0 - p.recoilVy * 8.5) * dt;
        p.recoilDecay = Math.max(0, p.recoilDecay - dt * 1.8);
      } else {
        p.recoilX = 0;
        p.recoilY = 0;
      }

      if (isFaulted) {
        // Stalled motor: shuddering vibration & smoke/sparks
        p.speed = 0;
        p.pitch = (Math.random() - 0.5) * 0.06;
        p.suspension = (Math.random() - 0.5) * 1.5;

        // Continuous particulate emission during fault
        if (Math.random() < 0.38) {
          this.spawnSmokeParticle(p.x, p.y);
        }
        if (Math.random() < 0.28) {
          this.spawnSparkParticle(p.x, p.y);
        }
        return;
      }

      const dx = p.targetX - p.x;
      const dy = p.targetY - p.y;
      const dist = Math.hypot(dx, dy);

      const prevX = p.x;
      const prevY = p.y;

      p.x += dx * Math.min(1.0, springRate * dt);
      p.y += dy * Math.min(1.0, springRate * dt);

      p.vx = (p.x - prevX) / Math.max(0.001, dt);
      p.vy = (p.y - prevY) / Math.max(0.001, dt);
      const currentSpeed = Math.hypot(p.vx, p.vy);

      // Check sudden deceleration for rubber skid marks
      const decel = (p.lastSpeed || 0) - currentSpeed;
      p.skidCooldown = Math.max(0, (p.skidCooldown || 0) - dt);

      if (this.enablePhysics && decel > 0.42 && p.skidCooldown <= 0) {
        this.addSkidMark(p.x, p.y, p.angle, Math.min(0.9, decel * 0.85));
        p.skidCooldown = 0.35;
      }
      p.lastSpeed = currentSpeed;
      p.speed = currentSpeed;

      // Heading orientation with inertial damping
      if (dist > 0.035) {
        const targetAngle = Math.atan2(dy, dx);
        let angleDiff = (targetAngle - p.angle + Math.PI * 3) % (Math.PI * 2) - Math.PI;
        p.angle += angleDiff * Math.min(1.0, 9.5 * dt);

        if (this.enablePhysics) {
          // Dynamic chassis roll banking into turns
          p.roll = Math.max(-0.15, Math.min(0.15, angleDiff * 0.48));
        }
      } else {
        p.roll *= 0.82;
      }

      // Dynamic acceleration pitch & suspension bobbing
      if (this.enablePhysics) {
        const accel = (currentSpeed - (p.lastSpeed || 0)) / Math.max(0.001, dt);
        // Forward pitch on braking, backward tilt on acceleration
        p.pitch = Math.max(-0.18, Math.min(0.18, accel * 0.028));

        if (currentSpeed > 0.08) {
          p.suspension = Math.sin(timeSec * 24) * Math.min(1.8, currentSpeed * 1.4);
          p.cargoSettle = Math.sin(timeSec * 16) * 2.0 * Math.min(1.0, currentSpeed);
        } else {
          p.suspension *= 0.8;
          p.cargoSettle *= 0.8;
        }

        // 360° LiDAR scanner rotation (360 RPM)
        p.lidarAngle = (p.lidarAngle + dt * 6.5) % (Math.PI * 2);
      } else {
        p.pitch = 0;
        p.roll = 0;
        p.suspension = 0;
        p.cargoSettle = 0;
      }
    });

    // 2. Skid Marks Fade
    for (let i = this.skidMarks.length - 1; i >= 0; i--) {
      const sm = this.skidMarks[i];
      sm.life -= sm.decayRate * dt;
      sm.alpha = Math.max(0, sm.life);
      if (sm.life <= 0) {
        this.skidMarks.splice(i, 1);
      }
    }

    // 3. Collision Ripples Expansion
    for (let i = this.collisionRipples.length - 1; i >= 0; i--) {
      const cr = this.collisionRipples[i];
      cr.radius += cr.speed * dt;
      cr.alpha = Math.max(0, 1.0 - (cr.radius / cr.maxRadius));
      if (cr.radius >= cr.maxRadius) {
        this.collisionRipples.splice(i, 1);
      }
    }

    // 4. Malfunction Particles Update (Smoke & Sparks)
    for (let i = this.faultParticles.length - 1; i >= 0; i--) {
      const fp = this.faultParticles[i];
      fp.life -= dt;
      if (fp.life <= 0) {
        this.faultParticles.splice(i, 1);
        continue;
      }

      if (fp.type === 'smoke') {
        fp.x += fp.vx * dt;
        fp.y += fp.vy * dt;
        fp.z += fp.vz * dt;
        fp.radius += fp.growth * dt;
        fp.alpha = Math.max(0, (fp.life / fp.maxLife) * fp.initialAlpha);
      } else if (fp.type === 'spark') {
        fp.x += fp.vx * dt;
        fp.y += fp.vy * dt;
        fp.z += fp.vz * dt;
        fp.vz -= 12.0 * dt; // Gravity
        if (fp.z <= 0) {
          fp.z = 0;
          fp.vz = -fp.vz * 0.45; // Elastic bounce off concrete
          fp.vx *= 0.75;
          fp.vy *= 0.75;
        }
        fp.alpha = Math.max(0, fp.life / fp.maxLife);
      }
    }

    // 5. Ambient Atmospheric Dust
    if (this.enablePhysics) {
      this.dustParticles.forEach(pt => {
        pt.x += pt.vx;
        pt.y += pt.vy;
        if (pt.y < 0) {
          pt.y = this.displayHeight || 700;
          pt.x = Math.random() * (this.displayWidth || 1000);
        }
        if (pt.x < 0) pt.x = this.displayWidth || 1000;
        if (pt.x > (this.displayWidth || 1000)) pt.x = 0;
      });
    }
  }

  spawnSmokeParticle(gx, gy) {
    this.faultParticles.push({
      type: 'smoke',
      gx: gx + (Math.random() - 0.5) * 0.25,
      gy: gy + (Math.random() - 0.5) * 0.25,
      x: 0,
      y: 0,
      z: 10 + Math.random() * 6,
      vx: (Math.random() - 0.5) * 14,
      vy: (Math.random() - 0.5) * 14,
      vz: 18 + Math.random() * 22,
      radius: 4 + Math.random() * 3,
      growth: 10 + Math.random() * 8,
      life: 1.5 + Math.random() * 0.8,
      maxLife: 2.2,
      initialAlpha: 0.6,
      alpha: 0.6,
      gray: 45 + Math.floor(Math.random() * 55),
    });
  }

  spawnSparkParticle(gx, gy) {
    const angle = Math.random() * Math.PI * 2;
    const speed = 30 + Math.random() * 50;
    this.faultParticles.push({
      type: 'spark',
      gx: gx + (Math.random() - 0.5) * 0.2,
      gy: gy + (Math.random() - 0.5) * 0.2,
      x: 0,
      y: 0,
      z: 8,
      vx: Math.cos(angle) * speed,
      vy: Math.sin(angle) * speed,
      vz: 28 + Math.random() * 38,
      radius: 1.4 + Math.random() * 1.4,
      life: 0.7 + Math.random() * 0.4,
      maxLife: 0.9,
      alpha: 1.0,
      color: Math.random() > 0.35 ? '#f59e0b' : '#ef4444',
    });
  }

  drawFrame(timeSec) {
    const ctx = this.ctx;
    const w = this.displayWidth || 1000;
    const h = this.displayHeight || 700;

    ctx.clearRect(0, 0, w, h);

    if (!this.latestSnapshot || !this.latestSnapshot.warehouse) {
      this.drawPlaceholder(ctx, w, h);
      return;
    }

    // Apply Camera Matrix (Pan & Zoom)
    ctx.save();
    ctx.translate(this.camera.panX, this.camera.panY);
    ctx.scale(this.camera.zoom, this.camera.zoom);

    if (this.viewMode === '3d') {
      this.render3D(ctx, timeSec);
    } else {
      this.render2D(ctx, timeSec);
    }

    ctx.restore();

    // Atmospheric warehouse motes overlay (screen space, 3D mode only)
    if (this.enablePhysics && this.viewMode === '3d') {
      this.drawAtmosphere(ctx);
    }
  }

  drawPlaceholder(ctx, w, h) {
    ctx.fillStyle = '#64748b';
    ctx.font = '600 14px "Inter", sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Awaiting live warehouse digital twin telemetry stream...', w / 2, h / 2);
  }

  // =========================================================================
  // HIGH-DETAIL INDUSTRIAL 3D ISOMETRIC WAREHOUSE ENGINE
  // =========================================================================
  render3D(ctx, timeSec) {
    const snap = this.latestSnapshot;
    const gw = this.gridWidth;
    const gh = this.gridHeight;
    const tw = this.isoTileW;
    const th = this.isoTileH;

    // 1. Concrete Polished Floor with Specular Sheen & Expansion Joints
    this.drawIsoConcreteFloor(ctx, gw, gh, tw, th, timeSec);

    // 2. Safety Hazard Chevron Borders around Human Zones & Stations
    if (snap.warehouse.human_zones) {
      snap.warehouse.human_zones.forEach(hz => {
        if (!hz.active) return;
        this.drawIsoSafetyChevrons(ctx, hz.bounds, tw, th, timeSec);
      });
    }

    // 3. Residual Tire Skid Marks (Baked on concrete floor)
    if (this.skidMarks.length > 0) {
      this.skidMarks.forEach(sm => this.drawIsoSkidMark(ctx, sm));
    }

    // 4. Ground Collision Shockwaves (Physics)
    if (this.collisionRipples.length > 0) {
      this.collisionRipples.forEach(cr => this.drawIsoCollisionRipple(ctx, cr));
    }

    // 5. Human Safety Derating Zones (3D Volumetric Amber Caution Cubes)
    if (this.showHumanZones && snap.warehouse.human_zones) {
      snap.warehouse.human_zones.forEach(hz => {
        if (!hz.active) return;
        this.drawIsoSafetyZone(ctx, hz.bounds, timeSec);
      });
    }

    // 6. Industrial Roller Conveyor Stations with 3-Tier Andon Stack Lights
    if (snap.warehouse.pickup_stations) {
      snap.warehouse.pickup_stations.forEach(s => this.drawIsoStation(ctx, s.location, 'IN', '#10b981', timeSec));
    }
    if (snap.warehouse.dropoff_stations) {
      snap.warehouse.dropoff_stations.forEach(s => this.drawIsoStation(ctx, s.location, 'OUT', '#d97706', timeSec));
    }

    // 7. Dynamic Congestion Heatmap Glow
    if (this.showHeatmap && snap.congestion_heatmap && snap.congestion_heatmap.cells) {
      snap.congestion_heatmap.cells.forEach(c => this.drawIsoHeatmapCell(ctx, c.x, c.y, c.intensity));
    }

    // 8. Forward Intent Trajectory Ribbons
    if (this.showIntentPaths && snap.robots) {
      Object.values(snap.robots).forEach(r => this.drawIsoIntentPath(ctx, r));
    }

    // 9. Depth-Sorted Rendering (Z-buffer order by (gx + gy))
    const renderList = [];

    // Add Pallet Racks
    if (snap.warehouse.obstacles) {
      snap.warehouse.obstacles.forEach(([ox, oy]) => {
        renderList.push({
          type: 'rack',
          sortKey: ox + oy,
          gx: ox,
          gy: oy,
        });
      });
    }

    // Add AMRs
    if (snap.robots) {
      Object.values(snap.robots).forEach(r => {
        const p = this.robotPhysics[r.id];
        const gx = p ? p.x : r.position[0];
        const gy = p ? p.y : r.position[1];
        renderList.push({
          type: 'amr',
          sortKey: gx + gy,
          robot: r,
          physics: p,
        });
      });
    }

    // Sort back-to-front for realistic depth occlusion
    renderList.sort((a, b) => a.sortKey - b.sortKey);

    renderList.forEach(item => {
      if (item.type === 'rack') {
        this.drawIsoStorageRack(ctx, item.gx, item.gy);
      } else if (item.type === 'amr') {
        this.drawIsoAMR(ctx, item.robot, item.physics, timeSec);
      }
    });

    // 10. Malfunction Smoke Puffs & Sparks (Rendered in 3D world space)
    if (this.faultParticles.length > 0) {
      this.drawIsoFaultParticles(ctx);
    }
  }

  // 1. Matte Industrial Concrete Floor with Specular Highlights & Expansion Joint Seams
  drawIsoConcreteFloor(ctx, gw, gh, tw, th, timeSec) {
    // A. Extruded 3D Concrete Foundation Slab Platform & Outer Boundary Curb
    const pFarRight = this.toIso(gw, 0);
    const pFrontCorner = this.toIso(gw, gh);
    const pFarLeft = this.toIso(0, gh);
    const curbDepth = 14;

    // Right foundation wall face
    ctx.beginPath();
    ctx.moveTo(pFrontCorner.x, pFrontCorner.y + th * 0.5);
    ctx.lineTo(pFarRight.x, pFarRight.y + th * 0.5);
    ctx.lineTo(pFarRight.x, pFarRight.y + th * 0.5 + curbDepth);
    ctx.lineTo(pFrontCorner.x, pFrontCorner.y + th * 0.5 + curbDepth);
    ctx.closePath();
    ctx.fillStyle = '#1e2638';
    ctx.fill();
    ctx.strokeStyle = '#0f1422';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Left foundation wall face
    ctx.beginPath();
    ctx.moveTo(pFrontCorner.x, pFrontCorner.y + th * 0.5);
    ctx.lineTo(pFarLeft.x, pFarLeft.y + th * 0.5);
    ctx.lineTo(pFarLeft.x, pFarLeft.y + th * 0.5 + curbDepth);
    ctx.lineTo(pFrontCorner.x, pFrontCorner.y + th * 0.5 + curbDepth);
    ctx.closePath();
    ctx.fillStyle = '#263147';
    ctx.fill();
    ctx.strokeStyle = '#151d2c';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Safety yellow/black hazard striping along the outer foundation curb
    this.drawChevronLine(ctx, pFrontCorner.x, pFrontCorner.y + th * 0.5 + 4, pFarRight.x, pFarRight.y + th * 0.5 + 4);
    this.drawChevronLine(ctx, pFarLeft.x, pFarLeft.y + th * 0.5 + 4, pFrontCorner.x, pFrontCorner.y + th * 0.5 + 4);

    // B. Base Concrete Slab Rendering (Authentic High-Contrast Industrial Concrete)
    for (let gy = 0; gy < gh; gy++) {
      for (let gx = 0; gx < gw; gx++) {
        const pt = this.toIso(gx, gy);

        // Industrial concrete slab tones (Satisfactory / Siemens factory simulator style)
        const isMainAisle = (gx >= 3 && gx <= 5) || (gx >= 11 && gx <= 13) || (gx >= 19 && gx <= 21);
        let slabColor;
        if (isMainAisle) {
          // Polished high-traffic epoxy corridor
          slabColor = (gx + gy) % 2 === 0 ? '#455066' : '#4b5770';
        } else {
          // Matte factory concrete slabs
          slabColor = (gx + gy) % 2 === 0 ? '#384256' : '#3d485c';
        }

        // Clean, distinct expansion seam joint stroke
        const isExpansionX = (gx % 4 === 0);
        const isExpansionY = (gy % 4 === 0);
        const strokeColor = (isExpansionX || isExpansionY) ? '#182030' : '#2d3546';
        const strokeWidth = (isExpansionX || isExpansionY) ? 1.8 : 0.8;

        this.drawIsoTile(ctx, pt.x, pt.y, tw, th, slabColor, strokeColor, strokeWidth);

        // Tactile 3D expansion groove bevel highlight (gives tangible depth to the concrete seam!)
        if (isExpansionX || isExpansionY) {
          ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
          ctx.lineWidth = 0.8;
          ctx.beginPath();
          ctx.moveTo(pt.x + tw * 0.5, pt.y + th * 0.5);
          ctx.lineTo(pt.x, pt.y + th);
          ctx.stroke();
        }

        // Clean 1m x 1m grid crosshairs at tile corners (non-intrusive coordinate grid)
        this.drawIsoGridCrosshair(ctx, pt.x, pt.y + th * 0.5, tw, th);

        // Floor QR Fiducials on alternate nodes
        if (this.showFloorDetails && (gx % 2 === 0 && gy % 2 === 0)) {
          this.drawIsoFiducialMarker(ctx, pt.x, pt.y + th * 0.5, tw, th, gx, gy);
        }
      }
    }

    // C. Specular Highlight Overhead High-Bay Floodlight Wash (Matte Polished Sheen)
    const centerPt = this.toIso(gw * 0.5, gh * 0.5);
    const sheenW = gw * tw * 0.48;
    const sheenH = gh * th * 0.42;
    const sheenGrad = ctx.createRadialGradient(centerPt.x, centerPt.y - 30, 20, centerPt.x, centerPt.y, sheenW);
    sheenGrad.addColorStop(0, 'rgba(255, 255, 255, 0.08)');
    sheenGrad.addColorStop(0.4, 'rgba(186, 230, 253, 0.035)');
    sheenGrad.addColorStop(1, 'transparent');

    ctx.save();
    ctx.fillStyle = sheenGrad;
    ctx.beginPath();
    ctx.ellipse(centerPt.x, centerPt.y, sheenW, sheenH, 0, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();

    // D. Safety Floor Guidelines (Dotted Yellow Aisle Dividers)
    if (this.showFloorDetails) {
      ctx.strokeStyle = 'rgba(250, 204, 21, 0.45)';
      ctx.lineWidth = 1.4;
      ctx.setLineDash([5, 6]);

      [3, 6, 11, 14, 19].forEach(gx => {
        if (gx < gw) {
          const p1 = this.toIso(gx, 0);
          const p2 = this.toIso(gx, gh);
          ctx.beginPath();
          ctx.moveTo(p1.x, p1.y + th * 0.5);
          ctx.lineTo(p2.x, p2.y + th * 0.5);
          ctx.stroke();
        }
      });
      ctx.setLineDash([]);
    }
  }

  // Non-intrusive floor coordinate tick / crosshairs (+) at cell corners (Clean Grid Projection)
  drawIsoGridCrosshair(ctx, cx, cy, tw, th) {
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.26)';
    ctx.lineWidth = 1.2;

    const armW = 3.2;
    const armH = 1.6;

    ctx.beginPath();
    ctx.moveTo(cx - armW, cy - armH);
    ctx.lineTo(cx + armW, cy + armH);
    ctx.moveTo(cx - armW, cy + armH);
    ctx.lineTo(cx + armW, cy - armH);
    ctx.stroke();
  }

  // High-Contrast Yellow & Black Industrial Chevron Hazard Stripes around Docks & Human Zones
  drawIsoSafetyChevrons(ctx, bounds, tw, th, timeSec) {
    const [minX, minY, maxX, maxY] = bounds;
    const p1 = this.toIso(minX, minY);
    const p2 = this.toIso(maxX + 1, minY);
    const p3 = this.toIso(maxX + 1, maxY + 1);
    const p4 = this.toIso(minX, maxY + 1);

    ctx.save();
    ctx.lineWidth = 4.5;
    ctx.lineCap = 'square';

    // Segment 1: p1 -> p2
    this.drawChevronLine(ctx, p1.x, p1.y, p2.x, p2.y);
    // Segment 2: p2 -> p3
    this.drawChevronLine(ctx, p2.x, p2.y, p3.x, p3.y);
    // Segment 3: p3 -> p4
    this.drawChevronLine(ctx, p3.x, p3.y, p4.x, p4.y);
    // Segment 4: p4 -> p1
    this.drawChevronLine(ctx, p4.x, p4.y, p1.x, p1.y);

    ctx.restore();
  }

  drawChevronLine(ctx, x1, y1, x2, y2) {
    const dist = Math.hypot(x2 - x1, y2 - y1);
    const step = 9;
    const count = Math.max(1, Math.floor(dist / step));
    const dx = (x2 - x1) / count;
    const dy = (y2 - y1) / count;

    for (let i = 0; i < count; i++) {
      const sx = x1 + i * dx;
      const sy = y1 + i * dy;
      const ex = sx + dx;
      const ey = sy + dy;

      ctx.strokeStyle = (i % 2 === 0) ? '#facc15' : '#0f172a';
      ctx.lineWidth = 4.5;
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(ex, ey);
      ctx.stroke();
    }
  }

  drawIsoTile(ctx, sx, sy, tw, th, fillColor, strokeColor, strokeWidth = 1) {
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(sx + tw * 0.5, sy + th * 0.5);
    ctx.lineTo(sx, sy + th);
    ctx.lineTo(sx - tw * 0.5, sy + th * 0.5);
    ctx.closePath();

    ctx.fillStyle = fillColor;
    ctx.fill();
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = strokeWidth;
    ctx.stroke();
  }

  // Realistic Data-Matrix QR Fiducial Floor Markers (Amazon Kiva style)
  drawIsoFiducialMarker(ctx, cx, cy, tw, th, gx, gy) {
    const sizeW = tw * 0.2;
    const sizeH = th * 0.2;

    // Metallic white marker plate
    ctx.beginPath();
    ctx.moveTo(cx, cy - sizeH * 0.5);
    ctx.lineTo(cx + sizeW * 0.5, cy);
    ctx.lineTo(cx, cy + sizeH * 0.5);
    ctx.lineTo(cx - sizeW * 0.5, cy);
    ctx.closePath();
    ctx.fillStyle = 'rgba(241, 245, 249, 0.4)';
    ctx.fill();

    // Dark matrix code dots
    ctx.fillStyle = 'rgba(15, 23, 42, 0.7)';
    ctx.beginPath();
    ctx.arc(cx, cy, 1.1, 0, Math.PI * 2);
    ctx.fill();
  }

  // Physics Rubber Tire Skid Marks baked onto concrete
  drawIsoSkidMark(ctx, sm) {
    const pt = this.toIso(sm.gx, sm.gy);
    const th = this.isoTileH;
    const tw = this.isoTileW;
    const cx = pt.x;
    const cy = pt.y + th * 0.5;

    ctx.save();
    ctx.translate(cx, cy);
    
    // Dual parallel black rubber tire tread skid tracks
    const trackDist = tw * 0.16;
    const trackLen = tw * 0.38;
    const cos = Math.cos(sm.angle);
    const sin = Math.sin(sm.angle) * 0.5;

    ctx.strokeStyle = `rgba(10, 15, 26, ${sm.alpha * 0.8})`;
    ctx.lineWidth = 2.4;
    ctx.lineCap = 'round';

    // Left wheel skid
    ctx.beginPath();
    ctx.moveTo(-sin * trackDist - cos * trackLen * 0.5, cos * trackDist - sin * trackLen * 0.5);
    ctx.lineTo(-sin * trackDist + cos * trackLen * 0.5, cos * trackDist + sin * trackLen * 0.5);
    ctx.stroke();

    // Right wheel skid
    ctx.beginPath();
    ctx.moveTo(sin * trackDist - cos * trackLen * 0.5, -cos * trackDist - sin * trackLen * 0.5);
    ctx.lineTo(sin * trackDist + cos * trackLen * 0.5, -cos * trackDist + sin * trackLen * 0.5);
    ctx.stroke();

    ctx.restore();
  }

  // Impact Shockwave Ripple on Floor
  drawIsoCollisionRipple(ctx, cr) {
    const pt = this.toIso(cr.gx, cr.gy);
    const th = this.isoTileH;
    const tw = this.isoTileW;
    const cx = pt.x;
    const cy = pt.y + th * 0.5;
    const rx = cr.radius * tw * 0.5;
    const ry = cr.radius * th * 0.5;

    ctx.beginPath();
    ctx.ellipse(cx, cy, rx, ry, 0, 0, Math.PI * 2);
    ctx.strokeStyle = `rgba(239, 68, 68, ${cr.alpha * 0.85})`;
    ctx.lineWidth = 2.2;
    ctx.stroke();
  }

  // =========================================================================
  // SOLID INDUSTRIAL PALLET RACKS (Steel Columns, Safety Beams, Cargo)
  // =========================================================================
  drawIsoStorageRack(ctx, gx, gy) {
    const tw = this.isoTileW;
    const th = this.isoTileH;
    const pt = this.toIso(gx, gy);
    const sx = pt.x;
    const sy = pt.y;
    const rackH = 44; // Extruded structural height

    // 1. Directional Soft Floor Ambient Occlusion Shadow
    ctx.beginPath();
    ctx.ellipse(sx + 3, sy + th * 0.5 + 2, tw * 0.5, th * 0.44, 0, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
    ctx.fill();

    // 2. Left Frame Face (Industrial Structural Frame Shadowed)
    ctx.beginPath();
    ctx.moveTo(sx - tw * 0.5, sy + th * 0.5);
    ctx.lineTo(sx, sy + th);
    ctx.lineTo(sx, sy + th - rackH);
    ctx.lineTo(sx - tw * 0.5, sy + th * 0.5 - rackH);
    ctx.closePath();
    ctx.fillStyle = '#0f172a';
    ctx.fill();
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    ctx.stroke();

    // 3. Right Frame Face (Mid-Tone Lit)
    ctx.beginPath();
    ctx.moveTo(sx, sy + th);
    ctx.lineTo(sx + tw * 0.5, sy + th * 0.5);
    ctx.lineTo(sx + tw * 0.5, sy + th * 0.5 - rackH);
    ctx.lineTo(sx, sy + th - rackH);
    ctx.closePath();
    ctx.fillStyle = '#172554'; // Deep navy steel
    ctx.fill();
    ctx.strokeStyle = '#1e3a8a';
    ctx.stroke();

    // 4. Slotted Heavy Steel Upright Columns (Industrial Blue with Teardrop Slots)
    const postColor = '#1d4ed8'; // Standard industrial racking blue
    ctx.fillStyle = postColor;

    // Corner 1: Front center post
    ctx.fillRect(sx - 1.8, sy + th - rackH, 3.6, rackH);
    // Corner 2: Left post
    ctx.fillRect(sx - tw * 0.5, sy + th * 0.5 - rackH, 3, rackH);
    // Corner 3: Right post
    ctx.fillRect(sx + tw * 0.5 - 3, sy + th * 0.5 - rackH, 3, rackH);

    // Teardrop slot holes down front upright
    ctx.fillStyle = '#0f172a';
    for (let slotY = sy + th - rackH + 4; slotY < sy + th - 4; slotY += 6) {
      ctx.fillRect(sx - 0.7, slotY, 1.4, 3);
    }

    // High-Vis Safety Yellow Corner Column Impact Guards at Base
    ctx.fillStyle = '#eab308';
    ctx.fillRect(sx - 3, sy + th - 8, 6, 8);
    ctx.fillRect(sx - tw * 0.5 - 1, sy + th * 0.5 - 7, 5, 7);
    ctx.fillRect(sx + tw * 0.5 - 4, sy + th * 0.5 - 7, 5, 7);

    // Diagonal Structural Truss Cross-Bracing Wires on Frame Flanks
    ctx.beginPath();
    ctx.moveTo(sx, sy + th);
    ctx.lineTo(sx + tw * 0.5, sy + th * 0.5 - rackH);
    ctx.moveTo(sx, sy + th - rackH);
    ctx.lineTo(sx + tw * 0.5, sy + th * 0.5);
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.28)';
    ctx.lineWidth = 1.0;
    ctx.stroke();

    // 5. Tier 1 & Tier 2 Structural Load Beams (Vivid Safety Orange)
    const beamTiers = [rackH * 0.48, rackH * 0.96];
    beamTiers.forEach(hTier => {
      // Horizontal load beam
      ctx.beginPath();
      ctx.moveTo(sx - tw * 0.5, sy + th * 0.5 - hTier);
      ctx.lineTo(sx, sy + th - hTier);
      ctx.lineTo(sx + tw * 0.5, sy + th * 0.5 - hTier);
      ctx.strokeStyle = '#ea580c'; // Heavy equipment safety orange
      ctx.lineWidth = 2.4;
      ctx.stroke();

      // Beam connector locking pins (yellow)
      ctx.fillStyle = '#fde047';
      ctx.fillRect(sx - 2, sy + th - hTier - 1.5, 4, 3);

      // Wooden Timber Pallet Deck (Pine Wood with stringers)
      const palletH = 3.5;
      ctx.beginPath();
      ctx.rect(sx - tw * 0.4, sy + th * 0.5 - hTier - palletH, tw * 0.8, palletH);
      ctx.fillStyle = '#92400e';
      ctx.fill();

      // Pallet timber stringer runners
      ctx.fillStyle = '#78350f';
      ctx.fillRect(sx - tw * 0.35, sy + th * 0.5 - hTier - palletH, 3, palletH);
      ctx.fillRect(sx - 1.5, sy + th * 0.5 - hTier - palletH, 3, palletH);
      ctx.fillRect(sx + tw * 0.35 - 3, sy + th * 0.5 - hTier - palletH, 3, palletH);
    });

    // 6. Top Deck Steel Grate
    ctx.beginPath();
    ctx.moveTo(sx, sy - rackH);
    ctx.lineTo(sx + tw * 0.5, sy + th * 0.5 - rackH);
    ctx.lineTo(sx, sy + th - rackH);
    ctx.lineTo(sx - tw * 0.5, sy + th * 0.5 - rackH);
    ctx.closePath();
    ctx.fillStyle = '#1e293b';
    ctx.fill();
    ctx.strokeStyle = '#334155';
    ctx.stroke();

    // 7. Textured Palletized Cargo Boxes with Labels & Strapping
    const seed = (gx * 23 + gy * 41);
    const boxConfigs = [
      { color: '#d97706', w: 14, h: 10, label: '#fff' }, // Kraft corrugated brown
      { color: '#b45309', w: 13, h: 9, label: '#fff' },  // Heavy carton
      { color: '#0369a1', w: 15, h: 11, label: '#fff' }, // Industrial bin steel blue
      { color: '#059669', w: 12, h: 9, label: '#fff' },  // Defense depot olive/green
    ];
    const box = boxConfigs[seed % boxConfigs.length];

    // Main box on top tier
    const bx = sx - box.w * 0.5;
    const by = sy + th * 0.5 - rackH - box.h - 1;
    ctx.beginPath();
    ctx.rect(bx, by, box.w, box.h);
    ctx.fillStyle = box.color;
    ctx.fill();
    ctx.strokeStyle = 'rgba(0, 0, 0, 0.4)';
    ctx.lineWidth = 1;
    ctx.stroke();

    // White shipping barcode label
    ctx.fillStyle = '#f8fafc';
    ctx.fillRect(bx + 2, by + 2, box.w * 0.5, 3.5);
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(bx + 3, by + 3, box.w * 0.35, 1.2);

    // Blue security strapping band
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 0.9;
    ctx.beginPath();
    ctx.moveTo(bx + box.w * 0.5, by);
    ctx.lineTo(bx + box.w * 0.5, by + box.h);
    ctx.stroke();

    // Second smaller box on middle tier
    const midH = rackH * 0.48;
    ctx.beginPath();
    ctx.rect(sx - 6, sy + th * 0.5 - midH - 9, 12, 8);
    ctx.fillStyle = '#c2410c';
    ctx.fill();
    ctx.strokeStyle = '#7c2d12';
    ctx.lineWidth = 0.8;
    ctx.stroke();
  }

  // =========================================================================
  // REALISTIC AMR (KIVA / GEEK+ STYLE LOW-PROFILE WAREHOUSE DRIVE UNIT)
  // =========================================================================
  drawIsoAMR(ctx, r, p, timeSec) {
    const tw = this.isoTileW;
    const th = this.isoTileH;
    const gx = p ? (p.x + (p.recoilX || 0)) : r.position[0];
    const gy = p ? (p.y + (p.recoilY || 0)) : r.position[1];
    const pt = this.toIso(gx, gy);

    const isHovered = this.mouse.hoveredRobot && this.mouse.hoveredRobot.id === r.id;
    const isFaulted = !!this.simulatedFaults[r.id];

    // Suspension micro-bobbing
    const suspY = p ? p.suspension : 0;
    const sx = pt.x;
    const sy = pt.y + th * 0.5 + suspY;

    // 1. Realistic Directional Drop Shadow on Polished Concrete
    ctx.beginPath();
    const shadowStretch = p ? 1.0 + Math.min(0.3, p.speed * 0.2) : 1.0;
    ctx.ellipse(sx + 2, sy + 1, tw * 0.45 * shadowStretch, th * 0.38, p ? p.angle : 0, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fill();

    // 2. Structural Damage / Recoil Indicator: Blinking Red Danger Halo on Ground
    if (isFaulted) {
      const pulseRad = (tw * 0.52) + Math.sin(timeSec * 14) * 4.0;
      ctx.beginPath();
      ctx.ellipse(sx, sy, pulseRad, pulseRad * 0.5, 0, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(239, 68, 68, 0.35)';
      ctx.fill();
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 2.2;
      ctx.stroke();

      // Outer danger chevron hazard perimeter
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.ellipse(sx, sy, pulseRad + 4, (pulseRad + 4) * 0.5, 0, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
    } else if (r.state === 'YIELDING') {
      const pulseRad = (tw * 0.46) + Math.sin(timeSec * 8) * 2.5;
      ctx.beginPath();
      ctx.ellipse(sx, sy, pulseRad, pulseRad * 0.5, 0, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(245, 158, 11, 0.22)';
      ctx.fill();
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 1.6;
      ctx.stroke();
    } else if (r.state === 'WAITING') {
      const pulseRad = (tw * 0.46) + Math.sin(timeSec * 10) * 2.5;
      ctx.beginPath();
      ctx.ellipse(sx, sy, pulseRad, pulseRad * 0.5, 0, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(244, 63, 94, 0.25)';
      ctx.fill();
      ctx.strokeStyle = '#f43f5e';
      ctx.lineWidth = 1.6;
      ctx.stroke();
    }

    // 3. Chassis Dimensions & Inertial Tilt
    const amrHeight = 16;
    const chW = tw * 0.64;
    const chH = th * 0.64;

    // Pitch tilt & Roll banking
    const pitchDy = p ? p.pitch * 18 : 0;
    const rollDx = p ? p.roll * 14 : 0;
    const baseCy = sy - 4 + pitchDy;
    const baseCx = sx + rollDx;

    // Industrial Safety Color Palette
    let chassisColor = '#ea580c'; // Safety orange
    let accentColor = '#0f172a';  // Matte graphite
    let lightBarColor = '#d97706'; // Active amber

    if (isFaulted) {
      chassisColor = '#991b1b';
      lightBarColor = (Math.sin(timeSec * 22) > 0) ? '#ef4444' : '#f59e0b'; // Fast strobe
    } else if (r.state === 'YIELDING') {
      chassisColor = '#d97706';
      lightBarColor = '#f59e0b'; // Amber
    } else if (r.state === 'WAITING') {
      chassisColor = '#be123c';
      lightBarColor = '#f43f5e'; // Rose
    } else if (r.state === 'REROUTING') {
      chassisColor = '#7e22ce';
      lightBarColor = '#c084fc'; // Purple
    } else if (r.state === 'IDLE') {
      chassisColor = '#334155';
      lightBarColor = '#64748b'; // Slate
    }

    // A. Lower Skirt & Recessed Wheel Wells with Rubber Tread Wheels
    ctx.beginPath();
    ctx.ellipse(baseCx, baseCy, chW * 0.52, chH * 0.52, 0, 0, Math.PI * 2);
    ctx.fillStyle = '#090d16'; // Deep graphite undercarriage
    ctx.fill();

    // Visible Dual Drive Wheels with Treads on Flanks
    ctx.fillStyle = '#1e293b';
    ctx.fillRect(baseCx - chW * 0.54, baseCy - 4, 3.5, 8);
    ctx.fillRect(baseCx + chW * 0.54 - 3.5, baseCy - 4, 3.5, 8);
    // Wheel Hubs
    ctx.fillStyle = '#f59e0b';
    ctx.fillRect(baseCx - chW * 0.54 + 0.8, baseCy - 1.5, 1.8, 3);
    ctx.fillRect(baseCx + chW * 0.54 - 2.6, baseCy - 1.5, 1.8, 3);

    // B. Mid-Body Extruded Chamfered Chassis (Industrial Safety Orange)
    ctx.beginPath();
    ctx.rect(baseCx - chW * 0.5, baseCy - amrHeight, chW, amrHeight);
    ctx.fillStyle = chassisColor;
    ctx.fill();
    ctx.strokeStyle = 'rgba(0, 0, 0, 0.4)';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Front & Rear Rubber Bumper Corner Guards
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(baseCx - chW * 0.5, baseCy - amrHeight, 4, amrHeight);
    ctx.fillRect(baseCx + chW * 0.5 - 4, baseCy - amrHeight, 4, amrHeight);

    // Front & Rear Stereo Depth Camera Vision Cutouts
    ctx.fillStyle = '#020617';
    ctx.fillRect(baseCx - 7, baseCy - amrHeight + 3, 14, 4);
    // Dual optical camera lenses (glass glint)
    ctx.fillStyle = '#64748b';
    ctx.beginPath();
    ctx.arc(baseCx - 3.5, baseCy - amrHeight + 5, 1.2, 0, Math.PI * 2);
    ctx.arc(baseCx + 3.5, baseCy - amrHeight + 5, 1.2, 0, Math.PI * 2);
    ctx.fill();

    // C. 360° Industrial Perimeter LED Safety Status Light Bar
    ctx.beginPath();
    ctx.ellipse(baseCx, baseCy - amrHeight * 0.35, chW * 0.5, chH * 0.5, 0, 0, Math.PI * 2);
    ctx.strokeStyle = lightBarColor;
    ctx.lineWidth = isFaulted ? 3.2 : 2.0;
    ctx.stroke();

    // D. Top Deck Housing with Semi-Transparent Acrylic Shield
    ctx.beginPath();
    ctx.ellipse(baseCx, baseCy - amrHeight, chW * 0.48, chH * 0.48, 0, 0, Math.PI * 2);
    ctx.fillStyle = '#1e293b'; // Graphite top plate
    ctx.fill();
    ctx.strokeStyle = isHovered ? '#ffffff' : 'rgba(255, 255, 255, 0.35)';
    ctx.lineWidth = isHovered ? 2.2 : 1.2;
    ctx.stroke();

    // Internal Mechanism Detailing (Semi-transparent top shield preview)
    ctx.save();
    // Glowing micro-LED status board inside chassis
    const boardGlow = 0.5 + Math.sin(timeSec * 5) * 0.3;
    ctx.fillStyle = `rgba(16, 185, 129, ${boardGlow})`;
    ctx.fillRect(baseCx - 6, baseCy - amrHeight - 3, 1.8, 1.8);
    ctx.fillStyle = `rgba(217, 119, 6, ${boardGlow})`;
    ctx.fillRect(baseCx - 3, baseCy - amrHeight - 3, 1.8, 1.8);

    // Cooling vent grille louvers
    ctx.strokeStyle = '#0f172a';
    ctx.lineWidth = 1;
    for (let l = -5; l <= 5; l += 2.5) {
      ctx.beginPath();
      ctx.moveTo(baseCx + l, baseCy - amrHeight + 2);
      ctx.lineTo(baseCx + l + 1.5, baseCy - amrHeight + 4);
      ctx.stroke();
    }
    ctx.restore();

    // E. Center Corkscrew Lift Turntable (Rotates for cargo alignment)
    const turnRadius = chW * 0.28;
    ctx.beginPath();
    ctx.ellipse(baseCx, baseCy - amrHeight - 2, turnRadius, turnRadius * 0.5, 0, 0, Math.PI * 2);
    ctx.fillStyle = '#0f172a';
    ctx.fill();
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1.2;
    ctx.stroke();

    // Radial Textured Anti-Slip Friction Pads
    ctx.fillStyle = '#f59e0b';
    for (let i = 0; i < 4; i++) {
      const ang = (timeSec * (r.current_task_id ? 0 : 0.5)) + (i * Math.PI * 0.5);
      const px = baseCx + Math.cos(ang) * (turnRadius * 0.58);
      const py = (baseCy - amrHeight - 2) + Math.sin(ang) * (turnRadius * 0.29);
      ctx.fillRect(px - 1.2, py - 1.2, 2.4, 2.4);
    }

    // 4. Cargo Pallet & Container on Top (If transporting logistics order)
    if (r.current_task_id) {
      const cargoW = 19;
      const cargoH = 16;
      const settleY = p ? p.cargoSettle : 0;
      const cy = baseCy - amrHeight - 5 + settleY;

      // Timber Pallet Deck under cargo
      ctx.fillStyle = '#92400e';
      ctx.fillRect(baseCx - cargoW * 0.52, cy - 2.5, cargoW * 1.04, 3.5);

      // Cardboard Shipping Carton
      ctx.beginPath();
      ctx.rect(baseCx - cargoW * 0.5, cy - cargoH, cargoW, cargoH);
      ctx.fillStyle = '#d97706';
      ctx.fill();
      ctx.strokeStyle = '#78350f';
      ctx.lineWidth = 1.0;
      ctx.stroke();

      // Industrial strapping band
      ctx.strokeStyle = '#d97706';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(baseCx, cy - cargoH);
      ctx.lineTo(baseCx, cy);
      ctx.stroke();

      // Shipping label barcode
      ctx.fillStyle = '#ffffff';
      ctx.fillRect(baseCx - cargoW * 0.4, cy - cargoH + 3, 7, 4);
    }

    // 5. Distinct 360° LiDAR Puck with Dynamic Ground Scan Fan Beam
    if (this.enablePhysics && p && !isFaulted) {
      const lidarAngle = p.lidarAngle;
      const fanSpread = Math.PI * 0.32; // 60-degree forward scan fan
      const beamLen = tw * 0.85;

      // Translucent illuminated ground laser scan cone
      const fanGrad = ctx.createRadialGradient(
        baseCx, baseCy - amrHeight, 4,
        baseCx, baseCy - amrHeight, beamLen
      );
      fanGrad.addColorStop(0, 'rgba(217, 119, 6, 0.35)');
      fanGrad.addColorStop(0.7, 'rgba(217, 119, 6, 0.12)');
      fanGrad.addColorStop(1, 'transparent');

      ctx.save();
      ctx.beginPath();
      ctx.moveTo(baseCx, baseCy - amrHeight - 3);
      ctx.lineTo(
        baseCx + Math.cos(lidarAngle - fanSpread * 0.5) * beamLen,
        (baseCy - amrHeight) + Math.sin(lidarAngle - fanSpread * 0.5) * (beamLen * 0.5)
      );
      ctx.lineTo(
        baseCx + Math.cos(lidarAngle + fanSpread * 0.5) * beamLen,
        (baseCy - amrHeight) + Math.sin(lidarAngle + fanSpread * 0.5) * (beamLen * 0.5)
      );
      ctx.closePath();
      ctx.fillStyle = fanGrad;
      ctx.fill();

      // Center laser beam line
      ctx.beginPath();
      ctx.moveTo(baseCx, baseCy - amrHeight - 3);
      ctx.lineTo(
        baseCx + Math.cos(lidarAngle) * beamLen,
        (baseCy - amrHeight) + Math.sin(lidarAngle) * (beamLen * 0.5)
      );
      ctx.strokeStyle = 'rgba(217, 119, 6, 0.65)';
      ctx.lineWidth = 1.4;
      ctx.stroke();
      ctx.restore();

      // Solid Miniature LiDAR Puck Turret (Black cylinder with optical glass slit)
      ctx.beginPath();
      ctx.ellipse(baseCx, baseCy - amrHeight - 3, 3.5, 1.8, 0, 0, Math.PI * 2);
      ctx.fillStyle = '#020617';
      ctx.fill();
      ctx.strokeStyle = '#d97706';
      ctx.lineWidth = 1.2;
      ctx.stroke();

      // Rotating scan head optic
      ctx.beginPath();
      ctx.arc(
        baseCx + Math.cos(lidarAngle) * 1.5,
        (baseCy - amrHeight - 3) + Math.sin(lidarAngle) * 0.8,
        1.1, 0, Math.PI * 2
      );
      ctx.fillStyle = '#fbbf24';
      ctx.fill();
    }

    // 6. Floating Holographic Telemetry Tag & Hazard Indicator
    const tagY = baseCy - amrHeight - (r.current_task_id ? 28 : 12);
    const numId = r.id.replace('AMR-', '');

    if (isFaulted) {
      ctx.fillStyle = '#ef4444';
      ctx.font = 'bold 10px "JetBrains Mono", monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`⚠️ FAULT: AMR-${numId}`, baseCx, tagY - 6);
      ctx.fillStyle = '#f59e0b';
      ctx.font = 'bold 8.5px "Inter", sans-serif';
      ctx.fillText('E-STOP ACTIVE', baseCx, tagY + 4);
    } else {
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 10px "JetBrains Mono", monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.shadowColor = 'rgba(0, 0, 0, 0.9)';
      ctx.shadowBlur = 4;
      ctx.fillText(`AMR-${numId}`, baseCx, tagY);
      ctx.shadowBlur = 0;
    }
  }

  // =========================================================================
  // ROLLER CONVEYOR STATIONS & ANDON TOWER LIGHTS
  // =========================================================================
  drawIsoStation(ctx, loc, label, color, timeSec) {
    const tw = this.isoTileW;
    const th = this.isoTileH;
    const pt = this.toIso(loc[0], loc[1]);
    const sx = pt.x;
    const sy = pt.y;

    // 1. Heavy Anchored Foundation Pad
    this.drawIsoTile(ctx, sx, sy - 3, tw * 0.96, th * 0.96, '#0f172a', color, 1.8);

    // 2. Motorized Steel Conveyor Bed with Metallic Cylindrical Rollers
    const numRollers = 4;
    for (let i = 0; i < numRollers; i++) {
      const offset = (i / (numRollers - 1)) - 0.5;
      const rx = sx + offset * (tw * 0.5);
      const ry = sy + th * 0.5 + offset * (th * 0.5) - 4;

      ctx.beginPath();
      ctx.ellipse(rx, ry, 6.5, 2.8, Math.PI * 0.25, 0, Math.PI * 2);
      ctx.fillStyle = '#475569'; // Zinc-plated roller
      ctx.fill();
      ctx.strokeStyle = '#94a3b8';
      ctx.lineWidth = 0.9;
      ctx.stroke();
    }

    // 3. Safety Yellow Steel Bollards
    const bollardX = sx - tw * 0.38;
    const bollardY = sy + th * 0.45;
    ctx.fillStyle = '#eab308';
    ctx.fillRect(bollardX, bollardY - 11, 3.5, 11);
    ctx.beginPath();
    ctx.arc(bollardX + 1.75, bollardY - 11, 1.75, Math.PI, 0);
    ctx.fill();

    // 4. Industrial 3-Tier Andon Stack Light Tower (Red, Amber, Green)
    const poleX = sx + tw * 0.38;
    const poleY = sy + th * 0.4;
    const poleH = 26;

    // Pole
    ctx.strokeStyle = '#64748b';
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.moveTo(poleX, poleY);
    ctx.lineTo(poleX, poleY - poleH);
    ctx.stroke();

    // Light stack tiers
    // Top: Red
    ctx.fillStyle = '#ef4444';
    ctx.fillRect(poleX - 2.5, poleY - poleH, 5, 3.5);
    // Middle: Amber
    ctx.fillStyle = '#f59e0b';
    ctx.fillRect(poleX - 2.5, poleY - poleH + 4.5, 5, 3.5);
    // Bottom: Active Green (glowing)
    const greenGlow = 0.65 + Math.sin(timeSec * 6) * 0.35;
    ctx.fillStyle = `rgba(16, 185, 129, ${greenGlow})`;
    ctx.fillRect(poleX - 2.5, poleY - poleH + 9, 5, 3.5);

    // 5. Floor Glow Halo
    const grad = ctx.createRadialGradient(sx, sy + th * 0.5, 2, sx, sy + th * 0.5, tw * 0.65);
    grad.addColorStop(0, color === '#10b981' ? 'rgba(16, 185, 129, 0.28)' : 'rgba(6, 182, 212, 0.28)');
    grad.addColorStop(1, 'transparent');
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.ellipse(sx, sy + th * 0.5, tw * 0.65, th * 0.65, 0, 0, Math.PI * 2);
    ctx.fill();

    // 6. Holographic Station Header
    ctx.fillStyle = color;
    ctx.font = 'bold 10px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`${label} DOCK`, sx, sy - 11);
  }

  // Human Safety Zone (3D Volumetric Caution Cuboid)
  drawIsoSafetyZone(ctx, bounds, timeSec) {
    const [minX, minY, maxX, maxY] = bounds;
    const p1 = this.toIso(minX, minY);
    const p2 = this.toIso(maxX + 1, minY);
    const p3 = this.toIso(maxX + 1, maxY + 1);
    const p4 = this.toIso(minX, maxY + 1);

    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.lineTo(p3.x, p3.y);
    ctx.lineTo(p4.x, p4.y);
    ctx.closePath();

    const alpha = 0.08 + Math.sin(timeSec * 3.5) * 0.04;
    ctx.fillStyle = `rgba(245, 158, 11, ${alpha})`;
    ctx.fill();

    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 1.6;
    ctx.setLineDash([6, 6]);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Congestion Heatmap 3D Glow
  drawIsoHeatmapCell(ctx, gx, gy, intensity) {
    const pt = this.toIso(gx, gy);
    const tw = this.isoTileW;
    const th = this.isoTileH;
    const rad = tw * 0.75;

    const grad = ctx.createRadialGradient(pt.x, pt.y + th * 0.5, 2, pt.x, pt.y + th * 0.5, rad);
    const alpha = Math.min(0.68, intensity * 0.24);
    grad.addColorStop(0, `rgba(244, 63, 94, ${alpha})`);
    grad.addColorStop(0.5, `rgba(245, 158, 11, ${alpha * 0.5})`);
    grad.addColorStop(1, 'transparent');

    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.ellipse(pt.x, pt.y + th * 0.5, rad, rad * 0.5, 0, 0, Math.PI * 2);
    ctx.fill();
  }

  // Forward Intent Paths (Trajectory Ribbons)
  drawIsoIntentPath(ctx, r) {
    if (!r.intent || r.intent.length === 0) return;
    const p = this.robotPhysics[r.id];
    const currPt = this.toIso(p ? p.x : r.position[0], p ? p.y : r.position[1]);
    const th = this.isoTileH;

    ctx.beginPath();
    ctx.moveTo(currPt.x, currPt.y + th * 0.5);

    r.intent.forEach(cell => {
      const pt = this.toIso(cell[0], cell[1]);
      ctx.lineTo(pt.x, pt.y + th * 0.5);
    });

    ctx.strokeStyle = r.state === 'YIELDING' ? 'rgba(245, 158, 11, 0.7)' :
                      r.state === 'WAITING' ? 'rgba(244, 63, 94, 0.7)' :
                      'rgba(6, 182, 212, 0.6)';
    ctx.lineWidth = 2.2;
    ctx.setLineDash([4, 4]);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Malfunction Particles (Volumetric Smoke Puffs & Electrical Sparks)
  drawIsoFaultParticles(ctx) {
    const th = this.isoTileH;

    this.faultParticles.forEach(fp => {
      const basePt = this.toIso(fp.gx, fp.gy);
      const px = basePt.x + fp.x;
      const py = basePt.y + th * 0.5 + fp.y - fp.z;

      if (fp.type === 'smoke') {
        ctx.beginPath();
        ctx.arc(px, py, fp.radius, 0, Math.PI * 2);
        ctx.fillStyle = `rgba(${fp.gray}, ${fp.gray}, ${fp.gray + 6}, ${fp.alpha})`;
        ctx.fill();
      } else if (fp.type === 'spark') {
        ctx.beginPath();
        ctx.arc(px, py, fp.radius, 0, Math.PI * 2);
        ctx.fillStyle = fp.color;
        ctx.shadowColor = fp.color;
        ctx.shadowBlur = 6;
        ctx.fill();
        ctx.shadowBlur = 0;
      }
    });
  }

  // =========================================================================
  // MODERN 2D INDUSTRIAL DIGITAL TWIN ENGINE (Clean, Advanced, Simple)
  // Inspired by modern CAD / SCADA robotics suites (Foxglove, OTTO, Siemens)
  // =========================================================================
  render2D(ctx, timeSec) {
    const snap = this.latestSnapshot;
    if (!snap || !snap.warehouse) return;

    const gw = this.gridWidth;
    const gh = this.gridHeight;
    const c = this.cellPx;
    const ox = this.origin2DX || 0;
    const oy = this.origin2DY || 0;

    // 1. Sleek Technical Floor & Blueprint Grid
    this.draw2DFloor(ctx, ox, oy, gw, gh, c, timeSec);

    // 2. Human Safety Derating Zones (ISO 3691-4)
    if (this.showHumanZones && snap.warehouse.human_zones) {
      this.draw2DHumanZones(ctx, ox, oy, snap.warehouse.human_zones, c, timeSec);
    }

    // 3. Inbound / Outbound Logistics Docking Stations
    this.draw2DStations(ctx, ox, oy, snap, c, timeSec);

    // 4. Modular Industrial Storage Racks
    if (snap.warehouse.obstacles) {
      this.draw2DStorageRacks(ctx, ox, oy, snap.warehouse.obstacles, c);
    }

    // 5. Calibrated Spatial Congestion Thermal Heatmap
    if (this.showHeatmap && snap.congestion_heatmap && snap.congestion_heatmap.cells) {
      this.draw2DHeatmap(ctx, ox, oy, snap.congestion_heatmap.cells, c);
    }

    // 6. Forward Intent Navigation Trajectories
    if (this.showIntentPaths && snap.robots) {
      this.draw2DIntentPaths(ctx, ox, oy, snap.robots, c, timeSec);
    }

    // 7. Autonomous Mobile Robots (AMRs) with Physics & Laser Sensor Cone
    if (snap.robots) {
      this.draw2DAMRs(ctx, ox, oy, snap.robots, c, timeSec);
    }

    // 8. Minimalist Technical Scale & Depot Info
    this.draw2DOverlayInfo(ctx, ox, oy, gw, gh, c);
  }

  // 1. Sleek Matte Industrial Concrete Base & Precision CAD Grid
  draw2DFloor(ctx, ox, oy, gw, gh, c, timeSec) {
    const totalW = gw * c;
    const totalH = gh * c;

    // A. Clean, deep slate technical foundation backdrop
    ctx.fillStyle = '#0a0e16';
    ctx.fillRect(ox - 16, oy - 16, totalW + 32, totalH + 32);

    // Technical foundation perimeter curb
    ctx.strokeStyle = '#1a2333';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(ox - 16, oy - 16, totalW + 32, totalH + 32);

    // Active facility floor area
    ctx.fillStyle = '#0f1520';
    ctx.fillRect(ox, oy, totalW, totalH);

    // B. Subtle high-throughput transit corridors (soft ambient distinction)
    ctx.fillStyle = 'rgba(255, 255, 255, 0.012)';
    // East-West main arteries (rows 6, 7 and 13, 14)
    ctx.fillRect(ox, oy + 6 * c, totalW, 2 * c);
    ctx.fillRect(ox, oy + 13 * c, totalW, 2 * c);
    // North-South crossing aisles
    ctx.fillRect(ox + 0 * c, oy, 2 * c, totalH);
    ctx.fillRect(ox + 7 * c, oy, 2 * c, totalH);
    ctx.fillRect(ox + 13 * c, oy, 2 * c, totalH);
    ctx.fillRect(ox + 19 * c, oy, 2 * c, totalH);

    // C. Crisp CAD Blueprint Grid Lines (Clean, low contrast, zero eye fatigue)
    // Minor single-meter grid lines
    ctx.beginPath();
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.04)';
    ctx.lineWidth = 0.5;
    for (let x = 0; x <= gw; x++) {
      ctx.moveTo(ox + x * c, oy);
      ctx.lineTo(ox + x * c, oy + totalH);
    }
    for (let y = 0; y <= gh; y++) {
      ctx.moveTo(ox, oy + y * c);
      ctx.lineTo(ox + totalW, oy + y * c);
    }
    ctx.stroke();

    // Major modular grid expansion seams (every 4 meters)
    ctx.beginPath();
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.08)';
    ctx.lineWidth = 1;
    for (let x = 0; x <= gw; x += 4) {
      ctx.moveTo(ox + x * c, oy);
      ctx.lineTo(ox + x * c, oy + totalH);
    }
    for (let y = 0; y <= gh; y += 4) {
      ctx.moveTo(ox, oy + y * c);
      ctx.lineTo(ox + totalW, oy + y * c);
    }
    ctx.stroke();

    // D. Floor Guidance Markings (Clean dashed aisle centerlines)
    if (this.showFloorDetails) {
      ctx.beginPath();
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.12)';
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 8]);
      // East-West central travel lines
      ctx.moveTo(ox + 2 * c, oy + 7 * c);
      ctx.lineTo(ox + (gw - 2) * c, oy + 7 * c);
      ctx.moveTo(ox + 2 * c, oy + 14 * c);
      ctx.lineTo(ox + (gw - 2) * c, oy + 14 * c);
      // North-South central travel lines
      ctx.moveTo(ox + 8 * c, oy + 2 * c);
      ctx.lineTo(ox + 8 * c, oy + (gh - 2) * c);
      ctx.moveTo(ox + 14 * c, oy + 2 * c);
      ctx.lineTo(ox + 14 * c, oy + (gh - 2) * c);
      ctx.moveTo(ox + 20 * c, oy + 2 * c);
      ctx.lineTo(ox + 20 * c, oy + (gh - 2) * c);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // E. Outer Wall Boundary
    ctx.strokeStyle = '#222d3d';
    ctx.lineWidth = 1.2;
    ctx.strokeRect(ox, oy, totalW, totalH);

    // Minimalist corner precision crosshairs
    const crossSize = 5;
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.3)';
    ctx.lineWidth = 1;
    [[ox, oy], [ox + totalW, oy], [ox, oy + totalH], [ox + totalW, oy + totalH]].forEach(([cx, cy]) => {
      ctx.beginPath();
      ctx.moveTo(cx - crossSize, cy); ctx.lineTo(cx + crossSize, cy);
      ctx.moveTo(cx, cy - crossSize); ctx.lineTo(cx, cy + crossSize);
      ctx.stroke();
    });
  }

  // 2. Human Safety Zones (ISO 3691-4 & ANSI B56.5 Compliance)
  draw2DHumanZones(ctx, ox, oy, humanZones, c, timeSec) {
    if (!humanZones) return;

    humanZones.forEach(hz => {
      if (!hz.active) return;
      const [minX, minY, maxX, maxY] = hz.bounds;
      const rx = ox + minX * c;
      const ry = oy + minY * c;
      const rw = (maxX - minX + 1) * c;
      const rh = (maxY - minY + 1) * c;

      // Soft, transparent amber area tint
      ctx.fillStyle = 'rgba(245, 158, 11, 0.04)';
      ctx.fillRect(rx, ry, rw, rh);

      // Clean, elegant dashed safety perimeter
      ctx.strokeStyle = 'rgba(245, 158, 11, 0.6)';
      ctx.lineWidth = 1.4;
      ctx.setLineDash([5, 5]);
      ctx.strokeRect(rx, ry, rw, rh);
      ctx.setLineDash([]);

      // Precision corner L-brackets
      const bLen = 8;
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 2;
      [[rx, ry], [rx + rw, ry], [rx, ry + rh], [rx + rw, ry + rh]].forEach(([cx, cy], i) => {
        const dx = (i % 2 === 0) ? bLen : -bLen;
        const dy = (i < 2) ? bLen : -bLen;
        ctx.beginPath();
        ctx.moveTo(cx + dx, cy); ctx.lineTo(cx, cy); ctx.lineTo(cx, cy + dy);
        ctx.stroke();
      });

      // Subtle watermark badge
      ctx.fillStyle = 'rgba(245, 158, 11, 0.75)';
      ctx.font = '600 8.5px "Inter", -apple-system, sans-serif';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'top';
      ctx.fillText('⚠ ISO 3691-4 SAFETY ZONE', rx + 6, ry + 6);
    });
  }

  // 3. Logistics Inbound / Outbound Stations
  draw2DStations(ctx, ox, oy, snap, c, timeSec) {
    // Inbound Pickup Stations (Emerald theme)
    if (snap.warehouse.pickup_stations) {
      snap.warehouse.pickup_stations.forEach((s, idx) => {
        const [sx, sy] = s.location;
        const px = ox + sx * c;
        const py = oy + sy * c;

        // Recessed docking floor pad
        ctx.fillStyle = 'rgba(16, 185, 129, 0.07)';
        ctx.beginPath();
        ctx.roundRect(px + 2, py + 2, c - 4, c - 4, 3);
        ctx.fill();

        // Technical docking perimeter
        ctx.strokeStyle = 'rgba(16, 185, 129, 0.55)';
        ctx.lineWidth = 1.2;
        ctx.stroke();

        // Docking direction indicator
        ctx.strokeStyle = 'rgba(16, 185, 129, 0.35)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(px + c * 0.35, py + c * 0.38);
        ctx.lineTo(px + c * 0.5, py + c * 0.5);
        ctx.lineTo(px + c * 0.35, py + c * 0.62);
        ctx.stroke();

        // Station Pill Badge
        ctx.fillStyle = 'rgba(16, 185, 129, 0.16)';
        ctx.beginPath();
        ctx.roundRect(px + 4, py + c * 0.5 - 6, c - 8, 12, 2);
        ctx.fill();

        ctx.fillStyle = '#34d399';
        ctx.font = '700 8.5px "Inter", sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(`IN-${idx + 1}`, px + c * 0.5, py + c * 0.5);
      });
    }

    // Outbound Dropoff Stations (Amber theme)
    if (snap.warehouse.dropoff_stations) {
      snap.warehouse.dropoff_stations.forEach((s, idx) => {
        const [sx, sy] = s.location;
        const px = ox + sx * c;
        const py = oy + sy * c;

        ctx.fillStyle = 'rgba(217, 119, 6, 0.07)';
        ctx.beginPath();
        ctx.roundRect(px + 2, py + 2, c - 4, c - 4, 3);
        ctx.fill();

        ctx.strokeStyle = 'rgba(217, 119, 6, 0.55)';
        ctx.lineWidth = 1.2;
        ctx.stroke();

        // Docking direction indicator
        ctx.strokeStyle = 'rgba(217, 119, 6, 0.35)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(px + c * 0.35, py + c * 0.38);
        ctx.lineTo(px + c * 0.5, py + c * 0.5);
        ctx.lineTo(px + c * 0.35, py + c * 0.62);
        ctx.stroke();

        ctx.fillStyle = 'rgba(217, 119, 6, 0.16)';
        ctx.beginPath();
        ctx.roundRect(px + 4, py + c * 0.5 - 6, c - 8, 12, 2);
        ctx.fill();

        ctx.fillStyle = '#fbbf24';
        ctx.font = '700 8.5px "Inter", sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(`OUT-${idx + 1}`, px + c * 0.5, py + c * 0.5);
      });
    }
  }

  // 4. Modular Industrial Storage Racks (Clean, High-Tech Structured Bays)
  draw2DStorageRacks(ctx, ox, oy, obstacles, c) {
    if (!obstacles || obstacles.length === 0) return;

    // Render each storage cell as a precision engineered warehouse bay
    obstacles.forEach(([gx, gy]) => {
      const rx = ox + gx * c;
      const ry = oy + gy * c;
      const pad = 2;
      const w = c - pad * 2;
      const h = c - pad * 2;

      // Reset fillStyle on EVERY cell (ensures zero color leakage)
      ctx.fillStyle = '#131923';
      ctx.beginPath();
      ctx.roundRect(rx + pad, ry + pad, w, h, 3);
      ctx.fill();

      // Sleek technical frame border
      ctx.strokeStyle = '#222d3e';
      ctx.lineWidth = 1;
      ctx.stroke();

      // Recessed pallet compartment slot
      const slotPad = 3;
      ctx.fillStyle = '#192230';
      ctx.beginPath();
      ctx.roundRect(rx + pad + slotPad, ry + pad + slotPad, w - slotPad * 2, h - slotPad * 2, 2);
      ctx.fill();

      // Clean, muted industrial payload container
      const boxPad = 5;
      const bw = w - boxPad * 2;
      const bh = h - boxPad * 2;

      // Soft shadow under cargo unit
      ctx.fillStyle = 'rgba(0, 0, 0, 0.3)';
      ctx.fillRect(rx + pad + boxPad + 1, ry + pad + boxPad + 1, bw, bh);

      // Cargo body (subtle alternating pallet tone)
      const isTopRack = (gy <= 5);
      ctx.fillStyle = isTopRack ? '#222f42' : '#263449';
      ctx.fillRect(rx + pad + boxPad, ry + pad + boxPad, bw, bh);

      // Central steel strapping line
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.15)';
      ctx.lineWidth = 0.8;
      ctx.beginPath();
      ctx.moveTo(rx + pad + boxPad + bw * 0.5, ry + pad + boxPad);
      ctx.lineTo(rx + pad + boxPad + bw * 0.5, ry + pad + boxPad + bh);
      ctx.stroke();

      // Discrete corner column protection marks (micro 2px accents)
      ctx.fillStyle = '#38bdf8';
      ctx.fillRect(rx + pad, ry + pad, 2, 2);
      ctx.fillRect(rx + pad + w - 2, ry + pad, 2, 2);
      ctx.fillRect(rx + pad, ry + pad + h - 2, 2, 2);
      ctx.fillRect(rx + pad + w - 2, ry + pad + h - 2, 2, 2);
    });

    // Elegant, discrete rack column headers
    const rackHeaders = [
      { x: 3.5, y: 1.45, name: 'BAY A' },
      { x: 9.5, y: 1.45, name: 'BAY B' },
      { x: 15.5, y: 1.45, name: 'BAY C' },
      { x: 21.5, y: 1.45, name: 'BAY D' },
      { x: 3.5, y: 13.25, name: 'BAY E' },
      { x: 9.5, y: 13.25, name: 'BAY F' },
      { x: 15.5, y: 13.25, name: 'BAY G' },
      { x: 21.5, y: 13.25, name: 'BAY H' },
    ];
    ctx.fillStyle = 'rgba(148, 163, 184, 0.4)';
    ctx.font = '600 8.5px "Inter", -apple-system, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    rackHeaders.forEach(rh => {
      ctx.fillText(rh.name, ox + rh.x * c, oy + rh.y * c);
    });
  }

  // 5. Calibrated Spatial Congestion Thermal Heatmap (Subtle, non-intrusive)
  draw2DHeatmap(ctx, ox, oy, cells, c) {
    if (!cells || cells.length === 0) return;

    cells.forEach(cell => {
      const cx = ox + (cell.x + 0.5) * c;
      const cy = oy + (cell.y + 0.5) * c;
      const rad = c * 1.25;

      // Soft thermal haze with capped maximum opacity (prevents muddy spots)
      const alpha = Math.min(0.16, cell.intensity * 0.06);
      const grad = ctx.createRadialGradient(cx, cy, 2, cx, cy, rad);
      grad.addColorStop(0, `rgba(244, 63, 94, ${alpha})`);
      grad.addColorStop(0.5, `rgba(245, 158, 11, ${alpha * 0.4})`);
      grad.addColorStop(1, 'transparent');

      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(cx, cy, rad, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  // 6. Forward Intent Navigation Trajectories (Smooth gradient paths)
  draw2DIntentPaths(ctx, ox, oy, robots, c, timeSec) {
    if (!robots) return;

    Object.values(robots).forEach(r => {
      if (!r.intent || r.intent.length === 0) return;
      const p = this.robotPhysics[r.id];
      const startX = ox + ((p ? p.x : r.position[0]) + 0.5) * c;
      const startY = oy + ((p ? p.y : r.position[1]) + 0.5) * c;

      let pathColor = 'rgba(6, 182, 212, 0.45)';
      let endGlow = 'rgba(6, 182, 212, 0.75)';
      if (r.state === 'YIELDING') {
        pathColor = 'rgba(245, 158, 11, 0.5)';
        endGlow = 'rgba(245, 158, 11, 0.8)';
      } else if (r.state === 'WAITING') {
        pathColor = 'rgba(244, 63, 94, 0.5)';
        endGlow = 'rgba(244, 63, 94, 0.8)';
      }

      // Smooth path ribbon
      ctx.beginPath();
      ctx.moveTo(startX, startY);
      r.intent.forEach(pt => {
        ctx.lineTo(ox + (pt[0] + 0.5) * c, oy + (pt[1] + 0.5) * c);
      });
      ctx.strokeStyle = pathColor;
      ctx.lineWidth = 1.6;
      ctx.setLineDash([4, 4]);
      ctx.stroke();
      ctx.setLineDash([]);

      // Destination target ring with glowing core
      const dest = r.intent[r.intent.length - 1];
      const destX = ox + (dest[0] + 0.5) * c;
      const destY = oy + (dest[1] + 0.5) * c;

      ctx.beginPath();
      ctx.arc(destX, destY, 3.5, 0, Math.PI * 2);
      ctx.fillStyle = endGlow;
      ctx.fill();

      ctx.beginPath();
      ctx.arc(destX, destY, 6.5, 0, Math.PI * 2);
      ctx.strokeStyle = endGlow;
      ctx.lineWidth = 1;
      ctx.stroke();
    });
  }

  // 7. Autonomous Mobile Robots (AMRs) with Realistic Geometry & Laser Scanner Field
  draw2DAMRs(ctx, ox, oy, robots, c, timeSec) {
    Object.values(robots).forEach(r => {
      const p = this.robotPhysics[r.id];
      const isFaulted = !!this.simulatedFaults[r.id];
      const isHovered = (this.mouse.hoveredRobot && this.mouse.hoveredRobot.id === r.id);

      const rx = ox + ((p ? (p.x + (p.recoilX || 0)) : r.position[0]) + 0.5) * c;
      const ry = oy + ((p ? (p.y + (p.recoilY || 0)) : r.position[1]) + 0.5) * c;
      const angle = p ? p.angle : 0;

      // Proportional chassis metrics
      const length = c * 0.80;
      const width = c * 0.66;
      const radius = 4;

      ctx.save();
      ctx.translate(rx, ry);

      // A. Soft Floor Occlusion Shadow
      ctx.save();
      ctx.fillStyle = 'rgba(0, 0, 0, 0.45)';
      ctx.beginPath();
      ctx.roundRect(-length * 0.5 + 1.5, -width * 0.5 + 2.5, length, width, radius);
      ctx.fill();
      ctx.restore();

      // Rotate chassis according to robot heading
      ctx.rotate(angle);

      // B. Dynamic Optical LiDAR Sensor Cone (Forward perception visualization)
      if (!isFaulted && r.state !== 'IDLE') {
        const coneLen = c * 1.05;
        const halfAngle = 0.42; // ~24° each side
        const grad = ctx.createRadialGradient(0, 0, 2, 0, 0, coneLen);
        grad.addColorStop(0, 'rgba(6, 182, 212, 0.11)');
        grad.addColorStop(1, 'transparent');

        ctx.fillStyle = grad;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.arc(0, 0, coneLen, -halfAngle, halfAngle);
        ctx.closePath();
        ctx.fill();
      }

      // C. Drive Wheels / Ground Contact Casters
      ctx.fillStyle = '#080c13';
      ctx.fillRect(-length * 0.22, -width * 0.5 - 1.5, length * 0.44, 2);
      ctx.fillRect(-length * 0.22, width * 0.5 - 0.5, length * 0.44, 2);

      // D. Main Low-Profile AMR Chassis Body
      ctx.fillStyle = '#161e2b';
      ctx.beginPath();
      ctx.roundRect(-length * 0.5, -width * 0.5, length, width, radius);
      ctx.fill();

      // Status indicator edge illumination
      let statusColor = '#06b6d4'; // Cyan default
      if (isFaulted) statusColor = '#ef4444';
      else if (r.state === 'YIELDING') statusColor = '#f59e0b';
      else if (r.state === 'WAITING') statusColor = '#f43f5e';
      else if (r.state === 'MOVING') statusColor = '#10b981';
      else if (r.state === 'IDLE') statusColor = '#64748b';

      ctx.strokeStyle = statusColor;
      ctx.lineWidth = 1.4;
      ctx.stroke();

      // E. Forward Headlight Directional LED Bar
      ctx.fillStyle = statusColor;
      ctx.fillRect(length * 0.5 - 2.5, -width * 0.35, 2.5, width * 0.7);

      // F. Recessed Cargo Turntable Top Plate
      ctx.fillStyle = '#202a3a';
      ctx.beginPath();
      ctx.arc(0, 0, width * 0.32, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#2f3d53';
      ctx.lineWidth = 0.8;
      ctx.stroke();

      // G. Center Optical LiDAR Scanner Puck
      ctx.fillStyle = '#0f172a';
      ctx.beginPath();
      ctx.arc(0, 0, 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#38bdf8';
      ctx.beginPath();
      ctx.arc(0, 0, 1.5, 0, Math.PI * 2);
      ctx.fill();

      ctx.restore(); // Undo rotation and translation

      // H. Upright Clean Number Badge (Always readable, un-rotated)
      const num = r.id.replace('AMR-', '');
      ctx.fillStyle = 'rgba(15, 23, 42, 0.88)';
      ctx.beginPath();
      ctx.roundRect(rx - 8.5, ry - 6.5, 17, 13, 3);
      ctx.fill();
      ctx.strokeStyle = statusColor;
      ctx.lineWidth = 1;
      ctx.stroke();

      ctx.fillStyle = '#ffffff';
      ctx.font = '700 9.5px "Inter", -apple-system, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(num, rx, ry + 0.5);

      // I. Precision CAD Targeting Reticle (if hovered)
      if (isHovered) {
        const bRad = c * 0.65;
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 1.2;
        const bLen = 5;

        // 4 corner reticle brackets
        [[-bRad, -bRad], [bRad, -bRad], [-bRad, bRad], [bRad, bRad]].forEach(([bx, by], idx) => {
          const sx = (idx % 2 === 0) ? bLen : -bLen;
          const sy = (idx < 2) ? bLen : -bLen;
          ctx.beginPath();
          ctx.moveTo(rx + bx + sx, ry + by);
          ctx.lineTo(rx + bx, ry + by);
          ctx.lineTo(rx + bx, ry + by + sy);
          ctx.stroke();
        });

        // Floating telemetry pill above robot
        ctx.fillStyle = 'rgba(15, 23, 42, 0.94)';
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 1;
        const lblText = `${r.id}  •  ${r.state}`;
        ctx.font = '600 10px "Inter", sans-serif';
        const tw = ctx.measureText(lblText).width;
        ctx.beginPath();
        ctx.roundRect(rx - tw * 0.5 - 6, ry - bRad - 18, tw + 12, 16, 3);
        ctx.fill();
        ctx.stroke();

        ctx.fillStyle = '#e2e8f0';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(lblText, rx, ry - bRad - 10);
      }
    });
  }

  // 8. Minimalist Technical Scale & Depot Info Overlay
  draw2DOverlayInfo(ctx, ox, oy, gw, gh, c) {
    const totalW = gw * c;
    const totalH = gh * c;

    ctx.fillStyle = 'rgba(148, 163, 184, 0.35)';
    ctx.font = '600 8.5px "Inter", -apple-system, sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText('BEL FLEET DIGITAL TWIN  |  GRID: 24m × 18m  |  SCALE: 1.0m/CELL', ox, oy + totalH + 6);

    ctx.textAlign = 'right';
    ctx.fillText('PROJECTION: TOP-DOWN ORTHOGONAL  |  ISO 3691-4 COMPLIANT', ox + totalW, oy + totalH + 6);
  }


  // Atmospheric dust motes in warehouse air
  drawAtmosphere(ctx) {
    this.dustParticles.forEach(pt => {
      ctx.beginPath();
      ctx.arc(pt.x, pt.y, pt.radius, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(255, 255, 255, ${pt.alpha})`;
      ctx.fill();
    });
  }
}
