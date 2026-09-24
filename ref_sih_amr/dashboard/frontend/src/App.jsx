import { useState, useEffect, useRef, useMemo } from 'react';
import './App.css';

const WS_URL = 'ws://localhost:8000/ws';

export default function App() {
  const [data, setData] = useState(null);
  const [connected, setConnected] = useState(false);
  const [selectedScenario, setSelectedScenario] = useState('S1_Normal');
  const [activeRobotId, setActiveRobotId] = useState(null);
  const [simRate, setSimRate] = useState(0.8);
  const [toggles, setToggles] = useState({ lidar: true, paths: true, ruler: false, tags: false });
  const wsRef = useRef(null);

  const changeSpeed = async (rate) => {
    setSimRate(rate);
    try {
      await fetch('http://localhost:8000/api/speed', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ rate })
      });
    } catch (e) { console.error('Speed error:', e); }
  };

  const changeScenario = async (scen) => {
    setSelectedScenario(scen);
    try {
      await fetch('http://localhost:8000/api/scenario', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ scenario: scen })
      });
    } catch (e) { console.error('Scenario error:', e); }
  };

  const connectWs = () => {
    if (wsRef.current && (wsRef.current.readyState === WebSocket.OPEN || wsRef.current.readyState === WebSocket.CONNECTING)) {
      return;
    }
    if (wsRef.current) {
      try {
        wsRef.current.onopen = null;
        wsRef.current.onclose = null;
        wsRef.current.onerror = null;
        wsRef.current.onmessage = null;
        wsRef.current.close();
      } catch (e) { }
      wsRef.current = null;
    }
    try {
      const ws = new WebSocket(WS_URL);
      wsRef.current = ws;
      ws.onopen = () => {
        if (wsRef.current === ws) setConnected(true);
      };
      ws.onclose = () => {
        if (wsRef.current === ws) {
          wsRef.current = null;
          setConnected(false);
        }
      };
      ws.onerror = () => {
        if (wsRef.current === ws) {
          wsRef.current = null;
          setConnected(false);
        }
      };
      ws.onmessage = (e) => {
        if (wsRef.current === ws) {
          try {
            const snap = JSON.parse(e.data);
            setData(snap);
            if (snap.scenario) {
              setSelectedScenario(snap.scenario);
            }
          } catch (err) {
            console.error('Snapshot parse error:', err);
          }
        }
      };
    } catch (err) {
      console.error('WS error:', err);
    }
  };

  useEffect(() => {
    connectWs();
    return () => {
      if (wsRef.current) {
        const ws = wsRef.current;
        wsRef.current = null;
        ws.onopen = null;
        ws.onclose = null;
        ws.onerror = null;
        ws.onmessage = null;
        try { ws.close(); } catch (e) { }
      }
    };
  }, []);

  const killWs = () => {
    if (wsRef.current) {
      const ws = wsRef.current;
      wsRef.current = null;
      ws.onopen = null;
      ws.onclose = null;
      ws.onerror = null;
      ws.onmessage = null;
      try { ws.close(); } catch (e) { }
    }
    setConnected(false);
  };

  const safeData = useMemo(() => {
    if (!data || Object.keys(data).length === 0)
      return { tick: 0, robots: [], tasks: [], metrics: {}, conflicts: [], grid: [] };
    return data;
  }, [data]);

  return (
    <div className="app-container">
      <header className="app-header">
        <div className="header-brand">
          <span className="brand-logo">Convoy</span>
          <div className="brand-divider" />
          <div className="brand-info">
            <h1>AMR Control Panel</h1>
            <div className="brand-sub">Fleet Dispatch &amp; Telemetry</div>
          </div>
        </div>

        <div className="header-right">
          <div className="hdr-chip">
            <span className="label">TICK</span>
            <span className="val">T+{safeData.tick || 0}</span>
          </div>
          <div className={`conn-indicator ${connected ? 'online' : 'offline'}`}>
            <span className="conn-led" />
            {connected ? 'LIVE' : 'OFFLINE'}
          </div>
          {connected
            ? <button className="btn-action danger" onClick={killWs} id="btn-disconnect">Disconnect</button>
            : <button className="btn-action primary" onClick={connectWs} id="btn-connect">Connect</button>
          }
        </div>
      </header>

      <main className="dashboard-body">
        <section className="map-view-container">
          <div className="map-control-bar">
            <div className="map-heading">
              <span>Live Warehouse Map</span>
              <select
                className="map-scenario-select"
                id="map-scenario-select"
                value={selectedScenario}
                onChange={e => changeScenario(e.target.value)}
                title="Select simulation mode / stress test map"
              >
                <option value="S1_Normal">S1 · Normal Warehouse</option>
                <option value="S2_Crossing">S2 · Crossing Traffic</option>
                <option value="S3_Narrow">S3 · Narrow Aisle</option>
                <option value="S4_Blocked">S4 · Blocked Aisle</option>
                <option value="S5_Failure">S5 · Robot Failure</option>
                <option value="S6_CommDelay">S6 · Comm Delay</option>
              </select>
            </div>

            <div className="map-toggles">
              <div className="speed-group">
                <span className="speed-label">SPEED</span>
                {[['0.5×', 1.4], ['1×', 0.8], ['2×', 0.4]].map(([label, rate]) => (
                  <button
                    key={rate}
                    className={`toggle-btn ${simRate === rate ? 'active' : ''}`}
                    onClick={() => changeSpeed(rate)}
                  >{label}</button>
                ))}
              </div>
              {[['LIDAR', 'lidar'], ['PATHS', 'paths'], ['GRID', 'ruler'], ['LABELS', 'tags']].map(([label, key]) => (
                <button
                  key={key}
                  className={`toggle-btn ${toggles[key] ? 'active' : ''}`}
                  onClick={() => setToggles(p => ({ ...p, [key]: !p[key] }))}
                >{label}</button>
              ))}
            </div>
          </div>

          <WarehouseMap
            robots={safeData.robots}
            tasks={safeData.tasks}
            grid={safeData.grid}
            toggles={toggles}
            simRate={simRate}
            activeRobotId={activeRobotId}
            setActiveRobotId={setActiveRobotId}
          />

          <div className="map-status-bar">
            <span>AMRs Active: <span className="stat">{safeData.robots?.filter(r => r.status !== 'OFFLINE').length || 0}</span></span>
            <span>In Transit: <span className="stat">{safeData.tasks?.filter(t => t.status === 'IN_PROGRESS' || t.status === 3).length || 0}</span></span>
            <span>Completed: <span className="stat">{safeData.tasks?.filter(t => t.status === 'COMPLETED' || t.status === 4).length || 0}</span></span>
            <span>Telemetry: <span className={`stat ${connected ? '' : 'warn'}`}>{connected ? 'Streaming' : 'Paused'}</span></span>
          </div>
        </section>

        <aside className="sidebar-container">
          <MetricsPanel metrics={safeData.metrics} />
          <TasksPanel tasks={safeData.tasks} />
          <RobotsPanel robots={safeData.robots} activeRobotId={activeRobotId} setActiveRobotId={setActiveRobotId} />
          <ConflictsPanel conflicts={safeData.conflicts} />
          <BenchmarkPanel scenario={selectedScenario} setScenario={changeScenario} />
        </aside>
      </main>
    </div>
  );
}

/* --------------------------------------------------------------------------
   WAREHOUSE MAP
   -------------------------------------------------------------------------- */
function WarehouseMap({ robots = [], tasks = [], grid = [], toggles, simRate = 0.8, activeRobotId, setActiveRobotId }) {
  const CELL = 40;

  if (!grid || grid.length === 0) {
    return (
      <div className="map-viewport" style={{ alignItems: 'center', justifyContent: 'center', display: 'flex' }}>
        <div style={{ color: 'var(--text-dim)', fontSize: '0.8rem', fontFamily: 'var(--font-mono)' }}>
          Waiting for warehouse map data...
        </div>
      </div>
    );
  }

  const mapH = grid.length;
  const mapW = grid[0].length;
  const width = mapW * CELL;
  const height = mapH * CELL;
  const selectedRobot = activeRobotId
    ? robots.find(r => r.robot_id === activeRobotId)
    : (robots[0] || null);

  return (
    <div className="map-viewport">
      <svg
        viewBox={`0 0 ${width} ${height}`}
        className="warehouse-svg"
        preserveAspectRatio="xMidYMid meet"
      >
        <defs>
          <pattern id="floor-grid" width={CELL} height={CELL} patternUnits="userSpaceOnUse">
            <rect width={CELL} height={CELL} fill="#07090f" stroke="#0e1420" strokeWidth="1" />
            <circle cx={CELL / 2} cy={CELL / 2} r="0.8" fill="#1a2236" />
          </pattern>
          <pattern id="caution" width="10" height="10" patternTransform="rotate(45)" patternUnits="userSpaceOnUse">
            <line x1="0" y1="0" x2="0" y2="10" stroke="#f59e0b" strokeWidth="5" />
            <line x1="5" y1="0" x2="5" y2="10" stroke="#1a0f00" strokeWidth="5" />
          </pattern>
          <filter id="glow-cyan">
            <feGaussianBlur stdDeviation="2" result="blur" />
            <feMerge><feMergeNode in="blur" /><feMergeNode in="SourceGraphic" /></feMerge>
          </filter>
          <filter id="glow-green">
            <feGaussianBlur stdDeviation="2.5" result="blur" />
            <feMerge><feMergeNode in="blur" /><feMergeNode in="SourceGraphic" /></feMerge>
          </filter>
          <radialGradient id="robot-glow" cx="50%" cy="50%" r="50%">
            <stop offset="0%" stopColor="#00d4ff" stopOpacity="0.3" />
            <stop offset="100%" stopColor="#00d4ff" stopOpacity="0" />
          </radialGradient>
        </defs>

        {/* Floor */}
        <rect x="0" y="0" width={width} height={height} fill="url(#floor-grid)" />

        {/* Coordinate rulers + Grid overlay — only when GRID is ON */}
        {toggles.ruler && (
          <g>
            {/* Bright grid lines overlay */}
            {grid[0].map((_, x) => (
              <line key={`gx-${x}`}
                x1={x * CELL} y1={0} x2={x * CELL} y2={height}
                stroke="#293447" strokeWidth="1" />
            ))}
            {grid.map((_, y) => (
              <line key={`gy-${y}`}
                x1={0} y1={y * CELL} x2={width} y2={y * CELL}
                stroke="#293447" strokeWidth="1" />
            ))}
            {/* Coordinate numbers */}
            {grid[0].map((_, x) => (
              <text key={`rx-${x}`} x={x * CELL + CELL / 2} y={10} fontSize="7" fontFamily="JetBrains Mono"
                fill="#64748b" textAnchor="middle" fontWeight="600">{x}</text>
            ))}
            {grid.map((_, y) => (
              <text key={`ry-${y}`} x={5} y={y * CELL + CELL / 2 + 3} fontSize="7" fontFamily="JetBrains Mono"
                fill="#64748b" textAnchor="start" fontWeight="600">{y}</text>
            ))}
          </g>
        )}

        {/* Static environment */}
        {grid.map((row, y) => row.map((cell, x) => {
          const cx = x * CELL, cy = y * CELL;
          if (cell === '#') return (
            <g key={`rack-${x}-${y}`}>
              <rect x={cx + 1} y={cy + 1} width={CELL - 2} height={CELL - 2} fill="#0d1525" stroke="#1e2d47" strokeWidth="1.5" rx="1" />
              <rect x={cx + 3} y={cy + 3} width={CELL / 2 - 4} height={CELL - 6} fill="#141c2b" stroke="#2a3d5c" strokeWidth="1" rx="1" />
              <rect x={cx + CELL / 2 + 1} y={cy + 3} width={CELL / 2 - 4} height={CELL - 6} fill="#101828" stroke="#1e2d47" strokeWidth="1" rx="1" />
              <line x1={cx + 2} y1={cy + 2} x2={cx + CELL - 2} y2={cy + CELL - 2} stroke="#1a2236" strokeWidth="0.5" />
              <line x1={cx + CELL - 2} y1={cy + 2} x2={cx + 2} y2={cy + CELL - 2} stroke="#1a2236" strokeWidth="0.5" />
              {toggles.tags && (
                <text x={cx + CELL / 2} y={cy + CELL / 2 + 2} fontSize="7" fontFamily="JetBrains Mono"
                  fill="#94a3b8" textAnchor="middle" fontWeight="600">{x},{y}</text>
              )}
            </g>
          );

          if (cell === 'P') return (
            <g key={`pk-${x}-${y}`}>
              <rect x={cx + 2} y={cy + 2} width={CELL - 4} height={CELL - 4} fill="rgba(16,185,129,0.08)" stroke="#10b981" strokeWidth="1.5" rx="2" />
              <rect x={cx + 2} y={cy + 2} width={CELL - 4} height={4} fill="url(#caution)" opacity="0.6" />
              <rect x={cx + 2} y={cy + CELL - 6} width={CELL - 4} height={4} fill="url(#caution)" opacity="0.6" />
              <text x={cx + CELL / 2} y={cy + CELL / 2 + 4} fontSize="8" fontFamily="JetBrains Mono" fontWeight="700"
                fill="#10b981" textAnchor="middle" filter="url(#glow-green)">PK</text>
            </g>
          );

          if (cell === 'D') return (
            <g key={`dp-${x}-${y}`}>
              <rect x={cx + 2} y={cy + 2} width={CELL - 4} height={CELL - 4} fill="rgba(0,212,255,0.06)" stroke="#00d4ff" strokeWidth="1.5" rx="2" />
              <rect x={cx + 2} y={cy + 2} width={CELL - 4} height={4} fill="url(#caution)" opacity="0.5" />
              <rect x={cx + 2} y={cy + CELL - 6} width={CELL - 4} height={4} fill="url(#caution)" opacity="0.5" />
              <text x={cx + CELL / 2} y={cy + CELL / 2 + 4} fontSize="8" fontFamily="JetBrains Mono" fontWeight="700"
                fill="#00d4ff" textAnchor="middle" filter="url(#glow-cyan)">DP</text>
            </g>
          );
          return null;
        }))}

        {/* Planned paths */}
        {toggles.paths && robots.map(r => {
          if (!r.planned_path?.length) return null;
          const cur = [r.position[0] * CELL + CELL / 2, r.position[1] * CELL + CELL / 2];
          const pts = [cur, ...r.planned_path.map(p => [p[0] * CELL + CELL / 2, p[1] * CELL + CELL / 2])];
          const col = r.status === 'WAITING' ? '#f59e0b' : '#10b981';
          return (
            <g key={`path-${r.robot_id}`} opacity="0.7">
              <polyline points={pts.map(p => p.join(',')).join(' ')}
                fill="none" stroke={col} strokeWidth="1.5" strokeDasharray="4 3" />
              {r.planned_path.slice(0, 8).map((p, i) => (
                <circle key={i} cx={p[0] * CELL + CELL / 2} cy={p[1] * CELL + CELL / 2} r="1.5" fill={col} />
              ))}
            </g>
          );
        })}

        {/* Robots */}
        {robots.map(r => {
          const cx = r.position[0] * CELL + CELL / 2;
          const cy = r.position[1] * CELL + CELL / 2;
          const heading = r.heading || 0;
          const isWaiting = r.status === 'WAITING';
          const isOffline = r.status === 'OFFLINE';
          const isDegraded = r.status === 'DEGRADED';
          const hasCargo = r.has_cargo || false;
          const isSelected = activeRobotId === r.robot_id;

          const bodyColor = isOffline ? '#1e293b'
            : isDegraded ? '#7f1d1d'
              : isWaiting ? '#78350f'
                : '#0c4a6e';

          const accentColor = isOffline ? '#334155'
            : isDegraded ? '#ef4444'
              : isWaiting ? '#f59e0b'
                : '#00d4ff';

          return (
            <g
              key={r.robot_id}
              style={{ transform: `translate(${cx}px,${cy}px)`, transition: `transform ${simRate}s linear`, cursor: 'pointer' }}
              onClick={() => setActiveRobotId(r.robot_id)}
            >
              {/* Selection glow ring */}
              {isSelected && (
                <circle cx="0" cy="0" r="22" fill="url(#robot-glow)" />
              )}

              {/* LiDAR rays */}
              {toggles.lidar && !isOffline && (
                <g transform={`rotate(${heading})`} style={{ transition: 'transform 0.3s ease' }}>
                  {[-50, -30, -15, 0, 15, 30, 50].map((angle, i) => {
                    const len = isWaiting ? 20 : 38;
                    const rad = (angle - 90) * Math.PI / 180;
                    return (
                      <g key={i}>
                        <line x1="0" y1="-6" x2={Math.cos(rad) * len} y2={Math.sin(rad) * len}
                          stroke={accentColor} strokeWidth="0.8" strokeOpacity="0.4" strokeDasharray="3 2" />
                        <circle cx={Math.cos(rad) * len} cy={Math.sin(rad) * len} r="1.2" fill={accentColor} opacity="0.6" />
                      </g>
                    );
                  })}
                </g>
              )}

              {/* Robot body */}
              <g transform={`rotate(${heading})`} style={{ transition: 'transform 0.3s ease' }}>
                {/* Wheels */}
                <rect x="-16" y="-8" width="4" height="16" fill="#060810" stroke="#1e2d47" strokeWidth="1" rx="1" />
                <rect x="12" y="-8" width="4" height="16" fill="#060810" stroke="#1e2d47" strokeWidth="1" rx="1" />

                {/* Chassis */}
                <rect x="-12" y="-13" width="24" height="26" fill={bodyColor} stroke={accentColor} strokeWidth="1.5" rx="2" />

                {/* Front accent stripe */}
                <rect x="-12" y="-13" width="24" height="4" fill={accentColor} opacity="0.5" rx="2" />

                {/* Direction arrow */}
                <polygon points="0,-11 -4,-6 4,-6" fill={accentColor} opacity="0.9" />

                {/* Sensor puck */}
                <circle cx="0" cy="-3" r="4" fill="#060810" stroke={accentColor} strokeWidth="1" />
                <circle cx="0" cy="-3" r="1.5" fill={accentColor} opacity="0.8" />

                {/* Cargo */}
                {hasCargo && (
                  <g transform="translate(0,6)">
                    <rect x="-9" y="-4" width="18" height="9" fill="#7c2d12" stroke="#ea580c" strokeWidth="1" rx="1" />
                    <rect x="-7" y="-3" width="14" height="7" fill="#c2410c" opacity="0.7" rx="1" />
                  </g>
                )}
              </g>

              {/* ID tag — always upright */}
              <g transform="translate(0, 19)">
                <rect x="-14" y="-6" width="28" height="12" fill="#07090f" stroke={isSelected ? accentColor : '#1e2d47'} strokeWidth="1" rx="2" />
                <text x="0" y="3.5" fontSize="7" fontFamily="JetBrains Mono" fontWeight="700"
                  fill={isSelected ? accentColor : '#64748b'} textAnchor="middle">
                  {r.robot_id.replace('robot-', 'AMR-')}
                </text>
              </g>
            </g>
          );
        })}
      </svg>

      {/* OSD Telemetry */}
      {selectedRobot && (
        <div className="diag-overlay">
          <div className="diag-title">{selectedRobot.robot_id.toUpperCase()} · {selectedRobot.status}</div>
          <div className="diag-row"><span className="lbl">Position</span><span className="val accent">({selectedRobot.position[0].toFixed(1)}, {selectedRobot.position[1].toFixed(1)})</span></div>
          <div className="diag-row"><span className="lbl">Heading</span><span className="val">{(selectedRobot.heading || 0).toFixed(1)}°</span></div>
          <div className="diag-row"><span className="lbl">Battery</span><span className={`val ${selectedRobot.battery > 50 ? 'good' : selectedRobot.battery > 20 ? 'warn' : 'err'}`}>{selectedRobot.battery.toFixed(0)}%</span></div>
          <div className="diag-row"><span className="lbl">Payload</span><span className="val">{selectedRobot.has_cargo ? 'Carrying' : 'Empty'}</span></div>
          {selectedRobot.current_task_id && (
            <div className="diag-row"><span className="lbl">Task</span><span className="val accent" style={{ fontSize: '0.62rem' }}>{selectedRobot.current_task_id.substring(0, 8)}</span></div>
          )}
        </div>
      )}
    </div>
  );
}

/* --------------------------------------------------------------------------
   METRICS PANEL
   -------------------------------------------------------------------------- */
function MetricsPanel({ metrics = {} }) {
  const makespan = metrics.makespan ?? metrics.MAKESPAN ?? 0;
  const throughput = metrics.throughput ?? metrics.THROUGHPUT ?? 0;
  const collisions = metrics.collision_count ?? metrics.COLLISION_COUNT ?? 0;
  const deadlocks = metrics.deadlock_count ?? metrics.DEADLOCK_COUNT ?? 0;
  const replans = metrics.replan_count ?? metrics.REPLAN_COUNT ?? 0;
  const waitTime = metrics.waiting_time ?? metrics.WAITING_TIME ?? 0;

  return (
    <div className="panel-block">
      <div className="panel-header">
        <span className="panel-title">Performance</span>
        <span className="panel-tag">Live</span>
      </div>
      <div className="metrics-grid">
        <div className="metric-card">
          <div className="metric-label">Makespan</div>
          <div className="metric-val">{makespan.toFixed(0)}<span className="unit">t</span></div>
        </div>
        <div className="metric-card">
          <div className="metric-label">Throughput</div>
          <div className="metric-val">{throughput.toFixed(3)}<span className="unit">t/s</span></div>
        </div>
        <div className={`metric-card ${collisions > 0 ? 'err' : ''}`}>
          <div className="metric-label">Collisions</div>
          <div className="metric-val">{collisions}</div>
        </div>
        <div className={`metric-card ${deadlocks > 0 ? 'warn' : ''}`}>
          <div className="metric-label">Deadlocks</div>
          <div className="metric-val">{deadlocks}</div>
        </div>
        <div className="metric-card">
          <div className="metric-label">Re-plans</div>
          <div className="metric-val">{replans}</div>
        </div>
        <div className="metric-card">
          <div className="metric-label">Wait Time</div>
          <div className="metric-val">{waitTime.toFixed(0)}<span className="unit">t</span></div>
        </div>
      </div>
    </div>
  );
}

/* --------------------------------------------------------------------------
   TASKS PANEL
   -------------------------------------------------------------------------- */
function TasksPanel({ tasks = [] }) {
  const queued = tasks.filter(t => t.status === 'QUEUED' || t.status === 1).length;
  const assigned = tasks.filter(t => t.status === 'ASSIGNED' || t.status === 2).length;
  const inprog = tasks.filter(t => t.status === 'IN_PROGRESS' || t.status === 3).length;
  const comp = tasks.filter(t => t.status === 'COMPLETED' || t.status === 4).length;
  const total = queued + assigned + inprog + comp || 1;

  return (
    <div className="panel-block">
      <div className="panel-header">
        <span className="panel-title">Task Queue</span>
        <span className="panel-tag">{tasks.length} total</span>
      </div>
      <div className="task-counters">
        <div className="task-counter">
          <div className="tc-label">QUEUED</div>
          <div className="tc-val">{queued}</div>
        </div>
        <div className="task-counter">
          <div className="tc-label">ASSIGNED</div>
          <div className="tc-val">{assigned}</div>
        </div>
        <div className="task-counter">
          <div className="tc-label">ACTIVE</div>
          <div className="tc-val">{inprog}</div>
        </div>
        <div className="task-counter">
          <div className="tc-label">DONE</div>
          <div className="tc-val">{comp}</div>
        </div>
      </div>
      <div className="task-bar">
        <div className="task-bar-seg seg-queued" style={{ width: `${(queued / total) * 100}%` }} />
        <div className="task-bar-seg seg-assigned" style={{ width: `${(assigned / total) * 100}%` }} />
        <div className="task-bar-seg seg-inprog" style={{ width: `${(inprog / total) * 100}%` }} />
        <div className="task-bar-seg seg-done" style={{ width: `${(comp / total) * 100}%` }} />
      </div>
    </div>
  );
}

/* --------------------------------------------------------------------------
   ROBOTS PANEL
   -------------------------------------------------------------------------- */
function RobotsPanel({ robots = [], activeRobotId, setActiveRobotId }) {
  return (
    <div className="panel-block">
      <div className="panel-header">
        <span className="panel-title">AMR Fleet</span>
        <span className="panel-tag">{robots.filter(r => r.status !== 'OFFLINE').length} online</span>
      </div>
      <div className="robot-list">
        {robots.map(r => {
          const statusL = (r.status || 'idle').toLowerCase();
          const batColor = r.battery > 50 ? 'var(--green)' : r.battery > 20 ? 'var(--amber)' : 'var(--red)';
          return (
            <div
              key={r.robot_id}
              className={`robot-card ${activeRobotId === r.robot_id ? 'selected' : ''}`}
              onClick={() => setActiveRobotId(r.robot_id)}
            >
              <div className="robot-card-top">
                <span className="robot-id">{r.robot_id.replace('robot-', 'AMR-').toUpperCase()}</span>
                <div style={{ display: 'flex', gap: '0.3rem', alignItems: 'center' }}>
                  {r.has_cargo && <span className="cargo-tag">CARGO</span>}
                  <span className={`status-pill ${statusL}`}>{r.status}</span>
                </div>
              </div>
              <div className="robot-meta">
                <span>({r.position[0].toFixed(1)}, {r.position[1].toFixed(1)})</span>
                <span>{r.battery.toFixed(0)}%</span>
              </div>
              <div className="bat-bar">
                <div className="bat-fill" style={{ width: `${r.battery}%`, background: batColor }} />
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}

/* --------------------------------------------------------------------------
   EVENTS / CONFLICTS PANEL
   -------------------------------------------------------------------------- */
function ConflictsPanel({ conflicts = [] }) {
  return (
    <div className="panel-block">
      <div className="panel-header">
        <span className="panel-title">Event Log</span>
        <span className="panel-tag">{conflicts.length} events</span>
      </div>
      {conflicts.length === 0 ? (
        <div className="no-events">✓ No active conflicts</div>
      ) : (
        <div className="event-log">
          {conflicts.slice(-8).reverse().map((c, i) => (
            <div key={i} className="event-entry">
              <span className="ev-tick">T+{c.tick}</span>
              <span className="ev-type">{c.type?.replace('_CONFLICT', '') || '—'}</span>
              <span className="ev-detail">{c.robot_id} ↔ {c.peer_id}</span>
              <span className="ev-out">{c.outcome}</span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

/* --------------------------------------------------------------------------
   BENCHMARK PANEL
   -------------------------------------------------------------------------- */
function BenchmarkPanel({ scenario, setScenario }) {
  const [running, setRunning] = useState(false);
  const [results, setResults] = useState(null);

  const runBenchmark = async () => {
    setRunning(true);
    setResults(null);
    try {
      const res = await fetch('http://localhost:8000/api/benchmark', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ scenario })
      });
      const d = await res.json();
      setResults(d.results);
    } catch (e) { console.error('Benchmark error:', e); }
    finally { setRunning(false); }
  };

  return (
    <div className="panel-block">
      <div className="panel-header">
        <span className="panel-title">Benchmark</span>
        <span className="panel-tag">4 strategies · 2 trials</span>
      </div>

      <div className="bench-select-row">
        <select value={scenario} onChange={e => setScenario(e.target.value)} className="scenario-select">
          <option value="S1_Normal">S1 · Normal Warehouse</option>
          <option value="S2_Crossing">S2 · Crossing Traffic</option>
          <option value="S3_Narrow">S3 · Narrow Aisle</option>
          <option value="S4_Blocked">S4 · Blocked Aisle</option>
          <option value="S5_Failure">S5 · Robot Failure</option>
          <option value="S6_CommDelay">S6 · Comm Delay</option>
        </select>
        <button className="btn-run" onClick={runBenchmark} disabled={running}>
          {running ? 'Running...' : 'Run'}
        </button>
      </div>

      {results && (
        <>
          <table className="bench-table">
            <thead>
              <tr>
                <th>Strategy</th>
                <th>Wait</th>
                <th>Coll</th>
                <th>Done</th>
              </tr>
            </thead>
            <tbody>
              {['B0', 'B1', 'B2', 'P1'].map(s => (
                <tr key={s} className={s === 'P1' ? 'best' : ''}>
                  <td>{s}</td>
                  <td>{results[s]?.wait?.toFixed(1) ?? '—'}</td>
                  <td style={{ color: results[s]?.collisions === 0 ? 'var(--green)' : 'var(--red)' }}>
                    {results[s]?.collisions?.toFixed(1) ?? '—'}
                  </td>
                  <td>{results[s]?.completed?.toFixed(1) ?? '—'}</td>
                </tr>
              ))}
            </tbody>
          </table>
          <div className="bench-note">Live Map Synced · Independent benchmark trials</div>
        </>
      )}
    </div>
  );
}
