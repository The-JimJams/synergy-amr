/**
 * ============================================================================
 * SYNERGY-AMR v5 — Realistic Industrial Warehouse & Autonomous Fleet Digital Twin
 * ============================================================================
 * Features:
 *   • Decentralized P2P Coordination Layer (ORCA + SVO + DC-MRTA + Spatial Reservation)
 *   • Dynamic WMS Order Engine & Physical Cargo Pick-Transport-Delivery Lifecycle
 *   • Real-time Distributed Auction Matrix (DC-MRTA) & Explainable Decision Popovers
 *   • True 2D Raycasting LiDAR Sensor Simulation against Racks, Obstacles & Peers
 *   • High-DPI Retina Canvas Display & Interactive HUD Layer Toggles
 *   • Ad-hoc Manual AMR Waypoint Dispatching (Right-click Navigation)
 *   • Live ROS 2 Humble / Zenoh DDS Topic Stream Telemetry Monitor
 *   • 4-State Network Degradation & Wi-Fi Blackout Safety Bubble Mode
 * ============================================================================
 */
(function () {
  "use strict";

  /* ================================================================
     1. CONSTANTS & THEME
     ================================================================ */
  const W = 960, H = 580;
  const BOT_W = 32, BOT_H = 24;
  const BOT_R = 15;
  const BASE_SPD = 2.0;

  const C = {
    floor: "#08090d",
    floorTile: "rgba(255,255,255,.018)",
    jointLine: "rgba(255,255,255,.035)",
    rackBody: "#12131a",
    rackFrame: "#232533",
    rackLabel: "#606275",
    dockFill: "rgba(34,197,94,.06)",
    dockBorder: "rgba(34,197,94,.3)",
    bayFill: "rgba(234,179,8,.05)",
    bayBorder: "rgba(234,179,8,.25)",
    deadZone: "rgba(244,63,94,.06)",
    deadZoneStroke: "rgba(244,63,94,.35)",
    obstacle: "#ef4444",
    alpha: "#38bdf8",
    beta: "#eab308",
    gamma: "#22c55e",
    delta: "#a78bfa",
    epsilon: "#f472b6",
  };

  /* Pallet Cargo Color Palette */
  const CARGO_COLORS = [
    { fill: "#3b82f6", stroke: "#60a5fa", label: "SKU-A", name: "Electronics Pack" },
    { fill: "#f59e0b", stroke: "#fbbf24", label: "SKU-B", name: "Automotive Parts" },
    { fill: "#10b981", stroke: "#34d399", label: "SKU-C", name: "Medical Kits" },
    { fill: "#8b5cf6", stroke: "#a78bfa", label: "SKU-D", name: "Aerospace Composites" },
    { fill: "#ec4899", stroke: "#f472b6", label: "SKU-E", name: "Optical Sensors" },
    { fill: "#d97706", stroke: "#fcd34d", label: "WOOD", name: "Raw Material Pallet" },
  ];

  /* ================================================================
     2. WAREHOUSE MAP (Realistic Industrial Storage Layout)
     ================================================================ */
  const RACKS = [
    // Row 1 (Top)
    { x: 55, y: 48, w: 135, h: 56, label: "BAY A1-A4", slots: 4, seed: 1, bayId: "A1" },
    { x: 230, y: 48, w: 125, h: 56, label: "BAY A5-A8", slots: 4, seed: 2, bayId: "A5" },
    { x: 605, y: 48, w: 125, h: 56, label: "BAY B1-B4", slots: 4, seed: 3, bayId: "B1" },
    { x: 770, y: 48, w: 135, h: 56, label: "BAY B5-B8", slots: 4, seed: 4, bayId: "B5" },
    // Row 2
    { x: 55, y: 184, w: 135, h: 56, label: "BAY C1-C4", slots: 4, seed: 5, bayId: "C1" },
    { x: 230, y: 184, w: 125, h: 56, label: "BAY C5-C8", slots: 4, seed: 6, bayId: "C5" },
    { x: 605, y: 184, w: 125, h: 56, label: "BAY D1-D4", slots: 4, seed: 7, bayId: "D1" },
    { x: 770, y: 184, w: 135, h: 56, label: "BAY D5-D8", slots: 4, seed: 8, bayId: "D5" },
    // Row 3
    { x: 55, y: 340, w: 135, h: 56, label: "BAY E1-E4", slots: 4, seed: 9, bayId: "E1" },
    { x: 230, y: 340, w: 125, h: 56, label: "BAY E5-E8", slots: 4, seed: 10, bayId: "E5" },
    { x: 605, y: 340, w: 125, h: 56, label: "BAY F1-F4", slots: 4, seed: 11, bayId: "F1" },
    { x: 770, y: 340, w: 135, h: 56, label: "BAY F5-F8", slots: 4, seed: 12, bayId: "F5" },
    // Row 4 (Bottom)
    { x: 55, y: 466, w: 135, h: 56, label: "BAY G1-G4", slots: 4, seed: 13, bayId: "G1" },
    { x: 230, y: 466, w: 125, h: 56, label: "BAY G5-G8", slots: 4, seed: 14, bayId: "G5" },
    { x: 605, y: 466, w: 125, h: 56, label: "BAY H1-H4", slots: 4, seed: 15, bayId: "H1" },
    { x: 770, y: 466, w: 135, h: 56, label: "BAY H5-H8", slots: 4, seed: 16, bayId: "H5" },
  ];

  /* Industrial Stations (Conveyor docks & Workstations) */
  const STATIONS = [
    { x: 4, y: 260, w: 34, h: 60, label: "PICKING CONVEYOR", type: "conveyor", nodeId: "2_0", side: "left" },
    { x: 922, y: 260, w: 34, h: 60, label: "PACKING DOCK", type: "conveyor", nodeId: "2_6", side: "right" },
    { x: 430, y: 4, w: 100, h: 26, label: "INBOUND INDUCTION", type: "dock", nodeId: "0_3", side: "top" },
    { x: 430, y: 550, w: 100, h: 26, label: "OUTBOUND DISPATCH", type: "dock", nodeId: "4_3", side: "bottom" },
  ];

  /* Staging Bays with Hazard Striping */
  const ALCOVES = [
    { x: 410, y: 115, w: 60, h: 38, label: "STAGING 01", nodeId: "1_2" },
    { x: 490, y: 115, w: 60, h: 38, label: "STAGING 02", nodeId: "1_4" },
    { x: 410, y: 425, w: 60, h: 38, label: "STAGING 03", nodeId: "3_2" },
    { x: 490, y: 425, w: 60, h: 38, label: "STAGING 04", nodeId: "3_4" },
  ];

  /* Charging Pads */
  const CHARGING_PADS = [
    { x: 28, y: 28, label: "PWR 1" },
    { x: 932, y: 28, label: "PWR 2" },
    { x: 28, y: 550, label: "PWR 3" },
    { x: 932, y: 550, label: "PWR 4" },
  ];

  const DEAD_ZONE = { x: 590, y: 35, w: 330, h: 195 };

  /* ================================================================
     3. SLAM NAVIGATION GRAPH (Corridor Network)
     ================================================================ */
  const NAV_X = [28, 208, 380, 480, 580, 750, 932];
  const NAV_Y = [28, 145, 290, 432, 550];
  const NODES = [];
  const ADJ = {};
  const blockedNodes = new Set();

  function nid(r, c) { return `${r}_${c}`; }

  function buildGraph() {
    NODES.length = 0;
    for (let r = 0; r < NAV_Y.length; r++) {
      for (let c = 0; c < NAV_X.length; c++) {
        NODES.push({ id: nid(r, c), x: NAV_X[c], y: NAV_Y[r], ci: c, ri: r });
      }
    }
    for (const n of NODES) {
      ADJ[n.id] = [];
      if (n.ci < NAV_X.length - 1) ADJ[n.id].push(nid(n.ri, n.ci + 1));
      if (n.ci > 0) ADJ[n.id].push(nid(n.ri, n.ci - 1));
      if (n.ri < NAV_Y.length - 1) ADJ[n.id].push(nid(n.ri + 1, n.ci));
      if (n.ri > 0) ADJ[n.id].push(nid(n.ri - 1, n.ci));
    }
  }

  function getNode(id) { return NODES.find((n) => n.id === id); }

  function nearestNode(x, y, skipBlocked) {
    let best = null, bestD = Infinity;
    for (const n of NODES) {
      if (skipBlocked && blockedNodes.has(n.id)) continue;
      const d = Math.hypot(n.x - x, n.y - y);
      if (d < bestD) {
        bestD = d;
        best = n;
      }
    }
    return best;
  }

  /* A* Pathfinding */
  function astar(startId, endId) {
    if (startId === endId) return [getNode(startId)];
    const open = new Set([startId]);
    const gS = new Map([[startId, 0]]);
    const fS = new Map([[startId, heur(startId, endId)]]);
    const from = new Map();
    const closed = new Set();

    while (open.size) {
      let cur = null, cf = Infinity;
      for (const id of open) {
        const f = fS.get(id) ?? 1e9;
        if (f < cf) {
          cf = f;
          cur = id;
        }
      }
      if (cur === endId) {
        const path = [];
        let c = cur;
        while (c !== undefined) {
          path.unshift(getNode(c));
          c = from.get(c);
        }
        return path;
      }
      open.delete(cur);
      closed.add(cur);
      for (const nb of ADJ[cur] || []) {
        if (closed.has(nb) || blockedNodes.has(nb)) continue;
        const cn = getNode(cur), nn = getNode(nb);
        const g = (gS.get(cur) ?? 1e9) + Math.hypot(nn.x - cn.x, nn.y - cn.y);
        if (g < (gS.get(nb) ?? 1e9)) {
          from.set(nb, cur);
          gS.set(nb, g);
          fS.set(nb, g + heur(nb, endId));
          open.add(nb);
        }
      }
    }
    return [];
  }

  function heur(aId, bId) {
    const a = getNode(aId), b = getNode(bId);
    if (!a || !b) return 0;
    return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
  }

  /* ================================================================
     4. VECTOR & GEOMETRY MATH
     ================================================================ */
  const v2 = {
    dist: (a, b) => Math.hypot(a.x - b.x, a.y - b.y),
    len: (v) => Math.hypot(v.x, v.y),
    norm: (v) => {
      const l = Math.hypot(v.x, v.y);
      return l > 1e-4 ? { x: v.x / l, y: v.y / l } : { x: 0, y: 0 };
    },
    sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y }),
    lerp: (a, b, t) => ({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t }),
  };

  function normalizeAngle(a) {
    while (a <= -Math.PI) a += Math.PI * 2;
    while (a > Math.PI) a -= Math.PI * 2;
    return a;
  }

  /* Catmull-Rom spline interpolation for smooth curved paths */
  function catmullRomPoint(p0, p1, p2, p3, t) {
    const t2 = t * t, t3 = t2 * t;
    return {
      x: 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3),
      y: 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3),
    };
  }

  /**
   * Smooth an A* grid path into a natural curved path using Catmull-Rom splines.
   * Returns an array of {x, y} sub-waypoints with smooth arcs at corners.
   */
  function smoothPath(gridNodes) {
    if (!gridNodes || gridNodes.length < 2) return gridNodes || [];
    if (gridNodes.length === 2) {
      // Simple 2-node path: just add a few interpolated points
      const pts = [];
      for (let t = 0; t <= 1.0; t += 0.25) {
        pts.push(v2.lerp(gridNodes[0], gridNodes[1], t));
      }
      pts.push({ x: gridNodes[1].x, y: gridNodes[1].y });
      return pts;
    }

    const SUBDIVISIONS = 6; // sub-waypoints per segment
    const result = [];

    for (let i = 0; i < gridNodes.length - 1; i++) {
      const p0 = gridNodes[Math.max(0, i - 1)];
      const p1 = gridNodes[i];
      const p2 = gridNodes[i + 1];
      const p3 = gridNodes[Math.min(gridNodes.length - 1, i + 2)];

      for (let s = 0; s < SUBDIVISIONS; s++) {
        const t = s / SUBDIVISIONS;
        result.push(catmullRomPoint(p0, p1, p2, p3, t));
      }
    }
    // Push final destination point
    const last = gridNodes[gridNodes.length - 1];
    result.push({ x: last.x, y: last.y });
    return result;
  }

  /* Line segment intersection helper for 2D raycasting */
  function lineIntersectsRect(x1, y1, x2, y2, rx, ry, rw, rh) {
    const minX = rx, maxX = rx + rw, minY = ry, maxY = ry + rh;
    let tmin = 0, tmax = 1;
    const dx = x2 - x1, dy = y2 - y1;

    // X slab
    if (Math.abs(dx) < 1e-5) {
      if (x1 < minX || x1 > maxX) return null;
    } else {
      let t1 = (minX - x1) / dx, t2 = (maxX - x1) / dx;
      if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
      tmin = Math.max(tmin, t1);
      tmax = Math.min(tmax, t2);
      if (tmin > tmax) return null;
    }

    // Y slab
    if (Math.abs(dy) < 1e-5) {
      if (y1 < minY || y1 > maxY) return null;
    } else {
      let t1 = (minY - y1) / dy, t2 = (maxY - y1) / dy;
      if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
      tmin = Math.max(tmin, t1);
      tmax = Math.min(tmax, t2);
      if (tmin > tmax) return null;
    }

    if (tmin > 0 && tmin <= 1) {
      return { x: x1 + dx * tmin, y: y1 + dy * tmin, dist: tmin * Math.hypot(dx, dy) };
    }
    return null;
  }

  /* ================================================================
     5. NETWORK MODES
     ================================================================ */
  const NET_MODES = [
    { name: "Connected", commR: 210, lat: 11, loss: "0.0%", safety: 1.0, color: "#22c55e" },
    { name: "Degraded", commR: 110, lat: 240, loss: "18.4%", safety: 1.3, color: "#eab308" },
    { name: "Offline", commR: 0, lat: 999, loss: "100%", safety: 1.5, color: "#f43f5e" },
    { name: "Recovery", commR: 170, lat: 75, loss: "2.1%", safety: 1.1, color: "#38bdf8" },
  ];

  /* ================================================================
     6. HETEROGENEOUS FLEET PROFILES
     ================================================================ */
  const PROFILES = [
    {
      id: "alpha", name: "Robot Alpha", letter: "α", color: C.alpha,
      hw: "Jetson Orin Nano", role: "Heavy Lifter",
      maxPayload: 500, maxSpeed: 1.2, bat: 94,
      svo: 25, svoLabel: "Individualistic",
      taskPriority: 5, deadlineUrgency: 3, cargoType: "pallet",
    },
    {
      id: "beta", name: "Robot Beta", letter: "β", color: C.beta,
      hw: "Raspberry Pi 5", role: "Agile Runner",
      maxPayload: 100, maxSpeed: 2.1, bat: 82,
      svo: 88, svoLabel: "Altruistic",
      taskPriority: 2, deadlineUrgency: 2, cargoType: "tote",
    },
    {
      id: "gamma", name: "Robot Gamma", letter: "γ", color: C.gamma,
      hw: "Jetson Orin NX", role: "Standard Hauler",
      maxPayload: 250, maxSpeed: 1.5, bat: 96,
      svo: 50, svoLabel: "Prosocial",
      taskPriority: 3, deadlineUrgency: 4, cargoType: "crate",
    },
    {
      id: "delta", name: "Robot Delta", letter: "δ", color: C.delta,
      hw: "Jetson AGX Orin", role: "Precision Handler",
      maxPayload: 150, maxSpeed: 1.8, bat: 88,
      svo: 40, svoLabel: "Prosocial",
      taskPriority: 3, deadlineUrgency: 2, cargoType: "bin",
    },
  ];

  /* Critical Intersections for Spatial Reservation Tokens */
  const RESERVATION_ZONES = [
    { id: "JX_2_3", name: "Central 4-Way", nodeId: "2_3", x: 480, y: 290, r: 42, owner: null, ttl: 0 },
    { id: "JX_1_3", name: "North Crossway", nodeId: "1_3", x: 480, y: 145, r: 38, owner: null, ttl: 0 },
    { id: "JX_3_3", name: "South Crossway", nodeId: "3_3", x: 480, y: 432, r: 38, owner: null, ttl: 0 },
    { id: "JX_2_1", name: "West Aisle", nodeId: "2_1", x: 208, y: 290, r: 38, owner: null, ttl: 0 },
    { id: "JX_2_5", name: "East Aisle", nodeId: "2_5", x: 750, y: 290, r: 38, owner: null, ttl: 0 },
  ];

  /* ================================================================
     7. STATE
     ================================================================ */
  const S = {
    canvas: null,
    ctx: null,
    dpr: 1,
    lt: 0,
    simT: 0,
    running: true,
    speed: 1,
    scenario: 1,
    serverOn: true,
    netMode: 0,
    selBot: "alpha",
    obstacles: [],
    decision: null,
    auctionOverlay: null,
    deliveredCount: 0,
    conflictsAvoided: 0,
    activeOrderSeq: 1042,
    activeTab: "telemetry",
    floatingTexts: [],
    manualGoal: null,
    layers: {
      lidar: true,
      intent: true,
      mesh: true,
      reserve: true,
      graph: false,
    },
    bots: {},
  };

  const HOME_BASES = {
    alpha: { nodeId: "0_0", label: "PWR 1", x: 28, y: 28 },
    beta: { nodeId: "0_6", label: "PWR 2", x: 932, y: 28 },
    gamma: { nodeId: "4_0", label: "PWR 3", x: 28, y: 550 },
    delta: { nodeId: "4_6", label: "PWR 4", x: 932, y: 550 },
  };

  /* ================================================================
     8. AMR AGENT CLASS
     ================================================================ */
  class Bot {
    constructor(p) {
      Object.assign(this, p);
      this.x = 0;
      this.y = 0;
      this.theta = 0;
      this.vx = 0;
      this.vy = 0;
      this.spd = 0;
      this.omega = 0;
      this.lidarAngle = Math.random() * Math.PI * 2;
      this.lidarHits = [];
      this.alive = true;
      this.halted = false;
      this.parked = false;
      this.homeNode = HOME_BASES[p.id]?.nodeId || "0_0";
      this.yielding = false;
      this.yieldTo = null;
      this.waitTime = 0;
      this.inDZ = false;
      this.safeR = BOT_R;
      this.navPath = [];
      this.wpIdx = 0;
      this.wp = null;
      this.patrol = [];
      this.patrolIdx = 0;
      this.intent = [];
      this.log = [];
      this.cargo = null; // { sku, fill, stroke, weight }
      this.currentTask = null; // { orderId, state: 'to_pickup'|'to_drop', targetNode }
      this.curPayload = `${p.role} (${p.maxPayload}kg)`;
      this.addLog("INIT", `SLAM active on ${p.hw}`);
    }

    addLog(tag, msg) {
      const m = String(Math.floor(S.simT / 60)).padStart(2, "0");
      const s = String(Math.floor(S.simT % 60)).padStart(2, "0");
      this.log.unshift({ ts: `${m}:${s}`, tag, msg });
      if (this.log.length > 35) this.log.pop();
    }

    halt() {
      this.halted = true;
      this.parked = false;
      this.yielding = false;
      this.yieldTo = null;

      // If holding a task, re-auction to active peers
      if (this.currentTask) {
        this.addLog("WMS", `Task #${this.currentTask.orderId} re-auctioned`);
        let best = null, bestD = Infinity;
        for (const [id, o] of Object.entries(S.bots)) {
          if (id === this.id || !o.alive || o.halted || o.parked) continue;
          const d = v2.dist(this, o);
          if (d < bestD) {
            bestD = d;
            best = o;
          }
        }
        if (best) {
          best.addLog("MRTA", `Took over task from ${this.name}`);
        }
        this.cargo = null;
        this.currentTask = null;
      }

      this.curPayload = `Returning to Base (${HOME_BASES[this.id].label})`;
      const start = nearestNode(this.x, this.y, true);
      if (start) {
        const path = astar(start.id, this.homeNode);
        if (path.length > 0) {
          this.navPath = smoothPath(path);
          this.wpIdx = 0;
          this.wp = this.navPath[0];
        }
      }
      this.addLog("HALT", `Recalled to Base ${HOME_BASES[this.id].label}`);
      addFloatingText(this.x, this.y - 12, "⚡ Returning to Base", "#f59e0b");
      showToast(`${this.name} recalled — navigating to home charging base`);
    }

    resume() {
      this.halted = false;
      this.parked = false;
      const prof = PROFILES.find((p) => p.id === this.id);
      this.curPayload = `${prof.role} (${prof.maxPayload}kg)`;
      this.recalcPath();
      this.addLog("INIT", "Resumed warehouse operation");
      addFloatingText(this.x, this.y - 12, "✔ Resumed Patrol", "#38bdf8");
      showToast(`${this.name} resumed warehouse patrol`);
    }

    setPatrol(nodeIds) {
      this.patrol = nodeIds;
      this.patrolIdx = 0;
      this._goToPatrolTarget();
    }

    _goToPatrolTarget() {
      if (this.patrol.length === 0) return;
      const targetId = this.patrol[this.patrolIdx];
      const start = nearestNode(this.x, this.y, true);
      if (!start) return;
      const path = astar(start.id, targetId);
      if (path.length > 0) {
        this.navPath = smoothPath(path);
        this.wpIdx = 0;
        this.wp = this.navPath[0];
      } else {
        this.patrolIdx = (this.patrolIdx + 1) % this.patrol.length;
        const start2 = nearestNode(this.x, this.y, true);
        if (start2) {
          const path2 = astar(start2.id, this.patrol[this.patrolIdx]);
          if (path2.length > 0) {
            this.navPath = smoothPath(path2);
            this.wpIdx = 0;
            this.wp = this.navPath[0];
          }
        }
      }
    }

    recalcPath() {
      if (this.currentTask) {
        const start = nearestNode(this.x, this.y, true);
        if (start) {
          const path = astar(start.id, this.currentTask.targetNode);
          if (path.length > 0) {
            this.navPath = smoothPath(path);
            this.wpIdx = 0;
            this.wp = this.navPath[0];
          }
        }
        return;
      }
      if (this.patrol.length > 0) this._goToPatrolTarget();
    }

    advancePatrol() {
      if (this.halted) {
        this.parked = true;
        this.vx = this.vy = this.spd = this.omega = 0;
        this.addLog("PWR", `Docked & Charging @ ${HOME_BASES[this.id].label}`);
        addFloatingText(this.x, this.y - 12, "🅿️ Parked @ Base", "#22c55e");
        return;
      }
      if (this.currentTask) {
        this.handleTaskWaypoint();
        return;
      }
      this.patrolIdx = (this.patrolIdx + 1) % this.patrol.length;
      this._goToPatrolTarget();
    }

    handleTaskWaypoint() {
      const task = this.currentTask;
      if (!task) return;

      if (task.state === "to_pickup") {
        // Pick up cargo at bay
        this.cargo = { ...task.cargo };
        this.curPayload = `CARRYING ${task.cargo.label} (${task.cargo.weight}kg)`;
        this.addLog("WMS", `Loaded ${task.cargo.label} at Bay ${task.bay}`);
        addFloatingText(this.x, this.y - 12, `+Loaded ${task.cargo.label}`, "#38bdf8");

        task.state = "to_drop";
        task.targetNode = task.dropNode;
        const start = nearestNode(this.x, this.y, true);
        if (start) {
          const path = astar(start.id, task.dropNode);
          if (path.length > 0) {
            this.navPath = smoothPath(path);
            this.wpIdx = 0;
            this.wp = this.navPath[0];
            return;
          }
        }
      } else if (task.state === "to_drop") {
        // Delivered at dock
        const deliveredSku = this.cargo?.label || "CARGO";
        this.cargo = null;
        this.curPayload = `${this.role} (${this.maxPayload}kg)`;
        this.currentTask = null;
        S.deliveredCount++;
        this.addLog("WMS", `Delivered ${deliveredSku} to ${task.dropName}`);
        addFloatingText(this.x, this.y - 12, `✔ Delivered ${deliveredSku}!`, "#22c55e");
        showToast(`Order #${task.orderId} Delivered by ${this.name} to ${task.dropName}`);

        // Return to patrol
        this.advancePatrol();
      }
    }

    update(dt) {
      if (!this.alive) {
        this.vx = this.vy = this.spd = this.omega = 0;
        return;
      }
      if (this.parked) {
        this.vx = this.vy = this.spd = this.omega = 0;
        this.bat = Math.min(100, this.bat + 3.0 * dt); // Quick charging at dock
        return;
      }
      this.bat = Math.max(12, this.bat - 0.0025 * dt);
      this.lidarAngle = (this.lidarAngle + dt * 7.5) % (Math.PI * 2);

      // Dead zone check (scenario 4)
      if (S.scenario === 4) {
        const dz = DEAD_ZONE;
        const inside =
          this.x >= dz.x &&
          this.x <= dz.x + dz.w &&
          this.y >= dz.y &&
          this.y <= dz.y + dz.h;
        if (inside && !this.inDZ) {
          this.inDZ = true;
          this.addLog("RF", "Wi-Fi Blackout — LiDAR safety bubble σ=1.5");
        } else if (!inside && this.inDZ) {
          this.inDZ = false;
          this.addLog("ZENOH", "P2P mesh link restored");
        }
      } else {
        this.inDZ = false;
      }

      const nm = NET_MODES[S.netMode];
      this.safeR =
        BOT_R * nm.safety +
        (this.inDZ ? 8 + Math.sin(S.simT * 4) * 2.5 : 0);

      if (this.yielding) this.waitTime += dt;
      else this.waitTime = Math.max(0, this.waitTime - dt * 2);

      // Decay avoidance flash timer
      if (this._avoidFlash > 0) this._avoidFlash -= dt;

      this.checkSpatialReservation();
      this.navigate(dt);
      this.calcIntent();
      this.updateLidar();
    }

    checkSpatialReservation() {
      if (!S.layers.reserve) return;
      const zone = RESERVATION_ZONES[0]; // Central 4-Way JX_2_3 (480, 290)
      const d = v2.dist(this, zone);

      // 1. Release token the instant the owner crosses past the center (d > 24px and moving away)
      if (zone.owner === this.id) {
        const movingAway = this.vx * (this.x - zone.x) + this.vy * (this.y - zone.y) > 0;
        if (d > 24 && movingAway) {
          zone.owner = null;
          zone.ttl = 0;
          this.yielding = false;
          this.yieldTo = null;
          this._queuePos = 0;
        }
      }

      // 2. If token is free, award it to the highest-priority approaching bot within 110px
      if (zone.owner === null || zone.ttl <= 0) {
        const candidates = Object.values(S.bots).filter(b => {
          if (!b.alive) return false;
          const bd = v2.dist(b, zone);
          return bd < 110;
        });

        if (candidates.length > 0) {
          candidates.sort((a, b) => calcScore(b) - calcScore(a));
          const winner = candidates[0];
          zone.owner = winner.id;
          zone.ttl = 1.8;
          winner.yielding = false;
          winner.yieldTo = null;
          winner._queuePos = 0;

          // Ordered queue for waiting bots
          for (let i = 1; i < candidates.length; i++) {
            const loser = candidates[i];
            loser.yielding = true;
            loser.yieldTo = winner.id;
            loser._queuePos = i;
          }

          if (candidates.length >= 3) {
            S.conflictsAvoided += candidates.length - 1;
            triggerMultiDecision(candidates.map(b => ({
              bot: b,
              score: calcScore(b),
              t: b.taskPriority || 3,
              d: b.deadlineUrgency || 2,
              w: Math.min(3, Math.floor(b.waitTime)),
              b: b.bat > 60 ? 1 : b.bat > 30 ? 2 : 3,
              svo: b.svo || 50,
            })), winner);
          }
        }
      }
    }

    navigate(dt) {
      if (!this.wp || this.navPath.length === 0) {
        this.vx = this.vy = this.spd = this.omega = 0;
        return;
      }

      // Advance waypoint index smoothly as robot reaches each point
      while (this.wpIdx < this.navPath.length - 1 && v2.dist(this, this.navPath[this.wpIdx]) < 18) {
        this.wpIdx++;
      }
      this.wp = this.navPath[this.wpIdx];

      if (this.wpIdx >= this.navPath.length - 1 && v2.dist(this, this.wp) < 14) {
        this.advancePatrol();
        return;
      }

      const zone = RESERVATION_ZONES[0];
      const distToIntersection = v2.dist(this, zone);
      const isOwner = zone.owner === this.id && zone.ttl > 0;

      // Pure pursuit lookahead target (looks 24px ahead along path to avoid subpixel spinning)
      let targetPt = this.wp;
      for (let i = this.wpIdx; i < this.navPath.length; i++) {
        if (v2.dist(this, this.navPath[i]) >= 22) {
          targetPt = this.navPath[i];
          break;
        }
      }

      const dToTarget = v2.dist(this, targetPt);
      const dir = dToTarget > 2.0 ? v2.norm(v2.sub(targetPt, this)) : { x: Math.cos(this.theta), y: Math.sin(this.theta) };
      const desiredTheta = Math.atan2(dir.y, dir.x);

      // STOP-LINE HOLDING:
      // If this bot does not own the intersection token and is approaching within 38px–80px,
      // it stops cleanly at the entrance threshold so the intersection stays 100% clear for the owner!
      const movingTowardsCenter = Math.cos(desiredTheta) * (zone.x - this.x) + Math.sin(desiredTheta) * (zone.y - this.y) > 0;
      if (!isOwner && zone.owner !== null && distToIntersection < 80 && distToIntersection > 36 && movingTowardsCenter) {
        this.vx = 0;
        this.vy = 0;
        this.spd = 0;
        this.omega = 0;
        this.waitTime += dt;
        return;
      }

      // Proportional steering with deterministic 180-degree turn-around
      let angleDiff = normalizeAngle(desiredTheta - this.theta);
      if (Math.abs(Math.abs(angleDiff) - Math.PI) < 0.05) {
        angleDiff = Math.PI - 0.05; // Force deterministic clockwise turn
      }

      if (dToTarget > 3.0) {
        const steerRate = 6.5;
        if (Math.abs(angleDiff) > 0.04) {
          this.theta += Math.sign(angleDiff) * Math.min(Math.abs(angleDiff), steerRate * dt);
          this.theta = normalizeAngle(this.theta);
          this.omega = Math.sign(angleDiff) * steerRate;
        } else {
          this.theta = desiredTheta;
          this.omega = 0;
        }
      } else {
        this.omega = 0;
      }

      // Turn penalty: scale forward velocity down during sharp turns so robot turns in place cleanly without looping
      const turnPenalty = Math.max(0.08, Math.cos(angleDiff));

      // Fast crossing speed: 1.4x boost when owning intersection token to clear in ~0.7 seconds
      const speedMult = isOwner ? 1.4 : 1.0;
      const dzMult = this.inDZ ? 0.6 : 1.0;
      const targetSpd = this.maxSpeed * speedMult * dzMult * BASE_SPD * turnPenalty;

      let vx = Math.cos(this.theta) * targetSpd;
      let vy = Math.sin(this.theta) * targetSpd;

      // Soft collision deceleration (Only in open aisles, token owner never slows down)
      if (!isOwner) {
        const nm = NET_MODES[S.netMode];
        const minGap = (BOT_R * 2 + 6) * nm.safety;
        for (const [id, o] of Object.entries(S.bots)) {
          if (id === this.id || !o.alive) continue;
          if (o.yieldTo === this.id) continue;

          const dd = v2.dist(this, o);
          if (dd < minGap && dd > 0.1) {
            const rp = v2.sub(o, this);
            const dot = rp.x * vx + rp.y * vy;
            if (dot > 0) {
              vx *= 0.35;
              vy *= 0.35;
            }
          }
        }
      }

      this.vx = vx;
      this.vy = vy;
      this.spd = v2.len({ x: vx, y: vy });

      this.x += vx * dt * 60;
      this.y += vy * dt * 60;
      this.x = Math.max(BOT_R, Math.min(W - BOT_R, this.x));
      this.y = Math.max(BOT_R, Math.min(H - BOT_R, this.y));
    }

    calcIntent() {
      this.intent = [];
      for (
        let i = this.wpIdx;
        i < Math.min(this.wpIdx + 10, this.navPath.length);
        i++
      ) {
        this.intent.push({ x: this.navPath[i].x, y: this.navPath[i].y });
      }
    }

    updateLidar() {
      this.lidarHits = [];
      if (!S.layers.lidar) return;

      const maxRayDist = 110;
      const numRays = 8;
      for (let i = 0; i < numRays; i++) {
        const ang = this.lidarAngle + (i * Math.PI * 2) / numRays;
        const rayEnd = {
          x: this.x + Math.cos(ang) * maxRayDist,
          y: this.y + Math.sin(ang) * maxRayDist,
        };

        let closestHit = null, minD = maxRayDist;

        // Raycast against storage racks
        for (const r of RACKS) {
          const hit = lineIntersectsRect(this.x, this.y, rayEnd.x, rayEnd.y, r.x, r.y, r.w, r.h);
          if (hit && hit.dist < minD) {
            minD = hit.dist;
            closestHit = hit;
          }
        }

        // Raycast against dynamic obstacles
        for (const obs of S.obstacles) {
          const hit = lineIntersectsRect(this.x, this.y, rayEnd.x, rayEnd.y, obs.x - obs.r, obs.y - obs.r, obs.r * 2, obs.r * 2);
          if (hit && hit.dist < minD) {
            minD = hit.dist;
            closestHit = hit;
          }
        }

        // Raycast against other AMRs
        for (const [id, o] of Object.entries(S.bots)) {
          if (id === this.id || !o.alive) continue;
          const hit = lineIntersectsRect(this.x, this.y, rayEnd.x, rayEnd.y, o.x - BOT_R, o.y - BOT_R, BOT_R * 2, BOT_R * 2);
          if (hit && hit.dist < minD) {
            minD = hit.dist;
            closestHit = hit;
          }
        }

        if (closestHit) {
          this.lidarHits.push({ x: closestHit.x, y: closestHit.y, ang, hit: true });
        } else {
          this.lidarHits.push({ x: rayEnd.x, y: rayEnd.y, ang, hit: false });
        }
      }
    }
  }

  /* ================================================================
     9. PRIORITY SCORING & CONFLICT RESOLUTION
     ================================================================ */
  function calcScore(b) {
    if (!b.alive || b.parked) return 0;
    const batUrg = b.bat > 60 ? 1 : b.bat > 30 ? 2 : 3;
    const wait = Math.min(3, Math.floor(b.waitTime));
    const haltMod = b.halted ? -5 : 0;
    return Math.max(1, (b.taskPriority || 3) + (b.deadlineUrgency || 2) + wait + batUrg + haltMod);
  }

  function checkConflicts() {
    const bots = Object.values(S.bots).filter((b) => b.alive);
    const zone = RESERVATION_ZONES[0];

    // Open corridor pairwise conflict resolution (excludes intersection zone)
    for (let i = 0; i < bots.length; i++) {
      for (let j = i + 1; j < bots.length; j++) {
        const a = bots[i], b = bots[j];
        if (v2.dist(a, zone) < 85 || v2.dist(b, zone) < 85) continue;
        if (a.yielding && a.yieldTo !== b.id && a.yieldTo) continue;
        if (b.yielding && b.yieldTo !== a.id && b.yieldTo) continue;

        const d = v2.dist(a, b);
        if (d < 65 && d > 5) {
          const closing =
            a.vx * (b.x - a.x) + a.vy * (b.y - a.y) > 0 ||
            b.vx * (a.x - b.x) + b.vy * (a.y - b.y) > 0;
          if (closing) {
            const as = calcScore(a);
            const bs = calcScore(b);
            const winner = as >= bs ? a : b;
            const loser = as >= bs ? b : a;
            const ws = Math.max(as, bs);
            const ls = Math.min(as, bs);

            if (!loser.yielding || loser.yieldTo !== winner.id) {
              loser.yielding = true;
              loser.yieldTo = winner.id;
              loser._avoidFlash = 0.5;
              winner._avoidFlash = 0.5;
              S.conflictsAvoided++;
              loser.addLog("SVO", `Yield to ${winner.name} (${ls} vs ${ws})`);
              winner.addLog("ORCA", `Priority pass vs ${loser.name}`);
              triggerDecision(winner, loser, ws, ls);
            }
          }
        }
      }
    }
  }

  function triggerMultiDecision(scoredList, winner) {
    if (S.decision && S.decision.timer > 1.8 && S.decision.isMulti) return;
    const avgX = scoredList.reduce((acc, it) => acc + it.bot.x, 0) / scoredList.length;
    const avgY = scoredList.reduce((acc, it) => acc + it.bot.y, 0) / scoredList.length;

    S.decision = {
      isMulti: true,
      x: avgX,
      y: avgY,
      winner,
      ranked: scoredList,
      timer: 5.0,
    };
  }

  function triggerDecision(winner, loser, ws, ls) {
    if (S.decision && S.decision.timer > 1.8) return;
    const bat = (b) => (b.bat > 60 ? 1 : b.bat > 30 ? 2 : 3);
    const wait = (b) => Math.min(3, Math.floor(b.waitTime));

    S.decision = {
      isMulti: false,
      x: (winner.x + loser.x) / 2,
      y: (winner.y + loser.y) / 2,
      winner, loser, ws, ls,
      wD: { t: winner.taskPriority, d: winner.deadlineUrgency, w: wait(winner), b: bat(winner) },
      lD: { t: loser.taskPriority, d: loser.deadlineUrgency, w: wait(loser), b: bat(loser) },
      timer: 4.2,
    };
  }

  /* ================================================================
     10. DISTRIBUTED DC-MRTA AUCTION ENGINE
     ================================================================ */
  function dispatchNewWmsOrder() {
    const orderId = ++S.activeOrderSeq;
    const rack = RACKS[Math.floor(Math.random() * RACKS.length)];
    const cargo = CARGO_COLORS[Math.floor(Math.random() * CARGO_COLORS.length)];
    const station = STATIONS[Math.floor(Math.random() * STATIONS.length)];
    const weight = Math.floor(40 + Math.random() * 200);

    const bayNode = nearestNode(rack.x + rack.w / 2, rack.y + rack.h / 2, true);
    if (!bayNode) return;

    // Calculate distributed bids across all alive AMRs
    const bids = [];
    for (const bot of Object.values(S.bots)) {
      if (!bot.alive) continue;
      const d = v2.dist(bot, bayNode);
      const capPenalty = weight > bot.maxPayload ? 9000 : 0;
      const busyPenalty = bot.currentTask ? 600 : 0;
      const batCost = (100 - bot.bat) * 1.8;
      const cost = Math.round(d * 0.6 + batCost + capPenalty + busyPenalty);
      bids.push({ bot, cost, d: Math.round(d), bat: Math.round(bot.bat) });
    }

    if (bids.length === 0) return;
    bids.sort((a, b) => a.cost - b.cost);
    const winner = bids[0].bot;

    winner.currentTask = {
      orderId,
      cargo: { ...cargo, weight },
      bay: rack.label,
      targetNode: bayNode.id,
      dropNode: station.nodeId,
      dropName: station.label,
      state: "to_pickup",
    };

    const start = nearestNode(winner.x, winner.y, true);
    if (start) {
      const path = astar(start.id, bayNode.id);
      if (path.length > 0) {
        winner.navPath = smoothPath(path);
        winner.wpIdx = 0;
        winner.wp = winner.navPath[0];
      }
    }

    winner.addLog("MRTA", `Won DC-MRTA Order #${orderId} (${cargo.label}, ${weight}kg)`);
    showToast(`Order #${orderId} Auctioned: ${winner.name} won (${bids[0].cost} cost)`);

    const pill = document.getElementById("wmsText");
    if (pill) pill.textContent = `WMS: #${orderId} → ${winner.name}`;

    // Pop-in Auction Matrix
    triggerAuctionOverlay(orderId, cargo, weight, rack.label, station.label, bids);
  }

  function triggerAuctionOverlay(orderId, cargo, weight, bay, dock, bids) {
    S.auctionOverlay = {
      orderId, cargo, weight, bay, dock, bids,
      timer: 5.0,
    };
  }

  /* Floating text notification */
  function addFloatingText(x, y, text, color) {
    S.floatingTexts.push({ x, y, text, color, life: 2.2 });
  }

  /* ================================================================
     11. SCENARIOS & PATROLS (4 AMRs)
     ================================================================ */
  const PATROLS = {
    // Scenario 1: 4-Way Crossing & Warehouse Logistics — 4 AMRs circumnavigating warehouse quadrants and crossing at Central 4-Way
    1: {
      alpha: ["2_0", "1_1", "0_3", "2_3", "3_5", "4_6", "2_6", "2_3"], // Northwest -> Central 4-Way -> Southeast -> Central 4-Way
      beta: ["0_3", "1_5", "2_6", "2_3", "3_1", "4_0", "4_3", "2_3"],  // Northeast -> Central 4-Way -> Southwest -> Central 4-Way
      gamma: ["2_6", "3_5", "4_3", "2_3", "1_1", "0_0", "2_0", "2_3"], // Southeast -> Central 4-Way -> Northwest -> Central 4-Way
      delta: ["4_3", "3_1", "2_0", "2_3", "1_5", "0_6", "0_3", "2_3"], // Southwest -> Central 4-Way -> Northeast -> Central 4-Way
    },
    // Scenario 2: SVO Yield — Corridor traverses with full loop returns
    2: {
      alpha: ["2_0", "2_1", "2_3", "2_5", "2_6", "1_5", "1_1"],
      beta: ["0_3", "1_3", "2_3", "3_3", "4_3", "4_1", "0_1"],
      gamma: ["2_6", "2_5", "2_3", "2_1", "2_0", "3_1", "3_5"],
      delta: ["4_3", "3_3", "2_3", "1_3", "0_3", "0_5", "4_5"],
    },
    // Scenario 3: Obstacles — Fleet auto-reroutes around blocked aisles via A*
    3: {
      alpha: ["1_0", "1_3", "1_6", "3_6", "3_3", "3_0"],
      beta: ["2_6", "2_3", "2_0", "0_0", "0_3", "0_6"],
      gamma: ["0_3", "2_3", "4_3", "4_5", "2_5", "0_5"],
      delta: ["3_1", "2_1", "1_1", "1_3", "1_5", "3_5"],
    },
    // Scenario 4: Dead Zone — Navigating through RF blackout with LiDAR bubble
    4: {
      alpha: ["1_4", "0_6", "1_6", "2_6", "2_3"],
      beta: ["2_5", "2_3", "2_1", "4_1", "4_3", "4_5"],
      gamma: ["3_0", "2_0", "2_3", "1_3", "1_0"],
      delta: ["0_5", "2_5", "2_3", "3_3", "4_3", "4_1", "0_1"],
    },
    // Scenario 5: Node Failure — Beta fails, remaining 3 bots take over
    5: {
      alpha: ["2_0", "1_1", "0_3", "2_3", "3_5", "2_6", "2_3"],
      beta: ["2_3"],
      gamma: ["3_6", "3_3", "2_3", "1_3", "1_5", "2_5", "2_3"],
      delta: ["4_3", "3_1", "2_0", "2_3", "0_3", "1_5", "2_3"],
    },
  };

  const START_NODES = {
    1: ["2_0", "0_3", "2_6", "4_3"],
    2: ["2_0", "0_3", "2_6", "4_3"],
    3: ["1_0", "2_6", "0_3", "3_1"],
    4: ["1_4", "2_5", "3_0", "0_5"],
    5: ["2_0", "2_3", "3_6", "4_3"],
  };

  function initBots() {
    S.bots = {};
    PROFILES.forEach((p) => {
      S.bots[p.id] = new Bot({ ...p });
    });
  }

  function loadScenario(n) {
    S.scenario = n;
    S.obstacles = [];
    blockedNodes.clear();
    S.decision = null;
    S.manualGoal = null;

    document
      .querySelectorAll(".sc-btn")
      .forEach((b) => b.classList.toggle("active", +b.dataset.scenario === n));

    const ids = Object.keys(S.bots);
    const sn = START_NODES[n] || START_NODES[1];
    const patrol = PATROLS[n] || PATROLS[1];

    ids.forEach((id, i) => {
      const b = S.bots[id];
      const p = PROFILES[i];
      b.alive = true;
      b.yielding = false;
      b.yieldTo = null;
      b.waitTime = 0;
      b.inDZ = false;
      b.bat = p.bat;
      b.svo = p.svo;
      b.cargo = null;
      b.currentTask = null;
      b.taskPriority = p.taskPriority;
      b.deadlineUrgency = p.deadlineUrgency;
      b.curPayload = `${p.role} (${p.maxPayload}kg)`;

      const startNode = getNode(sn[i]);
      if (startNode) {
        b.x = startNode.x;
        b.y = startNode.y;
      }
      b.theta = 0;
      b.setPatrol(patrol[id] || ["2_3"]);
    });

    if (n === 5) {
      S.bots.beta.alive = false;
      S.bots.beta.addLog("FAULT", "Heartbeat timeout > 100ms");
      setTimeout(() => {
        if (S.bots.alpha?.alive) {
          S.bots.alpha.addLog("MRTA", `Won DC-MRTA auction for Beta payload`);
          S.bots.alpha.curPayload += ` + [Beta's Task]`;
        }
      }, 800);
      setTimeout(() => {
        if (S.bots.gamma?.alive)
          S.bots.gamma.addLog("MRTA", `Standby — Alpha priority bid for Beta`);
      }, 1600);
    }

    if (n === 3) {
      spawnObstacle("2_3");
      spawnObstacle("1_3");
    }

    const msgs = {
      1: "ORCA: 5 AMRs crossing 4-way warehouse floor via SLAM + Spatial Reservation",
      2: "SVO: Beta yields into staging bay (θ=88° Altruistic) via Priority Score",
      3: "Click open floor to drop cargo crate obstacles — A* auto-reroutes fleet",
      4: "NE Wi-Fi blackout — AMRs switch to onboard LiDAR bubble (σ=1.5)",
      5: "Beta node failure — Alpha wins decentralized DC-MRTA bidding auction",
    };
    showToast(msgs[n] || "");
    updateInsp();
  }

  function spawnObstacle(nodeIdStr) {
    const node = getNode(nodeIdStr);
    if (!node || blockedNodes.has(nodeIdStr)) return;
    blockedNodes.add(nodeIdStr);
    S.obstacles.push({ x: node.x, y: node.y, r: 20, nodeId: nodeIdStr });
    for (const b of Object.values(S.bots)) {
      if (b.alive) b.recalcPath();
    }
  }

  function removeObstacle(idx) {
    const obs = S.obstacles[idx];
    if (obs?.nodeId) blockedNodes.delete(obs.nodeId);
    S.obstacles.splice(idx, 1);
    for (const b of Object.values(S.bots)) {
      if (b.alive) b.recalcPath();
    }
  }

  function showToast(msg) {
    const t = document.getElementById("toast");
    const m = document.getElementById("toastMsg");
    if (t && m) {
      m.textContent = msg;
      t.classList.remove("hidden");
      clearTimeout(t._tid);
      t._tid = setTimeout(() => t.classList.add("hidden"), 4500);
    }
  }

  /* ================================================================
     12. HYPER-REALISTIC INDUSTRIAL CAD CANVAS RENDER
     ================================================================ */
  function render(ctx) {
    ctx.save();

    // 1. Industrial Concrete & Epoxy Coated Floor
    drawWarehouseFloor(ctx);

    // 2. Wi-Fi Access Points (Overhead Wireless Mesh)
    drawOverheadWifiAPs(ctx);

    // 3. SLAM Graph Overlay (if toggled)
    if (S.layers.graph) {
      ctx.strokeStyle = "rgba(56,189,248,.14)";
      ctx.lineWidth = 1;
      for (const n of NODES) {
        for (const nbId of ADJ[n.id] || []) {
          const nb = getNode(nbId);
          if (nb) {
            ctx.beginPath();
            ctx.moveTo(n.x, n.y);
            ctx.lineTo(nb.x, nb.y);
            ctx.stroke();
          }
        }
      }
      for (const n of NODES) {
        ctx.fillStyle = blockedNodes.has(n.id) ? "rgba(244,63,94,.7)" : "rgba(56,189,248,.45)";
        ctx.beginPath();
        ctx.arc(n.x, n.y, 3, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    // 4. Spatial Reservation Zones
    if (S.layers.reserve) {
      for (const z of RESERVATION_ZONES) {
        const ownerBot = z.owner ? S.bots[z.owner] : null;
        ctx.save();
        if (ownerBot && z.ttl > 0) {
          ctx.fillStyle = `${ownerBot.color}18`;
          ctx.strokeStyle = `${ownerBot.color}75`;
          ctx.lineWidth = 1.5;
          ctx.setLineDash([4, 3]);
          ctx.beginPath();
          ctx.arc(z.x, z.y, z.r, 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();

          ctx.fillStyle = ownerBot.color;
          ctx.font = "bold 8px JetBrains Mono";
          ctx.textAlign = "center";
          ctx.fillText(`TOKEN: ${ownerBot.letter}`, z.x, z.y + 3);
        } else {
          ctx.strokeStyle = "rgba(255,255,255,.06)";
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.arc(z.x, z.y, z.r, 0, Math.PI * 2);
          ctx.stroke();
        }
        ctx.restore();
      }
    }

    // 5. Automated Fast-Charging Docks
    for (const pad of CHARGING_PADS) {
      drawChargingInfrastructure(ctx, pad);
    }

    // 6. Staging Bays with Hazard Stripes & Pallet Guides
    for (const al of ALCOVES) {
      drawAlcoveBays(ctx, al);
    }

    // 7. Heavy-Duty High-Bay Steel Pallet Racks
    for (const r of RACKS) {
      drawHighBayRack(ctx, r);
    }

    // 8. Stations (Motorized Roller Conveyors & Induction Docks)
    for (const st of STATIONS) {
      drawStationMachinery(ctx, st);
    }

    // 9. Wi-Fi Dead Zone (Scenario 4)
    if (S.scenario === 4) {
      const dz = DEAD_ZONE;
      ctx.fillStyle = C.deadZone;
      ctx.fillRect(dz.x, dz.y, dz.w, dz.h);
      ctx.strokeStyle = C.deadZoneStroke;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 4]);
      ctx.strokeRect(dz.x, dz.y, dz.w, dz.h);
      ctx.setLineDash([]);
      ctx.fillStyle = "rgba(244,63,94,.65)";
      ctx.font = "bold 10px Inter";
      ctx.textAlign = "center";
      ctx.fillText("⚠ RF BLACKOUT ZONE (OFFLINE MESH)", dz.x + dz.w / 2, dz.y + 16);
    }

    // 10. Dynamic Cargo Obstacles (Industrial Heavy Crates)
    for (const obs of S.obstacles) {
      drawIndustrialCrate(ctx, obs);
    }

    // 11. Manual Dispatch Pin (if set)
    if (S.manualGoal) {
      ctx.save();
      ctx.strokeStyle = "#38bdf8";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(S.manualGoal.x, S.manualGoal.y, 9 + Math.sin(S.simT * 6) * 3, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillStyle = "#38bdf8";
      ctx.font = "12px Inter";
      ctx.textAlign = "center";
      ctx.fillText("🎯", S.manualGoal.x, S.manualGoal.y + 4);
      ctx.restore();
    }

    // 12. P2P Mesh Wireless Links (with flowing data packets)
    const nm = NET_MODES[S.netMode];
    const bl = Object.values(S.bots);
    if (S.layers.mesh && nm.commR > 0) {
      for (let i = 0; i < bl.length; i++) {
        for (let j = i + 1; j < bl.length; j++) {
          const a = bl[i], b = bl[j];
          if (!a.alive || !b.alive) continue;
          const d = v2.dist(a, b);
          if (d <= nm.commR) {
            ctx.save();
            const dead = a.inDZ || b.inDZ;
            const strength = 1 - d / nm.commR;
            const col = dead ? "rgba(244,63,94," : "rgba(56,189,248,";

            // Link line
            ctx.strokeStyle = col + `${0.12 + 0.3 * strength})`;
            ctx.lineWidth = 1 + strength;
            ctx.setLineDash([3, 5]);
            ctx.lineDashOffset = -S.simT * 28;
            ctx.beginPath();
            ctx.moveTo(a.x, a.y);
            ctx.lineTo(b.x, b.y);
            ctx.stroke();

            // Flowing data packets (small dots moving along the link)
            if (!dead && strength > 0.3) {
              const numPkts = Math.ceil(strength * 3);
              for (let p = 0; p < numPkts; p++) {
                const t = ((S.simT * 1.2 + p * 0.33 + i * 0.17) % 1);
                const px = a.x + (b.x - a.x) * t;
                const py = a.y + (b.y - a.y) * t;
                ctx.fillStyle = col + `${0.5 + strength * 0.4})`;
                ctx.beginPath();
                ctx.arc(px, py, 1.5, 0, Math.PI * 2);
                ctx.fill();
              }
            }

            // Link quality label at midpoint (for strong connections)
            if (strength > 0.5 && d > 40) {
              const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
              ctx.fillStyle = col + "0.4)";
              ctx.font = "500 6px Inter";
              ctx.textAlign = "center";
              ctx.fillText(`${Math.round(strength * 100)}%`, mx, my - 4);
            }

            ctx.restore();
          }
        }
      }
    }

    // 13. Trajectory Intent Paths (smooth curves)
    if (S.layers.intent) {
      for (const b of bl) {
        if (!b.alive || b.intent.length < 2) continue;
        ctx.save();
        ctx.strokeStyle = b.color;
        ctx.lineWidth = 1.5;
        ctx.globalAlpha = 0.3;
        ctx.setLineDash([2, 3]);
        ctx.lineDashOffset = -S.simT * 30;
        ctx.beginPath();
        ctx.moveTo(b.x, b.y);
        // Use quadratic curves for smooth intent trail
        for (let i = 0; i < b.intent.length; i++) {
          if (i < b.intent.length - 1) {
            const cp = b.intent[i];
            const ep = b.intent[i + 1];
            const mx = (cp.x + ep.x) / 2, my = (cp.y + ep.y) / 2;
            ctx.quadraticCurveTo(cp.x, cp.y, mx, my);
          } else {
            ctx.lineTo(b.intent[i].x, b.intent[i].y);
          }
        }
        ctx.stroke();
        // Small endpoint dot
        if (b.intent.length > 0) {
          const last = b.intent[b.intent.length - 1];
          ctx.globalAlpha = 0.5;
          ctx.fillStyle = b.color;
          ctx.beginPath();
          ctx.arc(last.x, last.y, 2.5, 0, Math.PI * 2);
          ctx.fill();
        }
        ctx.restore();
      }
    }

    // 14. Autonomous Mobile Robots (AMRs)
    for (const b of bl) {
      drawAMR(ctx, b);
    }

    // 15. Floating Notifications
    for (const ft of S.floatingTexts) {
      ctx.save();
      ctx.globalAlpha = Math.min(1, ft.life / 0.5);
      ctx.fillStyle = ft.color;
      ctx.font = "bold 10px JetBrains Mono";
      ctx.textAlign = "center";
      ctx.fillText(ft.text, ft.x, ft.y);
      ctx.restore();
    }

    // 16. Explainable Decision Popover
    renderDecision(ctx);

    // 17. DC-MRTA Distributed Auction Matrix Overlay
    renderAuctionMatrix(ctx);

    ctx.restore();
  }

  /* ── 1. PRACTICAL INDUSTRIAL WAREHOUSE FLOOR ──────────────── */
  function drawWarehouseFloor(ctx) {
    // Polished dark epoxy-coated concrete floor
    ctx.fillStyle = "#08090d";
    ctx.fillRect(0, 0, W, H);

    // Subtle overhead high-bay lighting (3 bays)
    const bayPositions = [240, W / 2, 720];
    for (const bx of bayPositions) {
      const lg = ctx.createRadialGradient(bx, H / 2, 20, bx, H / 2, 320);
      lg.addColorStop(0, "rgba(255,255,255,.018)");
      lg.addColorStop(0.5, "rgba(255,255,255,.006)");
      lg.addColorStop(1, "transparent");
      ctx.fillStyle = lg;
      ctx.fillRect(0, 0, W, H);
    }

    // Very subtle concrete slab joints
    ctx.strokeStyle = "rgba(255,255,255,.012)";
    ctx.lineWidth = 0.5;
    for (let x = 0; x < W; x += 192) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
    }
    for (let y = 0; y < H; y += 192) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    }

    // Subtle solid aisle guide lines (real warehouses have these — thin solid yellow)
    ctx.save();
    ctx.strokeStyle = "rgba(234,179,8,.08)";
    ctx.lineWidth = 1;
    // Main horizontal aisles only
    [145, 290, 432].forEach(y => {
      ctx.beginPath(); ctx.moveTo(40, y); ctx.lineTo(920, y); ctx.stroke();
    });
    // Main vertical aisles only
    [208, 480, 750].forEach(x => {
      ctx.beginPath(); ctx.moveTo(x, 40); ctx.lineTo(x, 540); ctx.stroke();
    });
    ctx.restore();
  }

  /* ── 2. PALLET RACKS (Clean Industrial Look) ──────────────── */
  function drawHighBayRack(ctx, r) {
    // Soft shadow
    ctx.fillStyle = "rgba(0,0,0,.4)";
    roundRect(ctx, r.x + 2, r.y + 3, r.w, r.h, 3);
    ctx.fill();

    // Rack body
    ctx.fillStyle = "#0f1018";
    roundRect(ctx, r.x, r.y, r.w, r.h, 3);
    ctx.fill();
    ctx.strokeStyle = "#1c1e2e";
    ctx.lineWidth = 1;
    roundRect(ctx, r.x, r.y, r.w, r.h, 3);
    ctx.stroke();

    const slotW = r.w / r.slots;

    // Slots with cargo
    for (let s = 0; s < r.slots; s++) {
      const sx = r.x + s * slotW;

      // Thin vertical divider
      if (s > 0) {
        ctx.fillStyle = "rgba(255,255,255,.04)";
        ctx.fillRect(sx, r.y + 3, 1, r.h - 6);
      }

      // Cargo box
      const cargoIdx = (r.seed * 3 + s * 2) % CARGO_COLORS.length;
      const cargo = CARGO_COLORS[cargoIdx];
      const padX = 4, padY = 5;
      const pw = slotW - padX * 2;
      const ph = r.h - padY * 2;

      // Cargo fill
      ctx.fillStyle = cargo.fill;
      ctx.globalAlpha = 0.7;
      roundRect(ctx, sx + padX, r.y + padY, pw, ph, 2);
      ctx.fill();

      // Top bevel highlight
      ctx.fillStyle = "rgba(255,255,255,.15)";
      ctx.fillRect(sx + padX, r.y + padY, pw, 1.5);

      // Outline
      ctx.globalAlpha = 0.5;
      ctx.strokeStyle = cargo.stroke;
      ctx.lineWidth = 0.8;
      roundRect(ctx, sx + padX, r.y + padY, pw, ph, 2);
      ctx.stroke();
      ctx.globalAlpha = 1.0;
    }

    // Top & bottom accent beams
    ctx.fillStyle = "#334155";
    ctx.fillRect(r.x, r.y, r.w, 2);
    ctx.fillRect(r.x, r.y + r.h - 2, r.w, 2);
    ctx.fillStyle = "rgba(255,255,255,.06)";
    ctx.fillRect(r.x, r.y, r.w, 1);

    // Bay label
    ctx.fillStyle = "rgba(255,255,255,.45)";
    ctx.font = "600 7px Inter";
    ctx.textAlign = "center";
    ctx.fillText(r.label, r.x + r.w / 2, r.y - 4);
  }

  /* ── 3. STATIONS & DOCKS ──────────────────────────────────── */
  function drawStationMachinery(ctx, st) {
    ctx.save();
    // Background
    ctx.fillStyle = "rgba(34,197,94,.04)";
    roundRect(ctx, st.x, st.y, st.w, st.h, 4);
    ctx.fill();
    ctx.strokeStyle = "rgba(34,197,94,.2)";
    ctx.lineWidth = 1;
    roundRect(ctx, st.x, st.y, st.w, st.h, 4);
    ctx.stroke();

    if (st.type === "conveyor") {
      // Animated roller lines (subtle)
      const nr = Math.floor(st.h / 8);
      const rollerOffset = (S.simT * 25) % 8;
      ctx.strokeStyle = "rgba(255,255,255,.12)";
      ctx.lineWidth = 1;
      for (let i = 0; i < nr; i++) {
        const ry = st.y + 4 + i * 8 + rollerOffset;
        if (ry < st.y + st.h - 4) {
          ctx.beginPath();
          ctx.moveTo(st.x + 4, ry);
          ctx.lineTo(st.x + st.w - 4, ry);
          ctx.stroke();
        }
      }
    }

    // Status dot
    ctx.fillStyle = `rgba(34,197,94,${0.6 + Math.sin(S.simT * 3) * 0.3})`;
    ctx.beginPath();
    ctx.arc(st.x + st.w / 2, st.y + (st.side === "top" ? 7 : st.h - 7), 2, 0, Math.PI * 2);
    ctx.fill();

    // Label
    ctx.fillStyle = "rgba(34,197,94,.75)";
    ctx.font = "600 6px Inter";
    ctx.textAlign = "center";
    ctx.fillText(st.label, st.x + st.w / 2, st.y + st.h / 2 + 3);
    ctx.restore();
  }

  /* ── 4. STAGING BAYS ──────────────────────────────────────── */
  function drawAlcoveBays(ctx, al) {
    ctx.save();
    ctx.fillStyle = "rgba(234,179,8,.03)";
    roundRect(ctx, al.x, al.y, al.w, al.h, 4);
    ctx.fill();
    ctx.strokeStyle = "rgba(234,179,8,.15)";
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    roundRect(ctx, al.x, al.y, al.w, al.h, 4);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = "rgba(234,179,8,.45)";
    ctx.font = "600 7px Inter";
    ctx.textAlign = "center";
    ctx.fillText(al.label, al.x + al.w / 2, al.y + al.h / 2 + 3);
    ctx.restore();
  }

  /* ── 5. CHARGING PADS ─────────────────────────────────────── */
  function drawChargingInfrastructure(ctx, pad) {
    ctx.save();
    // Subtle glow
    const glow = ctx.createRadialGradient(pad.x, pad.y, 4, pad.x, pad.y, 18);
    glow.addColorStop(0, `rgba(56,189,248,${0.06 + Math.sin(S.simT * 4) * 0.03})`);
    glow.addColorStop(1, "transparent");
    ctx.fillStyle = glow;
    ctx.fillRect(pad.x - 20, pad.y - 20, 40, 40);

    // Pad outline
    ctx.strokeStyle = "rgba(56,189,248,.2)";
    ctx.lineWidth = 1;
    roundRect(ctx, pad.x - 12, pad.y - 12, 24, 24, 4);
    ctx.stroke();

    // Contact dots
    ctx.fillStyle = "rgba(56,189,248,.5)";
    ctx.beginPath();
    ctx.arc(pad.x - 4, pad.y, 2, 0, Math.PI * 2);
    ctx.arc(pad.x + 4, pad.y, 2, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  /* ── 6. OVERHEAD WI-FI ACCESS POINTS ──────────────────────── */
  function drawOverheadWifiAPs(ctx) {
    const aps = [
      { x: 120, y: 150 }, { x: 840, y: 150 },
      { x: 480, y: 350 },
      { x: 120, y: 490 }, { x: 840, y: 490 },
    ];
    for (const ap of aps) {
      // Tiny AP dot
      ctx.fillStyle = "rgba(56,189,248,.3)";
      ctx.beginPath();
      ctx.arc(ap.x, ap.y, 1.5, 0, Math.PI * 2);
      ctx.fill();

      // Subtle pulse ring (only when connected)
      if (S.netMode !== 2) {
        ctx.strokeStyle = `rgba(56,189,248,${0.04 + Math.sin(S.simT * 2.5 + ap.x) * 0.02})`;
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.arc(ap.x, ap.y, 10 + (S.simT * 8) % 14, 0, Math.PI * 2);
        ctx.stroke();
      }
    }
  }

  /* ── 7. OBSTACLE CRATE ───────────────────────────────────── */
  function drawIndustrialCrate(ctx, obs) {
    ctx.save();
    // Danger zone glow
    ctx.fillStyle = "rgba(244,63,94,.08)";
    ctx.beginPath();
    ctx.arc(obs.x, obs.y, obs.r + 6, 0, Math.PI * 2);
    ctx.fill();

    // Crate box
    const bw = obs.r * 1.4;
    ctx.fillStyle = "rgba(244,63,94,.25)";
    roundRect(ctx, obs.x - bw / 2, obs.y - bw / 2, bw, bw, 3);
    ctx.fill();
    ctx.strokeStyle = "rgba(244,63,94,.6)";
    ctx.lineWidth = 1;
    roundRect(ctx, obs.x - bw / 2, obs.y - bw / 2, bw, bw, 3);
    ctx.stroke();

    // X mark
    ctx.strokeStyle = "rgba(255,255,255,.3)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(obs.x - bw / 3, obs.y - bw / 3);
    ctx.lineTo(obs.x + bw / 3, obs.y + bw / 3);
    ctx.moveTo(obs.x + bw / 3, obs.y - bw / 3);
    ctx.lineTo(obs.x - bw / 3, obs.y + bw / 3);
    ctx.stroke();
    ctx.restore();
  }

  /* ── 8. AMR FLEET ROBOT ────────────────────────────────────── */
  function drawAMR(ctx, b) {
    ctx.save();
    ctx.translate(b.x, b.y);

    // Soft ground shadow
    ctx.fillStyle = "rgba(0,0,0,.35)";
    ctx.beginPath();
    ctx.ellipse(1, 2, BOT_W / 2 + 2, BOT_H / 2 + 1, 0, 0, Math.PI * 2);
    ctx.fill();

    // LiDAR rays (subtle)
    if (S.layers.lidar && b.alive) {
      for (const hit of b.lidarHits) {
        ctx.strokeStyle = hit.hit ? "rgba(56,189,248,.2)" : "rgba(56,189,248,.04)";
        ctx.lineWidth = 0.7;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(hit.x - b.x, hit.y - b.y);
        ctx.stroke();
        if (hit.hit) {
          ctx.fillStyle = "rgba(56,189,248,.6)";
          ctx.beginPath();
          ctx.arc(hit.x - b.x, hit.y - b.y, 1.5, 0, Math.PI * 2);
          ctx.fill();
        }
      }
    }

    // Safety ring (dead zone / offline)
    if (b.inDZ || S.netMode === 2) {
      ctx.beginPath();
      ctx.arc(0, 0, b.safeR + 5, 0, Math.PI * 2);
      ctx.strokeStyle = "rgba(244,63,94,.35)";
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Yielding pulse ring
    if (b.yielding) {
      ctx.beginPath();
      ctx.arc(0, 0, BOT_R + 7, 0, Math.PI * 2);
      ctx.strokeStyle = `rgba(234,179,8,${0.25 + Math.sin(S.simT * 6) * 0.2})`;
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // "YIELD" label
      ctx.save();
      ctx.fillStyle = `rgba(234,179,8,${0.5 + Math.sin(S.simT * 6) * 0.3})`;
      ctx.font = "bold 6px Inter";
      ctx.textAlign = "center";
      ctx.fillText("YIELD", 0, -BOT_R - 10);
      ctx.restore();
    }

    // Collision avoidance flash ring (triggers on conflict detection)
    if (b._avoidFlash > 0) {
      const flashAlpha = Math.min(1, b._avoidFlash / 0.3);
      ctx.beginPath();
      ctx.arc(0, 0, BOT_R + 12 + (0.6 - b._avoidFlash) * 20, 0, Math.PI * 2);
      ctx.strokeStyle = `rgba(244,63,94,${flashAlpha * 0.5})`;
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Selection ring
    if (S.selBot === b.id) {
      ctx.beginPath();
      ctx.arc(0, 0, BOT_R + 6, 0, Math.PI * 2);
      ctx.strokeStyle = "rgba(255,255,255,.6)";
      ctx.lineWidth = 1.2;
      ctx.setLineDash([2, 2]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Rotate to heading
    ctx.rotate(b.theta);

    // Subtle forward headlight glow
    if (b.alive) {
      const hGrad = ctx.createRadialGradient(14, 0, 2, 28, 0, 22);
      hGrad.addColorStop(0, "rgba(254,240,138,.2)");
      hGrad.addColorStop(1, "rgba(254,240,138,0)");
      ctx.fillStyle = hGrad;
      ctx.beginPath();
      ctx.moveTo(12, -5);
      ctx.lineTo(34, -12);
      ctx.lineTo(34, 12);
      ctx.lineTo(12, 5);
      ctx.closePath();
      ctx.fill();
    }

    // Chassis body
    ctx.fillStyle = b.alive ? "#14151e" : "#252530";
    roundRect(ctx, -BOT_W / 2, -BOT_H / 2, BOT_W, BOT_H, 5);
    ctx.fill();

    // Colored accent border
    ctx.strokeStyle = b.alive ? b.color : "#52525b";
    ctx.lineWidth = 1.8;
    ctx.globalAlpha = b.alive ? 0.8 : 0.4;
    roundRect(ctx, -BOT_W / 2, -BOT_H / 2, BOT_W, BOT_H, 5);
    ctx.stroke();
    ctx.globalAlpha = 1.0;

    // Front LED indicators
    if (b.alive) {
      ctx.fillStyle = "#fef08a";
      ctx.globalAlpha = 0.8;
      ctx.beginPath();
      ctx.arc(BOT_W / 2 - 2, -4, 1.3, 0, Math.PI * 2);
      ctx.arc(BOT_W / 2 - 2, 4, 1.3, 0, Math.PI * 2);
      ctx.fill();
      ctx.globalAlpha = 1.0;
    }

    // Cargo on bed
    if (b.cargo) {
      ctx.fillStyle = b.cargo.fill;
      ctx.globalAlpha = 0.85;
      roundRect(ctx, -7, -6, 14, 12, 2);
      ctx.fill();
      ctx.strokeStyle = b.cargo.stroke;
      ctx.lineWidth = 0.8;
      roundRect(ctx, -7, -6, 14, 12, 2);
      ctx.stroke();
      ctx.globalAlpha = 1.0;
    } else {
      // Top plate accent
      ctx.fillStyle = b.alive ? b.color : "#3f3f46";
      ctx.globalAlpha = 0.5;
      roundRect(ctx, -6, -5, 10, 10, 2);
      ctx.fill();
      ctx.globalAlpha = 1.0;
    }

    // LiDAR turret dot
    ctx.fillStyle = "#0a0a0f";
    ctx.beginPath();
    ctx.arc(0, 0, 3.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "rgba(255,255,255,.4)";
    ctx.lineWidth = 0.8;
    ctx.stroke();

    // Rotating laser indicator
    if (b.alive && S.layers.lidar) {
      ctx.save();
      ctx.rotate(b.lidarAngle - b.theta);
      ctx.strokeStyle = `rgba(56,189,248,${0.25 + Math.sin(S.simT * 5) * 0.1})`;
      ctx.lineWidth = 0.8;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(22, 0);
      ctx.stroke();
      ctx.restore();
    }

    // Letter badge (always upright)
    ctx.rotate(-b.theta);
    ctx.fillStyle = "#ffffff";
    ctx.font = "bold 8px Inter";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(b.letter, 0, 0);

    // Name label below
    ctx.fillStyle = "rgba(255,255,255,.6)";
    ctx.font = "500 7px Inter";
    ctx.textBaseline = "top";
    ctx.fillText(b.name.split(" ")[1], 0, BOT_H / 2 + 3);

    if (b.cargo) {
      ctx.fillStyle = b.cargo.stroke;
      ctx.font = "600 6px JetBrains Mono";
      ctx.fillText(b.cargo.label, 0, -BOT_H / 2 - 9);
    }

    ctx.restore();
  }

  /* Decision Popover */
  function renderDecision(ctx) {
    const d = S.decision;
    if (!d || d.timer <= 0) return;

    if (d.isMulti && d.ranked && d.ranked.length >= 3) {
      // 4-Way Multi-Bot Decision Matrix
      const pw = 295, ph = 158;
      let px = d.x + 35;
      let py = d.y - ph / 2;
      if (px + pw > W - 10) px = d.x - pw - 35;
      if (py < 8) py = 8;
      if (py + ph > H - 8) py = H - ph - 8;

      const alpha = Math.min(1, d.timer / 0.4);
      ctx.save();
      ctx.globalAlpha = alpha;

      ctx.fillStyle = "rgba(10,11,16,.96)";
      roundRect(ctx, px, py, pw, ph, 8);
      ctx.fill();
      ctx.strokeStyle = "rgba(234,179,8,.35)";
      ctx.lineWidth = 1.2;
      roundRect(ctx, px, py, pw, ph, 8);
      ctx.stroke();

      ctx.strokeStyle = "rgba(234,179,8,.25)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(d.x, d.y);
      ctx.lineTo(px < d.x ? px + pw : px, py + ph / 2);
      ctx.stroke();

      ctx.fillStyle = "rgba(234,179,8,.12)";
      roundRectTop(ctx, px, py, pw, 22, 8);
      ctx.fill();
      ctx.fillStyle = "#eab308";
      ctx.font = "bold 9px JetBrains Mono";
      ctx.textAlign = "left";
      ctx.fillText("⚡ 4-WAY DECENTRALIZED CONFLICT RESOLUTION", px + 8, py + 14);

      let y = py + 34;
      ctx.fillStyle = "#9ca3af";
      ctx.font = "8px Inter";
      ctx.fillText("Multi-agent collision avoidance via ORCA + Spatial Tokens", px + 8, y);
      y += 13;

      ctx.fillStyle = "#64748b";
      ctx.font = "7px JetBrains Mono";
      ctx.fillText("RANK   AMR PEER         SCORE  (T+D+W+B)   ACTION", px + 8, y);
      y += 11;

      d.ranked.forEach((item, idx) => {
        const isWin = idx === 0;
        ctx.fillStyle = item.bot.color;
        ctx.font = isWin ? "bold 8px JetBrains Mono" : "8px JetBrains Mono";
        const rankStr = `#${idx + 1}`.padEnd(6, " ");
        const nameStr = item.bot.name.padEnd(16, " ");
        const scStr = `${item.score}`.padEnd(6, " ");
        const dStr = `${item.t}+${item.d}+${item.w}+${item.b}`.padEnd(11, " ");
        const actStr = isWin ? "PASS (TOKEN)" : `YIELD (HOLD #${idx})`;
        ctx.fillText(`${rankStr}${nameStr}${scStr}${dStr}${actStr}`, px + 8, y);
        y += 13;
      });

      y += 2;
      ctx.fillStyle = "#22c55e";
      ctx.font = "600 7.5px JetBrains Mono";
      ctx.fillText("✓ Deterministic Queue Handover — 0 Deadlocks, 0 Collisions", px + 8, y);

      ctx.restore();
      return;
    }

    // Pairwise 2-Bot Decision Popover
    const pw = 240, ph = 125;
    let px = d.x + 40;
    let py = d.y - ph / 2;
    if (px + pw > W - 10) px = d.x - pw - 40;
    if (py < 8) py = 8;
    if (py + ph > H - 8) py = H - ph - 8;

    const alpha = Math.min(1, d.timer / 0.4);
    ctx.save();
    ctx.globalAlpha = alpha;

    ctx.fillStyle = "rgba(11,11,15,.95)";
    roundRect(ctx, px, py, pw, ph, 8);
    ctx.fill();
    ctx.strokeStyle = "rgba(255,255,255,.12)";
    ctx.lineWidth = 1;
    roundRect(ctx, px, py, pw, ph, 8);
    ctx.stroke();

    ctx.strokeStyle = "rgba(255,255,255,.12)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(d.x, d.y);
    ctx.lineTo(px < d.x ? px + pw : px, py + ph / 2);
    ctx.stroke();

    ctx.fillStyle = "rgba(255,255,255,.05)";
    roundRectTop(ctx, px, py, pw, 20, 8);
    ctx.fill();
    ctx.fillStyle = "#fff";
    ctx.font = "bold 8px Inter";
    ctx.textAlign = "left";
    ctx.fillText("🔀 DECENTRALIZED CONFLICT RESOLUTION", px + 7, py + 13);

    let y = py + 28;
    ctx.fillStyle = "#6b7280";
    ctx.font = "7px JetBrains Mono";
    ctx.fillText("Score = Task + Urgency + Wait + Battery", px + 7, y);
    y += 12;

    ctx.fillStyle = d.winner.color;
    ctx.font = "600 9px Inter";
    ctx.textAlign = "left";
    ctx.fillText(`▸ ${d.winner.name} PASSES`, px + 7, y);
    ctx.fillStyle = "#fff";
    ctx.font = "600 9px JetBrains Mono";
    ctx.textAlign = "right";
    ctx.fillText(`${d.ws}`, px + pw - 7, y);
    y += 11;
    ctx.textAlign = "left";
    ctx.fillStyle = "#64748b";
    ctx.font = "8px JetBrains Mono";
    ctx.fillText(`T:${d.wD.t}  D:${d.wD.d}  W:${d.wD.w}  B:${d.wD.b}`, px + 14, y);
    y += 14;

    ctx.fillStyle = d.loser.color;
    ctx.font = "600 9px Inter";
    ctx.textAlign = "left";
    ctx.fillText(`▾ ${d.loser.name} YIELDS`, px + 7, y);
    ctx.fillStyle = "#fff";
    ctx.font = "600 9px JetBrains Mono";
    ctx.textAlign = "right";
    ctx.fillText(`${d.ls}`, px + pw - 7, y);
    y += 11;
    ctx.textAlign = "left";
    ctx.fillStyle = "#64748b";
    ctx.font = "8px JetBrains Mono";
    ctx.fillText(`T:${d.lD.t}  D:${d.lD.d}  W:${d.lD.w}  B:${d.lD.b}`, px + 14, y);
    y += 14;

    ctx.fillStyle = C.beta;
    ctx.font = "8px JetBrains Mono";
    ctx.fillText(`θ_SVO → ${d.loser.svo}° (${d.loser.svoLabel})`, px + 7, y);

    ctx.restore();
  }

  /* DC-MRTA Auction Overlay */
  function renderAuctionMatrix(ctx) {
    const a = S.auctionOverlay;
    if (!a || a.timer <= 0) return;

    const pw = 270, ph = 145;
    const px = W - pw - 14, py = 45;
    const alpha = Math.min(1, a.timer / 0.5);

    ctx.save();
    ctx.globalAlpha = alpha;

    ctx.fillStyle = "rgba(10,11,16,.96)";
    roundRect(ctx, px, py, pw, ph, 8);
    ctx.fill();
    ctx.strokeStyle = "rgba(56,189,248,.35)";
    ctx.lineWidth = 1.2;
    roundRect(ctx, px, py, pw, ph, 8);
    ctx.stroke();

    ctx.fillStyle = "rgba(56,189,248,.12)";
    roundRectTop(ctx, px, py, pw, 22, 8);
    ctx.fill();

    ctx.fillStyle = "#38bdf8";
    ctx.font = "bold 9px JetBrains Mono";
    ctx.textAlign = "left";
    ctx.fillText(`📢 DC-MRTA AUCTION: ORDER #${a.orderId}`, px + 8, py + 14);

    let y = py + 34;
    ctx.fillStyle = "#9ca3af";
    ctx.font = "8px Inter";
    ctx.fillText(`Cargo: ${a.cargo.label} (${a.weight}kg) • ${a.bay} → ${a.dock}`, px + 8, y);
    y += 14;

    ctx.fillStyle = "#64748b";
    ctx.font = "7px JetBrains Mono";
    ctx.fillText("AMR PEER         DIST   BAT    STATUS    BID COST", px + 8, y);
    y += 10;

    a.bids.forEach((bid, idx) => {
      const isWinner = idx === 0;
      ctx.fillStyle = isWinner ? "#22c55e" : "#9ca3af";
      ctx.font = isWinner ? "bold 8px JetBrains Mono" : "8px JetBrains Mono";
      const name = bid.bot.name.padEnd(14, " ");
      const dist = `${bid.d}px`.padEnd(7, " ");
      const bat = `${bid.bat}%`.padEnd(7, " ");
      const st = isWinner ? "WINNER " : "LOSE   ";
      ctx.fillText(`${name} ${dist}${bat}${st}  ${bid.cost}`, px + 8, y);
      y += 13;
    });

    ctx.restore();
  }

  function roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.lineTo(x + w - r, y);
    ctx.quadraticCurveTo(x + w, y, x + w, y + r);
    ctx.lineTo(x + w, y + h - r);
    ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
    ctx.lineTo(x + r, y + h);
    ctx.quadraticCurveTo(x, y + h, x, y + h - r);
    ctx.lineTo(x, y + r);
    ctx.quadraticCurveTo(x, y, x + r, y);
    ctx.closePath();
  }

  function roundRectTop(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.lineTo(x + w - r, y);
    ctx.quadraticCurveTo(x + w, y, x + w, y + r);
    ctx.lineTo(x + w, y + h);
    ctx.lineTo(x, y + h);
    ctx.lineTo(x, y + r);
    ctx.quadraticCurveTo(x, y, x + r, y);
    ctx.closePath();
  }

  /* ================================================================
     13. INSPECTOR & TELEMETRY UI
     ================================================================ */
  function updateInsp() {
    const b = S.bots[S.selBot];
    if (!b) return;
    const el = (id) => document.getElementById(id);

    el("iName").textContent = b.name;
    el("iHw").textContent = `${b.hw} — ${b.role}`;

    const st = el("iStatus");
    if (b.parked) {
      st.textContent = "PARKED @ BASE";
      st.className = "status-pill base-parked";
    } else if (b.halted) {
      st.textContent = "RETURNING TO BASE";
      st.className = "status-pill yield";
    } else if (!b.alive) {
      st.textContent = "OFFLINE";
      st.className = "status-pill off";
    } else if (b.inDZ) {
      st.textContent = "BLACKOUT";
      st.className = "status-pill yield";
    } else if (b.yielding) {
      st.textContent = b._queuePos ? `HOLD #${b._queuePos}` : "YIELDING";
      st.className = "status-pill yield";
    } else if (b.currentTask) {
      st.textContent = "TASK ACTIVE";
      st.className = "status-pill";
    } else {
      st.textContent = "ONLINE";
      st.className = "status-pill";
    }

    el("iPose").textContent = `${Math.round(b.x)}, ${Math.round(b.y)}, ${(b.theta * 180 / Math.PI).toFixed(0)}°`;
    el("iVel").textContent = `${(b.spd / BASE_SPD).toFixed(2)} m/s (ω:${b.omega.toFixed(1)})`;
    el("iBat").textContent = `${Math.round(b.bat)}%`;
    const batBar = el("iBatBar");
    if (batBar) {
      batBar.style.width = `${Math.max(4, Math.round(b.bat))}%`;
      batBar.style.background = b.bat > 50 ? "var(--green)" : b.bat > 25 ? "var(--amber)" : "var(--rose)";
    }
    el("iSvo").textContent = `θ:${b.svo}° (${b.svoLabel})`;
    el("iCap").textContent = `${b.maxPayload}kg / ${b.maxSpeed} m/s`;
    el("iPayload").textContent = b.cargo ? `${b.cargo.label} (${b.cargo.weight}kg)` : (b.currentTask ? "Navigating to Pickup" : "Empty (Standby)");

    const score = calcScore(b);
    el("iScore").textContent = `${score}  (T:${b.taskPriority} D:${b.deadlineUrgency} W:${Math.min(3, Math.floor(b.waitTime))} B:${b.bat > 60 ? 1 : b.bat > 30 ? 2 : 3})`;

    const nm = NET_MODES[S.netMode];
    const crl = el("commRLabel");
    if (crl) crl.textContent = nm.commR || "0";

    const pl = el("iPeers");
    let ph = "";
    for (const [id, o] of Object.entries(S.bots)) {
      if (id === b.id) continue;
      const d = Math.round(v2.dist(b, o));
      const inR = nm.commR > 0 && d <= nm.commR;
      const cls = !o.alive ? "dead" : inR ? "ok" : "disc";
      const txt = !o.alive ? "OFFLINE" : inR ? `${Math.round(nm.lat + d / 15)}ms` : "DISC";
      ph += `<div class="peer-row"><span>${o.name} (${d}px)</span><span class="peer-tag ${cls}">${txt}</span></div>`;
    }
    pl.innerHTML = ph;

    const lg = el("iLog");
    const tagCls = {
      ORCA: "t-orca", SVO: "t-svo", MRTA: "t-mrta", RF: "t-rf",
      OBS: "t-obs", INIT: "t-init", ZENOH: "t-zenoh", FAULT: "t-mrta",
      WMS: "t-wms", RES: "t-res",
    };
    let lh = "";
    for (const e of b.log.slice(0, 6)) {
      const tc = tagCls[e.tag] || "t-init";
      lh += `<div class="log-row">[${e.ts}] <b class="${tc}">${e.tag}</b> ${e.msg}</div>`;
    }
    lg.innerHTML = lh;

    if (b.parked) {
      el("killText").textContent = `✔ Resume ${b.name}`;
      el("killBtn").className = "kill-btn revive";
    } else if (b.halted) {
      el("killText").textContent = `🚶 Returning...`;
      el("killBtn").className = "kill-btn yield-btn";
    } else {
      el("killText").textContent = `⚡ Recall ${b.name}`;
      el("killBtn").className = "kill-btn";
    }

    let alive = 0;
    for (const r of Object.values(S.bots)) if (r.alive) alive++;
    el("peerCount").textContent = alive;

    // Update ROS 2 Topics
    updateRosTopics(b);
  }

  function updateRosTopics(b) {
    const el = (id) => document.getElementById(id);
    if (!el("topCmdVel")) return;

    el("topCmdVel").textContent = `linear.x: ${(b.spd / BASE_SPD).toFixed(2)}, angular.z: ${b.omega.toFixed(2)}`;
    el("topOdom").textContent = `pose.pos: (${(b.x / 100).toFixed(2)}, ${(b.y / 100).toFixed(2)}) θ: ${(b.theta * 180 / Math.PI).toFixed(0)}°`;
    const hitCount = b.lidarHits.filter((h) => h.hit).length;
    el("topScan").textContent = `ranges: 360 beams, obstacles_in_range: ${hitCount}`;
    el("topIntent").textContent = `waypoints: ${b.intent.length}, prio: ${b.taskPriority}, svo: ${b.svo}°`;
    el("topBid").textContent = b.currentTask
      ? `assigned: Order #${b.currentTask.orderId} (${b.currentTask.cargo.label})`
      : `last_bid: DC-MRTA active (cost: ${calcScore(b)})`;
    
    // Find active zone
    const activeZone = RESERVATION_ZONES.find((z) => z.owner === b.id && z.ttl > 0);
    el("topReserve").textContent = activeZone
      ? `claimed_zone: ${activeZone.id} (TTL: ${activeZone.ttl.toFixed(1)}s)`
      : `claimed_zone: None (Monitoring mesh)`;

    const nm = NET_MODES[S.netMode];
    el("ddsLoss").textContent = nm.loss;
    el("ddsLoss").className = "card-v mono " + (S.netMode === 0 ? "green" : S.netMode === 1 ? "amber" : "rose");
  }

  function updateKPI() {
    const nm = NET_MODES[S.netMode];
    const latEl = document.getElementById("kpiLat");
    if (latEl) {
      const j = nm.lat + Math.sin(S.simT * 3) * nm.lat * 0.08;
      latEl.textContent = nm.lat >= 999 ? "∞" : `${j.toFixed(0)}ms`;
      latEl.style.color = nm.color;
    }
    const netEl = document.getElementById("kpiNet");
    if (netEl) {
      netEl.textContent = nm.name;
      netEl.style.color = nm.color;
    }
    const delEl = document.getElementById("kpiDelivered");
    if (delEl) {
      delEl.textContent = `${S.deliveredCount} pkgs`;
    }
    const avEl = document.getElementById("kpiAvoided");
    if (avEl) {
      avEl.textContent = `${S.conflictsAvoided}`;
    }
    const meshEl = document.getElementById("kpiMesh");
    if (meshEl) {
      let activeLinks = 0;
      const bl = Object.values(S.bots).filter(b => b.alive);
      if (nm.commR > 0) {
        for (let i = 0; i < bl.length; i++) {
          for (let j = i + 1; j < bl.length; j++) {
            if (v2.dist(bl[i], bl[j]) <= nm.commR && !bl[i].inDZ && !bl[j].inDZ) activeLinks++;
          }
        }
      }
      meshEl.textContent = nm.commR === 0 ? "Offline (0)" : `${activeLinks} P2P`;
      meshEl.style.color = nm.color;
    }
  }

  /* Trigger simultaneous 4-way conflict edge case */
  function trigger4BotConflict() {
    if (Object.keys(S.bots).length < 4) return;
    const center = { x: 480, y: 290 };
    const d = 115;
    const approaches = [
      { id: "alpha", x: center.x - d, y: center.y, theta: 0, patrol: ["2_3", "2_6", "2_3", "2_0"] },
      { id: "beta", x: center.x, y: center.y - d, theta: Math.PI / 2, patrol: ["2_3", "4_3", "2_3", "0_3"] },
      { id: "gamma", x: center.x + d, y: center.y, theta: Math.PI, patrol: ["2_3", "2_0", "2_3", "2_6"] },
      { id: "delta", x: center.x, y: center.y + d, theta: -Math.PI / 2, patrol: ["2_3", "0_3", "2_3", "4_3"] },
    ];

    approaches.forEach(app => {
      const b = S.bots[app.id];
      if (b) {
        b.alive = true;
        b.x = app.x;
        b.y = app.y;
        b.theta = app.theta;
        b.yielding = false;
        b.yieldTo = null;
        b._queuePos = 0;
        b.waitTime = 0;
        b.setPatrol(app.patrol);
      }
    });

    showToast("💥 4-Way Multi-Bot Conflict Triggered: Simultaneous Arrival at Central 4-Way!");
    addFloatingText(480, 260, "⚡ 4-WAY CONVERGENCE", "#eab308");
  }

  /* ================================================================
     14. EVENT HANDLERS & INTERACTIONS
     ================================================================ */
  function setupEvents() {
    document
      .querySelectorAll(".sc-btn")
      .forEach((b) => b.addEventListener("click", () => loadScenario(+b.dataset.scenario)));

    const ppb = document.getElementById("playPauseBtn");
    ppb.addEventListener("click", () => {
      S.running = !S.running;
      ppb.textContent = S.running ? "⏸" : "▶";
    });

    const sb = document.getElementById("speedBtn");
    sb.addEventListener("click", () => {
      S.speed = S.speed === 1 ? 2 : S.speed === 2 ? 0.5 : 1;
      sb.textContent = S.speed + "x";
    });

    document.getElementById("resetBtn").addEventListener("click", () => {
      S.simT = 0;
      S.deliveredCount = 0;
      S.conflictsAvoided = 0;
      initBots();
      loadScenario(S.scenario);
    });

    const st = document.getElementById("serverToggle");
    const hdr = document.getElementById("appHeader");
    const sl = document.getElementById("serverLabel");
    st.addEventListener("change", (e) => {
      S.serverOn = e.target.checked;
      hdr.classList.toggle("offline", !S.serverOn);
      sl.textContent = S.serverOn ? "Cloud" : "Severed";
      showToast(
        S.serverOn
          ? "Central gateway online"
          : "Central severed — P2P edge autonomy continues"
      );
    });

    document.querySelectorAll(".net-btn").forEach((b) =>
      b.addEventListener("click", () => {
        const mode = +b.dataset.mode;
        S.netMode = mode;
        document
          .querySelectorAll(".net-btn")
          .forEach((x) => x.classList.toggle("active", +x.dataset.mode === mode));
        const nm = NET_MODES[mode];
        showToast(
          `Network: ${nm.name}${mode === 2 ? " — LiDAR safety bubble σ=1.5" : nm.commR > 0 ? ` (Rc=${nm.commR}px, ~${nm.lat}ms)` : ""}`
        );
        for (const bot of Object.values(S.bots)) {
          if (bot.alive) bot.addLog("ZENOH", `Net → ${nm.name}`);
        }
      })
    );

    // Engineer Quick Actions (WMS Order Dispatch & 4-Bot Conflict)
    const iob = document.getElementById("inspOrderBtn");
    if (iob) iob.addEventListener("click", dispatchNewWmsOrder);

    const tcb = document.getElementById("triggerConflictBtn");
    if (tcb) tcb.addEventListener("click", trigger4BotConflict);

    // Sub-Tabs Switcher (Telemetry vs ROS 2 Topics)
    document.getElementById("subTabTele").addEventListener("click", () => {
      document.getElementById("subTabTele").classList.add("active");
      document.getElementById("subTabRos").classList.remove("active");
      document.getElementById("viewTelemetry").style.display = "flex";
      document.getElementById("viewRos").style.display = "none";
    });

    document.getElementById("subTabRos").addEventListener("click", () => {
      document.getElementById("subTabRos").classList.add("active");
      document.getElementById("subTabTele").classList.remove("active");
      document.getElementById("viewTelemetry").style.display = "none";
      document.getElementById("viewRos").style.display = "flex";
    });

    document.getElementById("robotTabs").addEventListener("click", (e) => {
      const tab = e.target.closest(".tab");
      if (!tab) return;
      document.querySelectorAll(".tab").forEach((t) => t.classList.remove("active"));
      tab.classList.add("active");
      S.selBot = tab.dataset.robot;
      updateInsp();
    });

    document.getElementById("killBtn").addEventListener("click", () => {
      const b = S.bots[S.selBot];
      if (!b) return;
      if (!b.halted && !b.parked) {
        b.halt();
      } else {
        b.resume();
      }
      updateInsp();
    });

    // Prevent default context menu for right click manual dispatch
    S.canvas.addEventListener("contextmenu", (e) => {
      e.preventDefault();
      const rect = S.canvas.getBoundingClientRect();
      const cx = ((e.clientX - rect.left) / rect.width) * W;
      const cy = ((e.clientY - rect.top) / rect.height) * H;

      const sel = S.bots[S.selBot];
      if (sel && sel.alive) {
        const targetNode = nearestNode(cx, cy, true);
        if (targetNode) {
          const start = nearestNode(sel.x, sel.y, true);
          if (start) {
            const path = astar(start.id, targetNode.id);
            if (path.length > 0) {
              sel.navPath = smoothPath(path);
              sel.wpIdx = 0;
              sel.wp = sel.navPath[0];
              sel.addLog("ORCA", `Manual dispatch to (${Math.round(cx)}, ${Math.round(cy)})`);
              S.manualGoal = { x: cx, y: cy };
              showToast(`Dispatched ${sel.name} to target waypoint`);
            }
          }
        }
      }
    });

    // Left Click Canvas Interaction
    S.canvas.addEventListener("click", (e) => {
      const rect = S.canvas.getBoundingClientRect();
      const cx = ((e.clientX - rect.left) / rect.width) * W;
      const cy = ((e.clientY - rect.top) / rect.height) * H;

      for (const [id, b] of Object.entries(S.bots)) {
        if (v2.dist({ x: cx, y: cy }, b) < BOT_R + 12) {
          S.selBot = id;
          document
            .querySelectorAll(".tab")
            .forEach((t) => t.classList.toggle("active", t.dataset.robot === id));
          updateInsp();
          return;
        }
      }

      if (S.scenario === 3) {
        const ex = S.obstacles.findIndex((o) => v2.dist({ x: cx, y: cy }, o) < o.r + 12);
        if (ex !== -1) {
          removeObstacle(ex);
          showToast("Obstacle removed — A* paths recalculated");
        } else {
          const nn = nearestNode(cx, cy, false);
          if (nn && !blockedNodes.has(nn.id) && v2.dist({ x: cx, y: cy }, nn) < 60) {
            spawnObstacle(nn.id);
            for (const bot of Object.values(S.bots)) {
              if (bot.alive) bot.addLog("OBS", `Blocked node ${nn.id} — rerouting`);
            }
            showToast(`Obstacle placed at (${nn.x}, ${nn.y}) — fleet auto-rerouted`);
          }
        }
      }
    });
  }

  /* Build Robot Tabs */
  function buildTabs() {
    const container = document.getElementById("robotTabs");
    let html = "";
    PROFILES.forEach((cfg, i) => {
      const active = i === 0 ? " active" : "";
      html += `<button class="tab${active}" data-robot="${cfg.id}"><span class="tab-dot" style="background:${cfg.color}"></span>${cfg.name.split(" ")[1]}</button>`;
    });
    container.innerHTML = html;
  }

  /* High-DPI Canvas Resolution Setup */
  function setupRetinaCanvas() {
    S.canvas = document.getElementById("simCanvas");
    if (!S.canvas) return;
    S.dpr = window.devicePixelRatio || 1;
    S.canvas.width = W * S.dpr;
    S.canvas.height = H * S.dpr;
    S.ctx = S.canvas.getContext("2d");
    S.ctx.scale(S.dpr, S.dpr);
  }

  /* ================================================================
     15. MAIN ANIMATION LOOP
     ================================================================ */
  function loop(ts) {
    if (!S.lt) S.lt = ts;
    const dt = Math.min((ts - S.lt) / 1000, 0.1) * S.speed;
    S.lt = ts;

    if (S.running) {
      S.simT += dt;

      // Update AMRs
      for (const b of Object.values(S.bots)) b.update(dt);
      checkConflicts();

      // Update Spatial Reservation Token TTLs
      for (const zone of RESERVATION_ZONES) {
        if (zone.ttl > 0) {
          zone.ttl -= dt;
          if (zone.ttl <= 0) {
            zone.owner = null;
            zone.ttl = 0;
          }
        }
      }

      // Update Decision Popover
      if (S.decision) {
        S.decision.timer -= dt;
        if (S.decision.timer <= 0) S.decision = null;
      }

      // Update Auction Overlay
      if (S.auctionOverlay) {
        S.auctionOverlay.timer -= dt;
        if (S.auctionOverlay.timer <= 0) S.auctionOverlay = null;
      }

      // Update Floating Texts
      for (let i = S.floatingTexts.length - 1; i >= 0; i--) {
        const ft = S.floatingTexts[i];
        ft.life -= dt;
        ft.y -= dt * 16;
        if (ft.life <= 0) S.floatingTexts.splice(i, 1);
      }

      // Periodically trigger ambient WMS order every 28 seconds
      if (Math.floor(S.simT) % 28 === 0 && Math.random() < 0.03) {
        dispatchNewWmsOrder();
      }
    }

    if (S.ctx) render(S.ctx);
    updateInsp();
    updateKPI();

    requestAnimationFrame(loop);
  }

  /* ================================================================
     15. STARTUP SPLASH SCREEN (7-8s INITIALIZATION)
     ================================================================ */
  function startSplashLoading() {
    const splash = document.getElementById("splashScreen");
    const bar = document.getElementById("splashBarFill");
    const pctEl = document.getElementById("splashPercent");
    const stepEl = document.getElementById("splashStepText");
    const skipBtn = document.getElementById("splashSkipBtn");
    if (!splash || !bar) return;

    const totalDuration = 7500; // 7.5 seconds (7-8 sec range)
    const startTime = performance.now();
    let completed = false;

    const steps = [
      { at: 0, text: "Initializing ROS 2 Humble DDS & rmw_zenoh P2P middleware..." },
      { at: 18, text: "Loading SLAM corridor topology & warehouse storage bays (A–H)..." },
      { at: 38, text: "Calibrating ORCA & Social Value Orientation (SVO) collision avoidance..." },
      { at: 60, text: "Synchronizing DC-MRTA task auctioning & spatial reservation token mesh..." },
      { at: 82, text: "Bootstrapping 4 heterogeneous edge compute nodes (Jetson Orin & RPi 5)..." },
      { at: 96, text: "All systems nominal. Launching decentralized fleet coordinator..." },
    ];

    function finishSplash() {
      if (completed) return;
      completed = true;
      if (bar) bar.style.width = "100%";
      if (pctEl) pctEl.textContent = "100%";
      if (stepEl) stepEl.textContent = "All systems nominal. Ready!";
      setTimeout(() => {
        splash.classList.add("hidden");
        showToast("⬡ Synergy AMR: 4-Bot Decentralized Fleet Online");
      }, 350);
    }

    if (skipBtn) skipBtn.addEventListener("click", finishSplash);

    function tick(now) {
      if (completed) return;
      const elapsed = now - startTime;
      const progress = Math.min(100, (elapsed / totalDuration) * 100);

      if (bar) bar.style.width = `${progress.toFixed(1)}%`;
      if (pctEl) pctEl.textContent = `${Math.floor(progress)}%`;

      // Update current step text
      for (let i = steps.length - 1; i >= 0; i--) {
        if (progress >= steps[i].at) {
          if (stepEl && stepEl.textContent !== steps[i].text) {
            stepEl.textContent = steps[i].text;
          }
          break;
        }
      }

      if (progress < 100) {
        requestAnimationFrame(tick);
      } else {
        finishSplash();
      }
    }

    requestAnimationFrame(tick);
  }

  /* ================================================================
     16. INITIALIZATION
     ================================================================ */
  function init() {
    setupRetinaCanvas();
    buildGraph();
    buildTabs();
    initBots();
    setupEvents();
    loadScenario(1);
    startSplashLoading();
    requestAnimationFrame(loop);
  }

  if (document.readyState === "loading")
    document.addEventListener("DOMContentLoaded", init);
  else init();
})();