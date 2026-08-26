/**
 * ============================================================================
 * SYNERGY-AMR v4 — Realistic Industrial Warehouse & Autonomous Fleet
 * ============================================================================
 * Features:
 *   • Natural Open-Floor SLAM Navigation (No fixed lane/track lines)
 *   • High-fidelity AMR rendering: industrial chassis, wheels, LED headlights,
 *     LiDAR 360° sweep beam, top payload pallets, and status lighting
 *   • Realistic warehouse: industrial rack shelving with colorful cargo pallets,
 *     conveyor roller stations, hazard-bordered staging bays, charging docks
 *   • A* pathfinding on natural brownfield floor
 *   • Explainable Decision Popover for conflict negotiation (Priority Score)
 *   • Heterogeneous fleet profiles & 4-state network degradation
 * ============================================================================
 */
(function () {
  "use strict";

  /* ================================================================
     1. CONSTANTS & THEME
     ================================================================ */
  const W = 960, H = 580;
  const BOT_W = 30, BOT_H = 24; // AMR physical dimensions
  const BOT_R = 14;
  const BASE_SPD = 1.6;

  const C = {
    floor: "#0b0c10",
    floorTile: "rgba(255,255,255,.015)",
    jointLine: "rgba(255,255,255,.03)",
    rackBody: "#14151c",
    rackFrame: "#252736",
    rackLabel: "#606275",
    dockFill: "rgba(34,197,94,.05)",
    dockBorder: "rgba(34,197,94,.25)",
    bayFill: "rgba(234,179,8,.04)",
    bayBorder: "rgba(234,179,8,.2)",
    deadZone: "rgba(244,63,94,.05)",
    deadZoneStroke: "rgba(244,63,94,.3)",
    obstacle: "#ef4444",
    alpha: "#38bdf8",
    beta: "#eab308",
    gamma: "#22c55e",
    delta: "#a78bfa",
    epsilon: "#f472b6",
  };

  /* Pallet Cargo Color Palette */
  const CARGO_COLORS = [
    { fill: "#3b82f6", stroke: "#60a5fa", label: "SKU-A" },
    { fill: "#f59e0b", stroke: "#fbbf24", label: "SKU-B" },
    { fill: "#10b981", stroke: "#34d399", label: "SKU-C" },
    { fill: "#8b5cf6", stroke: "#a78bfa", label: "SKU-D" },
    { fill: "#64748b", stroke: "#94a3b8", label: "SKU-E" },
    { fill: "#d97706", stroke: "#fcd34d", label: "WOOD" },
  ];

  /* ================================================================
     2. WAREHOUSE MAP (Realistic Storage Layout)
     ================================================================ */
  const RACKS = [
    // Row 1 (Top)
    { x: 55, y: 48, w: 135, h: 56, label: "BAY A1-A4", slots: 4, seed: 1 },
    { x: 230, y: 48, w: 125, h: 56, label: "BAY A5-A8", slots: 4, seed: 2 },
    { x: 605, y: 48, w: 125, h: 56, label: "BAY B1-B4", slots: 4, seed: 3 },
    { x: 770, y: 48, w: 135, h: 56, label: "BAY B5-B8", slots: 4, seed: 4 },
    // Row 2
    { x: 55, y: 184, w: 135, h: 56, label: "BAY C1-C4", slots: 4, seed: 5 },
    { x: 230, y: 184, w: 125, h: 56, label: "BAY C5-C8", slots: 4, seed: 6 },
    { x: 605, y: 184, w: 125, h: 56, label: "BAY D1-D4", slots: 4, seed: 7 },
    { x: 770, y: 184, w: 135, h: 56, label: "BAY D5-D8", slots: 4, seed: 8 },
    // Row 3
    { x: 55, y: 340, w: 135, h: 56, label: "BAY E1-E4", slots: 4, seed: 9 },
    { x: 230, y: 340, w: 125, h: 56, label: "BAY E5-E8", slots: 4, seed: 10 },
    { x: 605, y: 340, w: 125, h: 56, label: "BAY F1-F4", slots: 4, seed: 11 },
    { x: 770, y: 340, w: 135, h: 56, label: "BAY F5-F8", slots: 4, seed: 12 },
    // Row 4 (Bottom)
    { x: 55, y: 466, w: 135, h: 56, label: "BAY G1-G4", slots: 4, seed: 13 },
    { x: 230, y: 466, w: 125, h: 56, label: "BAY G5-G8", slots: 4, seed: 14 },
    { x: 605, y: 466, w: 125, h: 56, label: "BAY H1-H4", slots: 4, seed: 15 },
    { x: 770, y: 466, w: 135, h: 56, label: "BAY H5-H8", slots: 4, seed: 16 },
  ];

  /* Industrial Stations (Conveyor docks & Workstations) */
  const STATIONS = [
    { x: 4, y: 260, w: 32, h: 60, label: "PICKING CONVEYOR", type: "conveyor", side: "left" },
    { x: 924, y: 260, w: 32, h: 60, label: "PACKING DOCK", type: "conveyor", side: "right" },
    { x: 430, y: 4, w: 100, h: 26, label: "INBOUND INDUCTION", type: "dock", side: "top" },
    { x: 430, y: 550, w: 100, h: 26, label: "OUTBOUND DISPATCH", type: "dock", side: "bottom" },
  ];

  /* Staging Bays with Hazard Striping */
  const ALCOVES = [
    { x: 410, y: 115, w: 60, h: 38, label: "STAGING 01" },
    { x: 490, y: 115, w: 60, h: 38, label: "STAGING 02" },
    { x: 410, y: 425, w: 60, h: 38, label: "STAGING 03" },
    { x: 490, y: 425, w: 60, h: 38, label: "STAGING 04" },
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
    return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
  }

  /* ================================================================
     4. VECTOR MATH
     ================================================================ */
  const v2 = {
    dist: (a, b) => Math.hypot(a.x - b.x, a.y - b.y),
    len: (v) => Math.hypot(v.x, v.y),
    norm: (v) => {
      const l = Math.hypot(v.x, v.y);
      return l > 1e-4 ? { x: v.x / l, y: v.y / l } : { x: 0, y: 0 };
    },
    sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y }),
  };

  /* ================================================================
     5. NETWORK MODES
     ================================================================ */
  const NET_MODES = [
    { name: "Connected", commR: 210, lat: 11, safety: 1.0, color: "#22c55e" },
    { name: "Degraded", commR: 110, lat: 240, safety: 1.3, color: "#eab308" },
    { name: "Offline", commR: 0, lat: 999, safety: 1.5, color: "#f43f5e" },
    { name: "Recovery", commR: 170, lat: 75, safety: 1.1, color: "#38bdf8" },
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
    {
      id: "epsilon", name: "Robot Epsilon", letter: "ε", color: C.epsilon,
      hw: "Raspberry Pi 5", role: "Scout / Courier",
      maxPayload: 50, maxSpeed: 2.6, bat: 91,
      svo: 60, svoLabel: "Prosocial",
      taskPriority: 2, deadlineUrgency: 3, cargoType: "small_tote",
    },
  ];

  /* ================================================================
     7. STATE
     ================================================================ */
  const S = {
    canvas: null,
    ctx: null,
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
    bots: {},
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
      this.alive = true;
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
      this.curPayload = `${p.role} (${p.maxPayload}kg)`;
      this.addLog("INIT", `SLAM active on ${p.hw}`);
    }

    addLog(tag, msg) {
      const m = String(Math.floor(S.simT / 60)).padStart(2, "0");
      const s = String(Math.floor(S.simT % 60)).padStart(2, "0");
      this.log.unshift({ ts: `${m}:${s}`, tag, msg });
      if (this.log.length > 30) this.log.pop();
    }

    setPatrol(nodeIds) {
      this.patrol = nodeIds;
      this.patrolIdx = 0;
      this._goToPatrolTarget();
    }

    _goToPatrolTarget() {
      const targetId = this.patrol[this.patrolIdx];
      const start = nearestNode(this.x, this.y, true);
      if (!start) return;
      const path = astar(start.id, targetId);
      if (path.length > 0) {
        this.navPath = path;
        this.wpIdx = 0;
        this.wp = this.navPath[0];
      } else {
        this.patrolIdx = (this.patrolIdx + 1) % this.patrol.length;
        const start2 = nearestNode(this.x, this.y, true);
        if (start2) {
          const path2 = astar(start2.id, this.patrol[this.patrolIdx]);
          if (path2.length > 0) {
            this.navPath = path2;
            this.wpIdx = 0;
            this.wp = this.navPath[0];
          }
        }
      }
    }

    recalcPath() {
      if (this.patrol.length === 0) return;
      this._goToPatrolTarget();
    }

    advancePatrol() {
      this.patrolIdx = (this.patrolIdx + 1) % this.patrol.length;
      this._goToPatrolTarget();
    }

    update(dt) {
      if (!this.alive) {
        this.vx = this.vy = this.spd = 0;
        return;
      }
      this.bat = Math.max(12, this.bat - 0.003 * dt);
      this.lidarAngle = (this.lidarAngle + dt * 8) % (Math.PI * 2);

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
        (this.inDZ ? 8 + Math.sin(S.simT * 4) * 2 : 0);

      if (this.yielding) this.waitTime += dt;
      else this.waitTime = Math.max(0, this.waitTime - dt * 2);

      this.navigate(dt);
      this.calcIntent();
    }

    navigate(dt) {
      if (!this.wp) {
        this.vx = this.vy = this.spd = 0;
        return;
      }

      const d = v2.dist(this, this.wp);
      if (d < 12) {
        this.wpIdx++;
        if (this.wpIdx < this.navPath.length) {
          this.wp = this.navPath[this.wpIdx];
        } else {
          this.advancePatrol();
          return;
        }
      }

      const dir = v2.norm(v2.sub(this.wp, this));
      const yieldMult = this.yielding ? 0.12 : 1.0;
      const dzMult = this.inDZ ? 0.55 : 1.0;
      const effSpd = this.maxSpeed * yieldMult * dzMult * BASE_SPD;

      let vx = dir.x * effSpd;
      let vy = dir.y * effSpd;

      // Soft decentralized collision deceleration
      const nm = NET_MODES[S.netMode];
      const minGap = (BOT_R * 2 + 10) * nm.safety;
      for (const [id, o] of Object.entries(S.bots)) {
        if (id === this.id || !o.alive) continue;
        const dd = v2.dist(this, o);
        if (dd < minGap * 2 && dd > 0.1) {
          const rp = v2.sub(o, this);
          const dot = rp.x * vx + rp.y * vy;
          if (dot > 0) {
            const f = Math.max(0.08, dd / (minGap * 2));
            vx *= f;
            vy *= f;
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

      if (this.spd > 0.03) {
        const tt = Math.atan2(vy, vx);
        let diff = tt - this.theta;
        while (diff < -Math.PI) diff += Math.PI * 2;
        while (diff > Math.PI) diff -= Math.PI * 2;
        this.theta += diff * 0.28;
        this.omega = diff * 3;
      }
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
  }

  /* ================================================================
     9. PRIORITY SCORING & CONFLICT RESOLUTION
     ================================================================ */
  function calcScore(b) {
    const batUrg = b.bat > 60 ? 1 : b.bat > 30 ? 2 : 3;
    const wait = Math.min(3, Math.floor(b.waitTime));
    return (b.taskPriority || 3) + (b.deadlineUrgency || 2) + wait + batUrg;
  }

  function checkConflicts() {
    const bots = Object.values(S.bots).filter((b) => b.alive);

    for (const b of bots) {
      if (b.yielding && b.yieldTo) {
        const target = S.bots[b.yieldTo];
        if (!target || !target.alive || v2.dist(b, target) > 110) {
          b.yielding = false;
          b.yieldTo = null;
        }
      }
    }

    for (let i = 0; i < bots.length; i++) {
      for (let j = i + 1; j < bots.length; j++) {
        const a = bots[i], b = bots[j];
        if (a.yielding && a.yieldTo !== b.id && a.yieldTo) continue;
        if (b.yielding && b.yieldTo !== a.id && b.yieldTo) continue;

        const d = v2.dist(a, b);
        if (d < 75 && d > 5) {
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
              if (Math.random() < 0.12) {
                loser.addLog("SVO", `Yield to ${winner.name} (${ls} vs ${ws})`);
                winner.addLog("ORCA", `Priority pass vs ${loser.name}`);
              }
              triggerDecision(winner, loser, ws, ls);
            }
          }
        }
      }
    }
  }

  function triggerDecision(winner, loser, ws, ls) {
    if (S.decision && S.decision.timer > 1.5) return;
    const bat = (b) => (b.bat > 60 ? 1 : b.bat > 30 ? 2 : 3);
    const wait = (b) => Math.min(3, Math.floor(b.waitTime));

    S.decision = {
      x: (winner.x + loser.x) / 2,
      y: (winner.y + loser.y) / 2,
      winner, loser, ws, ls,
      wD: { t: winner.taskPriority, d: winner.deadlineUrgency, w: wait(winner), b: bat(winner) },
      lD: { t: loser.taskPriority, d: loser.deadlineUrgency, w: wait(loser), b: bat(loser) },
      timer: 4.0,
    };
  }

  /* ================================================================
     10. SCENARIO PATROLS
     ================================================================ */
  const PATROLS = {
    1: {
      alpha: ["2_0", "2_3", "2_6", "2_3"],
      beta: ["0_3", "2_3", "4_3", "2_3"],
      gamma: ["2_6", "2_3", "2_0", "2_3"],
      delta: ["1_1", "1_5", "3_5", "3_1"],
      epsilon: ["3_1", "3_5", "1_5", "1_1"],
    },
    2: {
      alpha: ["2_0", "2_6", "2_0"],
      beta: ["2_6", "2_0", "2_6"],
      gamma: ["4_1", "4_5", "0_5", "0_1"],
      delta: ["1_1", "1_3", "3_3", "3_1"],
      epsilon: ["3_5", "1_5", "1_3", "3_3"],
    },
    3: {
      alpha: ["1_0", "1_6", "3_6", "3_0"],
      beta: ["2_6", "2_0", "0_0", "0_6"],
      gamma: ["0_3", "4_3", "4_5", "0_5"],
      delta: ["3_1", "1_1", "1_5", "3_5"],
      epsilon: ["4_0", "4_6", "2_6", "2_0"],
    },
    4: {
      alpha: ["1_4", "0_6", "1_6", "2_6", "2_4"],
      beta: ["2_5", "2_1", "4_1", "4_5"],
      gamma: ["3_0", "3_3", "1_3", "1_0"],
      delta: ["0_5", "2_5", "2_3", "0_3"],
      epsilon: ["4_1", "4_5", "2_5", "2_1"],
    },
    5: {
      alpha: ["2_0", "2_2", "2_3", "2_6"],
      beta: ["2_3"],
      gamma: ["3_6", "3_5", "2_5", "2_4"],
      delta: ["1_1", "1_3", "3_3", "3_1"],
      epsilon: ["0_5", "0_1", "4_1", "4_5"],
    },
  };

  const START_NODES = {
    1: ["2_0", "0_3", "2_6", "1_1", "3_5"],
    2: ["2_0", "2_6", "4_1", "1_1", "3_5"],
    3: ["1_0", "2_6", "0_3", "3_1", "4_0"],
    4: ["1_4", "2_5", "3_0", "0_5", "4_1"],
    5: ["2_0", "2_3", "3_6", "1_1", "0_5"],
  };

  /* ================================================================
     11. INIT & LOAD
     ================================================================ */
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
        if (S.bots.alpha?.alive)
          S.bots.alpha.addLog("MRTA", `Won DC-MRTA auction for ${S.bots.beta.name}`);
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
      1: "ORCA: 5 AMRs crossing open 4-way warehouse floor via local SLAM",
      2: "SVO: Beta yields into staging bay (θ=88° Altruistic) via Priority Score",
      3: "Click open floor to place cargo obstacles — A* auto-reroutes",
      4: "NE Wi-Fi blackout — AMRs switch to onboard LiDAR bubble (σ=1.5)",
      5: "Beta node failure — Alpha wins DC-MRTA bidding with staggered arrival",
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
      t._tid = setTimeout(() => t.classList.add("hidden"), 5000);
    }
  }

  /* ================================================================
     12. HIGH-FIDELITY INDUSTRIAL RENDER
     ================================================================ */
  function render(ctx) {
    // 1. Industrial Concrete / Epoxy Floor (Realistic Open Floor)
    ctx.fillStyle = C.floor;
    ctx.fillRect(0, 0, W, H);

    // Subtle concrete slab expansion joints (realistic industrial floor tiles)
    ctx.strokeStyle = C.jointLine;
    ctx.lineWidth = 1;
    for (let x = 0; x < W; x += 120) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, H);
      ctx.stroke();
    }
    for (let y = 0; y < H; y += 120) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(W, y);
      ctx.stroke();
    }

    // Micro surface grid
    ctx.strokeStyle = C.floorTile;
    ctx.lineWidth = 0.5;
    for (let x = 0; x < W; x += 30) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, H);
      ctx.stroke();
    }
    for (let y = 0; y < H; y += 30) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(W, y);
      ctx.stroke();
    }

    // 2. Charging Dock Pads on floor perimeter
    for (const pad of CHARGING_PADS) {
      ctx.fillStyle = "rgba(56,189,248,.04)";
      ctx.strokeStyle = "rgba(56,189,248,.25)";
      ctx.lineWidth = 1;
      ctx.strokeRect(pad.x - 16, pad.y - 16, 32, 32);
      ctx.fillRect(pad.x - 16, pad.y - 16, 32, 32);
      // Lightning bolt icon
      ctx.fillStyle = "rgba(56,189,248,.7)";
      ctx.font = "10px Inter";
      ctx.textAlign = "center";
      ctx.fillText("⚡", pad.x, pad.y + 4);
    }

    // 3. Staging Bays with Hazard Stripes
    for (const al of ALCOVES) {
      ctx.fillStyle = C.bayFill;
      ctx.fillRect(al.x, al.y, al.w, al.h);
      ctx.strokeStyle = C.bayBorder;
      ctx.lineWidth = 1.5;
      ctx.strokeRect(al.x, al.y, al.w, al.h);
      // Hazard striped top & bottom edge
      drawHazardStrip(ctx, al.x, al.y, al.w, 4);
      drawHazardStrip(ctx, al.x, al.y + al.h - 4, al.w, 4);
      ctx.fillStyle = "rgba(234,179,8,.55)";
      ctx.font = "8px JetBrains Mono";
      ctx.textAlign = "center";
      ctx.fillText(al.label, al.x + al.w / 2, al.y + al.h / 2 + 3);
    }

    // 4. Industrial Storage Racks (with realistic cargo inventory)
    for (const r of RACKS) {
      // Drop shadow for depth
      ctx.fillStyle = "rgba(0,0,0,.45)";
      ctx.fillRect(r.x + 3, r.y + 3, r.w, r.h);

      // Rack base frame
      ctx.fillStyle = C.rackBody;
      ctx.fillRect(r.x, r.y, r.w, r.h);
      ctx.strokeStyle = C.rackFrame;
      ctx.lineWidth = 1.5;
      ctx.strokeRect(r.x, r.y, r.w, r.h);

      // Vertical structural upright posts (heavy steel beams)
      const slotW = r.w / r.slots;
      for (let s = 0; s < r.slots; s++) {
        const sx = r.x + s * slotW;
        ctx.strokeStyle = "#2e3144";
        ctx.lineWidth = 1.5;
        ctx.strokeRect(sx, r.y, slotW, r.h);

        // Render realistic cargo pallets inside slots
        const cargoIdx = (r.seed * 3 + s * 2) % CARGO_COLORS.length;
        const cargo = CARGO_COLORS[cargoIdx];
        const padX = 4, padY = 5;
        const pw = slotW - padX * 2;
        const ph = r.h - padY * 2;

        // Wood pallet base
        ctx.fillStyle = "#78350f";
        ctx.fillRect(sx + padX, r.y + r.h - padY - 3, pw, 3);

        // Pallet container box
        ctx.fillStyle = cargo.fill;
        ctx.globalAlpha = 0.85;
        ctx.fillRect(sx + padX + 1, r.y + padY, pw - 2, ph - 4);
        ctx.strokeStyle = cargo.stroke;
        ctx.lineWidth = 1;
        ctx.strokeRect(sx + padX + 1, r.y + padY, pw - 2, ph - 4);
        ctx.globalAlpha = 1.0;

        // Cross strapping tape on box
        ctx.strokeStyle = "rgba(0,0,0,.25)";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(sx + padX + 1, r.y + padY + (ph - 4) / 2);
        ctx.lineTo(sx + padX + pw - 1, r.y + padY + (ph - 4) / 2);
        ctx.stroke();
      }

      // Bay Header Label Plate
      ctx.fillStyle = "rgba(10,11,16,.9)";
      ctx.fillRect(r.x + r.w / 2 - 32, r.y - 6, 64, 11);
      ctx.strokeStyle = "#383c50";
      ctx.lineWidth = 1;
      ctx.strokeRect(r.x + r.w / 2 - 32, r.y - 6, 64, 11);
      ctx.fillStyle = C.rackLabel;
      ctx.font = "bold 7px JetBrains Mono";
      ctx.textAlign = "center";
      ctx.fillText(r.label, r.x + r.w / 2, r.y + 2);
    }

    // 5. Stations (Conveyor rollers & Docks)
    for (const st of STATIONS) {
      ctx.fillStyle = C.dockFill;
      ctx.fillRect(st.x, st.y, st.w, st.h);
      ctx.strokeStyle = C.dockBorder;
      ctx.lineWidth = 1.5;
      ctx.strokeRect(st.x, st.y, st.w, st.h);

      if (st.type === "conveyor") {
        // Draw conveyor rollers
        const nr = Math.floor(st.h / 8);
        ctx.strokeStyle = "rgba(255,255,255,.15)";
        ctx.lineWidth = 1.5;
        for (let i = 0; i < nr; i++) {
          const ry = st.y + 4 + i * 8;
          ctx.beginPath();
          ctx.moveTo(st.x + 3, ry);
          ctx.lineTo(st.x + st.w - 3, ry);
          ctx.stroke();
        }
      } else {
        // Dock hazard strip
        drawHazardStrip(ctx, st.x, st.y, st.w, 4);
      }

      // Station label
      ctx.fillStyle = "rgba(34,197,94,.8)";
      ctx.font = "bold 7px Inter";
      ctx.textAlign = "center";
      ctx.fillText(st.label, st.x + st.w / 2, st.y + st.h / 2 + 3);
    }

    // 6. Wi-Fi Dead Zone (Scenario 4)
    if (S.scenario === 4) {
      const dz = DEAD_ZONE;
      ctx.fillStyle = C.deadZone;
      ctx.fillRect(dz.x, dz.y, dz.w, dz.h);
      ctx.strokeStyle = C.deadZoneStroke;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 4]);
      ctx.strokeRect(dz.x, dz.y, dz.w, dz.h);
      ctx.setLineDash([]);
      ctx.fillStyle = "rgba(244,63,94,.55)";
      ctx.font = "10px Inter";
      ctx.textAlign = "center";
      ctx.fillText("⚠ WI-FI BLACKOUT ZONE", dz.x + dz.w / 2, dz.y + 16);
    }

    // 7. Dynamic Obstacles (Industrial Cargo Crates)
    for (const obs of S.obstacles) {
      ctx.fillStyle = "rgba(244,63,94,.12)";
      ctx.beginPath();
      ctx.arc(obs.x, obs.y, obs.r + 8, 0, Math.PI * 2);
      ctx.fill();

      // Industrial wooden/metal crate box
      const bw = obs.r * 1.5;
      ctx.fillStyle = "#b91c1c";
      ctx.fillRect(obs.x - bw / 2, obs.y - bw / 2, bw, bw);
      ctx.strokeStyle = "#fca5a5";
      ctx.lineWidth = 1.5;
      ctx.strokeRect(obs.x - bw / 2, obs.y - bw / 2, bw, bw);
      // Warning diagonal slash
      ctx.strokeStyle = "rgba(255,255,255,.6)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(obs.x - bw / 2 + 2, obs.y - bw / 2 + 2);
      ctx.lineTo(obs.x + bw / 2 - 2, obs.y + bw / 2 - 2);
      ctx.moveTo(obs.x + bw / 2 - 2, obs.y - bw / 2 + 2);
      ctx.lineTo(obs.x - bw / 2 + 2, obs.y + bw / 2 - 2);
      ctx.stroke();
    }

    // 8. P2P Mesh Communication Links
    const nm = NET_MODES[S.netMode];
    const bl = Object.values(S.bots);
    if (nm.commR > 0) {
      for (let i = 0; i < bl.length; i++) {
        for (let j = i + 1; j < bl.length; j++) {
          const a = bl[i], b = bl[j];
          if (!a.alive || !b.alive) continue;
          const d = v2.dist(a, b);
          if (d <= nm.commR) {
            ctx.save();
            const dead = a.inDZ || b.inDZ;
            ctx.strokeStyle = dead
              ? "rgba(244,63,94,.4)"
              : `rgba(56,189,248,${0.15 + 0.25 * (1 - d / nm.commR)})`;
            ctx.lineWidth = 1.2;
            ctx.setLineDash([4, 4]);
            ctx.lineDashOffset = -S.simT * 22;
            ctx.beginPath();
            ctx.moveTo(a.x, a.y);
            ctx.lineTo(b.x, b.y);
            ctx.stroke();
            ctx.restore();
          }
        }
      }
    }

    // 9. Intent Trajectories (Translucent spline ahead)
    for (const b of bl) {
      if (!b.alive || b.intent.length < 2) continue;
      ctx.save();
      ctx.strokeStyle = b.color;
      ctx.lineWidth = 1.8;
      ctx.globalAlpha = 0.22;
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.moveTo(b.x, b.y);
      for (const p of b.intent) ctx.lineTo(p.x, p.y);
      ctx.stroke();
      ctx.restore();
    }

    // 10. REALISTIC AUTONOMOUS MOBILE ROBOTS (AMRs)
    for (const b of bl) {
      ctx.save();
      ctx.translate(b.x, b.y);

      // A. Safety Envelope / LiDAR Ring
      if (b.inDZ || S.netMode === 2) {
        ctx.beginPath();
        ctx.arc(0, 0, b.safeR + 6, 0, Math.PI * 2);
        ctx.strokeStyle = "rgba(244,63,94,.4)";
        ctx.lineWidth = 1.5;
        ctx.setLineDash([3, 3]);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // B. Yielding Status Ring
      if (b.yielding) {
        ctx.beginPath();
        ctx.arc(0, 0, BOT_R + 8, 0, Math.PI * 2);
        ctx.strokeStyle = `rgba(234,179,8,${0.35 + Math.sin(S.simT * 7) * 0.25})`;
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // C. Selection Halo
      if (S.selBot === b.id) {
        ctx.beginPath();
        ctx.arc(0, 0, BOT_R + 6, 0, Math.PI * 2);
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }

      // D. Rotate to AMR heading
      ctx.rotate(b.theta);

      // Forward Headlight Beams (casting light on floor)
      if (b.alive) {
        const hGrad = ctx.createRadialGradient(16, 0, 2, 28, 0, 24);
        hGrad.addColorStop(0, "rgba(254,240,138,.35)");
        hGrad.addColorStop(1, "rgba(254,240,138,0)");
        ctx.fillStyle = hGrad;
        ctx.beginPath();
        ctx.moveTo(14, -6);
        ctx.lineTo(38, -14);
        ctx.lineTo(38, 14);
        ctx.lineTo(14, 6);
        ctx.closePath();
        ctx.fill();
      }

      // Differential Drive Wheels (Side Tires)
      ctx.fillStyle = "#1e1e24";
      ctx.strokeStyle = "#40404c";
      ctx.lineWidth = 1;
      // Left wheel
      roundRect(ctx, -6, -BOT_H / 2 - 2, 12, 4, 1.5);
      ctx.fill();
      ctx.stroke();
      // Right wheel
      roundRect(ctx, -6, BOT_H / 2 - 2, 12, 4, 1.5);
      ctx.fill();
      ctx.stroke();

      // Front & Rear Caster Wheels
      ctx.fillStyle = "#2a2a34";
      ctx.beginPath();
      ctx.arc(10, 0, 2.5, 0, Math.PI * 2);
      ctx.arc(-10, 0, 2.5, 0, Math.PI * 2);
      ctx.fill();

      // AMR Main Industrial Chassis (Rounded Rectangular Body)
      ctx.fillStyle = b.alive ? "#1b1c24" : "#2d2d38";
      roundRect(ctx, -BOT_W / 2, -BOT_H / 2, BOT_W, BOT_H, 6);
      ctx.fill();

      // Chassis Colored Accent Bumper
      ctx.strokeStyle = b.alive ? b.color : "#52525b";
      ctx.lineWidth = 2;
      roundRect(ctx, -BOT_W / 2, -BOT_H / 2, BOT_W, BOT_H, 6);
      ctx.stroke();

      // Directional Headlights (LED dots)
      if (b.alive) {
        ctx.fillStyle = "#fef08a";
        ctx.beginPath();
        ctx.arc(BOT_W / 2 - 2, -5, 1.5, 0, Math.PI * 2);
        ctx.arc(BOT_W / 2 - 2, 5, 1.5, 0, Math.PI * 2);
        ctx.fill();
        // Red Taillights
        ctx.fillStyle = "#ef4444";
        ctx.beginPath();
        ctx.arc(-BOT_W / 2 + 2, -5, 1.2, 0, Math.PI * 2);
        ctx.arc(-BOT_W / 2 + 2, 5, 1.2, 0, Math.PI * 2);
        ctx.fill();
      }

      // Top Cargo Bed / Payload Carrier
      ctx.fillStyle = b.alive ? b.color : "#3f3f46";
      ctx.globalAlpha = 0.85;
      roundRect(ctx, -8, -6, 12, 12, 2);
      ctx.fill();
      ctx.globalAlpha = 1.0;

      // Center Rotating LiDAR Scanner Turret
      ctx.fillStyle = "#09090d";
      ctx.beginPath();
      ctx.arc(0, 0, 4.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "rgba(255,255,255,.6)";
      ctx.lineWidth = 1;
      ctx.stroke();

      // LiDAR 360° Laser Scan Beam (Rotating SLAM sweep)
      if (b.alive) {
        ctx.save();
        ctx.rotate(b.lidarAngle - b.theta);
        ctx.strokeStyle = `rgba(56,189,248,${0.3 + Math.sin(S.simT * 5) * 0.15})`;
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(26, 0);
        ctx.stroke();
        // Laser dot at tip
        ctx.fillStyle = "#38bdf8";
        ctx.beginPath();
        ctx.arc(26, 0, 1.5, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
      }

      // Reset rotation for text
      ctx.rotate(-b.theta);

      // AMR Letter Badge
      ctx.fillStyle = "#ffffff";
      ctx.font = "bold 9px Inter";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(b.letter, 0, 0);

      // AMR Name Tag
      ctx.fillStyle = "rgba(255,255,255,.85)";
      ctx.font = "8px JetBrains Mono";
      ctx.textBaseline = "top";
      ctx.fillText(b.name.split(" ")[1], 0, BOT_H / 2 + 4);

      ctx.restore();
    }

    // 11. Explainable Decision Popover
    renderDecision(ctx);
  }

  /* Helper to draw industrial hazard stripes */
  function drawHazardStrip(ctx, x, y, w, h) {
    ctx.save();
    ctx.beginPath();
    ctx.rect(x, y, w, h);
    ctx.clip();
    ctx.fillStyle = "#eab308";
    ctx.fillRect(x, y, w, h);
    ctx.fillStyle = "#18181b";
    for (let i = -h; i < w + h; i += 8) {
      ctx.beginPath();
      ctx.moveTo(x + i, y);
      ctx.lineTo(x + i + 4, y);
      ctx.lineTo(x + i - 4, y + h);
      ctx.lineTo(x + i - 8, y + h);
      ctx.closePath();
      ctx.fill();
    }
    ctx.restore();
  }

  /* ================================================================
     13. DECISION POPOVER
     ================================================================ */
  function renderDecision(ctx) {
    const d = S.decision;
    if (!d || d.timer <= 0) return;

    const pw = 235, ph = 120;
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
     14. INSPECTOR UI
     ================================================================ */
  function updateInsp() {
    const b = S.bots[S.selBot];
    if (!b) return;
    const el = (id) => document.getElementById(id);

    el("iName").textContent = b.name;
    el("iHw").textContent = `${b.hw} — ${b.role}`;

    const st = el("iStatus");
    st.textContent = b.alive
      ? b.inDZ
        ? "BLACKOUT"
        : b.yielding
          ? "YIELDING"
          : "ONLINE"
      : "OFFLINE";
    st.className = "status-pill" + (b.alive ? (b.yielding ? " yield" : "") : " off");

    el("iPose").textContent = `${Math.round(b.x)}, ${Math.round(b.y)}`;
    el("iVel").textContent = `${(b.spd / BASE_SPD).toFixed(2)} m/s`;
    el("iBat").textContent = `${Math.round(b.bat)}%`;
    el("iSvo").textContent = `θ:${b.svo}° (${b.svoLabel})`;
    el("iCap").textContent = `${b.maxPayload}kg / ${b.maxSpeed} m/s`;
    el("iPayload").textContent = b.curPayload;

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
    };
    let lh = "";
    for (const e of b.log.slice(0, 6)) {
      const tc = tagCls[e.tag] || "t-init";
      lh += `<div class="log-row">[${e.ts}] <b class="${tc}">${e.tag}</b> ${e.msg}</div>`;
    }
    lg.innerHTML = lh;

    el("killText").textContent = b.alive ? `Kill ${b.name}` : `Revive ${b.name}`;
    el("killBtn").className = "kill-btn" + (b.alive ? "" : " revive");

    let alive = 0;
    for (const r of Object.values(S.bots)) if (r.alive) alive++;
    el("peerCount").textContent = alive;
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
  }

  /* ================================================================
     15. EVENTS
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
      b.alive = !b.alive;
      if (!b.alive) {
        b.addLog("FAULT", "Node killed");
        let best = null, bestD = Infinity;
        for (const [id, o] of Object.entries(S.bots)) {
          if (id === b.id || !o.alive) continue;
          const d = v2.dist(b, o);
          if (d < bestD) {
            bestD = d;
            best = o;
          }
        }
        if (best) {
          best.addLog("MRTA", `Won auction for ${b.name}`);
          best.curPayload += ` + [${b.name}]`;
        }
        showToast(`${b.name} killed — nearest peer re-auctions`);
      } else {
        const prof = PROFILES.find((p) => p.id === b.id);
        b.bat = prof.bat;
        b.curPayload = `${prof.role} (${prof.maxPayload}kg)`;
        b.recalcPath();
        showToast(`${b.name} revived`);
      }
      updateInsp();
    });

    S.canvas.addEventListener("click", (e) => {
      const rect = S.canvas.getBoundingClientRect();
      const cx = ((e.clientX - rect.left) / rect.width) * W;
      const cy = ((e.clientY - rect.top) / rect.height) * H;

      for (const [id, b] of Object.entries(S.bots)) {
        if (v2.dist({ x: cx, y: cy }, b) < BOT_R + 10) {
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
            showToast(`Obstacle placed on floor at (${nn.x}, ${nn.y}) — AMR fleet auto-rerouted`);
          }
        }
      }
    });
  }

  /* ================================================================
     16. BUILD TABS
     ================================================================ */
  function buildTabs() {
    const container = document.getElementById("robotTabs");
    let html = "";
    PROFILES.forEach((cfg, i) => {
      const active = i === 0 ? " active" : "";
      html += `<button class="tab${active}" data-robot="${cfg.id}"><span class="tab-dot" style="background:${cfg.color}"></span>${cfg.name.split(" ")[1]}</button>`;
    });
    container.innerHTML = html;
  }

  /* ================================================================
     17. MAIN LOOP
     ================================================================ */
  function loop(ts) {
    if (!S.lt) S.lt = ts;
    const dt = Math.min((ts - S.lt) / 1000, 0.1) * S.speed;
    S.lt = ts;

    if (S.running) {
      S.simT += dt;
      for (const b of Object.values(S.bots)) b.update(dt);
      checkConflicts();
      if (S.decision) {
        S.decision.timer -= dt;
        if (S.decision.timer <= 0) S.decision = null;
      }
    }

    if (S.ctx) render(S.ctx);
    updateInsp();
    updateKPI();

    requestAnimationFrame(loop);
  }

  /* ================================================================
     18. INIT
     ================================================================ */
  function init() {
    S.canvas = document.getElementById("simCanvas");
    if (!S.canvas) return;
    S.ctx = S.canvas.getContext("2d");
    buildGraph();
    buildTabs();
    initBots();
    setupEvents();
    loadScenario(1);
    requestAnimationFrame(loop);
  }

  if (document.readyState === "loading")
    document.addEventListener("DOMContentLoaded", init);
  else init();
})();