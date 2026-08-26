/**
 * ============================================================================
 * SYNERGY-AMR v3 — Decentralized Fleet Coordination Engine
 * ============================================================================
 * Features:
 *   • A* pathfinding on a 35-node navigation graph (no more stuck robots)
 *   • 5 heterogeneous AMRs with unique profiles
 *   • Priority-based conflict resolution with explainable decision popover
 *   • 4-mode network degradation (Connected → Degraded → Offline → Recovery)
 *   • 5 demo scenarios
 * ============================================================================
 */
(function () {
  "use strict";

  /* ================================================================
     1. CONSTANTS & COLORS
     ================================================================ */
  const W = 960, H = 580;
  const BOT_R = 13;
  const BASE_SPD = 1.6;

  const C = {
    bg: "#08080c",
    grid: "rgba(255,255,255,.018)",
    rack: "#16161e",
    rackStroke: "#2a2a36",
    rackLabel: "#4a4a5a",
    aisle: "rgba(56,189,248,.03)",
    laneStripe: "rgba(234,179,8,.12)",
    alcove: "rgba(234,179,8,.06)",
    alcoveStroke: "rgba(234,179,8,.2)",
    station: "rgba(34,197,94,.06)",
    stationStroke: "rgba(34,197,94,.25)",
    deadZone: "rgba(244,63,94,.06)",
    deadZoneStroke: "rgba(244,63,94,.25)",
    obstacle: "#ef4444",
    alpha: "#38bdf8",
    beta: "#eab308",
    gamma: "#22c55e",
    delta: "#a78bfa",
    epsilon: "#f472b6",
  };

  /* ================================================================
     2. WAREHOUSE MAP
     ================================================================
     Racks are arranged in 4 rows × 4 columns with 50px+ gaps between
     pairs for robot navigation channels.

     Vertical nav channels:  x = 28, 210, 380, 480, 580, 750, 932
     Horizontal nav channels: y = 28, 148, 290, 430, 550
  */
  const RACKS = [
    // Row 1 (y: 50–105)
    { x: 55, y: 50, w: 130, h: 55, label: "A1–A4" },
    { x: 235, y: 50, w: 120, h: 55, label: "A5–A8" },
    { x: 605, y: 50, w: 120, h: 55, label: "B1–B4" },
    { x: 775, y: 50, w: 130, h: 55, label: "B5–B8" },
    // Row 2 (y: 185–240)
    { x: 55, y: 185, w: 130, h: 55, label: "C1–C4" },
    { x: 235, y: 185, w: 120, h: 55, label: "C5–C8" },
    { x: 605, y: 185, w: 120, h: 55, label: "D1–D4" },
    { x: 775, y: 185, w: 130, h: 55, label: "D5–D8" },
    // Row 3 (y: 340–395)
    { x: 55, y: 340, w: 130, h: 55, label: "E1–E4" },
    { x: 235, y: 340, w: 120, h: 55, label: "E5–E8" },
    { x: 605, y: 340, w: 120, h: 55, label: "F1–F4" },
    { x: 775, y: 340, w: 130, h: 55, label: "F5–F8" },
    // Row 4 (y: 465–520)
    { x: 55, y: 465, w: 130, h: 55, label: "G1–G4" },
    { x: 235, y: 465, w: 120, h: 55, label: "G5–G8" },
    { x: 605, y: 465, w: 120, h: 55, label: "H1–H4" },
    { x: 775, y: 465, w: 130, h: 55, label: "H5–H8" },
  ];

  const STATIONS = [
    { x: 6, y: 268, w: 28, h: 44, label: "PICK" },
    { x: 926, y: 268, w: 28, h: 44, label: "PACK" },
    { x: 440, y: 4, w: 80, h: 24, label: "INBOUND" },
    { x: 440, y: 554, w: 80, h: 22, label: "OUTBOUND" },
  ];

  const ALCOVES = [
    { x: 415, y: 115, w: 55, h: 35, label: "Bay 1" },
    { x: 490, y: 115, w: 55, h: 35, label: "Bay 2" },
    { x: 415, y: 425, w: 55, h: 35, label: "Bay 3" },
  ];

  // Visual aisle regions (just for rendering)
  const H_AISLES = [
    { x: 20, y: 110, w: 920, h: 70 },
    { x: 20, y: 245, w: 920, h: 90 },
    { x: 20, y: 400, w: 920, h: 60 },
  ];
  const V_AISLE = { x: 388, y: 35, w: 184, h: 510 };
  const DEAD_ZONE = { x: 590, y: 35, w: 330, h: 195 };

  /* ================================================================
     3. NAVIGATION GRAPH — 7 cols × 5 rows = 35 nodes
     ================================================================
     Every node connects to its orthogonal neighbors. Robots use A*
     to route between nodes and only travel along corridor edges.
  */
  const NAV_X = [28, 210, 380, 480, 580, 750, 932];
  const NAV_Y = [28, 148, 290, 430, 550];
  const NODES = [];
  const ADJ = {};
  const blockedNodes = new Set();

  function nid(r, c) {
    return `${r}_${c}`;
  }

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

  function getNode(id) {
    return NODES.find((n) => n.id === id);
  }

  function nearestNode(x, y, skipBlocked) {
    let best = null,
      bestD = Infinity;
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
      let cur = null,
        cf = Infinity;
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
        const cn = getNode(cur),
          nn = getNode(nb);
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
    const a = getNode(aId),
      b = getNode(bId);
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
    { name: "Connected", commR: 200, lat: 12, safety: 1.0, color: "#22c55e" },
    { name: "Degraded", commR: 100, lat: 250, safety: 1.3, color: "#eab308" },
    { name: "Offline", commR: 0, lat: 999, safety: 1.5, color: "#f43f5e" },
    { name: "Recovery", commR: 160, lat: 80, safety: 1.1, color: "#38bdf8" },
  ];

  /* ================================================================
     6. ROBOT PROFILES — Heterogeneous Fleet
     ================================================================ */
  const PROFILES = [
    {
      id: "alpha", name: "Robot Alpha", letter: "α", color: C.alpha,
      hw: "Jetson Orin Nano", role: "Heavy Lifter",
      maxPayload: 500, maxSpeed: 1.2, bat: 92,
      svo: 25, svoLabel: "Individualistic",
      taskPriority: 5, deadlineUrgency: 3,
    },
    {
      id: "beta", name: "Robot Beta", letter: "β", color: C.beta,
      hw: "Raspberry Pi 5", role: "Agile Runner",
      maxPayload: 100, maxSpeed: 2.0, bat: 78,
      svo: 88, svoLabel: "Altruistic",
      taskPriority: 2, deadlineUrgency: 2,
    },
    {
      id: "gamma", name: "Robot Gamma", letter: "γ", color: C.gamma,
      hw: "Jetson Orin NX", role: "Standard Hauler",
      maxPayload: 250, maxSpeed: 1.5, bat: 95,
      svo: 50, svoLabel: "Prosocial",
      taskPriority: 3, deadlineUrgency: 4,
    },
    {
      id: "delta", name: "Robot Delta", letter: "δ", color: C.delta,
      hw: "Jetson AGX Orin", role: "Precision Handler",
      maxPayload: 150, maxSpeed: 1.8, bat: 84,
      svo: 40, svoLabel: "Prosocial",
      taskPriority: 3, deadlineUrgency: 2,
    },
    {
      id: "epsilon", name: "Robot Epsilon", letter: "ε", color: C.epsilon,
      hw: "Raspberry Pi 5", role: "Scout",
      maxPayload: 50, maxSpeed: 2.5, bat: 91,
      svo: 60, svoLabel: "Prosocial",
      taskPriority: 2, deadlineUrgency: 3,
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
     8. ROBOT CLASS
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
      this.addLog("INIT", `${p.hw} | ${p.role}`);
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
        // Can't reach target — try next patrol point
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
      this.bat = Math.max(12, this.bat - 0.004 * dt);

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
          this.addLog("RF", "Dead zone — LiDAR bubble σ=1.5");
        } else if (!inside && this.inDZ) {
          this.inDZ = false;
          this.addLog("ZENOH", "P2P mesh restored");
        }
      } else {
        this.inDZ = false;
      }

      // Safety radius based on network mode
      const nm = NET_MODES[S.netMode];
      this.safeR =
        BOT_R * nm.safety +
        (this.inDZ ? 8 + Math.sin(S.simT * 4) * 2 : 0);

      // Yielding decay
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

      // Advance waypoint if close enough
      const d = v2.dist(this, this.wp);
      if (d < 10) {
        this.wpIdx++;
        if (this.wpIdx < this.navPath.length) {
          this.wp = this.navPath[this.wpIdx];
        } else {
          this.advancePatrol();
          return;
        }
      }

      // Direction to current waypoint
      const dir = v2.norm(v2.sub(this.wp, this));
      const yieldMult = this.yielding ? 0.12 : 1.0;
      const dzMult = this.inDZ ? 0.5 : 1.0;
      const effSpd = this.maxSpeed * yieldMult * dzMult * BASE_SPD;

      let vx = dir.x * effSpd;
      let vy = dir.y * effSpd;

      // Proximity slowdown — slow when approaching another robot
      const nm = NET_MODES[S.netMode];
      const minGap = (BOT_R * 2 + 8) * nm.safety;
      for (const [id, o] of Object.entries(S.bots)) {
        if (id === this.id || !o.alive) continue;
        const dd = v2.dist(this, o);
        if (dd < minGap * 2 && dd > 0.1) {
          const rp = v2.sub(o, this);
          const dot = rp.x * vx + rp.y * vy;
          if (dot > 0) {
            // Converging — scale down proportionally
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

      // Smooth heading
      if (this.spd > 0.03) {
        const tt = Math.atan2(vy, vx);
        let diff = tt - this.theta;
        while (diff < -Math.PI) diff += Math.PI * 2;
        while (diff > Math.PI) diff -= Math.PI * 2;
        this.theta += diff * 0.25;
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

    // Clear yields for distant pairs
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
        const a = bots[i],
          b = bots[j];
        // Skip if either is already yielding to someone else
        if (a.yielding && a.yieldTo !== b.id && a.yieldTo) continue;
        if (b.yielding && b.yieldTo !== a.id && b.yieldTo) continue;

        const d = v2.dist(a, b);
        if (d < 75 && d > 5) {
          // Check if converging
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
                loser.addLog("SVO", `Yield to ${winner.name} (${ls}→${ws})`);
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
    // Don't spam — only show if no decision active or expiring
    if (S.decision && S.decision.timer > 1.5) return;

    const bat = (b) => (b.bat > 60 ? 1 : b.bat > 30 ? 2 : 3);
    const wait = (b) => Math.min(3, Math.floor(b.waitTime));

    S.decision = {
      x: (winner.x + loser.x) / 2,
      y: (winner.y + loser.y) / 2,
      winner,
      loser,
      ws,
      ls,
      wD: {
        t: winner.taskPriority,
        d: winner.deadlineUrgency,
        w: wait(winner),
        b: bat(winner),
      },
      lD: {
        t: loser.taskPriority,
        d: loser.deadlineUrgency,
        w: wait(loser),
        b: bat(loser),
      },
      timer: 4.0,
    };
  }

  /* ================================================================
     10. SCENARIO PATROLS
     ================================================================
     Each scenario defines patrol routes as sequences of nav-graph
     node IDs.  Robots A* between consecutive patrol waypoints.

     Node ID format: "row_col"  (0-indexed)
     Grid:   col→  0     1     2     3     4     5     6
     row 0:       28    210   380   480   580   750   932  (y=28)
     row 1:        "     "     "     "     "     "     "   (y=148)
     row 2:        "     "     "     "     "     "     "   (y=290)
     row 3:        "     "     "     "     "     "     "   (y=430)
     row 4:        "     "     "     "     "     "     "   (y=550)
  */
  const PATROLS = {
    // 1. ORCA — all 5 cross through center node (2_3)
    1: {
      alpha: ["2_0", "2_3", "2_6", "2_3"],
      beta: ["0_3", "2_3", "4_3", "2_3"],
      gamma: ["2_6", "2_3", "2_0", "2_3"],
      delta: ["1_1", "1_5", "3_5", "3_1"],
      epsilon: ["3_1", "3_5", "1_5", "1_1"],
    },
    // 2. SVO Yield — Alpha & Beta head-on in main aisle
    2: {
      alpha: ["2_0", "2_6", "2_0"],
      beta: ["2_6", "2_0", "2_6"],
      gamma: ["4_1", "4_5", "0_5", "0_1"],
      delta: ["1_1", "1_3", "3_3", "3_1"],
      epsilon: ["3_5", "1_5", "1_3", "3_3"],
    },
    // 3. Dynamic Obstacles — wide patrols
    3: {
      alpha: ["1_0", "1_6", "3_6", "3_0"],
      beta: ["2_6", "2_0", "0_0", "0_6"],
      gamma: ["0_3", "4_3", "4_5", "0_5"],
      delta: ["3_1", "1_1", "1_5", "3_5"],
      epsilon: ["4_0", "4_6", "2_6", "2_0"],
    },
    // 4. Dead Zone (NE quadrant)
    4: {
      alpha: ["1_4", "0_6", "1_6", "2_6", "2_4"],
      beta: ["2_5", "2_1", "4_1", "4_5"],
      gamma: ["3_0", "3_3", "1_3", "1_0"],
      delta: ["0_5", "2_5", "2_3", "0_3"],
      epsilon: ["4_1", "4_5", "2_5", "2_1"],
    },
    // 5. Node Failure — Beta dead at center (2_3)
    5: {
      alpha: ["2_0", "2_2", "2_3", "2_6"],
      beta: ["2_3"],
      gamma: ["3_6", "3_5", "2_5", "2_4"], // arrives AFTER alpha
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

    // Scenario 5: kill Beta
    if (n === 5) {
      S.bots.beta.alive = false;
      S.bots.beta.addLog("FAULT", "Heartbeat timeout > 100ms");
      setTimeout(() => {
        if (S.bots.alpha?.alive)
          S.bots.alpha.addLog(
            "MRTA",
            `Won DC-MRTA auction for ${S.bots.beta.name}`
          );
      }, 800);
      setTimeout(() => {
        if (S.bots.gamma?.alive)
          S.bots.gamma.addLog(
            "MRTA",
            `Standby — Alpha has priority bid for Beta`
          );
      }, 1600);
    }

    // Scenario 3: pre-spawn obstacles
    if (n === 3) {
      spawnObstacle("2_3");
      spawnObstacle("1_3");
    }

    const msgs = {
      1: "ORCA: 5 AMRs crossing 4-way intersection — 50Hz local ORCA",
      2: "SVO: Beta yields (θ=88° Altruistic) — priority scoring active",
      3: "Click aisles to spawn/remove obstacles — A* auto-reroute",
      4: "NE Wi-Fi dead zone — LiDAR safety bubble inflated (σ=1.5)",
      5: "Beta killed — Alpha wins DC-MRTA auction (staggered arrival)",
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
     12. CANVAS RENDER
     ================================================================ */
  function render(ctx) {
    ctx.fillStyle = C.bg;
    ctx.fillRect(0, 0, W, H);

    // Grid
    ctx.strokeStyle = C.grid;
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

    // Horizontal aisles
    for (const a of H_AISLES) {
      ctx.fillStyle = C.aisle;
      ctx.fillRect(a.x, a.y, a.w, a.h);
      ctx.strokeStyle = C.laneStripe;
      ctx.lineWidth = 1;
      ctx.setLineDash([6, 6]);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y + a.h / 2);
      ctx.lineTo(a.x + a.w, a.y + a.h / 2);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Vertical cross-aisle
    ctx.fillStyle = C.aisle;
    ctx.fillRect(V_AISLE.x, V_AISLE.y, V_AISLE.w, V_AISLE.h);
    ctx.strokeStyle = C.laneStripe;
    ctx.lineWidth = 1;
    ctx.setLineDash([6, 6]);
    ctx.beginPath();
    ctx.moveTo(V_AISLE.x + V_AISLE.w / 2, V_AISLE.y);
    ctx.lineTo(V_AISLE.x + V_AISLE.w / 2, V_AISLE.y + V_AISLE.h);
    ctx.stroke();
    ctx.setLineDash([]);

    // Alcoves
    for (const al of ALCOVES) {
      ctx.fillStyle = C.alcove;
      ctx.fillRect(al.x, al.y, al.w, al.h);
      ctx.strokeStyle = C.alcoveStroke;
      ctx.lineWidth = 1;
      ctx.strokeRect(al.x, al.y, al.w, al.h);
      ctx.fillStyle = "rgba(234,179,8,.4)";
      ctx.font = "8px Inter";
      ctx.textAlign = "center";
      ctx.fillText(al.label, al.x + al.w / 2, al.y + al.h / 2 + 3);
    }

    // Racks
    for (const r of RACKS) {
      ctx.fillStyle = "rgba(0,0,0,.2)";
      ctx.fillRect(r.x + 2, r.y + 2, r.w, r.h);
      ctx.fillStyle = C.rack;
      ctx.fillRect(r.x, r.y, r.w, r.h);
      ctx.strokeStyle = C.rackStroke;
      ctx.lineWidth = 1;
      ctx.strokeRect(r.x, r.y, r.w, r.h);
      const ns = 4;
      ctx.strokeStyle = "rgba(255,255,255,.025)";
      for (let i = 1; i < ns; i++) {
        const sx = r.x + (r.w / ns) * i;
        ctx.beginPath();
        ctx.moveTo(sx, r.y);
        ctx.lineTo(sx, r.y + r.h);
        ctx.stroke();
      }
      ctx.fillStyle = C.rackLabel;
      ctx.font = "9px JetBrains Mono";
      ctx.textAlign = "center";
      ctx.fillText(r.label, r.x + r.w / 2, r.y + r.h / 2 + 3);
    }

    // Stations
    for (const st of STATIONS) {
      ctx.fillStyle = C.station;
      ctx.fillRect(st.x, st.y, st.w, st.h);
      ctx.strokeStyle = C.stationStroke;
      ctx.lineWidth = 1;
      ctx.strokeRect(st.x, st.y, st.w, st.h);
      ctx.fillStyle = "rgba(34,197,94,.6)";
      ctx.font = "bold 7px Inter";
      ctx.textAlign = "center";
      ctx.fillText(st.label, st.x + st.w / 2, st.y + st.h / 2 + 3);
    }

    // Dead zone (scenario 4)
    if (S.scenario === 4) {
      const dz = DEAD_ZONE;
      ctx.fillStyle = C.deadZone;
      ctx.fillRect(dz.x, dz.y, dz.w, dz.h);
      ctx.strokeStyle = C.deadZoneStroke;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 4]);
      ctx.strokeRect(dz.x, dz.y, dz.w, dz.h);
      ctx.setLineDash([]);
      ctx.fillStyle = "rgba(244,63,94,.5)";
      ctx.font = "10px Inter";
      ctx.textAlign = "center";
      ctx.fillText("⚠ WI-FI DEAD ZONE", dz.x + dz.w / 2, dz.y + 16);
    }

    // Dynamic obstacles
    for (const obs of S.obstacles) {
      ctx.fillStyle = "rgba(244,63,94,.1)";
      ctx.beginPath();
      ctx.arc(obs.x, obs.y, obs.r + 10, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = C.obstacle;
      const h2 = obs.r * 0.7;
      ctx.fillRect(obs.x - h2, obs.y - h2, h2 * 2, h2 * 2);
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 1.5;
      ctx.strokeRect(obs.x - h2, obs.y - h2, h2 * 2, h2 * 2);
      ctx.strokeStyle = "rgba(255,255,255,.5)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(obs.x - 4, obs.y - 4);
      ctx.lineTo(obs.x + 4, obs.y + 4);
      ctx.moveTo(obs.x + 4, obs.y - 4);
      ctx.lineTo(obs.x - 4, obs.y + 4);
      ctx.stroke();
    }

    // Navigation graph nodes (subtle dots)
    ctx.fillStyle = "rgba(255,255,255,.04)";
    for (const n of NODES) {
      if (blockedNodes.has(n.id)) continue;
      ctx.beginPath();
      ctx.arc(n.x, n.y, 2, 0, Math.PI * 2);
      ctx.fill();
    }

    // P2P links
    const nm = NET_MODES[S.netMode];
    const bl = Object.values(S.bots);
    if (nm.commR > 0) {
      for (let i = 0; i < bl.length; i++) {
        for (let j = i + 1; j < bl.length; j++) {
          const a = bl[i],
            b = bl[j];
          if (!a.alive || !b.alive) continue;
          const d = v2.dist(a, b);
          if (d <= nm.commR) {
            ctx.save();
            const dead = a.inDZ || b.inDZ;
            ctx.strokeStyle = dead
              ? "rgba(244,63,94,.35)"
              : `rgba(56,189,248,${0.12 + 0.2 * (1 - d / nm.commR)})`;
            ctx.lineWidth = 1;
            ctx.setLineDash([3, 3]);
            ctx.lineDashOffset = -S.simT * 18;
            ctx.beginPath();
            ctx.moveTo(a.x, a.y);
            ctx.lineTo(b.x, b.y);
            ctx.stroke();
            ctx.restore();
          }
        }
      }
    }

    // Intent trajectories
    for (const b of bl) {
      if (!b.alive || b.intent.length < 2) continue;
      ctx.save();
      ctx.strokeStyle = b.color;
      ctx.lineWidth = 1.5;
      ctx.globalAlpha = 0.18;
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.moveTo(b.x, b.y);
      for (const p of b.intent) ctx.lineTo(p.x, p.y);
      ctx.stroke();
      ctx.restore();
    }

    // Robots
    for (const b of bl) {
      ctx.save();
      ctx.translate(b.x, b.y);

      // LiDAR safety ring
      if (b.inDZ || S.netMode === 2) {
        ctx.beginPath();
        ctx.arc(0, 0, b.safeR + 5, 0, Math.PI * 2);
        ctx.strokeStyle = "rgba(244,63,94,.35)";
        ctx.lineWidth = 1.5;
        ctx.setLineDash([3, 3]);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // Yielding pulsing ring
      if (b.yielding) {
        ctx.beginPath();
        ctx.arc(0, 0, BOT_R + 6, 0, Math.PI * 2);
        ctx.strokeStyle = `rgba(234,179,8,${
          0.3 + Math.sin(S.simT * 6) * 0.2
        })`;
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // Selection ring
      if (S.selBot === b.id) {
        ctx.beginPath();
        ctx.arc(0, 0, BOT_R + 4, 0, Math.PI * 2);
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }

      // Body
      ctx.fillStyle = b.alive ? b.color : "#3f3f46";
      ctx.globalAlpha = b.alive ? 1 : 0.45;
      ctx.beginPath();
      ctx.arc(0, 0, BOT_R, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "rgba(255,255,255,.55)";
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.globalAlpha = 1;

      // Heading arrow
      ctx.rotate(b.theta);
      ctx.fillStyle = "#09090b";
      ctx.beginPath();
      ctx.moveTo(BOT_R - 2, -3);
      ctx.lineTo(BOT_R + 4, 0);
      ctx.lineTo(BOT_R - 2, 3);
      ctx.closePath();
      ctx.fill();
      ctx.rotate(-b.theta);

      // Letter
      ctx.fillStyle = "#09090b";
      ctx.font = "bold 10px Inter";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(b.letter, 0, 1);

      // Label under robot
      ctx.fillStyle = "rgba(255,255,255,.7)";
      ctx.font = "8px JetBrains Mono";
      ctx.textBaseline = "top";
      ctx.fillText(b.name.split(" ")[1], 0, BOT_R + 4);

      ctx.restore();
    }

    // Decision popover (canvas-rendered)
    renderDecision(ctx);
  }

  /* ================================================================
     13. DECISION POPOVER (rendered on canvas)
     ================================================================ */
  function renderDecision(ctx) {
    const d = S.decision;
    if (!d || d.timer <= 0) return;

    const pw = 235,
      ph = 120;
    let px = d.x + 40;
    let py = d.y - ph / 2;
    if (px + pw > W - 10) px = d.x - pw - 40;
    if (py < 8) py = 8;
    if (py + ph > H - 8) py = H - ph - 8;

    const alpha = Math.min(1, d.timer / 0.4);
    ctx.save();
    ctx.globalAlpha = alpha;

    // Background
    ctx.fillStyle = "rgba(11,11,15,.95)";
    roundRect(ctx, px, py, pw, ph, 8);
    ctx.fill();
    ctx.strokeStyle = "rgba(255,255,255,.1)";
    ctx.lineWidth = 1;
    roundRect(ctx, px, py, pw, ph, 8);
    ctx.stroke();

    // Connector line
    ctx.strokeStyle = "rgba(255,255,255,.1)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(d.x, d.y);
    ctx.lineTo(px < d.x ? px + pw : px, py + ph / 2);
    ctx.stroke();

    // Title bar
    ctx.fillStyle = "rgba(255,255,255,.04)";
    roundRectTop(ctx, px, py, pw, 20, 8);
    ctx.fill();
    ctx.fillStyle = "#fff";
    ctx.font = "bold 8px Inter";
    ctx.textAlign = "left";
    ctx.fillText("🔀 INTERSECTION NEGOTIATION", px + 7, py + 13);

    let y = py + 28;

    // Formula label
    ctx.fillStyle = "#5e5e6e";
    ctx.font = "7px JetBrains Mono";
    ctx.fillText("Score = Task + Deadline + Wait + Battery", px + 7, y);
    y += 12;

    // Winner line
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
    ctx.fillStyle = "#555";
    ctx.font = "8px JetBrains Mono";
    ctx.fillText(
      `T:${d.wD.t}  D:${d.wD.d}  W:${d.wD.w}  B:${d.wD.b}`,
      px + 14,
      y
    );
    y += 14;

    // Loser line
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
    ctx.fillStyle = "#555";
    ctx.font = "8px JetBrains Mono";
    ctx.fillText(
      `T:${d.lD.t}  D:${d.lD.d}  W:${d.lD.w}  B:${d.lD.b}`,
      px + 14,
      y
    );
    y += 14;

    // SVO badge
    ctx.fillStyle = C.beta;
    ctx.font = "8px JetBrains Mono";
    ctx.fillText(
      `θ_SVO → ${d.loser.svo}° (${d.loser.svoLabel})`,
      px + 7,
      y
    );

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
        ? "DEAD-ZONE"
        : b.yielding
          ? "YIELDING"
          : "ONLINE"
      : "OFFLINE";
    st.className =
      "status-pill" + (b.alive ? (b.yielding ? " yield" : "") : " off");

    el("iPose").textContent = `${Math.round(b.x)}, ${Math.round(b.y)}`;
    el("iVel").textContent = `${(b.spd / BASE_SPD).toFixed(2)} m/s`;
    el("iBat").textContent = `${Math.round(b.bat)}%`;
    el("iSvo").textContent = `θ:${b.svo}° (${b.svoLabel})`;
    el("iCap").textContent = `${b.maxPayload}kg / ${b.maxSpeed} m/s`;
    el("iPayload").textContent = b.curPayload;

    const score = calcScore(b);
    el("iScore").textContent = `${score}  (T:${b.taskPriority} D:${b.deadlineUrgency} W:${Math.min(3, Math.floor(b.waitTime))} B:${b.bat > 60 ? 1 : b.bat > 30 ? 2 : 3})`;

    // Comm radius label
    const nm = NET_MODES[S.netMode];
    const crl = el("commRLabel");
    if (crl) crl.textContent = nm.commR || "0";

    // Peers
    const pl = el("iPeers");
    let ph = "";
    for (const [id, o] of Object.entries(S.bots)) {
      if (id === b.id) continue;
      const d = Math.round(v2.dist(b, o));
      const inR = nm.commR > 0 && d <= nm.commR;
      const cls = !o.alive ? "dead" : inR ? "ok" : "disc";
      const txt = !o.alive
        ? "OFFLINE"
        : inR
          ? `${Math.round(nm.lat + d / 15)}ms`
          : "DISC";
      ph += `<div class="peer-row"><span>${o.name} (${d}px)</span><span class="peer-tag ${cls}">${txt}</span></div>`;
    }
    pl.innerHTML = ph;

    // Log
    const lg = el("iLog");
    const tagCls = {
      ORCA: "t-orca",
      SVO: "t-svo",
      MRTA: "t-mrta",
      RF: "t-rf",
      OBS: "t-obs",
      INIT: "t-init",
      ZENOH: "t-zenoh",
      FAULT: "t-mrta",
    };
    let lh = "";
    for (const e of b.log.slice(0, 6)) {
      const tc = tagCls[e.tag] || "t-init";
      lh += `<div class="log-row">[${e.ts}] <b class="${tc}">${e.tag}</b> ${e.msg}</div>`;
    }
    lg.innerHTML = lh;

    // Kill button
    el("killText").textContent = b.alive
      ? `Kill ${b.name}`
      : `Revive ${b.name}`;
    el("killBtn").className = "kill-btn" + (b.alive ? "" : " revive");

    // Peer count
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
    // Scenarios
    document
      .querySelectorAll(".sc-btn")
      .forEach((b) =>
        b.addEventListener("click", () => loadScenario(+b.dataset.scenario))
      );

    // Play/Pause
    const ppb = document.getElementById("playPauseBtn");
    ppb.addEventListener("click", () => {
      S.running = !S.running;
      ppb.textContent = S.running ? "⏸" : "▶";
    });

    // Speed
    const sb = document.getElementById("speedBtn");
    sb.addEventListener("click", () => {
      S.speed = S.speed === 1 ? 2 : S.speed === 2 ? 0.5 : 1;
      sb.textContent = S.speed + "x";
    });

    // Reset
    document.getElementById("resetBtn").addEventListener("click", () => {
      S.simT = 0;
      initBots();
      loadScenario(S.scenario);
    });

    // Cloud toggle
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

    // Network mode
    document.querySelectorAll(".net-btn").forEach((b) =>
      b.addEventListener("click", () => {
        const mode = +b.dataset.mode;
        S.netMode = mode;
        document
          .querySelectorAll(".net-btn")
          .forEach((x) =>
            x.classList.toggle("active", +x.dataset.mode === mode)
          );
        const nm = NET_MODES[mode];
        showToast(
          `Network: ${nm.name}${mode === 2 ? " — LiDAR safety bubble σ=1.5" : nm.commR > 0 ? ` (Rc=${nm.commR}px, ~${nm.lat}ms)` : ""}`
        );
        for (const bot of Object.values(S.bots)) {
          if (bot.alive) bot.addLog("ZENOH", `Net → ${nm.name}`);
        }
      })
    );

    // Robot tabs
    document.getElementById("robotTabs").addEventListener("click", (e) => {
      const tab = e.target.closest(".tab");
      if (!tab) return;
      document
        .querySelectorAll(".tab")
        .forEach((t) => t.classList.remove("active"));
      tab.classList.add("active");
      S.selBot = tab.dataset.robot;
      updateInsp();
    });

    // Kill
    document.getElementById("killBtn").addEventListener("click", () => {
      const b = S.bots[S.selBot];
      if (!b) return;
      b.alive = !b.alive;
      if (!b.alive) {
        b.addLog("FAULT", "Node killed");
        let best = null,
          bestD = Infinity;
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

    // Canvas click
    S.canvas.addEventListener("click", (e) => {
      const rect = S.canvas.getBoundingClientRect();
      const cx = ((e.clientX - rect.left) / rect.width) * W;
      const cy = ((e.clientY - rect.top) / rect.height) * H;

      // Click robot to select
      for (const [id, b] of Object.entries(S.bots)) {
        if (v2.dist({ x: cx, y: cy }, b) < BOT_R + 10) {
          S.selBot = id;
          document
            .querySelectorAll(".tab")
            .forEach((t) =>
              t.classList.toggle("active", t.dataset.robot === id)
            );
          updateInsp();
          return;
        }
      }

      // Scenario 3: spawn/remove obstacles on nav nodes
      if (S.scenario === 3) {
        const ex = S.obstacles.findIndex(
          (o) => v2.dist({ x: cx, y: cy }, o) < o.r + 12
        );
        if (ex !== -1) {
          removeObstacle(ex);
          showToast("Obstacle removed — A* paths recalculated");
        } else {
          const nn = nearestNode(cx, cy, false);
          if (nn && !blockedNodes.has(nn.id) && v2.dist({ x: cx, y: cy }, nn) < 60) {
            spawnObstacle(nn.id);
            for (const bot of Object.values(S.bots)) {
              if (bot.alive)
                bot.addLog("OBS", `Blocked node ${nn.id} — rerouting`);
            }
            showToast(
              `Obstacle at (${nn.x}, ${nn.y}) — all robots auto-rerouted`
            );
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