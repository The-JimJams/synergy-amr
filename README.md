# ⬡ Synergy AMR — Edge-AI Based Distributed Fleet Coordination for AMRs in Smart Warehouses

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-3498db?logo=ros)](https://docs.ros.org/en/humble/)
[![Middleware](https://img.shields.io/badge/Middleware-rmw__zenoh-22c55e?logo=rust)](https://zenoh.io/)
[![Architecture](https://img.shields.io/badge/Architecture-Decentralized_P2P_Mesh-eab308)](#core-architectural-pillars)
[![License](https://img.shields.io/badge/License-MIT-purple)](#license)

> **Demonstration Notice:** *This repository provides an interactive, full-stack simulation demonstrating **Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots (AMRs) in Smart Warehouses**.*

---

## 📌 Executive Summary

Traditional Automated Guided Vehicles (AGVs) and Autonomous Mobile Robots (AMRs) in logistics rely on **centralized Fleet Management Systems (FMS)**. While simple to deploy, centralized coordination suffers from critical vulnerabilities:
- **Single Point of Failure (SPOF):** If the central server or Wi-Fi gateway fails, all warehouse robots freeze.
- **Latency & Scalability Bottleneck:** As fleet size scales beyond dozens of robots, central path planners experience exponential computational complexity and network congestion.
- **Deadlock Vulnerability:** Corridors and 4-way intersections frequently deadlock when dynamic obstacles or communication drops occur.

**Synergy AMR** demonstrates a **fully decentralized, peer-to-peer (P2P) edge architecture** where AMRs autonomously negotiate right-of-way, allocate warehouse tasks, and avoid collisions using onboard compute (NVIDIA Jetson / Raspberry Pi), local LiDAR perception, and P2P mesh networking.

---

## 🏛️ Core Architectural Pillars

```
                     ┌─────────────────────────────────────────┐
                     │    WAREHOUSE MANAGEMENT SYSTEM (WMS)    │
                     │         (Central Task Broadcast)        │
                     └────────────────────┬────────────────────┘
                                          │ (Order Requests)
                                          ▼
      ┌───────────────────────────────────────────────────────────────────────┐
      │               DECENTRALIZED P2P MESH (ROS 2 / rmw_zenoh)              │
      ├───────────────────┬───────────────────┬───────────────────┬───────────┤
      │   Robot Alpha (α) │   Robot Beta (β)  │  Robot Gamma (γ)  │ Robot δ   │
      │ Jetson Orin Nano  │  Raspberry Pi 5   │  Jetson Orin NX   │ AGX Orin  │
      │  Heavy Lifter     │   Agile Runner    │  Standard Hauler  │ Pallet    │
      └─────────┬─────────┴─────────┬─────────┴─────────┬─────────┴─────┬─────┘
                │                   │                   │               │
       [ORCA / SVO Nav]    [Spatial Tokens]    [DC-MRTA Auction]  [LiDAR SLAM]
```

### 1. Decentralized P2P Mesh Communication (`rmw_zenoh`)
- Replaces heavyweight centralized DDS brokers with **Zenoh P2P middleware**.
- AMRs publish **Trajectory Intent Vectors** and **Kinematic Odometry** directly to nearby peers within communication radius ($R_c = 210\text{px}$, latency $\sim 11\text{ms}$).
- Subscriptions are dynamically managed on edge hardware without reliance on external servers.

### 2. Multi-Agent Collision Avoidance & SVO Turn-Taking
- **Optimal Reciprocal Collision Avoidance (ORCA):** Each AMR continuously computes a velocity obstacle half-plane and selects safe collision-free velocities.
- **Social Value Orientation (SVO):** Heterogeneous robots adjust yielding aggressiveness based on their role:
  - **Robot Alpha (Heavy Lifter, SVO $25^\circ$):** Prioritized passage due to heavy momentum and pallet payload.
  - **Robot Beta (Agile Runner, SVO $88^\circ$):** Highly altruistic yielding behaviour, pausing and hugging corridor walls to let heavy haulers pass.
- **Pure-Pursuit Spline Steering:** Lookahead trajectory interpolation along Catmull-Rom splines eliminates $180^\circ$ singularity oscillations.

### 3. Spatial Reservation Tokens (Zero-Deadlock Intersections)
- Critical intersections (e.g., Central 4-Way `JX_2_3`) operate under decentralized spatial reservation tokens.
- Approaching AMRs broadcast reservation requests. The robot with the highest composite priority score (calculated from task priority, deadline urgency, wait time, and battery SOC) claims the intersection token for a short Time-to-Live (TTL).
- Other AMRs hold at designated stop-lines ($d \approx 72\text{px}$) and clear sequentially in $< 3.0\text{s}$ with zero deadlocks.

### 4. Distributed Task Allocation (DC-MRTA)
- When a new WMS order is broadcast (e.g., *Pick SKU-A from Bay 02, Deliver to Packing Dock*), the fleet runs a **Dynamic Consensus Multi-Robot Task Allocation (DC-MRTA)** auction locally.
- Each robot computes a bid score:
  $$\text{Bid Score} = f(\text{Distance to Bay}, \text{Battery SOC}, \text{Payload Capacity}, \text{Current Load})$$
- The optimal robot wins the auction, loads cargo at the storage bay, and navigates autonomously to the conveyor dock.

### 5. Autonomous Home Base Recall & Docking
- Halting an AMR does **not** stop it dead in the middle of a busy aisle.
- Instead, the robot automatically re-auctions any carried task to its nearest peer and navigates to its dedicated charging pad:
  - **Alpha** $\rightarrow$ **PWR 1** (Top-Left: $x=28, y=28$)
  - **Beta** $\rightarrow$ **PWR 2** (Top-Right: $x=932, y=28$)
  - **Gamma** $\rightarrow$ **PWR 3** (Bottom-Left: $x=28, y=550$)
  - **Delta** $\rightarrow$ **PWR 4** (Bottom-Right: $x=932, y=550$)
- Upon arrival, it enters `PARKED @ BASE` mode and rapidly charges its battery.

---

## 🤖 Heterogeneous Fleet Profiles

| AMR | Callout | Onboard Edge Compute | Primary Role | Max Payload | Max Velocity | SVO Character | Home Base |
| :--- | :---: | :--- | :--- | :---: | :---: | :---: | :---: |
| **Alpha** | $\alpha$ (Cyan) | NVIDIA Jetson Orin Nano | Heavy Lifter | $500\text{ kg}$ | $1.2\text{ m/s}$ | $25^\circ$ (Individualistic) | **PWR 1** |
| **Beta** | $\beta$ (Amber) | Raspberry Pi 5 (8GB) | Agile Runner | $100\text{ kg}$ | $2.1\text{ m/s}$ | $88^\circ$ (Altruistic) | **PWR 2** |
| **Gamma** | $\gamma$ (Green) | NVIDIA Jetson Orin NX | Standard Hauler | $250\text{ kg}$ | $1.5\text{ m/s}$ | $50^\circ$ (Prosocial) | **PWR 3** |
| **Delta** | $\delta$ (Purple) | NVIDIA Jetson AGX Orin | Precision Handler | $150\text{ kg}$ | $1.8\text{ m/s}$ | $40^\circ$ (Prosocial) | **PWR 4** |

---

## 🎮 Interactive Simulation Scenarios

The simulator includes 5 built-in edge case test scenarios selectable from the top navigation bar:

1. **Scenario 1 — 4-Way Cross:**
   4 AMRs simultaneously converge on the Central 4-Way Junction (`JX_2_3`). Demonstrates decentralized spatial token arbitration, stop-line holding, and rapid turn-taking without deadlocks.
2. **Scenario 2 — SVO Yield:**
   Head-on encounter between Robot Alpha (Heavy) and Robot Beta (Agile). Demonstrates altruistic SVO yielding where Beta gives way to Alpha.
3. **Scenario 3 — Obstacles:**
   Dynamic obstacles placed in warehouse aisles. AMRs use real-time raycast LiDAR (8 laser beams) and dynamic A* to re-route around blockages.
4. **Scenario 4 — Dead Zone (RF Blackout):**
   Wi-Fi communication blackout in Quadrant 2. AMRs detect lost P2P packets and dynamically inflate their LiDAR safety bubble ($\sigma = 1.5$) to safely navigate blindly.
5. **Scenario 5 — Node Failure:**
   Simulates catastrophic hardware failure of an AMR node. Peer robots detect heartbeat loss via Zenoh gossip and automatically re-auction its pending task.

---

## 🌐 Network Resilience & Edge Controls

The top toolbar allows testing various real-world industrial networking conditions:

- **`Cloud / Severed` Toggle:**
  - **`Cloud` (ON):** Standard operation with cloud telemetry.
  - **`Severed` (OFF):** Simulates complete central server disconnect. The fleet continues navigating, avoiding collisions, and auctioning tasks with 100% edge autonomy.
- **Network Modes:**
  - **`Full`:** $R_c = 210\text{px}$, $11\text{ms}$ latency, $0.0\%$ packet loss.
  - **`Degraded`:** $R_c = 110\text{px}$, $240\text{ms}$ latency, $18.4\%$ packet loss.
  - **`Offline`:** $R_c = 0$, $100\%$ loss (AMRs switch to LiDAR safety bubbles).
  - **`Recovery`:** Fast state reconciliation over restored mesh links.

---

## 🚀 Quick Start Guide

### Prerequisites
- **Python 3.8+**
- Modern web browser (Chrome, Edge, Firefox, Safari)

### Installation & Execution

1. **Clone the repository:**
   ```bash
   git clone https://github.com/The-JimJams/synergy-amr.git
   cd synergy-amr
   ```

2. **Install dependencies:**
   ```bash
   pip install flask
   ```

3. **Run the server:**
   ```bash
   python app.py
   ```

4. **Open in browser:**
   Navigate to [http://127.0.0.1:5000](http://127.0.0.1:5000) in your web browser.

---

## 📂 Repository Structure

```
synergy-amr/
├── app.py                 # Flask web server serving index.html and static assets
├── templates/
│   └── index.html         # Single-page interface, telemetry inspector, ROS 2 topics
├── static/
│   ├── script.js          # Core simulation engine:
│   │                      # - SLAM grid & A* pathfinder
│   │                      # - ORCA & SVO multi-agent physics
│   │                      # - Spatial reservation token manager
│   │                      # - DC-MRTA auction algorithm
│   │                      # - Canvas renderer & LiDAR raycasting
│   └── style.css          # Industrial dark-theme CSS, glassmorphism telemetry cards
└── README.md              # Project documentation & engineering specifications
```

---

## 📜 License

This project is open-source and available under the [MIT License](LICENSE).
