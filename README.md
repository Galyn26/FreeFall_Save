# 🪂 FreeFall_Save — Egress Vector Redirection Simulation

A physics-driven R&D prototype built in Python using **Pymunk** and **PyBullet** to evaluate the mechanical feasibility and human survivability of high-altitude emergency vector redirection.

---

## 📌 Project Overview
When dropping from extreme heights, absorbing $100\%$ of vertical kinetic energy upon ground impact results in catastrophic, non-survivable G-force spikes ($>2,000\text{ Gs}$).

**FreeFall_Save** models a state-machine-triggered egress system that intercepts a high-velocity falling body at a predetermined altitude threshold ($15\text{ m}$), using a $45^\circ$ deflection profile and non-Newtonian drag parameters to convert downward vertical momentum into controlled horizontal sliding kinetic energy.

---

## 📊 Telemetry & Proof of Concept

By smoothing the vector redirection over a finite time window ($\Delta t \approx 0.35\text{ s}$) rather than an instantaneous step, the simulation successfully demonstrated a **$99.4\%$ reduction in peak acceleration load**, bringing impact forces down from a lethal $2,300\text{ Gs}$ to a survivable **$12.5\text{ G}$** operational impulse.

*(Insert your Matplotlib graph screenshot here: `![Telemetry Analysis](./telemetry_graph.png)`)*

### Key Simulation Variables Tested
* **Drop Altitude:** $50.0\text{ m}$
* **Trigger Threshold:** $15.0\text{ m}$
* **Subject Mass:** $75.0\text{ kg}$
* **Vector Conversion:** $70\%$ Horizontal Conversion / $30\%$ Shock Absorption
* **Peak G-Force Load:** $\approx 12.5\text{ Gs}$ (within human tolerance limits for sub-second pulses)

---

## 🚀 Repository Structure

```text
FreeFall_Save/
├── egress_sim.py        # 2D kinematic telemetry simulation (Pymunk + Matplotlib)
├── egress_sim_3d.py     # 3D rigid-body spatial simulation (PyBullet OpenGL GUI)
├── README.md            # Project documentation & analysis
└── .gitignore           # Environment & IDE exclusions
```

## 🛠️ Tech Stack & Dependencies 
* Language: Python 3.12+
* IDE: JetBrains IntelliJ IDE
* APhysics Engines: pymunk (2D rigid-body dynamics), pybullet (3D multi-body OpenGL physics)
* Data Visualization: matplotlib🏃 

## Quickstart

1. Clone the Repository
```Bash
git clone [https://github.com/Galyn26/FreeFall_Save.git](https://github.com/YOUR_GITHUB_USERNAME/FreeFall_Save.git)
cd FreeFall_Save
```

2. Set Up Virtual Environment & Install Dependencies

```Bash
python3 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install pymunk pybullet matplotlib
```
### Run the SimulationsRun 2D Kinematic Telemetry Engine:
```Bash
python egress_sim.py
```
Generates real-time graphs tracking Altitude, Vertical/Horizontal Velocity, and G-Force Load.

### Run 3D Visual Physics Simulation:
```Bash
python egress_sim_3d.py
```
Launches an interactive PyBullet OpenGL window displaying the $3\text{D}$ spatial trajectory and horizontal rollout.