# Field and Service Robotics - Homework 3

**University of Naples Federico II**  
Master's Degree in Automation and Robotics Engineering

**Student:** Emanuela Varone  

---

## Overview

This repository contains the solutions for Homework 3 of the Field and Service Robotics course. The homework focuses on UAV dynamics, aerodynamic effects, disturbance estimation, and advanced control strategies for quadrotors.

## Repository Structure

```
Field_and_Service_Robotics_HW3/
├── README.md
├── Report_HW3_FSR.pdf
├── POINT_3/
│   ├── POINT3_HW3.mlx
│   ├── quaternion_utils.m
│   └── ws_homework_3_2025.mat
├── POINT_4/
│   ├── POINT_4.mlx
│   ├── geometric_control_template.slx
│   └── PLOT/
│       ├── angular_velocity_errors.eps
│       ├── control_signals.eps
│       ├── position_errors.eps
│       ├── position_tracking.eps
│       ├── rotational_errors.eps
│       └── velocity_errors.eps
└── POINT_5/
    ├── POINT_5.mlx
    ├── tilting_control_voliro_template.slx
    └── VIDEO/
        └── UAV_trajectory.mp4
```

## Topics Covered

### 1. Octocopter System Analysis
Analysis of an octocopter with co-planar propellers:
- **Degrees of Freedom**: 8 DoFs (body fixed) / 14 DoFs (flying)
- **Configuration Space Topology**: T⁸ (fixed) / R³ × S² × S¹ × T⁸ (flying)
- **Underactuation Analysis**: Despite 8 actuators, the system is underactuated (can only control 4 DoFs directly)
- **Allocation Matrix Derivation**: 4×8 matrix relating control inputs to rotor speeds

### 2. Aerodynamic Effects
Comprehensive analysis of proximity aerodynamic phenomena:

| Effect | Mechanism | Stability | Formula |
|--------|-----------|-----------|---------|
| **Ground Effect** | Compressed air cushion below rotor | Stabilizing | T_IGE/T_OGE = 1/(1-(ρ/4z)²) |
| **Ceiling Effect** | Vacuum effect reducing drag | Destabilizing | T_ICE/T_OCE = 1/(1-1/k₁(ρ/(z+k₂))²) |

Key considerations for multi-rotor configurations and control implications.

### 3. Momentum-based Estimator
Implementation of a recursive disturbance estimator for UAV systems:

**Disturbances applied:**
- 1N along x-axis and y-axis
- -0.4 Nm around yaw axis

**Parameter Analysis (k₀ = 20):**

| r Value | Behavior |
|---------|----------|
| 1-15 | Increasing accuracy, reduced oscillations |
| 20-40 | Good compromise, stable estimates |
| 40-80 | Performance plateau, marginal improvements |
| >100 | Numerical instability |

**Optimal range:** r = 40-60

**Mass Estimation:** Real UAV mass estimated at 1.25 kg (16.7% lower than nominal 1.5 kg)

### 4. Geometric Control of a Quadrotor
Implementation of a geometric controller with cascaded architecture:

**Control Structure:**
- **Outer Loop**: Position control generating desired thrust
- **Inner Loop**: Attitude control on SO(3)

**Tuned Gains:**
- Position: Kp = diag([80, 80, 150]), Kv = diag([8, 8, 20])
- Attitude: KR = 60·I₃, Kω = 20·I₃

**Trajectory:** From p_i = [0, 0, -1]m to p_f = [1, 1, -4]m with yaw rotation 0° → 20°

**Performance:**
- Position errors: ~10⁻⁸ m (x,y), ~10⁻⁶ m (z)
- Velocity errors: ~10⁻⁷ m/s (x,y), ~10⁻⁵ m/s (z)
- Excellent tracking with minimal control effort

### 5. Tilting Quadrotor Controller (Voliro Approach)
Implementation of a controller for a tilting-rotor quadrotor achieving full 6-DoF control:

**Control Architecture:**
- PD control with feedforward terms
- Pseudoinverse allocation strategy
- Independent position and orientation control

**Tuned Gains:**
- Position: Kp = diag([140, 140, 250]), Kd = diag([70, 70, 85])
- Attitude: Kr = diag([200, 200, 350]), Kw = diag([120, 120, 160])

**Key Results:**
- Position tracking accuracy: <10⁻⁵ m
- Coordinated tilt angles reaching ±1 rad
- Successful 3D diagonal descending trajectory

## Requirements

- MATLAB R2020a or later
- Simulink
- Symbolic Math Toolbox
- Aerospace Blockset (recommended)

## Usage

Each point folder contains:
- A MATLAB Live Script (`.mlx`) for initialization and visualization
- A Simulink model (`.slx`) for control system simulation (where applicable)

**To run the simulations:**

1. Open MATLAB and navigate to the desired point folder
2. Open the `.mlx` file
3. Run all sections sequentially
4. For Simulink models, the script will configure and run the simulation

## Videos

- `POINT_5/VIDEO/UAV_trajectory.mp4` - Tilting quadrotor 3D trajectory tracking demonstration

## Key Findings Summary

| Point | Topic | Main Result |
|-------|-------|-------------|
| 1 | Octocopter | Underactuated despite 8 rotors (rank deficiency) |
| 2 | Aerodynamics | Ground effect stabilizes, ceiling effect destabilizes |
| 3 | Estimator | Optimal r ∈ [40-60], real mass = 1.25 kg |
| 4 | Geometric Control | Position errors < 10⁻⁶ m |
| 5 | Voliro Control | Full 6-DoF control with tilting rotors |

## License

This project is for educational purposes as part of the Field and Service Robotics course at University of Naples Federico II.
