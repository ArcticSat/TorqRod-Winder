# TorqRod-Winder

# Precision Inductor Winder — 36 AWG

A precision winding machine designed
for Nanosat torque rods. Features a barrel cam traversal
mechanism, active wire tension control, and a full Simulink simulation of
the control system.

---

## Overview

This machine winds 36 AWG (0.127mm bare / ~0.140mm enamelled) copper magnet
wire onto cylindrical bobbins with close-wound precision. It was designed
around a few specific goals:

- **Consistent layer-by-layer winding** with ~0.137mm pitch (within enamel
  tolerance of wire diameter)
- **Active tension control** to keep wire tension at 0.025 N throughout,
  avoiding breaks (break load ~0.25 N) and slack
- **Repeatable traversal** via a reversing single-groove barrel cam —
  no groove-crossing interference, clean layer transitions
- **Fully modelled control system** in Simulink before any hardware was
  built, with tuned PI gains and a notch filter for stepper resonance

---

## Key Specifications

| Parameter              | Value                                      |
|------------------------|--------------------------------------------|
| Wire                   | 36 AWG copper magnet wire, 0.140mm eff. dia |
| Winding length (stroke)| 76 mm                                      |
| Bobbin core radius     | 4.5 mm                                     |
| Spindle drive          | NEMA 17 stepper (SparkFun 09238)           |
| Belt drive ratio       | 92.16:1 (4-stage GT2)                      |
| Cam type               | Reversing single-groove barrel cam         |
| Cam radius             | 30 mm                                      |
| Traversal rail         | MGN12H linear rail, 76 mm stroke           |
| Tension setpoint       | 0.025 N                                    |
| Tension sensor         | AS5600 magnetic encoder on dancer arm      |
| Supply spool drive     | Pololu 4887 brushed DC gearmotor (99:1)    |
| Frame                  | 4040 aluminium T-slot extrusion            |
| Microcontroller        | STM32F303K8                     |

---

## Repository Structure

TorqRod-Winder/
│
├── Winder Docs/
│   ├── Welcome.md                  — project overview
│   ├── Winder_project_summary.md   — full design reference
│   ├── Winder_layout.md            — cross member positions and components
│   ├── Winder BOM.md               — bill of materials
│   ├── Winder electronics setup.md — wiring and electronics guide
│   └── Winder TODO.md              — project task checklist
│
├── matlab/
│   ├── winder.slx                  — Simulink control system model
│   ├── torqRodWinder.m             — main simulation script
│   ├── winder_init.m               — initialization script
│   ├── camSVAJ.m                   — cam profile generation
│   ├── cam_groove1.csv             — cam profile data (groove 1)
│   └── cam_groove2.csv             — cam profile data (groove 2)
│
├── firmware/                       — STM32 firmware (not started)
│
└── LICENSE

---

## Mechanical Design

### Frame
4040 aluminium T-slot extrusion, 730 × 280 × 150 mm overall.
Twelve cross members span the Y axis and carry all major components.
All drive shafts run parallel to Y at 172 mm above the base.

### Belt Drive
Four-stage GT2 timing belt reduction from spindle to cam shaft:

| Stage | Driver | Driven | Ratio |
|-------|--------|--------|-------|
| 1     | 18T    | 72T    | 4.0:1 |
| 2     | 18T    | 72T    | 4.0:1 |
| 3     | 20T    | 48T    | 2.4:1 |
| 4     | 20T    | 48T    | 2.4:1 |
| **Total** |    |        | **92.16:1** |

Stages 1 & 3 share belt plane A (inner); stages 2 & 4 share plane B
(+15 mm axial offset) to prevent interference.

### Barrel Cam
Reversing single-groove design eliminates the groove-crossing problem that
affected an earlier two-groove concept (groove centerlines came within 2.2 mm
of each other — insufficient clearance for a 3 mm follower ball). The single
reversing groove has 12.7 mm spacing between adjacent parallel passes.

- Cam profile generated in MATLAB, imported to Fusion 360 via CSV spline
- Cycloidal motion law
- 3 mm Si3N4 ceramic ball follower in a V-groove (90° included angle)

### Dancer Arm / Tension Sensing
A spring-loaded dancer arm between the supply spool and bobbin measures
wire tension indirectly via a 6 mm diametrically magnetised disc magnet
and an AS5600 12-bit magnetic encoder. A 0.15 mm stainless shim stock
torsion spring provides a calibrated restoring force (K = 0.0229 Nm/rad).

---

## Control System

The full control system is modelled in `matlab/winder.slx`.

### Spindle Speed Controller
Feedforward + PI architecture. A rate limiter ramps the speed setpoint at
5.24 rad/s² to avoid resonance issues. A notch filter on the speed feedback
(ωₙ ≈ 42 rad/s) suppresses the stepper resonance at ~43 Hz.

### Tension Controller
PI controller (Kp = 90 V/N, Ki = 170 V/N·s) driving the supply spool
brushed DC motor bidirectionally. The dancer arm AS5600 provides direct
physical tension measurement — faster and more reliable than a back-EMF
estimator for handling transients like cam reversals.

### Running the Simulation

1. Open MATLAB in the `matlab/` directory
2. Run `winder_init.m` to load all workspace variables
3. Open and run `winder.slx`

---

## Project Status

| Item                  | Status         |
|-----------------------|----------------|
| Frame CAD             | ✅ Complete     |
| Cam cylinder CAD      | ✅ Complete     |
| U-bracket (CM7)       | ✅ Complete     |
| Dancer arm CAD        | 🔧 In progress |
| Cam follower arm CAD  | 🔧 In progress |
| Supply spool hub      | ❌ Not started  |
| Bobbin CAD            | ❌ Not started  |
| Simulink model        | ✅ Complete     |
| STM32 firmware        | ❌ Not started  |

---
