# Discrete-Time PID Controller with Anti-Windup and Disturbance Rejection

## Overview
This project implements a discrete-time PID controller in C++ and analyzes its behavior using Python-based visualization.
The project follows a structured control design workflow: initial parameter exploration is first performed to build intuition,
followed by focused experiments on disturbance rejection under actuator saturation with anti-windup.

---

## System Model
The plant is modeled as a first-order system with an additive disturbance:

dx/dt = -x + (u + d)  
y = x
(u = control input  d = external disturbance)

The system is implemented in discrete time using forward Euler integration.

- Actuator saturation:  
  u ∈ [u_min, u_max]
- Disturbance: pulse disturbance applied between t = 5 s and t = 10 s

---
## Experiments and Results

### Reference and Saturation Effects
Initial experiments explored different reference values and anti-windup gains
to understand tracking behavior under actuator saturation.

### Disturbance Rejection with Anti-Windup
- Reference: r = 3
- Disturbance applied between 5 s and 10 s

Two cases were considered:
1. Infeasible reference under saturation (u_max = 2)
2. Saturation released after disturbance (u_max = 3)

When saturation is active, output responses are similar for different anti-windup gains,
but internal controller states differ significantly.
When saturation is released, anti-windup affects recovery speed and control effort.

---

## Quantitative Analysis
A quantitative analysis confirmed that anti-windup limits integrator growth
and results in more stable recovery behavior after disturbance removal,
although faster recovery is not always guaranteed.

---
## Conclusion
Anti-windup does not always improve output recovery speed.
However, it is essential for preventing integrator windup, bounding internal control signals,
and ensuring safe and predictable controller behavior under actuator saturation.

By progressing from basic parameter exploration to focused disturbance rejection experiments,
this project demonstrates both the benefits and trade-offs of anti-windup in practical control systems.

---

## Project Structure
- src/ C++ simualtion code
- analysis/ Python for plotting and analysis
- data/ Logged simulation data
