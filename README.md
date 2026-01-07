# Discrete-Time PID Controller with Anti-Windup and Disturbance Rejection

## Overview
This project implements a discrete-time PID controller in C++ and analyzes its behavior using Python-based visualization.
The project follows a structured control design workflow: initial parameter exploration is first performed to build intuition,
followed by focused experiments on disturbance rejection under actuator saturation with anti-windup.

---

## System Model
The plant is modeled as a first-order system with an additive disturbance:

\[
\dot{x} = -x + (u + d), \quad y = x
\]

The system is implemented in discrete time using forward Euler integration.

- Actuator saturation:
\[
u \in [u_{\min}, u_{\max}]
\]
- Disturbance: pulse disturbance applied between **t = 5 s** and **t = 10 s**

---

## Controller Design
A PID controller with back-calculation anti-windup is implemented:

\[
u_{unsat} = K_p e + K_i I + K_d \dot{e}
\]

\[
u = \mathrm{sat}(u_{unsat})
\]

\[
\dot{I} = e + K_{aw}(u - u_{unsat})
\]

where \(e = r - y\) is the tracking error and \(K_{aw}\) is the anti-windup gain.

---

## Initial Exploration: Reference and Anti-Windup Gain
As an initial step, the controller behavior was explored for different reference values and anti-windup gains.
These experiments were used to study tracking performance under feasible and infeasible reference conditions,
understand actuator saturation effects, and gain intuition on integrator windup behavior.

The results of this exploration informed the design of the main disturbance rejection experiments.

---

## Main Experiments: Saturation and Disturbance Rejection

### Baseline Case (Infeasible Reference)
- Reference: \( r = 3 \)
- Actuator limit: \( u_{\max} = 2 \)

In this case, output responses are nearly identical for different values of \(K_{aw}\) due to actuator saturation.
However, without anti-windup, the integrator state and unsaturated control input grow excessively.
This shows that anti-windup primarily improves internal controller stability and safety when output performance is constrained.

---

### Extension Case (Saturation Released)
- Actuator limit increased to \( u_{\max} = 3 \)

When saturation is released after disturbance removal, output recovery behavior differs depending on \(K_{aw}\).
A controller without anti-windup recovers faster due to accumulated integrator action but exhibits aggressive control effort,
while larger anti-windup gains produce slower but more stable and predictable recovery.
This highlights the trade-off between recovery speed and robustness.

---

## Quantitative Analysis
Recovery time after disturbance removal (t = 10 s) is defined as the time required for the output
to return within **±2% of the steady-state value**.

| Kaw | Recovery Time (s) | max |I| | max |u_unsat| |
|-----|------------------|------|--------------|
| 0   | 2.81             | 8.00 | 72.0 |
| 1   | 3.64             | 1.00 | 72.0 |
| 5   | 3.84             | 2.21 | 72.0 |

---

## Project Structure
