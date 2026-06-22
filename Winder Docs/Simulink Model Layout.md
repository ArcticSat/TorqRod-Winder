
# Model Configuration

Solver: ode4
Step size: 1e-3 sec

# Subsystems
## Speed Controller

Input: speed_sense
Output: step_count
### Subsystems
- Signal Generator 
	- step -> ramp
	- out: speed input
- Transfer Function (notch filter)
	- in: speed_sense
	- out: omega_filtered
- PI Controller
	- in: speed error
	- out: correction signal
- Feed forward controller (not connected)
	- in: speed input
	- out: feed forward signal
- Integrator 
	- in: (feed forward - correction)
	- out: step count


## Tension Controller

Input: T_measured
Output: spool_torq_cmd
### Subsystems
- PI Controller
	- in: error signal
	- out: spool_torq_cmd
	- output limited to DC.V_max


## Spindle Motor

Input: T_load / step_count
Output: omega_spindle / theta_spindle
### Subsystems
- stepper_simplified (matlab function)
	- in: step_count, omega, theta, T_load
	- out: domega, dtheta


## Spool Motor

Input: spool_torq_cmd, T_wire
Output: wire_supply_rate
### Subsystems
- Transfer function (electrical)
	- in: voltage_error
	- 1/(DC.L\*s + DC.R)
	- out: motor_current
- torque conversion
	- in: T_wire
	- out: shaft_torque
- transfer function (mechanical)
	- in: motor_torque - shaft_torque
	- 1/(DC.J_reflected\*s + DC.B)
	- out: omega_motor
- spool
	- in: omega_motor
	- out: wire_supply_rate


## Winding Geometry

input: T_wire, omega_spindle, theta_spindle
output: wire_consump_rate, layer, T_load
### Subsystems
- winding_geometry (matlab function)
	- in: theta_spindle, omega_spindle, T_wire
	- out: R_eff, wire_consump_rate, turns_total, T_load, layer


## Cam/Follower

input: omega_spindle, theta_spindle
output: v_follower


## Disturbance

input: v_follower, omega_spindle, layer
output: disturbance

## Subsystems
- cam_reversal_disturbance
	- in: dz_dt, dz_dt_prev, dt
	- out: reversal_disturbance
- layer_transition_disturbance
	- in: layer, layer_prev, omega_spindle
	- out: layer_disturbance


## Wire Tension

input: wire_consump_rate, wire_supply_rate, disturbance
output: T_wire


## Dancer Arm

input: T_wire
output: T_measured

### Subsystems
- dancer_arm_ode (matlab function)
	- in: theta, dtheta, T_wire
	- out: d2theta
