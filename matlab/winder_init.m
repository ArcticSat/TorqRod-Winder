% winder_init.m
% initializes torque rod winder simulation


clear all
close all
%% Timing
Time.sim_time = 3; % [s]

%% Wire parameters
Core.R = 4.5e-3; % [m]
Core.woundL = 76e-3; % wound length[m]
Wire.diam = 0.140e-3; % [m] 36 AWG with enamel
Wire.v_max  = 0.5; % [m/s] conservative for fine wire
Wire.omega_max = Wire.v_max / Core.R;
Wire.RPM_max = Wire.omega_max*60/(2*pi);
Wire.E_copper = 120e9; % Pa
Wire.A = pi * (0.0635e-3)^2; % m^2, 36 AWG bare radius
Wire.L_free = 0.2; % [m], free wire length between spool and core
Wire.K = 7.6; % [N/m]
Wire.K_spring = 2.4; % spring constant [N/m]
Core.layer_turns = floor(Core.woundL/Wire.diam);
Wire.tau_spool = 0.05; % [s] spool motor response time estimate
Wire.T_pre = 0.010; % [N]
Wire.T_set = 0.025; % [N]
Wire.T_min = 0.005; % [N]
Wire.T_max = 0.150; % [N]
Wire.T_trip = 0.6*Wire.T_max; % [N]

%% Spool Parameters
Spool.R = 0.02; % spool radius [m]

%% Spindle motor
Stepp.V_supp = 12;     % supply voltage [V]
Stepp.R_phase  = 13.0;       % Ohms per phase (from datasheet)
Stepp.L  = 0.0057;     % 5.7 mH per phase [H]
Stepp.Nr = 50;         % rotor teeth (200 steps / 4 phases)
Stepp.Kt = 0.034;      % [Nm/A] (holding torque 0.23Nm / 6.8 peak amps... 
                        % verify from your datasheet)
Stepp.Ke = Stepp.Kt;   % equal in SI
Stepp.J  = 68e-7;      % NEMA 17 [kg m^2] 
Stepp.B  = 3.1e-2;       % Nm/(rad/s), light damping — tune this
Stepp.I_rated   = 0.33;       % A per phase

Stepp.microstep_divisor = 16;
Stepp.step_per_rev = (200*Stepp.microstep_divisor);
Stepp.omega_target = 1; % speed target [rad/s]
Stepp.max_accel = 5; % rate limiter [rad/s^2]
Stepp.RPM_max = 300; % max spindle RPM
Stepp.omega_max = Stepp.RPM_max * 2*pi/60; % [rad/s]
Stepp.step_rate_max = Stepp.omega_max*(200*Stepp.microstep_divisor)/(2*pi);

% resonant frequency
Stepp.k_spring = Stepp.Nr*Stepp.Kt*Stepp.I_rated; % spring constant [Nm/rad]
Stepp.omega_res = sqrt(Stepp.k_spring / Stepp.J); % resonance speed [rad/s]
Stepp.f_res = Stepp.omega_res / (2*pi); % resonance freq [Hz]
% Corresponding shaft speed
Stepp.RPM_res  = Stepp.f_res*60 / Stepp.step_per_rev;
Stepp.omega_res_shaft = Stepp.RPM_res * 2*pi/60;

%% Spool motor - Pololu 4887

%  Motor (bare, pre-gearbox) 
DC.V_rated = 12; % [V]
DC.RPM_noload = 5600; % [RPM] (motor shaft, no-load at 12V)
DC.I_noload = 0.020; % [A] (typ 20mA free-run; ±50%)
DC.I_stall = 0.9; % [A] (extrapolated stall at 12V)
DC.T_stall = 0.0014; % [Nm] (0.14 kg·cm motor stall torque)

% Derived electrical parameters
DC.omega_noload = DC.RPM_noload * (2*pi/60);   % 586.4 [rad/s]
% DC.Ke = (DC.V_rated - DC.I_noload*R_mot)/omega_noload; % iterate below
DC.R = DC.V_rated / DC.I_stall; % ~13.3 [Ohm] (back-EMF≈0 at stall)
DC.Ke = (DC.V_rated - DC.I_noload*DC.R) / DC.omega_noload;
DC.Kt = DC.Ke; % ~0.0198 V·s/rad = [Nm/A]

% Inductance — not published; estimate from similar small brushed motors
DC.L = 1.0e-3; % [H] (1 mH — measure or use as tuning param)

% Inertia — not published; estimate
DC.J = 1.5e-7; % [kgm^2] (motor rotor only)
DC.B = 1e-5; % [Nms/rad] (viscous friction, estimate)

%  Gearbox 
DC.gear_ratio = 98.78; % datasheet
DC.eta_gearbox = 0.65; % efficiency (spur gearbox, conservative)

% Gearbox output shaft
DC.RPM_noload_out = DC.RPM_noload / DC.gear_ratio; % ~56.7 [RPM]
DC.omega_out_max = DC.RPM_noload_out * (2*pi/60); % ~5.94 [rad/s]
DC.T_stall_out = DC.T_stall*DC.gear_ratio*DC.eta_gearbox; % ~0.090 [Nm]

% Reflected inertia to output shaft
DC.J_reflected = DC.J * DC.gear_ratio^2; % ~1.46e-3 [kgm^2]

% Encoder
DC.CPR = 48; % counts per rev, quadrature (12×4)
DC.counts_per_rev_out = DC.CPR*DC.gear_ratio*4; % 18,965 counts/output rev

% sanity check
% T_output_needed = Wire.T_set * Spool.R;          % 6.25e-4 Nm
% I_operating     = T_output_needed / (DC.Kt * DC.gear_ratio * DC.eta_gearbox);
% fprintf('=== Pololu 4887 Operating Point ===\n')
% fprintf('Motor Ke/Kt       = %.4f Nm/A\n', DC.Kt)
% fprintf('R_motor           = %.1f Ω\n', DC.R)
% fprintf('Noload output RPM = %.1f RPM\n', DC.RPM_noload_out)
% fprintf('Stall torque out  = %.1f g·cm\n', DC.T_stall_out*1000/9.81*100)
% fprintf('I at T_setpoint   = %.2f mA (%.1f%% of stall)\n', ...
%         I_operating*1000, I_operating/DC.I_stall*100)
% fprintf('Encoder res (out) = %.0f counts/rev\n', DC.counts_per_rev_out)

%% Cam/Follower

Cam.gear_ratio = 1/542; % cam/spindle 
% Load the two groove CSVs
Cam.rise_data   = readmatrix('cam_groove1.csv');    % [x, y, z] columns
Cam.return_data = readmatrix('cam_groove2.csv');

% Extract axial position (z column) and compute cam angle from x,y
% Since x = R*cos(theta), y = R*sin(theta):
Cam.R = 0.03;      % [m] cylinder radius

Cam.theta_rise   = atan2(Cam.rise_data(:,2), Cam.rise_data(:,1)); % [rad]
Cam.theta_return = atan2(Cam.return_data(:,2),...
                         Cam.return_data(:,1)); % [rad]

Cam.z_rise_raw   = Cam.rise_data(:,3)*1e-2;    % convert cm to [m]
Cam.z_return_raw = Cam.return_data(:,3)*1e-2;

% Remove offset
Cam.z_rise = Cam.z_rise_raw - Cam.z_rise_raw(1);
Cam.z_return = Cam.z_return_raw - Cam.z_return_raw(1);

% Flip return
Cam.z_return = max(Cam.z_rise) + Cam.z_return;

% Unwrap angles to make them monotonically increasing
Cam.theta_rise   = unwrap(Cam.theta_rise);
Cam.theta_return = unwrap(Cam.theta_return);

% Shift return angles to start where rise ends (at 2*pi)
Cam.theta_return = Cam.theta_return - Cam.theta_return(1) + ...
                    Cam.theta_rise(end);

% Concatenate into one full cycle table (0 to 4*pi)
Cam.theta_full = [Cam.theta_rise; Cam.theta_return];
Cam.z_full     = [Cam.z_rise; Cam.z_return];

% Remove duplicate theta values
[Cam.theta_full, Cam.unique_idx] = unique(Cam.theta_full, 'stable');
Cam.z_full = Cam.z_full(Cam.unique_idx);

% Verify monotonically increasing (griddedInterpolant requires this)
if any(diff(Cam.theta_full) <= 0)
    % Sort to enforce monotonic order
    [Cam.theta_full, Cam.sort_idx] = sort(Cam.theta_full);
    Cam.z_full = Cam.z_full(Cam.sort_idx);
end
% Derivatives
Cam.dz_dtheta  = gradient(Cam.z_full, Cam.theta_full); % first derivative
Cam.d2z_dtheta2 = gradient(Cam.dz_dtheta, Cam.theta_full); % second derivative

% Build interpolant
% cam_interp = griddedInterpolant(Cam.theta_full, Cam.z_full, 'spline');
theta_table = Cam.theta_full;
z_table = Cam.z_full;

%% Dancer arm

% tension parameters
Dancer.T_min = 0.005; % [N] — minimum tension
Dancer.T_set = 0.025; % [N] — setpoint
Dancer.T_max = 0.150; % [N] — maximum before wire breaks

% arm length — longer = more sensitive but more inertia
Dancer.L_arm = 0.040;    % [m] — 40mm, compact but measurable

% Choose angular range — AS5600 is 12-bit over 360°
% Use 30° total range for good resolution
Dancer.theta_max = 30 * pi/180; % rad at T_max
Dancer.theta_set = Dancer.theta_max * ...
                    (Dancer.T_set/Dancer.T_max); % ~5° at setpoint

% Required spring constant:
% At theta_max: 2 * T_max * L = K_spring * theta_max
Dancer.K_spring = 2 * Dancer.T_max * Dancer.L_arm / Dancer.theta_max;

% Arm mass — lightweight, say 2g aluminium rod
Dancer.m_arm = 0.002; % [kg]
Dancer.J_arm = (1/3) * Dancer.m_arm * Dancer.L_arm^2; % [kgm^2]
% Natural frequency of dancer arm
Dancer.omega_n_arm = sqrt(Dancer.K_spring / Dancer.J_arm);
Dancer.f_n_arm = Dancer.omega_n_arm / (2*pi); % [Hz]

% Damping — want overdamped to avoid oscillation
% Critical damping:
Dancer.B_crit = 2 * sqrt(Dancer.K_spring * Dancer.J_arm);
% Choose zeta = 0.7 (slightly underdamped for fast response)
Dancer.zeta_arm = 0.7;
Dancer.B_arm = 2 * Dancer.zeta_arm * sqrt(Dancer.K_spring * Dancer.J_arm);

% initial condition
Dancer.theta_IC = Wire.T_pre * 2 * Dancer.L_arm / Dancer.K_spring;
% Arm can't deflect past physical stops
Dancer.theta_min = 0; % [rad] — arm hits stop if wire goes slack
Dancer.theta_max = 30*pi/180; % [rad] — arm hits stop at T_max
% AS5600 output — angle to tension
% theta_measured → T_estimated
% T_estimated = K_spring * theta / (2 * L_arm)
Dancer.K_sensor = Dancer.K_spring / (2 * Dancer.L_arm);

%% Speed controller

% Mechanical time constant
Speed.tau_mech = Stepp.J / Stepp.B; % [s]
Speed.rad_per_step = 2*pi / (200 * Stepp.microstep_divisor); % [rad]
% Plant transfer function (step rate → angular velocity)
% G(s) = rad_per_step / (tau_mech * s + 1)
% This is a first order lag — gain = rad_per_step, pole at s = -1/tau_mech
% Rule of thumb: lambda = 2~5 * tau_mech for robustness
Speed.lambda  = 2*Speed.tau_mech; % [s] — desired closed loop response time
Speed.kp = Speed.tau_mech/(Speed.rad_per_step * (Speed.lambda + 0)); % 101.85
Speed.ki = 1/Speed.lambda; % 2.94
Speed.kp = 1.0; 
Speed.ki = 0.5;

% notch filter
Speed.zeta = 0.2; % notch width
% Notch transfer function numerator and denominator
Speed.notchNum = [1, 0, Stepp.omega_res^2];
Speed.notchDen = [1, 2*Speed.zeta*Stepp.omega_res, Stepp.omega_res^2];

% Anti-windup limit
Speed.AW_limit = 0.05 * Stepp.step_rate_max; % 5% correction range


%% Tension controller

Tension.K_plant = (DC.Kt * DC.gear_ratio * DC.eta_gearbox) / ...
                    (DC.R * Spool.R); % [N/V]
Tension.K_integ = Wire.K_spring * DC.Kt * DC.gear_ratio * DC.eta_gearbox / DC.R;
Tension.kp = 90;
Tension.ki = 165.0; % [V/(Ns)]

%% Run Sim

simOut = sim('winder.slx');