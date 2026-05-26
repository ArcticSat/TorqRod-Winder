% TorqRodWinder
% Calculations for winder & cam/follower design
clear all
close all

%% Magnetorquer dimensions

Rod.mountLength = 7.0e-3; % [m]
Rod.woundLength = 76e-3; % [m]
Rod.totL = 2*Rod.mountLength + Rod.woundLength; % [m]
Rod.Nwind = 2460;
Rod.rad = 4.5e-3; % [m]
Rod.wireRad = 0.14e-3/2; % 36 AWG [m]
Rod.nLoops = floor(Rod.woundLength/(2*Rod.wireRad));
Rod.nPasses = Rod.Nwind/Rod.nLoops;

%% Belt Drive Design

% Three stage GT2 belt reduction
Belt.Pitch = 2; % [mm]
Belt.T_18T = 18;
Belt.T_20T = 20;
Belt.T_48T = 48;
Belt.T_72T = 72;

% Stage 1: 18T → 72T = 4:1
% Stage 2: 18T → 72T = 4:1
% Stage 3: 20T → 48T = 2.4:1
% Stage 4: 20T → 48T = 2.4:1
gearReduction = 1/((Belt.T_18T/Belt.T_72T)^2*(Belt.T_20T/Belt.T_48T)^2)

% Pulley pitch diameters (GT2, 2mm pitch):
Belt.PD = @(T,Pitch) (T * Pitch) / pi;
Belt.PD_18T = Belt.PD(Belt.T_18T,Belt.Pitch); % [mm]
Belt.PD_20T = Belt.PD(Belt.T_20T,Belt.Pitch); % [mm]
Belt.PD_48T = Belt.PD(Belt.T_48T,Belt.Pitch); % [mm]
Belt.PD_72T = Belt.PD(Belt.T_72T,Belt.Pitch); % [mm]

% Pitch diameter = teeth * pitch / pi
Belt.T_hgt = 0.75; % [mm]
Belt.OD = @(T) T + Belt.T_hgt*2;
Belt.OD_18T = Belt.OD(Belt.PD_18T); % [mm]
Belt.OD_20T = Belt.OD(Belt.PD_20T); % [mm]
Belt.OD_48T = Belt.OD(Belt.PD_48T); % [mm]
Belt.OD_72T = Belt.OD(Belt.PD_72T); % [mm]

% Center distances — want compact but enough wrap angle
% and has to fit realistically
% and can't be shaking
% C > (D2 - D1) / 2  for adequate wrap
Belt.wrap = @(PD_2,PD_1) (PD_2 - PD_1) / 2;
% Minimum C = (OD_large1 + OD_large2)/2 + clearance
Belt.clearance = 5; % [mm]
Belt.minDist = @(OD_1,OD_2) ceil((OD_1 + OD_2)/2 + Belt.clearance);
% C < 3 * (D1 + D2) to avoid belt vibration
Belt.vibe = @(PD_2,PD_1) (PD_2 + PD_1) * 3;
% Stage 1: 18T → 72T, S1(18T) → S2(72T+18T)
Belt.C1_wrap = Belt.wrap(Belt.PD_72T,Belt.PD_18T); % [mm]
Belt.C1_clearance = Belt.minDist(Belt.OD_18T,Belt.OD_72T); % [mm]
Belt.C1_vibe = Belt.vibe(Belt.PD_72T, Belt.PD_18T); % [mm]
Belt.C1min = max(Belt.C1_wrap,Belt.C1_clearance);
Belt.C1 = 53; % [mm]
% Stage 2: 18T → 72T 
Belt.C2_wrap = Belt.wrap(Belt.PD_72T, Belt.PD_18T); % [mm]
Belt.C2_clearance = Belt.minDist(Belt.OD_18T,Belt.OD_72T); % [mm]
Belt.C2_vibe = Belt.vibe(Belt.PD_72T, Belt.PD_18T); % [mm]
Belt.C2min = max(Belt.C2_wrap,Belt.C2_clearance);
Belt.C2 = 53; % [mm]
% Stage 3: 20T → 48T
Belt.C3_wrap = Belt.wrap(Belt.PD_48T, Belt.PD_20T); % [mm]
Belt.C3_clearance = Belt.minDist(Belt.OD_20T,Belt.OD_48T); % [mm]
Belt.C3_vibe = Belt.vibe(Belt.PD_48T, Belt.PD_20T); % [mm]
Belt.C3min = max(Belt.C3_wrap,Belt.C3_clearance);
Belt.C3 = 30; % [mm]
% Stage 4: 20T → 48T
Belt.C4_wrap = Belt.wrap(Belt.PD_48T, Belt.PD_20T); % [mm]
Belt.C4_clearance = Belt.minDist(Belt.OD_20T,Belt.OD_48T); % [mm]
Belt.C4_vibe = Belt.vibe(Belt.PD_48T, Belt.PD_20T); % [mm]
Belt.C4min = max(Belt.C4_wrap,Belt.C4_clearance);
Belt.C4 = 30; % [mm]

% GT2 belt length formula:
% L = 2*C + pi*(PD2+PD1)/2 + (PD2-PD1)^2/(4*C)
% where C = center distance, D1/D2 = pulley pitch diameters
% Belt lengths
Belt.Length = @(C,PD_1,PD_2) 2*C + pi*(PD_2+PD_1)/2 + (PD_2-PD_1)^2/(4*C);
Belt.L1 = Belt.Length(Belt.C1,Belt.PD_18T,Belt.PD_72T); % [mm]
Belt.L2 = Belt.Length(Belt.C2,Belt.PD_18T,Belt.PD_72T); % [mm]
Belt.L3 = Belt.Length(Belt.C3,Belt.PD_20T,Belt.PD_48T); % [mm]
Belt.L4 = Belt.Length(Belt.C4,Belt.PD_20T,Belt.PD_48T); % [mm]

% Belt teeth (for ordering)

Belt.T1 = floor(Belt.L1/Belt.Pitch);
Belt.T2 = floor(Belt.L2/Belt.Pitch);
Belt.T3 = floor(Belt.L3/Belt.Pitch);
Belt.T4 = floor(Belt.L4/Belt.Pitch);

%% SVAJ Diagrams
numPoints = 250; % number of points to generate
Beta = (Rod.nLoops/gearReduction)*2*pi; % total angle of segment [rad]
h = Rod.woundLength;

camAngles1 = linspace(0,Beta,numPoints); % camshaft angles [rad]
cycloid1 = camSVAJ(camAngles1,Beta,h);

camAngles2 = linspace(Beta,2*Beta,numPoints);
cycloid2 = camSVAJ(camAngles1,Beta,h);
cycloid2.s = fliplr(cycloid2.s);
cycloid2.v = fliplr(cycloid2.v);
cycloid2.a = fliplr(cycloid2.a);
cycloid2.j = fliplr(cycloid2.j);

camAnglesFull = [camAngles1 camAngles2];
cycloidFull.s = [cycloid1.s cycloid2.s];
cycloidFull.v = [cycloid1.v cycloid2.v];
cycloidFull.a = [cycloid1.a cycloid2.a];
cycloidFull.j = [cycloid1.j cycloid2.j];

%% SVAJ plots
figure();
plot(camAnglesFull,cycloidFull.j,'.');
title('j');
figure();
plot(camAnglesFull,cycloidFull.a,'.');
title('a');
figure();
plot(camAnglesFull,cycloidFull.v,'.');
title('v');
figure();
plot(camAnglesFull,cycloidFull.s,'.');
title('s');


%% Barrel Cam Design

omega = 1; % angular velocity [rad/s]
R_p = 0.01; % prime cylinder radius [m]
R_f = 1.5e-3; % follower radius [m]

pressureAngle1 = atan(cycloid1.v/omega/R_p); % pressure angle [rad]
radCurvature1 = -((1+(cycloid1.v/omega/R_p).^2).^(3/2)) ./ ...
                    (cycloid1.a./(omega^2*R_p^2));

% follower contact point in cylindrical coords - rise
z_s1 = cycloid1.s + sign(omega)*R_f*cos(pressureAngle1);
delta_s1 = camAngles1 - sign(omega)*R_f/R_p*sin(pressureAngle1);

pressureAngle2 = atan(cycloid2.v/omega/R_p); % pressure angle

% follower contact point in cylindrical coords - return
z_s2 = fliplr(z_s1);
delta_s2 = camAngles2 - omega*R_f/R_p*sin(pressureAngle2);

% z_s_interp = interp1(delta_s2, z_s2, delta_s1, 'linear','extrap');
% min_sep = min(abs(z_s1 - z_s2))

spline1.x = R_p*cos(delta_s1);
spline1.y = R_p*sin(delta_s1);
spline1.z  = z_s1;

spline2.x = R_p*cos(delta_s2);
spline2.y = R_p*sin(delta_s2);
spline2.z  = z_s2;

pressureAngle = [pressureAngle1 pressureAngle2];

%% Post processing

% convert to cm to appease Fusion
spline1.x = spline1.x.*100;
spline1.y = spline1.y.*100;
spline1.z = spline1.z.*100;

spline2.x = spline2.x.*100;
spline2.y = spline2.y.*100;
spline2.z = spline2.z.*100;

spline.x = [spline1.x, spline2.x(2:end)];
spline.y = [spline1.y, spline2.y(2:end)];
spline.z = [spline1.z, spline2.z(2:end)];

% Write to .csv
writematrix([spline1.x', spline1.y', spline1.z'], 'cam_groove1.csv');
writematrix([spline2.x', spline2.y', spline2.z'], 'cam_groove2.csv');
writematrix([spline.x', spline.y', spline.z'], 'cam_groove_full.csv');

%% Geometric Validation
path = [spline.x', spline.y', spline.z'].*10; % convert to mm;
% Check for NaN/Inf
if any(isnan(path(:))) || any(isinf(path(:)))
    error('Path contains NaN or Inf values!');
end

% Verify cylindrical constraint
R_actual = sqrt(path(:,1).^2 + path(:,2).^2);
radius_error = max(abs(R_actual - R_p*1000));
fprintf('Max radial deviation: %.4f mm (target: < 0.01 mm)\n', radius_error);
if radius_error > 0.01
    warning('Points deviate from nominal cylinder - check CSV generation');
end

% Identify reversal indices (z ≈ 0 and z ≈ 76)
tol_z = 0.1;  % mm tolerance
idx_start = find(abs(path(:,3) - 0) < tol_z, 1, 'first');
idx_mid1   = find(abs(path(:,3) - Rod.woundLength*1000) < tol_z, 1, 'first');  % reversal
idx_mid2   = find(abs(path(:,3) - Rod.woundLength*1000) < tol_z, 1, 'last');  % reversal
idx_mid = mean([idx_mid1, idx_mid2]);
idx_end   = find(abs(path(:,3) - 0) < tol_z, 1, 'last');

% Finite-difference derivatives (central difference for interior points)
dz_dtheta = gradient(path(:,3));  % Approximate dz/dtheta
d2z_dtheta2 = gradient(dz_dtheta);

% Check reversal continuity (assume its the same at both sides)
fprintf('\n=== Reversal Continuity Check (z=76mm) ===\n');
fprintf('Position: z = %.6f mm (target: 76.000)\n', path(idx_mid,3));
fprintf('Velocity: dz/dθ = %.4e mm/rad (target: ~0)\n', dz_dtheta(idx_mid));
fprintf('Acceleration: d²z/dθ² = %.4e mm/rad² (target: ~0)\n', d2z_dtheta2(idx_mid));

%% 3D Curvature Validation

% Compute first and second derivatives of full 3D path
dx = gradient(path(:,1)); dy = gradient(path(:,2)); dz = gradient(path(:,3));
d2x = gradient(dx); d2y = gradient(dy); d2z = gradient(dz);

% 3D curvature formula: κ = |r' × r''| / |r'|^3
cross_x = dy.*d2z - dz.*d2y;
cross_y = dz.*d2x - dx.*d2z;
cross_z = dx.*d2y - dy.*d2x;
numerator = sqrt(cross_x.^2 + cross_y.^2 + cross_z.^2);
denominator = (dx.^2 + dy.^2 + dz.^2).^(3/2);

kappa = numerator ./ denominator;  % Curvature [1/mm]
R_curvature = 1 ./ kappa;           % Radius of curvature [mm]

% Find minimum curvature radius (most critical point)
[min_R, idx_min] = min(R_curvature);
fprintf('\n=== Curvature Analysis ===\n');
fprintf('Min curvature radius: %.3f mm at point %d (z=%.2f mm)\n', ...
    min_R, idx_min, path(idx_min,3));

% Compare to groove geometry requirement
groove_half_width = 1.75;  % [mm] (for 3.5mm wide V-groove)
% Conservative rule: R_curvature > 3-5× groove half-width for clean sweep
R_min_safe = 3 * groove_half_width;
fprintf('Safe minimum for 3.5mm V-groove: ~%.2f mm\n', R_min_safe);

if min_R < R_min_safe
    warning('Curvature too tight! May cause self-intersection in Fusion.');
    fprintf('  → Try: (1) Increase R_cam, (2) Reduce groove width, or (3) Add tip fillet\n');
end

% Helix angle
helix_angle_rad = atan2(abs(dz), R_p*1000);  % dz is per-point, approx dz/dθ
helix_angle_deg = helix_angle_rad * 180/pi;

fprintf('\n=== Helix Angle Check ===\n');
fprintf('Max helix angle: %.2f° (recommended: < 20°)\n', max(helix_angle_deg));

if max(helix_angle_deg) > 25
    warning('High helix angle may cause sweep instability or follower binding');
end

% Plot curvature vs. axial position
figure('Name','Cam Path Curvature');
plot(path(:,3), R_curvature, 'b-', 'LineWidth',1);
yline(R_min_safe, 'r--', 'Safe minimum');
xline(76, 'k:', 'Reversal');
xlabel('Axial position z (mm)'); ylabel('Path curvature radius (mm)');
title('Check: Curvature radius vs. groove width requirement');
grid on;

%% Self intersection check
% Parameters for V-groove (90°, 3.5mm wide @ 1.75mm depth)
groove_angle = 90;  % degrees
groove_width = groove_half_width*2; % mm at surface
groove_depth = groove_half_width;% mm

% For each point, compute the "envelope" of the groove in the normal plane
% Simplified check: sample points along the path and check 3D distance
sample_stride = 2;  % Check every Nth point to save time
check_points = 1:sample_stride:numPoints*2;

fprintf('\n=== Groove Envelope Proximity Check ===\n');
min_sep = inf;
collision_found = false;
close_count = 0;

for i = 1:length(check_points)
    idx_i = check_points(i);
    p_i = path(idx_i,:);
    
    for j = i+1:length(check_points)
        idx_j = check_points(j);
        p_j = path(idx_j,:);
        
        % 3D distance between centerline points
        d_center = norm(p_i - p_j);
        
        % Approximate minimum separation between groove envelopes:
        % If centerlines are closer than groove_width, envelopes likely overlap
        if d_center < groove_width * 0.9  % 10% safety margin
            % More detailed check: project onto local normal plane
            % (Simplified: just flag for manual inspection)
            fprintf('  ⚠ Close approach: points %d & %d, d=%.3f mm, z=%.1f/%.1f mm\n', ...
                idx_i, idx_j, d_center, path(idx_i,3), path(idx_j,3));
            min_sep = min(min_sep, d_center);
            if d_center < groove_width * 0.5
                collision_found = true;
                close_count = close_count + 1;
            end
        end
    end
end

if collision_found
    error('Groove envelopes likely intersect at %d points! Reduce groove width or increase R_cam.',...
        close_count);
else
    fprintf('Min centerline separation: %.3f mm (groove width: %.2f mm)\n', ...
        min_sep, groove_width);
end

%% Plots
figure();
plot(camAnglesFull,rad2deg(pressureAngle),'.');
title('pressure angle');
% figure();
% plot(camAnglesFull,z_s,'.');
% title('z_s');
% figure();
% plot(camAnglesFull,delta_s,'.');
% title('delta_s');
figure();
plot3(spline1.x,spline1.y,spline1.z,'.',spline2.x,spline2.y,spline2.z,'.'); 
axis equal
figure();
plot3(spline.x,spline.y,spline.z,'.'); 
axis equal