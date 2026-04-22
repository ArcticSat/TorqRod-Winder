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

% Belt teeth (for ordering

Belt.T1 = floor(Belt.L1/Belt.Pitch);
Belt.T2 = floor(Belt.L2/Belt.Pitch);
Belt.T3 = floor(Belt.L3/Belt.Pitch);
Belt.T4 = floor(Belt.L4/Belt.Pitch);

%% SVAJ Diagrams

Beta = (Rod.nLoops/gearReduction)*2*pi;% total angle of segment [rad]
h = Rod.woundLength;

camAngles1 = linspace(0,Beta,450); % camshaft angles [rad]
cycloid1 = camSVAJ(camAngles1,Beta,h);

camAngles2 = linspace(Beta,2*Beta,450);
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
R_p = 0.03; % prime cylinder radius [m]
R_f = 3e-3; % follower radius [m]

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

% Write to .csv
writematrix([spline1.x', spline1.y', spline1.z'], 'cam_groove1.csv');
writematrix([spline2.x', spline2.y', spline2.z'], 'cam_groove2.csv');

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