% =========================================================
%  ISTrain 2026 - Chain Transmission Sizing (v3)
% =========================================================
%  Powertrain - Mechanical Systems
%  Authors: Miguel Meninas, Gonçalo Tomé, Miguel Pimentel
%  Last updated: 08/05/2026
% =========================================================
%  Sizes and verifies a two-stage chain drive.
%  Refer to the Notion doc for full methodology and references
% =========================================================

clearvars; close all; clc;

%% ---------- PARAMETERS ----------
T_max   = 12;               % N·m  – motor maximum torque
rpm_max = 4000;             % rpm  – motor maximum speed
g1      = 3.35;             % stage 1 gear ratio
g2      = 3;                % stage 2 gear ratio

% Sprocket teeth
Z1 = 17;  Z2 = round(Z1*g1);   % stage 1: driver / driven
Z3 = 19;  Z4 = round(Z3*g2);   % stage 2: driver / driven

% Correction factors
f_app  = 1.4;
f_tooth = @(Z) 19/Z;

safety_factor = 2;          % TODO: apply Pugsley method

% Geometry limits (mm)
L1_max      = 180;          % max longitudinal dist., motor shaft to bogie centre
L_wheelbase = 650;


%% ---------- POWER ----------
omega_max   = rpm_max * 2*pi/60;    % rad/s
omega_intermediate  = omega_max / g1;
omega_wheel = omega_intermediate / g2;

T_intermediate = T_max * g1;        % intermediate shaft torque
T_output   = T_intermediate * g2;   % total output torque (both wheelsets)
T_wheelset = T_output / 2;          % torque per wheelset

% Selection power
P_stage1 = f_app * f_tooth(Z1) * omega_intermediate  * T_intermediate;
P_stage2 = f_app * f_tooth(Z3) * omega_wheel * T_wheelset;


%% ---------- CHAIN STAGE DEFINITIONS ----------
% Pitch is chosen here, now that the selection power is known.

stage(1).Z_driver     = Z1;        stage(1).Z_driven = Z2;
stage(1).pitch        = 9.525;              % mm  – 06B
stage(1).q_mass       = 0.40;               % kg/m
stage(1).C_est        = 30;                 % pitches
stage(1).omega_driver = omega_max;          % speed of the driver sprocket
stage(1).power        = P_stage1;
stage(1).torque       = T_intermediate;     % output torque of this stage

stage(2).Z_driver     = Z3;        stage(2).Z_driven = Z4;
stage(2).pitch        = 9.525;              % mm  – 06B
stage(2).q_mass       = 0.40;               % kg/m
stage(2).C_est        = [30 50];            % range in pitches
stage(2).omega_driver = omega_intermediate;
stage(2).power        = P_stage2;
stage(2).torque       = T_wheelset;


%% ---------- CHAIN GEOMETRY & TENSION ----------
for i = 1:2
    p   = stage(i).pitch;
    Zd  = stage(i).Z_driver;
    Zn  = stage(i).Z_driven;

    % Pitch diameters (m)
    stage(i).Dp_driver = (p/1000) / sind(180/Zd);
    stage(i).Dp_driven = (p/1000) / sind(180/Zn);

    % Chain length [pitches] and resulting centre distance [mm] for each C_est.
    n_est = numel(stage(i).C_est);
    L_pit = zeros(1, n_est);
    C_out = zeros(1, n_est);
    for k = 1:n_est
        Cp       = stage(i).C_est(k);              % centre distance estimate, in pitches
        Lk       = (Zd+Zn)/2 + 2*Cp + ((Zn-Zd)/(2*pi))^2 / Cp;
        Lk       = ceil(Lk/2) * 2;                 % round up to an even number of links
        L_pit(k) = Lk;
        C_out(k) = (p/8) * ( 2*Lk - Zn - Zd + ...
                    sqrt((2*Lk - Zn - Zd)^2 - (pi/3.88)^2*(Zn-Zd)^2) );
    end
    stage(i).L_pitches = L_pit;
    stage(i).L_mm      = L_pit * p;
    stage(i).C_mm      = C_out;    % scalar for stage 1,  [min max] for stage 2

    % Chain velocity (m/s) and chain pull (N)
    v = stage(i).omega_driver * Zd * p / (2*pi * 1000);
    stage(i).v       = v;
    stage(i).F_chain = stage(i).power / v;

    % Centrifugal tension and total chain tension (N)
    stage(i).F_oc    = stage(i).q_mass * v^2;
    stage(i).F_total = stage(i).F_chain + stage(i).F_oc;
end

% Stage-2 centre distances derived from wheelbase geometry constraint
C1       = stage(1).C_mm;
C2_short = L_wheelbase/2 - (C1 - L1_max);
C2_long  = L_wheelbase  - C2_short;


%% ---------- MATERIALS ----------
S_uts = 420e6;          % Pa – shaft material UTS (AISI 1020)
S_y   = 350e6;          % Pa – shaft material yield stress
tensile_strength_chain = 8900;   % N – min tensile, 06B chain (ISO 606 – verify with vendor)


%% ---------- SHAFT DEFINITIONS ----------
F_motor  = 2 * T_max      / stage(1).Dp_driver;
F_intermediate   = 2 * T_intermediate     / stage(1).Dp_driven;
F_wheel  =     T_wheelset / stage(2).Dp_driven;

shaft(1).length   = 0.50;    shaft(1).support  = 0.25;
shaft(1).diameter = 0.02125;
shaft(1).F        = F_motor;
shaft(1).T        = T_max;

shaft(2).length   = 0.50;    shaft(2).support  = 0.25;
shaft(2).diameter = 0.021;
shaft(2).F        = F_intermediate;
shaft(2).T        = T_intermediate;

shaft(3).length   = 0.50;    shaft(3).support  = 0.125;  % equals shaft(2) sprocket spacing
shaft(3).diameter = 0.03;
shaft(3).F        = F_wheel;
shaft(3).T        = T_wheelset;


%% ---------- SHAFT ANALYSIS ----------
% Keyway stress concentration factors + notch sensitivity (Shigley figs 6.20 & 6.21)
% S_uts = 420 MPa, keyway root radius ≈ 0.2 mm  →  q ≈ 0.5
Kt_b = 2.2;   Kt_t = 3.0;
q_notch = 0.5;
Kf_b = q_notch*(Kt_b + 1) - 1;
Kf_t = q_notch*(Kt_t + 1) - 1;

for i = 1:3
    d_m  = shaft(i).diameter;       % m
    d_mm = d_m * 1000;

    % -- Endurance limit via Marin factors (machined surface finish) --
    k_surf = 4.51 * (S_uts/1e6)^(-0.265);
    if d_mm <= 51
        k_size = 1.24 * d_mm^(-0.107);
    else
        k_size = 1.51 * d_mm^(-0.157);
    end
    shaft(i).S_e = 0.5 * S_uts * k_surf * k_size;

    % -- Support reactions (simply-supported beam, one concentrated load) --
    R1 = shaft(i).F * (shaft(i).support / shaft(i).length);
    R2 = shaft(i).F - R1;

    shaft(i).V = max(R1, R2);               % N   – max shear force
    shaft(i).M = R1 * shaft(i).length;      % N·m – bending moment at critical section

    % -- Nominal stresses at outer fibre (bending + torsion) --
    d3      = d_m^3;
    sigma_b = 32 * shaft(i).M / (pi * d3);  % bending stress
    tau_t   = 16 * shaft(i).T / (pi * d3);  % torsional shear stress

    % -- Static assessment (von Mises) --
    shaft(i).vm         = sqrt(sigma_b^2 + 3*tau_t^2);
    shaft(i).static_SF  = S_y / shaft(i).vm;
    shaft(i).static_OK  = shaft(i).static_SF >= safety_factor;

    % -- Fatigue assessment (modified Goodman) --
    %    Rotating bending → amplitude = sigma_b  |  Torsion → mean = tau_t
    shaft(i).fatigue_SF = 1 / (Kf_b*sigma_b/shaft(i).S_e + Kf_t*tau_t/S_uts);
    shaft(i).fatigue_OK = shaft(i).fatigue_SF >= safety_factor;
end


%% ---------- PRINT RESULTS ----------
fprintf("SELECTION POWER\n")
fprintf('   Stage 1 = %.2f kW\n', P_stage1/1000);
fprintf('   Stage 2 = %.2f kW\n\n', P_stage2/1000);


fprintf('CHAIN STAGES\n')
for i = 1:2
    s = stage(i);
    fprintf('  Stage %d  (pitch = %.3f mm)\n', i, s.pitch)
    fprintf('    Driver (%dT): Dp = %.4f mm  |  Driven (%dT): Dp = %.4f mm\n', ...
            s.Z_driver, s.Dp_driver*1000, s.Z_driven, s.Dp_driven*1000)
    if isscalar(s.C_mm)
        fprintf('    Centre distance: %.1f mm\n\n', ...
                s.C_mm)
    else
        fprintf('    Feasible centre distance: %.1f – %.1f mm\n', s.C_mm(1), s.C_mm(2))
        fprintf('    Geometric C:  short = %.1f mm,  long = %.1f mm\n\n', C2_short, C2_long)
    end
end

fprintf('SHAFTS\n')
for i = 1:3
    sh = shaft(i);
    fprintf('   Shaft %d:  static SF = %.2f [%s]  |  fatigue SF = %.2f [%s]\n', i, ...
            sh.static_SF,  pass_fail(sh.static_OK), ...
            sh.fatigue_SF, pass_fail(sh.fatigue_OK))
end

fprintf("\n")

%% ---------- LOCAL FUNCTIONS ----------
function s = pass_fail(flag)
    if flag; s = 'OK'; else; s = 'FAIL'; end
end
