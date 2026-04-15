% chain_06B_sizing.m
% 2-stage chain sizing using 06B (p = 9.525 mm). Both sprung idlers placed on stage-2.
clearvars; close all; clc;


%% ---------- PARAMETERS ----------
T_max = 12;                         % N·m - motor max torque
speed_max = 4000;                   % max rpm (at the motor)
g1 = 3.35;                          % stage 1 gear ratio
g2 = 3;                             % stage 2 gear ratio

% number of sprocket teeth
Z1 = 17;                            % stage 1 driver
Z2 = Z1*g1;                         %         driven
Z3 = 19;                            % stage 2 driver
Z4 = Z3*g2;                         %         driven
                                    
% correction factors
f1 = 1.3;                           % application factor, see renold pg 104
f2_stage1 = 19/Z1;                  % tooth factor for each stage
f2_stage2 = 19/Z3;

safety_factor = 1.5;


%% ---------- POWER ----------
speed_max = (speed_max*2*pi)/60;    % convert to rad/s

% STAGE 1
speed_jackshaft = speed_max/g1;
T_stage1 = T_max*g1;
power_stage1 = f1*f2_stage1*speed_jackshaft*T_max;

% STAGE 2
speed_wheelset = speed_jackshaft/g2;
% note: total output torque and power are 2x what is given to each wheelset
T_output = T_stage1*g2;
T_per_wheelset = T_output/2;
power_output = f1*f2_stage2*speed_wheelset*T_stage1;
power_per_wheelset = power_output/2;


%% ---------- PITCH ----------
% choose pitch from power and speed needed
pitch = 9.525;                      % mm - 06B pitch
% center distance in pitches
C_est = 40*pitch;                   % estimate, actual C calculated later

% chain length
L_stage1 =  (Z1+Z2)/2 + (2*C_est)/pitch;
L_stage2 =  (Z3+Z4)/2 + (2*C_est)/pitch;

% Actual center distance
C1 = (pitch/8)*(2*L_stage1-Z2-Z1+sqrt((2*L_stage1-Z2-Z1)^2 - ((pi/3.88)*(Z2-Z1)^2)));
C2 = (pitch/8)*(2*L_stage2-Z4-Z3+sqrt((2*L_stage2-Z4-Z3)^2 - ((pi/3.88)*(Z4-Z3)^2)));


%% ---------- TRANSMISSION EQUATIONS ----------
% stage 1
v_stage1 = (speed_max*Z1*pitch)/60000;              % m/s, chain linear velocity
chainpull_stage1 = (1000*power_stage1)/v_stage1;    % N

% stage 1
v_stage2 = (speed_jackshaft*Z3*pitch)/60000;
chainpull_stage2 = (1000*power_per_wheelset)/v_stage2;


%% ---------- MATERIALS & SAFETY ----------
% Approximate values for 06B chain (verify with manufacturer)
tensile_strength_chain = 8900;   % in N, (minimum tensile strength for 06B chain according to ISO 606 - CHECK vendor)


%% ---------- IDLERS ----------
% Idler configuration: both idlers on stage-2
num_idlers_stage1 = 0;   % no sprung idlers on stage-1
num_idlers_stage2 = 2;   % both sprung idlers on stage-2 (split preload)

% Idler defaults
idler_travel = 25;     % mm - available travel for each idler
idler_preload_ratio = 0.08; % fraction of working tension used as total preload for stage-2

% Minimum per-stage total preload (keeps tiny systems from choosing too small springs)
min_total_preload_stage1 = 0;   % N (stage1 no idlers)
min_total_preload_stage2 = 80;  % N (total across all idlers on stage2)

dynamic_factor = 1.3;
make_plots = true;


%% ---------- SHAFT DESIGN PARAMETERS ----------
% shaft dimensions (in m)
% motor shaft
shaft1_length = 0.10;
shaft1_support = 0.5;       % distance from sprocket to one of the bearings
shaft1_diameter = 0.01;     % critical cross section diameter
% jackshaft
shaft2_length = 0.10;
shaft2_support = 0.5;
shaft2_spacing = 0.25;   % d between s-1 and s-2 sprockets, assume symmetry
shaft2_diameter = 0.01;
% wheelset
shaft3_length = 0.10;
shaft3_support = 0.5;
shaft3_diameter = 0.01;

% material properties
S_uts = 420e6;              % UTS in Pa for shaft material (AISI 1020)
S_y = 350e6;                % yield stress
% endurance stress for each shaft
[S_e1, S_e2, S_e3] = endurance_stress(S_uts, [d1,d2,d3]);


%% ---------- SHAFT STRESSES & SUPPORT REACTIONS ----------
% pitch diameters
Dp1 = (p/1000) / sind(180/Z1);      % in m
Dp2 = (p/1000) / sind(180/Z2);
Dp3 = (p/1000) / sind(180/Z3);
Dp4 = (p/1000) / sind(180/Z4);

% Applied forces (in NEWTON)
F1 = 2*T_max/Dp1;          % F of s-1 driving sprocket on motor shaft
F2 = 2*T_stage1/Dp2;       % F of s-1 driven sprocket on jackshaft
% F3 = T_stage1/Dp3;         % F of *EACH* s-2 driving sprocket on jackshaft    I DONT THINK THIS IS NEEDED
F4 = T_per_wheelset/Dp4;   % F of s-2 driven sprocket on each wheelset

% Reactions on the bearings
shaft1_reaction1 = F1*(shaft1_support/shaft1_length);   % motor shaft
shaft1_reaction2 = F1 - shaft1_reaction1;

shaft2_reaction1 = F2*(shaft2_support/shaft2_length);   % jackshaft
shaft2_reaction2 = F2 - shaft2_reaction1;

shaft3_reaction1 = F4*(shaft3_support/shaft3_length);   % wheelset
shaft3_reaction2 = F4 - shaft3_reaction1;

% Maximum shear force and bending moment
shaft1_V = max([shaft1_reaction1 shaft1_reaction2]);    % N
shaft1_M = shaft1_reaction1*shaft1_length;              % N.m

shaft2_V = max([shaft2_reaction1 shaft2_reaction2]);
shaft2_M = shaft2_reaction1*shaft2_length;

shaft3_V = max([shaft3_reaction1 shaft3_reaction2]);
shaft3_M = shaft3_reaction1*shaft3_length;

% Stresses AT OUTER FIBER
% (assuming bending + torsion is critical rather than shear force)
shaft1_bending = (32*shaft1_M)/(pi*shaft1_diameter^3);
shaft1_shear = (16*T_max)/(pi*shaft1_diameter^3);

shaft2_bending = (32*shaft2_M)/(pi*shaft2_diameter^3);
shaft2_shear = (16*T_stage1)/(pi*shaft2_diameter^3);

shaft3_bending = (32*shaft3_M)/(pi*shaft3_diameter^3);
shaft3_shear = (16*T_per_wheelset)/(pi*shaft3_diameter^3);


%% ---------- STATIC SAFETY ----------
% von Mises stresses
shaft1_vm = sqrt(shaft1_bending^2 + 3*shaft1_shear^2);
shaft2_vm = sqrt(shaft2_bending^2 + 3*shaft2_shear^2);
shaft3_vm = sqrt(shaft3_bending^2 + 3*shaft3_shear^2);

shaft1_static_safety = S_y/shaft1_vm;
shaft2_static_safety = S_y/shaft2_vm;
shaft3_static_safety = S_y/shaft3_vm;

static_isOK = [shaft1_static_safety shaft2_static_safety shaft3_static_safety] >= safety_factor;


%% ---------- FATIGUE SAFETY (for infinite life) ----------
% we have torsion + rotating bending, this means that for fatigue:
% shear stress = mean stress, bending stress = stress amplitude

% Keyway static stress concentration coefficents, Kt
Kt_torsion = 3.0;
Kt_bending = 2.2;
% notch sensitivity (for S_uts=420 MPa, and keyway radius of 0.2 mm)
% values taken from figures 6.20 and 6.21 in Shigley
notch_sensitivity = 0.5;
% Fatigue stress concentration coefficents Kf = q*(Kt+1)-1
Kf_bending = notch_sensitivity*(Kt_bending+1)-1;
Kf_torsion = notch_sensitivity*(Kt_torsion+1)-1;

% corrected stresses
shaft1_bending_K = shaft1_bending*Kf_bending;
shaft2_bending_K = shaft2_bending*Kf_bending;
shaft3_bending_K = shaft2_bending*Kf_bending;
shaft1_shear_K = shaft1_shear*Kf_torsion;
shaft2_shear_K = shaft2_shear*Kf_torsion;
shaft3_shear_K = shaft2_shear*Kf_torsion;

% Check modified Goodman failure criterion
shaft1_fatigue_safety = 1/(shaft1_bending_K/S_e1)+(shaft1_shear_K/S_uts);
shaft2_fatigue_safety = 1/(shaft2_bending_K/S_e2)+(shaft2_shear_K/S_uts);
shaft3_fatigue_safety = 1/(shaft3_bending_K/S_e3)+(shaft3_shear_K/S_uts);

fatigue_isOK = [shaft1_fatigue_safety shaft2_fatigue_safety shaft3_fatigue_safety] >= safety_factor;


%% ---------- CALCULATIONS (old) ----------
% Conservative forces (driver vs driven) + dynamic factor
F1_driver = T_max / r1_driver;
F1_driven = T_int / r1_driven;
F_stage1 = max(F1_driver, F1_driven) * dynamic_factor;

F2_driver = T_int / r2_driver;
F2_driven = T_out / r2_driven;
F_stage2 = max(F2_driver, F2_driven) * dynamic_factor;

% Capacity and margins
tensile_workable = tensile_strength_chain / safety_factor_chain;
margin1_pct = (tensile_workable - F_stage1) / tensile_workable * 100;
margin2_pct = (tensile_workable - F_stage2) / tensile_workable * 100;

% Shaft diameters (approx torsion)
d_motor_mm = ( (16*T_max/(pi*tau_allow))^(1/3) )*1000;
d_intermediate_mm = ( (16*T_int/(pi*tau_allow))^(1/3) )*1000;
d_output_mm = ( (16*T_out/(pi*tau_allow))^(1/3) )*1000;

d_motor_suggest = pick_std(d_motor_mm);
d_intermediate_suggest = pick_std(d_intermediate_mm);
d_output_suggest = pick_std(d_output_mm);


% ----------------- IDLER SIZING (both idlers on stage-2) -----------------
% Stage-1: no sprung idlers
total_preload_stage1 = min_total_preload_stage1;   % 0 N
per_idler_preload_stage1 = 0;
k_idler_stage1 = 0;

% Stage-2: total preload (distributed across num_idlers_stage2)
if num_idlers_stage2 <= 0
    error('num_idlers_stage2 must be >= 1 when placing idlers on stage-2.');
end

total_preload_stage2 = max(min_total_preload_stage2, idler_preload_ratio * F_stage2);   % N
per_idler_preload_stage2 = total_preload_stage2 / num_idlers_stage2;                    % each idler supplies this preload
k_idler_stage2 = per_idler_preload_stage2 / idler_travel;                               % N/mm per idler


%% ---------- DISPLAY ----------
fprintf('--- 06B solution (both idlers on stage-2) ---\n');
fprintf('Total ratio = %.3g (%.3g x %.3g)\n', ratio_total, ratio1, ratio2);
fprintf('Pitch diameters (mm): %dT=%.1f, %dT=%.1f, %dT=%.1f, %dT=%.1f\n', ...
    N_stage1_driver, Dp1_driver, N_stage1_driven, Dp1_driven, N_stage2_driver, Dp2_driver, N_stage2_driven, Dp2_driven);
fprintf('Torques (N·m): motor=%.2f, interm=%.2f, out=%.2f\n', T_max, T_int, T_out);
fprintf('Design chain forces (dyn x%.2f): stage1=%.1f N, stage2=%.1f N\n', dynamic_factor, F_stage1, F_stage2);
fprintf('Chain workable (SF %.2f)= %.1f N => margins: s1=%.1f%% s2=%.1f%%\n', safety_factor_chain, tensile_workable, margin1_pct, margin2_pct);
fprintf('Chain lengths: stage1 %d links (%.1f mm), stage2 %d links (%.1f mm)\n', Lp1, L1_mm, Lp2, L2_mm);

fprintf('\nIdler configuration:\n');
fprintf('  Stage1: num_idlers = %d, total_preload = %.1f N, per-idler = %.1f N, k_per_idler = %.2f N/mm\n', ...
    num_idlers_stage1, total_preload_stage1, per_idler_preload_stage1, k_idler_stage1);
fprintf('  Stage2: num_idlers = %d, total_preload = %.1f N, per-idler = %.1f N, k_per_idler = %.2f N/mm\n', ...
    num_idlers_stage2, total_preload_stage2, per_idler_preload_stage2, k_idler_stage2);

fprintf('\nSuggested shaft diameters (mm): motor=%d, interm=%d, out=%d\n', d_motor_suggest, d_intermediate_suggest, d_output_suggest);

%% plot (optional)
if make_plots
    figure; Tm_vec = linspace(1,30,200);
    plot(Tm_vec./r1_driver * dynamic_factor); hold on;
    plot((Tm_vec*ratio1*eta1)./r2_driver * dynamic_factor);
    yline(tensile_workable,'k--'); legend('Stage1','Stage2','Chain workable');
    xlabel('Motor torque (N·m)'); ylabel('Chain force (N)'); grid on;
end

%% helpers
function s = pick_std(d_mm)
    stds = [6 8 10 12 14 15 16 18 20 22 24 25 28 30 32 35 38 40 45 50];
    idx = find(stds >= ceil(d_mm), 1, 'first');
    if isempty(idx), s = ceil(d_mm); else s = stds(idx); end
end

function S_e = endurance_stress(S_uts, d)
    % theoretical endurance stress
    S_e_theory = 0.5*S_uts;
    
    % convert diameters to mm
    d = d*1000;

    % Marin correction factors (check Shigley/PCMec slides)
        % surface finish correction factor
        k_surface = 4.51*S_uts^(-0.265);        % machined finish
        % size factor
        k_size = zeros(1,3);
        for i = d
            if i <= 51
                k_size = 1.24*i^(-0.107);
            else
                k_size = 1.51*i^(-0.157);
            end
        end

    % actual endurance stress for each shaft
    S_e = [S_e_theory*k_surface*k_size(1) S_e_theory*k_surface*k_size(2) S_e_theory*k_surface*k_size(3)];
end