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


%% ---------- POWER ----------
speed_max = (speed_max*2*pi)/60;    % convert to rad/s for calculations
% STAGE 1
speed_jackshaft = speed_max/g1;
T_stage1 = T_max*g1;
power_stage1 = f1*f2_stage1*speed_jackshaft*T_max;

% STAGE 2
speed_wheelset = speed_jackshaft/g2;
power_output = f1*f2_stage2*speed_wheelset*T_stage1;      % total power output of the vehicle
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
v_stage1 = (speed_max*Z1*pitch)/60000;              % chain linear velocity
chainpull_stage1 = (1000*power_stage1)/v_stage1;

% stage 1
v_stage2 = (speed_jackshaft*Z3*pitch)/60000;
chainpull_stage2 = (1000*power_per_wheelset)/v_stage2;






%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE ON CURRENT STATE OF THE FILE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% "CALCULATIONS" SECTION BELOW SHOULD BE REDONE
% EVERYTHING ELSE SHOULD BE REVISED

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%









%% ---------- MATERIALS & SAFETY ----------
% Approximate values for 06B chain (verify with manufacturer)
tensile_strength_chain = 10000;  % in N, (approx ultimate for 06B single strand - CHECK vendor)
tau_allow = 50e6;                % Pa Moderate austenitic steel


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


%% ---------- CALCULATIONS ----------
p_m = pitch/1000;
C1_p = C1_mm / pitch;
C2_p = C2_mm / pitch;

ratio1 = N_stage1_driven / N_stage1_driver;
ratio2 = N_stage2_driven / N_stage2_driver;
ratio_total = ratio1 * ratio2;

% Pitch diameters
Dp1_driver = pitch / sind(180/N_stage1_driver);
Dp1_driven = pitch / sind(180/N_stage1_driven);
Dp2_driver = pitch / sind(180/N_stage2_driver);
Dp2_driven = pitch / sind(180/N_stage2_driven);

% Radii (m)
r1_driver = (Dp1_driver/1000)/2;
r1_driven = (Dp1_driven/1000)/2;
r2_driver = (Dp2_driver/1000)/2;
r2_driven = (Dp2_driven/1000)/2;

% Torques
T_int = T_max * ratio1 * eta1;
T_out = T_int * ratio2 * eta2;

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

% Chain lengths
Lp1 = chain_links_from_centres(N_stage1_driver, N_stage1_driven, C1_p);
L1_mm = Lp1 * pitch;
Lp2 = chain_links_from_centres(N_stage2_driver, N_stage2_driven, C2_p);
L2_mm = Lp2 * pitch;


% ----------------- IDLER SIZING (both idlers on stage-2) -----------------
% Stage-1: no sprung idlers
total_preload_stage1 = min_total_preload_stage1;   % 0 N
per_idler_preload_stage1 = 0;
k_idler_stage1 = 0;

% Stage-2: total preload (distributed across num_idlers_stage2)
if num_idlers_stage2 <= 0
    error('num_idlers_stage2 must be >= 1 when placing idlers on stage-2.');
end

total_preload_stage2 = max(min_total_preload_stage2, idler_preload_ratio * F_stage2); % N
per_idler_preload_stage2 = total_preload_stage2 / num_idlers_stage2; % each idler supplies this preload
k_idler_stage2 = per_idler_preload_stage2 / idler_travel;   % N/mm per idler


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
function Lp = chain_links_from_centres(N1,N2,C_p)
    if C_p <= 0, error('C_p must be > 0'); end
    Lp_calc = 2*C_p + (N1+N2)/2 + ((N2 - N1)^2)/(4*pi^2 * C_p);
    Lp = round(Lp_calc);
    if mod(Lp,2) ~= 0, Lp = Lp + 1; end
end

function s = pick_std(d_mm)
    stds = [6 8 10 12 14 15 16 18 20 22 24 25 28 30 32 35 38 40 45 50];
    idx = find(stds >= ceil(d_mm), 1, 'first');
    if isempty(idx), s = ceil(d_mm); else s = stds(idx); end
end
