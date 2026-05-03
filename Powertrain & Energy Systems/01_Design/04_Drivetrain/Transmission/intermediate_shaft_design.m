%% =========================================================
%  INTERMEDIATE SHAFT DESIGN  –  Steps 1–14 (Shigley Ch.6)
%
%  Layout along x-axis:
%
%   BL ---- S1 ---- SI ---- S2 ---- BR
%   x=0    x=a1   x=a2    x=a3    x=L
%
%   BL, BR : left and right bearings
%   S1, S2 : secondary sprockets (output, drive wheel shafts)
%   SI     : intermediate (input) sprocket, driven by motor chain
%
%  Torque diagram:
%   - SI receives torque T_in from the motor chain
%   - Each secondary sprocket delivers T_out = T_in/2  (symmetry)
%   - Torque carried between sections:
%       [BL → S1]  : 0          (bearing carries no torque)
%       [S1 → SI]  : T_out      (one wheel-shaft already fed)
%       [SI → S2]  : T_out      (symmetric)
%       [S2 → BR]  : 0
%
%  Each sprocket has a chain resultant force in its own plane:
%   - SI : input chain, angle  phi_I  from +x (default: downward = -90°)
%   - S1 : output chain, angle phi_S1 from +x
%   - S2 : output chain, angle phi_S2 from +x (often same as S1 by symmetry)
%% =========================================================
clear; clc; close all;

%% ---- STEP 1-2 : MATERIAL ----------------------------------
S_u = 570e6;    % [Pa] ultimate tensile strength  (e.g. AISI 1045 HR)
S_y = 310e6;    % [Pa] yield strength
E   = 207e9;    % [Pa] Young's modulus

%% ---- STEP 3 : SAFETY FACTOR -------------------------------
n = 2.0;        % design safety factor

%% ---- GEOMETRY INPUTS --------------------------------------
% Shaft positions  [m]  (measured from left bearing BL)
L  = 0.450;      % total bearing span
a1 = 0.08;      % S1 position  (left secondary sprocket)
a2 = 0.20;      % SI position  (intermediate/input sprocket)  – must satisfy a1<a2<a3
a3 = 0.32;      % S2 position  (right secondary sprocket)
% Verify ordering
assert(0 < a1 && a1 < a2 && a2 < a3 && a3 < L, ...
    'Positions must satisfy 0 < a1 < a2 < a3 < L');

% Sprocket radii  [m]
R_motor = 0.0518;  % motor sprocket radius (on motor side, drives SI)
R_I     = 0.1729;  % intermediate sprocket radius  (SI, input)
R_S     = 0.0518;  % secondary sprocket radius  (S1 = S2, output)

% Motor torque  [N·m]
M_motor = 20;

% Preload (slack-side tension) – set to 0 for negligible preload
% Increase T_s_I and T_s_S later when preload is known
T_s_I = 0;      % [N] preload on input chain (motor -> SI)
T_s_S = 0;      % [N] preload on output chains (SI -> wheel shafts)

% Chain force directions (angle from +x axis, CCW positive)  [deg]
% Adjust these to match the physical layout of the bogie
phi_I_deg  = 0;     % input chain pulls SI to front  (motor chain comes from behind)
phi_S1_deg = 180;    % S1 output chain pulls downward (wheel shaft below)
phi_S2_deg = 0;    % S2 output chain pulls front (symmetric)

%% ---- STEP 4 : CHAIN TENSIONS & TORQUES -------------------
% Torque at intermediate sprocket SI
%   M_I = (T_t_I - T_s_I) * R_I
%   M_I = M_motor * (R_I / R_motor)   [torque scales with sprocket ratio]
M_I    = M_motor * (R_I / R_motor);    % [N·m] torque entering shaft at SI
dT_I   = M_I / R_I;                    % tight-slack difference, input chain
T_t_I  = dT_I + T_s_I;                % tight side, input chain

% Each output sprocket delivers half the input torque (symmetric load)
M_out  = M_I / 2;                      % [N·m] torque per output sprocket
dT_S   = M_out / R_S;                  % tight-slack difference, output chains
T_t_S  = dT_S + T_s_S;                % tight side, output chains

% Chain resultant forces on shaft (parallel-spans approx → F = T_t + T_s)
F_I  = T_t_I + T_s_I;                 % [N] force magnitude at SI
F_S1 = T_t_S + T_s_S;                 % [N] force magnitude at S1
F_S2 = T_t_S + T_s_S;                 % [N] force magnitude at S2  (= F_S1 by symmetry)

% Decompose into y (vertical) and z (horizontal) components
% Convention: x = shaft axis, y = vertical, z = horizontal
phi_I  = deg2rad(phi_I_deg);
phi_S1 = deg2rad(phi_S1_deg);
phi_S2 = deg2rad(phi_S2_deg);

FI_y  = F_I  * sin(phi_I);   FI_z  = F_I  * cos(phi_I);
FS1_y = F_S1 * sin(phi_S1);  FS1_z = F_S1 * cos(phi_S1);
FS2_y = F_S2 * sin(phi_S2);  FS2_z = F_S2 * cos(phi_S2);

fprintf('=== CHAIN LOADS ===\n');
fprintf('Input  sprocket SI : F = %.1f N  at %.0f deg  (Fy=%.1f, Fz=%.1f)\n', ...
    F_I, phi_I_deg, FI_y, FI_z);
fprintf('Output sprocket S1 : F = %.1f N  at %.0f deg  (Fy=%.1f, Fz=%.1f)\n', ...
    F_S1, phi_S1_deg, FS1_y, FS1_z);
fprintf('Output sprocket S2 : F = %.1f N  at %.0f deg  (Fy=%.1f, Fz=%.1f)\n', ...
    F_S2, phi_S2_deg, FS2_y, FS2_z);

%% ---- STEP 5 : BEARING REACTIONS (each plane separately) --
% Equilibrium in Y-plane (vertical):
%   BL_y + BR_y + FI_y + FS1_y + FS2_y = 0
%   Moments about BL=0:
%   BR_y*L + FI_y*a2 + FS1_y*a1 + FS2_y*a3 = 0
BR_y = -(FI_y*a2 + FS1_y*a1 + FS2_y*a3) / L;
BL_y = -(FI_y + FS1_y + FS2_y + BR_y);

% Equilibrium in Z-plane (horizontal):
BR_z = -(FI_z*a2 + FS1_z*a1 + FS2_z*a3) / L;
BL_z = -(FI_z + FS1_z + FS2_z + BR_z);

fprintf('\n=== BEARING REACTIONS ===\n');
fprintf('Left  bearing BL : Fy=%.1f N,  Fz=%.1f N,  |F|=%.1f N\n', ...
    BL_y, BL_z, norm([BL_y BL_z]));
fprintf('Right bearing BR : Fy=%.1f N,  Fz=%.1f N,  |F|=%.1f N\n', ...
    BR_y, BR_z, norm([BR_y BR_z]));

%% ---- STEP 6-7 : INTERNAL LOAD DIAGRAMS -------------------
nx = 2000;
x  = linspace(0, L, nx);

% --- Shear force (Macaulay step functions) ---
Vy = BL_y*ones(1,nx) ...
   + FS1_y*(x >= a1) ...
   + FI_y *(x >= a2) ...
   + FS2_y*(x >= a3);

Vz = BL_z*ones(1,nx) ...
   + FS1_z*(x >= a1) ...
   + FI_z *(x >= a2) ...
   + FS2_z*(x >= a3);

% --- Bending moment ---
My = BL_y*x ...
   + FS1_y*(x-a1).*(x>=a1) ...
   + FI_y *(x-a2).*(x>=a2) ...
   + FS2_y*(x-a3).*(x>=a3);   % bending in xz-plane (from y-forces)

Mz = BL_z*x ...
   + FS1_z*(x-a1).*(x>=a1) ...
   + FI_z *(x-a2).*(x>=a2) ...
   + FS2_z*(x-a3).*(x>=a3);   % bending in xy-plane (from z-forces)

% --- Torque diagram ---
% Section [BL → S1]  : 0
% Section [S1 → SI]  : +M_out   (S1 has absorbed its share)
% Section [SI → S2]  : +M_out   (SI fed both sides; before S2 takes its share)
% Section [S2 → BR]  : 0
T_diag = zeros(1,nx);
T_diag = T_diag + M_out*(x >= a1);   % S1 feeds torque into shaft
T_diag = T_diag - M_out*(x >= a2);   % SI adds full torque, net cancels left portion
% Restate cleanly:
T_diag = M_out*(x >= a1 & x < a2) ...   % between S1 and SI
        + M_out*(x >= a2 & x < a3);      % between SI and S2
% (both sides carry M_out, SI is in the middle – consistent with torque balance)

% Equivalent quantities
M_eq = sqrt(My.^2 + Mz.^2);
V_eq = sqrt(Vy.^2 + Vz.^2);

%% ---- PLOTS ------------------------------------------------
pos = [a1 a2 a3]*1e3;   % sprocket positions in mm for markers
col = {'r','g','b'};    % S1=red, SI=green, S2=blue

figure('Name','Intermediate Shaft – Internal Loads', ...
    'NumberTitle','off','Position',[40 40 960 780]);

sp(1) = subplot(5,1,1);
plot(x*1e3, Vy,'b', x*1e3, Vz,'r--','LineWidth',1.4); hold on;
yline(0,'k:'); xline(pos(1),'r:'); xline(pos(2),'g:'); xline(pos(3),'b:');
ylabel('V [N]'); legend('Vy','Vz'); title('Shear Force'); grid on;

sp(2) = subplot(5,1,2);
plot(x*1e3, My,'b', x*1e3, Mz,'r--','LineWidth',1.4); hold on;
yline(0,'k:'); xline(pos(1),'r:'); xline(pos(2),'g:'); xline(pos(3),'b:');
ylabel('M [N·m]'); legend('My','Mz'); title('Bending Moment'); grid on;

sp(3) = subplot(5,1,3);
plot(x*1e3, M_eq,'m','LineWidth',1.8); hold on;
yline(0,'k:'); xline(pos(1),'r:'); xline(pos(2),'g:'); xline(pos(3),'b:');
ylabel('M_{eq} [N·m]'); title('Equivalent Bending Moment'); grid on;

sp(4) = subplot(5,1,4);
plot(x*1e3, T_diag,'k','LineWidth',1.6); hold on;
yline(0,'k:'); xline(pos(1),'r:'); xline(pos(2),'g:'); xline(pos(3),'b:');
ylabel('T [N·m]'); title('Torque'); grid on;
% Annotate sections
text(mean([0 pos(1)]),  M_out*0.5, 'T=0',      'HorizontalAlignment','center','Color','k');
text(mean([pos(1) pos(2)]), M_out*1.1, sprintf('T=%.0f',M_out), 'HorizontalAlignment','center','Color','k');
text(mean([pos(2) pos(3)]), M_out*1.1, sprintf('T=%.0f',M_out), 'HorizontalAlignment','center','Color','k');
text(mean([pos(3) L*1e3]),  M_out*0.5, 'T=0',  'HorizontalAlignment','center','Color','k');

sp(5) = subplot(5,1,5);
plot(x*1e3, V_eq,'Color',[0.2 0.6 0.2],'LineWidth',1.4); hold on;
yline(0,'k:'); xline(pos(1),'r:'); xline(pos(2),'g:'); xline(pos(3),'b:');
ylabel('V_{eq} [N]'); xlabel('x [mm]'); title('Equivalent Shear Force'); grid on;

% Shared x-label and vertical markers legend
annotation('textbox',[0.75 0.01 0.22 0.04],'String', ...
    'Red: S1 | Green: SI | Blue: S2','EdgeColor','none','FontSize',8);

linkaxes(sp,'x'); xlim([0 L]*1e3);

%% ---- STEP 8-9 : CRITICAL SECTIONS ------------------------
% Check ALL candidate sections: S1, SI, S2 and between them
% We evaluate at each sprocket location (discontinuities in torque)
check_x   = [a1, a2, a3];
check_lbl = {'S1','SI','S2'};

fprintf('\n=== CANDIDATE CRITICAL SECTIONS ===\n');
fprintf('%-6s  %8s  %8s  %8s  %8s\n','Pos','Meq[Nm]','T[Nm]','Veq[N]','Combined');

results = zeros(length(check_x), 4);
for k = 1:length(check_x)
    xi   = check_x(k);
    % Evaluate at this x (using the continuous arrays)
    [~,idx] = min(abs(x - xi));
    Me  = M_eq(idx);
    Ti  = T_diag(idx);   % torque just to the RIGHT of the discontinuity
    Ve  = V_eq(idx);
    % Von Mises combined stress indicator (proportional to 1/d^3)
    combined = sqrt(Me^2 + (3/4)*Ti^2);  % von Mises grouping (before pi/d^3 factor)
    results(k,:) = [Me Ti Ve combined];
    fprintf('%-6s  %8.2f  %8.2f  %8.2f  %8.2f\n', check_lbl{k}, Me, Ti, Ve, combined);
end

[~, crit_idx]  = max(results(:,4));
x_crit         = check_x(crit_idx);
M_eq_crit      = results(crit_idx,1);
T_crit         = results(crit_idx,2);
V_eq_crit      = results(crit_idx,3);

fprintf('\n--> Critical section: %s at x = %.1f mm\n', check_lbl{crit_idx}, x_crit*1e3);
fprintf('    M_eq = %.2f N·m,  T = %.2f N·m,  V_eq = %.2f N\n', ...
    M_eq_crit, T_crit, V_eq_crit);

%% ---- STEP 10 : STATIC DIAMETER (von Mises, neglect Veq) --
d_static = ((32*n)/(pi*S_y) * sqrt(M_eq_crit^2 + (3/4)*T_crit^2))^(1/3);

fprintf('\n=== STEP 10 – STATIC DIAMETER (no Veq) ===\n');
fprintf('d_static = %.2f mm\n', d_static*1e3);

%% ---- STEP 11 : LONG SHAFT CHECK --------------------------
ratio = L / d_static;
fprintf('\n=== STEP 11 – LONG SHAFT CHECK ===\n');
fprintf('L/d = %.1f  -->  %s\n', ratio, ...
    ternary(ratio > 10, 'LONG SHAFT – deflection check recommended', 'Short shaft – OK'));

%% ---- STEP 12 : ITERATE INCLUDING Veq ---------------------
d_iter = d_static;
for k = 1:50
    sigma_b  = 32*M_eq_crit / (pi*d_iter^3);
    tau_t    = 16*T_crit    / (pi*d_iter^3);
    tau_v    = 16*V_eq_crit / (3*pi*d_iter^2);   % average shear from Veq
    sigma_vm = sqrt(sigma_b^2 + 3*(tau_t + tau_v)^2);
    d_new    = d_iter * (n*sigma_vm/S_y)^(1/3);
    if abs(d_new-d_iter)/d_iter < 1e-8; break; end
    d_iter   = d_new;
end
d_min_static = d_iter;

fprintf('\n=== STEP 12 – ITERATED STATIC DIAMETER (with Veq) ===\n');
fprintf('d_min_static = %.2f mm\n', d_min_static*1e3);

%% ---- STEP 13 : FATIGUE DIAMETER ESTIMATE -----------------
d_fatigue = 1.6 * d_min_static;

fprintf('\n=== STEP 13 – FATIGUE DIAMETER ESTIMATE ===\n');
fprintf('d_fatigue ≈ 1.6 × d_static = %.2f mm\n', d_fatigue*1e3);

%% ---- STEP 14 : GEOMETRY DETAILING ------------------------
d_nom  = ceil(d_fatigue*1e3 / 5)*5;   % round up to nearest 5 mm
d_bear = d_nom - 5;                    % bearing seat slightly smaller

fprintf('\n=== STEP 14 – GEOMETRY DETAILING ===\n');
fprintf('Recommended shaft diameter zones:\n');
fprintf('  Shaft body (under sprockets)  : d = %g mm\n', d_nom);
fprintf('  Bearing seats (BL, BR)        : d = %g mm  (check bearing catalogue)\n', d_bear);
fprintf('  Shoulders at S1, SI, S2       : diameter steps needed for axial location\n');
fprintf('  Keyways at S1, SI, S2         : stress concentration → Kf applied in Steps 21-22\n');
fprintf('  Shaft between S1-SI and SI-S2 : carries torque M_out = %.1f N·m\n', M_out);

%% ---- SUMMARY TABLE ----------------------------------------
fprintf('\n========================================\n');
fprintf('      INTERMEDIATE SHAFT SUMMARY\n');
fprintf('========================================\n');
fprintf('Motor torque             M_motor = %6.1f N·m\n', M_motor);
fprintf('Input torque at SI       M_I     = %6.1f N·m\n', M_I);
fprintf('Output torque per wheel  M_out   = %6.1f N·m\n', M_out);
fprintf('Input chain force        F_I     = %6.1f N\n',   F_I);
fprintf('Output chain force (ea)  F_S     = %6.1f N\n',   F_S1);
fprintf('Left  bearing force      |F_BL|  = %6.1f N\n',   norm([BL_y BL_z]));
fprintf('Right bearing force      |F_BR|  = %6.1f N\n',   norm([BR_y BR_z]));
fprintf('Critical section         pos     = %6.1f mm  (%s)\n', x_crit*1e3, check_lbl{crit_idx});
fprintf('  M_eq at critical       Meq     = %6.2f N·m\n', M_eq_crit);
fprintf('  Torque at critical     T       = %6.2f N·m\n', T_crit);
fprintf('Static diameter          d_st    = %6.2f mm\n',  d_min_static*1e3);
fprintf('Fatigue estimate         d_fat   = %6.2f mm\n',  d_fatigue*1e3);
fprintf('Recommended nominal      d       = %6g mm\n',    d_nom);
fprintf('Bearing seat diameter    d_bear  = %6g mm\n',    d_bear);
fprintf('========================================\n');

%% ---- HELPER FUNCTION -------------------------------------
function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end
