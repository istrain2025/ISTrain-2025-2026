%% =========================================================
%  SHAFT DESIGN UNDER FATIGUE  –  Steps 1–14 (Shigley Ch.6)
%  Chain-drive system: motor sprocket (A) -> driven sprocket (B)
%  Shaft of sprocket B held by two bearings (left=L, right=R)
%  Assumptions:
%    - Chain spans are parallel  (simplifies bearing load calc)
%    - Preload T_s is included as a variable (set to 0 here)
%    - Static design first, then estimate fatigue diameter
%% =========================================================
clear; clc; close all;

%% ---- STEP 1-2 : MATERIAL ----------------------------------
% Steel – ductile material (e.g. AISI 1045 HR, Shigley Table A-20)
S_u  = 570e6;   % [Pa] ultimate tensile strength
S_y  = 310e6;   % [Pa] yield strength
E    = 207e9;   % [Pa] Young's modulus (for deflection checks if needed)

%% ---- STEP 3 : SAFETY FACTOR -------------------------------
n = 2.0;        % design safety factor (typical: 1.5 / 2 / 2.5 / 3)

%% ---- GEOMETRY & LOADING INPUTS ----------------------------
% Sprocket radii
R_A = 0.0518;     % [m] driver sprocket (on motor side)
R_B = 0.1729;     % [m] driven sprocket (on this shaft)

% Motor torque applied at sprocket A
M_motor = 12;  % [N·m]

% Preload on slack side (set to 0 → negligible preload assumption)
T_s = 0;        % [N]  slack-side tension (change later if needed)

% Shaft geometry: bearings at x=0 (left) and x=L (right)
%                 sprocket B sits at position x=a from left bearing
L = 0.450;       % [m] total bearing span
a = 0.01;       % [m] sprocket B location from left bearing

% Chain orientation: angle of the chain-span resultant from +x axis
% (0 = horizontal pull, 90 = vertical pull, etc.)
% For a horizontal chain pulling downward-ish, use e.g. 270° = -90°
chain_angle_deg = 270;          % [deg]  direction of chain pull on B
chain_angle     = deg2rad(chain_angle_deg);

%% ---- STEP 4 : CHAIN TENSIONS & SPROCKET LOADS -------------
% Torque balance on sprocket B (driven):
%   M_B = (T_t - T_s) * R_B   and   M_B / M_motor = R_B / R_A
M_B   = M_motor * (R_B / R_A);         % [N·m] torque delivered at B
dT    = M_B / R_B;                      % [N]   tight - slack tension difference
T_t   = dT + T_s;                       % [N]   tight-side tension

fprintf('=== LOADING SUMMARY ===\n');
fprintf('M_motor = %.1f N·m\n', M_motor);
fprintf('M_B     = %.1f N·m  (gear ratio = %.3f)\n', M_B, R_B/R_A);
fprintf('T_t     = %.1f N\n', T_t);
fprintf('T_s     = %.1f N\n', T_s);

% Resultant chain force on shaft B (both spans pull in same direction
% when spans are parallel → F = T_t + T_s)
F_chain = T_t + T_s;                    % [N]   magnitude
Fx = F_chain * cos(chain_angle);        % [N]   x-component
Fy = F_chain * sin(chain_angle);        % [N]   y-component

fprintf('F_chain = %.1f N  at %.0f deg\n', F_chain, chain_angle_deg);

%% ---- STEP 5 : FREE-BODY DIAGRAM ---------------------------
% Shaft: simply supported at x=0 (bearing L) and x=L (bearing R)
% Sprocket load applied at x=a: forces (Fx, Fy) and torque T = M_B
%
%   RL_y + RR_y + Fy = 0   (vertical equilibrium)
%   RL_x + RR_x + Fx = 0   (horizontal equilibrium)
%   Torque T carried uniformly between sprocket and one bearing

% Bearing reactions in Y (vertical plane)
RR_y = -Fy * a / L;                     % [N]
RL_y = -Fy - RR_y;                      % [N]

% Bearing reactions in X (horizontal plane)
RR_x = -Fx * a / L;                     % [N]
RL_x = -Fx - RR_x;                      % [N]

fprintf('\n=== BEARING REACTIONS ===\n');
fprintf('Left  bearing:  Rx=%.1f N,  Ry=%.1f N\n', RL_x, RL_y);
fprintf('Right bearing:  Rx=%.1f N,  Ry=%.1f N\n', RR_x, RR_y);

%% ---- STEP 6 : SHEAR & BENDING MOMENT DIAGRAMS ------------
% Evaluate at fine resolution along shaft length
nx  = 500;
x   = linspace(0, L, nx);

% Shear force (each plane separately, using Macaulay)
Vy  = RL_y * ones(1,nx) + Fy  * (x >= a);
Vx  = RL_x * ones(1,nx) + Fx  * (x >= a);

% Bending moment
My  = RL_y * x + Fy  * (x - a) .* (x >= a);   % bending in xz-plane
Mz  = RL_x * x + Fx  * (x - a) .* (x >= a);   % bending in xy-plane

% Torque (applied at sprocket, carried to nearest bearing)
T_shaft = zeros(1,nx);
T_shaft(x >= a) = M_B;                          % torque from sprocket to right bearing

%% ---- STEP 7 : PLOTS ---------------------------------------
figure('Name','Shaft Internal Loads','NumberTitle','off','Position',[50 50 900 700]);

subplot(4,1,1);
plot(x*1e3, Vy,'b', x*1e3, Vx,'r--','LineWidth',1.4);
yline(0,'k:'); xlabel('x [mm]'); ylabel('V [N]');
legend('Vy (vertical)','Vx (horizontal)'); title('Shear Force'); grid on;

subplot(4,1,2);
plot(x*1e3, My,'b', x*1e3, Mz,'r--','LineWidth',1.4);
yline(0,'k:'); xlabel('x [mm]'); ylabel('M [N·m]');
legend('My','Mz'); title('Bending Moment'); grid on;

subplot(4,1,3);
M_eq = sqrt(My.^2 + Mz.^2);                    % equivalent bending moment (step 9)
plot(x*1e3, M_eq,'m','LineWidth',1.6);
yline(0,'k:'); xlabel('x [mm]'); ylabel('M_{eq} [N·m]');
title('Equivalent Bending Moment  M_{eq} = sqrt(My^2+Mz^2)'); grid on;

subplot(4,1,4);
plot(x*1e3, T_shaft,'k','LineWidth',1.4);
ylim([-0.1*M_B, 1.5*M_B]);
xlabel('x [mm]'); ylabel('T [N·m]');
title('Torque'); grid on;

%% ---- STEP 8-9 : CRITICAL SECTION -------------------------
% Find position of maximum equivalent bending moment
[M_eq_max, idx_crit] = max(M_eq);
x_crit = x(idx_crit);
T_crit = T_shaft(idx_crit);

V_eq_max = max(sqrt(Vy.^2 + Vx.^2));           % equivalent shear (step 9)

fprintf('\n=== CRITICAL SECTION at x = %.1f mm ===\n', x_crit*1e3);
fprintf('M_eq = %.2f N·m\n', M_eq_max);
fprintf('T    = %.2f N·m\n', T_crit);
fprintf('V_eq = %.2f N\n',   V_eq_max);

%% ---- STEP 10 : STATIC DIAMETER (von Mises, step 10) ------
% Neglecting axial force N and shear Veq (conservative first estimate)
% d_est = [32n/pi/Sy * sqrt(M_eq^2 + 3/4*T^2)]^(1/3)
d_static = ( (32*n)/(pi*S_y) * sqrt(M_eq_max^2 + (3/4)*T_crit^2) )^(1/3);

fprintf('\n=== STATIC DESIGN (Step 10) ===\n');
fprintf('d_static = %.2f mm\n', d_static*1e3);

%% ---- STEP 11 : LONG SHAFT CHECK --------------------------
ratio_Ld = L / d_static;
fprintf('\n=== LONG SHAFT CHECK (Step 11) ===\n');
fprintf('L/d = %.1f  -->  %s\n', ratio_Ld, ...
    ternary(ratio_Ld > 10, 'LONG SHAFT (deflection may govern)', 'Short shaft – OK'));

%% ---- STEP 12 : ITERATE WITH Veq AND N (Step 12) ----------
% Re-compute d including shear Veq (axial N assumed zero here)
% Shear stress from Veq: tau_Veq = 16*Veq / (3*pi*d^2)
% Iterate once (using d_static as initial guess):
d_iter = d_static;
for k = 1:20
    tau_Veq = (16*V_eq_max) / (3*pi*d_iter^2);
    % Von Mises equivalent stress including shear:
    % sigma_eq = sqrt( (32*M_eq/pi/d^3)^2 + 3*(16*T/pi/d^3 + tau_Veq)^2 )
    % Set sigma_eq = Sy/n and solve for d iteratively
    sigma_bend = 32*M_eq_max / (pi*d_iter^3);
    tau_tors   = 16*T_crit    / (pi*d_iter^3);
    sigma_vm   = sqrt(sigma_bend^2 + 3*(tau_tors + tau_Veq)^2);
    d_new      = d_iter * (sigma_vm * n / S_y)^(1/3);
    if abs(d_new - d_iter)/d_iter < 1e-6; break; end
    d_iter = d_new;
end
d_min_static = d_iter;

fprintf('\n=== ITERATED STATIC DIAMETER (Step 12) ===\n');
fprintf('d_min_static = %.2f mm\n', d_min_static*1e3);

%% ---- STEP 13 : FATIGUE DIAMETER ESTIMATE -----------------
% Simple first estimate: d_fatigue ≈ 1.6 * d_static  (from lecture)
d_fatigue_est = 1.6 * d_min_static;

fprintf('\n=== FATIGUE DIAMETER ESTIMATE (Step 13) ===\n');
fprintf('d_fatigue ≈ 1.6 * d_static = %.2f mm\n', d_fatigue_est*1e3);

%% ---- STEP 14 : GEOMETRY DETAILING NOTE -------------------
fprintf('\n=== STEP 14 : GEOMETRY DETAILING ===\n');
fprintf('Nominal shaft diameters to consider:\n');
d_nom = ceil(d_fatigue_est*1e3 / 5) * 5;       % round up to nearest 5 mm
fprintf('  Shaft body  (under sprocket)  : d = %g mm\n', d_nom);
fprintf('  Shaft ends  (bearing seats)   : d ≈ %g mm  (check bearing catalogue)\n', d_nom - 5);
fprintf('  Shoulders / fillets required at sprocket and bearing seats.\n');
fprintf('  Keyway at sprocket will require stress concentration (Step 21+).\n');

%% ---- SUMMARY TABLE ----------------------------------------
fprintf('\n========================================\n');
fprintf('         DESIGN SUMMARY\n');
fprintf('========================================\n');
fprintf('Motor torque          M_A  = %6.1f N·m\n', M_motor);
fprintf('Torque on shaft B     M_B  = %6.1f N·m\n', M_B);
fprintf('Tight side tension    T_t  = %6.1f N\n',   T_t);
fprintf('Slack side tension    T_s  = %6.1f N\n',   T_s);
fprintf('Chain resultant       F    = %6.1f N\n',   F_chain);
fprintf('Critical section      x    = %6.1f mm\n',  x_crit*1e3);
fprintf('Max equiv. moment     Meq  = %6.2f N·m\n', M_eq_max);
fprintf('Torque at section     T    = %6.2f N·m\n', T_crit);
fprintf('Static diameter       d_st = %6.2f mm\n',  d_min_static*1e3);
fprintf('Fatigue estimate      d_f  = %6.2f mm\n',  d_fatigue_est*1e3);
fprintf('Recommended nominal   d    = %6g mm\n',    d_nom);
fprintf('========================================\n');

%% ---- HELPER FUNCTION (inline) ----------------------------
function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end
