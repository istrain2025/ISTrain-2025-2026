
clearvars; close all; clc;


%% ---------- PARAMETERS ----------
T_max = 12;                         % N·m - motor max torque
speed_max = 4000;                   % max rpm (at the motor)
g1 = 3.35;                          % stage 1 gear ratio
g2 = 3;                             % stage 2 gear ratio

% number of sprocket teeth
Z1 = 17;                            % stage 1 driver
Z2 = round(Z1*g1);                  %         driven
Z3 = 19;                            % stage 2 driver
Z4 = round(Z3*g2);                  %         driven
                                    
% correction factors
f1 = 1.4;                           % application factor, see renold pg 104
f2_stage1 = 19/Z1;                  % tooth factor for each stage
f2_stage2 = 19/Z3;

safety_factor = 2;                % todo: apply pugsley method

% geometry limitations (due to bogie design) in mm
L1_max = 180;                       % center to motor shaft longitudinal dist 
L_wheelbase = 650;


%% ---------- POWER ----------
speed_max = (speed_max*2*pi)/60;    % convert to rad/s

% STAGE 1
speed_jackshaft = speed_max/g1;
T_stage1 = T_max*g1;
power_stage1 = f1*f2_stage1*speed_jackshaft*T_stage1;

% STAGE 2
speed_wheelset = speed_jackshaft/g2;
% total output torque is 2x what is given to each wheelset
T_output = T_stage1*g2;
T_per_wheelset = T_output/2;
power_per_wheelset = f1*f2_stage2*speed_wheelset*T_per_wheelset;



%% ---------- PITCH ----------
% choose pitch from power and speed needed
% Stage 1
pitch_1 = 9.525;
q_mass_1 = 0.4;                       % Chain mass per meter
C_est_1 = 30*pitch_1;                 % estimate in pitches, actual C calculated later

% chain length
L_stage1 = (Z1+Z2)/2 + (2*C_est_1)/pitch_1 + ((((Z2-Z1)/(2*pi))^2)*pitch_1)/C_est_1 ;   % [pitches]
L_stage1 = ceil(L_stage1 / 2) * 2;
L_stage1_mm = L_stage1*pitch_1;                                                         % [mm]
% actual center distance in mm
C1 = (pitch_1/8)*(2*L_stage1-Z2-Z1+sqrt((2*L_stage1-Z2-Z1)^2 - ((pi/3.88)*(Z2-Z1)^2)));

% Stage 2
pitch_2 = 9.525;                      % mm - 06B pitch
q_mass_2 = 0.40;                      % 06B
C_est_2 = [30*pitch_2 50*pitch_2];

j=1;
L_stage2 = zeros(2,1);
L_stage2_mm = zeros(2,1);
C2 = zeros(2,1);
for C = C_est_2
    L_stage2(j) =  (Z3+Z4)/2 + (2*C)/pitch_2 + ((((Z4-Z3)/(2*pi))^2)*pitch_2)/C ;
    L_stage2(j) = ceil(L_stage2(j) / 2) * 2;
    L_stage2_mm(j) = pitch_2*L_stage2(j);
    
    C2(j) = (pitch_2/8)*(2*L_stage2(j)-Z4-Z3+sqrt((2*L_stage2(j)-Z4-Z3)^2 - ((pi/3.88)*(Z4-Z3)^2)));
    
    j=j+1;
end


% center distances function of the center distance of stage 1
C_stage2_short = L_wheelbase/2 - (C1 - L1_max);
C_stage2_long = L_wheelbase - C_stage2_short;

% pitch diameters
Dp1 = (pitch_1/1000) / sind(180/Z1);      % in m
Dp2 = (pitch_1/1000) / sind(180/Z2);
Dp3 = (pitch_2/1000) / sind(180/Z3);
Dp4 = (pitch_2/1000) / sind(180/Z4);


%% ---------- TRANSMISSION EQUATIONS ----------
% stage 1
v_stage1 = (speed_max*Z1*pitch_1)/(2*pi*1000);              % m/s, chain linear velocity
chainpull_stage1 = power_stage1/v_stage1;    % N

% stage 1
v_stage2 = (speed_jackshaft*Z3*pitch_2)/(2*pi*1000);
chainpull_stage2 = power_per_wheelset/v_stage2;

%% ---------- CENTRIFUGAL FORCE & TOTAL TENSION ----------
% F_oc = q * v^2 [N]
% F_total = F0 + F_oc [N]
% where q = weight of 1 m chain [kg/m], v = chain velocity [m/s]

% Stage 1 centrifugal force
F_oc_stage1 = q_mass_2 * v_stage1^2;                           % N

% Stage 2 centrifugal force
F_oc_stage2 = q_mass_2 * v_stage2^2;                           % N

% Total chain tension (traction force) for each stage
% F1 = F0 + F_oc
F_total_stage1 = chainpull_stage1 + F_oc_stage1;            % N
F_total_stage2 = chainpull_stage2 + F_oc_stage2;            % N




%% ---------- MATERIALS & SAFETY ----------
% Approximate values for 06B chain (verify with manufacturer)
tensile_strength_chain = 8900;   % in N, (minimum tensile strength for 06B chain according to ISO 606 - CHECK vendor)


%% ---------- SHAFT DESIGN PARAMETERS ----------
% shaft dimensions (in m)
% motor shaft
shaft1_length = 0.50;
shaft1_support = 0.25;      % distance from sprocket to one of the bearings
shaft1_diameter = 0.02125;     % critical cross section diameter
% jackshaft
shaft2_length = 0.50;
shaft2_support = 0.25;
shaft2_spacing = 0.125;  % d between s-1 and s-2 sprockets, assume symmetry
shaft2_diameter = 0.021;
% wheelset
shaft3_length = 0.50;
shaft3_support = shaft2_spacing;
shaft3_diameter = 0.03;

% material properties
S_uts = 420e6;              % UTS in Pa for shaft material (AISI 1020)
S_y = 350e6;                % yield stress
% endurance stress for each shaft
[S_e1, S_e2, S_e3] = endurance_stress(S_uts, [shaft1_diameter, shaft2_diameter, shaft3_diameter]);


%% ---------- SHAFT STRESSES & SUPPORT REACTIONS ----------
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
shaft3_bending_K = shaft3_bending*Kf_bending;
shaft1_shear_K = shaft1_shear*Kf_torsion;
shaft2_shear_K = shaft2_shear*Kf_torsion;
shaft3_shear_K = shaft3_shear*Kf_torsion;

% Check modified Goodman failure criterion
shaft1_fatigue_safety = 1/((shaft1_bending_K/S_e1)+(shaft1_shear_K/S_uts));
shaft2_fatigue_safety = 1/((shaft2_bending_K/S_e2)+(shaft2_shear_K/S_uts));
shaft3_fatigue_safety = 1/((shaft3_bending_K/S_e3)+(shaft3_shear_K/S_uts));

fatigue_isOK = [shaft1_fatigue_safety shaft2_fatigue_safety shaft3_fatigue_safety] >= safety_factor;




%% ---------- PRINT RESULTS ----------
fprintf('Selection power: stage 1 = %.1f W, stage 2 = %.1f W\n\n', power_stage1, power_per_wheelset);

fprintf('STAGE 1 - 06B chain (pitch = %.3f mm)\n', pitch_1)
fprintf('   Driving sprocket (%dT): Dp = %.5f mm, driven sprocket (%dT): Dp = %.5f mm\n', ...
    Z1, Dp1*1000, Z2, Dp2*1000);
fprintf('   Chain length: %.1f mm\n', L_stage1_mm);
fprintf('   Center distance: %.1f mm\n\n', C1);

fprintf('STAGE 2 - 06B chain (pitch = %.3f mm)\n', pitch_2)
fprintf('   Driving sprocket (%dT): Dp = %.5f mm, driven sprocket (%dT): Dp = %.5f mm\n', ...
    Z3, Dp3*1000, Z4, Dp4*1000);
%fprintf('   Chain length: %.1f mm\n', L_stage2_mm);
fprintf('   Center distances - short one: %.1f mm; long one: %.1f mm\n', C_stage2_short, C_stage2_long);
fprintf('   IDEAL center distance: %.1f mm < C < %.1f mm \n\n', C2(1), C2(2));





%% helper functions

function [S_e1, S_e2, S_e3] = endurance_stress(S_uts, d)
    % theoretical endurance stress
    S_e_theory = 0.5*S_uts;
    
    % convert diameters to mm
    d = d*1000;

    % Marin correction factors (check Shigley/PCMec slides)
        % surface finish correction factor
        k_surface = 4.51*(S_uts/1000000)^(-0.265);        % machined finish
        % size factor
        k_size = zeros(1,3);
        j = 1;
        for i = d
            if i <= 51
                k_size(j) = 1.24*i^(-0.107);
            else
                k_size(j) = 1.51*i^(-0.157);
            end
            j = j+1;
        end

    % actual endurance stress for each shaft
    S_e1 = S_e_theory*k_surface*k_size(1);
    S_e2 = S_e_theory*k_surface*k_size(2);
    S_e3 = S_e_theory*k_surface*k_size(3);
end
