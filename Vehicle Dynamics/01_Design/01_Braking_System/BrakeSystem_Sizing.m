clear; 
close all; 
clc;

%% Inputs %%
Mass = 1000;                 % [kg]
Deceleration = 1.3;          % [m/s^2] 
Wheel_Radius = 0.1;          % [m] 
Inner_DiscRadius = 0.02;     % [m] 
Total_Calipers = 4;
Mu_Pad = 0.4;                % Friction Coefficient (Pads)
Mu_Rail = 0.2;               % Friction Coefficient (Rail)
Max_Pad_Pressure = 1e6;      % Max allowed pad pressure [Pa]
Target_Angle = 35;           % Desired pad angle [º]
Gradient_Angle = 2;          % Track inclination, (flat track = 0)
Compressor_Pressure = 10e5 ;   % [Pa]
Aux_Tank_Pressure = 6e5;     % Auxiliary tank pressure [Pa]
Actuator_Stroke = 5e-3;      % [m]
Refill_time = 30;            % Tempo de refill do reservatório main [s]

%% Forces %%

F_Inertia = Mass * Deceleration;                                     
F_Gravity = Mass * 9.81 * sind(Gradient_Angle);    
F_Total_Req = F_Inertia + F_Gravity;                                 % Total Force needed
Num_Active_Calipers = Total_Calipers * 0.5;                          % Number of active calipers due to 50% rule
Force_Per_Active_Caliper = (F_Total_Req / Num_Active_Calipers);      % Force required by each caliper


Normal_Force = Mass * 9.81 * cosd(Gradient_Angle) / Total_Calipers;  
Grip_Force = Normal_Force * Mu_Rail;                                     

if Force_Per_Active_Caliper > Grip_Force                             
    Actual_F_Per_Wheel = Grip_Force;

else
    Actual_F_Per_Wheel = Force_Per_Active_Caliper;
end
fprintf('Grip force: %.1f\n', Grip_Force);
fprintf('Force per active caliper: %.1f\n', Force_Per_Active_Caliper);
fprintf('F per wheel: %.1f \n', Actual_F_Per_Wheel);

%% Calculations %%

Outer_DiscRadius = Inner_DiscRadius + 0.005;  
Tolerance = 0.05;                             % º
Current_Angle = 0;
Iteration = 0;
Max_iter = 5000;

while abs(Current_Angle - Target_Angle) > Tolerance
    Iteration = Iteration + 1;
    R_eff = (2/3)*((Outer_DiscRadius^3-Inner_DiscRadius^3)/(Outer_DiscRadius^2-Inner_DiscRadius^2)); % Effective radius
    
    Clamp_Force = (Actual_F_Per_Wheel * Wheel_Radius) / (2*Mu_Pad * R_eff);     % Force by the actuator on the pads
   

    Pad_Area = Clamp_Force / Max_Pad_Pressure;
    Swept_Area = pi * (Outer_DiscRadius^2 - Inner_DiscRadius^2);
    Current_Angle = (Pad_Area / Swept_Area) * 360;
    
    if Current_Angle > Target_Angle
        Outer_DiscRadius = Outer_DiscRadius + 0.00005;   
    else
        Outer_DiscRadius= Outer_DiscRadius - 0.00005;    
    end

    if Iteration > Max_iter
        fprintf('Loop couldnt converge');
        break;
    end

    if Outer_DiscRadius > Wheel_Radius
        fprintf('Outer_DiscRadius > Wheel Radius [Impossible]');
        break;
    end
  
end
torque = 2 * Mu_Pad * Clamp_Force * R_eff   % Torque da travagem;
%% Actuator %%

Actuator_Area = (Clamp_Force / Aux_Tank_Pressure) * 1.2;         % Actuator area 
Bore_Size = 2*sqrt(Actuator_Area / pi);                          % Actuator diameter (bore) 

%% Air Volume %%

Vol_per_cyl = Actuator_Area * Actuator_Stroke;                   % Volume of each actuator [m3]
Comp_ratio = (Aux_Tank_Pressure + 1e5) / 1e5;                    % Compression ratio
Air_Per_Cyl = Vol_per_cyl * Comp_ratio * 1.1 * 1000;             % Air needed for each actuator [L]
Total_Air_Cycle = Air_Per_Cyl * Total_Calipers;                  % Total air per cycle [L]

%% Main e Aux Tank sizing %%

% -- Aux tank -- %

Pressure_drop = 0.1;                                                % Pressure drop allowed due to braking P_1 * V_1 = P_2 * V_2
Aux_Vol = (Vol_per_cyl * 1000) * (1 - Pressure_drop)/Pressure_drop; % Vol. tanque auxiliar [L]

% -- Main tank -- %

Safety_stops = 3;                                                    % Number of brakings possible without compressor
Delta_P = (Compressor_Pressure - Aux_Tank_Pressure) / 1e5;                   % Possible variation of pressure
Main_Vol = (Total_Air_Cycle * Safety_stops) / Delta_P;

Q_Compressor = (Total_Air_Cycle / Refill_time) * 1.3 * 60;                % Compressor Flow rate [L/min]

Brake_Release_Time = 0.8;                   %[s]                          % [s]

Q_peak = (Total_Air_Cycle / Brake_Release_Time) * 60;   % Peak flow rate [L/min]
Required_Flow = Q_peak * 1.3;              % Safety factor for the max flow
%% Results %%

fprintf('--- 50%% Rule ---\n');
fprintf('Target Deceleration:  %.2f m/s^2\n', Deceleration);
fprintf('Active Calipers: %.0f (out of %.0f)\n', Num_Active_Calipers, Total_Calipers);
fprintf('Force required per wheel: %.2f N\n', Force_Per_Active_Caliper);
fprintf('Max grip per wheel: %.2f N\n', Grip_Force);
fprintf('Disc Outer Radius:  %.1f mm\n', Outer_DiscRadius * 1000);
fprintf('Disc Inner Radius:  %.1f mm\n', Inner_DiscRadius * 1000);
fprintf('Effective Radius:   %.1f mm\n', R_eff * 1000);
fprintf('-------------------------------\n');
fprintf('Total Braking Force: %.1f N (Includes Hill: %.1f N)\n', F_Total_Req, F_Gravity);
fprintf('Clamp Force Req:     %.1f N \n', Clamp_Force);
fprintf('Pad Area Needed:     %.1f cm^2\n', Pad_Area * 10000);
fprintf('Pad Angle:           %.1f deg\n', Current_Angle);
fprintf('Operating Pressure:  %.1f bar\n', Compressor_Pressure/1e5);
fprintf('-------------------------------\n');
fprintf('Actuator Area:         %.1f mm^2\n', Actuator_Area*1e6);
fprintf('Minimum Bore Size:     %.1f mm\n', Bore_Size*1000);
fprintf('Actuator Stroke:       %.1f mm\n', Actuator_Stroke*1000);
fprintf('Volume per Cycle per actuator:     %.2f L\n', Vol_per_cyl*1000);
fprintf('Total Air per Cycle:   %.2f L\n',Total_Air_Cycle);
fprintf('Compressor flow rate required:  %.2f L/min\n', Q_Compressor);
fprintf('Minimum Aux. Vol.: %.2f L\n', Aux_Vol*2); 
fprintf('Mininmum Main Vol.: %.2f L\n', Main_Vol);
fprintf('Required FLR Air service unit flow rate: %.1f L/min\n', Required_Flow);
fprintf('Torque: %.1f N.m\n', torque);