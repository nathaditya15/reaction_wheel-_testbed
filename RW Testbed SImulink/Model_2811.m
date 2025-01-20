                                                                                                          %% Spacecraft Attitude Control System - Parameter Definition
% This script defines all parameters for the spacecraft attitude control simulation

%% Spacecraft Inertia Parameters
% Moments of Inertia (kg*m^2)
Ix = 0.06;  % Pitch axis moment of inertia
Iy = 0.02;  % Roll axis moment of inertia
Iz= 0.06;   % Yaw axis moment of inertia

%% Reaction Wheel Parameters
IRw = 0.01;     % Wheel moment of inertia (kg*m^2)
RWVmax= 3000 * (2*pi/60);  % Maximum wheel speed (rad/s)
RWMaxTorque= 0.5;    % Maximum wheel torque (N*m)

%% DC Motor Equivalent Model Parameters
R= 1;     % Electrical resistance (Ohm)
L= 0.001; % Electrical inductance (H)
Km= 0.01;  % Torque constant (N*m/A)
Kb= 0.01; % Back-EMF constant (V*s/rad)
f_v=0.001; % Viscous friction coefficient

%% Control System Parameters
% PID Controller Gains
Kp = 50;  % Kp
Ki= 0;      % Ki
Kd= 20;    % Kd

%% Simulation Parameters
Simulation.Time = 100;         % Total simulation time (seconds)
Simulation.SampleTime = 0.01;  % Sample time for simulation

%% Reference Input Parameters
ReferenceInput.StartTime = 0;   % When reference input starts
ReferenceInput.StepMagnitude = 10;  % Magnitude of step input (degrees)

% Print confirmation message
disp('Spacecraft Attitude Control Parameters Defined Successfully!');