% Comprehensive Satellite Attitude Control Simulink Model

% Cleanup
clear all
close all
clc

% Create New Simulink Model
model_name = 'SatADCSSIM10';
new_system(model_name);
open_system(model_name);

% Model Configuration
set_param(model_name, 'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'StartTime', '0', ...
    'StopTime', '100', ...
    'FixedStep', '0.01');

% 1. Reference Input Subsystem
add_block('simulink/Sources/Constant', [model_name, '/Reference_Attitude']);
set_param([model_name, '/Reference_Attitude'], ...
    'Value', 'deg2rad([10; 20; 30])');

% 2. Error Computation Subsystem
add_block('simulink/Math Operations/Sum', [model_name, '/Error_Computation']);
set_param([model_name, '/Error_Computation'], ...
    'Inputs', '+-');

% 3. PID Controller Subsystem
add_block('simulink/Continuous/PID Controller', [model_name, '/PID_Controller']);
set_param([model_name, '/PID_Controller'], ...
    'Kp', '10', ...
    'Ki', '0.1', ...
    'Kd', '1', ...
    'N', '100');

% 4. Reaction Wheel Dynamics Subsystem
add_block('simulink/Continuous/Transfer Fcn', [model_name, '/Reaction_Wheel_Dynamics']);
set_param([model_name, '/Reaction_Wheel_Dynamics'], ...
    'Numerator', '[0.5]', ...
    'Denominator', '[0.001 1]');

% 5. Satellite Dynamics Subsystem
add_block('simulink/Continuous/State-Space', [model_name, '/Satellite_Dynamics']);
set_param([model_name, '/Satellite_Dynamics'], ...
    'A', '[0 1 0; 0 0 1; -1 -2 -3]', ...
    'B', '[0; 0; 1]', ...
    'C', '[1 0 0]', ...
    'D', '[0]');

% 6. Sensor/Estimation Subsystem
add_block('simulink/Discrete/Discrete Filter', [model_name, '/Sensor_Estimation']);
set_param([model_name, '/Sensor_Estimation'], ...
    'Numerator', '[1]', ...
    'Denominator', '[1 -0.9]');

% 7. Visualization Scopes
add_block('simulink/Sinks/Scope', [model_name, '/Attitude_Scope']);

% Block Connections
try
    % Connect Reference to Error Computation
    add_line(model_name, 'Reference_Attitude/1', 'Error_Computation/1');
    
    % Connect Satellite Dynamics to Error Computation
    add_line(model_name, 'Satellite_Dynamics/1', 'Error_Computation/2');
    
    % Connect Error Computation to PID Controller
    add_line(model_name, 'Error_Computation/1', 'PID_Controller/1');
    
    % Connect PID Controller to Reaction Wheel Dynamics
    add_line(model_name, 'PID_Controller/1', 'Reaction_Wheel_Dynamics/1');
    
    % Connect Reaction Wheel Dynamics to Satellite Dynamics
    add_line(model_name, 'Reaction_Wheel_Dynamics/1', 'Satellite_Dynamics/1');
    
    % Connect Satellite Dynamics to Sensor Estimation
    add_line(model_name, 'Satellite_Dynamics/1', 'Sensor_Estimation/1');
    
    % Connect Sensor Estimation to Scope
    add_line(model_name, 'Sensor_Estimation/1', 'Attitude_Scope/1');
catch ME
    disp('Error connecting blocks:');
    disp(ME.message);
end

% Simulation Configuration
sim_params = Simulink.SimulationParameters;
sim_params.SaveOutput = 'on';
sim_params.OutputSaveName = 'simout';

% Run Simulation
try
    simout = sim(model_name, sim_params);
    
    % Plotting Results
    figure;
    plot(simout.time, simout.signals.values);
    title('Satellite Attitude Response');
    xlabel('Time (s)');
    ylabel('Attitude (rad)');
    legend('Roll', 'Pitch', 'Yaw');
catch ME
    disp('Simulation Error:');
    disp(ME.message);
end

% Save and Close
save_system(model_name);
close_system(model_name);

disp('Satellite Attitude Control System Simulation Complete');