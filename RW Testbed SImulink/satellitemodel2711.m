%% Satellite Reaction Wheel Control System Model

%% 1. Satellite Dynamics Function
function [sys_dynamics] = satellite_dynamics(omega, external_torque)
    % Define satellite parameters
    I_xx = 10;  % Moment of inertia around x-axis (kg*m^2)
    I_yy = 12;  % Moment of inertia around y-axis (kg*m^2)
    I_zz = 15;  % Moment of inertia around z-axis (kg*m^2)
    
    % State space model of satellite rotational dynamics
    A = [0 1 0;
         0 0 1;
         0 0 -0.1];  % Simplified damping term
    
    B = [0 0 0;
         0 0 0;
         1/I_zz 1/I_yy 1/I_xx];
    
    % Incorporate external torques and wheel momentum
    sys_dynamics.A = A;
    sys_dynamics.B = B;
    sys_dynamics.external_torque = external_torque;
end

%% 2. Kalman Filter for Noise Reduction
function [kf_estimate] = satellite_kalman_filter(measurements, process_noise, measurement_noise)
    % Kalman filter parameters
    persistent K P x
    
    if isempty(P)
        P = eye(3);  % Initial error covariance
        x = zeros(3,1);  % Initial state estimate
    end
    
    % Prediction step
    x_pred = x;
    P_pred = P + process_noise;
    
    % Update step
    K = P_pred * inv(P_pred + measurement_noise);
    x = x_pred + K * (measurements - x_pred);
    P = (eye(3) - K) * P_pred;
    
    kf_estimate = x;
end

%% 3. Reaction Wheel Dynamics (DC Motor Model)
function [wheel_dynamics] = reaction_wheel_dynamics(control_torque)
    % DC Motor parameters for reaction wheel
    J_wheel = 0.01;     % Wheel moment of inertia (kg*m^2)
    B_friction = 0.001; % Viscous friction coefficient
    K_t = 0.1;          % Torque constant
    R_motor = 1;        % Motor resistance
    L_motor = 0.01;     % Motor inductance
    
    % State space model of reaction wheel
    A_wheel = [-B_friction/J_wheel   -K_t/(J_wheel*R_motor);
               K_t/L_motor           -R_motor/L_motor];
    
    B_wheel = [0; 1/L_motor];
    
    wheel_dynamics.A = A_wheel;
    wheel_dynamics.B = B_wheel;
    wheel_dynamics.control_input = control_torque;
end

%% 4. PID Controller Design
function [control_signal] = pid_attitude_controller(error, Kp, Ki, Kd)
    persistent integral previous_error
    
    if isempty(integral)
        integral = 0;
        previous_error = 0;
    end
    
    % PID gains
    Kp = 10;   % Proportional gain
    Ki = 0.1;  % Integral gain
    Kd = 1;    % Derivative gain
    
    % Compute PID terms
    integral = integral + error;
    derivative = error - previous_error;
    
    % Compute control signal
    control_signal = Kp * error + Ki * integral + Kd * derivative;
    
    % Anti-windup and saturation
    control_signal = max(min(control_signal, 10), -10);
    
    previous_error = error;
end

%% Simulink Model Configuration Script
function configure_satellite_model()
    % Create new Simulink model
    model_name = 'satellitemodel2711';
    new_system(model_name);
    
    % Add subsystems and blocks
    add_block('simulink/Subsystems/Satellite Dynamics', [model_name '/Satellite Dynamics']);
    add_block('simulink/Subsystems/Kalman Filter', [model_name '/Kalman Filter']);
    add_block('simulink/Subsystems/Reaction Wheel', [model_name '/Reaction Wheel']);
    add_block('simulink/Subsystems/PID Controller', [model_name '/PID Controller']);
    
    % Connect blocks
    add_line(model_name, 'Satellite Dynamics/1', 'Kalman Filter/1');
    add_line(model_name, 'Kalman Filter/1', 'PID Controller/1');
    
    % Save and open model
    save_system(model_name);
    open_system(model_name);
end

% Run configuration
configure_satellite_model();