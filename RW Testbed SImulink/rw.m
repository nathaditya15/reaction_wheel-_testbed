% Reaction Wheel DC Motor Model
function [torque] = rw(angular_velocity, motor_params)
    % Input parameters:
    % angular_velocity: Current angular velocity of the wheel (rad/s)
    % motor_params: Struct containing motor characteristics
    
    % Default motor parameters if not provided
    if nargin < 2
        motor_params.resistance = 1.0;  % Ohms
        motor_params.kt = 0.01;         % Torque constant (Nm/A)
        motor_params.ke = 0.01;         % Back-EMF constant (V-s/rad)
        motor_params.inertia = 0.001;   % kg-m^2
        motor_params.friction = 0.0001; % Viscous friction coefficient
    end
    
    % Calculate input voltage required to achieve desired angular velocity
    input_voltage = angular_velocity * motor_params.ke;
    
    % Calculate motor current
    motor_current = input_voltage / motor_params.resistance;
    
    % Calculate generated torque
    torque = motor_params.kt * motor_current - ...
             motor_params.friction * angular_velocity;
end

% Satellite Dynamics Model
function [satellite_state] = satellite_dynamics(reaction_wheel_torques, current_state)
    % Input parameters:
    % reaction_wheel_torques: [Tx, Ty, Tz] - Torques from reaction wheels (Nm)
    % current_state: [roll, pitch, yaw, angular_vel_x, angular_vel_y, angular_vel_z]
    
    % Satellite inertia matrix (example values)
    I = [10, 0, 0;
         0, 15, 0;
         0, 0, 20];  % kg-m^2
    
    % Extract current angular velocities
    omega = current_state(4:6);
    
    % Calculate satellite angular acceleration using Euler's equation
    angular_accel = inv(I) * (reaction_wheel_torques' - cross(omega, I * omega));
    
    % Update satellite state
    satellite_state = [
        current_state(1:3) + omega * 0.1;  % Update attitude
        angular_accel'                     % New angular accelerations
    ];
end

% PID Controller for Satellite Attitude Control
function [control_torques] = satellite_pid_controller(desired_attitude, current_state, pid_params)
    % Input parameters:
    % desired_attitude: [roll_des, pitch_des, yaw_des] (rad)
    % current_state: [roll, pitch, yaw, angular_vel_x, angular_vel_y, angular_vel_z]
    % pid_params: Struct containing PID gains
    
    % Default PID parameters if not provided
    if nargin < 3
        pid_params.Kp = [5, 5, 5];   % Proportional gains
        pid_params.Ki = [0.1, 0.1, 0.1]; % Integral gains
        pid_params.Kd = [2, 2, 2];   % Derivative gains
    end
    
    % Calculate attitude errors
    attitude_error = desired_attitude - current_state(1:3);
    
    % Calculate angular velocity errors
    angular_vel_error = -current_state(4:6);
    
    % Persistent variables for integral term
    persistent integral_error;
    if isempty(integral_error)
        integral_error = zeros(3,1);
    end
    
    % Update integral error
    integral_error = integral_error + attitude_error * 0.1;
    
    % Calculate PID control torques
    control_torques = pid_params.Kp .* attitude_error + ...
                      pid_params.Ki .* integral_error + ...
                      pid_params.Kd .* angular_vel_error;
end

% Main Simulation Function
function simulate_satellite_control()
    % Simulation parameters
    sim_time = 100;  % seconds
    dt = 0.1;        % time step
    
    % Initial conditions
    initial_attitude = [0, 0, 0];  % Initial [roll, pitch, yaw]
    initial_angular_vel = [0, 0, 0];  % Initial angular velocities
    desired_attitude = [0.5, 0.3, 0.2];  % Desired attitude
    
    % Initialize state
    current_state = [initial_attitude, initial_angular_vel];
    
    % Simulation loop
    for t = 0:dt:sim_time
        % PID Controller
        control_torques = satellite_pid_controller(desired_attitude, current_state);
        
        % Reaction Wheel Model
        wheel_torques = zeros(3,1);
        for i = 1:3
            wheel_torques(i) = rw(current_state(i+3), []);
        end
        
        % Satellite Dynamics
        current_state = satellite_dynamics(control_torques + wheel_torques, current_state);
        
        % Log or plot results as needed
    end
end