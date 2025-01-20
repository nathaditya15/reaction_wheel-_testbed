% Define constants and input matrices
J_T = diag([0.05, 0.02, 0.05]); % Spacecraft inertia matrix
T_b = [0.5; 0.5; 0.5]; % Body torque matrix (as a column vector)
Iw_s = 0.85 * eye(3); % Reaction wheel inertia matrix about spin axis
w_b = [0.2; 0.2; 0.2]; % Body angular rates
omega = [0.1; 0.1; 0.1]; % Angular rates of 3-axis reaction wheel
dt = 0.01; % Time step
omega_dot = omega * dt; % Change in angular rates
J_T_inv=inv(J_T);

% Compute the skew-symmetric matrix of w_b
Wb_skew = getWbSkew(w_b);

% Compute the derivative of body angular rates (w_b_dot)
w_b_dot = J_T_inv*(T_b - (Iw_s * omega_dot) - Wb_skew * (J_T * w_b + Iw_s * omega));

% Display results
disp('Skew-symmetric matrix of w_b:');
disp(Wb_skew);

disp('Derivative of body angular rates (w_b_dot):');
disp(w_b_dot);

% Function to compute skew-symmetric matrix
function Wb_skew = getWbSkew(Wb)
    % Check if the input is a 3x1 vector
    if length(Wb) ~= 3
        error('Input Wb must be a 3x1 vector');
    end
    
    % Extract components
    wx = Wb(1);
    wy = Wb(2);
    wz = Wb(3);
    
    % Construct the skew-symmetric matrix
    Wb_skew = [  0,   -wz,   wy;
                wz,     0,  -wx;
               -wy,    wx,    0];
end
