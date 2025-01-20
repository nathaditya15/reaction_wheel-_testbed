L_sat = diag([0.06, 0.02, 0.06]); % kg·m² ( values for 3 axes)
L_w = 0.05; % kg·m²
omega_desired = [0.1; 0.2; 0.3]; % rad/s
omega_sat_initial = [0; 0; 0]; % rad/s
Kp = diag([0.05, 0.05, 0.05]); % proportional gain
Kd = diag([0.01, 0.01, 0.01]); % derivative gain
Ki = diag([0, 0, 0]); % (optional) integral gain

