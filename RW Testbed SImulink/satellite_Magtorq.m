Is =diag([0.06, 0.05, 0.06]); %the matrix of the inertia tensor of the satellite
t_final = 300; %final time for each desired angles
Euler_d = [0;0;0]; %final desired angles [deg] - first z, then y, then x
initial_Euler = [0;0;0]; %initial euler angles [deg]
Iw = 50e-6;
D = [1 0 0;
 0 1 0;
 0 0 1];
KM = 9.5e-3; %[Nm/A] torque constant
Imax = 0.5; %[A]
Wmax = 40; %[red/sec]
N = 200; %number of coils
Amag = 49E-4; %cross-sectional area
Imax = 0.5; %maximum current
