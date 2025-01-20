% Reaction Wheel Parameters

J=0.00725;
K_e=0.071;
K_t=0.070;
b=0.000178;
L=0.00339;
R=1.52;


% Purpose: Initialize constants for ADCS Simulink Simulation and create
% init.h code file for Main Arduino on ADCS Testbed
% Clear workspace, clear command window, close figures
% clear all
% close all
% clc
% Initial Conditions for Simulation and Physical Testbed
% Set time step
% timestep divided by 4096 equals delta T in seconds
% 205/4096 = 0.05 seconds

timestep = 205;
% timestep in seconds, must correspond to timestep from above
dtrw = 0.05;
% Set initial reaction wheel angular velocity in radians per second
RW1_init = 0;
RW2_init = 0;
RW3_init = 0;
% Set initial air bearing angular orientation in radians
x_posinit = 0;
y_posinit = 0;
z_posinit = 0;
% Set initial air bearing angular velocity in radians per second
x_rate_init = 0;
y_rate_init = 0;
z_rate_init = 0;
% Air Bearing inertia tensor
Ixx = 4.0774;
Iyy = 4.1360;
Izz = 2.3216;
Iab = [Ixx 0 0;
0 Iyy 0;
0 0 Izz];

% Reaction Wheel Tensor 
I_a=0.0725;
I_rw=[I_a 0 0; 0 I_a 0; 0 0 I_a];

%DCM from reaction wheel frame to body frame
DCM_BODY_wrt_RW = [ 0.408248 0.408248 -0.816496;
-0.7071 0.7071 0;
0.5774 0.5774 0.5774];



A_rw = [-b/J 0 0 K_t/J 0 0; 0 -b/J 0 0 K_t/J 0; 0 0 -b/J 0 0 K_t/J; -K_e/L 0 0 -R/L 0 0; 0 -K_e/L 0 0 -R/L 0; 0 0 -K_e/L 0 0 -R/L];
B_rw = [0 0 0; 0 0 0; 0 0 0; 1/L 0 0; 0 1/L 0; 0 0 1/L];
C_rw = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];