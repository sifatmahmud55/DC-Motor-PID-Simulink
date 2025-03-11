% MATLAB script for setting up Simulink DC Motor Model with PID Control

% Project Description:
% This project focuses on controlling the speed of a DC motor using a PID controller in Simulink. 
% A DC motor is widely used in industrial and automation applications where precise speed control is required. 
% The main objective of this project is to model a DC motor using its transfer function and apply a PID 
% controller to regulate its speed. By tuning the PID parameters (Kp, Ki, Kd), we can achieve optimal 
% performance in terms of stability and response time. 
%
% The simulation begins by defining the motor parameters, including moment of inertia, damping coefficient, 
% motor constant, resistance, and inductance. These parameters help derive the transfer function of the motor. 
% The PID controller is then implemented to adjust the control input based on the speed error, ensuring that 
% the motor reaches the desired speed with minimal overshoot and steady-state error. 
%
% In Simulink, users can create a model by connecting blocks such as the PID Controller, Transfer Function, 
% Step Input, and Scope to visualize the system’s response. By tuning the PID gains, users can explore the 
% effects on motor speed, observing transient and steady-state performance. 
%
% This project is beneficial for learning control system concepts and practical motor speed regulation.

% Motor Parameters
J = 0.01;   % Moment of inertia (kg.m^2)
B = 0.1;    % Damping coefficient (N.m.s)
K = 0.01;   % Motor constant (N.m/A or V/rad/s)
R = 1;      % Resistance (Ohm)
L = 0.5;    % Inductance (H)

% Define Transfer Function of DC Motor
s = tf('s');
P_motor = K / ((J*s + B) * (L*s + R) + K^2);

% PID Controller Parameters
Kp = 100;
Ki = 200;
Kd = 10;
C = pid(Kp, Ki, Kd);

% Open-Loop Response
figure;
step(P_motor);
title('Open-Loop Response of DC Motor');

display('Now, open Simulink and create the following blocks:');
display('1. Step input -> PID Controller -> DC Motor Transfer Function -> Scope');
display('2. Use Simulink blocks like "PID Controller", "Transfer Function", and "Scope"');
display('3. Tune PID values and observe the speed response');





