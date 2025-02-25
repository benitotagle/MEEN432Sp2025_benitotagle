clc; clear; close all;

R = 200;           % m - Radius of curved sections
L = 900;           % m - Length of each straight section
track_width = 15;  % m - Track width
half_width = track_width / 2;

% Create arrays for each segment of the track
t1 = linspace(-L/2, L/2, 100);  % Straight 1
x1 = t1;
y1 = zeros(size(t1));
theta1 = zeros(size(t1));

theta2 = linspace(-pi/2, pi/2, 100);  % Curve 1
x2 = (L/2) + R * cos(theta2);
y2 = R + R * sin(theta2);
theta2 = theta2 + pi/2;

t2 = linspace(L/2, -L/2, 100);  % Straight 2
x3 = t2;
y3 = 2 * R * ones(size(t2));
theta3 = zeros(size(t2));

theta4 = linspace(pi/2, 3*pi/2, 100);  % Curve 2
x4 = (-L/2) + R * cos(theta4);
y4 = R + R * sin(theta4);
theta4 = theta4 + pi/2;

% Combine the track
x_track     = [x1, x2, x3, x4];
y_track     = [y1, y2, y3, y4];
theta_track = [theta1, theta2, theta3, theta4];

% Inner and Outer Boundaries
x_inner = x_track - half_width * cos(theta_track + pi/2);
y_inner = y_track - half_width * sin(theta_track + pi/2);
x_outer = x_track + half_width * cos(theta_track + pi/2);
y_outer = y_track + half_width * sin(theta_track + pi/2);

g     = 9.81;      % m/s^2 (gravity)
vmass = 1500;      % kg
vin   = 2500;      % kg*m^2 (yaw moment of inertia)
vwbase = 2.5;      % m (wheelbase)
vlf   = 1.25;      % m (distance from CoG to front axle)
vlr   = vwbase - vlf;  % distance from CoG to rear axle

% Tire cornering stiffness
ca_f  = 60000;  % N/rad (front cornering stiffness)
ca_r  = 80000;  % N/rad (rear cornering stiffness)
miu   = 100.0;    % friction coefficient

% Normal forces (static load distribution)
nfzf = (vmass * g * vlr) / vwbase; % front axle normal force
nfzr = (vmass * g * vlf) / vwbase; % rear axle normal force

X0     = 0;    % m - initial X position
Y0     = 0;    % m - initial Y position
vx0    = 0.1;  % m/s - small initial longitudinal speed
vy0    = 0;    % m/s - initial lateral speed
omega0 = 0;    % rad/s - initial yaw rate
psi0   = 0;    % rad - initial heading

% Visualize the track
%{
figure; hold on; grid on; axis equal;
plot(x_inner, y_inner, 'k', 'LineWidth', 0.2);
plot(x_outer, y_outer, 'k', 'LineWidth', 0.2);
plot(x_track, y_track, 'k', 'LineWidth', 0.2);
xlabel('X (m)');
ylabel('Y (m)');
title('Project 2 Race Track');
%}

num_waypoints = 120;  % Number of waypoints along the track

% Generate interpolated waypoints
s_track = linspace(1, length(x_track), num_waypoints);
Xd = interp1(1:length(x_track), x_track, s_track, 'linear');
Yd = interp1(1:length(y_track), y_track, s_track, 'linear');
Theta_d = interp1(1:length(theta_track), theta_track, s_track, 'linear');

% Define time steps for waypoints 
t_waypoints = linspace(0, 600, num_waypoints)';

% Format waypoints for Simulink ("From Workspace" requires time-series data)
Xd_simulink = [t_waypoints, Xd'];  % [Time, Xd]
Yd_simulink = [t_waypoints, Yd'];  % [Time, Yd]
Theta_simulink = [t_waypoints, Theta_d'];  % [Time, Theta]

% Save waypoints and track boundaries into Project2Data.mat
save('Project2Data.mat', 'Xd_simulink', 'Yd_simulink', 'Theta_simulink', ...
    'x_inner', 'y_inner', 'x_outer', 'y_outer', ...
    'R', 'L', 'track_width', 'x_track', 'y_track', 'theta_track', ...
    'vmass', 'vin', 'vwbase', 'vlf', 'vlr', 'ca_f', 'ca_r', 'miu', ...
    'nfzf', 'nfzr', 'g', ...
    'X0', 'Y0', 'vx0', 'vy0', 'omega0', 'psi0');

