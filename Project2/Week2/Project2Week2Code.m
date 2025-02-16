clc; clear; close all;

%%% VEHICLE PARAMETERS %%%
vehicleData.Mass = 1000; % kg - Vehicle Mass
vehicleData.Inertia = 1600; % kg*m^2 - Vehicle Rotational Inertia
vehicleData.Length = 30; % m - Vehicle Length
vehicleData.Width = 15; % m - Vehicle Width
vehicleData.WheelBase = 2.5; % m - Distance between front and rear axles
vehicleData.TireRadius = 0.3; % m - Tire Radius
vehicleData.Calpha_f = 40000; % N/rad - Front Tire Cornering Stiffness
vehicleData.Calpha_r = 40000; % N/rad - Rear Tire Cornering Stiffness
vehicleData.lf = 1.5; % m - Distance from CG to Front Axle
vehicleData.lr = 1.0; % m - Distance from CG to Rear Axle
vehicleData.vehicleSpeed = 5; % m/s - Vehicle Speed

% Understeer Coefficient
vehicleData.understeerCoeff = ...
    vehicleData.Mass / ((vehicleData.lr + vehicleData.lf) * 200) * ...
    (vehicleData.lr / vehicleData.Calpha_f - vehicleData.lf / vehicleData.Calpha_r);

%%% INITIAL CONDITIONS %%%
vehicleData.init.X0 = 0;        % m - Initial X Position of the Car
vehicleData.init.Y0 = 0;        % m - Initial Y Position of the Car
vehicleData.init.vx0 = 0.1;     % m/s - Initial Velocity in X of the Car
vehicleData.init.vy0 = 0;       % m/s - Initial Velocity in Y of the Car
vehicleData.init.omega0 = 0;    % rad/s - Initial Yaw Rate of the Car
vehicleData.init.psi0 = 0;      % rad - Initial Heading of the Car

%%% TRACK PARAMETERS %%%
trackData.Radius = 200; % m - Track Corner Radius
trackData.Length = 900; % m - Straightaway Length
trackData.Width = 15; % m - Track Width
trackData.HalfWidth = trackData.Width / 2;

%%% TRACK PATH GENERATION %%%
t1 = linspace(-trackData.Length/2, trackData.Length/2, 100);
x1 = t1;
y1 = zeros(size(t1));
theta1 = zeros(size(t1)); 

theta2 = linspace(-pi/2, pi/2, 100);
x2 = (trackData.Length/2) + trackData.Radius * cos(theta2);
y2 = trackData.Radius + trackData.Radius * sin(theta2);
theta2 = theta2 + pi/2;

t2 = linspace(trackData.Length/2, -trackData.Length/2, 100);
x3 = t2;
y3 = 2 * trackData.Radius * ones(size(t2));
theta3 = zeros(size(t2)); 

theta4 = linspace(pi/2, 3*pi/2, 100);
x4 = (-trackData.Length/2) + trackData.Radius * cos(theta4);
y4 = trackData.Radius + trackData.Radius * sin(theta4);
theta4 = theta4 + pi/2; 

x = [x1 x2 x3 x4];
y = [y1 y2 y3 y4];
theta = [theta1 theta2 theta3 theta4];

x_inner = x - trackData.HalfWidth * cos(theta + pi/2);
y_inner = y - trackData.HalfWidth * sin(theta + pi/2);

x_outer = x + trackData.HalfWidth * cos(theta + pi/2);
y_outer = y + trackData.HalfWidth * sin(theta + pi/2);

%%% PLOTTING TRACK %%%
figure;
hold on;
plot(x_inner, y_inner, 'k', 'LineWidth', 2); 
plot(x_outer, y_outer, 'k', 'LineWidth', 2); 
plot(x, y, 'k', 'LineWidth', 1.5); 
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Race Track');

%%% VEHICLE ANIMATION %%%
vehicle_shape = [-vehicleData.Length/2, vehicleData.Length/2, vehicleData.Length/2, -vehicleData.Length/2;
                 -vehicleData.Width/2, -vehicleData.Width/2, vehicleData.Width/2, vehicleData.Width/2]; 

veh_patch = patch(vehicle_shape(1, :) + x(1), vehicle_shape(2, :) + y(1), 'r');

path_x = [];
path_y = [];

%%% INTERPOLATION FOR VEHICLE MOVEMENT %%%
dist = cumsum([0, sqrt(diff(x).^2 + diff(y).^2)]);
[dist_unique, idx] = unique(dist); 
x_unique = x(idx);
y_unique = y(idx);
theta_unique = theta(idx);

desired_dist = 0:vehicleData.vehicleSpeed:max(dist_unique);
x_interp = interp1(dist_unique, x_unique, desired_dist, 'linear');
y_interp = interp1(dist_unique, y_unique, desired_dist, 'linear');
theta_interp = interp1(dist_unique, theta_unique, desired_dist, 'linear');

for i = 1:length(x_interp)
    path_x = [path_x, x_interp(i)];
    path_y = [path_y, y_interp(i)];
    
    R_mat = [cos(theta_interp(i)), -sin(theta_interp(i)); sin(theta_interp(i)), cos(theta_interp(i))];
    rotated_vehicle = R_mat * vehicle_shape;
    veh_patch.XData = rotated_vehicle(1, :) + x_interp(i);
    veh_patch.YData = rotated_vehicle(2, :) + y_interp(i);
    
    plot(path_x, path_y, 'b', 'LineWidth', 1.5);
    
    pause(0.005);
end
