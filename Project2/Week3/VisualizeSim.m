
% Load saved track data
load('Project2Data.mat', 'x_inner', 'y_inner', 'x_outer', 'y_outer');

% Debugging: Check what variables exist
whos  % Lists all workspace variables

% Ensure Simulink data exists
if exist('X_sim', 'var') && exist('Y_sim', 'var') && exist('psi_sim', 'var')
    X = X_sim;  
    Y = Y_sim;
    psi = psi_sim;
else
    error('Simulink data not found. Run the Simulink model first.');
end

% Define vehicle dimensions
veh_length = 30;  % m
veh_width = 15;   % m
vehicle_shape = [-veh_length/2, veh_length/2, veh_length/2, -veh_length/2;
                 -veh_width/2, -veh_width/2, veh_width/2, veh_width/2];

% Initialize figure
figure; hold on;
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Vehicle Animation');
plot(x_inner, y_inner, 'k', 'LineWidth', 2);
plot(x_outer, y_outer, 'k', 'LineWidth', 2);
veh_patch = patch(vehicle_shape(1, :) + X(1), vehicle_shape(2, :) + Y(1), 'r');

% Animate the vehicle movement
for i = 1:length(X)
    % Compute rotation
    R_mat = [cos(psi(i)), -sin(psi(i)); sin(psi(i)), cos(psi(i))];
    rotated_vehicle = R_mat * vehicle_shape;

    % Update vehicle position
    veh_patch.XData = rotated_vehicle(1, :) + X(i);
    veh_patch.YData = rotated_vehicle(2, :) + Y(i);

    % Plot the path
    plot(X(1:i), Y(1:i), 'b', 'LineWidth', 1.5);
    
    pause(0.01);
end
