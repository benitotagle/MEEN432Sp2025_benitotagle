% Load saved track data
load('Project2Data.mat', 'x_inner', 'y_inner', 'x_outer', 'y_outer', 'R', 'L', 'track_width');

if exist('X_sim', 'var') && exist('Y_sim', 'var') && exist('psi_sim', 'var') && exist('tout', 'var')
    X = X_sim;  
    Y = Y_sim;
    time = tout;  % Time array from Simulink
else
    error('Simulink data not found. Run the Simulink model first.');
end

total_track_length = 2 * L + 2 * pi * R;

num_points = length(X);
centerline_dist = zeros(num_points, 1);

for i = 1:num_points
    x = X(i);
    y = Y(i);

    if abs(y) < 1e-3  % Bottom Straight
        centerline_dist(i) = L/2 + x;
    elseif abs(y - 2*R) < 1e-3  % Top Straight
        centerline_dist(i) = 3*L/2 + (L - x);
    elseif x >= L/2  % Right Turn
        theta = atan2(y - R, x - L/2);
        centerline_dist(i) = L + R * (theta + pi/2);
    elseif x <= -L/2  % Left Turn
        theta = atan2(y - R, x + L/2);
        centerline_dist(i) = 2*L + 2*pi*R + R * (theta - pi/2);
    end
end

completed_laps = floor(max(centerline_dist) / total_track_length);

completion_time = time(end);

out_of_bounds = false;

for i = 1:num_points
    x = X(i);
    y = Y(i);

    if x >= L/2  % Right Turn
        track_center_x = L/2 + R * cos(atan2(y - R, x - L/2));
        track_center_y = R + R * sin(atan2(y - R, x - L/2));
    elseif x <= -L/2  % Left Turn
        track_center_x = -L/2 + R * cos(atan2(y - R, x + L/2));
        track_center_y = R + R * sin(atan2(y - R, x + L/2));
    else  % Straight Sections
        track_center_x = x;
        track_center_y = (y < R) * 0 + (y >= R) * (2*R);
    end

    % Compute lateral deviation from centerline
    distance_from_center = sqrt((x - track_center_x)^2 + (y - track_center_y)^2);

    if distance_from_center > track_width / 2
        out_of_bounds = true;
        break;
    end
end

% Display Results
disp('Race Statistics:')
fprintf('Completed Laps: %d\n', completed_laps);
fprintf('Completion Time: %.2f seconds\n', completion_time);

if out_of_bounds
    fprintf('Status: Vehicle went off the track\n');
else
    fprintf('Status: Vehicle stayed on the track!\n');
end
