clc; clear; close all;

%% EPA DATA %%
highway_data = readtable("Highway test.xlsx", 'ReadVariableNames', false);
urban_data = readtable("Urban Driving.xlsx");

user_input = input('If you want to run highway test say yes, if you want to run urban driving test say no: ', 's'); if strcmpi(user_input, 'yes'), disp('Running highway test...'); elseif strcmpi(user_input, 'no'), disp('Running urban driving test...'); else, error('Invalid input. Please enter "yes" or "no".'); end
if strcmpi(user_input, 'yes')
    l_time = str2double(highway_data{3:end, 1});  
    l_vel = str2double(highway_data{3:end, 2});
    l_time = l_time(2:end);
    l_vel = l_vel(2:end) * 0.44704;

elseif strcmpi(user_input, 'no')
    l_time = str2double(urban_data{3:end, 1});  
    l_vel = str2double(urban_data{3:end, 2});

    l_time = l_time(2:end);
    l_vel = l_vel(2:end) * 0.44704;

else
    error('Invalid input. Please enter "yes" for highway test or "no" for urban test.');
end

if length(l_time) < 2 || length(l_vel) < 2
    error('Insufficient data points in l_time or l_vel. Check the data file.');
end



% Function to get reference speed at any time
v_ref = @(t) interp1(l_time, l_vel, t, 'linear', 'extrap');


%% Car Parameters %%
rw = 0.48/2; % m, wheel radius
vm = 1600; % kg, vehicle mass
iw = 9; % kg·m^2, wheel inertia
ipt = 10; % kg·m^2, driveline inertia
cd = 0.34; % coefficient of drag
ad = 2.25; % m^2, frontal area
tau_max = 1600; % Nm, max torque

%% Environment %%
rho = 1.293; % kg/m^3, air density

%% Controller Gains %%
kp=0.0075; %0.0075
ki=.02; %0.02
kd=.005; %0.005
%% Run Simulink Model %%
simOut = sim("Project3Week1_notireslip");

%% Extract Simulation Data %%
actual_v_dataset = simOut.actual_v; % Get dataset
actual_signal = actual_v_dataset.get(1); % Extract first signal
time = actual_signal.Values.Time; % Extract time vector
actual_velocity = actual_signal.Values.Data; % Extract velocity data

% Compute target velocity for the same time values
target_velocity = v_ref(time);



%% Actual vs. Target Velocity with ±3 mph Bounds %%
figure;
plot(time, actual_velocity, 'b', 'LineWidth', 2); hold on;
plot(time, target_velocity, 'r--', 'LineWidth', 2);
plot(time, target_velocity + 1.35, 'k--', 'LineWidth', 1); % Upper bound (+3 m/s)
plot(time, target_velocity - 1.35, 'k--', 'LineWidth', 1); % Lower bound (-3 m/s)

xlabel("Time (s)");
ylabel("Velocity (m/s)");
title("Actual vs. Target Velocity with ±1.35 m/s (3 mph) Bounds");
legend("Actual Velocity", "Target Velocity", "+3 m/s", "-3 m/s");
grid on;
