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
tau_brakes=1500; % Nm, max braking torque
tau_motor=800; %Nm
gratio=2; %2:1 gear ratio
i_gear_i=0.5; %kg·m^2, input gear inertia
o_gear_i=0.7; %kg·m^2, output gear inertia

%% Battery Parameters %%
 v_batt = 350;         % Battery voltage (V)
 r_internal = 0.05;    % Internal resistance (ohms)
 eta_motor = 0.9;      % Motor efficiency
 tau_motor_max = 800;  % Max motor torque (Nm)
 soc_min = 0.1;        % Minimum allowed SOC for discharge
 b_capacity=50; %amp hours
 tau_regen_max=300; %n-m
%% Environment %%
rho = 1.293; % kg/m^3, air density

%% Motor Torque Curve - Scaled to 800 Nm Max
motor_rpm = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000];
motor_torque = [800.00, 800.00, 785.71, 742.86, 714.29, 657.14, ...
                571.43, 500.00, 400.00, 342.86, 285.71, 214.29, 0.00];

motor_torque_curve = [motor_rpm', motor_torque'];
tau_motor_max = max(motor_torque);  % Still 800 Nm


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
energy_signal = simOut.energy_consumption; % Access timeseries object
time_energy = energy_signal.Time; % Extract time vector
energy_consumed = energy_signal.Data; % Extract energy consumption data

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
legend("Actual Velocity", "Target Velocity", "+3 mph", "-3 mph");
grid on;

%% Energy Consumed vs. Time %%
figure;
plot(time, energy_consumed, 'b', 'LineWidth', 2); hold on;

xlabel("Time (s)");
ylabel("Energy Consumed (J)");
title("Energy Consumed Over Time");
grid on;

%% Battery Percentage vs. Time %%
battery_signal = simOut.battery_pctg;  % From 'To Workspace' block
time_battery = battery_signal.Time;
battery_pctg = battery_signal.Data;

figure;
plot(time_battery, battery_pctg * 100, 'g', 'LineWidth', 2);  % Convert from fraction to %
xlabel("Time (s)");
ylabel("Battery State of Charge (%)");
title("Battery Percentage Over Time");
grid on;
