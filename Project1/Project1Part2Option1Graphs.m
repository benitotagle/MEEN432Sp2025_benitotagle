clear; clc; close all;

% ===========================
% DEFINE SIMULATION PARAMETERS
% ===========================
step_sizes = [0.001, 0.1, 1.0];  % Step sizes to test
solvers = {'ode1', 'ode4', 'ode45', 'ode23tb'}; % Solver methods
initial_conditions = [10, 0];  % Initial angular velocities
torques = [0, 100]; % Only constant torques (sinusoidal was removed)
k_values = [10, 100, 1000]; % New variable k with values 10, 100, 1000

% Load Simulink Model
model_name = 'Project2Option1'; % No .slx extension
load_system(model_name);

results = [];  % Store simulation results

% ===========================
% RUN SIMULATIONS
% ===========================
for ic = initial_conditions
    for torque = torques
        for solver = solvers
            for dt = step_sizes
                for k = k_values  % Loop over values of k

                    % Ensure these parameters exist in the Simulink workspace before each run
                    assignin('base', 'J1', 100);
                    assignin('base', 'J2', 1);
                    assignin('base', 'b1', 1);
                    assignin('base', 'b2', 1);
                    assignin('base', 'k', k);  % Assign k to Simulink model

                    % Assign initial conditions and torque
                    assignin('base', 'w_0', ic);
                    assignin('base', 'A', double(torque));

                    % Set simulation parameters
                    set_param(model_name, 'Solver', solver{1});
                    set_param(model_name, 'FixedStep', num2str(dt));
                    set_param(model_name, 'StartTime', '0', 'StopTime', '25');

                    % Run simulation
                    tic;
                    simOut = sim(model_name); % Note: Use model_name here, not "Project2Option1.slx"
                    cpu_time = toc;

                    % Extract outputs (handling timeseries format)
                    omega = simOut.get('omega1').Data;  
                    time = simOut.get('omega1').Time;

                    % Store results in a structured array
                    results = [results; struct('IC', ic, 'Torque', torque, 'Solver', solver{1}, ...
                                              'StepSize', dt, 'K', k, 'CPUTime', cpu_time, 'Time', time, 'Omega', omega)];
                end
            end
        end
    end
end

save('simulation_results.mat', 'results'); % Save for further use

% ===========================
% ANALYSIS AND PLOTTING
% ===========================

% Load results (if needed in future runs)
load('simulation_results.mat');

% ---- PLOT 1: Angular Velocity vs. Time ----
figure; hold on;
legendEntries = {};

for i = 1:length(results)
    plot(results(i).Time, results(i).Omega, 'LineWidth', 1.5);
    legendEntries{end+1} = sprintf('%s (dt=%.3f, k=%d)', results(i).Solver, results(i).StepSize, results(i).K);
end

xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
title('Angular Velocity Response for Different Solvers');
legend(legendEntries, 'Location', 'best');
grid on;
hold off;

% ---- PLOT 2: CPU Time vs. Step Size ----
figure; hold on;
solversList = unique({results.Solver});

for s = solversList
    idx = strcmp({results.Solver}, s{1});
    dt_values = [results(idx).StepSize];
    cpu_times = [results(idx).CPUTime];
    plot(dt_values, cpu_times, '-o', 'LineWidth', 1.5);
end

xlabel('Step Size (s)'); ylabel('CPU Time (s)');
title('CPU Time vs. Step Size for Different Solvers');
legend(solversList, 'Location', 'northwest');
grid on;
hold off;

% ---- PLOT 3: Max Error vs. Step Size ----
figure; hold on;
for s = solversList
    idx = strcmp({results.Solver}, s{1});
    dt_values = [results(idx).StepSize];

    % Compute max error (using ode4 with dt=0.001 as reference)
    ref_idx = find(strcmp({results.Solver}, 'ode4') & [results.StepSize] == 0.001);
    omega_ref = results(ref_idx).Omega;
    time_ref = results(ref_idx).Time;
    
    max_errors = arrayfun(@(res) max(abs(interp1(time_ref, omega_ref, res.Time) - res.Omega)), results(idx));

    plot(dt_values, max_errors, '-o', 'LineWidth', 1.5);
end

xlabel('Step Size (s)'); ylabel('Max Simulation Error');
title('Max Simulation Error vs. Step Size');
legend(solversList, 'Location', 'northwest');
grid on;
hold off;

disp('Simulation and Analysis Complete. All Plots Generated.');
