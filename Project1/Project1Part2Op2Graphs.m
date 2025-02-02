clear; clc; close all;

step_sizes = [0.001, 0.1, 1.0];  
solvers = {'ode1', 'ode4', 'ode45', 'ode23tb'};  
initial_conditions = [10, 0];  
torques = [0, 100];  

solver_labels = struct('ode1', 'Euler', ...
                       'ode4', 'RK4', ...
                       'ode45', 'ODE45', ...
                       'ode23tb', 'ODE23tb');

model_name = 'Project1Part2Op2';  
load_system(model_name);

results = [];  

for ic = initial_conditions
    for torque = torques
        for solver = solvers
            for dt = step_sizes

                assignin('base', 'J1', 100);
                assignin('base', 'J2', 1);
                assignin('base', 'B1', 1);
                assignin('base', 'B2', 1);

                assignin('base', 'w_0', ic);
                assignin('base', 'A', double(torque));

                set_param(model_name, 'Solver', solver{1});
                set_param(model_name, 'FixedStep', num2str(dt));
                set_param(model_name, 'StartTime', '0', 'StopTime', '25');

                tic;
                simOut = sim(model_name);
                cpu_time = toc;

                omega = simOut.get('OmegaOp2').Data;  
                time = simOut.get('OmegaOp2').Time;

                results = [results; struct('IC', ic, 'Torque', torque, 'Solver', solver{1}, ...
                                          'StepSize', dt, 'CPUTime', cpu_time, 'Time', time, 'Omega', omega)];
            end
        end
    end
end

save('simulation_results.mat', 'results');  
load('simulation_results.mat');

figure; hold on;
legendEntries = {};
plotEvery = 5; 

for i = 1:length(results)
    if mod(i, plotEvery) == 0  
        plot(results(i).Time, results(i).Omega, 'LineWidth', 1.5);
        solverName = solver_labels.(results(i).Solver);  
        legendEntries{end+1} = sprintf('%s (dt=%.3f)', solverName, results(i).StepSize);
    else
        plot(results(i).Time, results(i).Omega, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
end

xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
title('Angular Velocity Response for Different Solvers');
legend(legendEntries, 'Location', 'best');
grid on;
hold off;

figure; hold on;
solversList = unique({results.Solver});
legendEntries = {};

for s = solversList
    idx = strcmp({results.Solver}, s{1});
    dt_values = [results(idx).StepSize];
    cpu_times = [results(idx).CPUTime];
    solverName = solver_labels.(s{1});  
    plot(dt_values, cpu_times, '-o', 'LineWidth', 1.5);
    legendEntries{end+1} = solverName;
end

xlabel('Step Size (s)'); ylabel('CPU Time (s)');
title('CPU Time vs. Step Size for Different Solvers');
legend(legendEntries, 'Location', 'northwest');
grid on;
hold off;

figure; hold on;
legendEntries = {};

for s = solversList
    idx = strcmp({results.Solver}, s{1});
    dt_values = [results(idx).StepSize];

    ref_idx = find(strcmp({results.Solver}, 'ode4') & [results.StepSize] == 0.001);
    omega_ref = results(ref_idx).Omega;
    time_ref = results(ref_idx).Time;
    
    max_errors = arrayfun(@(res) max(abs(interp1(time_ref, omega_ref, res.Time) - res.Omega)), results(idx));

    solverName = solver_labels.(s{1});  
    plot(dt_values, max_errors, '-o', 'LineWidth', 1.5);
    legendEntries{end+1} = solverName;
end

xlabel('Step Size (s)'); ylabel('Max Simulation Error');
title('Max Simulation Error vs. Step Size');
legend(legendEntries, 'Location', 'northwest');
grid on;
hold off;

disp('Done');
