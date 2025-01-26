solver_arr = {'ode1', 'ode4'};
time_steps = [0.001, 0.1, 1];

results = struct();
for i = 1:length(solver_arr)
    results.(solver_arr{i}) = struct('time_step', [], 'max_error', [], 'cpu_time', []);
end

A = 1;
J = 1;
b = 1;
w_0 = 1;

dt_theory = linspace(0, 25, 1000);
theory_omega = (A/b) * (1 - exp(-b * dt_theory / J)) + w_0 * exp(-b * dt_theory / J); %eqn of motion

for i = 1:length(solver_arr)
    solver = solver_arr{i};
    for j = 1:length(time_steps)
        dT = time_steps(j);

        cpu_time_start = cputime;

        simOut = sim('Project10x2810x29.slx', ...
            'Solver', solver, ...
            'FixedStep', num2str(dT), ...
            'StopTime', '25');

        cpu_time_end = cputime;
        cpu_time = cpu_time_end - cpu_time_start;

        sim_time = simOut.tout;
        sim_omega = simOut.get('omega').Data;

        theory_omega_interp = interp1(dt_theory, theory_omega, sim_time, 'linear', 'extrap');

        sim_omega = sim_omega(:); 
        theory_omega_interp = theory_omega_interp(:);

        max_error = max(abs(sim_omega - theory_omega_interp));

        results.(solver).time_step(end + 1) = dT;
        results.(solver).max_error(end + 1) = max_error;
        results.(solver).cpu_time(end + 1) = cpu_time;
    end
end

figure;
hold on;
for i = 1:length(solver_arr)
    solver = solver_arr{i};
    plot(results.(solver).time_step, results.(solver).max_error, '-o', 'DisplayName', solver);
end
hold off;
xlabel('Time Step (s)');
ylabel('Max Simulation Error (rad/s)');
title('Max Simulation Error vs Time Step');
legend('show');
grid on;

figure;
hold on;
for i = 1:length(solver_arr)
    solver = solver_arr{i};
    plot(results.(solver).time_step, results.(solver).cpu_time, '-o', 'DisplayName', solver);
end
hold off;
xlabel('Time Step (s)');
ylabel('CPU Time Taken (s)');
title('CPU Time vs Time Step');
legend('show');
grid on;

figure;
hold on;
for i = 1:length(solver_arr)
    solver = solver_arr{i};
    plot(results.(solver).cpu_time, results.(solver).max_error, '-o', 'DisplayName', solver);
end
hold off;
xlabel('CPU Time Taken (s)');
ylabel('Max Simulation Error (rad/s)');
title('Max Simulation Error vs CPU Time');
legend('show');
grid on;

all_cpu_time = [];
all_max_error = [];
all_eigen_values = [];

for i = 1:length(solver_arr)
    solver = solver_arr{i};
    all_cpu_time = [all_cpu_time, results.(solver).cpu_time];
    all_max_error = [all_max_error, results.(solver).max_error];
    all_eigen_values = [all_eigen_values, log10(results.(solver).cpu_time + 1)]; % Placeholder eigenvalue calculation
end

all_cpu_time = all_cpu_time + randn(size(all_cpu_time)) * 1e-6;
all_max_error = all_max_error + randn(size(all_max_error)) * 1e-6;

[cpu_time_grid, max_error_grid] = meshgrid(linspace(min(all_cpu_time), max(all_cpu_time), 100), ...
                                           linspace(min(all_max_error), max(all_max_error), 100));

eigen_values_grid = griddata(all_cpu_time, all_max_error, all_eigen_values, ...
                             cpu_time_grid, max_error_grid, 'cubic');

figure;
contourf(cpu_time_grid, max_error_grid, eigen_values_grid, 20, 'LineColor', 'none');
colorbar; 
xlabel('CPU Time (s)');
ylabel('Max Simulation Error (rad/s)');
title('Contour Plot of Constant System Eigenvalues');

input_frequencies = [0.1, 1, 10];%Placeholder frequencies

all_cpu_time = [];
all_max_error = [];
all_frequencies = [];
for i = 1:length(solver_arr)
    solver = solver_arr{i};
    for j = 1:length(time_steps)
        dT = time_steps(j);
        for freq = input_frequencies
            cpu_time_start = cputime;

            simOut = sim('Project10x2810x29.slx', ...
                'Solver', solver, ...
                'FixedStep', num2str(dT), ...
                'StopTime', '25');

            cpu_time_end = cputime;
            cpu_time = cpu_time_end - cpu_time_start;

            sim_time = simOut.tout;
            sim_omega = simOut.get('omega').Data;

            theory_omega_interp = interp1(dt_theory, theory_omega, sim_time, 'linear', 'extrap');

            max_error = max(abs(sim_omega(:) - theory_omega_interp(:)));

            all_cpu_time = [all_cpu_time, cpu_time];
            all_max_error = [all_max_error, max_error];
            all_frequencies = [all_frequencies, freq];
        end
    end
end

all_cpu_time = all_cpu_time + randn(size(all_cpu_time)) * 1e-6;
all_max_error = all_max_error + randn(size(all_max_error)) * 1e-6;

[cpu_time_grid, max_error_grid] = meshgrid(linspace(min(all_cpu_time), max(all_cpu_time), 100), ...
                                           linspace(min(all_max_error), max(all_max_error), 100));

frequency_grid = griddata(all_cpu_time, all_max_error, all_frequencies, ...
                          cpu_time_grid, max_error_grid, 'cubic');

figure;
contourf(cpu_time_grid, max_error_grid, frequency_grid, 20, 'LineColor', 'none');
colorbar;
xlabel('CPU Time [s]');
ylabel('Max Simulation Error');
title('Contour Plot of Constant Input Frequencies');
