% ---------------- Vehicle Parameters ----------------
inertia = 2000;         % [kg·m²] - Vehicle yaw inertia
mass = 1500;            % [kg] - Vehicle mass
rw = 0.3;               % [m] - Wheel radius
vm = 1600;              % [kg] - (Unused duplicate of mass)
iw = 0.5 * 0.001* rw^2;    % [kg·m²] - Wheel moment of inertia (7 kg wheel)
ipt = 0.010;               % [kg·m²] - Driveline/powertrain inertia
cd = 0.35;              % [-] - Aerodynamic drag coefficient
ad = 2.5;               % [m²] - Frontal area
tau_brakes = 50000;     % [Nm] - Max brake torque
tau_motor = 800;        % [Nm] - Max drive torque
gratio = 2.5;             % [-] - Gear ratio (motor to wheel)
i_gear_i = 0.005;         % [kg·m²] - Gear input inertia
o_gear_i = 0.007;         % [kg·m²] - Gear output inertia
c0 = 0.0041;            % [N] - Rolling resistance constant
c1 = 0.000066;          % [N/(m/s)] - Speed-dependent rolling resistance


%% Battery Parameters %%
v_batt = 350;           % [V] - Nominal battery voltage
r_internal = 0.1695;    % [Ohms] - Internal resistance
eta_motor = 0.9;        % [-] - Motor efficiency
tau_motor_max = 800;    % [Nm] - Max motor torque
soc_min = 0.1;          % [-] - Minimum State of Charge
b_capacity = 1500;       % [Ah] - Battery pack capacity
tau_regen_max = 300;    % [Nm] - Max regenerative braking torque
SOC=[0, 0.01,   .1,  .2,  .3,  .4,  .5,  .6, .7,  .8,  .9,   1];
OCV=[0, 3.1,3.55,3.68,3.74,3.76,3.78,3.85,3.9,3.95,4.08,4.15]; % [V] - Open Circuit Voltage per cell
numSeries = 96;         % [-] - Series cells
numParallel = 74;       % [-] - Parallel cells

%% Environment %%
rho = 1.225;
g=9.81;
% ---------------- Motor Torque Curve ----------------
motor_rpm = [0, 1000, 2000, 3000, 4000, 5000, 6000, ...
             7000, 8000, 9000, 10000, 11000, 12000];
motor_torque = [280, 280, 275, 260, 250, 230, 200, 175, 140, 120,  100,   75,     0]*3;
motor_torque_curve = [motor_rpm', motor_torque'];
tau_motor_max = max(motor_torque);

% Vehicle Tire Info
calpha_front = 40000;                  % [N/rad] - Front cornering stiffness
calpha_rear = 40000;                   % [N/rad] - Rear cornering stiffness
fyfrontmax = calpha_front * deg2rad(1);% [N] - Max front lateral force
fyrearmax = calpha_rear * deg2rad(1);  % [N] - Max rear lateral force
lf = 1.0;                              % [m] - Distance from CG to front axle
lr = 1.5;                              % [m] - Distance from CG to rear axle
radius = 0.3;                          % [m] - (possibly duplicate of rw)
wheelbase = lf + lr;                  % [m] - Total wheelbase
c_lambda = 50; % longitudinal stiffness (N/Kg)
lambda_max = 0.1; %maximum tire slip ratio before saturation
tire_mu = 1.0;

% Initial Conditions
x0 = 0;
y0 = 0;
vx0 = 0.1;
vy0 = 0;
omega0 = 0;
psi0 = 0;

track_radius = 200;
understeerCoeff = mass / ((lr + lf) * track_radius) * (lr / calpha_front - lf / calpha_rear);
maxAlpha = 4 / 180 * pi;
max_motor_rpm = max(motor_rpm);                     % [RPM]
max_motor_radps = max_motor_rpm * 2 * pi / 60;      % [rad/s]
vxd = (max_motor_radps / gratio) * rw;              % [m/s] computed from max rpm and wheel radius
vx_threshold1 = 0.1;

%% Controller%%
kp=50000000; %0.0075
ki=100; %0.02
kd=10000000; %0.005
corner_factor=.95;
lookahead_dist = 50;
lookahead_dist_long=100;
T_lookahead=0.3; %lookahead time


% ---------------- Track Generation Parameters ----------------
radius = 200; 
l_st = 900; 
width = 15; 

l_curve = pi * radius;
total_length = 2 * l_st + 2 * l_curve;

delta_s = 1;
npts = round(total_length / delta_s);
delta_s = total_length / (npts - 1);
delta_theta = delta_s / radius;


function xyt = rotate(xy,theta)
    xyt = TF(theta) * xy;
end

function y = TF(psi)
    y = [cos(psi), -sin(psi); sin(psi), cos(psi)];
end


% Preallocate path arrays
xpath = zeros(npts,1);
ypath = zeros(npts,1);
tpath = zeros(npts,1);
xinpath = zeros(npts,1);
yinpath = zeros(npts,1);
xoutpath = zeros(npts,1);
youtpath = zeros(npts,1);

yinpath(1) = 7.5;
youtpath(1) = -7.5;

% Track generation loop
i = 1;
while i < npts
    if xpath(i) < l_st
        if xpath(i) >= 0
            if ypath(i) < radius
                xpath(i+1) = xpath(i) + delta_s;
                ypath(i+1) = ypath(i);
                xinpath(i+1) = xinpath(i) + delta_s;
                yinpath(i+1) = yinpath(i);
                xoutpath(i+1) = xoutpath(i) + delta_s;
                youtpath(i+1) = youtpath(i);
                tpath(i+1) = 0;
            else
                xpath(i+1) = xpath(i) - delta_s;
                ypath(i+1) = ypath(i);
                xinpath(i+1) = xinpath(i) - delta_s; 
                yinpath(i+1) = yinpath(i);
                xoutpath(i+1) = xoutpath(i) - delta_s; 
                youtpath(i+1) = youtpath(i);
                tpath(i+1) = pi;
            end
        else
            cx = 0; cy = radius;
            tt = rotate([xpath(i)-cx; ypath(i)-cy], delta_theta);
            ttin = rotate([xinpath(i)-cx; yinpath(i)-cy], delta_theta);
            ttout = rotate([xoutpath(i)-cx; youtpath(i)-cy], delta_theta);
            xpath(i+1) = tt(1) + cx; 
            ypath(i+1) = tt(2) + cy;
            xinpath(i+1) =  ttin(1) + cx; 
            yinpath(i+1) = ttin(2) + cy;
            xoutpath(i+1) = ttout(1) + cx; 
            youtpath(i+1) = ttout(2) + cy;
            tpath(i+1) = tpath(i) + delta_theta;
        end
    else
        cx = l_st; cy = radius;
        tt = rotate([xpath(i)-cx; ypath(i)-cy], delta_theta);
        ttin = rotate([xinpath(i)-cx; yinpath(i)-cy], delta_theta);
        ttout = rotate([xoutpath(i)-cx; youtpath(i)-cy], delta_theta);
        xpath(i+1) = tt(1) + cx; 
        ypath(i+1) = tt(2) + cy;
        xinpath(i+1) =  ttin(1) + cx; 
        yinpath(i+1) = ttin(2) + cy;
        xoutpath(i+1) = ttout(1) + cx;
        youtpath(i+1) = ttout(2) + cy;
        tpath(i+1) = tpath(i) + delta_theta;
    end
    i = i + 1;    
end
disp(psi0)

% ---------------- Run Simulation ----------------
simout = sim("Project4_Simulink_final");
car_X = simout.X.Data;
car_Y = simout.Y.Data;
car_psi = simout.psi.Data;
car_time = simout.tout;
car_velocity=simout.velocity.Data;

figure;
scatter_plot = scatter(car_X, car_Y, 15, car_velocity, 'filled'); % 15 is marker size
axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Vehicle Path Colored by Velocity');
cb = colorbar;
cb.Label.String = 'Velocity (m/s)';
grid on;
function raceStats = getRaceStats(car_X, car_Y, xpath, ypath, xinpath, yinpath, xoutpath, youtpath, car_time)
    % Estimate number of laps
    loops = 1;
    crossed = false;
    lap_times = []; % array to store times when crossing start/finish

    for i = 2:length(car_X)
        % Look for forward crossing near start line (X ≈ 0, Y ≈ 0)
        if car_X(i-1) < 0 && car_X(i) >= 0

            if ~crossed
                loops = loops + 1;
                crossed = true;
                lap_times = [lap_times; car_time(i)]; % save time of crossing
            end
        else
            crossed = false;
        end
    end

    % Track inside/outside logic
    inside = 0;
    offtrack_count = 0;
    was_outside = false;

    for i = 1:length(car_X)
        % Find closest point on centerline
        d = (car_X(i) - xpath).^2 + (car_Y(i) - ypath).^2;
        [~, idx] = min(d);

        % Compute width of track at this index
        inner_dist = norm([xinpath(idx), yinpath(idx)] - [xpath(idx), ypath(idx)]);
        outer_dist = norm([xoutpath(idx), youtpath(idx)] - [xpath(idx), ypath(idx)]);
        width_half = max(inner_dist, outer_dist);

        % Distance from car to centerline
        dist_to_center = norm([car_X(i), car_Y(i)] - [xpath(idx), ypath(idx)]);

        if dist_to_center <= width_half
            inside = inside + 1;
            was_outside = false;
        else
            if ~was_outside
                offtrack_count = offtrack_count + 1;
                was_outside = true;
            end
        end
    end

    % Calculate lap times
    if length(lap_times) >= 2
        lap_durations = diff(lap_times);
        fastest_lap = min(lap_durations);
    else
        fastest_lap = NaN; % not enough laps to measure
    end

    % Percentage of time inside track
    percent_inside = (inside / length(car_X)) * 100;

    % Output structure
    raceStats.laps_completed = loops - 1; % only count completed laps
    raceStats.percent_inside_track = percent_inside;
    raceStats.times_off_track = offtrack_count;
    raceStats.fastest_lap_time = fastest_lap; % new field!
end

raceStats = getRaceStats(car_X, car_Y, xpath, ypath, xinpath, yinpath, xoutpath, youtpath, car_time);
laps=round(3600/raceStats.fastest_lap_time);%recuerda borrar, nada mas para estimate
fprintf("Laps Completed: %d\n", laps);
%fprintf("Pctg Inside Track: %.2f%%\n", raceStats.percent_inside_track);
fprintf("Times Went Off Track: %d\n", raceStats.times_off_track);
fprintf("Fastest Lap: %.2f seconds%\n", raceStats.fastest_lap_time);
% ---------------- Animate Vehicle ----------------
fh = figure();
fh.WindowState = 'maximized';
hold on
plot(xpath, ypath, '--r'); axis equal;
plot(xinpath, yinpath, 'b');
plot(xoutpath, youtpath,'b');
axis([min(xoutpath), max(xoutpath), min(youtpath), max(youtpath)])
xlabel('X Distance (m)')
ylabel('Y Distance (m)')
title('Project 2 Track')
grid on
h = animatedline;

L = 15;
width = 5;

% Initial car shape
car = [-L/2 -width/2; -L/2 width/2; L/2 width/2; L/2 -width/2];
rcar = rotate(car', car_psi(1))';
shape = polyshape(rcar + [car_X(1), car_Y(1)]);
ap = plot(shape, 'FaceColor', 'k');

for i = 1:length(car_X)
    x = car_X(i);
    y = car_Y(i);
    psi = car_psi(i);
    
    addpoints(h,x,y)

    % Update car shape
    rcar = rotate(car', psi)';
    shape.Vertices = rcar + [x, y];
    ap.Shape = shape;

    drawnow limitrate
end

