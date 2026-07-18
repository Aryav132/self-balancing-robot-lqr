%% Two-Wheeled Self-Balancing Robot LQR Control Simulation
clear; clc; close all;

%% 1. System Parameters
params.M = 1.5;     % Mass of the robot chassis (kg)
params.m = 0.2;     % Mass of the wheels combined (kg)
params.R = 0.05;    % Radius of the wheels (m)
params.l = 0.15;    % Distance from wheel axle to chassis center of mass (m)
params.Ib = 0.012;  % Moment of inertia of the robot chassis (kg·m^2)
params.Iw = 0.0002; % Moment of inertia of the wheels (kg·m^2)
params.g = 9.81;    % Acceleration due to gravity (m/s^2)
params.b = 0.05;    % Viscous friction coefficient at the wheel axle (N·s/m)

%% 2. State-Space Linearization (About upright equilibrium: theta = 0)
% State vector x = [position; velocity; pitch_angle; pitch_rate]
% Input u = Torque applied by motors (N·m)

p_den = params.Ib*(params.M + params.m) + params.M*params.m*params.l^2;

A32 = (params.M^2 * params.g * params.l^2) / p_den;
A42 = (params.M * params.g * params.l * (params.M + params.m)) / p_den;

A = [0, 1, 0, 0;
     0, -params.b/params.M, -A32, 0;
     0, 0, 0, 1;
     0, params.b/(params.M*params.l), A42, 0];

B = [0; 
     1 / (params.M * params.R); 
     0; 
     -1 / (params.M * params.l * params.R)];

%% 3. LQR Controller Design
Q = diag([10, 2, 50, 5]); 
R = 0.8;

K = lqr(A, B, Q, R);

fprintf('--- LQR Control Configuration ---\n');
fprintf('Calculated Feedback Gains K: [%.4f, %.4f, %.4f, %.4f]\n', K);

%% 4. Closed-Loop Simulation Configuration
tspan = 0:0.01:20;      % up to 20s of headroom — event below will stop it early if it balances
x_ref = [0.5; 0; 0; 0]; % Target: Move 0.5 meters forward and stay upright

% Initial State: tilted back by 50 degrees (0.8727 rad).
% WARNING: this is a LARGE initial tilt. LQR here is designed from a
% LINEARIZED model around theta=0 (small-angle assumption, sin(theta)~theta).
% Past roughly 30-35 degrees for this parameter set, that linearization
% stops being a good approximation of the real nonlinear dynamics. At 50
% degrees there is a real chance the controller cannot recover at all —
% the "balance event" below may simply never fire. That's an honest
% possible outcome, not a bug.
tilt_deg = 50;
x0 = [0; 0; deg2rad(tilt_deg); 0];

%% 5. Event function: stop integration once the robot is actually balanced
% "Balanced" = angle, angular velocity, and cart velocity all small AND
% staying small (checked via a short settle window isn't possible inside
% a single event call, so we use tight thresholds instead as a practical
% proxy for "settled").
angle_tol = deg2rad(0.5);   % within 0.5 degree of upright
rate_tol  = 0.02;           % rad/s
vel_tol   = 0.02;           % m/s

options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, ...
    'Events', @(t,x) balance_event(t, x, x_ref, angle_tol, rate_tol, vel_tol));

[t, x, te, xe, ie] = ode45(@(t, x) robot_dynamics(t, x, K, x_ref, params), tspan, x0, options);

balanced = ~isempty(te);
if balanced
    fprintf('\nRobot balanced at t = %.2f s (started at %d degrees tilt).\n', te(end), tilt_deg);
else
    fprintf('\nRobot did NOT balance within %.1f s (started at %d degrees tilt).\n', tspan(end), tilt_deg);
    fprintf('Final angle: %.2f deg | Final velocity: %.3f m/s -> controller failed to recover.\n', ...
        rad2deg(x(end,3)), x(end,2));
end

%% 6. Compute Control Effort Post-Simulation
u = zeros(length(t), 1);
for i = 1:length(t)
    u(i) = -K * (x(i, :)' - x_ref);
    if t(i) >= 3.0 && t(i) <= 3.1
        u(i) = u(i) + 1.5; % external disturbance push
    end
end

%% 7. Performance Visualizations
figure('Name', 'Self-Balancing Robot Performance Metrics', 'Color', 'w');

subplot(3, 1, 1);
plot(t, x(:, 1), 'b-', 'LineWidth', 2); hold on;
yline(x_ref(1), 'r--', 'Target Position', 'LineWidth', 1.2);
ylabel('Position (m)');
if balanced
    title(sprintf('Two-Wheeled Robot Response — balanced at t=%.2fs (start tilt %d°)', te(end), tilt_deg));
else
    title(sprintf('Two-Wheeled Robot Response — DID NOT BALANCE (start tilt %d°)', tilt_deg));
end
grid on;

subplot(3, 1, 2);
plot(t, rad2deg(x(:, 3)), 'g-', 'LineWidth', 2); hold on;
xline(3.0, 'k:', 'External Disturbance Push', 'LabelVerticalAlignment', 'bottom');
ylabel('Tilt Angle (deg)');
grid on;

subplot(3, 1, 3);
plot(t, u, 'm-', 'LineWidth', 2);
ylabel('Motor Torque (N·m)');
xlabel('Time (s)');
grid on;

fprintf('Max |angle| during simulation: %.2f deg\n', max(abs(rad2deg(x(:,3)))));
fprintf('Max |torque| during simulation: %.3f N·m\n', max(abs(u)));
fprintf('Final angle: %.3f deg | Final position: %.3f m\n', rad2deg(x(end,3)), x(end,1));

%% 8. Non-Linear Dynamics Differential Equation Function
function dx = robot_dynamics(t, x, K, x_ref, params)
    u = -K * (x - x_ref);
    if t >= 3.0 && t <= 3.1
        u = u + 1.5;
    end

    theta = x(3);
    thetadot = x(4);
    Sx = sin(theta);
    Cx = cos(theta);

    denom = params.Ib + params.M*params.l^2 * (1 - Cx^2);

    xddot = (u/params.R - params.b*x(2) + params.M*params.l*params.g*Sx*Cx ...
            - params.M*params.l*thetadot^2*Sx) / (params.M + params.m);

    thetaddot = (params.M*params.g*params.l*Sx - params.M*params.l*xddot*Cx) / denom;

    dx = zeros(4, 1);
    dx(1) = x(2);
    dx(2) = xddot;
    dx(3) = x(4);
    dx(4) = thetaddot;
end

%% 9. Event function: fires (value crosses zero) when the robot is balanced
function [value, isterminal, direction] = balance_event(~, x, x_ref, angle_tol, rate_tol, vel_tol)
    angle_err = abs(x(3));
    rate_err  = abs(x(4));
    vel_err   = abs(x(2) - x_ref(2));

    % "value" must cross zero to trigger. We define it as the amount by
    % which the WORST of the three errors exceeds its tolerance; it
    % crosses zero exactly when all three conditions are simultaneously
    % satisfied.
    value = max([angle_err - angle_tol, rate_err - rate_tol, vel_err - vel_tol]);
    isterminal = 1;  % stop integration when triggered
    direction = -1;  % only trigger going from positive (not balanced) to negative (balanced)
end
