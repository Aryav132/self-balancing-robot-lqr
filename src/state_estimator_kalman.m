%% STATE_ESTIMATOR_KALMAN  LQG-style control: LQR + Kalman filter state estimation
%
% Everything up to this point assumed full, perfect state feedback:
% u = -K*x, where x is the TRUE state. Real robots don't get that — an
% IMU gives a noisy angle measurement, a wheel encoder gives a noisy
% position measurement, and velocities aren't measured directly at all.
%
% This script replaces the "cheat" of perfect state feedback with a
% realistic sensor model + a Kalman filter estimator, then closes the
% loop on the ESTIMATE: u = -K*x_hat, not u = -K*x_true.
%
% Sensor model: position (wheel encoder) and angle (IMU) are measured,
% each with realistic noise. Velocity and angular velocity are NOT
% measured directly — the Kalman filter has to infer them.

clear; clc; close all;

%% 1. System parameters (same as two_wheeled_robot_simulation.m)
params.M = 1.5; params.m = 0.2; params.R = 0.05; params.l = 0.15;
params.Ib = 0.012; params.Iw = 0.0002; params.g = 9.81; params.b = 0.05;

p_den = params.Ib*(params.M + params.m) + params.M*params.m*params.l^2;
A32 = (params.M^2 * params.g * params.l^2) / p_den;
A42 = (params.M * params.g * params.l * (params.M + params.m)) / p_den;

A = [0, 1, 0, 0;
     0, -params.b/params.M, -A32, 0;
     0, 0, 0, 1;
     0, params.b/(params.M*params.l), A42, 0];
B = [0; 1/(params.M*params.R); 0; -1/(params.M*params.l*params.R)];

%% 2. LQR controller (same design as before)
Q = diag([10, 2, 50, 5]);
Rlqr = 0.8;
K = lqr(A, B, Q, Rlqr);

%% 3. Sensor model — this is the new, realistic part
% Measured outputs: [position; angle]. Velocity and angular velocity are
% NOT directly measured, which is realistic for a cheap encoder+IMU setup.
C = [1 0 0 0;
     0 0 1 0];

% Measurement noise: how noisy are the actual sensors?
%   position (wheel encoder): +-2mm-ish std dev -> variance ~ (0.002)^2
%   angle (IMU):              +-0.3 deg-ish std dev -> variance in rad^2
pos_noise_std   = 0.002;              % m
angle_noise_std = deg2rad(0.3);       % rad
Rn = diag([pos_noise_std^2, angle_noise_std^2]);

% Process noise: unmodeled disturbances (wind, floor bumps, motor
% ripple) entering as if they were small random accelerations.
%
% RETUNED: velocity and angular rate are NOT directly measured (C only
% sees position and angle) -- they are purely inferred by propagating
% the LINEAR model forward. Once the true nonlinear plant drifts even
% slightly from that linear model (which it will, once angle isn't
% tiny), that drift has no direct measurement to correct against and
% can accumulate silently, eventually destabilizing the loop. Setting
% these process-noise terms much higher tells the filter "trust your
% own propagation of these two states much less" -- forcing it to lean
% harder on what the position/angle corrections imply about them,
% instead of blindly trusting the (increasingly wrong, at large angles)
% linear model. Values below were increased ~100-500x from the
% under-tuned original.
Qn = diag([0, 5e-2, 0, 5e-1]);

% Steady-state Kalman filter gain (continuous-time), via the standard
% Linear Quadratic Estimator formulation. G=eye(4) means process noise
% is assumed to enter directly on each state derivative.
[L, ~, ~] = lqe(A, eye(4), C, Qn, Rn);

fprintf('--- Kalman filter gain L ---\n');
disp(L);
fprintf('Estimator poles (should all be stable, left half-plane):\n');
disp(eig(A - L*C));

%% 4. Discrete-time joint simulation: TRUE nonlinear plant + noisy
%    sensors + Kalman estimator + LQR control on the ESTIMATE
dt = 0.005;
T_final = 10;
N = round(T_final/dt);

x_ref = [0.5; 0; 0; 0];
x0_true = [0; 0; deg2rad(15); 0];   % moderate 15-degree initial tilt for this test

x_true = zeros(N,4); x_hat = zeros(N,4); y_meas = zeros(N,2); u_hist = zeros(N,1);
x_true(1,:) = x0_true;

rng(1);  % reproducible noise

% Realistic estimator initialization: take ONE sensor reading before
% control starts, and seed position/angle from it. Velocity/angular
% rate are unknown at t=0 (no prior reading to differentiate), so those
% start at zero. This is what a real system does — it does NOT start by
% blindly assuming "definitely upright, definitely stationary" with zero
% basis, which is what caused the estimator to lose a race against this
% system's fast (~11.8 rad/s) open-loop instability and diverge.
y0 = C*x0_true + [pos_noise_std; angle_noise_std] .* randn(2,1);
x_hat0 = [y0(1); 0; y0(2); 0];
x_hat(1,:) = x_hat0;

nonlinear_deriv = @(xt, u) local_robot_deriv(xt, u, params);

for k = 1:N-1
    xt = x_true(k,:)';
    xh = x_hat(k,:)';

    % Control uses the ESTIMATE, not the true state
    u = -K * (xh - x_ref);
    u_hist(k) = u;

    % --- True nonlinear plant, one RK4 step (much more accurate than
    %     Euler near this system's fast instability), plus process noise ---
    k1 = nonlinear_deriv(xt, u);
    k2 = nonlinear_deriv(xt + dt/2*k1, u);
    k3 = nonlinear_deriv(xt + dt/2*k2, u);
    k4 = nonlinear_deriv(xt + dt*k3, u);
    dx_true = (k1 + 2*k2 + 2*k3 + k4)/6;

    process_noise = sqrt(dt) * sqrt([Qn(1,1); Qn(2,2); Qn(3,3); Qn(4,4)]) .* randn(4,1);
    x_true(k+1,:) = (xt + dt*dx_true + process_noise)';

    if any(~isfinite(x_true(k+1,:)))
        warning('True state went non-finite at step %d (t=%.3fs) — system diverged. Stopping early.', k, k*dt);
        x_true = x_true(1:k+1,:); x_hat = x_hat(1:k+1,:); t_stop_idx = k+1;
        break;
    end

    % --- Noisy sensor measurement of the TRUE state ---
    y = C*xt + [pos_noise_std; angle_noise_std] .* randn(2,1);
    y_meas(k,:) = y';

    % --- Kalman filter estimator update (runs on the LINEAR model —
    %     the estimator itself doesn't know the true nonlinear dynamics,
    %     which is realistic: you design an estimator from your best
    %     linear model, same as the controller) ---
    xhat_dot = A*xh + B*u + L*(y - C*xh);
    x_hat(k+1,:) = (xh + dt*xhat_dot)';
end

if ~exist('t_stop_idx', 'var')
    t_stop_idx = N;
end
x_true = x_true(1:t_stop_idx,:);
x_hat = x_hat(1:t_stop_idx,:);
N_actual = t_stop_idx;

%% 5. Metrics
pos_err = x_true(:,1) - x_hat(:,1);
angle_err_deg = rad2deg(x_true(:,3) - x_hat(:,3));
t = (0:N-1)*dt;

fprintf('\n--- Estimation performance ---\n');
fprintf('RMS position estimation error:  %.4f m\n', rms(pos_err));
fprintf('RMS angle estimation error:     %.4f deg\n', rms(angle_err_deg));
fprintf('Final true angle: %.3f deg | Final estimated angle: %.3f deg\n', ...
    rad2deg(x_true(end,3)), rad2deg(x_hat(end,3)));
fprintf('Final true position: %.3f m | Final estimated position: %.3f m\n', ...
    x_true(end,1), x_hat(end,1));

%% 6. Plots: true vs estimated state, and estimation error
figure('Name', 'LQG: True State vs Kalman Estimate', 'Color', 'w');

subplot(4,1,1);
plot(t, x_true(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, x_hat(:,1), 'r--', 'LineWidth', 1.2);
ylabel('Position (m)'); legend('True', 'Estimated', 'Location', 'best');
title('True vs Kalman-Estimated State (measured: position + angle only)');
grid on;

subplot(4,1,2);
plot(t, rad2deg(x_true(:,3)), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(x_hat(:,3)), 'r--', 'LineWidth', 1.2);
ylabel('Angle (deg)'); grid on;

subplot(4,1,3);
plot(t, x_true(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(t, x_hat(:,2), 'r--', 'LineWidth', 1.2);
ylabel('Velocity (m/s)');
title('Velocity/angular rate are NOT directly measured — pure Kalman inference');
grid on;

subplot(4,1,4);
plot(t, x_true(:,4), 'b-', 'LineWidth', 1.5); hold on;
plot(t, x_hat(:,4), 'r--', 'LineWidth', 1.2);
ylabel('Angular rate (rad/s)'); xlabel('Time (s)'); grid on;

fprintf('\nDone. Compare the solid (true) vs dashed (estimated) lines above,\n');
fprintf('especially velocity/angular rate, which are inferred, not measured.\n');

%% Local nonlinear dynamics (same physics as two_wheeled_robot_simulation.m's
%% robot_dynamics, minus the control law itself -- u is passed in already)
function dx = local_robot_deriv(x, u, params)
    theta = x(3); thetadot = x(4);
    Sx = sin(theta); Cx = cos(theta);
    denom = params.Ib + params.M*params.l^2 * (1 - Cx^2);
    xddot = (u/params.R - params.b*x(2) + params.M*params.l*params.g*Sx*Cx ...
            - params.M*params.l*thetadot^2*Sx) / (params.M + params.m);
    thetaddot = (params.M*params.g*params.l*Sx - params.M*params.l*xddot*Cx) / denom;
    dx = [x(2); xddot; x(4); thetaddot];
end
