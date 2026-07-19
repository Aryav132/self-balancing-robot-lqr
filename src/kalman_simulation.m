%% KALMAN FILTER SIMULATION WITH MOTOR SATURATION (kalman_simulation.m)
% LQG-style control: LQR + Kalman state estimation + realistic torque
% saturation, tested against a 50-degree initial tilt.
%
% Two fixes vs. the first draft of this script:
%   1. The estimator is seeded from a real first sensor reading, not a
%      blind [0;0;0;0] guess -- otherwise it loses a race against this
%      system's fast (~11.8 rad/s) open-loop instability before it can
%      learn the true tilt, and diverges.
%   2. The TRUE plant uses the actual nonlinear dynamics, not the linear
%      A,B model. The linear model is only valid for small angles
%      (sin(theta) ~ theta) -- using it as "truth" at 50 degrees isn't
%      actually testing recovery from 50 degrees, it's testing a linear
%      system that's mislabeled as being at 50 degrees. The ESTIMATOR
%      still correctly uses the linear model internally (realistic: you
%      design a Kalman filter from your best linear model of the plant).

if ~exist('A', 'var') || ~exist('B', 'var') || ~exist('K', 'var') || ~exist('params', 'var')
    error('Workspace is empty. Run two_wheeled_robot_simulation.m first (need A, B, K, params).');
end

%% --- 1. Kalman Filter Setup ---
C_measure = [1 0 0 0; 0 0 1 0];   % measure position + angle only, same as state_estimator_kalman.m
% RETUNED: velocity/angular rate are unmeasured and purely inferred from
% the linear model -- under-trusting that inference (higher process
% noise here) makes the filter lean harder on position/angle
% corrections instead of accumulating silent drift over a long horizon.
% See state_estimator_kalman.m for the full explanation.
W = diag([0, 5e-2, 0, 5e-1]);    % process noise (continuous-time intensity)
V = diag([(0.002)^2, deg2rad(0.3)^2]);  % measurement noise: 2mm encoder, 0.3deg IMU
L = lqr(A', C_measure', W, V)';   % Kalman gain via the LQR/estimator duality

fprintf('--- Kalman filter gain L ---\n'); disp(L);
fprintf('Estimator poles: '); disp(eig(A - L*C_measure)');

%% --- 2. Simulation Setup ---
dt = 0.005;
T_end = 8;
t = 0:dt:T_end;
N = length(t);
x_true = zeros(4, N);
x_est  = zeros(4, N);
u_hist = zeros(1, N);

% Initial Conditions
x_true(:,1) = [0; 0; deg2rad(50); 0];  % Massive 50-degree drop -- real nonlinear plant now

if ~exist('x_ref', 'var')
    x_ref = [0.5; 0; 0; 0];
end

rng(1);

% Realistic estimator init: take ONE real (noisy) sensor reading of the
% TRUE initial state before control starts, instead of assuming "upright,
% stationary" with zero basis.
y0 = C_measure * x_true(:,1) + sqrt(V) * randn(2,1);
x_est(:,1) = [y0(1); 0; y0(2); 0];

%% --- PHASE 2: PHYSICAL LIMITS ---
u_max = 10;  % Hard limit on motor torque (N.m)

%% --- 3. The Dual-Universe Loop ---
for k = 1:(N-1)
    % A. Read Noisy Sensors (of the TRUE state)
    y_noisy = C_measure * x_true(:,k) + sqrt(V) * randn(2,1);

    % B. Calculate Control Torque from the ESTIMATE
    u_raw = -K * (x_est(:,k) - x_ref);

    % --- SATURATION CLAMP --- (same u drives both the real plant and the estimator)
    u = max(min(u_raw, u_max), -u_max);
    u_hist(k) = u;

    % C. Advance TRUE physics -- nonlinear model, RK4 for accuracy near
    %    this system's fast instability (plain Euler is fragile here)
    xt = x_true(:,k);
    f = @(xx) local_robot_deriv(xx, u, params);
    k1 = f(xt);
    k2 = f(xt + dt/2*k1);
    k3 = f(xt + dt/2*k2);
    k4 = f(xt + dt*k3);
    dx_true = (k1 + 2*k2 + 2*k3 + k4)/6;
    process_noise = sqrt(dt) * sqrt([W(1,1); W(2,2); W(3,3); W(4,4)]) .* randn(4,1);
    x_true(:,k+1) = x_true(:,k) + dx_true*dt + process_noise;

    if any(~isfinite(x_true(:,k+1)))
        warning('True state diverged at t=%.3fs. Stopping early.', k*dt);
        x_true = x_true(:,1:k+1); x_est = x_est(:,1:k+1); u_hist = u_hist(1:k+1);
        t = t(1:k+1); N = k+1;
        break;
    end

    % D. Advance Estimator (linear model -- this is what the estimator
    %    itself is designed from, it does not "know" the real nonlinear dynamics)
    dx_est = A * x_est(:,k) + B * u + L * (y_noisy - C_measure * x_est(:,k));
    x_est(:,k+1) = x_est(:,k) + dx_est * dt;
end

%% --- 4. Format Data for 3D Visualization ---
x = x_true';
u_hist = u_hist';

fprintf('\nFinal true angle: %.2f deg | Final estimated angle: %.2f deg\n', ...
    rad2deg(x_true(3,end)), rad2deg(x_est(3,end)));
fprintf('Final true position: %.3f m | target: %.3f m\n', x_true(1,end), x_ref(1));
fprintf('Max |torque| commanded: %.3f N.m (saturation limit: %.1f N.m)\n', max(abs(u_hist)), u_max);

%% --- 5. Plot True vs. Estimated Performance ---
figure('Name', 'Kalman Filter Performance');
plot(t, rad2deg(x_true(3,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(x_est(3,:)), 'r--', 'LineWidth', 1.5);
title('Tilt Angle: True Physical Robot vs. Estimator Belief (50-degree drop, nonlinear plant)');
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('True Physical Angle', 'Kalman Estimate', 'Location', 'best');
grid on;

%% --- 6. Plot Motor Saturation ---
figure('Name', 'Motor Torque');
plot(t, u_hist, 'm-', 'LineWidth', 1.5); hold on;
yline(u_max, 'k--', 'Max Torque (+10)', 'LineWidth', 1.5);
yline(-u_max, 'k--', 'Min Torque (-10)', 'LineWidth', 1.5);
title('Motor Output: Hitting the Saturation Limits');
xlabel('Time (s)'); ylabel('Torque (N\cdotm)');
ylim([-u_max-5, u_max+5]);
grid on;

fprintf('Phase 2 complete. Saturation applied.\n');

%% Local nonlinear dynamics (matches two_wheeled_robot_simulation.m's robot_dynamics,
%% minus the LQR control law itself -- u is passed in already-saturated)
function dx = local_robot_deriv(x, u, params)
    theta = x(3); thetadot = x(4);
    Sx = sin(theta); Cx = cos(theta);
    denom = params.Ib + params.M*params.l^2 * (1 - Cx^2);
    xddot = (u/params.R - params.b*x(2) + params.M*params.l*params.g*Sx*Cx ...
            - params.M*params.l*thetadot^2*Sx) / (params.M + params.m);
    thetaddot = (params.M*params.g*params.l*Sx - params.M*params.l*xddot*Cx) / denom;
    dx = [x(2); xddot; x(4); thetaddot];
end
