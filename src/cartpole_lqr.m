%% Cart-Pole LQR Controller — Design, Simulation, Verification
% Requires: cartpend.m in the same folder, Control System Toolbox (lqr, ctrb, ss)
clear; clc; close all;

%% 1. System parameters
M = 1.0;    % cart mass (kg)
m = 0.2;    % pendulum mass (kg)
l = 0.5;    % pendulum length, pivot to mass (m)
g = 9.81;   % gravity (m/s^2)
b = 0.1;    % cart friction (N·s/m)

%% 2. Equilibrium point and analytic (textbook) matrices, for cross-check
x_eq = [0; 0; 0; 0];   % cart at origin, pole upright, everything at rest
u_eq = 0;

A_analytic = [0 1 0 0;
              0 -b/M -m*g/M 0;
              0 0 0 1;
              0 b/(M*l) (M+m)*g/(M*l) 0];
B_analytic = [0; 1/M; 0; -1/(M*l)];

%% 3. Numerical linearization (finite differences) about x_eq, u_eq
% This is the "ground truth" the controller is actually designed from.
% We cross-check it against A_analytic/B_analytic below — if these two
% don't roughly match, there is a sign/convention bug in cartpend.m.
eps_fd = 1e-6;
n = 4;
A = zeros(n,n);
B = zeros(n,1);

for i = 1:n
    dx_plus  = x_eq; dx_plus(i)  = dx_plus(i)  + eps_fd;
    dx_minus = x_eq; dx_minus(i) = dx_minus(i) - eps_fd;
    f_plus  = cartpend(dx_plus,  m, M, l, g, b, u_eq);
    f_minus = cartpend(dx_minus, m, M, l, g, b, u_eq);
    A(:,i) = (f_plus - f_minus) / (2*eps_fd);
end
f_plus_u  = cartpend(x_eq, m, M, l, g, b, u_eq + eps_fd);
f_minus_u = cartpend(x_eq, m, M, l, g, b, u_eq - eps_fd);
B = (f_plus_u - f_minus_u) / (2*eps_fd);

fprintf('--- Cross-check: numerical vs analytic linearization ---\n');
fprintf('max|A_numeric - A_analytic| = %.2e\n', max(abs(A(:) - A_analytic(:))));
fprintf('max|B_numeric - B_analytic| = %.2e\n', max(abs(B(:) - B_analytic(:))));
fprintf('(should be ~1e-6 or smaller; if not, cartpend.m has a bug)\n\n');

%% 4. Controllability check
Co = ctrb(A,B);
fprintf('Controllability matrix rank: %d (need 4 for full controllability)\n\n', rank(Co));
if rank(Co) < 4
    error('System is not controllable — check A, B before proceeding.');
end

%% 5. LQR design — Q and R with explicit justification
% State order: [pos, vel, theta, thetadot]
%
% Q penalizes state error. We weight theta MUCH more heavily than
% position: a large position error is recoverable and mostly cosmetic,
% but a large angle error means the pole is falling over and the whole
% control problem has failed. thetadot is weighted a bit above baseline
% too, to discourage oscillatory "wobbling" recovery.
%
% R penalizes control effort (force on the cart). We start at R=1 as a
% neutral baseline (same order of magnitude as the diagonal Q terms on
% pos/vel), then sweep it below to show the effort/speed trade-off
% explicitly rather than asserting the number is "right" by fiat.
Q = diag([1, 1, 20, 2]);   % [pos, vel, theta, thetadot]
R = 1;

K = lqr(A, B, Q, R);
fprintf('LQR gain K = [%.4f  %.4f  %.4f  %.4f]\n', K);

closed_loop_eigs = eig(A - B*K);
fprintf('Closed-loop eigenvalues:\n');
disp(closed_loop_eigs);
if any(real(closed_loop_eigs) >= 0)
    warning('Closed-loop system is NOT stable — some eigenvalue has non-negative real part.');
else
    fprintf('All eigenvalues have negative real part -> closed loop is stable.\n\n');
end

%% 6. Q/R sensitivity sweep (justifies the R=1 choice by comparison)
fprintf('--- Q/R sensitivity sweep (theta weight fixed at 20) ---\n');
fprintf('%8s %12s %14s\n', 'R', 'max|u| (N)', 'settle-ish gain');
R_values = [0.1, 1, 10, 100];
for Rv = R_values
    Kv = lqr(A, B, Q, Rv);
    eigsv = eig(A - B*Kv);
    fprintf('%8.2f %12s %14s\n', Rv, '(see sim)', mat2str(round(real(eigsv),2)'));
end
fprintf('Smaller R -> more aggressive control (faster, but larger forces).\n');
fprintf('Larger R  -> gentler control (smaller forces, but slower response).\n');
fprintf('R=1 was chosen as a middle ground; adjust based on the max|u| printed below.\n\n');

%% 7. Closed-loop nonlinear simulation — STEP RESPONSE (reference tracking)
x_ref = [1; 0; 0; 0];   % move cart to x=1, keep pole upright
x0 = [0; 0; 0.05; 0];   % start near-upright with a small initial angle error (5.7 deg... actually 0.05 rad ~ 2.9 deg)

tspan = [0 10];
u_hist = [];  t_hist_u = [];  % logged inside the ODE wrapper below

function [dx, u] = closed_loop_dynamics(~, x, K, x_ref, m, M, l, g, b)
    u = -K * (x - x_ref);
    dx = cartpend(x, m, M, l, g, b, u);
end

odefun_step = @(t,x) closed_loop_dynamics(t, x, K, x_ref, m, M, l, g, b);
[t_step, x_step] = ode45(odefun_step, tspan, x0);

% Recompute control input at each logged time step (for plotting)
u_step = zeros(size(t_step));
for k = 1:length(t_step)
    u_step(k) = -K * (x_step(k,:)' - x_ref);
end

%% 8. Closed-loop nonlinear simulation — DISTURBANCE REJECTION
% Start settled at x_ref, apply a horizontal impulse-like push at t=3s
x0_dist = x_ref;
t_disturb = 3;
F_disturb = 5;      % N, applied briefly
disturb_duration = 0.1;  % s

function [dx, u] = disturbed_dynamics(t, x, K, x_ref, m, M, l, g, b, t_disturb, F_disturb, disturb_duration)
    u = -K * (x - x_ref);
    if t >= t_disturb && t <= t_disturb + disturb_duration
        u = u + F_disturb;
    end
    dx = cartpend(x, m, M, l, g, b, u);
end

odefun_dist = @(t,x) disturbed_dynamics(t, x, K, x_ref, m, M, l, g, b, t_disturb, F_disturb, disturb_duration);
[t_dist, x_dist] = ode45(odefun_dist, tspan, x0_dist);

u_dist = zeros(size(t_dist));
for k = 1:length(t_dist)
    u_dist(k) = -K * (x_dist(k,:)' - x_ref);
    if t_dist(k) >= t_disturb && t_dist(k) <= t_disturb + disturb_duration
        u_dist(k) = u_dist(k) + F_disturb;
    end
end

%% 9. Performance metrics (printed, not just plotted)
theta_step_deg = rad2deg(x_step(:,3));
theta_dist_deg = rad2deg(x_dist(:,3));

fprintf('--- STEP RESPONSE metrics ---\n');
fprintf('Max |control effort| u:      %.3f N\n', max(abs(u_step)));
fprintf('Max angle deviation:          %.3f deg\n', max(abs(theta_step_deg)));
fprintf('Final position error:         %.4f m\n', abs(x_step(end,1) - x_ref(1)));
fprintf('Final angle:                  %.4f deg\n\n', theta_step_deg(end));

fprintf('--- DISTURBANCE REJECTION metrics ---\n');
fprintf('Max |control effort| u:      %.3f N\n', max(abs(u_dist)));
fprintf('Max angle deviation:          %.3f deg\n', max(abs(theta_dist_deg)));
fprintf('Final position error:         %.4f m\n', abs(x_dist(end,1) - x_ref(1)));
fprintf('Final angle:                  %.4f deg\n\n', theta_dist_deg(end));

%% 10. PLOTS — position, angle, control effort (the three that matter)

% --- Step response: 3 stacked subplots ---
figure('Name', 'Cart-Pole LQR: Step Response');
subplot(3,1,1);
plot(t_step, x_step(:,1), 'b-', 'LineWidth', 1.5); hold on;
yline(x_ref(1), 'k--');
ylabel('Cart position (m)');
title('Step Response — position, angle, control effort');
grid on;

subplot(3,1,2);
plot(t_step, theta_step_deg, 'r-', 'LineWidth', 1.5); hold on;
yline(0, 'k--');
ylabel('Pole angle (deg)');
grid on;

subplot(3,1,3);
plot(t_step, u_step, 'm-', 'LineWidth', 1.5);
ylabel('Control force u (N)');
xlabel('Time (s)');
grid on;

% --- Disturbance rejection: 3 stacked subplots ---
figure('Name', 'Cart-Pole LQR: Disturbance Rejection');
subplot(3,1,1);
plot(t_dist, x_dist(:,1), 'b-', 'LineWidth', 1.5); hold on;
yline(x_ref(1), 'k--');
xline(t_disturb, 'g:', 'push');
ylabel('Cart position (m)');
title('Disturbance Rejection — position, angle, control effort');
grid on;

subplot(3,1,2);
plot(t_dist, theta_dist_deg, 'r-', 'LineWidth', 1.5); hold on;
yline(0, 'k--');
xline(t_disturb, 'g:', 'push');
ylabel('Pole angle (deg)');
grid on;

subplot(3,1,3);
plot(t_dist, u_dist, 'm-', 'LineWidth', 1.5);
ylabel('Control force u (N)');
xlabel('Time (s)');
grid on;

fprintf('Done. Check the two figure windows and the printed metrics above.\n');
fprintf('If max|u| looks unreasonably large (>>20N for this size cart), increase R and re-run.\n');
