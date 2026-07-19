%% MPC_VS_LQR  Three-way controller comparison: LQR vs PID vs MPC
% Same nonlinear plant, same initial tilt, same +-10 N.m torque
% saturation, same true-state feedback (no estimator here -- isolating
% CONTROLLER differences, not estimation differences, which were already
% covered in state_estimator_kalman.m / kalman_simulation.m).
%
% RESULTS SUMMARY (50-degree test, all under +-10 N.m):
%   LQR: final angle -4.33 deg, position error 0.081m -- clean, smooth
%   PID: final angle -4.27 deg, position error 0.153m -- recovers, but
%        chatters against the torque limit noticeably more than LQR
%   MPC: does not reliably recover -- see the KNOWN LIMITATION note
%        below the MPC weight definitions further down this file
%
% PID BUG FOUND AND FIXED: the inner angle-loop PID initially had the
% error sign backwards (theta_des - theta instead of theta - theta_des),
% which produced POSITIVE feedback on an already-unstable system --
% diverged monotonically to 10000+ degrees. Confirmed by comparing
% against the working LQR gain K's sign on the theta/thetadot terms.
% Fixed by flipping the error definition; re-verified against LQR with
% the outer position loop disabled first (isolate before compose), then
% re-enabled once the inner loop was confirmed solid.
%
% MPC: built from scratch via quadprog, not the toolbox mpc() object,
% so it explicitly plans around the +-10 N.m constraint rather than
% clipping after the fact like LQR/PID do. Despite that theoretical
% advantage, hand-tuning the cost weights did not converge to reliable
% recovery -- see the KNOWN LIMITATION comment near Q_mpc/R_mpc for the
% full story (three tuning attempts, likely root cause: horizon-length
% numerical conditioning). Documented honestly as unresolved rather than
% forced to look like it works.
%
% REQUIRES: Control System Toolbox (lqr, c2d) AND Optimization Toolbox
% (quadprog, for the MPC controller).

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

u_max = 10;          % N.m, same hard actuator limit for all three controllers
x_ref = [0.5; 0; 0; 0];

% TEST_ANGLE_DEG: run this at 50 first (hardest case, matches earlier
% tests). If MPC fails here, ALSO try 20 degrees (where the linear model
% is much closer to valid) before concluding MPC is "worse" -- that
% distinguishes "MPC struggles because the model is wrong at 50 degrees"
% from "MPC is fundamentally broken," which are very different findings.
TEST_ANGLE_DEG = 50;   % the real headline test -- see file header for full results summary
x0 = [0; 0; deg2rad(TEST_ANGLE_DEG); 0];

dt = 0.01;
T_final = 8;
N_steps = round(T_final/dt);

nonlinear_deriv = @(x, u) local_robot_deriv(x, u, params);

%% 2. Controller 1: LQR (baseline, same design as before)
Q_lqr = diag([10, 2, 50, 5]);
R_lqr = 0.8;
K = lqr(A, B, Q_lqr, R_lqr);

%% 3. Controller 2: Cascade PID
% Outer loop: position error -> a DESIRED lean angle (clipped to a
% modest range so the small-angle PID doesn't get asked to track
% something absurd). Inner loop: PID on angle error -> torque.
%
% NOTE ON TUNING: these gains are a reasoned starting point (matched
% roughly to this system's ~11.8 rad/s open-loop instability speed), NOT
% verified by running the simulation myself. If the angle response is
% oscillatory, reduce Kd_ang first; if it's sluggish/doesn't recover in
% time, increase Kp_ang. Retune here if needed.
% Inner loop confirmed working (settles near 0 deg instead of diverging)
% with the outer loop disabled. Re-enabling it now to test its sign
% convention next -- if position runs away instead of converging toward
% x_ref, this loop needs the same kind of sign flip the inner loop did.
Kp_pos = 0.15;  Kd_pos = 0.30;
theta_des_limit = deg2rad(15);           % never command more than 15 deg lean
Kp_ang = 200;   Ki_ang = 80;   Kd_ang = 18;   % angle -> torque

%% 4. Controller 3: Linear MPC (built from scratch via quadprog)
if isempty(which('quadprog'))
    error(['quadprog not found -- MPC controller requires the ' ...
           'Optimization Toolbox. LQR and PID sections can still be ' ...
           'run independently if you comment out the MPC block below.']);
end

sys_d = c2d(ss(A, B, eye(4), 0), dt);
Ad = sys_d.A; Bd = sys_d.B;

N_mpc = 40;   % prediction horizon (0.4s lookahead at dt=0.01) -- increased from
              % 20 steps (0.2s) to test whether short horizon was limiting recovery
% RETUNED: copying LQR's Q=diag([10,2,50,5]), R=0.8 directly into a
% SUMMED discrete-horizon cost (without dt scaling) made MPC's "optimal"
% response too gentle -- it under-reacted for the first several steps
% while the real plant (unstable at ~11.8 rad/s) needed an immediate
% strong correction, and by the time MPC did react hard it had already
% overshot too far to recover (see the 20-degree test: gentle torques of
% 1-2 Nm for the first 5 steps while LQR/PID were already near the 10 Nm
% ceiling). Weighting angle error much more heavily and cutting the
% control-effort penalty forces MPC to match that urgency.
% KNOWN LIMITATION (documented, not resolved): three tuning attempts were
% made here -- (1) LQR-matched weights Q=diag([10,2,50,5]), R=0.8 summed
% over the horizon: too gentle, under-reacted, diverged; (2) heavily
% angle-weighted Q=diag([10,2,400,10]), R=0.05: kept the angle bounded
% but abandoned position tracking (119m final error); (3) a middle
% ground Q=diag([30,5,200,10]), R=0.15: diverged again, monotonically.
% Hand-tuning did not converge to a working set of weights. The likely
% root cause: with a 40-step horizon and this system's discrete growth
% rate (~1.125x/step, compounding to ~112x by the end of the horizon),
% the prediction matrices Sx/Su span a very large numerical range, which
% makes the QP poorly conditioned -- quadprog reports success
% (exitflag=1) each time, but small weight changes produce
% disproportionate, hard-to-predict behavior changes as a result. A
% shorter horizon (better conditioned, though a shorter-sighted planner)
% was identified as the most likely fix but not yet tried. Left here as
% the LQR-matched baseline rather than the last (worse) attempt.
Q_mpc = diag([10, 2, 50, 5]);
R_mpc = 0.8;

% Condense the QP: predicted state stack X = Sx*x0 + Su*U
n = 4;
Sx = zeros(n*N_mpc, n);
Su = zeros(n*N_mpc, N_mpc);
Apow = eye(n);
for i = 1:N_mpc
    Apow = Apow * Ad;
    Sx((i-1)*n+1:i*n, :) = Apow;
    for j = 1:i
        power = i - j;
        Aj = Ad^power;
        Su((i-1)*n+1:i*n, j) = Aj * Bd;
    end
end
Qbar = kron(eye(N_mpc), Q_mpc);
Rbar = kron(eye(N_mpc), R_mpc);
lb_mpc = -u_max * ones(N_mpc, 1);
ub_mpc =  u_max * ones(N_mpc, 1);
qp_opts = optimoptions('quadprog', 'Display', 'off');

%% 5. Run all three controllers through the SAME nonlinear plant + test

controllers = {'LQR', 'PID', 'MPC'};
results = struct();

for c = 1:3
    name = controllers{c};
    fprintf('--- Running %s ---\n', name);

    x = zeros(4, N_steps); u_hist = zeros(1, N_steps);
    x(:,1) = x0;
    integral_ang_err = 0;   % PID integral term, reset per controller

    for k = 1:N_steps-1
        xk = x(:,k);

        switch name
            case 'LQR'
                u_raw = -K * (xk - x_ref);

            case 'PID'
                pos_err = xk(1) - x_ref(1);
                theta_des = -Kp_pos*pos_err - Kd_pos*xk(2);
                theta_des = max(min(theta_des, theta_des_limit), -theta_des_limit);
                % SIGN FIX: error must be (actual - desired), not
                % (desired - actual). Comparing against the working LQR
                % gain K showed the theta/thetadot feedback needs to be
                % POSITIVE for this model's sign convention (u ~
                % +14*theta + 3.5*thetadot). The previous (theta_des -
                % theta) definition produced the opposite sign --
                % positive feedback on an already-unstable system, which
                % is exactly why it diverged monotonically instead of
                % oscillating.
                ang_err = xk(3) - theta_des;
                u_unsat = Kp_ang*ang_err + Ki_ang*integral_ang_err + Kd_ang*xk(4);
                if abs(u_unsat) < u_max
                    integral_ang_err = integral_ang_err + ang_err*dt;
                end
                u_raw = Kp_ang*ang_err + Ki_ang*integral_ang_err + Kd_ang*xk(4);

            case 'MPC'
                xref_stack = repmat(x_ref, N_mpc, 1);
                H = 2*(Su'*Qbar*Su + Rbar);
                H = (H+H')/2;  % ensure symmetric for numerical stability
                f = 2*Su'*Qbar*(Sx*xk - xref_stack);
                [U_opt, ~, exitflag] = quadprog(H, f, [], [], [], [], lb_mpc, ub_mpc, [], qp_opts);
                if isempty(U_opt) || exitflag <= 0
                    warning('quadprog exitflag=%d at step %d (t=%.3fs) -- solution may be unreliable', ...
                        exitflag, k, k*dt);
                    if isempty(U_opt)
                        U_opt = zeros(N_mpc,1);
                    end
                end
                u_raw = U_opt(1);   % receding horizon: apply only the first move
                if k <= 5
                    fprintf('  MPC step %d: theta=%.2f deg, u_raw=%.3f Nm, exitflag=%d\n', ...
                        k, rad2deg(xk(3)), u_raw, exitflag);
                end
        end

        u = max(min(u_raw, u_max), -u_max);
        u_hist(k) = u;

        f_dyn = @(xx) nonlinear_deriv(xx, u);
        k1 = f_dyn(xk);
        k2 = f_dyn(xk + dt/2*k1);
        k3 = f_dyn(xk + dt/2*k2);
        k4 = f_dyn(xk + dt*k3);
        x(:,k+1) = xk + dt/6*(k1 + 2*k2 + 2*k3 + k4);

        if any(~isfinite(x(:,k+1)))
            warning('%s diverged at t=%.3fs. Stopping early.', name, k*dt);
            x = x(:,1:k+1); u_hist = u_hist(1:k+1);
            break;
        end
    end

    results.(name).x = x;
    results.(name).u = u_hist;
    results.(name).t = (0:size(x,2)-1)*dt;
end

%% 6. Metrics table
fprintf('\n%-6s %12s %12s %14s %14s\n', 'Ctrl', 'MaxAngle', 'FinalAngle', 'FinalPosErr', 'Max|u|');
for c = 1:3
    name = controllers{c};
    xr = results.(name).x; ur = results.(name).u;
    fprintf('%-6s %11.2f deg %11.2f deg %13.4f m %13.3f Nm\n', name, ...
        max(abs(rad2deg(xr(3,:)))), rad2deg(xr(3,end)), ...
        abs(xr(1,end)-x_ref(1)), max(abs(ur)));
end

%% 7. Comparison plots
figure('Name', 'LQR vs PID vs MPC -- Angle', 'Color', 'w');
colors = {'b', 'g', 'r'};
for c = 1:3
    name = controllers{c};
    plot(results.(name).t, rad2deg(results.(name).x(3,:)), colors{c}, 'LineWidth', 1.5); hold on;
end
yline(0, 'k--');
legend(controllers, 'Location', 'best');
title('Pole Angle: LQR vs PID vs MPC (50-degree drop, +-10 N.m limit)');
xlabel('Time (s)'); ylabel('Angle (deg)'); grid on;

figure('Name', 'LQR vs PID vs MPC -- Position', 'Color', 'w');
for c = 1:3
    name = controllers{c};
    plot(results.(name).t, results.(name).x(1,:), colors{c}, 'LineWidth', 1.5); hold on;
end
yline(x_ref(1), 'k--', 'Target');
legend(controllers, 'Location', 'best');
title('Cart Position: LQR vs PID vs MPC');
xlabel('Time (s)'); ylabel('Position (m)'); grid on;

figure('Name', 'LQR vs PID vs MPC -- Control Effort', 'Color', 'w');
for c = 1:3
    name = controllers{c};
    plot(results.(name).t, results.(name).u, colors{c}, 'LineWidth', 1.5); hold on;
end
yline(u_max, 'k--', 'Saturation limit');
yline(-u_max, 'k--');
legend(controllers, 'Location', 'best');
title('Control Effort: LQR vs PID vs MPC (all hitting the same +-10 N.m limit)');
xlabel('Time (s)'); ylabel('Torque (N.m)'); grid on;

fprintf('\nDone. Compare the three overlaid trajectories above.\n');

%% Local nonlinear dynamics (same physics as two_wheeled_robot_simulation.m)
function dx = local_robot_deriv(x, u, params)
    theta = x(3); thetadot = x(4);
    Sx = sin(theta); Cx = cos(theta);
    denom = params.Ib + params.M*params.l^2 * (1 - Cx^2);
    xddot = (u/params.R - params.b*x(2) + params.M*params.l*params.g*Sx*Cx ...
            - params.M*params.l*thetadot^2*Sx) / (params.M + params.m);
    thetaddot = (params.M*params.g*params.l*Sx - params.M*params.l*xddot*Cx) / denom;
    dx = [x(2); xddot; x(4); thetaddot];
end
