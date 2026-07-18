function dx = cartpend(x, m, M, l, g, b, u)
% CARTPEND  Nonlinear dynamics of a cart-pole (inverted pendulum on a cart).
%
% State convention (IMPORTANT — this is the single source of truth for
% sign conventions used throughout this project):
%   x = [ pos ; vel ; theta ; thetadot ]
%   theta = 0   -> pole pointing straight UP (unstable equilibrium, our goal)
%   theta = pi  -> pole hanging straight DOWN (stable equilibrium)
%
% Parameters:
%   m  - pendulum point mass (kg)
%   M  - cart mass (kg)
%   l  - distance from pivot to pendulum mass (m)
%   g  - gravity (m/s^2)
%   b  - viscous friction coefficient on the cart (N·s/m)
%   u  - force applied to the cart (N)
%
% Derivation (Lagrangian, point-mass pendulum, theta measured from the
% upward vertical):
%   Xddot     = (u - b*Xdot - m*g*sin(theta)*cos(theta) + m*l*sin(theta)*thetadot^2) ...
%               / (M + m*sin(theta)^2)
%   thetaddot = (g*sin(theta) - Xddot*cos(theta)) / l
%
% Linearizing about theta=0, thetadot=0, u=0 recovers the standard
% textbook cart-pole matrices:
%   A = [0 1 0 0; 0 -b/M -m*g/M 0; 0 0 0 1; 0 b/(M*l) (M+m)*g/(M*l) 0]
%   B = [0; 1/M; 0; -1/(M*l)]
% This is used below only as a numerical cross-check — the script
% linearizes numerically (finite differences) so it stays correct even
% if you change the nonlinear model.

Sx = sin(x(3));
Cx = cos(x(3));

xddot = (u - b*x(2) - m*g*Sx*Cx + m*l*Sx*x(4)^2) / (M + m*Sx^2);
thetaddot = (g*Sx - xddot*Cx) / l;

dx = zeros(4,1);
dx(1) = x(2);
dx(2) = xddot;
dx(3) = x(4);
dx(4) = thetaddot;

end
