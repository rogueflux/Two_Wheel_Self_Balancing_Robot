% design_lqr.m
% Assumes Ac, Bc already in workspace

% State weighting matrix Q  (tune these)
Q = diag([100, 10, 50, 1]);   % [alpha, omega, x_cart, v_cart]

% Control weighting scalar R (tune)
R = 0.01;

% LQR gain: u = -K * x
K = lqr(Ac, Bc, Q, R);

% For reference
eig_CL = eig(Ac - Bc*K);   % closed-loop poles