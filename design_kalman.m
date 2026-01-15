% design_kalman.m
% Assumes Ac, Bc, Cc, Dc are already in the workspace
% (from linearization_script.m)

%% Discretize continuous model
Ts   = 0.01;                      % sample time [s]
sys_d = c2d(ss(Ac,Bc,Cc,Dc), Ts);

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

% Measurements: all 4 states
Cd_meas = Cd;                     % y = Cd_meas * x

%% Kalman filter noise tuning (intensities)
Qn = 1e-8;                        % process noise intensity (tune)
Rn = 1e-2;                        % measurement noise intensity (tune)

%% Design steady‑state discrete Kalman filter
[kf_sys,Ld,P] = kalman(sys_d, Qn, Rn);   % Ld is the Kalman gain

%% Estimator closed‑loop matrix for debugging
A_est = Ad - Ld*Cd_meas;
eig_A_est = eig(A_est)           % all |eig_A_est| < 1  → stable