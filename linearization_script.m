% Inverted pendulum parameters (example values)
M1 = 1.0;      % cart mass [kg]
M2 = 0.1;      % pendulum mass [kg]
L  = 0.5;      % pendulum length [m]
g  = 9.81;     % gravity [m/s^2]

% Continuous‑time linearized state‑space (around upright eq.)
% x = [alpha; omega; x_cart; v_cart]
Ac = [ 0              1                        0                     0;
       (M1+M2)*g/(M1*L)   0                    0                     0;
       0              0                        0                     1;
      -M2*g/M1        0                        0                     0 ];

Bc = [ 0;
      -1/(M1*L);
       0;
       1/M1 ];

Cc = eye(4);          % measure all states
Dc = zeros(4,1);

% Operating point (upright, zero velocity, zero force)
x_op = [0; 0; 0; 0];
u_op = 0;

% Pack as state‑space object (useful for checks)
sys = ss(Ac,Bc,Cc,Dc);

% Check eigenvalues (should show open‑loop instability)
eig_Ac = eig(Ac)