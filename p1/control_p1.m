%% Control Systems Engineering

% Mass-Spring Damper System

%% Initial Conditions
m_1 = 1;   % units in Kg
m_2 = 1;   % units in Kg
k_1 = 10;  % units in N/m
k_2 = 10;  % units in N/m
c_1 = 5;   % units in Kg/s
c_2 = 5;   % units in Kg/s

X = [0.5; 1; -0.1; 0.5];

% State Space matrices

A = [ 0             0         1              0
      0             0         0              1
    -(k_1+k_2)/m_1  k_2/m_1 -(c_1+c_2)/m_1   c_2/m_1
      k_2/m_2      -k_2/m_2   c_2/m_2       -c_2/m_2];

B = [0; 0; 1; 0];

C = [1 0 0 0];

D = 0;


pol = eig(A)

