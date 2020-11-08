%% Control Systems Engineering

% Mass-Spring Damper System

%% Initial Conditions
m_1 = 1;   % units in Kg
m_2 = 1;   % units in Kg
k_1 = 1;   % units in N/m   changes for other conditions (1,1,0) 
k_2 = 1;   % units in N/m   changes for other conditions (1,1,0)
c_1 = 1;   % units in Kg/s  changes for other conditions (1,0,0)
c_2 = 1;   % units in Kg/s  changes for other conditions (1,0,0)

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

