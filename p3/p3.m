%% Control Systems Engineering

% Mass-Spring Damper System

%% Initial Conditions
m_1 = 1;     % units in Kg
m_2 = 1;     % units in Kg
k_1 = 10;    % units in N/m   
k_2 = 10;    % units in N/m   
c_1 = 0.1;   % units in Kg/s  
c_2 = 0.1;   % units in Kg/s  

X = [0.5; 1; -0.1; 0.5];

% State Space matrices

A = [ 0             0         1              0
      0             0         0              1
    -(k_1+k_2)/m_1  k_2/m_1 -(c_1+c_2)/m_1   c_2/m_1
      k_2/m_2      -k_2/m_2   c_2/m_2       -c_2/m_2]

B = [0; 0; 1; 0]

C = [1 0 0 0]

D = 0

% to track x_1 = 10m
r = 10

% Conditions for Riccati Equation
Qy = 50;
Qw = 1;
R  = 2;

nu = size(B,2); % No of Inputs
nx = length(A); % No of States
ny = size(C,1); % No of outputs

% LQR gains 
[K,S] = lqr(A,B,C'*Qy*C,R)

% Adding Integral Action
A_ = [A  zeros(nx,ny)
     -C  zeros(ny,ny)]
B_ = [B;zeros(ny,nu)]
C_ = [C zeros(ny,ny)]

F = -inv(R)*B'*inv((A-B*K)')*C'*Qy;

Q_ = [C'*Qy*C      zeros(nx,ny)
      zeros(ny,nx) Qw];

% LQR for the new state
[K_ S_] = lqr(A_,B_,Q_,R)


W  = inv(R)*B_'*inv((A_-B_*K_)');
W1 = W(:,1:nx);

G  = W*S_;
G2 = G(:,nx+1:nx+ny);

Kr = G2-W1*C'*Qy
Kw = K_(:,nx+1:nx+ny)
Kx = Khat(:,1:nx)

% [t,X] = sim('p3.slx');



