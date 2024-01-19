clc
clear
close all

%% Define the parameter
mc = 1.5;       % Mass of the cart
mp = 0.5;       % Mass of the pendulum
g = 9.82;       % Earth gravity
L = 1;          % Length of the pendulum
d1 = 1e-2;      % Damping of the cart displacement
d2 = 1e-2;      % Damping in the joint

%% Define the matrices
% First define the matrices to use their elements easily
A = [0, 0               ,   1           , 0                         ;
     0, 0               ,   0           , 1                         ;
     0, (g*mp)/mc       ,   (-d1)/mc    , (-d2)/(L*mc)              ;
     0, g*(mc+mp)/(L*mc),   (-d1)/(L*mc), (-d2)*(mc+mp)/(L^2*mc*mp)];

B = [0, 0,  1/mc,   1/(L*mc)]';

C = [1, 0,  0,  0];

D = [0];

x0 = [0;    5*pi/180;   0;  0]

%% Build the system
sys = ss(A,B,C,D)
poles_1 = eig(A)
poles_2 = pole(sys)
Sc = ctrb(sys)
So = obsv(sys)
rank_Sc = rank(Sc)
rank_So = rank(So)
% rlocus(sys)

%% Control
des_poles = [-3;-3;-3;-3];
K = acker(A,B,des_poles)

Q = eye(4);
R = 0.1;
K_lqr = lqr(A,B,Q,R);

%% Discrete Control
Ts = 0.1;           % Sample time
sys_d = c2d(sys,Ts)
Ad = sys_d.a;
Bd = sys_d.b;
Cd = sys_d.c;
Dd = sys_d.d;

% Gain should remain within unit circle in discrete system
des_poles_d = [.3;.3;.3;.3];
K_d = acker(Ad,Bd,des_poles_d)

% Observer Gain
des_poles_Ob = [.3;.3;.3;.3]*0.1;
Ob = acker(Ad',Cd',des_poles_Ob)


