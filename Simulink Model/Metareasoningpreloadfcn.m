%% Load variables for metareasoning.slx

g = 9.81; % gravity m/s^2

tau_t = 1;  % thrust time constant, sec 
tau_b = .5; % bank angle time constant, sec 
tau_n = .5; % load factor time constant, sec 

rho = 1.225; % air density, kg/m^3
S = 37.161; % wing area, ft^2
Cd0 = .02; % parasite drag coefficient
k = .1; % induced drag coefficient
W = 14515; % weight, lb

omega_v = .3; % velocity bandwidth, sec
omega_gamma = .2; % gamma bandwidth, sec
omega_chi = .2; % chi bandwidth, sec

Tmax = 25600*4.44822; % maximum thrust, N
Clmax = 2.0; % maximum lift coefficient
Clmin = -.5; % minimum lift coefficient
nmax = 7; % maximum load factor
look_ahead = 120; % collision look-ahead [s]
LOS_m = 9260; % Loss of separation distance [m]







