%% Initial conditions for CD&R simulation 

num_ac = 2; %Number of aircraft
pos_ic = [0,0,0; 22224, 23150,0]; %Initial positions [m]
vel_ic = [205.7783, 246.9339]; %Initial velocities [m/s]
chi_ic = deg2rad([0, 270]); %Initial headings BASED OFF EAST [rad]
gamma_ic = zeros(1,num_ac); %Initial flight path angles [rad]
goal_pos = [9000, 1E6, 0; -1E6, 23150+9000, 0]; %Goal positions (Frazzoli 2005)

x0 = pos_ic(:,1).';
y0 = pos_ic(:,2).';
h0 = pos_ic(:,3).';
Cl = 2*W./(rho*vel_ic.^2*S);
Cd = Cd0+k*Cl.^2;
thrust_ic = 0.5*rho.*vel_ic.^2*S.*Cd;