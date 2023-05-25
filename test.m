clear;
clc;
net = network_IEEE9bus();
c = net.a_bus{2}.component;

t = 0;
xeq = c.x_equilibrium;
Veq = c.V_equilibrium;
Ieq = c.I_equilibrium;
u0 = zeros(c.get_nu,1);
[dx,con] = c.get_dx_constraint(t,xeq,[real(Veq); imag(Veq)],[real(Ieq); imag(Ieq)],u0);


% time = [0,10,20,40];
% u_idx = 7;
% u = [1, 0, 0, 0;...
%      0, 0, 0, 0];
% 
% out1 = net.simulate(time, u, u_idx);