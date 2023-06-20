clear;
load_gfmi_params;
net = network_IEEE9bus();
c = gfmi(vsc_params, dc_source_params, controller_params, ref_model_params, 'droop');
net.a_bus{2}.set_component(c);

t = 0;
xeq = c.x_equilibrium;
Veq = c.V_equilibrium;
Ieq = c.I_equilibrium;
u0 = zeros(c.get_nu,1);
[dx,con] = c.get_dx_constraint(t,xeq,[real(Veq); imag(Veq)],[real(Ieq); imag(Ieq)],u0);
disp(dx(6))

% time = [0,10,20,40];
% u_idx = 7;
% u = [0.7, 0, 0, 0;...
%      0, 0, 0, 0];
% 
% out1 = net.simulate(time, u, u_idx);