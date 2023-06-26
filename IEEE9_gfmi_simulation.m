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
disp(dx)

% n = 10000;
% time = linspace(0, 120, n);
% u_idx = 4;
% u = [0.75 * ones(1,n); zeros(1,n)];

% time = [0,5];
% u_idx = 5;
% u = [0.1,  0.1;...
%      0,    0 ];
% 
% out1 = net.simulate(time, u, u_idx);
% sampling_time = out1.t;
% figure
% omega1 = out1.X{1}(:,2);
% plot(sampling_time, omega1, 'LineWidth', 2)
% hold on
% 
% omega2 = out1.X{2}(:,2);
% plot(sampling_time, omega2, 'LineWidth', 2)
% hold on
% 
% omega3 = out1.X{3}(:,2);
% plot(sampling_time, omega3, 'LineWidth', 2)
% hold on
% 
% xlabel('Time (s)', 'FontSize', 15)
% ylabel('Frequency deviation', 'FontSize', 15)
% legend('Synchronous machine','Grid Forming Droop Converter', 'Synchronous Machine')
% hold off