load_gfmi_params3;
comp = gfmi_droop(vsc_params,dc_source_params,controller_params,ref_model_params, 0.082726);
net = network_IEEE9bus();
net.a_bus{2}.set_component(comp);

% con = controller_broadcast_PI_AGC(net, [1,3],[1,3],-10, -500);
% net.add_controller_global(con);

time = [0, 1, 25, 100];
u_idx = 8;
u = [   0,         0.75,         0.75,           0.75;...
        0,            0,            0,              0];

out1 = net.simulate(time, u, u_idx, 'method', 'foh');
sampling_time = out1.t;

omega0 = 2*pi*60;
figure
omega1 = out1.X{1}(:,2);
plot(sampling_time, omega1, 'LineWidth', 2)
hold on

omega2 = out1.X{2}(:,13);
plot(sampling_time, omega2/omega0, 'LineWidth', 2)
hold on

omega3 = out1.X{3}(:,2);
plot(sampling_time, omega3, 'LineWidth', 2)
hold on

% omega1 = diff(out1.X{1}(:,1))./diff(sampling_time);
% t = (sampling_time(2:end)+sampling_time(1:(end-1)))/2;
% plot(t, omega1/omega0, 'LineWidth', 2)
% hold on
% 
% omega2 = diff(out1.X{2}(:,11))./diff(sampling_time);
% t = (sampling_time(2:end)+sampling_time(1:(end-1)))/2;
% plot(t, omega2/omega0, 'LineWidth', 2)
% hold on
% 
% omega3 = diff(out1.X{3}(:,1))./diff(sampling_time);
% t = (sampling_time(2:end)+sampling_time(1:(end-1)))/2;
% plot(t, omega3/omega0, 'LineWidth', 2)
% hold on

xlabel('Time (s)', 'FontSize', 15)
ylabel('Frequency deviation', 'FontSize', 15)
legend('SM1','GFMI','SM3')
hold off

comp = net.a_bus{2}.component;
xeq = comp.x_equilibrium;
Veq = comp.V_equilibrium;
Ieq = comp.I_equilibrium;
[dx,con] = comp.get_dx_constraint(0, xeq, [real(Veq);imag(Veq)],[real(Ieq);imag(Ieq)],[0,0]);