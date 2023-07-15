load_gfmi_params;
net = network_IEEE9bus();
% comp = gfmi_droop(vsc_params,dc_source_params,controller_params,ref_model_params);
% net.a_bus{2}.set_component(comp);

% con = controller_broadcast_PI_AGC(net, [1,3],[1,3],-10, -500);
% net.add_controller_global(con);

time = [0, 25, 50];
u_idx = 6;
u = [   0.1,          0.1,          0.1;...
        0.1,          0.1,          0.1];

out1 = net.simulate(time, u, u_idx);
sampling_time = out1.t;

figure
omega1 = out1.X{1}(:,2);
plot(sampling_time, omega1, 'LineWidth', 2)
hold on

omega2 = out1.X{2}(:,2);
plot(sampling_time, omega2, 'LineWidth', 2)
hold on

% omega2 = diff(out1.X{2}(:,1))./diff(sampling_time);
% t = (sampling_time(2:end)+sampling_time(1:(end-1)))/2;
% plot(t, omega2, 'LineWidth', 2)
% hold on

omega3 = out1.X{3}(:,2);
plot(sampling_time, omega3, 'LineWidth', 2)
hold on

xlabel('Time (s)', 'FontSize', 15)
ylabel('Frequency deviation', 'FontSize', 15)
legend('SM1','GFM','SM2')
hold off

% xeq = comp.x_equilibrium;
% Veq = comp.V_equilibrium;
% Ieq = comp.I_equilibrium;
% [dx,con] = comp.get_dx_constraint(0, xeq, [real(Veq);imag(Veq)],[real(Ieq);imag(Ieq)],0);