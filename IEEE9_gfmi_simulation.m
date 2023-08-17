%% Definition of the power network
load_droop_params;
comp = gfmi_droop(vsc_params,controller_params,ref_model_params);
net = network_IEEE9bus();
net.a_bus{2}.set_component(comp);
net.a_bus{3}.set_component(comp);

% con = controller_broadcast_PI_AGC(net, [1,3],[1,3],-10, -500);
% net.add_controller_global(con);

%% Check equilibrium point of the GFMI
comp = net.a_bus{2}.component;
xeq = comp.x_equilibrium;
Veq = comp.V_equilibrium;
Ieq = comp.I_equilibrium;
[dx,con] = comp.get_dx_constraint(0, xeq, [real(Veq);imag(Veq)],[real(Ieq);imag(Ieq)],[0,0]);

%% Simulation of power system
time = [0, 1, 2, 10];
u_idx = 8;
u = [   0.75,         0.75,         0.75,           0.75;...
        0,            0,            0,              0];

out1 = net.simulate(time, u, u_idx);
sampling_time = out1.t;

%% Plot of frequency deviation of the three machines
figure
omega1 = out1.X{1}(:,2);
plot(sampling_time, omega1, 'LineWidth', 2)
hold on

omega2 = out1.X{2}(:,13);
plot(sampling_time, omega2, 'LineWidth', 2)
hold on

omega3 = out1.X{3}(:,13);
plot(sampling_time, omega3, 'LineWidth', 2)
hold on

%% Plot of the state variables of the GFMI
figure
for idx = 1 : 12
    subplot(4, 3, idx)
    y = out1.X{2}(:, idx);
    plot(sampling_time, y, 'LineWidth', 2)
    xlabel('Time (s)', 'FontSize', 15)
end
