clearvars; clc;
load_vsm_params;
gfmi = gfmi_vsm(vsc_params,controller_params,ref_model_params);

%% Creation of power networks
avr_type = false; pss = false;
plot_separated = false;
x_lim = [0, 50];
y_lim = [-inf, inf];

net1 = network_sample3bus(avr_type, pss);
net2 = network_sample3bus(avr_type, pss);
net2.a_bus{3}.set_component(gfmi);
net2.initialize();

con1 = controller_broadcast_PI_AGC(net1,[1],[1],-10,-500);
con2 = controller_broadcast_PI_AGC(net2,[1],[1],-10,-500);
net1.add_controller_global(con1);
net2.add_controller_global(con2);

%% Simulation settings
time = [0,1,5,60];

% 10% load increase at bus 2
% u_idx = 2;
% u = [0,  0.01, 0.01, 0.01;...
%      0,    0,   0,   0];
% out1 = net1.simulate(time, u, u_idx);
% out2 = net2.simulate(time, u, u_idx);

% Initial value disturbance
% option1 = struct();
% option1.x0_sys = net1.x_equilibrium;
% option1.x0_sys(1) = option1.x0_sys(1) + pi/6;
% option1.x0_sys(3) = option1.x0_sys(3) + 0.1;
% option2 = struct();
% option2.x0_sys = net2.x_equilibrium;
% option2.x0_sys(1) = option2.x0_sys(1) + pi/6;
% option2.x0_sys(3) = option2.x0_sys(3) + 0.1;
% out1 = net1.simulate(time, option1);
% out2 = net2.simulate(time, option2);

% Ground fault disturbance
option = struct();
option.fault = {{[0 0.05], 1}};
out1 = net1.simulate(time, option);
out2 = net2.simulate(time, option);

%% Read output data
sampling_time1 = out1.t;
sampling_time2 = out2.t;
omega1 = out1.X{1}(:,2);
omega2 = out1.X{3}(:,2);
omega3 = out2.X{1}(:,2);
omega4 = out2.X{3}(:,13) - 1;

%% Plot graphs
if plot_separated
    figure;
    subplot(2,1,1)
    plot(sampling_time1, 60 * (1 + omega1),'LineWidth',2)
    hold on
    plot(sampling_time1, 60 * (1 + omega2),'LineWidth',2)
    grid on
    legend('Scenario 1: SM1', 'Scenario 1: SM3')
    xlabel('Time (s)', 'FontSize', 15)
    ylabel('Frequency (Hz)', 'FontSize', 15)
    xlim(x_lim)
    ylim(y_lim)

    subplot(2,1,2)
    plot(sampling_time2, 60 * (1 + omega3),'LineWidth',2)
    hold on
    plot(sampling_time2, 60 * (1 + omega4),'LineWidth',2)
    grid on
    legend('Scenario 2: SM1', 'Scenario 2: VSG-GFMI')
    xlabel('Time (s)', 'FontSize', 15)
    ylabel('Frequency (Hz)', 'FontSize', 15)
    xlim(x_lim)
    ylim(y_lim)
else
    figure;
    plot(sampling_time1, 60 * (1 + omega1),'LineWidth',2)
    hold on
    plot(sampling_time1, 60 * (1 + omega2),'LineWidth',2)
    hold on
    plot(sampling_time2, 60 * (1 + omega3),'--','LineWidth',2)
    hold on
    plot(sampling_time2, 60 * (1 + omega4),'--','LineWidth',2)
    grid on
    legend('Scenario 1: SM1', 'Scenario 1: SM3', 'Scenario 2: SM1', 'Scenario 2: VSG-GFMI')
    xlabel('Time (s)', 'FontSize', 15)
    ylabel('Frequency (Hz)', 'FontSize', 15)
    xlim(x_lim)
    ylim(y_lim)
end