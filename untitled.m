clearvars; clc; close all;
load_vsm2axis_params;
gfmi = gfmi_vsm_2axis(vsc_params,controller_params,ref_model_params);

Te = 1;
Ka = 0.5;
avr_params = table(Te, Ka);
avr_gfmi = avr_sadamoto2019(avr_params);
gfmi.set_avr(avr_gfmi);

net2 = network_IEEE9bus();
net2.a_bus{2}.set_component(gfmi);
net2.initialize();

con2 = controller_broadcast_PI_AGC(net2,[1,3],[1,3],-10,-500);
net2.add_controller_global(con2);

g = net2.a_bus{2}.component;
t = 0;
xeq = g.x_equilibrium;
Veq = g.V_equilibrium;
Ieq = g.I_equilibrium;
u0 = zeros(g.get_nu,1);
[dx,con] = g.get_dx_constraint(t,xeq,[real(Veq); imag(Veq)],[real(Ieq); imag(Ieq)],u0);

%% Simulation an plot parameters
t_final = 10;
x_lim = [0, t_final];
% x_lim = [0.997, 1.03];
y_lim = [-inf, inf];
plot_separated = true;

time = [0, 1 , 5, t_final];
u_idx = 8;
u = [0, 0.1, 0.1, 0.1;...
     0,   0,   0,   0];

%% Scenario 1: all machines are SM
net1 = network_IEEE9bus();
gen_model = 'generator_2axis';

% L_g = 2.6476e-04
Xd = 0.8958;
Xd_prime = 0.1198;
Xq = 0.8645;
Xq_prime = 0.1969;
T = 6;
Tdo = 6;
Tqo = 0.535;
M = 12.8;
D = 4;
params = table(Xd, Xd_prime, Xq, Xq_prime, T, Tdo, Tqo, M, D);
generator = str2func(gen_model);
comp = generator(omega_st, params);

Te = 1;
Ka = 0.5;
avr_params = table(Te, Ka);
avr = avr_sadamoto2019(avr_params);
comp.set_avr(avr);

net1.a_bus{2}.set_component(comp);
net1.initialize()

con1 = controller_broadcast_PI_AGC(net1,[1,3],[1,3],-10,-500);
net1.add_controller_global(con1);

%% Simulation of Scenario 1
out1 = net1.simulate(time, u, u_idx, 'method', 'zoh');

%% Read output data
sampling_time1 = out1.t;

omega11 = out1.X{1}(:,2);
omega12 = out1.X{2}(:,2);
omega13 = out1.X{3}(:,2);

nbus1 = numel(out1.V);
V1complex = cell(nbus1, 1);
I1complex = cell(nbus1, 1);
power1    = cell(nbus1, 1);

for i = 1:numel(out1.V)
    Vi = out1.V{i};
    V1complex{i} = Vi(:,1) + 1j * Vi(:,2);
    Ii = out1.I{i};
    I1complex{i} = Ii(:,1) + 1j * Ii(:,2);
    power1{i} = V1complex{i} .* conj(I1complex{i});
end

%% Scenario 2: SM in bus 2 is replaced by GFMI
Te = 1;
Ka = 0.5;
avr_params = table(Te, Ka);
avr_gfmi = avr_sadamoto2019(avr_params);
gfmi.set_avr(avr_gfmi);

net2 = network_IEEE9bus();
net2.a_bus{2}.set_component(gfmi);
net2.initialize();

con2 = controller_broadcast_PI_AGC(net2,[1,3],[1,3],-10,-500);
net2.add_controller_global(con2);

%% Simulation of Scenario 2
out2 = net2.simulate(time, u, u_idx, 'method', 'zoh');

%% Read output data
sampling_time2 = out2.t;

omega21 = out2.X{1}(:,2);
omega22 = out2.X{2}(:,12);
omega23 = out2.X{3}(:,2);

nbus2 = numel(out2.V);
V2complex = cell(nbus2, 1);
I2complex = cell(nbus2, 1);
power2    = cell(nbus2, 1);

for i = 1:numel(out2.V)
    Vi = out2.V{i};
    V2complex{i} = Vi(:,1) + 1j * Vi(:,2);
    Ii = out2.I{i};
    I2complex{i} = Ii(:,1) + 1j * Ii(:,2);
    power2{i} = V2complex{i} .* conj(I2complex{i});
end

%% Plot frequency deviation

fig1 = figure;
subplot(3, 5, 1)
plot(sampling_time1, omega11, 'LineWidth', 2)
hold on
plot(sampling_time2, omega21, 'LineWidth', 2)
title('Frequency deviation at Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Frequency deviation (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 5, 6)
plot(sampling_time1, omega12, 'LineWidth', 2)
hold on
plot(sampling_time2, omega22, 'LineWidth', 2)
title('Frequency deviation at Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Frequency deviation (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 5, 11)
plot(sampling_time1, omega13, 'LineWidth', 2)
hold on
plot(sampling_time2, omega23, 'LineWidth', 2)
title('Frequency deviation at Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Frequency deviation (PU)')
legend('Scenario 1','Scenario 2')

%% Plot active power
subplot(3, 5, 2)
plot(sampling_time1, real(power1{1}), 'LineWidth', 2)
hold on
plot(sampling_time2, real(power2{1}), 'LineWidth', 2)
title('Active power at Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Active power (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 5, 7)
plot(sampling_time1, real(power1{2}), 'LineWidth', 2)
hold on
plot(sampling_time2, real(power2{2}), 'LineWidth', 2)
title('Active power at Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Active power (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 5, 12)
plot(sampling_time1, real(power1{3}), 'LineWidth', 2)
hold on
plot(sampling_time2, real(power2{3}), 'LineWidth', 2)
title('Active power at Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Active power (PU)')
legend('Scenario 1','Scenario 2')
legend('Scenario 1','Scenario 2')

%% Plot reactive power
subplot(3, 5, 3)
plot(sampling_time1, imag(power1{1}), 'LineWidth', 2)
hold on
plot(sampling_time2, imag(power2{1}), 'LineWidth', 2)
title('Reactive power at Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Reactive power (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 5, 8)
plot(sampling_time1, imag(power1{2}), 'LineWidth', 2)
hold on
plot(sampling_time2, imag(power2{2}), 'LineWidth', 2)
title('Reactive power at Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Reactive power (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 5, 13)
plot(sampling_time1, imag(power1{3}), 'LineWidth', 2)
hold on
plot(sampling_time2, imag(power2{3}), 'LineWidth', 2)
title('Reactive power at Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Reactive power (PU)')
legend('Scenario 1','Scenario 2')
legend('Scenario 1','Scenario 2')

%% Plot voltage magnitude
subplot(3, 5, 4)
plot(sampling_time1, abs(V1complex{1}), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(V2complex{1}), 'LineWidth', 2)
title('Voltage magnitude at Bus 1')
xlim(x_lim)
ylim([1.035, 1.045])
xlabel('Time (s)')
ylabel('Voltage magnitude (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 5, 9)
plot(sampling_time1, abs(V1complex{2}), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(V2complex{2}), 'LineWidth', 2)
title('Voltage magnitude at Bus 2')
xlim(x_lim)
ylim([1.02, 1.03])
xlabel('Time (s)')
ylabel('Voltage magnitude (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 5, 14)
plot(sampling_time1, abs(V1complex{3}), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(V2complex{3}), 'LineWidth', 2)
title('Voltage magnitude at Bus 3')
xlim(x_lim)
ylim([1.02, 1.03])
xlabel('Time (s)')
ylabel('Voltage magnitude (PU)')
legend('Scenario 1','Scenario 2')

%% Plot current magnitude
subplot(3, 5, 5)
plot(sampling_time1, abs(I1complex{1}), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(I2complex{1}), 'LineWidth', 2)
title('Current magnitude at Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Current magnitude (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 5, 10)
plot(sampling_time1, abs(I1complex{2}), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(I2complex{2}), 'LineWidth', 2)
title('Current magnitude at Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Current magnitude (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 5, 15)
plot(sampling_time1, abs(I1complex{3}), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(I2complex{3}), 'LineWidth', 2)
title('Current magnitude at Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Current magnitude (PU)')
legend('Scenario 1','Scenario 2')