clearvars; clc; close all;
load_vsm_params;
gfmi = gfmi_vsm(vsc_params,controller_params,ref_model_params);

%% Simulation an plot parameters
t_final = 20;
x_lim = [0, t_final];
% x_lim = [0.997, 1.03];
y_lim = [-inf, inf];
plot_separated = true;

time = [0,1,5,t_final];
u_idx = 8;
u = [0,  0.1, 0.1, 0.1;...
     0,    0,   0,   0];

%% Scenario 1: all machines are SM
net1 = network_IEEE9bus();
gen_model = 'generator_classical';

% L_g = 2.6476e-04
% Xd = 0.8958;
% Xd_prime = 0.1198;
% Xq = 0.8645;
% Xq_prime = 0.1969;
Xd = 2.6476e-04;
Xd_prime = 0.1198;
Xq = 2.6476e-04;
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

% Kpss = 250;
% Tpss = 10;
% TL1p = 0.07;
% TL1 = 0.02;
% TL2p = 0.07;
% TL2 = 0.02;
% pss_params = table(Kpss, Tpss, TL1p, TL1, TL2p, TL2);
% pss = pss(pss_params);
% comp.set_pss(pss);

net1.a_bus{2}.set_component(comp);
net1.initialize()

con1 = controller_broadcast_PI_AGC(net1,[1,3],[1,3],-10,-500);
net1.add_controller_global(con1);

%% Simulation of Scenario 1
out1 = net1.simulate(time, u, u_idx);

% Ground fault disturbance
% option = struct();
% option.fault = {{[0 0.05], 5}};
% out1 = net1.simulate(time, option);

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
net2 = network_IEEE9bus();
net2.a_bus{2}.set_component(gfmi);
net2.initialize();

con2 = controller_broadcast_PI_AGC(net2,[1,3],[1,3],-10,-500);
net2.add_controller_global(con2);

%% Simulation of Scenario 2
out2 = net2.simulate(time, u, u_idx);

% Ground fault disturbance
% option = struct();
% option.fault = {{[0 0.05], 5}};
% out2 = net2.simulate(time, option);

%% Read output data
sampling_time2 = out2.t;

omega21 = out2.X{1}(:,2);
omega22 = out2.X{2}(:,13) - 1;
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

%% Interpolation
% Remove duplicate values from sampling_time1
[sampling_time1_unique, idx_unique] = unique(sampling_time1);
omega11_unique = omega11(idx_unique);
omega12_unique = omega12(idx_unique);
omega13_unique = omega13(idx_unique);
V11complex_unique = V1complex{1}(idx_unique);
V12complex_unique = V1complex{2}(idx_unique);
V13complex_unique = V1complex{3}(idx_unique);
I11complex_unique = I1complex{1}(idx_unique);
I12complex_unique = I1complex{2}(idx_unique);
I13complex_unique = I1complex{3}(idx_unique);
power11_unique = power1{1}(idx_unique);
power12_unique = power1{2}(idx_unique);
power13_unique = power1{3}(idx_unique);

% Interpolated values of simulation 1 to match same length of simulation 2
method = 'pchip';
omega11_interpolated = interp1(sampling_time1_unique, omega11_unique, sampling_time2, method, 'extrap');
omega12_interpolated = interp1(sampling_time1_unique, omega12_unique, sampling_time2, method, 'extrap');
omega13_interpolated = interp1(sampling_time1_unique, omega13_unique, sampling_time2, method, 'extrap');
V11complex_interpolated = interp1(sampling_time1_unique, V11complex_unique, sampling_time2, method, 'extrap');
V12complex_interpolated = interp1(sampling_time1_unique, V12complex_unique, sampling_time2, method, 'extrap');
V13complex_interpolated = interp1(sampling_time1_unique, V13complex_unique, sampling_time2, method, 'extrap');
I11complex_interpolated = interp1(sampling_time1_unique, I11complex_unique, sampling_time2, method, 'extrap');
I12complex_interpolated = interp1(sampling_time1_unique, I12complex_unique, sampling_time2, method, 'extrap');
I13complex_interpolated = interp1(sampling_time1_unique, I13complex_unique, sampling_time2, method, 'extrap');
power11_interpolated = interp1(sampling_time1_unique, power11_unique, sampling_time2, method, 'extrap');
power12_interpolated = interp1(sampling_time1_unique, power12_unique, sampling_time2, method, 'extrap');
power13_interpolated = interp1(sampling_time1_unique, power13_unique, sampling_time2, method, 'extrap');

%% Plot frequency deviation
smoothing_factor = 1e-10;

fig1 = figure;
subplot(3, 2, 1)
plot(sampling_time2, omega11_interpolated, 'LineWidth', 2)
hold on
plot(sampling_time2, omega21, 'LineWidth', 2)
title('Frequency deviation at Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Frequency deviation (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 2)
plot(sampling_time2, abs(omega21 - omega11_interpolated), 'LineWidth', 2)
title('Absolute error Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 3)
plot(sampling_time2, omega12_interpolated, 'LineWidth', 2)
hold on
plot(sampling_time2, omega22, 'LineWidth', 2)
title('Frequency deviation at Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Frequency deviation (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 4)
plot(sampling_time2, abs(omega22 - omega12_interpolated), 'LineWidth', 2)
title('Absolute error Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 5)
plot(sampling_time2, omega13_interpolated, 'LineWidth', 2)
hold on
plot(sampling_time2, omega23, 'LineWidth', 2)
title('Frequency deviation at Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Frequency deviation (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 6)
plot(sampling_time2, abs(omega23 - omega13_interpolated), 'LineWidth', 2)
title('Absolute error Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

%% Plot active power
fig2 = figure;
subplot(3, 2, 1)
plot(sampling_time2, real(power11_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, real(power2{1}), 'LineWidth', 2)
title('Active power at Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Active power (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 2)
plot(sampling_time2, abs(real(power2{1}) - real(power11_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 3)
plot(sampling_time2, real(power12_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, real(power2{2}), 'LineWidth', 2)
title('Active power at Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Active power (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 4)
plot(sampling_time2, abs(real(power2{2}) - real(power12_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 5)
plot(sampling_time2, real(power13_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, real(power2{3}), 'LineWidth', 2)
title('Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Active power (PU)')
legend('Scenario 1','Scenario 2')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 6)
plot(sampling_time2, abs(real(power2{3}) - real(power13_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

%% Plot reactive power
fig3 = figure;
subplot(3, 2, 1)
plot(sampling_time2, imag(power11_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, imag(power2{1}), 'LineWidth', 2)
title('Reactive power at Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Reactive power (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 2)
plot(sampling_time2, abs(imag(power2{1}) - imag(power11_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 3)
plot(sampling_time2, imag(power12_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, imag(power2{2}), 'LineWidth', 2)
title('Reactive power at Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Reactive power (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 4)
plot(sampling_time2, abs(imag(power2{2}) - imag(power12_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 5)
plot(sampling_time2, imag(power13_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, imag(power2{3}), 'LineWidth', 2)
title('Reactive power at Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Reactive power (PU)')
legend('Scenario 1','Scenario 2')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 6)
plot(sampling_time2, abs(imag(power2{3}) - imag(power13_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

%% Plot voltage magnitude
fig4 = figure;
subplot(3, 2, 1)
plot(sampling_time2, abs(V11complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(V2complex{1}), 'LineWidth', 2)
title('Voltage magnitude at Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Voltage magnitude (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 2)
plot(sampling_time2, abs(abs(V2complex{1}) - abs(V11complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 3)
plot(sampling_time2, abs(V12complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(V2complex{2}), 'LineWidth', 2)
title('Voltage magnitude at Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Voltage magnitude (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 4)
plot(sampling_time2, abs(abs(V2complex{2}) - abs(V12complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 5)
plot(sampling_time2, abs(V13complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(V2complex{3}), 'LineWidth', 2)
title('Voltage magnitude at Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Voltage magnitude (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 6)
plot(sampling_time2, abs(abs(V2complex{3}) - abs(V13complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

%% Plot voltage angle
fig5 = figure;
subplot(3, 2, 1)
plot(sampling_time2, angle(V11complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, angle(V2complex{1}), 'LineWidth', 2)
title('Voltage angle at Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Voltage angle')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 2)
plot(sampling_time2, abs(angle(V2complex{1}) - angle(V11complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 3)
plot(sampling_time2, angle(V12complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, angle(V2complex{2}), 'LineWidth', 2)
title('Voltage angle at Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Voltage angle')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 4)
plot(sampling_time2, abs(angle(V2complex{2}) - angle(V12complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 5)
plot(sampling_time2, angle(V13complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, angle(V2complex{3}), 'LineWidth', 2)
title('Voltage angle at Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Voltage angle')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 6)
plot(sampling_time2, abs(angle(V2complex{3}) - angle(V13complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

%% Plot current magnitude
fig6 = figure;
subplot(3, 2, 1)
plot(sampling_time2, abs(I11complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(I2complex{1}), 'LineWidth', 2)
title('Current magnitude at Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Current magnitude (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 2)
plot(sampling_time2, abs(abs(I2complex{1}) - abs(I11complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 3)
plot(sampling_time2, abs(I12complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(I2complex{2}), 'LineWidth', 2)
title('Current magnitude at Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Current magnitude (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 4)
plot(sampling_time2, abs(abs(I2complex{2}) - abs(I12complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 5)
plot(sampling_time2, abs(I13complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, abs(I2complex{3}), 'LineWidth', 2)
title('Current magnitude at Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Current magnitude (PU)')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 6)
plot(sampling_time2, abs(abs(I2complex{3}) - abs(I13complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

%% Plot current angle
fig7 = figure;
subplot(3, 2, 1)
plot(sampling_time2, angle(I11complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, angle(I2complex{1}), 'LineWidth', 2)
title('Current angle at Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Current angle')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 2)
plot(sampling_time2, abs(angle(I2complex{1}) - angle(I11complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 1')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 3)
plot(sampling_time2, angle(I12complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, angle(I2complex{2}), 'LineWidth', 2)
title('Current angle at Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Voltage angle')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 4)
plot(sampling_time2, abs(angle(I2complex{2}) - angle(I12complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 2')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')

subplot(3, 2, 5)
plot(sampling_time2, angle(I13complex_interpolated), 'LineWidth', 2)
hold on
plot(sampling_time2, angle(I2complex{3}), 'LineWidth', 2)
title('Current angle at Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('Current angle')
legend('Scenario 1','Scenario 2')

subplot(3, 2, 6)
plot(sampling_time2, abs(angle(I2complex{3}) - angle(I13complex_interpolated)), 'LineWidth', 2)
title('Absolute error Bus 3')
xlim(x_lim)
ylim(y_lim)
xlabel('Time (s)')
ylabel('abs(Scenario2 - Scenario1)')