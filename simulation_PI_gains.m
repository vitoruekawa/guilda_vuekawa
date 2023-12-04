clearvars; clc;
omega_st = 2 * pi * 60;

%% Simulation an plot parameters
t_final = 20;
x_lim = [0, t_final];
y_lim = [-inf, inf];
plot_separated = true;

time = [0, 1, 5, t_final];
u_idx = 8;
u = [0, 0.4, 0.4, 0.4; ...
         0, 0, 0, 0];

%% Scenario 1: all machines are SM
net1 = network_IEEE9bus();
gen_model = 'generator_classical';

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

net1.a_bus{2}.set_component(comp);
net1.initialize()

con1 = controller_broadcast_PI_AGC(net1, [1, 3], [1, 3], -10, -500);
net1.add_controller_global(con1);

% Simulation of Scenario 1
out1 = net1.simulate(time, u, u_idx);

% Read output data
sampling_time1 = out1.t;
[V1, I1, P1] = calculate_output(out1);

%% Scenario 2: SM in bus 2 is replaced by GFMI
gainVals = [0.1, 0, 2, 0.5, 1, 2, 5, 10, 20, 50, 100];

Sr = 100 * 1e+3;
Vr = 480;

Ir = Sr / Vr;
Zr = Vr ^ 2 / Sr;
Lr = Zr;
Cr = 1 / Zr;

%% Multiple converter parameters (in network base)
R_f = 0;
L_f = 0.367 * 1e-3 / Lr;
C_f = 191 * 1e-6 / Cr;
vdc_st = 2.44 * 1e+3 / Vr;

R_g = 0;
L_g = 0.061 * 1e-2 / Lr;

%% Droop control
Jr = 12.8;
Dp = 4;
Kp = 0.001;
Ki = 0.5;

%% AC and DC current and voltage control
vals = [0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500];
Kp_v = 20;
Ki_v = 400;
Kp_i = 0.5;
Ki_i = 0.5;

% Initialize matrix to store peak values
peak_matrix = zeros(length(vals), length(vals));
average_matrix = zeros(length(vals), length(vals));

for i = 1:length(vals)
    for j = 1:length(vals)
        Kp_i = vals(i);
        Ki_i = vals(j);
        disp([Kp_i, Ki_i])

        vsc_params = table(L_f, R_f, C_f, R_g, L_g);
        controller_params = table(L_f, C_f, R_f, Kp_v, Ki_v, Kp_i, Ki_i, vdc_st);
        ref_model_params = table(omega_st, Jr, Dp, Kp, Ki);
        
        gfmi = gfmi_vsm(vsc_params, controller_params, ref_model_params);
        
        net2 = network_IEEE9bus();
        net2.a_bus{2}.set_component(gfmi);
        net2.initialize();
        
        con2 = controller_broadcast_PI_AGC(net2, [1, 3], [1, 3], -10, -500);
        net2.add_controller_global(con2);
        
        % Simulation of Scenario 2
        out2 = net2.simulate(time, u, u_idx);
        
        % Read output data
        sampling_time2 = out2.t;
        [V2, I2, P2] = calculate_output(out2);
        
        % Interpolation
        [V1, I1, P1] = resample(sampling_time1, sampling_time2, V1, I1, P1, 'pchip');
        
        % Calculate peak for the given metric ('max' in this case)
        peak = calculate_metric(V1, I1, P1, V2, I2, P2, 'max');
        average = calculate_metric(V1, I1, P1, V2, I2, P2, 'mean');
        
        % Store the peak value in the matrix
        peak_matrix(i, j) = peak.Vabs_diff_max(1); % Assuming Vabs_diff is the metric you want to plot
        average_matrix(i, j) = average.Vabs_diff_mean(1);
    end
end

figure;
surf(vals, vals, average_matrix);
xlabel('Kp_i');
ylabel('Ki_i');
zlabel('Peak Value');
title('3D Plot of Peak Value vs Kp_v and Ki_v');

% vsc_params = table(L_f, R_f, C_f, R_g, L_g);
% controller_params = table(L_f, C_f, R_f, Kp_v, Ki_v, Kp_i, Ki_i, vdc_st);
% ref_model_params = table(omega_st, Jr, Dp, Kp, Ki);

% gfmi = gfmi_vsm(vsc_params, controller_params, ref_model_params);

% net2 = network_IEEE9bus();
% net2.a_bus{2}.set_component(gfmi);
% net2.initialize();

% con2 = controller_broadcast_PI_AGC(net1, [1, 3], [1, 3], -10, -500);
% net2.add_controller_global(con2);

% % Simulation of Scenario 2
% out2 = net2.simulate(time, u, u_idx);

% % Read output data
% sampling_time2 = out2.t;
% [V2, I2, P2] = calculate_output(out2);

% %% Interpolation
% [V1, I1, P1] = resample(sampling_time1, sampling_time2, V1, I1, P1, 'pchip');

% peak = calculate_metric(V1, I1, P1, V2, I2, P2, 'max');
% average = calculate_metric(V1, I1, P1, V2, I2, P2, 'mean');

function [V, I, P] = calculate_output(out)
    nbus = numel(out.V);
    V = cell(nbus, 1);
    I = cell(nbus, 1);
    P = cell(nbus, 1);

    for i = 1:nbus
        V{i} = out.V{i}(:, 1) + 1j * out.V{i}(:, 2);
        I{i} = out.I{i}(:, 1) + 1j * out.I{i}(:, 2);
        P{i} = V{i} .* conj(I{i});
    end

end

function [Vresample, Iresample, Presample] = resample(time1, time2, V, I, P, method)
    [time1_unique, idx_unique] = unique(time1);
    nbus = numel(V);
    Vresample = cell(nbus, 1);
    Iresample = cell(nbus, 1);
    Presample = cell(nbus, 1);

    for i = 1:nbus
        Vresample{i} = interp1(time1_unique, V{i}(idx_unique), time2, method);
        Iresample{i} = interp1(time1_unique, I{i}(idx_unique), time2, method);
        Presample{i} = interp1(time1_unique, P{i}(idx_unique), time2, method);
    end

end

function outTable = calculate_metric(V1, I1, P1, V2, I2, P2, metric)
    nbus = 3;
    Vabs_diff = zeros(nbus,1);
    Vang_diff = zeros(nbus,1);
    Iabs_diff = zeros(nbus,1);
    Iang_diff = zeros(nbus,1);
    P_diff = zeros(nbus,1);
    Q_diff = zeros(nbus, 1);
    metricFunction = str2func(metric);

    for i = 1:nbus
        Vabs_diff(i) = metricFunction(abs(abs(V2{i}) - abs(V1{i})));
        Vang_diff(i) = metricFunction(abs(angle(V2{i}) - angle(V1{i})));
        Iabs_diff(i) = metricFunction(abs(abs(I2{i}) - abs(I1{i})));
        Iang_diff(i) = metricFunction(abs(angle(I2{i}) - angle(I1{i})));
        P_diff(i) = metricFunction(abs(real(P2{i}) - real(P1{i})));
        Q_diff(i) = metricFunction(abs(imag(P2{i}) - imag(P1{i})));
    end
    metricSuffix = ['_', metric];
    varNames = strcat({'Vabs_diff', 'Vang_diff', 'Iabs_diff', 'Iang_diff', 'P_diff', 'Q_diff'}, metricSuffix);

    outTable = table(Vabs_diff, Vang_diff, Iabs_diff, Iang_diff, P_diff, Q_diff, 'VariableNames', varNames);
end
