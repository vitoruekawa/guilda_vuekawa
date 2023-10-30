%% Network base values
Sb = 100 * 1e+6;
Vb = 230 * 1e+3;
omega_st = 2 * pi * 60;

Ib = Sb / Vb;
Zb = (Vb^2) / Sb;
Lb = Zb / omega_st;
Cb = 1 / (omega_st * Zb);

%% Converter base values
Sr = 100 * 1e+3;
Vr = 480;

Ir = Sr / Vr;
Zr = Vr^2 / Sr;
Lr = Zr;
Cr = 1 / Zr;

%% Multiple converter parameters (in network base)
R_f = 0;
L_f = 0.367 * 1e-3 / Lr;
C_f = 191 * 1e-6 / Cr;
vdc_st = 2.44 * 1e+3 / Vr;

R_g = 0;
% L_g = 0.061 * 1e-2 / Lr;
L_g = 0.1198;

%% AC and DC current and voltage control
Kp_v = 20;
Ki_v = 400;
Kp_i = 2;
Ki_i = 100;

%% VSM 2 axis params 
% Xd = 0.8958;
% Xd_prime = 0.1198;
% Xq = 0.8958;
% Xq_prime = 0.1198;
% Tdo = 6;
% Tqo = 0.535;
% M = 12.8;
% D = 4;

Xd = 0.8958;
Xd_prime = 0.1198;
Xq = 0.8645;
Xq_prime = 0.1969;
Tdo = 6;
Tqo = 0.535;
M = 12.8;
D = 4;

%% Store parameters in tables
vsc_params = table(L_f, R_f, C_f, R_g, L_g);
controller_params = table(L_f, C_f, R_f, Kp_v, Ki_v, Kp_i, Ki_i, vdc_st);
ref_model_params = table(omega_st, Xd, Xd_prime, Xq, Xq_prime, Tdo, Tqo, M, D);

% clearvars -except vsc_params dc_source_params controller_params ref_model_params 