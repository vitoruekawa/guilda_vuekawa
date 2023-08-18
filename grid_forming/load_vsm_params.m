%% Network base values
Sb = 100 * 1e+6;
Vb = 230 * 1e+3;
omega_st = 2 * pi * 60;

Ib = Sb / Vb;
Zb = (Vb^2) / Sb;
Lb = Zb / omega_st;
Cb = 1 / (omega_st * Zb);

%% Converter base values
Sr = 500 * 1e+3;
Vr = 2.44 * 1e+3;

Ir = Sr / Vr;
Zr = Vr^2 / Sr;
Lr = Zr;
Cr = 1 / Zr;

%% Multiple converter parameters (in network base)
R_f = 0;
L_f = 0.367 * 1e-3 / Lr;
C_f = 191 * 1e-6 / Cr;
vdc_st = 2.44 * 1e+3;

R_g = 0;
L_g = 0.061 * 1e-2 / Lr;

%% AC and DC current and voltage control

n=200;
Kp_v =(n)*0.52;
Ki_v =(n)*1.161022;
Kp_i =(1/n)*0.738891;
Ki_i =(1/n)*1.19;

k_dc = 1.6 * 1e3;

%% Droop control
d_w = 2 * pi * 0.6 / omega_st;
Jr = 12;
Dp = 10;
Kp = 0.001;
Ki = 0.5;

%% Store parameters in tables
vsc_params = table(L_f, R_f, C_f, R_g, L_g);
controller_params = table(L_f, C_f, R_f, Kp_v, Ki_v, Kp_i, Ki_i, vdc_st);
ref_model_params = table(Jr, Dp, Kp, Ki);

% clearvars -except vsc_params dc_source_params controller_params ref_model_params 