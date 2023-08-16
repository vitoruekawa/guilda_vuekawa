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
Vr = 1 * 1e+3;

Ir = Sr / Vr;
Zr = Vr^2 / Sr;
Lr = Zr / omega_st;
Cr = 1 / (omega_st * Zr);

%% Multiple converter parameters (in network base)
n = 200;
R_f = (1/n) * 0.001 / Zb;
L_f = (1/n) * 200 * 1e-6 / Lb;
C_f = n * 300 * 1e-6 / Cb;
R_dc = 1.2 / Zb; % Why isn't this term multiplied by n?
C_dc = n * 0.008 / Cb;
vdc_st = 2.44 * 1e+3 / Vb;
tau_dc = 0.05;
idc_max = 1.2 * Ir / Ib; % Why isn't this term multiplied by n?

R_g = 0;
L_g = 0.00008;

%% AC and DC current and voltage control

n=200;
Kp_v =(n)*0.52;
Ki_v =(n)*1.161022;
Kp_i =(1/n)*0.738891;
Ki_i =(1/n)*1.19;

k_dc = 1.6 * 1e3;

%% Droop control
d_w = 2 * pi * 0.6 / omega_st;
Kp = 0.001;
Ki = 0.5;

%% Store parameters in tables
vsc_params = table(C_dc, R_dc, L_f, R_f, C_f, R_g, L_g);
dc_source_params = table(vdc_st, idc_max, tau_dc, k_dc, R_dc);
controller_params = table(L_f, C_f, R_f, Kp_v, Ki_v, Kp_i, Ki_i, vdc_st);
ref_model_params = table(omega_st, d_w, Kp, Ki);

clearvars -except vsc_params dc_source_params controller_params ref_model_params 