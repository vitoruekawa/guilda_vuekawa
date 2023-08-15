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
R_f = (1/n) * 0.001 / Zr * (Vr^2 / Vb^2) * (Sb / Sr);
L_f = (1/n) * 200 * 1e-6 / Lr * (Vr^2 / Vb^2) * (Sb / Sr);
C_f = n * 300 * 1e-6 / Cr * (Vr^2 / Vb^2) * (Sb / Sr);
R_dc = 1.2 / Zr * (Vr^2 / Vb^2) * (Sb / Sr); % Why isn't this term multiplied by n?
C_dc = n * 0.008 / Cr * (Vr^2 / Vb^2) * (Sb / Sr);
vdc_st = 2.44 * 1e+3 / Vb;
tau_dc = 0.05;
idc_max = 1.2 * Ir / Ib; % Why isn't this term multiplied by n?

R_g = 0;
L_g = L_f * 3;

%% AC and DC current and voltage control
Kp_v = 0.52;
Ki_v = 232.2;
Kp_i = 0.73;
Ki_i = 0.0059;
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