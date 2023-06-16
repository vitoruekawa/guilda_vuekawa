%% Network base values
S_b = 100 * (10 ^ 6);
V_b = 230 * (10 ^ 3);
omega_b = 2 * pi * 60;
P_b = S_b; Q_b = S_b;
I_b = S_b / (sqrt(3) * V_b);
Z_b = (V_b ^ 2) / S_b;
L_b = Z_b / omega_b; 
C_b = 1 / (omega_b * Z_b);

%% Converter params
n = 200;
R_f = (0.001 * n) / Z_b;
L_f = (200e-6 / n) / L_b;
C_f = (300e-6 * n) / C_b;
R_dc = 1.2 / Z_b; % No need to multiply by n?
C_dc = (0.008 * n) / C_b;
idc_max = 1.2;% No need to multiply by n?
tau_dc = 50e-3;
vdc_st = 2.44e+3 / V_b;% No need to multiply by n?

%% VSC controller
Kp_v = 0.52;
Ki_v = 232.2;
Kp_i = 0.73;
Ki_i = 0.0059;
k_dc = 1.6e+3;

%% Droop controller
d_w = 2 * pi * 0.05 / omega_b;
omega_st = 1;
Ki=0.5;
Kp=0.001;

vsc_params = table(C_dc, R_dc, L_f, R_f, C_f);
dc_source_params = table(vdc_st, idc_max, tau_dc, k_dc, R_dc);
controller_params = table(L_f, C_f, R_f, Kp_v, Ki_v, Kp_i, Ki_i, vdc_st);
ref_model_params = table(omega_st, d_w, Kp, Ki);

clearvars -except vsc_params dc_source_params controller_params ref_model_params 
