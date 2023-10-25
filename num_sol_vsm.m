clc
clear

% Signals from power flow
syms Vreal_st Vimag_st Ireal_st Iimag_st Vangle Vabs P_st Q_st Vd_st Vq_st real

% State variables
syms isd_st isq_st vd_st vq_st id_st iq_st xvd_st xvq_st xid_st xiq_st delta_st zeta_st real
vdq_st_abs = sqrt(vd_st^2 + vq_st^2);

% Parameters
syms Gdc Rf Lf Cf Rg Lg d_w Ki kp ki Kvp Kvi Kip Kii kdc Xg vdc_st_st v_st real 

%% Droop control reference signals
omega_st = 1;
MfIf_st = kp * (v_st - vdq_st_abs) + zeta_st;
vd_hat_st = 0;
vq_hat_st = 2 * MfIf_st * omega_st;

%% AC voltage control
isd_st_st = id_st - omega_st * Cf * vq_st + Kvp * (vd_hat_st - vd_st) + Kvi * xvd_st;
isq_st_st = iq_st + omega_st * Cf * vd_st + Kvp * (vq_hat_st - vq_st) + Kvi * xvq_st;

%% AC current control
vsd_st_st = vd_st - omega_st * Lf * isq_st + Kip * (isd_st_st - isd_st) + Kii * xid_st;
vsq_st_st = vq_st + omega_st * Lf * isd_st + Kip * (isq_st_st - isq_st) + Kii * xiq_st;

%% Modulation and intermediate signals
md_st = 2 * vsd_st_st / vdc_st_st;
mq_st = 2 * vsq_st_st / vdc_st_st;
ix_st = (1/2) * [md_st, mq_st] * [isd_st; isq_st];
vsd_st = (1/2) * md_st * vdc_st_st;
vsq_st = (1/2) * mq_st * vdc_st_st;

%% Dynamic equations

% VSC
eq1 = omega_st * Lf * isq_st - vd_st + vsd_st == 0;
eq2 = - omega_st * Lf * isd_st - vq_st + vsq_st == 0;
eq3 = omega_st * Cf * vq_st + isd_st - id_st == 0;
eq4 = - omega_st * Cf * vd_st + isq_st - iq_st == 0;
eq5 = omega_st * Lg * iq_st - Vd_st + vd_st == 0;
eq6 = - omega_st * Lg * id_st - Vq_st + vq_st == 0;

% AC current control
eq7 = isd_st_st - isd_st == 0;
eq8 = isq_st_st - isq_st == 0;

% AC voltage control
eq9 = vd_hat_st - vd_st == 0;
eq10 = vq_hat_st - vq_st == 0;

% Output power
eq11 = P_st - (Q_st + Vabs^2/Xg)*tan(delta_st - Vangle) == 0;
eq12 = v_st - vdq_st_abs == 0;

eq = [eq1; eq2; eq3; eq4; eq5; eq6; eq7; eq8; eq9; eq10; eq11; eq12];
vars = [isd_st, isq_st, vd_st, vq_st, id_st, iq_st, xvd_st, xvq_st, xid_st, xiq_st, delta_st, zeta_st];
sol = solve(eq, vars);
solTable = struct2table(sol);
disp(solTable)