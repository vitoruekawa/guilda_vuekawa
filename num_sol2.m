clc
clear

% Signals from power flow
syms Vreal_st Vimag_st Ireal_st Iimag_st Vangle Vabs P_st Q_st real

% State variables
syms vdc_st isd_st isq_st vd_st vq_st i_tau_st xvd_st xvq_st xid_st xiq_st delta_st zeta_st real
vdq_st_abs = sqrt(vd_st^2 + vq_st^2);

% Parameters
syms Gdc R L C d_w Ki kp ki Kvp Kvi Kip Kii kdc Xg vdc_st_st v_st real 

% Input value
syms id_st iq_st real

% Droop control reference signals
vd_hat_st = kp * (v_st - vdq_st_abs) + zeta_st;
vq_hat_st = 0;

% AC voltage control
omega_st = 1;
isd_st_st = id_st - C * omega_st * vq_st + Kvp * (vd_hat_st - vd_st) + Kvi * xvd_st;
isq_st_st = iq_st + C * omega_st * vd_st + Kvp * (vq_hat_st - vq_st) + Kvi * xvq_st;

% AC current control
vsd_st_st = vd_st + (R * isd_st - omega_st * L * isq_st) + Kip * (isd_st_st - isd_st) + Kii * xid_st;
vsq_st_st = vq_st + (R * isq_st + omega_st * L * isd_st) + Kip * (isq_st_st - isq_st) + Kii * xiq_st;

% Modulation and intermediate signals
md_st = 2 * vsd_st_st / vdc_st;
mq_st = 2 * vsq_st_st / vdc_st;
ix_st = (1/2) * [md_st, mq_st] * [isd_st; isq_st];
vsd_st = (1/2) * md_st * vdc_st;
vsq_st = (1/2) * mq_st * vdc_st;

% DC voltage control
idc_st_st = kdc * (vdc_st_st - vdc_st) + (P_st / vdc_st_st) + (Gdc * vdc_st + ((vdc_st * ix_st - P_st)/vdc_st_st));

% Dynamic equations
eq1 = idc_st_st - i_tau_st == 0;
eq2 = i_tau_st - Gdc * vdc_st - ix_st == 0;
eq3 = - (R * isd_st - omega_st * L * isq_st) - vd_st + vsd_st == 0;
eq4 = - (R * isq_st + omega_st * L * isd_st) - vq_st + vsq_st == 0;
eq5 = C * omega_st * vq_st + isd_st - id_st == 0;
eq6 = - C * omega_st * vd_st + isq_st - iq_st == 0;
eq7 = isd_st_st - isd_st == 0;
eq8 = isq_st_st - isq_st == 0;
eq9 = vd_hat_st - vd_st == 0;
eq10 = vq_hat_st - vq_st == 0;
eq11 = P_st - vdq_st_abs*Vabs*sin(delta_st - Vangle)/Xg == 0;
eq12 = v_st - vdq_st_abs == 0;
% eq11 = P_st - (Q_st + Vabs^2/Xg)*tan(delta_st - Vangle) == 0;
% eq12 = v_st - vdq_st_abs == 0;

eq = [eq1; eq2; eq3; eq4; eq5; eq6; eq7; eq8; eq9; eq10; eq11; eq12];
vars = [vdc_st, i_tau_st, isd_st, isq_st, vd_st, vq_st, xvd_st, xvq_st, xid_st, xiq_st, delta_st, zeta_st];
sol = solve(eq, vars,"IgnoreAnalyticConstraints",true);
solTable = struct2table(sol);