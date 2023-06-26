clc
clear
syms Vd_st Vq_st Id_st Iq_st delta_st
vd_st = Vd_st * sin(delta_st) - Vq_st * cos(delta_st);
vq_st = Vd_st * cos(delta_st) + Vq_st * sin(delta_st);
id_st = Id_st * sin(delta_st) - Iq_st * cos(delta_st);
iq_st = Id_st * cos(delta_st) + Iq_st * sin(delta_st);

% Droop control reference signals
syms zeta_st
vd_hat_st = zeta_st;
vq_hat_st = 0;

% AC voltage control
syms C Kvp Kvi xvd_st xvq_st
isd_st_st = id_st + Kvp * (vd_hat_st - vd_st) + Kvi * xvd_st;
isq_st_st = iq_st + Kvp * (vq_hat_st - vq_st) + Kvi * xvq_st;

% AC current control
syms R L isd_st isq_st Kip Kii xid_st xiq_st
omega_st = 1;
vsd_st_st = vd_st + (R * isd_st - omega_st * L * isq_st) + Kip * (isd_st_st - isd_st) + Kii * xid_st;
vsq_st_st = vq_st + (R * isq_st + omega_st * L * isd_st) + Kip * (isq_st_st - isq_st) + Kii * xiq_st;

% Modulation and intermediate signals
syms vdc_st_st vdc_st
md_st = 2 * vsd_st_st / vdc_st_st;
mq_st = 2 * vsq_st_st / vdc_st_st;
ix_st = (1/2) * [md_st, mq_st] * [isd_st; isq_st];
vsd_st = (1/2) * md_st * vdc_st;
vsq_st = (1/2) * mq_st * vdc_st;

% DC voltage control
syms kdc Gdc P_st
idc_st_st = kdc * (vdc_st_st - vdc_st) + (P_st / vdc_st_st) + (Gdc * vdc_st + ((vdc_st * ix_st - P_st)/vdc_st_st));

% Dynamic equations
syms i_tau_st
eq1 = idc_st_st - i_tau_st == 0;
eq2 = i_tau_st - Gdc * vdc_st - ix_st == 0;
eq3 = - (R * isd_st - omega_st * L * isq_st) - vd_st + vsd_st == 0;
eq4 = - (R * isq_st + omega_st * L * isd_st) - vq_st + vsq_st == 0;
eq5 = isd_st - id_st == 0;
eq6 = isq_st - iq_st == 0;
eq7 = isd_st_st - id_st == 0;
eq8 = isq_st_st - iq_st == 0;
eq9 = vd_hat_st - vd_st == 0;
eq10 = vq_hat_st - vq_st == 0;

eq = [eq1; eq2; eq3; eq4; eq5; eq6; eq7; eq8; eq9; eq10];
vars = [vdc_st, i_tau_st, isd_st, isq_st, xvd_st, xvq_st, xid_st, xiq_st, delta_st, zeta_st];
sol = solve(eq, vars);
solTable = struct2table(sol);
disp(solTable)
