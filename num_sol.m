syms zeta_st
vd_hat_st = zeta_st;
vq_hat_st = 0;

syms id_st iq_st  C omega_st vd_st vq_st Kvp Kvi xvd_st xvq_st
%isd_st_st = id_st - C * omega_st * vq_st + Kvi * xvd_st;
%isq_st_st = iq_st + C * omega_st * vd_st + Kvi * xvq_st;

isd_st_st = id_st - C * omega_st * vq_st + Kvp * (vd_hat_st - vd_st) + Kvi * xvd_st;
isq_st_st = iq_st + C * omega_st * vd_st + Kvp * (vq_hat_st - vq_st) + Kvi * xvq_st;


syms R L isd_st isq_st Kip Kii xid_st xiq_st
%vsd_st_st = vd_st + (R - omega_st*L) * isd_st + Kii * xid_st;
%vsq_st_st = vq_st + (R + omega_st*L) * isq_st + Kii * xiq_st;

vsd_st_st = vd_st + (R - omega_st*L) * isd_st + Kip * (isd_st_st - isd_st) + Kii * xid_st;
vsq_st_st = vq_st + (R + omega_st*L) * isq_st + Kip * (isq_st_st - isq_st) + Kii * xiq_st;

syms vdc_st vdc
md_st = 2 * vsd_st_st / vdc_st;
mq_st = 2 * vsq_st_st / vdc_st;
ix_st = (1/2) * [md_st, mq_st] * [isd_st_st; isq_st_st];
vsd_st = (1/2) * md_st * vdc_st;
vsq_st = (1/2) * mq_st * vdc_st;

syms kdc P_st Gdc P
idc_st_st = kdc * (vdc_st - vdc_st) + (P_st / vdc_st) + (Gdc * vdc_st + ((vdc_st * ix_st - P_st)/vdc_st));

syms i_tau_st
eq1 = idc_st_st - i_tau_st == 0;
% eq2 = idc_st - Gdc * vdc_st - ix_st == 0;
eq3 = - (R - omega_st*L) * isd_st - vd_st + vsd_st == 0;
eq4 = - (R + omega_st*L) * isq_st - vq_st + vsq_st == 0;
eq5 = C * omega_st * vq_st + isd_st - id_st == 0;
eq6 = - C * omega_st * vd_st + isq_st - iq_st == 0;
eq7 = vd_hat_st - vd_st == 0;
% eq8 = vq_hat_st - vq_st == 0;
eq9 = isd_st_st - isd_st == 0; 
eq10 = isq_st_st - isq_st == 0; 

eq = [eq1; eq3; eq4; eq5; eq6; eq7; eq9; eq10];
vars = [i_tau_st, isd_st, isq_st, xvd_st, xvq_st, xid_st, xiq_st, zeta_st];
sol = solve(eq, vars);