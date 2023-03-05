%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define solar farm parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%1-Rated power [MVA]
%2-Rated voltage
%3-LG (H)
%4-RG (ohm)
%5-Gdc (ohm^-1)
%6-Cdc (uF)
%7-ipv_s (A)  not used
%8-Rpv (ohm)
%9-Vpv (V)
%10-Vdc' (V)
pv_con = [2 690 0.002 0.000238 1e-6 1e4 7616.2 0.0366 568.128 289.3; ];

% 1-tauG
% 2-Kp_dG
% 3-Ki_dG
% 4-Kp_qG
% 5-Ki_qG
% 6-duty saturation
pvp = [0.7 -0.01 -0.1 0.01 0.1 1e0; ];

gamma_pv = 25;
omega0 = 2*pi*60;

Pbase = 100 * 1e6;
Ppvbase = pv_con(1) * 1e6;
Zbase = pv_con(2) ^ 2 / Ppvbase;

% VSC params
L = pv_con(3) / Zbase * omega0 * (Pbase / Ppvbase);
R = pv_con(4) / Zbase * (Pbase / Ppvbase);
Gsw = pv_con(5) * Zbase * (Pbase / Ppvbase);
Cdc = pv_con(6) * 1e-6 * Zbase * omega0 * (Pbase / Ppvbase);
vsc_params = table(L, R, Gsw, Cdc);

% VSC controller params
tauG = pvp(1);
Kp_dg = pvp(2);
Ki_dg = pvp(3);
Kp_qg = pvp(4);
Ki_qg = pvp(5);
m_max = pvp(6);
controller_params = table(R, L, tauG, Kp_dg, Ki_dg, Kp_qg, Ki_qg, m_max);

% PV array params
Rpv = pv_con(8) / Zbase * (Pbase / Ppvbase);
Vpv = pv_con(9) / pv_con(2);
Vdcp_s = pv_con(10) / pv_con(2);
pv_params = table(Rpv, Vpv, Vdcp_s);