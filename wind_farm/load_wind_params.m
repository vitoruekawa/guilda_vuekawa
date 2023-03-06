%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define wind farm parameters
%1- wind gen No.
%2- wind bus No. Specify the bus number to which the k-th wind plant connect
%3- air density in Kg/m^2
%4- Wind blade area in sq m
%5- wind speed in m/s (Note: this term is used for equilibrium calculation) 9.8(m/s) if B2B is ignored
%6-Jr
%7-Br
%8-Jg
%9-Bg
%10-Bdt
%11-Kdt
%12-Ng
%13-p
%14-Xs
%15-Xr
%16-Xm
%17-Rs
%18-Rr
%19-Rated power [MVA]
%20-Rated voltage (stator side)
%21-Rated voltage (rotor side)
%22-LG (H)   0.02308
%23-RG (ohm)   0.000238
%24-Gdc (ohm^-1)
%25-Cdc (uF)
%26-wind bus transformer reactance (pu)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
wind_con = [
            1 22 1.225 pi * (55.7 ^ 2) 9.9 55e6 27.8e3 390 3.034 945e3 2.7e8 90 4 ...
                4.0452 4.0523 3.95279 0.00488 0.00549 2 690 400 0.008 0.000238 1e-6 1e5 0.01;
            ];
%

ws = cell(1, size(wind_con, 1));

for lp = 1:size(wind_con, 1)
    ws{lp, 1}.typ = 1;

    switch ws{lp, 1}.typ
        case 1
            % sinusoidal wind speed
            ws{lp, 1}.w = 0; %frequency (rad/s)
            ws{lp, 1}.bias = wind_con(lp, 5); %bias of wind speed (m/s)
            ws{lp, 1}.fl = 0; %gain of wind speed variation (m/s)
        case 2
            % trapezoidal wind speed
            ws{lp, 1}.t = [0, 100];
            ws{lp, 1}.i = [wind_con(lp, 5), wind_con(lp, 5)];
    end

end

% 1-tauG
% 2-Kp_dG (positive)
% 3-Ki_dG (positive)
% 4-Kp_qG (negative)
% 5-Ki_qG (negative; larger than 1/(4*vdg_star))
% 6-Kp_dR
% 7-Ki_dR
% 8-Kp_qR
% 9-Ki_qR
% 10-PG_star (pu)
% 11-QG_star (pu)
% 12-e_method
% 13-duty saturation
% 14-KdPR (d-axis P gain of outer loop of RSC) -1
% 15-KdIR (d-axis I gain of outer loop of RSC) -0.2 (not)
% 16-KqPR (q-axis P gain of outer loop of RSC) -1
% 17-KqIR (q-axis I gain of outer loop of RSC) -0.2 (not)
winp = [
        0.1 0.1 1e-4 -0.01 -0.001 5 1 5 1 1e-4 1e-4 1 1e0 0.01 -0.01 0.01 0;
        ];

RSC_flag = 0; %1: outer RSC controller is PI   0: that is P

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Storage
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sto_flag = 0; %1- battery is used   0-not used

%1- Cb (uF)
%2- Lb (H)
%3- Gb (ohm^-1)
%4- Rb (ohm)
%5- initial S
%6- max duty
sto_con = [
           2e6 0.001 1e-6 0.001 2 0;
           ];

gamma = 100;
omega0 = 2 * pi * 60;

Pbase = 100 * 1e6;
Pwbase = wind_con(19) * 1e6;
Zbase = wind_con(20) ^ 2 / (wind_con(19) * 1e6);
Tbase = Pbase / (omega0 / 2);

% Wind turbine parameters
Jl = wind_con(6) * (omega0 / 2) / Tbase;
Bl = wind_con(7) * (omega0 / 2) / Tbase;
Jr = wind_con(8) * (omega0 / 2) / Tbase;
Br = wind_con(9) * (omega0 / 2) / Tbase;
dc = wind_con(10) * (omega0 / 2) / Tbase;
Kc = wind_con(11) / Tbase;
Ng = wind_con(12);
Cp = 0.410955856214939;
coeff_Pa = 0.5 * Cp * wind_con(3) * wind_con(4) / Pbase;
Pa_st = coeff_Pa * wind_con(5) ^ 3;
wt_params = table(Jl, Bl, Jr, Br, dc, Kc, Ng, coeff_Pa, Pa_st);

% Others
Shm = 1;
Shl = wind_con(21) / wind_con(20);

% DFIG parameters
Xs = wind_con(14) * (Pbase / Pwbase); % for grid base
Xr = wind_con(15) * (Pbase / Pwbase);
Xm = wind_con(16) * (Pbase / Pwbase);
Rs = wind_con(17) * (Pbase / Pwbase);
Rr = wind_con(18) * (Pbase / Pwbase);
Pr_st = winp(10);
Qr_st = winp(11);
dfig_params = table(Xs, Xr, Xm, Rs, Rr, Pr_st, Qr_st);

% B2B converter parameters
LG = wind_con(22) / Zbase * omega0 * (Pbase / Pwbase);
RG = wind_con(23) / Zbase * (Pbase / Pwbase);
Gsw = wind_con(24) * Zbase * (Pbase / Pwbase);
Cdc = wind_con(25) * 1e-6 * Zbase * omega0 * (Pbase / Pwbase);
Pr_st = winp(10);
Qr_st = winp(11);
b2b_params = table(LG, RG, Gsw, Cdc, Pr_st, Qr_st);

% RSC controller parameters
kappaPd = winp(6);
kappaPq = winp(8);
kappaId = winp(7);
kappaIq = winp(9);
KPdR = winp(14);
KPqR = winp(16);
m_max = winp(13);
rsc_con_params = table(kappaPd, kappaPq, kappaId, kappaIq, KPdR, KPqR, m_max);

% GSC controller parameters
tauG = winp(1);
KPdG = -winp(2);
KPqG = -winp(4);
KIdG = -winp(3);
KIqG = -winp(5);
gsc_con_params = table(LG, RG, tauG, KPdG, KPqG, KIdG, KIqG, m_max);

% Battery parameters
S = sto_con(5);
Lb = sto_con(2) / Zbase * omega0 * (Pbase / Pwbase);
Rb = sto_con(4) / Zbase * (Pbase / Pwbase);
Gb = sto_con(3) * Zbase * (Pbase / Pwbase);
Cb = sto_con(1) * 1e-6 * Zbase * omega0 * (Pbase / Pwbase);
battery_params = table(S, Lb, Rb, Gb, Cb);


