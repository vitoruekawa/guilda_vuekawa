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
  1 22 1.225 pi*(55.7^2) 9.9 55e6 27.8e3 390 3.034 945e3 2.7e8 90 4 ...      
      4.0452 4.0523 3.95279 0.00488 0.00549 2 690 400 0.008 0.000238 1e-6 1e5 0.01;
  ];
%

ws = cell(1, size(wind_con,1));
for lp=1:size(wind_con,1)
  ws{lp,1}.typ = 1;
  switch ws{lp,1}.typ
    case 1
      % sinusoidal wind speed
      ws{lp,1}.w = 0; %frequency (rad/s)  
      ws{lp,1}.bias = wind_con(lp,5); %bias of wind speed (m/s)
      ws{lp,1}.fl = 0; %gain of wind speed variation (m/s)
    case 2
      % trapezoidal wind speed      
      ws{lp,1}.t = [0,100];
      ws{lp,1}.i = [wind_con(lp,5), wind_con(lp,5)];      
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
  0.1  0.1  1e-4  -0.01  -0.001  5  1  5  1  1e-4  1e-4 1 1e0 0.01 -0.01 0.01 0;
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
  2e6  0.001  1e-6  0.001  2  0;
  ];