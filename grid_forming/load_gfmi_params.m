%% Network base values
S_b=100*(10^6);
V_b=230*(10^3);%L-L rms voltage
f_b=60;omega_st=2*pi*f_b;
P_b=S_b;Q_b=S_b;
I_b=S_b/(sqrt(3)*V_b);
Z_b=(V_b^2)/S_b;L_b=Z_b/omega_st;C_b=1/(omega_st*Z_b);

%% Converter params
V1_rms=1000/V_b; %Low voltage side
V_st=sqrt(2/3)*V1_rms;
vdc_st=3*V_st;
n=200;
C_dc=0.008*(n)/C_b;
R_f=0.001/n/Z_b;
L_f=(1/n)*200*10^-6/L_b;
C_f=n*300*10^-6/C_b;
R_dc=(vdc_st^2*V_b^2)/(0.05*(S_b))/Z_b;

%DC source
tau_dc=0.05;
% grid-forming converter control----------------
i_loss_dc=V_b*vdc_st/(Z_b*R_dc);
idc_max=(1.15*(S_b/(V_b*vdc_st))+i_loss_dc)/I_b;%dc source saturation limits

% DC voltage control--------------------------------
eta_1= omega_st/(vdc_st*V_b);
d_w=2*pi*0.06; % what is this, and why 2*pi*0.5?
k_dc=eta_1/(vdc_st*V_b*d_w/S_b);
%k_dc = 1.6667e+03;

% AC voltage control--------------------------------
Ki=2*0.25;
Kp=0.001;

% Voltage loop----------------------------------------
n=200;
Kp_v =0.52;
Ki_v =(n)*1.161022;

% Current loop
Kp_i =0.738891;
Ki_i =(1/n)*1.19;

%% Store parameters in tables
vsc_params = table(C_dc, R_dc, L_f, R_f, C_f);
dc_source_params = table(vdc_st, idc_max, tau_dc, k_dc, R_dc);
controller_params = table(L_f, C_f, R_f, Kp_v, Ki_v, Kp_i, Ki_i, vdc_st);
ref_model_params = table(omega_st, V_st, d_w, Kp, Ki);

clearvars -except vsc_params dc_source_params controller_params ref_model_params 
