close all;
clear;
clc;

%% Initialize network
net = power_network;

%% Add branches
branch12 = branch_pi(1,2,[0.010,0.085],0);
net.add_branch(branch12);
branch23 = branch_pi(2,3,[0.017,0.092],0);
net.add_branch(branch23);

%% Add buses
shunt = [0,0];
bus_1 = bus_slack(2,0,shunt);
net.add_bus(bus_1);
bus_2 = bus_PV(0.5,2,shunt);
net.add_bus(bus_2);
bus_3 = bus_PQ(-3,0,shunt);
net.add_bus(bus_3);

%% Add components
omega0 = 60*2*pi;

Xd = 1.569; Xd_prime = 0.963; Xq = 0.963; T = 5.14; M = 100; D = 10;
mac_data = table(Xd,Xd_prime,Xq,T,M,D);
component1 = generator_1axis( omega0, mac_data);
net.a_bus{1}.set_component(component1);


%Xd = 1.220; Xd_prime = 0.667; Xq = 0.667; T = 8.97; M = 12; D = 10;
%mac_data = table(Xd,Xd_prime,Xq,T,M,D);
%comp2 = generator_1axis( omega0, mac_data);
%net.a_bus{2}.set_component(comp2);

%load_wind_params;
%comp2 = v0_wind_farm(wind_con, winp, sto_con, 10, 100, 100, omega0);
%net.a_bus{2}.set_component(comp2);

load_wind_params;
comp2 = wind_farm(omega0, Shm, Shl, gamma, 10, wt_params, dfig_params, b2b_params, rsc_con_params, gsc_con_params, battery_params);
net.a_bus{2}.set_component(comp2);

comp3 = load_impedance();
net.a_bus{3}.set_component(comp3);

%% Initialize
net.initialize

%% Plot simulation results
    time = [0,10,20,60,80,100];
    u_idx = 3;
    u = [0, 0.05, 0.1, 0.1, 0.1, 0.1;...
         0,    0,   0,   0,   0,   0];

    %入力信号波形プロット
    figure; hold on;
    u_percent = u*100;
    stairs(time,u_percent(1,:),'LineWidth',2)
    stairs(time,u_percent(2,:),'--','LineWidth',2)
    xlabel('時刻(s)','FontSize',15); 
    ylabel('定常値からの変化率(%)','FontSize',15);
    ylim([-20,20])
    legend({'インピーダンスの実部','インピーダンスの虚部'},'Location','southeast')
    title('母線3のインピーダンスの値の変化','FontSize',20)
    hold off;

    %解析実行
    out1 = net.simulate(time,u, u_idx);

    %データ抽出
    sampling_time = out1.t;
    omega1 = out1.X{1}(:,2);
    %omega2 = out1.X{2}(:,2);

    %プロット
    figure; hold on;
    %plot(sampling_time, omega2,'LineWidth',2)
    plot(sampling_time, omega1,'LineWidth',2)
    xlabel('時刻(s)','FontSize',15); 
    ylabel('周波数偏差','FontSize',15);
    legend('機器1の周波数偏差')
    title('各同期発電機の周波数偏差','FontSize',20)
    hold off