classdef network_sample3bus < power_network
% モデル ：Tutorial【簡単なモデルを用いた一連の解析実行例】ページで作成した3busモデル
%親クラス：power_networkクラス
%実行方法：net =network_sample3bus
%　引数　：なし
%　出力　：power_networkクラスのインスタンス

    methods
        function obj = network_sample3bus(avr_type, pss)
        
        %ブランチ(branch)の定義
            
            %母線1と母線2を繋ぐ送電線の定義
            y12 = 1.3652 - 11.6041j;
            y12 = 1/y12;
            branch12 = branch_pi(1,2,[real(y12),imag(y12)],0);
            obj.add_branch(branch12);
            
            y13 = -10.5107j;
            y13 = 1/y13;
            branch23 = branch_pi(2,3,[real(y13),imag(y13)],0);
            obj.add_branch(branch23);    
        
        %母線(bus)の定義
            shunt = [0,0];
            %母線1の定義
            bus_1 = bus_slack(2,0,shunt);
            obj.add_bus(bus_1);
            %母線2の定義
            bus_2 = bus_PQ(-3,0,shunt);
            obj.add_bus(bus_2);
            %母線3の定義
            bus_3 = bus_PV(0.5,2,shunt);
            obj.add_bus(bus_3);
        
            
        %機器(component)の定義
            
            %系統周波数の定義
            omega0 = 60*2*pi;
            
            %母線1に同期発電機の1軸モデルを付加
            Xd = 1.569; Xd_prime = 0.963; Xq = 1.569; Xq_prime = 0.963; 
            Tdo = 5.14; Tqo = 0.535; M = 100; D = 10;
            mac_data = table(Xd,Xd_prime,Xq,Xq_prime,Tdo,Tqo,M,D);
            comp1 = generator_2axis( omega0, mac_data);

            %母線2には定インピーダンスモデルを付加
            comp2 = load_impedance();
            obj.a_bus{2}.set_component(comp2);
            
            %母線3にも同期発電機の1軸モデルを付加
            Xd = 1.220;Xd_prime = 0.667; Xq = 1.220; Xq_prime = 0.677; 
            Tdo = 8.97; Tqo = 0.31; M = 12; D = 10;
            mac_data = table(Xd,Xd_prime,Xq,Xq_prime,Tdo,Tqo,M,D);
            comp3 = generator_2axis( omega0, mac_data);

            switch avr_type
                case 'avr2019sadamoto'
                    Ka = 20; Ta = 0.2; Ke = 1; Te = 0.314; Kf = 0.063; Tf = 0.35;
                    avr_data = table(Ka,Ta,Ke,Te,Kf,Tf);
                    comp1.set_avr(avr_sadamoto2019(avr_data));
                    comp3.set_avr(avr_sadamoto2019(avr_data));
                case 'IEEE_ST1'
                    t_tr = 0.015; k_ap = 200; k0 = 0.04; gamma_max = 7; gamma_min = -6.4;
                    avr_data = table(t_tr, k_ap, k0, gamma_max, gamma_min);
                    comp1.set_avr(avr_IEEE_ST1(avr_data));
                    comp3.set_avr(avr_IEEE_ST1(avr_data));
                % case false
                %     Ka = 20; Ta = 0.2; Ke = 1; Te = 0.314; Kf = 0.063; Tf = 0.35;
                %     avr_data = table(Ka,Ta,Ke,Te,Kf,Tf);
                %     comp3.set_avr(avr_sadamoto2019(avr_data));
            end
            
            if pss
                disp('hey')
                Kpss = 3.15; Tws = 10; Td1 = 0.01; Tn1 = 0.76; Td2 = 0.01; Tn2 = 0.76; 
                Vpss_max = 9e-2; Vpss_min = -9e-2;
                pss_data = table(Kpss, Tws, Td1, Tn1, Td2, Tn2, Vpss_max, Vpss_min);
                comp1.set_pss(pss_IEEE_PSS1(pss_data))
                comp3.set_pss(pss_IEEE_PSS1(pss_data))
            end

            obj.a_bus{1}.set_component(comp1);
            obj.a_bus{3}.set_component(comp3);
        
        %潮流計算の実行
        obj.initialize
        end
    end
end