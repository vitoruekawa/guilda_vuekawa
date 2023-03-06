classdef wind_farm < component

    properties (SetAccess = private)
        omega0
        Shm
        Shl 
        x_equilibrium
        gamma
        wind_turbine
        dfig
        b2b_converter
        gsc_controller
        rsc_controller
        battery
        windspeed
    end

    methods

        function obj = wind_farm(omega0, Shm, Shl, gamma, windspeed, wt_params, dfig_params, b2b_params, rsc_con_params, gsc_con_params, battery_params)
            obj.omega0 = omega0;
            obj.Shm = Shm;
            obj.Shl = Shl;
            obj.gamma = gamma;
            obj.windspeed = windspeed;
            obj.wind_turbine = wind_turbine(wt_params);
            obj.dfig = dfig(dfig_params, gamma);
            obj.b2b_converter = b2b_converter(b2b_params, omega0);
            obj.gsc_controller = gsc_controller(gsc_con_params);
            obj.rsc_controller = rsc_controller(rsc_con_params);
            obj.battery = battery(battery_params, omega0);
        end

        function nx = get_nx(obj)
            nx = obj.wind_turbine.get_nx() + obj.dfig.get_nx() + obj.b2b.get_nx() + obj.gsc_controller.get_nx() + obj.rsc_controller.get_nx() + obj.battery.get_nx();
        end

        function nu = get_nu(obj)
            nu = 5;
        end

        function [dx, con] = get_dx_constraint(obj, t, x, V_grid, I_grid, u)
        end

        function x_st = set_equilibrium(obj, V, I)
            Vm = obj.Shm * V;
            Vl = obj.Shl * V;

            % Calculate DFIG setpoints and T_st
            [idr_st, iqr_st, ids_st, iqs_st] = obj.dfig.calculate_equilibrium(V, Vm, I);
            T_st = obj.dfig.calculate_T(idr_st, iqr_st, ids_st, iqs_st);

            % Calculate wind turbine setpoints
            [omega_l_st, omega_r_st, theta_st] = obj.wind_turbine.calculate_equilibrium(T_st);
            
            % Calculate vr_st (RSC controller setpoints)
            ir_st = idr_st + j * iqr_st;
            is_st = ids_st + j * iqs_st;
            vr_st = obj.dfig.calculate_vr_st(ir_st, is_st, omega_r_st);

            % Calculate B2B (RSC + DC-Link + GSC) setpoints
            bat = obj.battery.get_bat();
            [iGd_st, iGq_st, vdc_st] = obj.b2b_converter.calculate_equilibrium(Vl, vr_st, ir_st, bat);
            iG_st = iGd_st + j * iGq_st;

            % Calculate battery setpoints
            [vb_st, idcp_st] = obj.battery.calculate_equilibrium(vdc_st);

            % GSC controller setpoints
            chi_dG_st = iGd_st;
            chi_qG_st = iGq_st;
            zeta_dG_st = iGd_st;
            zeta_qG_st = iGq_st;
            
            % RSC controller setpoints
            chi_dR_st = real(vr_st); 
            chi_qR_st = imag(vr_st);

            x_st = [omega_l_st; omega_r_st; theta_st; idr_st; iqr_st; ids_st; iqs_st; iGd_st; iGq_st; chi_dG_st; chi_qG_st; zeta_dG_st; zeta_qG_st; chi_dR_st; chi_qR_st; vdc_st; vb_st; idcp_st];
            obj.x_equilibrium = x_st;
        end

    end

end
