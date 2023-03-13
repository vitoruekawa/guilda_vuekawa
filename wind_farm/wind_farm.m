classdef wind_farm < component

    properties (SetAccess = private)
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
            obj.Shm = Shm;
            obj.Shl = Shl;
            obj.gamma = gamma;
            obj.windspeed = windspeed;
            obj.wind_turbine = wind_turbine(wt_params, omega0);
            obj.dfig = dfig(dfig_params, gamma);
            obj.b2b_converter = b2b_converter(b2b_params, omega0);
            obj.gsc_controller = gsc_controller(gsc_con_params, gamma, omega0);
            obj.rsc_controller = rsc_controller(rsc_con_params);
            obj.battery = battery(battery_params, omega0);
        end

        function nx = get_nx(obj)
            nx = obj.wind_turbine.get_nx() + obj.dfig.get_nx() + obj.b2b_converter.get_nx() ...
                + obj.gsc_controller.get_nx() + obj.rsc_controller.get_nx() + obj.battery.get_nx();
        end

        function nu = get_nu(obj)
            nu = 5;
        end

        function [dx, con] = get_dx_constraint(obj, t, x, V_grid, I_grid, u)
            % Assign state variables to local variables for better readability
            omega_l = x(1); omega_r = x(2); thetaT = x(3); idr = x(4); iqr = x(5); ids = x(6); iqs = x(7);
            iG = x(8:9); chiG = x(10:11); zetaG = x(12:13); chiR = x(14:15); vdc = x(16); vb = x(17); idcp = x(18);

            % Assign input signal to local variables for better readability
            uG = u(1:2); uR = u(3:4); uS = u(5);

            dx = zeros(obj.get_nx(), 1);

            % Calculate voltage in the machine-side and load-side (transformers)
            Vm = obj.Shm * V_grid;
            Vl = obj.Shl * V_grid;

            % Calculate current output and constraint
            I = obj.gamma * (obj.Shm * [ids; iqs] - obj.Shl * iG);
            con = I_grid - I;

            % Calculate RSC duty cycle
            Vabs_m = sqrt(Vm' * Vm);
            ir_ref = obj.rsc_controller.calculate_irref(Vabs_m, omega_r);
            mR = obj.rsc_controller.calculate_mR([idr; iqr], ir_ref, chiR, uR, vdc);

            % Calculate GSC duty cycle
            Qr = Vl' * [-iG(2); iG(1)];
            iGref = obj.gsc_controller.calculate_iGref(vdc, Qr, zetaG);
            mG = obj.gsc_controller.calculate_mG(chiG, iG, iGref, Vl, vdc, uG);

            % Calculate power from wind and electrical torque from DFIG
            Pa = obj.wind_turbine.calculate_Pa(obj.windspeed);
            T = obj.dfig.calculate_T(idr, iqr, ids, iqs);

            % Calculate wind turbine state derivatives (omega_l, omega_r, thetaT)
            dx(1:3) = obj.wind_turbine.get_dx(x(1:3), Pa, T);

            vr = mR * vdc / 2;
            dx(4:7) = obj.dfig.get_dx(x(4:7), omega_r, Vm, vr);

            [idc, vdcp] = obj.battery.calculate_idc_vdcp(idcp, vdc, uS);
            dx_b2b = obj.b2b_converter.get_dx([iG; vdc], [idr; iqr], idc, mR, mG, Vl);
            dx(8:9) = dx_b2b(1:2);

            dx(10:13) = obj.gsc_controller.get_dx(iGref, iG, vdc, Qr);
            dx(14:15) = obj.rsc_controller.get_dx([idr; iqr], ir_ref);
            dx(16) = dx_b2b(3);
            dx(17:18) = obj.battery.get_dx(x(17:18), vdcp);
            dx = real(dx);

        end

        function set_equilibrium(obj, V, I)
            Vm = obj.Shm * V;
            Vl = obj.Shl * V;

            % Calculate DFIG setpoints and T_st
            [idr_st, iqr_st, ids_st, iqs_st] = obj.dfig.calculate_equilibrium(V, Vm, I);
            T_st = obj.dfig.calculate_T(idr_st, iqr_st, ids_st, iqs_st);

            % Calculate wind turbine setpoints
            [omega_l_st, omega_r_st, theta_st] = obj.wind_turbine.calculate_equilibrium(T_st);

            % Calculate vr_st (RSC controller setpoints)
            ir_st = idr_st + 1i * iqr_st;
            is_st = ids_st + 1i * iqs_st;
            vr_st = obj.dfig.calculate_vr_st(ir_st, is_st, omega_r_st);

            % Calculate B2B (RSC + DC-Link + GSC) setpoints
            bat = obj.battery.get_bat();
            [iGd_st, iGq_st, vdc_st] = obj.b2b_converter.calculate_equilibrium(Vl, vr_st, ir_st, bat);

            % Calculate battery setpoints
            [vb_st, idcp_st] = obj.battery.calculate_equilibrium(vdc_st);

            % GSC controller setpoints
            chi_dG_st = iGd_st;
            chi_qG_st = iGq_st;
            zeta_dG_st = iGd_st;
            zeta_qG_st = iGq_st;
            obj.gsc_controller.set_equilibrium(vdc_st);

            % RSC controller setpoints
            chi_dR_st = real(vr_st);
            chi_qR_st = imag(vr_st);
            obj.rsc_controller.set_equilibrium(abs(Vm), omega_r_st, [idr_st; iqr_st]);

            % Store the equilibrium point
            obj.x_equilibrium = [omega_l_st; omega_r_st; theta_st; idr_st; iqr_st; ids_st; ...
                                     iqs_st; iGd_st; iGq_st; chi_dG_st; chi_qG_st; zeta_dG_st; ...
                                     zeta_qG_st; chi_dR_st; chi_qR_st; vdc_st; vb_st; idcp_st];

        end

    end

end
