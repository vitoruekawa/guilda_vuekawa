classdef solar_farm < component

    properties (SetAccess = private)
        x_equilibrium
        gamma_pv
        pv_array
        vsc
        vsc_controller
    end

    methods

        function obj = solar_farm(omega0, gamma_pv, pv_params, vsc_params, controller_params)
            obj.gamma_pv = gamma_pv;
            obj.pv_array = pv_array(pv_params);
            obj.vsc = vsc(vsc_params, omega0, gamma_pv);
            obj.vsc_controller = vsc_controller(controller_params, omega0, gamma_pv);
        end

        function out = get_nx(obj)
            out = obj.pv_array.get_nx() + obj.vsc.get_nx() + obj.vsc_controller.get_nx();
        end

        function nu = get_nu(obj)
            nu = 2;
        end

        function [dx, con] = get_dx_constraint(obj, t, x, V_grid, I_grid, u)
            i = x(1:2); vdc = x(3); chi = x(4:5); zeta = x(6:7);
            dx = zeros(obj.get_nx(), 1);

            I = -obj.gamma_pv * i;
            con = I_grid - I;

            P = V_grid' * I;
            Q = -I(2) * V_grid(1) + I(1) * V_grid(2);

            iref = obj.vsc_controller.calculate_iref(zeta, P, Q);
            idc = obj.pv_array.calculate_idc(vdc); 
            mG = obj.vsc_controller.calculate_mG([chi; zeta], i, iref, V_grid, vdc, u);

            dx(1:3) = obj.vsc.get_dx(i, vdc, idc, V_grid, mG);
            dx(4:7) = obj.vsc_controller.get_dx(i, iref, P, Q);

        end

        function set_equilibrium(obj, V, I)
            P = V * conj(I);
            Pst = real(P);
            Qst = imag(P);

            P_s = obj.pv_array.calculate_P_s();
            [i_st, vdc_st] = obj.vsc.calculate_equilibrium(V, Pst, Qst, P_s);
            obj.pv_array.set_S(vdc_st);
            [chi_st, zeta_st] = obj.vsc_controller.calculate_equilibrium(i_st);
            obj.vsc_controller.set_PQst(Pst, Qst);

            obj.x_equilibrium = [i_st; vdc_st; chi_st; zeta_st];
        end

    end

end