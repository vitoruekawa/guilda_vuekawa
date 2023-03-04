classdef solar_farm < component

    properties (SetAccess = private)
        x_equilibrium
        gamma_pv
        pv_array
        vsc
        vsc_controller
        Pst
        Qst
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
            I = -obj.gamma_pv * x(1:2);
            con = I_grid - I;

            dx = zeros(obj.get_nx(), 1);

            P = V_grid' * I;
            Q = -I(2) * V_grid(1) + I(1) * V_grid(2);

            iref = obj.vsc_controller.calculate_iref(x(4:7), obj.Pst, obj.Qst, P, Q);
            idc = obj.pv_array.calculate_idc(x(3));

            mG = obj.vsc_controller.calculate_mG(x(4:7), x(1:2), iref, V_grid, x(3), u);

            dx(1:3) = obj.vsc.get_dx(x(1:3), idc, V_grid, mG);
            dx(4:7) = obj.vsc_controller.get_dx(x(1:2), iref, obj.Pst, obj.Qst, P, Q);

        end

        function x_st = set_equilibrium(obj, V, I) % ok
            P = V * conj(I);
            obj.Pst = real(P);
            obj.Qst = imag(P);

            P_s = obj.pv_array.calculate_P_s();
            [i_st, vdc_st] = obj.vsc.calculate_equilibrium(V, obj.Pst, obj.Qst, P_s);
            obj.pv_array.set_S(vdc_st);
            [chi_st, zeta_st] = obj.vsc_controller.calculate_equilibrium(i_st);

            x_st = [i_st; vdc_st; chi_st; zeta_st];
            obj.x_equilibrium = x_st;
        end

    end

end
