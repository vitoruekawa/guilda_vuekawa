classdef solar_farm < component

    properties (SetAccess = private)
        x_equilibrium
        pv_array
        vsc
        vsc_controller
        omega0
        gamma_pv
        Pst
        Qst
    end

    methods

        function obj = solar_farm(omega0, gamma_pv, pv_params, vsc_params, controller_params)
            obj.omega0 = omega0;
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

        function [dx, con] = get_dx_constraint(obj, t, x, V, I, u)
            I_ = -obj.gamma_pv * x(1:2);
            con = I - I_;

            dx = zeros(7, 1);

            idc = obj.pv_array.get_idc(x(3));
            mG = obj.vsc_controller.get_mG(x(4:7), x(1), x(2), V, x(3), obj.Pst, obj.Qst, u);
            dx(1:3) = obj.vsc.get_dx(x(1:3), idc, V, mG);
            dx(4:7) = obj.vsc_controller.get_dx(x(4:7), x(1), x(2), V, obj.Pst, obj.Qst);

        end

        function x_st = set_equilibrium(obj, V, I)
            P = V * conj(I);
            obj.Pst = real(P);
            obj.Qst = imag(P);

            P_s = obj.pv_array.set_equilibrium();
            [i_st, vdc_st] = obj.vsc.set_equilibrium(V, I, P_s);
            obj.pv_array.set_S(vdc_st);
            [chi_st, zeta_st] = obj.vsc_controller.set_equilibrium(i_st);

            x_st = [i_st; vdc_st; chi_st; zeta_st];
            obj.x_equilibrium = x_st;
        end

    end

end
