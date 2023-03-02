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
        S
    end

    methods

        function obj = solar_farm(omega0, gamma_pv, pv_params, vsc_params, controller_params)
            obj.omega0 = omega0;
            obj.gamma_pv = gamma_pv;
            obj.pv_array = pv_array(pv_params)
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

            P = V' * I_;
            Q = -I_(2) * V(1) + I_(1) * V(2);
            i = x(1:2);
            vdc = x(3);
            chi = x(4:5);
            zeta = x(6:7);

            idc = (obj.S * obj.pv_array.Vpv - obj.S ^ 2 * x(3)) / obj.pv_array.Rpv;

            mG = obj.vsc_controller.get_mG(x(4:7), x(1), x(2), V, x(3), u);
            dx(1:3) = obj.vsc.get_dx(x(1:3), idc, V, mG(1), mG(2));
            dx(4:7) = obj.vsc_controller.get_dx(x(4:7), x(1), x(2), V, obj.Pst, obj.Qst);

        end

        function x_st = set_equilibrium(obj, V, I)
            P = V * conj(I);
            obj.Pst = real(P);
            obj.Qst = imag(P);

            P_s = obj.pv_array.P_s;
            [i_st, vdc_st] = obj.vsc.set_equilibirum(V, I, P_s);
            chi_st = i_st;
            zeta_st = i_st;
            x_st = [i_st, vdc_st, chi_st, zeta_st];
            obj.x_equilibrium = x_st;
        end

    end

end
