classdef solar_farm < component

    properties (Access = private)
        parameter_vec
        system_matrix
        x_st
        V_st
        I_st
    end

    properties (SetAccess = private)
        x_equilibrium
        V_equilibrium
        I_equilibrium
        vsc
        vsc_controller
        omega0
        gamma_pv
        ipv_s
        P_s
    end

    properties (SetAccess = public)
        parameter
    end

    methods

        function obj = solar_farm(omega0, gamma_pv, pv_params, vsc_params, controller_params)
            obj.omega0 = omega0;
            obj.gamma_pv = gamma_pv;

            Rpv = pv_params{:, 'Rpv'};
            Vpv_s = pv_params{:, 'Vpv_s'};
            Vdcp_s = pv_params{:, 'Vdcp_s'};
            obj.ipv_s = (Vpv_s - Vdcp_s) / Rpv;
            obj.P_s = obj.ipv_s * Vdcp_s;
            obj.vsc = vsc(vsc_params, omega0, gamma_pv);
            obj.vsc_controller = vsc_controller(controller_params, omega0, gamma_pv);
        end

        function out = get_nx(obj)
            out = obj.vsc.get_nx() + obj.vsc_controller.get_nx();
        end

        function nu = get_nu(obj)
            nu = 2;
        end

        function [dx, con] = get_dx_constraint(obj, t, x, V, I, u)
            dx_vsc_controller = obj.vsc_controller.get_dx()
            dx_vsc = obj.vsc.get_dx()
            dx = [dx_vsc, dx_vsc_controller]
        end

        function x_st = set_equilibrium(obj, V, I)

        end

    end

end
