classdef droop < handle
    properties
        omega_st
        V_st
        P_st
        d_w
        Kp
        Ki
    end

    methods
        function obj = droop(droop_params)
            obj.omega_st = droop_params{:, 'omega_st'};
            obj.d_w = droop_params{:, 'd_w'};
            obj.Kp = droop_params{:, 'Kp'};
            obj.Ki = droop_params{:, 'Ki'};
        end

        function nx = get_nx(obj)
            nx = 2;
        end

        function nu = get_nu(obj)
            nu = 0;
        end

        % State variables: theta and zeta (PI controller)
        function [d_delta, d_zeta] = get_dx(obj, P, vdq)
            d_delta = obj.d_w * (obj.P_st - P);
            d_zeta = obj.Ki * (obj.V_st - norm(vdq)); 
        end

        function vdq_hat = calculate_vdq_hat(obj, vdq, zeta)
            vdq_hat = [obj.Kp * (obj.V_st - norm(vdq)) + zeta; 0];
        end

        function omega = calculate_omega(obj, P)
            omega = 1 + obj.d_w * (obj.P_st - P);
        end

        function set_constants(obj, V_st, P_st)
            obj.V_st = norm(V_st);
            obj.P_st = P_st;
        end

    end

end
