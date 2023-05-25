classdef droop < handle
    properties
        omega_st
        v_st
        P_st
        d_w
        Kp
        Ki
    end

    methods
        function obj = droop(droop_params)
            obj.omega_st = droop_params{:, 'omega_st'};
            obj.v_st = droop_params{:, 'v_st'};
            obj.P_st = droop_params{:, 'P_st'};
            obj.d_w = droop_params{:, 'd_w'};
            obj.Kp = droop_params{:, 'Kp'};
            obj.Ki = droop_params{:, 'Ki'};
        end

        function nx = get_nx(obj)
            nx = 2;
        end

        % State variables: theta and zeta (PI controller)
        function [d_theta, d_zeta] = get_dx(obj, P, Vdq)
            d_theta = obj.calculate_omega(P);
            d_zeta = obj.Ki * (obj.v_st - Vdq(1)); % Is Vdq(1) or abs(Vdq)?
        end

        function vdq_hat = calculate_vdq_hat(obj, Vdq, zeta)
            vdq_hat = [obj.Kp * (obj.v_st - Vdq(1)) + zeta; 0]; % Is Vdq(1) or abs(Vdq)?
        end

        function omega = calculate_omega(obj, P)
            omega = obj.omega_st + obj.d_w * (obj.P_st - P);
        end

        function set_equilibrium(obj, v_st, P_st)
            obj.v_st = norm(v_st);
            obj.P_st = P_st;
        end

    end

end
