
classdef vsm < handle
    properties
        omega_st
        P_st
        V_st
        Jr
        Dp
        Kp
        Ki
    end
    
    methods
        function obj = vsm(vsm_params)
            obj.omega_st = vsm_params{:, 'omega_st'};
            obj.Jr = vsm_params{:, 'Jr'};
            obj.Dp = vsm_params{:, 'Dp'};
            obj.Kp = vsm_params{:, 'Kp'};
            obj.Ki = vsm_params{:, 'Ki'};
        end
        
        function nx = get_nx(obj)
            nx = 3;
        end
        
        function nu = get_nu(obj)
            nu = 0;
        end
        
        % State variables: theta and zeta (PI controller)
        function [d_delta, d_zeta, d_omega] = get_dx(obj, P, vdq, omega)
            d_delta = obj.omega_st * (omega - 1);
            d_zeta = obj.Ki * (obj.V_st - norm(vdq));
            d_omega = (obj.P_st - P) / obj.Jr + obj.Dp * (1 - omega) / obj.Jr;
        end
        
        function vdq_hat = calculate_vdq_hat(obj, vdq, zeta, omega)
            i_f = obj.Kp * (obj.V_st - norm(vdq)) + zeta;
            vdq_hat = [0; 2 * i_f * omega];
        end
        
        function set_constants(obj, V_st, P_st)
            obj.V_st = norm(V_st);
            obj.P_st = P_st;
        end
        
    end
    
end
