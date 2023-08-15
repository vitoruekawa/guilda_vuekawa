
classdef vsm < handle
    properties
        P_st
        V_st
        Jr
        Dp
        Kp
        Ki
    end
    
    methods
        function obj = vsm(vsm_params)
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
        function [d_delta, d_zeta, d_omega] = get_dx(obj, P, Vdq, omega)
            d_delta = omega - 1;
            d_zeta = obj.Ki * (obj.V_st - norm(Vdq));
            d_omega = (obj.P_st - P) / obj.Jr + obj.Dp * (1 - omega) / obj.Jr;
        end
        
        function vdq_hat = calculate_vdq_hat(obj, Vdq, zeta, omega)
            i_f = obj.Kp * (obj.V_st - norm(Vdq)) + zeta;
            vdq_hat = [2 * i_f * omega; 0];
        end
        
        function set_constants(obj, V_st, P_st)
            obj.V_st = norm(V_st);
            obj.P_st = P_st;
        end
        
    end
    
end
