classdef dc_source < handle

    properties
        P_st
        vdc_st
        idc_max
        tau_dc
        k_dc
        R_dc 
    end

    methods

        function obj = dc_source(params)
            obj.vdc_st = params{:, 'vdc_st'};
            obj.idc_max = params{:, 'idc_max'};
            obj.tau_dc = params{:, 'tau_dc'};
            obj.k_dc = params{:, 'k_dc'};
            obj.R_dc = params{:, 'R_dc'};
        end

        function nx = get_nx(obj)
            nx = 1;
        end

        function nu = get_nu(obj)
            nu = 0;
        end

        % dx = i_tau
        function dx = get_dx(obj, i_tau, vdc, P, ix)
            idc_st = obj.k_dc * (obj.vdc_st - vdc) + (obj.P_st / obj.vdc_st) + (vdc / obj.R_dc) + ((vdc * ix - P) / obj.vdc_st);
            dx = (idc_st - i_tau) / obj.tau_dc;
        end

        % DC current limitation (Equation 3)
        function idc = calculate_idc(obj, i_tau)
            idc = sign(i_tau) * min(abs(i_tau), obj.idc_max);
        end

        function set_constants(obj, P_st)
            obj.P_st = P_st;
        end

    end

end