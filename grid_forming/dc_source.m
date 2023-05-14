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
            obj.P_st = params{:, 'P_st'};
            obj.vdc_st = params{:, 'vdc_st'};
            obj.idc_max = params{:, 'idc_max'};
            obj.tau_dc = params{:, 'tau_dc'};
            obj.k_dc = params{:, 'k_dc'};
            obj.R_dc = params{:, 'R_dc'};
        end

        function nx = get_nx(obj)
            nx = 1
        end

        % dx = i_tau
        function dx = get_dx(obj, i_tau, ix, vdc, P)
            idc_st = obj.k_dc * (obj.vdc_st - vdc) + obj.P_st / obj.vdc_st + (obj.R_dc * vdc + (vdc * ix - P) / obj.vdc_st);
            dx = (idc_st - i_tau) / obj.tau_dc;
        end

        % DC current limitation (Equation 3)
        function idc = calculate_idc(obj, i_tau)
            idc = sign(i_tau) * min(abs(i_tau), obj.idc_max);
        end

        function i_tau_st = calculate_i_tau_st(obj, vdq_st, idq_st, Z, x_idq_st, Ki_i)
            i_tau_st = obj.R_dc * obj.vdc_st + (vdq_st + Z * idq_st + Ki_i * eye(2) * x_idq_st) * idq_st / obj.vdc_st;
        end

    end

end
