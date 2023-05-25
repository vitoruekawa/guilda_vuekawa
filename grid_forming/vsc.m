classdef vsc < handle

    properties
        C_dc
        R_dc
        L_f
        R_f
        C_f
    end

    methods

        function obj = vsc(vsc_params)
            obj.C_dc = vsc_params{:, 'C_dc'};
            obj.R_dc = vsc_params{:, 'R_dc'}; % Gdc in the paper
            obj.L_f = vsc_params{:, 'L_f'};
            obj.R_f = vsc_params{:, 'R_f'};
            obj.C_f = vsc_params{:, 'C_f'};
        end

        function nx = get_nx(obj)
            nx = 3;
        end

        function [d_vdc, d_isdq] = get_dx(obj, idc, vdc, ix, isdq, omega, vdq, vsdq)
            d_vdc = (idc - obj.R_dc * vdc - ix) / obj.C_dc;
            d_isdq = (-obj.R_f * isdq - omega * obj.L_f * [0, -1; 1, 0] * isdq + vdq - vsdq) / obj.L_f;
            % d_vdq = (isdq - idq) / obj.C_f;
        end

    end

end
