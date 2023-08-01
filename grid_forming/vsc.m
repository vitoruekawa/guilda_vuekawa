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
            obj.R_dc = vsc_params{:, 'R_dc'};
            obj.L_f = vsc_params{:, 'L_f'};
            obj.R_f = vsc_params{:, 'R_f'};
            obj.C_f = vsc_params{:, 'C_f'};
        end

        function nx = get_nx(obj)
            nx = 5;
        end

        function nu = get_nu(obj)
            nu = 0;
        end

        function [d_vdc, d_isdq, d_vdq] = get_dx(obj, idc, vdc, ix, isdq, Idq, omega, vdq, vsdq)
            d_vdc = (idc - (vdc / obj.R_dc) - ix) / obj.C_dc;
            d_isdq = (-(obj.R_f * eye(2) + omega * obj.L_f * [0, -1; 1, 0]) * isdq - vdq + vsdq) / obj.L_f;
            d_vdq =  (-obj.C_f * omega * [0, -1; 1, 0] * vdq + isdq - Idq) / obj.C_f;
        end

    end

end
