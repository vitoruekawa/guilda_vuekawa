classdef vsc < handle

    properties
        C_dc
        R_dc
        L_f
        R_f
    end

    methods

        function obj = vsc(vsc_params)
            obj.C_dc = vsc_params{:, 'C_dc'};
            obj.R_dc = vsc_params{:, 'G_dc'}; % Gdc in the paper
            obj.L_f = vsc_params{:, 'L_f'};
            obj.R_f = vsc_params{:, 'R_f'};
            obj.C_f = vsc_params{:, 'C_f'};
        end

        function nx = get_nx(obj)
            nx = 3;
        end

        function dx = get_dx(obj, idc, vdc, ix, isdq, omega, vdq, idq, vsdq)
            dx = [(idc - obj.R_dc * vdc - ix) / obj.C_dc;
                  (-obj.R_f * isdq - omega * obj.L_f * [0, -1; 1, 0] * isdq + vdq - vsdq) / obj.L_f;
                  (isdq - idq) / obj.C_f];
        end

        function idq = calculate_idq(obj, isdq, d_vdq)
            i_dq = i_sdq - obj.C_f * d_vdq;
        end

    end

end
