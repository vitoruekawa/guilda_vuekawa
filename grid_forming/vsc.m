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

        % State variables: vdc, isdq
        function dx = get_dx(obj, idc, vdc, ix, v_sab, i_sab, v_ab, i_ab)
            dx = [(idc - obj.R_dc * vdc - ix) / obj.C_dc;
                  (v_sab - obj.R_dc * i_sab - v_ab) / obj.L_f;
                  (i_sab - i_ab) / obj.C_f; ];
        end

        function i_ab = calculate_i_ab(obj, i_sab, d_v_ab)
            i_ab = i_sab - obj.C_f * d_v_ab;
        end

    end

end
