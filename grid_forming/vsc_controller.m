classdef vsc_controller < handle

    properties
        L_f
        C_f
        Kp_v
        Ki_v
        Kp_i
        Ki_i
        i_ac_max
        vdc_st
        isdq_st
    end

    methods

        function obj = vsc_controller(controller_params)
            obj.L_f = controller_params{:, 'L_f'};
            obj.C_f = controller_params{:, 'C_f'};
            obj.Kp_v = controller_params{:, 'Kp_v'};
            obj.Ki_v = controller_params{:, 'Ki_v'};
            obj.Kp_i = controller_params{:, 'Kp_i'};
            obj.Ki_i = controller_params{:, 'Ki_i'};
            obj.i_ac_max = controller_params{:, 'i_ac_max'};
            obj.vdc_st = controller_params{:, 'vdc_st'}
        end

        function nx = get_nx(obj)
            nx = 4;
        end

        % State variables: x_vdq, x_idq
        function dx = get_dx(obj, vdq, isdq, vdq_hat)
            dx_vdq = vdq_hat - vdq;
            dx_idq = obj.isdq_st - isdq;
            dx = [dx_vdq; dx_idq];
        end

        % Modulation
        function m = calculate_m(obj, idq, vdq, omega, vdq_hat, isdq, x_vdq, x_idq, theta)
            % AC voltage control
            obj.isdq_st = idq + obj.C_f * omega * [0, -1; 1, 0] * vdq + obj.Kp_v * eye(2) * (vdq_hat - vdq) - obj.Ki_v * eye(2) * x_vdq;

            % AC current control
            vsdq_st = vdq + (obj.L_f * omega * [0, -1; 1, 0] + obj.R_f * eye(2)) * isdq + obj.Kp_i * eye(2) * (obj.isdq_st - isdq) + obj.Ki_i * x_idq;

            % Modulation
            vsab_st = vsdq_st * [sin(theta), cos(theta); -cos(theta), sin(theta)];
            m = 2 * vsab_st / obj.vdc_st;
        end

    end

end
