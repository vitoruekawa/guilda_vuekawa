classdef vsc_controller < handle

    properties
        L_f
        C_f
        R_f
        Kp_v
        Ki_v
        Kp_i
        Ki_i
        vdc_st
        isdq_st
    end

    methods

        function obj = vsc_controller(controller_params)
            obj.L_f = controller_params{:, 'L_f'};
            obj.C_f = controller_params{:, 'C_f'};
            obj.R_f = controller_params{:, 'R_f'};
            obj.Kp_v = controller_params{:, 'Kp_v'};
            obj.Ki_v = controller_params{:, 'Ki_v'};
            obj.Kp_i = controller_params{:, 'Kp_i'};
            obj.Ki_i = controller_params{:, 'Ki_i'};
            obj.vdc_st = controller_params{:, 'vdc_st'};
        end

        function nx = get_nx(obj)
            nx = 4;
        end

        % State variables: x_vdq, x_idq
        function [dx_vdq, dx_idq] = get_dx(obj, vdq, isdq, vdq_hat)
            dx_vdq = vdq_hat - vdq;
            dx_idq = obj.isdq_st - isdq;
        end

        % Modulation
        function m = calculate_m(obj, idq, vdq, omega, vdq_hat, isdq, x_vdq, x_idq)
            % AC voltage control
            obj.isdq_st = idq + obj.C_f * omega * [0, -1; 1, 0] * vdq + obj.Kp_v * eye(2) * (vdq_hat - vdq) + obj.Ki_v * eye(2) * x_vdq;

            % AC current control
            vsdq_st = vdq + (obj.R_f * eye(2) + obj.L_f * omega * [0, -1; 1, 0]) * isdq + obj.Kp_i * eye(2) * (obj.isdq_st - isdq) + obj.Ki_i * eye(2) * x_idq;

            % Modulation
            m = 2 * vsdq_st / obj.vdc_st;
        end

        function [x_vdq_st, x_idq_st] = calculate_equilibrium(obj, omega_st, vdq_st, idq_st)
            x_vdq_st = - (obj.C_f * omega_st / obj.Ki_v) * [0, -1; 1, 0] * vdq_st;
            x_idq_st =  2 * (obj.R_f * eye(2) + obj.L_f * omega_st * [0, -1; 1, 0]) * idq_st / obj.Ki_i;
        end

    end

end
