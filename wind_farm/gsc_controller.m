classdef gsc_controller < handle

    properties
        omega0
        vdc_st
        Qr_st
        f
        g
    end

    methods

        function obj = gsc_controller(controller_params)
            obj.set_controller(controller_params);
        end

        function nx = get_nx(obj)
            nx = 4;
        end

        % x = [Didg; Diqg; Zeta_dg; Zeta_qg];
        % u = [idG, iqG, vdc, udG, uqG, vgrid]
        function [dx, mG] = get_dx_mG(obj, x, u)
            dx = obj.f(x, u);
            mG = obj.g(x, u);
        end

        function x = initialize(obj, omega0, vdc_st, Qr_st)
            obj.omega0 = omega0;
            obj.vdc_st = vdc_st;
            obj.Qr_st = Qr_st;
            x = zeros(obj.get_nx, 1);
        end

        function set_controller(obj, controller_params)

            if istable(controller_params)
                Lg = controller_params{:, 'Lg'};
                Rg = controller_params{:, 'Rg'};
                Gdc = controller_params{:, 'Gdc'};
                Cdc = controller_params{:, 'Cdc'};
                tauG = controller_params{:, 'tauG'};
                Kp_dg = controller_params{:, 'Kp_dg'};
                Ki_dg = controller_params{:, 'Ki_dg'};
                Kp_qg = controller_params{:, 'Kp_qg'};
                Ki_qg = controller_params{:, 'Ki_qg'};
                Kp_dr = controller_params{:, 'Kp_dr'};
                Ki_dr = controller_params{:, 'Ki_dr'};
                Kp_qr = controller_params{:, 'Kp_qr'};
                Ki_qr = controller_params{:, 'Ki_qr'};
                m_max = controller_params{:, 'm_max'};
            end

            % How to calculate Qr?
            % In Sadamoto et. al the order of the subtractions is reversed
            obj.f = @(x, u)[(1 / tauG) * (Kp_dg * (obj.vdc_st - u(3)) + x(3) - u(1));
                            (1 / tauG) * (Kp_qg * (obj.Q_st - u(6) * [-u(2); u(1)]) + x(4) - u(2));
                            Ki_dg * (u(3) - vdc_st);
                            Ki_qg * (u(6) * [-u(2); u(1)] - obj.Q_st)];

            obj.g = @(x, u)[max(min(2 / u(3) * (u(4) + u(6)(1) + Lg * u(2) - Rg * x(1) - Lg / (obj.omega0 * tauG) * (Kp_dg * (obj.vdc_st - u(3)) + x(3) - u(1))), m_max), -m_max);
                            max(min(2 / u(3) * (u(5) + u(6)(2) - Lg * u(1) - Rg * x(2) - Lg / (obj.omega0 * tauG) * (Kp_qg * (obj.Q_st - u(6)' * [-u(2); u(1)]) + x(4) - u(2))), m_max), -m_max)]

        end

    end

end
