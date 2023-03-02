classdef vsc_controller < handle

    properties
        f
        mG
        omega0
        gamma_pv
        params
    end

    methods

        function obj = vsc_controller(controller_params, omega0, gamma_pv)
            obj.set_controller(controller_params);
            obj.params = controller_params;
            obj.omega0 = omega0;
            obj.gamma_pv = gamma_pv;
        end

        function nx = get_nx(obj)
            nx = 4;
        end

        % x = [chi_d, chi_q, zeta_dg, zeta_qg];
        function [dx, mG] = get_dx_mG(obj, x, idg, iqg, vdc, vgrid, P_s, Q_s, u)
            dx = obj.f(x, idg, iqg, vgrid, P_s, Q_s);
            mG = obj.mG(x, idg, iqg, vgrid, vdc, u);
        end

        function mG = get_mG(obj, x, idg, iqg, vgrid, vdc, u)
            mG = obj.mG(x, idg, iqg, vgrid, vdc, u);
        end

        function x = initialize(obj)
            x = zeros(obj.get_nx, 1);
        end

        function x = set_equilibrium(obj, V, I)
            % To implement
            % Maybe no need?
        end

        function set_controller(obj, controller_params)

            if istable(controller_params)
                Kp_dg = controller_params{:, 'Kp_dg'};
                Kp_qg = controller_params{:, 'Kp_qg'};
                Ki_dg = controller_params{:, 'Ki_dg'};
                Ki_qg = controller_params{:, 'Ki_qg'};
                tauG = controller_params{:, 'tauG'};
                R = controller_params{:, 'R'};
                L = controller_params{:, 'L'};
                m_max = controller_params{:, 'm_max'};
                obj.f = @(x, idg, iqg, vgrid, P_s, Q_s)[(1 / tauG) * (Kp_dg * (P_s + obj.gamma_pv * vgrid' * [idg; iqg]) + x(4) - idg);
                                                        (1 / tauG) * (Kp_qg * (Q_s + obj.gamma_pv * vgrid' * [-iqg; idg]) + x(5) - iqg);
                                                        Ki_dg * (P_s + obj.gamma_pv * vgrid' * [idg; iqg]);
                                                        Ki_qg * (Q_s + obj.gamma_pv * vgrid' * [-iqg; idg]); ];

                obj.mG = @(x, idg, iqg, vgrid, vdc, u) [max(min(2 / vdc * (u(1) + vgrid(1) + L * iqg - R * x(1) - L / (obj.omega0 * tauG) * (Kp_dg * (P_s + gamma_pv * vgrid' * [idg; iqg]) + x(3) - idg)), m_max), -m_max);
                                                        max(min(2 / vdc * (u(2) + vgrid(2) - L * idg - R * x(2) - L / (obj.omega0 * tauG) * (Kp_qg * (Q_s + gamma_pv * vgrid' * [-iqg; idg]) + x(4) - iqg)), m_max), -m_max); ];
            end

        end

    end

end
