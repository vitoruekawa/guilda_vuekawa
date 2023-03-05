classdef vsc_controller < handle

    properties
        omega0
        gamma_pv
        Kp_dg
        Kp_qg
        Ki_dg
        Ki_qg
        tauG
        R
        L
        m_max
        Pst
        Qst
    end

    methods

        function obj = vsc_controller(controller_params, omega0, gamma_pv)
            obj.omega0 = omega0;
            obj.gamma_pv = gamma_pv;

            obj.Kp_dg = controller_params{:, 'Kp_dg'};
            obj.Kp_qg = controller_params{:, 'Kp_qg'};
            obj.Ki_dg = controller_params{:, 'Ki_dg'};
            obj.Ki_qg = controller_params{:, 'Ki_qg'};
            obj.tauG = controller_params{:, 'tauG'};
            obj.R = controller_params{:, 'R'};
            obj.L = controller_params{:, 'L'};
            obj.m_max = controller_params{:, 'm_max'};

        end

        function nx = get_nx(obj)
            nx = 4;
        end

        % x = [chi_d, chi_q, zeta_dg, zeta_qg];
        function dx = get_dx(obj, i, iref, P, Q)
            dx = [(iref - i) / obj.tauG;
                  obj.Ki_dg * (obj.Pst - P);
                  obj.Ki_qg * (obj.Qst - Q)];
        end

        function mG = calculate_mG(obj, x, ig, iref, vgrid, vdc, u)
            mG = 2 / vdc * (vgrid + [obj.L / (obj.omega0 * obj.tauG), obj.L; - obj.L, obj.L / (obj.omega0 * obj.tauG)] * ig -obj.R * x(1:2) - (obj.L / (obj.omega0 * obj.tauG)) * iref) + u;
            mG(mG > obj.m_max) = obj.m_max;
            mG(mG <- obj.m_max) = -obj.m_max;
        end

        function iref = calculate_iref(obj, zeta, P, Q)
            iref = [obj.Kp_dg * (obj.Pst - P); obj.Kp_qg * (obj.Qst - Q)] + zeta;
        end

        function [chi_st, zeta_st] = calculate_equilibrium(~, i_st)
            chi_st = i_st;
            zeta_st = i_st;
        end

        function set_PQst(obj, Pst, Qst)
            obj.Pst = Pst;
            obj.Qst = Qst;
        end

    end

end
