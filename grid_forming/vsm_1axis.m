classdef vsm_1axis < handle

    properties
        omega_st
        Xd
        Xd_prime
        Xq
        Tdo
        M
        D
        P_st
        V_st
    end

    methods

        function obj = vsm_1axis(vsm_params)
            obj.omega_st = vsm_params{:, 'omega_st'};
            obj.Xd = vsm_params{:, 'Xd'};
            obj.Xd_prime = vsm_params{:, 'Xd_prime'};
            obj.Xq = vsm_params{:, 'Xq'};
            obj.Tdo = vsm_params{:, 'Tdo'};
            obj.M = vsm_params{:, 'M'};
            obj.D = vsm_params{:, 'D'};
        end

        function nx = get_nx(obj)
            nx = 3;
        end

        function nu = get_nu(obj)
            nu = 0;
        end

        % State variables: theta and zeta (PI controller)
        function Efd = get_Efd(obj, Eq, Vq)
            Efd = obj.Xd * Eq / obj.Xd_prime - (obj.Xd / obj.Xd_prime - 1) * Vq;
        end

        function vdq_hat = calculate_vdq_hat(obj, Idq, Eq)
            Vd = obj.Xq * Idq(2);
            Vq = Eq - obj.Xd_prime * Idq(1);
            vdq_hat = [Vd; Vq];
        end

        function [d_delta, d_omega, d_Eq] = get_dx(obj, P, Pmech, Vfd, omega, Efd)
            d_delta = obj.omega_st * omega;
            d_omega = (- obj.D * omega + Pmech - P) / obj.M;
            d_Eq = (-Efd + Vfd) / obj.Tdo;
        end

        function [delta_st, domega_st, Eq_st, Vfd] = get_equilibrium(obj, P, Q, Vabs, Vangle)
            delta_st = Vangle + atan(P * obj.Xq / (Q * obj.Xq + Vabs ^ 2));

            domega_st = 0;

            Vd = Vabs * sin(delta_st - Vangle);
            Vq = Vabs * cos(delta_st - Vangle);
            Vfd = (obj.Xd * Vd * P + (obj.Xd * Q + Vabs ^ 2)* Vq) / Vabs^2;
            Eq_st = (1 - obj.Xd_prime / obj.Xd) * Vq + Vfd *obj.Xd_prime / obj.Xd;

            % Enum = Vabs ^ 4 + Q ^ 2 * obj.Xd_prime * obj.Xq + Q * Vabs ^ 2 * obj.Xd_prime + Q * Vabs ^ 2 * obj.Xq + P ^ 2 * obj.Xd_prime * obj.Xq;
            % Eden = Vabs * sqrt(P ^ 2 * obj.Xq ^ 2 + Q ^ 2 * obj.Xq ^ 2 + 2 * Q * Vabs ^ 2 * obj.Xq + Vabs ^ 4);
            % Eq_st = Enum / Eden;
            % Vfd = obj.Xd * Eq_st / obj.Xd_prime - (obj.Xd / obj.Xd_prime - 1) * Vabs * cos(delta_st - Vangle);
        end

    end

end
