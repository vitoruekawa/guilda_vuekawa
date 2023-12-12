classdef vsm_2axis < handle

    properties
        omega_st
        Xd
        Xd_prime
        Xq
        Xq_prime
        Tdo
        Tqo
        M
        D
        P_st
        V_st
    end

    methods

        function obj = vsm_2axis(vsm_params)
            obj.omega_st = vsm_params{:, 'omega_st'};
            obj.Xd = vsm_params{:, 'Xd'};
            obj.Xd_prime = vsm_params{:, 'Xd_prime'};
            obj.Xq = vsm_params{:, 'Xq'};
            obj.Xq_prime = vsm_params{:, 'Xq_prime'};
            obj.Tdo = vsm_params{:, 'Tdo'};
            obj.Tqo = vsm_params{:, 'Tqo'};
            obj.M = vsm_params{:, 'M'};
            obj.D = vsm_params{:, 'D'};
        end

        function nx = get_nx(obj)
            nx = 4;
        end

        function nu = get_nu(obj)
            nu = 0;
        end

        % State variables: theta and zeta (PI controller)
        function [Efd, Efq] = get_Ef(obj, Ed, Eq, Vdq)
            Efd = obj.Xd * Eq / obj.Xd_prime - (obj.Xd / obj.Xd_prime - 1) * Vdq(2);
            Efq = obj.Xq * Ed / obj.Xq_prime - (obj.Xq / obj.Xq_prime - 1) * Vdq(1);
        end

        function vdq_hat = calculate_vdq_hat(obj, Idq, Ed, Eq)
            Vd = Ed + obj.Xq_prime * Idq(2);
            Vq = Eq - obj.Xd_prime * Idq(1);
            vdq_hat = [Vd; Vq];
        end

        function [d_delta, d_omega, d_Ed, d_Eq] = get_dx(obj, P, Pmech, Vfd, omega, Efd, Efq)
            d_delta = obj.omega_st * omega;
            d_omega = (- obj.D * omega + Pmech - P) / obj.M;

            d_Ed = (-Efq) / obj.Tqo;
            d_Eq = (-Efd + Vfd) / obj.Tdo;
        end

        function [delta_st, domega_st, Ed_st, Eq_st, Vfd] = get_equilibrium(obj, P, Q, Vabs, Vangle)

            delta_st = Vangle + atan((P * obj.Xq) / (Vabs^2 + Q * obj.Xq));

            domega_st = 0;

            Vd = Vabs * sin(delta_st - Vangle);
            Vq = Vabs * cos(delta_st - Vangle);
            Vfd = (obj.Xd * Vd * P + (obj.Xd * Q + Vabs ^ 2)* Vq) / Vabs^2;

            Ed_st = (1 - obj.Xq_prime / obj.Xq) * Vd;
            Eq_st = (1 - obj.Xd_prime / obj.Xd) * Vq + (obj.Xd_prime / obj.Xd) * Vfd;

        end

    end

end
