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

        function [d_delta, d_omega, d_Ed, d_Eq] = get_dx(obj, P, Pmech, Vfd, omega, Efd, Efq)
            d_delta = obj.omega_st * omega;
            d_omega = (- obj.D * omega + Pmech - P) / obj.M;

            d_Ed = (-Efq) / obj.Tqo;
            d_Eq = (-Efd + Vfd) / obj.Tdo;
        end

        % function [delta_st, omega_st, Ed_st, Eq_st, Vfd] = get_equilibrium(obj, P, Q, I)
        %     syms del e
        %     Iq = real(I)*cos(del)+imag(I)*sin(del);
        %     Id = real(I)*sin(del)-imag(I)*cos(del);
        %     Ed_st = (obj.Xq-obj.Xq_prime)*Iq;
        %     Vd = obj.Xq_prime * Iq + Ed_st;
        %     Vq = -obj.Xd_prime * Id + e;

        %     eq1 = P-Vq*(Iq)-Vd*(Id) == 0;
        %     eq2 = Q-Vq*(Id)+Vd*(Iq) == 0;
        %     eqs = [eq1; eq2];
        %     sol = solve(eqs);
        %     %solve(eq) gives more than 1 solution
        %     if(sol.e(1)>0);delta_st = double(sol.del(1)); Eq_st = double(sol.e(1));
        %     else; delta_st = double(sol.del(2)); Eq_st = double(sol.e(2));end

        %     Iq = real(I)*cos(delta_st)+imag(I)*sin(delta_st);
        %     Id = real(I)*sin(delta_st)-imag(I)*cos(delta_st);
        %     Ed_st = (obj.Xq-obj.Xq_prime)*Iq;

        %     omega_st = 0;
        %     Vfd = Eq_st + (obj.Xd-obj.Xd_prime)*Id;

        % end

        function [delta_st, domega_st, Ed_st, Eq_st, Vfd] = get_equilibrium(obj, P, Q, Vabs, Vangle, Iabs, Iangle)

            delta_st = Vangle + atan((P * obj.Xq) / (Vabs^2 + Q * obj.Xq));

            domega_st = 0;

            Eqnum = P ^ 2 * obj.Xd_prime * obj.Xq + Q ^ 2 * obj.Xd_prime * obj.Xq + Vabs ^ 2 * Q * obj.Xq + Vabs ^ 2 * Q * obj.Xd_prime + Vabs ^ 4;
            Eqden = Vabs * sqrt(P ^ 2 * obj.Xq ^ 2 + Q ^ 2 * obj.Xq ^ 2 + 2 * Vabs ^ 2 * Q * obj.Xq + Vabs ^ 4);
            Eq_st = Eqnum / Eqden;

            Ednum = (obj.Xq - obj.Xq_prime) * Vabs * P;
            Edden = sqrt(P ^ 2 * obj.Xq ^ 2 + Q ^ 2 * obj.Xq ^ 2 + 2 * Vabs ^ 2 * Q * obj.Xq + Vabs ^ 4);
            Ed_st = Ednum / Edden;

            Vfd = Eq_st + (obj.Xd - obj.Xd_prime) * Iabs * sin(delta_st - Iangle);
        end

    end

end
