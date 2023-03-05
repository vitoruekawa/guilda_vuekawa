classdef v0_solar_farm < component

    properties
        params
    end

    properties (SetAccess = private)
        x_equilibrium
    end

    methods

        function obj = v0_solar_farm(pv_con, pv_params, baseMVA, gamma_pv, omega0)
            Pbase = baseMVA * 1e6;
            Ppvbase = pv_con(1) * 1e6;
            Zbase = pv_con(2) ^ 2 / Ppvbase;
            obj.params.Lac = pv_con(3) / Zbase * omega0 * (Pbase / Ppvbase);
            obj.params.Rac = pv_con(4) / Zbase * (Pbase / Ppvbase);
            obj.params.Gsw = pv_con(5) * Zbase * (Pbase / Ppvbase);
            obj.params.Cdc = pv_con(6) * 1e-6 * Zbase * omega0 * (Pbase / Ppvbase);
            obj.params.Rpv = pv_con(8) / Zbase * (Pbase / Ppvbase);
            obj.params.Vpv = pv_con(9) / pv_con(2);

            obj.params.tau_ac = pv_params(1);
            obj.params.KPd = pv_params(2);
            obj.params.KId = pv_params(3);
            obj.params.KPq = pv_params(4);
            obj.params.KIq = pv_params(5);
            obj.params.m_max = pv_params(6);

            obj.params.omega0 = omega0;
            obj.params.gamma_pv = gamma_pv;

            obj.params.vdc_st = pv_con(10) / pv_con(2);
            obj.params.idc_st = (obj.params.Vpv - obj.params.vdc_st) / obj.params.Rpv;

        end

        function [dx, con] = get_dx_constraint(obj, t, x, V, I_, u)
            Rac = obj.params.Rac;
            Lac = obj.params.Lac;
            omega0 = obj.params.omega0;
            tau_ac = obj.params.tau_ac;
            KPd = obj.params.KPd;
            KPq = obj.params.KPq;
            KId = obj.params.KId;
            KIq = obj.params.KIq;
            Cdc = obj.params.Cdc;
            Gsw = obj.params.Gsw;

            Vpv = obj.params.Vpv;
            Rpv = obj.params.Rpv;

            S = obj.params.S;

            Qst = obj.params.Qst;
            Pst = obj.params.Pst;

            gamma_pv = obj.params.gamma_pv;

            dx = zeros(7, 1);
            I = -gamma_pv * x(1:2);
            con = I_ - I;

            P = V' * I;
            Q = -I(2) * V(1) + I(1) * V(2);
            i = x(1:2);
            vdc = x(7);
            Chi = x(3:4);
            zeta = x(5:6);
            iref = [KPd * (Pst - P); KPq * (Qst - Q)] + zeta;
            idc = S * (Vpv - S * vdc) / Rpv;

            m = 2 / vdc * (V + [Lac / omega0 / tau_ac, Lac; -Lac, Lac / omega0 / tau_ac] * i -Rac * Chi - Lac / omega0 / tau_ac * iref) + u;
            m(m > obj.params.m_max) = obj.params.m_max;
            m(m <- obj.params.m_max) = -obj.params.m_max;

            dx(1:2) = omega0 / Lac * ([-Rac, Lac; -Lac, -Rac] * i + V - m * vdc / 2);
            dx(3:4) = (iref - i) / tau_ac;
            dx(5) = KId * (Pst - P);
            dx(6) = KIq * (Qst - Q);
            dx(7) = omega0 / Cdc * ((V' * i + vdc * idc - Rac * (i' * i)) / (2 * vdc) - Gsw * vdc);
        end

        function nu = get_nu(obj)
            nu = 2;
        end

        function nx = get_nx(obj)
            nx = 7;
        end

        function x = set_equilibrium(obj, V, I, net)
            P = V * conj(I);
            obj.params.Pst = real(P);
            obj.params.Qst = imag(P);
            Vr = real(V);
            Vi = imag(V);

            gamma_pv = obj.params.gamma_pv;
            Rac = obj.params.Rac;
            x0 =- [Vr, Vi; Vi, -Vr] \ [obj.params.Pst; obj.params.Qst] / gamma_pv;
            vdc0 = sqrt((obj.params.vdc_st * obj.params.idc_st - obj.params.Pst / gamma_pv - Rac * (x0' * x0)) / (2 * obj.params.Gsw));
            obj.params.S = obj.params.vdc_st / vdc0;
            x = [x0; x0; x0; vdc0];
            obj.x_equilibrium = x;
        end

    end

end
