classdef vsc < handle

    properties
        f
        omega0
        gamma_pv
        params
    end

    methods

        function obj = vsc(vsc_params, omega0, gamma_pv)
            obj.set_vsc(vsc_params);
            obj.params = vsc_params;
            obj.omega0 = omega0;
            obj.gamma_pv = gamma_pv;
        end

        function nx = get_nx(obj)
            nx = 3;
        end

        % x = [id, iq, vdc];
        function dx = get_dx(obj, x, idc, vgrid, md, mq)
            dx = obj.f(x, idc, vgrid, md, mq);
        end

        function x = initialize(obj)
            x = zeros(obj.get_nx, 1);
        end

        function [i_st, vdc_st] = set_equilibrium(obj, V, I, P_s)
            Pow = conj(I) * V;
            i_st = inv(real(V), imag(V); imag(V), -real(V)) * (-Pow / obj.gamma_pv);
            p_inp = Pow(1) / obj.gamma_pv + obj.params{:, 'R'} * i_st' * i_st;
            vdc_st = sqrt((P_s - p_inp) / (2 * obj.params{:, 'Gsw'}));
        end

        function set_vsc(obj, vsc_params)

            if istable(vsc_params)
                L = vsc_params{:, 'L'};
                R = vsc_params{:, 'R'};
                Gsw = vsc_params{:, 'Gsw'};
                Cdc = vsc_params{:, 'Cdc'};
                % Is it really necessary to multiply by omega0 here?
                obj.f = @(x, idc, vgrid, md, mq)[(obj.omega0 / L) * (-R * x(1) + obj.omega0 * L * x(2) + real(vgrid) - (md * x(3)) / 2);
                                                 (obj.omega0 / L) * (-obj.omega0 * L * x(1) - R * x(2) + imag(vgrid) - (mq * x(3)) / 2)
                                                 (obj.omega0 / (2 * x(3) * Cdc)) * (x(1) * real(vgrid) + x(2) * imag(vgrid) - x(3) * idc - R(x(1) ^ 2 + x(2) ^ 2) - 2 * Gsw * x(3) ^ 2)];
            end

        end

    end

end
