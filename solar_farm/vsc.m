classdef vsc < handle

    properties
        omega0
        gamma_pv
        L
        R
        Gsw
        Cdc
    end

    methods

        function obj = vsc(vsc_params, omega0, gamma_pv)
            obj.omega0 = omega0;
            obj.gamma_pv = gamma_pv;

            obj.L = vsc_params{:, 'L'};
            obj.R = vsc_params{:, 'R'};
            obj.Gsw = vsc_params{:, 'Gsw'};
            obj.Cdc = vsc_params{:, 'Cdc'};

        end

        function nx = get_nx(obj)
            nx = 3;
        end

        % x = [id, iq, vdc];
        function dx = get_dx(obj, x, idc, vgrid, mG)
            dx = [obj.omega0 / obj.L * ([-obj.R, obj.L; -obj.L, -obj.R]) * x(1:2) + vgrid - mG * x(3) / 2;
                  obj.omega0 / obj.Cdc * ((vgrid' * x(1:2) + x(3) * idc - obj.R * (x(1:2)' * x(1:2)))/(2 * x(3)) - obj.Gsw * x(3))];
            % dx = [(obj.omega0 / obj.L) * ([-obj.R, obj.omega0 * obj.L; -obj.omega0 * obj.L, -obj.R] * x(1:2) + vgrid - mG * x(3) / 2);
            %                                (obj.omega0 / (2 * x(3) * obj.Cdc)) * (vgrid' * x(1:2) + idc * x(3) - obj.R * (x(1:2)' * x(1:2)) - 2 * obj.Gsw * x(3) ^ 2)];
        end

        function [i_st, vdc_st] = calculate_equilibrium(obj, V, Pst, Qst, P_s)
            i_st =- [real(V), imag(V); imag(V), -real(V)] \ [Pst; Qst] / obj.gamma_pv;
            p_inp = Pst / obj.gamma_pv + obj.R * (i_st' * i_st);
            vdc_st = sqrt((P_s - p_inp) / (2 * obj.Gsw));
        end

    end

end
