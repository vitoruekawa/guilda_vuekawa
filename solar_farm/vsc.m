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
        function dx = get_dx(obj, i, vdc, idc, vgrid, mG)
            dx = [(obj.omega0 / obj.L) * ([-obj.R, obj.L; -obj.L, -obj.R] * i + vgrid - mG * vdc / 2);
                  obj.omega0 / obj.Cdc * ((vgrid' * i + vdc * idc - obj.R * (i' * i))/(2 * vdc) - obj.Gsw * vdc)];
        end

        function [i_st, vdc_st] = calculate_equilibrium(obj, V, Pst, Qst, P_s)
            i_st =- [real(V), imag(V); imag(V), -real(V)] \ [Pst; Qst] / obj.gamma_pv;
            p_inp = Pst / obj.gamma_pv + obj.R * (i_st' * i_st);
            vdc_st = sqrt((P_s - p_inp) / (2 * obj.Gsw));
        end

    end

end
