classdef b2b_converter < handle

    properties
        LG
        RG
        Gsw
        Cdc
        Pr_st % Are these real parameters? Should I store them in this class?
        Qr_st
        omega0
    end

    methods

        function obj = b2b_converter(b2b_params, omega0)
            obj.LG = b2b_params{:, 'LG'};
            obj.RG = b2b_params{:, 'RG'};
            obj.Gsw = b2b_params{:, 'Gsw'};
            obj.Cdc = b2b_params{:, 'Cdc'};
            obj.Pr_st = b2b_params{:, 'Pr_st'};
            obj.Qr_st = b2b_params{:, 'Qr_st'};
            obj.omega0 = omega0;
        end

        function nx = get_nx(obj)
            nx = 3;
        end

        % x = [idg, iqg, vdc];
        % u = [idr, iqr, idc, mdR, mqR, mdG, mqG, vgrid]
        function dx = get_dx(obj, x, ir, idc, mR, mG, Vl)
            dx = [([-obj.RG, obj.LG; -obj.LG, -obj.RG] * x(1:2) + Vl - mG * x(3) / 2) * obj.omega0 / obj.LG;
                    ((x(1:2)' * Vl + ir' * mR * x(3) / 2 - obj.RG * (x(1:2)' * x(1:2))) / (2 * x(3)) - obj.Gsw * x(3) + idc / 2) * obj.omega0 / obj.Cdc];

        end

        function [iGd_st, iGq_st, vdc_st] = calculate_equilibrium(obj, Vl, vr_st, ir_st, bat) % bat = Gb * S^2 / (Gb * Rb + 1)
            iG_st = conj((obj.Pr_st + 1j * obj.Qr_st) / Vl);
            vdc0 = sqrt(1 / (2 * obj.Gsw + bat) * (real(Vl * conj(iG_st) + vr_st * conj(ir_st)) - obj.RG * iG_st * conj(iG_st)));

            iGd_st = real(iG_st);
            iGq_st = imag(iG_st);
            vdc_st = vdc0;
        end

    end

end
