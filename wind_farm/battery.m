classdef battery < handle

    properties
        S
        Lb
        Rb
        Gb
        Cb
        omega0
    end

    methods

        function obj = battery(bat_params, omega0)
            obj.S = bat_params{:, 'S'};
            obj.Lb = bat_params{:, 'Lb'};
            obj.Rb = bat_params{:, 'Rb'};
            obj.Gb = bat_params{:, 'Gb'};
            obj.Cb = bat_params{:, 'Cb'};
            obj.omega0 = omega0;
        end

        function nx = get_nx(obj)
            nx = 2;
        end

        % x = [vb, idcp]
        % u = [vdc, uS]
        function dx = get_dx(obj, x, vdcp, uS)
            dx =[(- x(2) - obj.Gb * x(1)) * obj.omega0 / obj.Cb;
                (x(1) - obj.Rb * x(2) - vdcp) * obj.omega0 / obj.Lb];
        end

        function [vb_st, idcp_st] = calculate_equilibrium(obj, vdc_st)
            vb_st = obj.S * vdc_st / (1 + obj.Rb * obj.Gb);
            idcp_st = - obj.Gb * vb_st;
        end

        function bat = get_bat(obj)
            bat = obj.Gb * obj.S^2 / (obj.Gb * obj.Rb + 1);
        end

    end

end
