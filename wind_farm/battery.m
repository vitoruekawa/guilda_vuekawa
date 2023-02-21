classdef battery < handle

    properties
        f
        g
        omega0
    end

    methods

        function obj = battery(bat_params)
            obj.set_battery(bat_params);
        end

        function nx = get_nx(obj)
            nx = 1;
        end

        % x = [vb, idcp]
        % u = [vdc, uS]
        function [dx, idc] = get_idc(obj, x, u)
            dx = obj.f(x, u);
            idc = obj.g(x, u);
        end

        function x = initialize(obj, omega0)
            obj.omega0 = omega0; % this is expected to be grid's frequency
            x = zeros(obj.nx, 1);
        end

        function set_battery(obj, bat_params)

            if istable(bat_params)
                Rb = bat_params{:, 'Rb'};
                Lb = bat_params{:, 'Lb'};
                Gb = bat_params{:, 'Gb'};
                Cb = bat_params{:, 'Cb'};
                m_max = bat_params{:, 'm_max'};
                S = bat_params{:, 'S'};
                omega0 = obj.omega0;
                obj.f = @(x, u) = [
                                   omega0 / Cb * (-x(2) - Gb * x(1));
                                   omega0 / Lb * (x(1) - Rb * x(2) - max((S + u(2)), m_max) * u(1));
                                   ];
                obj.g = @(x, u) = max((S + u(2), m_max)) * x(2);
            end

        end

    end

end
