classdef b2b_converter < handle

    properties
        f
        omega0
    end

    methods

        function obj = b2b_converter(b2b_params)
            obj.set_b2b(b2b_params);
        end

        function nx = get_nx(obj)
            nx = 3;
        end

        % x = [idg, iqg, vdc];
        % u = [idr, iqr, idc, mdR, mqR, mdG, mqG, vgrid]
        function dx = get_dx(obj, x, u)
            dx = obj.f(x, u);
        end

        function x = initialize(obj, omega0)
            obj.omega0 = omega0;
            x = zeros(obj.get_nx, 1);
        end

        function set_b2b(obj, b2b_params)

            if istable(b2b_params)
                L = b2b_params{:, 'L'};
                R = b2b_params{:, 'R'};
                Gsw = b2b_params{:, 'Gsw'};
                Cdc = b2b_params{:, 'Cdc'};
                % Is it necessary to multiply by omega0 here?
                obj.f = @(x, u)[(1 / L) * (-R * x(1) + obj.omega0 * L * x(2) + real(u(8)) - (u(6) * x(3)) / 2);
                                (1 / L) * (-obj.omega0 * L * x(1) - R * x(2) + imag(u(8)) - (u(7) * x(3)) / 2);
                                (1 / (2 * x(3) * Cdc)) * (real(u(8)) * x(1) + imag(u(8)) * x(2) + (u(4) * x(3) * u(1) / 2) + (u(5) * x(3) * u(2) / 2) - Rg * (x(1) ^ 2 + x(2) ^ 2)) - 2 * Gsw * x(3) ^ 2 + x(3)u(3)]
            end

        end

    end

end
