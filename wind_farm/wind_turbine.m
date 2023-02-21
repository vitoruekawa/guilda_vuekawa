% Two-inertia model of wind turbine with gearbox.

classdef wind_turbine < handle

    properties (Access = private)
        A
        B
        R
        windspeed
    end

    methods

        function obj = wind_turbine(wt_params)
            obj.set_wind_turbine(wt_params);
        end

        function nx = get_nx(obj)
            nx = 3;
        end

        % x = [omega_l, omega_r, theta_T]
        % u = [Pa, T]
        function dx = get_dx(obj, x, u)
            dx = obj.A * x + obj.B * (obj.windspeed ^ 3 / x(1)) * u(1) + obj.R * u(2);
        end

        function x = initialize(obj)
            x = zeros(obj.get_nx, 1);
        end

        function set_wind_turbine(obj, wt)

            if istable(wt)
                Jl = wt{:, 'Jl'};
                Jr = wt{:, 'Jr'};
                dc = wt{:, 'dc'};
                Bl = wt{:, 'Bl'};
                Br = wt{:, 'Br'};
                Ng = wt{:, 'Ng'};
                Kc = wt{:, 'Kc'};
                obj.windspeed = wt{:, 'windspeed'};
                obj.A = [- (dc + Bl) / Jl, dc / (Jl * Ng), -Kc / Jl;
                         dc / (Jr * Ng), - (1 / Jr) * (dc / (Ng ^ 2) + Br), Kc / (Jr * Ng);
                         1, -1 / Ng, 0]; % In Sadamoto et. al: [omega0 / 2, - (omega0 / 2) / Ng, 0]; why?
                obj.B = [1 / Jl; 0; 0];
                obj.R = [0; -1 / Jr; 0];
            end

        end

    end

end
