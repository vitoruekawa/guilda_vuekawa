classdef wind_turbine < handle
    properties
        nx
        A
        B
        C
        D
        sys
    end

    methods
        function obj = wind_turbine(params)
            if nargin < 1 || isempty(params)
                params = ss(0);
            end
            obj.set_wind_farm(params);
            sys = ss(obj.A, obj.B, obj.C, obj.D);
            sys.InputGroup.Ta = 1;
            sys.InputGroup.Tg = 2;
            sys.OutputGroup.omega_r = 1;
            obj.sys = sys;
        end

        function nx = get_nx(obj)
            nx = obj.nx;
        end

        function [dx, omega_r] = get_omega_r(obj, x_wt, T)
            dx = obj.A * x_wt + obj.B * T;
            omega_r = obj.C * x_wt + obj.D * T;
        end

        function x = initialize(obj)
            x = zeros(obj.get_nx, 1);
        end

        function sys = get_sys(obj)
            sys = obj.sys;
        end

        function set_wind_farm(obj, params)
            if istable(params)
                Jl = params{:, 'Jl'};
                Jr = params{:, 'Jr'};
                dc = params{:, 'dc'};
                Bl = params{:, 'Bl'};
                Br = params{:, 'Br'};
                Ng = params{:, 'Ng'};
                Kc = params{:, 'Kc'};
                obj.A = [-(dc + Bl), dc/Ng, -Kc;
                        dc/Ng, -(dc/Ng^2 + Br), Kc/Ng;
                        1, -1/Ng, 0];
                obj.B = [1, 0;
                         0, -1;
                         0, 0];
                obj.C = [0, 1, 0];
                obj.D = [0; 0; 0];
            else
                [obj.A, obj.B, obj.C, obj.D] = ssdata(params);
            end
            obj.nx = size(obj.A, 1);
        end
    end
end