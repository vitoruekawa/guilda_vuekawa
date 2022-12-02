% Two-inertia model of wind turbine with gearbox.
 
classdef wind_turbine < handle
    properties(Access = private)
        nx
        A
        B
        C
        D
        sys
    end

    methods
        function obj = wind_turbine(wt_params)
            if nargin < 1 || isempty(wt_params)
                wt_params = ss(0);
            end
            obj.set_wind_turbine(wt_params);
            sys = ss(obj.A, obj.B, obj.C, obj.D);
            sys.InputGroup.Pa = 1;
            sys.InputGroup.T = 2;
            sys.OutputGroup.omega_r = 1;
            obj.sys = sys;
        end

        function nx = get_nx(obj)
            nx = obj.nx;
        end

        % Attention: the inputs here are Pa and T!
        function [dx, y] = get_y(obj, x, u)
            dx = obj.A * x + obj.B * u;
            y = obj.C * x + obj.D * u;
        end

        function x = initialize(obj)
            x = zeros(obj.get_nx, 1);
        end

        function sys = get_sys(obj)
            sys = obj.sys;
        end

        function set_wind_turbine(obj, wt)
            % Why is it necessary to multiply by omega_m_hat?
            if istable(wt)
                Jl = wt{:, 'Jl'};
                Jr = wt{:, 'Jr'};
                dc = wt{:, 'dc'};
                Bl = wt{:, 'Bl'};
                Br = wt{:, 'Br'};
                Ng = wt{:, 'Ng'};
                Kc = wt{:, 'Kc'};
                obj.A = [-(dc + Bl), dc/Ng, -Kc;
                        dc/Ng, -(dc/Ng^2 + Br), Kc/Ng;
                        1, -1/Ng, 0];
                obj.B = [1, 0;
                        0, -1;
                        0, 0];
                obj.C = [0, 1, 0]; 
                obj.D = zeros(1, 3);
            else
                [obj.A, obj.B, obj.C, obj.D] = ssdata(wt);
            end
            obj.nx = size(obj.A, 1);
        end
    end
end