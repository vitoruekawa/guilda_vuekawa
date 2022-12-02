% Model of Voltage Source Converter (gsc)
% Pena, Ruben, J. C. Clare, and G. M. Asher. "Doubly fed induction generator
% using back-to-back PWM converters and its application to variable-speed
% wind-energy generation." IEE Proceedings-Electric power applications 143.3
% (1996): 231-241.

classdef gsc < handle

    properties(Access = private)
        nx
        slip
        A
        B
        C
        D
        sys
    end

    methods
        function obj = gsc(gsc_params)
            if nargin < 1 || isempty(gsc_params)
                gsc_params = ss(0);
            end
            obj.set_gsc(gsc_params);
            sys = ss(obj.A, obj.B, obj.C, obj.D);
            sys.InputGroup.vdc = 1;
            sys.OutputGroup.idG = 1;
            sys.OutputGroup.iqG = 2;
            obj.sys = sys;
        end

        function nx = get_nx(obj)
            nx = obj.nx;
        end

        % here u = [Re(V) - (mdG * vdc)/2; Im(V) - (mqG * vdc)/2]
        function [dx, y] = get_y(obj, x, u)
            dx = obj.A * x + obj.B * u;
            y = obj.C * x + obj.D * u;
        end

        function x = initialize(obj, V)
            x = zeros(obj.get_nx, 1);
        end

        function sys = get_sys(obj)
            sys = obj.sys;
        end

        function set_gsc(obj, gsc_params)
            if istable(gsc_params)
                Rg = gsc_params{:, 'Rg'};
                Lg = gsc_params{:, 'Lg'};
                obj.A = [-Rg/Lg, 1;
                        -1, -Rg/Lg];
                obj.B = [-1/2;
                        -1/2];
                obj.C = [1, 1];
                obj.D = 0;
            else
                [obj.A, obj.B, obj.C, obj.D] = ssdata(pss);
            end
            obj.nx = size(obj.A, 1);
        end
    end
end