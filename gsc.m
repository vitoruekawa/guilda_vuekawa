classdef gsc < handle
    properties
        nx
        A
        B
        C
        D
        sys
    end

    properties(Access = private)
        mdG
        mqG
    end

    methods
        function obj = gsc(params)
            if nargin < 1 || isempty(params)
                params = ss(0);
            end
            obj.set_gsc(params);
            obj.mdG = params{: 'mdG'};
            obj.mqG = params{:, 'mqG'};
            sys = ss(obj.A, obj.B, obj.C, obj.D);
            sys.InputGroup.V_real = 1;
            sys.InputGroup.V_imag = 2;
            sys.OutputGroup.idG = 1;
            sys.OutputGroup.iqG = 2;
            obj.sys = sys;
        end

        function nx = get_nx(obj)
            nx = obj.nx;
        end

        function [diG, iG] = get_iG(obj, x_gsc, V, vdc)
            VG = [real(V) - obj.mdG * vdc / 2;
                  imag(V) - obj.mqG * vdc / 2];
            diG = obj.A * x_gsc + obj.B * VG;
            iG = obj.C * x_gsc + obj.D * VG;
        end

        function x = initialize(obj)
            x = zeros(obj.get_nx, 1);
        end

        function set_gsc(obj, params)
            if istable(params)
                Lg = params{:, 'Lg'};
                Rg = params{:, 'Rg'};
                obj.A = [-Rg/Lg, 1;
                        -Rg/Lg, -1];
                obj.B = eye(2);
                obj.C = [1, 1];
                obj.D = [0;
                         0];
            else
                [obj.A, obj.B, obj.C, obj.D] = ssdata(params);
            end
            obj.nx = size(obj.A, 1);
        end
    end
end