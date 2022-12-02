classdef gsc_controller < handle

    properties(Access = private)
        vdc_st
        Qr_st
        Lg
        Rg
        tauG
        KIG
        KPG
    end

    methods
        function obj = gsc_controller(controller_params)
            if nargin < 1 || isempty(controller_params)
                controller_params = ss(0);
            end
            obj.Lg = controller_params{:, 'Lg'};
            obj.Rg = controller_params{:, 'Rg'};
            obj.tauG = controller_params{:, 'tauG'};
            obj.KIG = controller_params{:, 'KIG'};
            obj.KPG = controller_params{:, 'KPG'};
            sys.InputGroup.udG = 1;
            sys.InputGroup.uqG = 2;
            sys.OutputGroup.mdG = 1;
            sys.OutputGroup.mqG = 2;
            obj.sys = sys;
        end

        function nx = get_nx(obj)
            nx = 4;
        end

        function x = initialize(obj, vdc, Qr)
            obj.vdc_st = vdc;
            obj.Qr_st = Qr;
            x = [];
        end

        function [dx, mG] = get_mG(obj, x, vdc, Qr, iG, uG, omega0)
            iGref = obj.KPG .* [vdc - obj.vdc_st;  Qr - obj.Qr_st] + x(3:4);
            dx(1:2) = (iGref - iG) / obj.tauG;
            dx(3:4) = obj.KIG .* [vdc - obj.vdc_st; Qr - obj.Qr_st]; 
            mG = (Vl + [obj.Lg / omega0 / obj.tauG obj.Lg; -obj.Lg obj.Lg / omega0 / obj.tauG] * iG - obj.Rg * x(1:2) - obj.Lg * iGref / omega0 / tauG + uG) * 2 / vdc;
        end

        function sys = get_sys(obj)
            sys = obj.sys;
        end

    end
end