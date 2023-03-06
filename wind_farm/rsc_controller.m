classdef rsc_controller < handle

    properties
        kappaPd
        kappaPq
        kappaId
        kappaIq
        KPdR
        KPqR
        m_max
    end

    methods

        function obj = rsc_controller(controller_params)
            obj.kappaPd = controller_params{:, 'kappaPd'};
            obj.kappaPq = controller_params{:, 'kappaPq'};
            obj.kappaId = controller_params{:, 'kappaId'};
            obj.kappaIq = controller_params{:, 'kappaIq'};
            obj.KPdR = controller_params{:, 'KPdR'}; 
            obj.KPqR = controller_params{:, 'KPqR'}; 
            obj.m_max = controller_params{:, 'm_max'};
        end

        function nx = get_nx(obj)
            nx = 4;
        end

        % x = [Didg; Diqg; Zeta_dg; Zeta_qg];
        % u = [idG, iqG, vdc, udG, uqG, vgrid]
        function [dx, mG] = get_dx_mG(obj, x, u)
            dx = obj.f(x, u);
            mG = obj.g(x, u);
        end

        function x = initialize(obj, omega0, vdc_st, Qr_st)
            obj.omega0 = omega0;
            obj.vdc_st = vdc_st;
            obj.Qr_st = Qr_st;
            x = zeros(obj.get_nx, 1);
        end

    end

end
