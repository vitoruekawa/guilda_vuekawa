classdef gsc_controller < handle

    properties
        LG
        RG
        tauG
        KPdG
        KPqG
        KIdG
        KIqG
        m_max
    end

    methods

        function obj = gsc_controller(controller_params)
            obj.LG = controller_params{:, 'LG'};
            obj.RG = controller_params{:, 'RG'};
            obj.tauG = controller_params{:, 'tauG'};
            obj.KPdG = controller_params{:, 'KPdG'};
            obj.KPqG = controller_params{:, 'KPqG'};
            obj.KIdG = controller_params{:, 'KIdG'};
            obj.KIqG = controller_params{:, 'KIqG'};
            obj.m_max = controller_params{:, 'm_max'};
        end

        function nx = get_nx(obj)
            nx = 4;
        end

        % x = [chi_dg; chi_qg; zeta_dg; zeta_qg];
        % u = [idG, iqG, vdc, udG, uqG, vgrid]
        function [dx, mG] = get_dx_mG(obj, x, u)
            dx = obj.f(x, u);
            mG = obj.g(x, u);
        end

        function mG = calculate_mG(obj, x, ig, iref, vgrid, vdc, u)
            mG = 2 / vdc * (vgrid + [obj.L / (obj.omega0 * obj.tauG), obj.L; - obj.L, obj.L / (obj.omega0 * obj.tauG)] * ig -obj.R * x(1:2) - (obj.L / (obj.omega0 * obj.tauG)) * iref) + u;
            mG(mG > obj.m_max) = obj.m_max;
            mG(mG <- obj.m_max) = -obj.m_max;
        end


    end

end
