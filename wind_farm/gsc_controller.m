classdef gsc_controller < handle

    properties
        Pr_st
        Qr_st
        LG
        RG
        tauG
        KPdG
        KPqG
        KIdG
        KIqG
        m_max
        gamma
        omega0
        vdc_st
    end

    methods

        function obj = gsc_controller(controller_params, gamma, omega0)
            obj.Pr_st = controller_params{:, 'Pr_st'};
            obj.Qr_st = controller_params{:, 'Qr_st'};
            obj.LG = controller_params{:, 'LG'};
            obj.RG = controller_params{:, 'RG'};
            obj.tauG = controller_params{:, 'tauG'};
            obj.KPdG = controller_params{:, 'KPdG'};
            obj.KPqG = controller_params{:, 'KPqG'};
            obj.KIdG = controller_params{:, 'KIdG'};
            obj.KIqG = controller_params{:, 'KIqG'};
            obj.m_max = controller_params{:, 'm_max'};
            obj.gamma = gamma;
            obj.omega0 = omega0;
        end

        function nx = get_nx(obj)
            nx = 4;
        end

        % x = [chi_dg; chi_qg; zeta_dg; zeta_qg];
        % u = [idG, iqG, vdc, udG, uqG, vgrid]
        function dx = get_dx(obj, iGref, iG, vdc, Qr)
            dx = [(iGref - iG) / obj.tauG;
                  [obj.KIdG; obj.KIqG] .* [vdc - obj.vdc_st; obj.gamma * (Qr - obj.Qr_st)]];
        end

        function set_equilibrium(obj, vdc_st)
            obj.vdc_st = vdc_st;
        end

        function iGref = calculate_iGref(obj, vdc, Qr, zetaG)
            iGref = [obj.KPdG; obj.KPqG] .* [(vdc - obj.vdc_st); obj.gamma * (Qr - obj.Qr_st)] + zetaG;
        end

        function mG = calculate_mG(obj, chiG, iG, iGref, Vl, vdc, uG)
            mG = (2 / vdc) * (Vl + [obj.LG / (obj.omega0 * obj.tauG), obj.LG; - obj.LG, obj.LG / (obj.omega0 * obj.tauG)] * iG -obj.RG * chiG - (obj.LG / (obj.omega0 * obj.tauG)) * iGref) + uG;
            mG(mG > obj.m_max) = obj.m_max;
            mG(mG <- obj.m_max) = -obj.m_max;
        end

    end

end
