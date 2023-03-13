classdef rsc_controller < handle

    properties
        kappaPd
        kappaPq
        kappaId
        kappaIq
        KPdR
        KPqR
        m_max
        Vabs_m_st
        omega_r_st
        ir_st
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
            nx = 2;
        end

        % x = [Didg; Diqg; Zeta_dg; Zeta_qg];
        % u = [idG, iqG, vdc, udG, uqG, vgrid]
        function dx = get_dx(obj, ir, ir_ref)
            dx = [obj.kappaId; obj.kappaIq] .* (ir - ir_ref);
        end

        function mR = calculate_mR(obj, ir, ir_ref, chiR, uR, vdc)
            mR = 2 * ([obj.kappaPd; obj.kappaPq] .* (ir - ir_ref) + chiR + uR) / vdc;
            mR(mR > obj.m_max) = obj.m_max;
            mR(mR < -obj.m_max) = -obj.m_max;
        end

        function ir_ref = calculate_irref(obj, Vabs_m, omega_r)
            ir_ref = [obj.KPdR; obj.KPqR] .* [Vabs_m - obj.Vabs_m_st; omega_r - obj.omega_r_st] + obj.ir_st;
        end

        function set_equilibrium(obj, Vabs_m_st, omega_r_st, ir_st)
            obj.Vabs_m_st = Vabs_m_st;
            obj.omega_r_st = omega_r_st;
            obj.ir_st = ir_st;
        end

        function x = initialize(obj, omega0, vdc_st, Qr_st)
            obj.omega0 = omega0;
            obj.vdc_st = vdc_st;
            obj.Qr_st = Qr_st;
            x = zeros(obj.get_nx, 1);
        end

    end

end
