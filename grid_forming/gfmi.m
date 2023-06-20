classdef gfmi < component

    properties (SetAccess = private)
        x_equilibrium
        V_equilibrium
        I_equilibrium
        dc_source
        vsc
        vsc_controller
        ref_model
    end

    methods

        function obj = gfmi(vsc_params, dc_source_params, controller_params, ref_model_params, type)
            obj.dc_source = dc_source(dc_source_params);
            obj.vsc = vsc(vsc_params);
            obj.vsc_controller = vsc_controller(controller_params);

            switch type
                case 'droop'
                    obj.ref_model = droop(ref_model_params);
                case 'vsm'
                    obj.ref_model = vsm(ref_model_params);
                case 'dvoc'
                    obj.ref_model = dvoc(ref_model_params);
                otherwise
                    obj.ref_model = matching(ref_model_params);
            end

        end

        function nx = get_nx(obj)
            nx = obj.dc_source.get_nx() + obj.vsc.get_nx() + obj.vsc_controller.get_nx() + obj.ref_model.get_nx();
        end

        function nu = get_nu(obj)
            nu = obj.dc_source.get_nu() + obj.vsc.get_nu() + obj.vsc_controller.get_nu() + obj.ref_model.get_nu();
        end

        % Implementation problems:
        % load_gfmi_params might be wrong, let's check
        function [dx, con] = get_dx_constraint(obj, t, x, V, I, u)
            % VSC state variables
            vdc = x(1);
            isdq = x(2:3);

            % DC source state variable
            i_tau = x(4);

            % VSC controller state variables
            x_vdq = x(5:6);
            x_idq = x(7:8);

            % Reference model state variables
            delta = x(9);
            zeta = x(10);

            % Active power 
            P = V' * I;

            % Convert from grid to converter reference 
            vdq = [V(1) * sin(delta) - V(2) * cos(delta);
                   V(1) * cos(delta) + V(2) * sin(delta)];

            % Calculate references from grid forming models
            vdq_hat = obj.ref_model.calculate_vdq_hat(vdq, zeta); % ok 
            omega = obj.ref_model.calculate_omega(P); % ok

            % Calculate modulation signal
            m = obj.vsc_controller.calculate_m(vdq, omega, vdq_hat, isdq, x_vdq, x_idq);

            ix = (1/2) * transpose(m) * isdq;
            vsdq = (1/2) * m * vdc;
            idc = obj.dc_source.calculate_idc(i_tau);

            % Calculate dx
            [d_vdc, d_isdq] = obj.vsc.get_dx(idc, vdc, ix, isdq, omega, vdq, vsdq);
            d_i_tau = obj.dc_source.get_dx(i_tau, vdc, P, ix);
            [d_x_vdq, d_x_idq] = obj.vsc_controller.get_dx(vdq, isdq, vdq_hat);
            [d_delta, d_zeta] = obj.ref_model.get_dx(P, vdq);

            dx = [d_vdc; d_isdq; d_i_tau; d_x_vdq; d_x_idq; d_delta; d_zeta];

            % Calculate constraint
            Ir = isdq(1) * sin(delta) + isdq(2) * cos(delta);
            Ii = - isdq(1) * cos(delta) + isdq(2) * sin(delta);
            con = I - [Ir; Ii];
        end

        function set_equilibrium(obj, V, I)
            Pow = conj(I) * V;
            P = real(Pow);
            Q = imag(Pow);
            Vabs = abs(V);

            obj.dc_source.set_constants(P);
            obj.ref_model.set_constants(V, P);

            vdc_st = obj.dc_source.vdc_st;

            delta_st = angle(V) + atan(P / (Q + Vabs^2));
            Idq = [real(I) * sin(delta_st) - imag(I) * cos(delta_st);
                   real(I) * cos(delta_st) + imag(I) * sin(delta_st)];

            Vdq = [real(V) * sin(delta_st) - imag(V) * cos(delta_st);
                   real(V) * cos(delta_st) + imag(V) * sin(delta_st)];

            isdq_st = Idq;

            i_tau_st = (obj.vsc.R_f * (Idq' * Idq) + (Vdq' * Idq) + (vdc_st^2 / obj.dc_source.R_dc)) / vdc_st;
            
            x_idq_st = [0; 0];
            x_vdq_st = obj.vsc_controller.calculate_x_vdq_st(Vdq);

            zeta_st = Vdq(1);

            t0 = 0;
            u0 = zeros(obj.get_nu, 1);
            x0 = [vdc_st; isdq_st; i_tau_st; x_vdq_st; x_idq_st; delta_st; zeta_st];

            options = optimoptions('fsolve', 'MaxFunctionEvaluations', 1e+5, 'MaxIterations', 1e+5);
            func = @(x) obj.get_dx_constraint(t0, x, [real(V); imag(V)], [real(I); imag(I)], u0);
            obj.x_equilibrium = fsolve(func, x0, options);
            obj.V_equilibrium = V;
            obj.I_equilibrium = I;

        end

    end

end