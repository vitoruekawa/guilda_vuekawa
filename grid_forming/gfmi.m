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
            nu = 0;
        end

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
            theta = x(9);
            zeta = x(10);

            % Apparent Power S = conj(I) * V = V(1) * I(1) - V(2) * I(2)
            Pow = V(1) * I(1) - V(2) * I(2);
            P = Pow(1);

            % Conversion to dq frame
            Vdq = [V(1) * sin(theta) - V(2) * cos(theta); V(1) * cos(theta) + V(2) * sin(theta)];
            Idq = [I(1) * sin(theta) - I(2) * cos(theta); I(1) * cos(theta) + I(2) * sin(theta)];

            % Calculate references from grid forming models
            vdq_hat = obj.ref_model.calculate_vdq_hat(Vdq, zeta);
            omega = obj.ref_model.calculate_omega(P);

            % Calculate modulation signal
            m = obj.vsc_controller.calculate_m(Idq, Vdq, omega, vdq_hat, isdq, x_vdq, x_idq);

            ix = (1/2) * transpose(m) * I;
            vsdq = (1/2) * m * vdc;
            idc = obj.dc_source.calculate_idc(i_tau);

            % Calculate dx
            [d_vdc, d_isdq] = obj.vsc.get_dx(idc, vdc, ix, isdq, omega, Vdq, vsdq);
            d_i_tau = obj.dc_source.get_dx(i_tau, ix, vdc, P);
            [d_x_vdq, d_x_idq] = obj.vsc_controller.get_dx(Vdq, isdq, vdq_hat);
            [d_theta, d_zeta] = obj.ref_model.get_dx(P, Vdq);

            dx = [d_vdc; d_isdq; d_i_tau; d_x_vdq; d_x_idq; d_theta; d_zeta];

            % Calculate constraint
            Ir = isdq(2) * cos(theta) + isdq(1) * sin(theta);
            Ii = isdq(2) * sin(theta) - isdq(1) * cos(theta);
            con = I - [Ir; Ii];
        end

        function set_equilibrium(obj, V, I)
            Pow = conj(I)*V;
            P_st = real(Pow);

            % Transform voltage and current to dq-frame
            theta_st = 2; % How to calculate it?
            Vdq_st = [real(V) * sin(theta_st) - imag(V) * cos(theta_st); real(V) * cos(theta_st) + imag(V) * sin(theta_st)];
            Idq_st = [real(I) * sin(theta_st) - imag(I) * cos(theta_st);  real(I) * cos(theta_st) + imag(I) * sin(theta_st)];

            % Define set points
            obj.ref_model.set_equilibrium(V, P_st);
            obj.dc_source.set_equilibrium(P_st);

            % Define equilibrium points
            isdq_st = Idq_st;
            zeta_st = Vdq_st(1);

            vdq_hat = obj.ref_model.calculate_vdq_hat(Vdq_st, zeta_st);
            omega_st = obj.ref_model.calculate_omega(P_st); % Is this correct? Is P_st = P?

            [x_vdq_st, x_idq_st] = obj.vsc_controller.calculate_equilibrium(omega_st, Vdq_st, Idq_st);

            m_st = obj.vsc_controller.calculate_m(Idq_st, Vdq_st, omega_st, vdq_hat, isdq_st, x_vdq_st, x_idq_st);
            ix_st = (1/2) * transpose(m_st) * isdq_st;
            i_tau_st = obj.dc_source.calculate_i_tau_st(ix_st);

            obj.V_equilibrium = V;
            obj.I_equilibrium = I;
            obj.x_equilibrium = [obj.dc_source.vdc_st; Idq_st; i_tau_st; x_vdq_st; x_idq_st; 376.9911; Vdq_st(1)];

        end

    end

end
