classdef gfmi < component

    properties
        x_equilibrium
        dc_source
        vsc
        vsc_controller
        ref_model
        vdc_st
    end

    methods

        function obj = gfmi(vsc_params, dc_source_params, controller_params, ref_model_params)
            obj.dc_source = dc_source(dc_source_params);
            obj.vsc = vsc(vsc_params);
            obj.vsc_controller = vsc_controller(controller_params);
            obj.vdc_st = dc_source_params{:, 'vdc_st'};

            switch ref_model_params{:, 'type'}
                case 'droop'
                    obj.ref_model = droop(ref_model_params)
                case 'vsm'
                    obj.ref_model = vsm(ref_model_params)
                case 'dvoc'
                    obj.ref_model = dvoc(ref_model_params)
                otherwise
                    obj.ref_model = matching(ref_model_params)
            end

        end

        function nx = get_nx(obj)
            nx = obj.dc_source.get_nx() + obj.vsc.get_nx() + obj.vsc_controller.get_nx() + obj.ref_model.get_nx();
        end

        function nu = get_nu(obj)
            nu = 0;
        end

        function [dx, con] = get_dx_constraint(obj, t, x, V, I, u)
            vdc = x(1); isdq = x(2:3); vdq = x(4);  % VSC state variables
            i_tau = x(5);                           % DC source state variable
            x_vdq = x(6:7); x_idq = x(8:9);         % VSC controller state variables
            theta = x(10); zeta = x(11);            % Reference model state variables

            Pow = conj(I) * V;
            P = real(Pow);

            % Conversion to dq frame
            Vdq = V * [sin(theta), -cos(theta); cos(theta), sin(theta)];
            Idq = I * [sin(theta), -cos(theta); cos(theta), sin(theta)];
            isdq = i_sab * [sin(theta), -cos(theta); cos(theta), sin(theta)];

            % Calculate references from grid forming models
            vdq_hat = obj.ref_model.calculate_vdq_hat(Vdq, zeta);
            omega = obj.ref_model.calculate_omega(P);

            % Calculate modulation signal
            m = obj.vsc_controller.calculate_m(Idq, Vdq, omega, vdq_hat, isdq, x_vdq, x_idq);

            ix = (1/2) * transpose(m) * isdq;
            vsdq = (1/2) * m * vdc;
            idc = obj.dc_source.calculate_idc(i_tau);

            % Calculate dx
            [d_vdc, d_isdq, d_vdq] = obj.vsc.get_dx(idc, vdc, ix, isdq, omega, Vdq, Idq, vsdq);
            d_i_tau = obj.dc_source.get_dx(i_tau, ix, vdc, P);
            [d_x_vdq, d_x_idq] = obj.vsc_controller.get_dx(Vdq, isdq, vdq_hat);
            [d_theta, d_zeta] = obj.ref_model.get_dx(P, Vdq);

            dx = [d_vdc; d_isdq; d_vdq; d_i_tau; d_x_vdq; d_x_idq; d_theta; d_zeta];

            % Calculate constraint
            idq = obj.vsc.calculate_idq(isdq, d_vdq);
            Ir = idq(1) * cos(theta) + idq(2) * sin(theta);
            Ii = idq(1) * sin(theta) - idq(2) * cos(theta);
            con = I - [Ir; Ii];
        end

        function set_equilibrium(obj, V, I)
            Pow = conj(I)*V;
            P = real(Pow);

            theta_st = 0;
            omega_st = obj.ref_model.calculate_omega(P); % Is this correct? Is P_st = P?

            Vdq_st = V * [sin(theta_st), -cos(theta_st); cos(theta_st), sin(theta_st)];
            Idq_st = I * [sin(theta_st), -cos(theta_st); cos(theta_st), sin(theta_st)];

            isdq_st = Idq_st;
            zeta_st = Vdq_st(1);
            x_vdq_st = obj.vsc_controller.calculate_x_vdq_st(omega_st, Vdq_st);
            x_idq_st = obj.vsc_controller.calculate_x_idq_st(omega_st, Idq_st);

            Ki_i = obj.vsc_controller.Ki_i;
            Z = obj.vsc_controller.calculate_Z(omega_st);
            i_tau_st = obj.dc_source.calculate_i_tau_st(Vdq_st, Idq_st, Z, x_idq_st, Ki_i);

            obj.x_equilibrium = [obj.dc_source.vdc_st; Idq_st; Vdq_st; i_tau_st; x_vdq_st; x_idq_st; theta_st; zeta_st];

        end

    end

end
