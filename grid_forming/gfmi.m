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

        % Questions:
        % How to calculate the constraint?
        % Is V and I already in alpha-beta coordinates?
        function [dx, con] = get_dx_constraint(obj, t, x, V, I, u)
            vdc = x(1); i_sab = x(2:3); v_ab = x(4); % VSC state variables
            i_tau = x(5); % DC source state variable
            x_vdq = x(6:7); x_idq = x(8:9); % VSC controller state variables
            theta = x(10); zeta = x(11); % Reference model state variables

            Pow = conj(I) * V;
            P = real(Pow);

            % Conversion to dq frame
            Vdq = V * [sin(theta), -cos(theta); cos(theta), sin(theta)];
            Idq = I * [sin(theta), -cos(theta); cos(theta), sin(theta)];
            isdq = i_sab * [sin(theta), -cos(theta); cos(theta), sin(theta)];

            % Calculate references from grid forming models
            vdq_hat = obj.ref_model.calculate_vdq_hat(Vdq, zeta); % ok
            omega = obj.ref_model.calculate_omega(P); % ok

            % Calculate modulation signal
            m = obj.vsc_controller.calculate_m(Idq, Vdq, omega, vdq_hat, isdq, x_vdq, x_idq, theta); % ok

            ix = (1/2) * transpose(m) * i_sab; % ok
            v_sab = (1/2) * m * vdc; % ok
            idc = obj.dc_source.calculate_idc(i_tau); % ok

            % Calculate dx
            [d_vdc, d_i_sab, d_v_ab] = obj.vsc.get_dx(idc, vdc, ix, v_sab, i_sab, v_ab, i_ab); %ok
            d_i_tau = obj.dc_source.get_dx(i_tau, ix, vdc, P); % ok
            [d_x_vdq, d_x_idq] = obj.vsc_controller.get_dx(Vdq, isdq, vdq_hat); % ok
            [d_theta, d_zeta] = obj.ref_model.get_dx(P, Vdq); % ok

            dx = [d_vdc; d_i_sab; d_v_ab; d_i_tau; d_x_vdq; d_x_idq; d_theta; d_zeta];

            % Calculate constraint
            i_ab = obj.vsc.calculate_i_ab(i_sab, d_v_ab);
            Ir = i_ab * cos(theta) + i_ab * sin(theta);
            Ii = i_ab * sin(theta) - i_ab * cos(theta);
            con = I - [Ir; Ii];
        end

        function set_equilibrium(obj, V, I)
            theta_st = 0;
            Vdq = V * [sin(theta_st), -cos(theta_st); cos(theta_st), sin(theta_st)];
            Idq = I * [sin(theta_st), -cos(theta_st); cos(theta_st), sin(theta_st)];
        end

    end

end
