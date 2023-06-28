classdef gfmi_droop < component

    properties (SetAccess = private)
        x_equilibrium
        V_equilibrium
        I_equilibrium
        dc_source
        vsc
        vsc_controller
        droop 
    end

    methods

        function obj = gfmi_droop(vsc_params, dc_source_params, controller_params, droop_params)
            obj.dc_source = dc_source(dc_source_params);
            obj.vsc = vsc(vsc_params);
            obj.vsc_controller = vsc_controller(controller_params);
            obj.droop = droop(droop_params);
        end

        function nx = get_nx(obj)
            nx = obj.dc_source.get_nx() + obj.vsc.get_nx() + obj.vsc_controller.get_nx() + obj.droop.get_nx();
        end

        function nu = get_nu(obj)
            nu = obj.dc_source.get_nu() + obj.vsc.get_nu() + obj.vsc_controller.get_nu() + obj.droop.get_nu();
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
            delta = x(9);
            zeta = x(10);

            % Active power 
            P = V' * I;

            % Convert from grid to converter reference 
            vdq = [V(1) * sin(delta) - V(2) * cos(delta);
                   V(1) * cos(delta) + V(2) * sin(delta)];
            
            % Calculate references from grid forming models
            vdq_hat = obj.droop.calculate_vdq_hat(vdq, zeta);
            domega = obj.droop.calculate_omega(P);

            % Calculate modulation signal
            m = obj.vsc_controller.calculate_m(vdq, 1 + domega, vdq_hat, isdq, x_vdq, x_idq);

            % Calculate intermediate signals
            ix = (1/2) * transpose(m) * isdq;
            vsdq = (1/2) * m * vdc;
            idc = obj.dc_source.calculate_idc(i_tau);

            % Calculate dx
            [d_vdc, d_isdq] = obj.vsc.get_dx(idc, vdc, ix, isdq, 1 + domega, vdq, vsdq);
            d_i_tau = obj.dc_source.get_dx(i_tau, vdc, P, ix);
            [d_x_vdq, d_x_idq] = obj.vsc_controller.get_dx(vdq, isdq, vdq_hat);
            [d_delta, d_zeta] = obj.droop.get_dx(P, vdq);

            dx = [d_vdc; d_isdq; d_i_tau; d_x_vdq; d_x_idq; d_delta; d_zeta];

            % Calculate constraint
            Ir = isdq(1) * sin(delta) + isdq(2) * cos(delta);
            Ii = - isdq(1) * cos(delta) + isdq(2) * sin(delta);
            con = I - [Ir; Ii];
        end

        function set_equilibrium(obj, V, I)
            Pow = conj(I) * V;
            P = real(Pow);

            obj.dc_source.set_constants(P);
            obj.droop.set_constants(V, P);

            % delta_st = atan(- real(V) / imag(V));
            delta_st = 2*atan((imag(V) + (real(V)^2 + imag(V)^2)^(1/2))/real(V));
            Idq = [real(I) * sin(delta_st) - imag(I) * cos(delta_st);
                   real(I) * cos(delta_st) + imag(I) * sin(delta_st)];

            Vdq = [real(V) * sin(delta_st) - imag(V) * cos(delta_st);
                   real(V) * cos(delta_st) + imag(V) * sin(delta_st)];

            % isdq_st = Idq;
            isdq_st = [(real(I)*real(V) + imag(I)*imag(V))/(real(V)^2 + imag(V)^2)^(1/2);
                        -(real(I)*imag(V) - imag(I)*real(V))/(real(V)^2 + imag(V)^2)^(1/2)];

            vdc_st = obj.dc_source.vdc_st;
            R_dc = obj.dc_source.R_dc;
            R_f = obj.vsc.R_f;
            i_tau_st = (R_f * (Idq' * Idq) + (Vdq' * Idq) + (vdc_st^2 / R_dc)) / vdc_st;
            
            x_idq_st = [0; 0];
            x_vdq_st = [0; 0];

            % zeta_st = Vdq(1);
            zeta_st = (real(V)^2 + imag(V)^2)^(1/2);

            % obj.x_equilibrium = [vdc_st; isdq_st; i_tau_st; x_vdq_st; x_idq_st; delta_st; zeta_st];
            % obj.V_equilibrium = V;
            % obj.I_equilibrium = I;

            t0 = 0;
            u0 = zeros(obj.get_nu, 1);
            x0 = [vdc_st; isdq_st; i_tau_st; x_vdq_st; x_idq_st; delta_st; zeta_st];

            options = optimoptions('fsolve', 'Algorithm', 'levenberg-marquardt', 'MaxFunctionEvaluations', 1e+5, 'MaxIterations', 1e+5);
            func = @(x) obj.get_dx_constraint(t0, x, [real(V); imag(V)], [real(I); imag(I)], u0);
            obj.x_equilibrium = fsolve(func, x0, options);
            obj.V_equilibrium = V;
            obj.I_equilibrium = I;

        end

    end

end