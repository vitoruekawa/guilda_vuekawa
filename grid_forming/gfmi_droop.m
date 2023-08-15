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
            % Attention, vdq != Vdq
            % vdq is the terminal voltage of the VSC
            % Vdq is the bus voltage converted to converter reference.

            % VSC state variables
            vdc = x(1);
            isdq = x(2:3);
            vdq = x(4:5);
            idq = x(6:7);

            % DC source state variable
            i_tau = x(8);

            % VSC controller state variables
            x_vdq = x(9:10);
            x_idq = x(11:12);

            % Reference model state variables
            delta = x(13);
            zeta = x(14);
            
            domega = x(15);

            % Convert from grid to converter reference
            Vdq = [cos(delta), sin(delta);
                   -sin(delta), cos(delta)] * V;

            % Active power 
            P = transpose(V) * I;

            % Calculate references from grid forming models
            vdq_hat = obj.droop.calculate_vdq_hat(vdq, zeta);
            omega = obj.droop.calculate_omega(P);

            % Calculate modulation signal
            m = obj.vsc_controller.calculate_m(vdq, idq, omega, vdq_hat, isdq, x_vdq, x_idq);

            % Calculate intermediate signals
            ix = (1/2) * transpose(m) * isdq;
            vsdq = (1/2) * m * vdc;
            % idc = obj.dc_source.calculate_idc(i_tau);
            idc = i_tau;

            % Calculate dx
            [d_vdc, d_isdq, d_vdq, d_idq] = obj.vsc.get_dx(idc, vdc, ix, isdq, idq, omega, vdq, vsdq, Vdq);
            d_i_tau = obj.dc_source.get_dx(i_tau, vdc, P, ix);
            [d_x_vdq, d_x_idq] = obj.vsc_controller.get_dx(vdq, isdq, vdq_hat);
            [d_delta, d_zeta, d_domega] = obj.droop.get_dx(P, vdq, domega);

            dx = [d_vdc; d_isdq; d_vdq; d_idq; d_i_tau; d_x_vdq; d_x_idq; d_delta; d_zeta; d_domega];

            % Calculate constraint
            I_ = [cos(delta), -sin(delta);
                  sin(delta), cos(delta)] * idq;
            con = I - I_;
        end

        function set_equilibrium(obj, V, I)
            % Power flow variables 
            Vangle = angle(V);
            Vabs =  abs(V);
            Pow = conj(I) * V;
            P = real(Pow);
            Q = imag(Pow);

            % Get converter parameters
            R_dc = obj.vsc.R_dc;
            R_f = obj.vsc.R_f;
            C_f = obj.vsc.C_f;
            L_f = obj.vsc.L_f;
            L_g = obj.vsc.L_g;

            % Calculation of steady state values of angle difference and
            % converter terminal voltage
            delta_st = Vangle + atan((P * L_g) / (Vabs^2 + Q * L_g));
            v_st = P * L_g / (Vabs * sin(delta_st - Vangle));

            % Convert from bus to converter reference frame
            id_st = real(I) * cos(delta_st) + imag(I) * sin(delta_st);
            iq_st = -real(I) * sin(delta_st) + imag(I) * cos(delta_st);
            idq_st = [id_st; iq_st];

            % Definition of steady state values
            vdc_st = obj.dc_source.vdc_st;
            isdq_st = [id_st; iq_st + C_f * v_st];
            vdq_st = [v_st; 0];

            vsdq_st = vdq_st + (R_f * eye(2) + L_f * [0, -1; 1, 0]) * isdq_st;
            i_tau_st = vdc_st / R_dc + (transpose(vsdq_st) * isdq_st) / vdc_st;
            x_vdq_st = [0; 0];
            x_idq_st = [0; 0];
            zeta_st = v_st;

            % Set reference values
            obj.droop.set_constants(v_st, P);
            obj.dc_source.set_constants(P);

            domega_st = 0;

            obj.x_equilibrium = [vdc_st; isdq_st; vdq_st; idq_st; i_tau_st; x_vdq_st; x_idq_st; delta_st; zeta_st; domega_st];
            obj.V_equilibrium = V;
            obj.I_equilibrium = I;

        end

    end

end