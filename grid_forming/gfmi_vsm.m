classdef gfmi_vsm < component

    properties (SetAccess = private)
        x_equilibrium
        V_equilibrium
        I_equilibrium
        vsc
        vsc_controller
        vsm
    end

    methods

        function obj = gfmi_vsm(vsc_params, controller_params, vsm_params)
            obj.vsc = vsc(vsc_params);
            obj.vsc_controller = vsc_controller(controller_params);
            obj.vsm = vsm(vsm_params);
        end

        function nx = get_nx(obj)
            nx = obj.vsc.get_nx() + obj.vsc_controller.get_nx() + obj.vsm.get_nx();
        end

        function nu = get_nu(obj)
            nu = obj.vsc.get_nu() + obj.vsc_controller.get_nu() + obj.vsm.get_nu();
        end

        function [dx, con] = get_dx_constraint(obj, t, x, V, I, u)
            % VSC state variables
            isdq = x(1:2);
            vdq = x(3:4);
            idq = x(5:6);

            % VSC controller state variables
            x_vdq = x(7:8);
            x_idq = x(9:10);

            % Reference model state variables
            delta = x(11);
            zeta = x(12);
            omega = x(13);

            % Convert from grid to converter reference
            Vdq = [sin(delta), -cos(delta);
                   cos(delta), sin(delta)] * V;

            % Active power
            % (equal both on sending and receiving ends)
            P = transpose(V) * I;

            % Calculate references from grid forming models
            vdq_hat = obj.vsm.calculate_vdq_hat(vdq, zeta, omega);

            % Calculate modulation signal
            m = obj.vsc_controller.calculate_m(vdq, idq, omega, vdq_hat, isdq, x_vdq, x_idq);

            % Calculate intermediate signals
            % Check modulation and the calculation of this signal
            % In the case of an ideal contant DC source, how should it work?
            vsdq = (1/2) * m * obj.vsc_controller.vdc_st;

            % Calculate dx
            [d_isdq, d_vdq, d_idq] = obj.vsc.get_dx(isdq, idq, omega, vdq, vsdq, Vdq);
            [d_x_vdq, d_x_idq] = obj.vsc_controller.get_dx(vdq, isdq, vdq_hat);
            [d_delta, d_zeta, d_omega] = obj.vsm.get_dx(P, vdq, omega);

            dx = [d_isdq; d_vdq; d_idq; d_x_vdq; d_x_idq; d_delta; d_zeta; d_omega];

            % Calculate constraint
            L_g = obj.vsc.L_g;
            Id = (vdq(2) - Vdq(2)) / L_g;
            Iq = (Vdq(1) - vdq(1)) / L_g;

            Ir = Id * sin(delta) + Iq * cos(delta); 
            Ii =- Id * cos(delta) + Iq * sin(delta);
            con = I - [Ir; Ii];

            % I_ = [sin(delta), cos(delta);
            %       -cos(delta), sin(delta)] * idq;
            % con = I - I_;
        end

        function set_equilibrium(obj, V, I)
            % Power flow variables
            Vangle = angle(V);
            Vabs = abs(V);
            Iangle = angle(I);
            Iabs = abs(I);

            Pow = conj(I) * V;
            P = real(Pow);
            Q = imag(Pow);

            % Get converter parameters
            C_f = obj.vsc.C_f;
            L_g = obj.vsc.L_g;

            syms delta
            eq = P * sin(Vangle - delta) == Q * cos(Vangle - delta) + Vabs * Iabs * sin(Iangle - delta);
            sol = solve(eq, delta);
            result = eval(sol);
            delta_st = norm(result(1));

            Vd = real(V) * sin(delta_st) - imag(V) * cos(delta_st);
            Vq = real(V) * cos(delta_st) + imag(V) * sin(delta_st);
            Vdq = [Vd; Vq];

            Id = real(I) * sin(delta_st) - imag(I) * cos(delta_st);
            Iq = real(I) * cos(delta_st) + imag(I) * sin(delta_st);
            Idq = [Id; Iq];

            vdq_st = L_g * [0, -1; 1, 0] * Idq + Vdq;
            idq_st = Idq;

            % Definition of steady state values
            isdq_st = idq_st + [0, -1; 1, 0] * C_f * vdq_st;

            x_vdq_st = [0; 0];
            x_idq_st = [0; 0];
            zeta_st = vdq_st(2) / 2;

            % Set reference values
            obj.vsm.set_constants(norm(vdq_st), P);

            omega_st = 1;

            obj.x_equilibrium = [isdq_st; vdq_st; idq_st; x_vdq_st; x_idq_st; delta_st; zeta_st; omega_st];
            obj.V_equilibrium = V;
            obj.I_equilibrium = I;

        end

    end

end
