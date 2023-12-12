classdef gfmi_vsm_2axis < component

    properties (SetAccess = private)
        x_equilibrium
        V_equilibrium
        I_equilibrium
        vsc
        vsc_controller
        vsm
        avr
        pss
        governor
    end

    methods

        function obj = gfmi_vsm_2axis(vsc_params, controller_params, vsm_params)
            obj.vsc = vsc(vsc_params);
            obj.vsc_controller = vsc_controller(controller_params);
            obj.vsm = vsm_2axis(vsm_params);
            obj.avr = avr();
            obj.pss = pss();
            obj.governor = governor();
        end

        function nx = get_nx(obj)
            nx = obj.vsc.get_nx() + obj.vsc_controller.get_nx() + obj.vsm.get_nx() + obj.avr.get_nx() + obj.pss.get_nx() + obj.governor.get_nx();
        end

        function nu = get_nu(obj)
            nu = 2;
        end

        function [dx, con] = get_dx_constraint(obj, t, x, V, I, u)
            % VSC state variables
            isdq = x(1:2);
            vdq = x(3:4);

            % VSC controller state variables
            x_vdq = x(5:6);
            x_idq = x(7:8);

            % Reference model state variables
            delta = x(9);
            domega = x(10);
            Ed = x(11);
            Eq = x(12);

            nx_avr = obj.avr.get_nx();
            nx_pss = obj.pss.get_nx();
            nx_gov = obj.governor.get_nx();

            x_avr = x(12+(1:nx_avr));
            x_pss = x(12+nx_avr+(1:nx_pss));
            x_gov = x(12+nx_avr+nx_pss+(1:nx_gov));

            % Convert from grid to converter reference
            Idq = [sin(delta), -cos(delta);
                   cos(delta), sin(delta)] * I;

            omega = domega + 1;

            % Active power 
            % (equal both on sending and receiving ends)
            P = transpose(V) * I;

            % Calculate references from grid forming models
            vdq_hat = obj.vsm.calculate_vdq_hat(Idq, Ed, Eq);

            % Calculate modulation signal
            m = obj.vsc_controller.calculate_m(vdq, Idq, omega, vdq_hat, isdq, x_vdq, x_idq);

            % Calculate intermediate signals
            % Check modulation and the calculation of this signal
            % In the case of an ideal contant DC source, how should it work?
            vsdq = (1/2) * m * obj.vsc_controller.vdc_st;

            [Efd, Efq] = obj.vsm.get_Ef(Ed, Eq, vdq);

            % Calculate dx
            [d_isdq, d_vdq] = obj.vsc.get_dx(isdq, Idq, omega, vdq, vsdq);
            [d_x_vdq, d_x_idq] = obj.vsc_controller.get_dx(vdq, isdq, vdq_hat);
            [dx_pss, v] = obj.pss.get_u(x_pss, domega);
            [dx_avr, Vfd] = obj.avr.get_Vfd(x_avr, norm(V), Efd, u(1)-v);
            [dx_gov, Pmech] = obj.governor.get_P(x_gov, domega, u(2));
            [d_delta, d_domega, d_Ed, d_Eq] = obj.vsm.get_dx(P, Pmech, Vfd, domega, Efd, Efq);

            dx = [d_isdq; d_vdq; d_x_vdq; d_x_idq; d_delta; d_domega; d_Ed; d_Eq; dx_avr; dx_pss; dx_gov];

            % Calculate constraint
            % L_g = obj.vsc.L_g;
            % Id = (vdq(2) - Vdq(2)) / L_g; ...
            % Iq = (Vdq(1) - vdq(1)) / L_g;

            % Ir = Id * sin(delta) + Iq * cos(delta);
            % Ii =- Id * cos(delta) + Iq * sin(delta);
            % con = I - [Ir; Ii];

            % I_ = [sin(delta), cos(delta);
            %       -cos(delta), sin(delta)] * idq;
            % con = I - I_;

            V_ = [sin(delta), cos(delta);
                  -cos(delta), sin(delta)] * vdq;
            con = V - V_;
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

            % Calculation of steady state values of angle difference and
            % converter terminal voltage
            [delta_st, domega_st, Ed_st, Eq_st, Vfd] = obj.vsm.get_equilibrium(P, Q, Vabs, Vangle);

            x_avr_st = obj.avr.initialize(Vfd, Vabs);
            x_pss_st = obj.pss.initialize();
            x_gov_st = obj.governor.initialize(P);

            % Convert from bus to converter reference frame
            Vd = real(V) * sin(delta_st) - imag(V) * cos(delta_st);
            Vq = real(V) * cos(delta_st) + imag(V) * sin(delta_st);
            Vdq = [Vd; Vq];

            Id = real(I) * sin(delta_st) - imag(I) * cos(delta_st);
            Iq = real(I) * cos(delta_st) + imag(I) * sin(delta_st);
            Idq = [Id; Iq];

            vdq_st = Vdq;
            idq_st = Idq;

            % Definition of steady state values
            isdq_st = idq_st + [0, -1; 1, 0] * C_f * vdq_st;

            x_vdq_st = [0; 0];
            x_idq_st = [0; 0];

            obj.x_equilibrium = [isdq_st; vdq_st; x_vdq_st; x_idq_st; delta_st; domega_st; Ed_st; Eq_st; x_avr_st; x_pss_st; x_gov_st];
            obj.V_equilibrium = V;
            obj.I_equilibrium = I;

        end

        function set_avr(obj, avr)
            if isa(avr, 'avr')
                obj.avr = avr;
            else
               error(''); 
            end
        end
        
        function set_pss(obj, pss)
            if isa(pss, 'pss')
                obj.pss = pss;
            else
                error('');
            end
        end

        function set_governor(obj, governor)
            if isa(governor, 'governor')
                obj.governor = governor;
            else
                error('');
            end
        end

    end

end