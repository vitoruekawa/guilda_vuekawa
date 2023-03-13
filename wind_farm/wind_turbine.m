% Two-inertia model of wind turbine with gearbox.

classdef wind_turbine < handle

    properties (Access = private)
        Jl
        Jr
        dc
        Bl
        Br
        Ng
        Kc
        coeff_Pa
        Pa_st
        omega0
        A
        B
    end

    methods

        function obj = wind_turbine(wt_params, omega0)
            obj.Jl = wt_params{:, 'Jl'};
            obj.Jr = wt_params{:, 'Jr'};
            obj.dc = wt_params{:, 'dc'};
            obj.Bl = wt_params{:, 'Bl'};
            obj.Br = wt_params{:, 'Br'};
            obj.Ng = wt_params{:, 'Ng'};
            obj.Kc = wt_params{:, 'Kc'};
            obj.coeff_Pa = wt_params{:, 'coeff_Pa'};
            obj.Pa_st = wt_params{:, 'Pa_st'};
            obj.omega0 = omega0;
            obj.A = [- (obj.dc + obj.Bl), obj.dc / obj.Ng, -obj.Kc;
                   obj.dc / obj.Ng, - (obj.dc / obj.Ng ^ 2 + obj.Br), obj.Kc / obj.Ng;
                   obj.omega0 / 2, -obj.omega0 / 2 / obj.Ng, 0];
            obj.B = [obj.Jl; obj.Jr; 1];
        end

        function nx = get_nx(obj)
            nx = 3;
        end

        % x = [omega_l, omega_r, theta_T]
        % u = [Pa, T]
        function dx = get_dx(obj, x, Pa, T)
            dx = (obj.A * x(1:3) + [Pa / x(1); -T; 0]) ./ obj.B;
        end

        function Pa = calculate_Pa(obj, windspeed)
            Pa = windspeed ^ 3 * obj.coeff_Pa;
        end

        function [omega_l_st, omega_r_st, theta_T_st] = calculate_equilibrium(obj, T_st)
            a = obj.Ng ^ 2 * obj.Br + obj.Bl;
            b = obj.Ng * T_st;
            c = -obj.Pa_st;
            omega_l_st = (-b + sqrt(b ^ 2 - 4 * a * c)) / (2 * a);
            omega_r_st = obj.Ng * omega_l_st;
            theta_T_st = (obj.Ng * T_st + obj.Ng ^ 2 * obj.Br * omega_l_st) / obj.Kc;
        end

    end

end
