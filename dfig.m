
% Model of Doubly Fed Indcution Generator (DFIG)

% C. E. Ugalde-Loo, J. B. Ekanayake and N. Jenkins, "State-Space Modeling of
% Wind Turbine Generators for Power System Studies," in IEEE Transactions on
% Industry Applications, vol. 49, no. 1, pp. 223-232, Jan.-Feb. 2013, doi:
% 10.1109/TIA.2012.2228836.


% DFIG has to be the last one to define?
classdef dfig < handle 
    properties(Access = private)
        omega_s
        A
        B
        C
        D
        nx
        sys
    end

    methods
        function obj = dfig(omega, dfig_params)
            omega_s = omega % Stator has same frequency as grid
            if nargin < 1 || isempty(dfig_params)
                dfig_params = ss(0);
            end
            obj.set_dfig(dfig_params);
            sys = ss(obj.A, obj.B, obj.C, obj.D);
            sys.InputGroup.vds = 1
            sys.InputGroup.vqs = 2
            sys.InputGroup.vdr = 3
            sys.InputGroup.vqr = 4
            sys.OutputGroup.idr = 1
            sys.OutputGroup.iqr = 2
            obj.sys = sys;
        end

        function nx = get_nx(obj)
            nx = obj.nx;
        end

        function [dx, y] = get_y(obj, x, u)
            dx = obj.A * x + obj.B * u;
            y = obj.C * x + obj.D * u;
        end

        function x = initialize(obj, omega_r, omega_s)
            obj.omega_r = omega_r
            obj.omega_s = omega_s
            x = zeros(obj.get_nx, 1);
        end

        function sys = get_sys(obj)
            sys = obj.sys;
        end

        function set_dfig(obj, dfig_params)
            if istable(dfig_params)
                Xm = dfig_params{:, 'Xm'};
                Xls = dfig_params{:, 'Xls'};
                Xlr = dfig_params{:, 'Xlr'};
                Rs = dfig_params{:, 'Rs'};
                Rr = dfig_params{:, 'Rr'};

                Xs = Xm + Xls;
                Xr = Xm + Xlr;

                omega_r = obj.omega_r;
                omega_s = obj.omega_s;
                s = (omega_s - omega_r) / omega_s;

                alpha1 = Xs * Xr - s * Xm^2;
                alpha2 = Xm^2 - s * Xs * Xr;
                beta_s = Xm * Xs * (1 - s);
                beta_m = Xm * Xr * (1 - s);
                sigma = 1 - (Xm^2 / (Xr * Xs));
                
                obj.A = [-Rs * Xr, alpha1 * omega_s, -Rr * Xm, -beta_r * omega_s;
                        -alpha1 * omega_s, -Rs * Xr, beta_r * omega_s, -Rr * Xm;
                        -Rs * Xm, beta_s * omega_s, - Rr * Xs, -alpha2 * omega_s;
                        -beta_s * omega_s, -Rs * Xm, alpha2 * omega_s, -Rr * Xs] / (Xs * Xr * sigma);

                obj.B = [-Xr, 0, Xm, 0;
                        0, -Xr, 0, Xm;
                        -Xm, 0, Xs, 0;
                        0, -Xm, 0, Xs] / (Xs * Xr * sigma);

                obj.C = [0, 0, 1, 0;
                         0, 0, 0, 1];

                obj.D = zeros(2,4);

            else
                [obj.A, obj.B, obj.C, obj.D] = ssdata(dfig_params);
            end
            obj.nx = size(obj.A, 1);
        end
    end
end