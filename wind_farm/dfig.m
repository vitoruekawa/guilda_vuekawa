% Model of Doubly Fed Indcution Generator (DFIG)

% C. E. Ugalde-Loo, J. B. Ekanayake and N. Jenkins, "State-Space Modeling of
% Wind Turbine Generators for Power System Studies," in IEEE Transactions on
% Industry Applications, vol. 49, no. 1, pp. 223-232, Jan.-Feb. 2013, doi:
% 10.1109/TIA.2012.2228836.

classdef dfig < handle

    properties (Access = private)
        omega_s
        A
        B
        C
        D
        T
    end

    methods

        function obj = dfig(dfig_params)
            obj.set_dfig(dfig_params);
        end

        function nx = get_nx(obj)
            nx = 4;
        end

        % x = [idr, iqr, ids, iqs]
        % u = [vds, vqs, vdr, vqr, omega_r]
        function [dx, ir, T] = get_ir_T(obj, x, u)
            dx = obj.A(u(5)) * x + obj.B * u(1:4);
            ir = obj.C * x + obj.D * u(1:4); 
            T = obj.T(x)
        end

        function x = initialize(obj, omega0)
            obj.omega_s = omega0
            x = zeros(obj.get_nx, 1);
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

                omega_s = obj.omega_s;
                s = @(omega_r)((omega_s - omega_r) / omega_s);

                alpha1 = @(omega_r)(Xs * Xr - s(omega_r) * Xm ^ 2);
                alpha2 = @(omega_r)(Xm ^ 2 - s(omega_r) * Xs * Xr);
                beta_s = @(omega_r)(Xm * Xs * (1 - s(omega_r)));
                beta_r = @(omega_r)(Xm * Xr * (1 - s(omega_r)));
                sigma = 1 - (Xm ^ 2 / (Xr * Xs));

                obj.A = @(omega_r)[-Rs * Xr, alpha1(omega_r) * omega_s, -Rr * Xm, -beta_r(omega_r) * omega_s;
                                   -alpha1(omega_r) * omega_s, -Rs * Xr, beta_r(omega_r) * omega_s, -Rr * Xm;
                                   -Rs * Xm, beta_s(omega_r) * omega_s, - Rr * Xs, -alpha2(omega_r) * omega_s;
                                   -beta_s(omega_r) * omega_s, -Rs * Xm, alpha2(omega_r) * omega_s, -Rr * Xs] / (Xs * Xr * sigma);

                obj.B = [-Xr, 0, Xm, 0;
                         0, -Xr, 0, Xm;
                         -Xm, 0, Xs, 0;
                         0, -Xm, 0, Xs] / (Xs * Xr * sigma);

                obj.C = [0, 0, 1, 0;
                         0, 0, 0, 1];

                obj.D = zeros(2, 4);

                obj.T = @(x)(Xm * (x(2) * x(3) - x(1) * x(4)));
            end

        end

    end

end
