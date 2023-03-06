% Model of Doubly Fed Indcution Generator (DFIG)

% C. E. Ugalde-Loo, J. B. Ekanayake and N. Jenkins, "State-Space Modeling of
% Wind Turbine Generators for Power System Studies," in IEEE Transactions on
% Industry Applications, vol. 49, no. 1, pp. 223-232, Jan.-Feb. 2013, doi:
% 10.1109/TIA.2012.2228836.

classdef dfig < handle

    properties (Access = private)
        Pr_st
        Qr_st
        Rs
        Rr
        Xs
        Xm
        Xr
        beta
        gamma
    end

    methods

        function obj = dfig(dfig_params, gamma)
            obj.Pr_st = dfig_params{:, 'Pr_st'};
            obj.Qr_st = dfig_params{:, 'Qr_st'};

            obj.Rs = dfig_params{:, 'Rs'};
            obj.Rr = dfig_params{:, 'Rr'};
            obj.Xs = dfig_params{:, 'Xs'};
            obj.Xm = dfig_params{:, 'Xm'};
            obj.Xr = dfig_params{:, 'Xr'};
            obj.beta = obj.Xs * obj.Xr - obj.Xm ^ 2;

            obj.gamma = gamma;
        end

        function nx = get_nx(obj)
            nx = 4;
        end

        % x = [idr, iqr, ids, iqs]
        % u = [vds, vqs, vdr, vqr, omega_r]
        function dx = get_dx(obj, x, omega_r, Vm, vr)
            Ai = [
                  -obj.Rr * obj.Xs, obj.beta - omega_r * obj.Xs * obj.Xr, obj.Rs * obj.Xm, -omega_r * obj.Xs * obj.Xm;
                  -obj.beta + omega_r * obj.Xs * obj.Xr, -obj.Rr * obj.Xs, omega_r * obj.Xs * obj.Xm, obj.Rs * obj.Xm; ...
                      obj.Rr * obj.Xm, omega_r * obj.Xr * obj.Xm, -obj.Rs * obj.Xr, obj.beta + omega_r * obj.Xm ^ 2; ...
                      -omega_r * obj.Xr * obj.Xm, obj.Rr * obj.Xm, -obj.beta - omega_r * obj.Xm ^ 2, -obj.Rs * obj.Xr] / obj.beta;
            Bi = [-obj.Xs, 0;
                  0, -obj.Xs;
                  obj.Xm, 0;
                  0, obj.Xm] / obj.beta;
            Gi = [obj.Xm, 0, -obj.Xr, 0; 0 obj.Xm, 0, -obj.Xr]' / obj.beta;

            dx = Ai * x + Gi * Vm + Bi * vr;
        end

        function T = calculate_T(obj, idr, iqr, ids, iqs)
            T = obj.Xm * (ids * iqr - iqs * idr);
        end

        function [idr_st, iqr_st, ids_st, iqs_st] = calculate_equilibrium(obj, V, Vm, I)
            PQ_s_st = V * conj(I) / obj.gamma + (obj.Pr_st + 1j * obj.Qr_st);
            is_st = conj(PQ_s_st / Vm);
            ir_st = 1j * (Vm + (obj.Rs + 1j * obj.Xs) * is_st) / obj.Xm;

            idr_st = real(ir_st);
            iqr_st = imag(ir_st);
            ids_st = real(is_st);
            iqs_st = imag(is_st);
        end

        function vr_st = calculate_vr_st(obj, ir_st, is_st, omega_r)
            vr_st =- (obj.Rr + 1j * obj.Xr * (1 - omega_r)) * ir_st - 1j * obj.Xm * (1 - omega_r) * is_st;
        end

    end

end

% function set_dfig(obj, dfig_params)

%     if istable(dfig_params)
%         Xm = dfig_params{:, 'Xm'};
%         Xls = dfig_params{:, 'Xls'};
%         Xlr = dfig_params{:, 'Xlr'};
%         Rs = dfig_params{:, 'Rs'};
%         Rr = dfig_params{:, 'Rr'};

%         Xs = Xm + Xls;
%         Xr = Xm + Xlr;

%         omega_s = obj.omega_s;
%         s = @(omega_r)((omega_s - omega_r) / omega_s);

%         alpha1 = @(omega_r)(Xs * Xr - s(omega_r) * Xm ^ 2);
%         alpha2 = @(omega_r)(Xm ^ 2 - s(omega_r) * Xs * Xr);
%         beta_s = @(omega_r)(Xm * Xs * (1 - s(omega_r)));
%         beta_r = @(omega_r)(Xm * Xr * (1 - s(omega_r)));
%         sigma = 1 - (Xm ^ 2 / (Xr * Xs));

%         obj.A = @(omega_r)[-Rs * Xr, alpha1(omega_r) * omega_s, -Rr * Xm, -beta_r(omega_r) * omega_s;
%                            -alpha1(omega_r) * omega_s, -Rs * Xr, beta_r(omega_r) * omega_s, -Rr * Xm;
%                            -Rs * Xm, beta_s(omega_r) * omega_s, - Rr * Xs, -alpha2(omega_r) * omega_s;
%                            -beta_s(omega_r) * omega_s, -Rs * Xm, alpha2(omega_r) * omega_s, -Rr * Xs] / (Xs * Xr * sigma);

%         obj.B = [-Xr, 0, Xm, 0;
%                  0, -Xr, 0, Xm;
%                  -Xm, 0, Xs, 0;
%                  0, -Xm, 0, Xs] / (Xs * Xr * sigma);

%         obj.C = [0, 0, 1, 0;
%                  0, 0, 0, 1];

%         obj.D = zeros(2, 4);

%         obj.T = @(x)(Xm * (x(2) * x(3) - x(1) * x(4)));
%     end

% end
