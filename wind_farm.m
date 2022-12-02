classdef wind_turbine < component

    properties
        Jl
        Jr
        dc
        Bl
        Ng
        Kc
        Xm
        Xls
        Xm
        Xlr
        Xs
        Xr
        Xm
        beta
        Lg
        Rg
        mdG %vector
        gamma_w
        KIG %vector
        KPG %vector
        tauG
        mR %vector
        KPR %Vector
        kappa_IR % Vector
        cdc
        Rg
        Gsw
        S
        Cb
        Gb
        Lb
        Rb
        x_equilibrium
    end

    methods

        function obj = wind_turbine(params)
            obj.Jl= obj.params{:, 'Jl'};
            obj.Jr= obj.params{:, 'Jr'};
            obj.dc= obj.params{:, 'dc'};
            obj.Bl= obj.params{:, 'Bl'};
            obj.Ng= obj.params{:, 'Ng'};
            obj.Kc= obj.params{:, 'Kc'};
            obj.Xm= obj.params{:, 'Xm'};
            obj.Xls= obj.params{:, 'Xls'};
            obj.Xm= obj.params{:, 'Xm'};
            obj.Xlr= obj.params{:, 'Xlr'};
            obj.Xs= obj.params{:, 'Xs'};
            obj.Xr= obj.params{:, 'Xr'};
            obj.Xm= obj.params{:, 'Xm'};
            obj.beta= obj.params{:, 'beta'};
            obj.Lg= obj.params{:, 'Lg'};
            obj.Rg= obj.params{:, 'Rg'};
            obj.mdG = obj.params{:, 'mdG'};
            obj.gamma_w= obj.params{:, 'gamma_w'};
            obj.KIG = obj.params{:, 'KIG'};
            obj.KPG = obj.params{:, 'KPG'};
            obj.tauG= obj.params{:, 'tauG'};
            obj.mR = obj.params{:, 'mR'};
            obj.KPR = obj.params{:, 'KPR'};
            obj.kappa_IR = obj.params{:, 'kappa_IR'};
            obj.cdc= obj.params{:, 'cdc'};
            obj.Rg= obj.params{:, 'Rg'};
            obj.Gsw= obj.params{:, 'Gsw'};
            obj.S= obj.params{:, 'S'};
            obj.Cb= obj.params{:, 'Cb'};
            obj.Gb= obj.params{:, 'Gb'};
            obj.Lb= obj.params{:, 'Lb'};
            obj.Rb= obj.params{:, 'Rb'};
        end

        function set_equilibrium(Veq, Ieq)
        end

        function get_nu(var)
        end

        function [dx, constraint] = get_dx_constraint(t, x, V, I, u)
        end

    end

end
