classdef wind_farm < component
    properties (SetAccess = private)
        x_equilibrium
        gamma_pv
        pv_array
        vsc
        vsc_controller
        Pst
        Qst
    end
end