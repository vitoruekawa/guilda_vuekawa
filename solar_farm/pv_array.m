% PV_array + Buck-boost converter
classdef pv_array < handle

    properties
        P_s
        Vpv
        Rpv
        Vdcp_s
        idc
        S
    end

    methods

        function obj = pv_array(pv_params)
            obj.set_pv(pv_params);
        end

        function nx = get_nx(obj)
            nx = 0;
        end

        function idc = get_idc(obj, vdc)
            idc = (obj.S * (obj.Vpv - obj.S * vdc) / obj.Rpv);
        end

        function P_s = set_equilibrium(obj)
            ipv_s = (obj.Vpv - obj.Vdcp_s) / obj.Rpv;
            P_s = ipv_s * obj.Vdcp_s;
        end

        function set_S(obj, vdc0)
            obj.S = obj.Vdcp_s / vdc0;
        end

        function set_pv(obj, pv_params)

            if istable(pv_params)
                obj.Rpv = pv_params{:, 'Rpv'};
                obj.Vpv = pv_params{:, 'Vpv'}; % Remind: Vpv = Vpv_s
                obj.Vdcp_s = pv_params{:, 'Vdcp_s'};
            end

        end

    end

end
