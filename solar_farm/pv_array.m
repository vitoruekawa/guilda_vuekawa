% PV_array + Buck-boost converter
classdef pv_array < handle

    properties
        Rpv
        Vpv
        Vdcp_s
        S
    end

    methods

        function obj = pv_array(pv_params)
            obj.Rpv = pv_params{:, 'Rpv'};
            obj.Vpv = pv_params{:, 'Vpv'}; % Remind: Vpv = Vpv_s
            obj.Vdcp_s = pv_params{:, 'Vdcp_s'};
        end

        function nx = get_nx(obj)
            nx = 0;
        end

        function idc = calculate_idc(obj, vdc)
            idc = (obj.S * (obj.Vpv - obj.S * vdc) / obj.Rpv);
        end

        function P_s = calculate_P_s(obj)
            ipv_s = (obj.Vpv - obj.Vdcp_s) / obj.Rpv;
            P_s = ipv_s * obj.Vdcp_s;
        end

        function set_S(obj, vdc0)
            obj.S = obj.Vdcp_s / vdc0;
        end

    end

end
