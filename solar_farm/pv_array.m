classdef pv_array < handle

    properties
        ipv_s
        P_s
        idcp
        Vpv
        Rpv
        params
    end

    methods

        function obj = pv_array(pv_params)
            obj.set_pv(pv_params);
            obj.params = pv_params;
        end

        function nx = get_nx(obj)
            nx = 0;
        end

        function [dx, idcp] = get_dx_idcp(obj, vdcp)
            idcp = obj.idcp(vdcp);
            dx = []
        end

        function x = initialize(obj)
            x = zeros(obj.get_nx, 1);
        end

        function set_pv(obj, pv_params)

            if istable(vsc_params)
                obj.Rpv = pv_params{:, 'Rpv'};
                obj.Vpv = pv_params{:, 'Vpv'}; % Remind: Vpv = Vpv_s
                Vdcp_s = pv_params{:, 'Vdcp_s'};

                obj.idcp = @(vdcp)(Vpv - vdcp) / Rpv;
                obj.ipv_s = (Vpv - Vdcp_s) / Rpv;
                obj.P_s = obj.ipv_s * Vdcp_s;
            end

        end

    end

end
