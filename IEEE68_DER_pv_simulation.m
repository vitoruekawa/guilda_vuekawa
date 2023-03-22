load_wind_params
bus_id = 4;
for gamma_pv = [60, 90, 180, 300, 400, 500, 600]

    net = network_IEEE68bus();
    
    comp = solar_farm(omega0, gamma_pv, pv_params, vsc_params, controller_params);
    net.a_bus{bus_id}.set_component(comp);
    net.a_bus{bus_id}.set_P(gamma_pv * 2 / 100);
    net.initialize();
    
    time = [0,10,20,40];
    u_idx = bus_id;
    u = [0.01, 0, 0, 0;...
         0.01, 0, 0, 0];
    
    out1 = net.simulate(time, u, u_idx, 'method', 'foh');
    
    sampling_time = out1.t;
    figure
    for idx = 1:16
        if idx ~= bus_id
         omega = out1.X{idx}(:,2);
         plot(sampling_time, omega, 'LineWidth', 2)
         hold on
        end
    end
    xlabel('Time (s)', 'FontSize', 15)
    ylabel('Frequency deviation', 'FontSize', 15)
    title(sprintf('Penetration \\gamma = %d', gamma_pv));
    hold off
    saveas(gcf, sprintf('result_%d.png',gamma_pv))
end