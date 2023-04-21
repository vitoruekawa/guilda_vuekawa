load_pv_params;
for gamma_pv = [60, 181, 306]
    net = network_IEEE68bus();
    bus = bus_PQ(gamma_pv * 2 / 100, 1, 0);
    branch = branch_pi_transformer(22, 69, 1i* 0.01, 0, 1, 0);
    
    comp = solar_farm(omega0, gamma_pv, pv_params, vsc_params, controller_params);
    bus.set_component(comp);

    net.add_bus(bus);
    net.add_branch(branch);
    net.initialize();

    time = [0, 1, 10,20,40];
    u_idx = 69;
    d1 = +0.7 * net.a_bus{69}.component.x_equilibrium(1);
    d2 = +0.7 * net.a_bus{69}.component.x_equilibrium(2);

    u = [0, 0, 0, 0, 0;...
         0, 0, 0, 0, 0;
         d1, 0, 0, 0, 0;
         d2, 0, 0, 0, 0];
   
    out1 = net.simulate(time, u, u_idx, 'method', 'foh');
    sampling_time = out1.t;
    figure
    for idx = 1:16
        omega = out1.X{idx}(:,2) * 60;
        plot(sampling_time, omega, 'LineWidth', 2)
        grid on; xlim([0 40]); ylim([-0.05 0.05]);
        set(gca, 'Xtick', [0 20 40]); set(gca, 'Ytick', [-0.04 -0.02 0 0.02 0.04])
        hold on
    end
    xlabel('Time (s)', 'FontSize', 15)
    ylabel('Frequency deviation', 'FontSize', 15)
    title(sprintf('Penetration \\gamma = %d', gamma_pv));
    hold off
    saveas(gcf, sprintf('result_%d.png',gamma_pv))
end
