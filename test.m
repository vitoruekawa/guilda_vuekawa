clear;
load_gfmi_params;
bus_id = 9;

net = network_IEEE68bus();
c = gfmi(vsc_params, dc_source_params, controller_params, ref_model_params, 'droop');
net.a_bus{bus_id}.set_component(c);

time = [0,10, 20, 200];
u_idx = 33;
u = [1.7,  1.7,  1.7,  1.7;...
       0,    0,    0,    0];
out1 = net.simulate(time, u, u_idx, 'method', 'foh');
sampling_time = out1.t;

figure
for idx = 1:9
    if idx ~= bus_id
        delta = out1.X{idx}(:,1);
    else
        delta = out1.X{bus_id}(:,9);
    end
    ddelta = delta(2:end) - delta(1:end-1);
    dt = sampling_time(2:end) - sampling_time(1:end-1);
    omega = ddelta./dt;
    t = (sampling_time(2:end)+sampling_time(1:(end-1)))/2;
    plot(t, omega, 'LineWidth', 2)
    hold on
end

Legend = cell(16,1);
for iter=1:16
  Legend{iter}=strcat('Bus', num2str(iter));
end
legend(Legend)

xlabel('Time (s)', 'FontSize', 15)
ylabel('Frequency deviation', 'FontSize', 15)
hold off