bus_id = 67;

net = network_IEEE68bus();
net.initialize();
   
time = [0,10,20,40, 60];
u_idx = bus_id;
u = [0,    0.05,   0.05,   0.05, 0;...
     0,    0,   0,   0, 0];
    
out1 = net.simulate(time, u, u_idx);

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
hold off