torque_constant = [5, 10, 20, 50, 70, 100, 130, -5, -10, -20, -50, -70, -100, -130];


function generate_torque(torque_des)
    dt = 0.001; 
    num_secs = 10;
    samples = num_secs/dt;
    torque = linspace(0, torque_des, samples);
    dataTable = table(torque', 'VariableNames', {'torque'});
    
    % Step 3: Save the table to a CSV file
    csvFileName = sprintf('torque_current_consumption/torque_current_consumption_%dNm.csv', torque_des);
    
    writetable(dataTable, csvFileName);
end

for i = 1:length(torque_constant)
    generate_torque(torque_constant(i));
end