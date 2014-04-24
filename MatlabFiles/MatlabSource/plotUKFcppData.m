%% Read Data (Position) output from CPP UKF Implementation and Plot vs Vicon
% Data Read from a .txt file resulting from outputDataUKF.cpp
% Fredy Monterroza

ukfDataFile = fopen('../../CPPFiles/cppUKFData.txt');
cppPos = [];
while(~feof(ukfDataFile))
    ukfPos = fscanf(ukfDataFile, '%f %f %f', 3); %Column Order Filled
    cppPos = [cppPos, ukfPos]; %Columns are 3D-Position
end
fclose(ukfDataFile);

%cppUKF vs Vicon
figure;
plot3(cppPos(1,:), cppPos(2,:), cppPos(3,:), 'b');
hold on
plot3(viconPos(1,522:2015)+ 1.5, viconPos(2,522:2015)+1.2081, viconPos(3,522:2015), 'g');
grid on;