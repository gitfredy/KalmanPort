%Script to parse the data in quadData.mat and write into a .txt file.
%Will be used in C++ EKF/UKF implementation.
%Possible integration into ROS with openGL/other to plot results.
%Fredy Monterroza

%% Load quadData.mat: Should be in KalmanPort directory.
%quadData contains: quadLog (vicon data), quadTime (time at which vicon
%data was recorded), and sensorLog (IMU/Camera data).
addpath('MatlabFiles');
addpath('Data');
load('/Data/quadData.mat');

%% Parse qdLog (Vicon Data)
viconSamples = numel(qdLog);
viconEuler = zeros(3,viconSamples); %Roll Pitch Yaw
viconPos = zeros(3, viconSamples);
viconTime = zeros(1,viconSamples);
bodyLinVel = zeros(3,viconSamples);
bodyAngVel = zeros(3,viconSamples);


for viconSamp = 1:viconSamples
    viconEuler(:,viconSamp) = qdLog{viconSamp}{1}.euler;
    viconPos(:,viconSamp) = qdLog{viconSamp}{1}.pos;
    viconTime(viconSamp) = qdTimeLog{viconSamp};
    bodyLinVel(:,viconSamp) = qdLog{viconSamp}{1}.vel_body;
    bodyAngVel(:,viconSamp) = qdLog{viconSamp}{1}.omega;
end

%% Parse Quadrotor Sensor (IMU) Data: Note, since this is wirelessly, transmitted packets may be lost.
sensorSamples = numel(sensorLog);
sensorIMU = zeros(6, sensorSamples);
sensorTS = zeros(1,sensorSamples);
%All this data is with respect to body, different sampling than images with
%tags but the same as imageIndices.
for sensorSamp = 1:sensorSamples
    accel = sensorLog{sensorSamp}.accImu;
    angVel = sensorLog{sensorSamp}.omegaImu;
    if ~isempty(accel) %assume angvel, t also not empty if accel isn't...
        sensorIMU(:,sensorSamp) = [accel; angVel];
        sensorTS(sensorSamp) = sensorLog{sensorSamp}.t; 
    end

end

%Valid Timestamps will be >0, they are initialized/declared as 0 above.
actualIMUinds = find(sensorTS>0);
actualIMUDataReceived = sensorIMU(:,actualIMUinds); %1293 Samples

%% Write the data of qdLog into text files.

fid = fopen('viconEuler.txt', 'w');
fprintf(fid, '%.15f %.15f %.15f\n', viconEuler);
fclose(fid);

fid = fopen('viconPos.txt', 'w');
fprintf(fid, '%.15f %.15f %.15f\n', viconPos);
fclose(fid);

fid = fopen('viconTime.txt', 'w');
fprintf(fid, '%.15f\n', viconTime);
fclose(fid);

fid = fopen('bodyLinVel.txt', 'w');
fprintf(fid, '%.15f %.15f %.15f\n', bodyLinVel);
fclose(fid);

fid = fopen('bodyAngVel.txt', 'w');
fprintf(fid, '%.15f %.15f %.15f\n', bodyAngVel);
fclose(fid);


%% Write the data of sensorLog into text files.

sensorIMUaccel = sensorIMU(1:3,:);
sensorIMUangVel = sensorIMU(4:6,:);

%Format Saved in .txt is : xaccel, yaccel, zaccel, [newline]...
fid = fopen('sensorIMUaccel.txt', 'w');
fprintf(fid, '%.15f %.15f %.15f\n', sensorIMUaccel);
fclose(fid);

fid = fopen('sensorIMUangVel.txt', 'w');
fprintf(fid, '%.15f %.15f %.15f\n', sensorIMUangVel);
fclose(fid);


%% Write nPointPose algorithm measurement to text file.

%Position and Orientation
fid = fopen('visionRobot.txt', 'w');
fprintf(fid, '%.15f %.15f %.15f\n', visionRobot);
fclose(fid);


%% Write Kalman Filter Data (State Vector for Trajectory and Norm of Covariance to text files.

%These are to be put into a file, they are computed as the KF runs because
%that is when the robotToWorld transformation is computed.
%bodyLin/AngVel to World Frame

fid = fopen('worldLinVel.txt', 'w'); 
fprintf(fid, '%.15f %.15f %.15f\n', worldLinVel);
fclose(fid);

fid = fopen('worldAngVel.txt', 'w');
fprintf(fid, '%.15f %.15f %.15f\n', worldAngVel);
fclose(fid);

%Note: Run EKF/UKF for respective state vectors, normP's
%State Vector from 14 Dim EKF 
stateVectorPos = stateVector(1:3,:);
fid = fopen('stateVectorPos.txt', 'w'); 
fprintf(fid, '%.15f %.15f %.15f\n', stateVectorPos);
fclose(fid);

stateVectorVel = stateVector(4:6,:);
fid = fopen('stateVectorVel.txt', 'w'); 
fprintf(fid, '%.15f %.15f %.15f\n', stateVectorVel);
fclose(fid);

stateVectorOrient = stateVector(7:9,:);
fid = fopen('stateVectorOrient.txt', 'w'); 
fprintf(fid, '%.15f %.15f %.15f\n', stateVectorOrient);
fclose(fid);

fid = fopen('normP.txt', 'w'); 
fprintf(fid, '%.15f\n', normP);
fclose(fid);

