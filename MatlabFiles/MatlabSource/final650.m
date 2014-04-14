
%% Fredy Monterroza
%ESE 650 Final Project: Quadrotor control using state estimation from
%an EKF/UKF using homography from 2D images and IMU sensor data. Possible 
%augmentation includes reinforcement learning of gains or a PD-type
%iterative learning control. Possible use, surveillance, deployed quadrotor
%can optimally adapt to their environment due to the fact that the patrol
%pattern is repeatable. I think the effect of having patrolling quadrotors
%is the same or similar to that produced by people leaving their TV on even
%though they aren't home. This is supposed to deter would-be intruders from
%breaking into a residence/establishment. I think this is more true now,
%when the populace still isn't too familiar with autonomous drones, most
%people would assume there is someone monitoring/controlling these drones
%directly. Just a thought.
%Note, since system is linearizable near hover, an EKF may be used.
%Here, the Extended Kalman Filter is used to form a state estimate.
%Uses nPointPose.m and Matlab's Jacobian computation.


%NOTE: 14 State EKF tracking 9 dim state vector + biases. final650.m

%% Global Vars
global tagWorldCoords;
global tagIDs;
global cam2RobotHomoStable;
global kMat;

%% Load Data: Should be in KalmanPort Directory
addpath(genpath('/home/fredy/Desktop/KalmanPort'));

load('/Data/quadData.mat'); %Quadrotor Data Set: Vicon + IMU/Images
load('MatlabEquations/14GEqn');
load('MatlabEquations/14GFunc');
load('MatlabEquations/14LFunc');


%Tag ID's (9x12 matrix)
tagIDs = [57401312644, 58383764297, 59366215950, 61331119256, 63296022562, 65260925868,  1453707397,  4401062356,  9313320621, 10295772274, 14225578886, 17172933845; ...
 18155385498, 19137837151, 21102740457, 22085192110, 24050095416, 27979902028, 28962353681, 33874611946, 34857063599, 35839515252, 37804418558, 42716676823; ...
 43699128476, 46646483435, 47628935088, 49593838394, 56470999965, 57453451618, 61383258230, 12312814554, 18207524472, 23119782737, 27049589349, 28032041002; ...
 29014492655, 29996944308, 37856557532, 42768815797, 46698622409, 50628429021, 56523138939, 58488042245, 61435397204, 67330107122,  6470243610, 10400050222; ...
 12364953528, 17277211793, 32013986588, 35943793200, 37908696506, 42820954771, 52645471301, 55592826260,  5539930931, 13399544155, 29118770603, 31083673909; ...
 36978383827, 37960835480, 38943287133, 44837997051, 46802900357, 49750255316, 49802394290, 57662007514,  5644208879, 20380983674, 21363435327, 27258145245; ...
 44942274999, 63608856406,  7661251159, 14538412730, 22398025954, 25345380913, 32222542484, 62678543727, 24415068234, 31292229805, 63713134354, 25449658861; ...
 33309272085, 51975853492, 62782821675, 15677281305, 45150830895, 20641678544, 21624130197, 36360904992, 45202969869, 62887099623,  6939494376,  9886849335; ...
 29535882395, 41325302231, 13868794921, 19763504839, 21728408145, 27623118063, 40447128526, 41429580179, 46341838444, 52236548362, 37551912541, 59165848907   ];

% Camera Matrix (zero-indexed): Units = Pixels "Calibration Matrix"
kMat = [312.554757, 0.00000000, 170.720170; ...
 0.00000000, 313.225581, 134.038406; ...
 0.00000000, 0.00000000, 1.00000000   ];


%Robot To Camera Transformation:
rotZ = [cos(-pi/4), -sin(-pi/4), 0; ...
    sin(-pi/4), cos(-pi/4), 0; ...
    0, 0, 1];
rotX = [1, 0, 0; ...
    0, cos(pi), -sin(pi); ...
    0, sin(pi), cos(pi)];
rotRobot = rotX*rotZ;
transRobot = [-.04; 0; -.03];
c2rHomo = [rotRobot; zeros(1, 3)];
tRHomo = [transRobot; 1];
cam2RobotHomoStable = [c2rHomo, tRHomo];
            
            
kMatInv = inv(kMat);
tagIDs = tagIDs'; %%12x9

%% Pre-Compute April Tag Corner Locations (12x9, 108 tags)
accumX = 0;
whiteSpaceX = 0;

tagWorldCoords = {}; %Initialize April Tag Mat
for j = 1:12
    whiteSpaceY = 0; %Reset White Space Y
    accumY = 0;
    for k = 1:9
        %disp(k);
        %disp(j);
        tagIndex = sub2ind(size(tagIDs), j, k);
        %tagIndex = find(
        %tagIndex = sub2ind(size(tagIDs), k, j);
        
        tagWorldCoords{tagIndex}.tlx = accumX + whiteSpaceX;
        tagWorldCoords{tagIndex}.trx = accumX + whiteSpaceX;
        
        tagWorldCoords{tagIndex}.blx = accumX + whiteSpaceX + .152;
        tagWorldCoords{tagIndex}.brx = accumX + whiteSpaceX + .152;
        
        tagWorldCoords{tagIndex}.tly = accumY + whiteSpaceY;
        tagWorldCoords{tagIndex}.bly = accumY + whiteSpaceY;
        
        tagWorldCoords{tagIndex}.try = accumY + whiteSpaceY + .152;
        tagWorldCoords{tagIndex}.bry = accumY + whiteSpaceY + .152;
        
        if k == 3 || k == 6 %This increment in whitespace happens after tag at col 3, col 6 has been calculated
            whiteSpaceY = whiteSpaceY + .178;
        else
            whiteSpaceY = whiteSpaceY + .152;
        end
        accumY = accumY + .152;
    end
    accumX = accumX + .152;
    whiteSpaceX = whiteSpaceX + .152;
end



%% Initialize Parameters

numSamples = numel(sensorLog);


%% Show Video of Flight

%Not only do I not always have an image, but I don't always have detected
%tags within that image that may have been received. (Contrast issues?)


imageIndices = []; %Holds Sample Indices which actually provide an image.
imageWithTagInd = [];
for imSamp = 1:numSamples
    image = sensorLog{imSamp}.img;
    if ~isempty(image);
        %Only a sample with an image will have a tag, so save that index.
        if ~isempty(sensorLog{imSamp}.id)
            imageWithTagInd = [imageWithTagInd; imSamp];
        end
        imageIndices = [imageIndices; imSamp];
        %disp(imSamp); %About sample 300 when its on the box.
        imshow(image);
        pause(.03); % I think its 30Hz image aquisition.
    end
end


%% Show Video of Images with Tags

for imWithTags = 1:numel(imageWithTagInd)
    figure(1);
    imshow(sensorLog{imageWithTagInd(imWithTags)}.img);
    pause(.03);
end


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
    if ~isempty(accel) %assume angvel, t also not epmpty if accel isn't...
        sensorIMU(:,sensorSamp) = [accel; angVel];
        sensorTS(sensorSamp) = sensorLog{sensorSamp}.t; %diff this.
    end

end

%Valid Timestamps will be >0, they are initialized/declared as 0 above.
actualIMUDataReceived = sensorIMU(:,find(sensorTS>0)); %1293 Samples
actualIMUinds = find(sensorTS>0);

%Data Collected at 33Hz, blind for 3.2424 seconds at peak
blindInds = diff(actualIMUinds); % blindInds(840) = 107 time steps blind

%IMU bias in accel/gyro aXb = mean(actualIMUDataReceived(3,1:126)); 
aXb = 0.4109;
aYb = 0.4024;
aZb = 9.6343;
gXb = -2.6667e-04;
gYb = 6.6667e-05;
gZb = -0.0024;


%% Initial Estimate of R, Q: Process and Measurement Covariance respectively.
%I think when these are diagonal matrices they represent noise of each
%element ierrespective of the correlation/dependence of other parts. i.e.
%error in x axis accelerometer vs y axis accelerometer. (Variance)

%Quadrotor is on box/cube/platform for samples 0-218.
stableQuadTime = sensorTS(1:281);
stableValidInds = find(stableQuadTime>0); %Indices of Valid IMU data.
numStableDataSamples = numel(stableValidInds); %124

%{
varAx = var(sensorIMU(1,stableValidInds));
varAy = var(sensorIMU(2,stableValidInds));
varAz = var(sensorIMU(3,stableValidInds));
varGx = var(sensorIMU(4,stableValidInds));
varGy = var(sensorIMU(5,stableValidInds));
varGz = var(sensorIMU(6,stableValidInds));
%}

varAx = var(actualIMUDataReceived(1,1:100));
varAy = var(actualIMUDataReceived(2,1:100));
varAz = var(actualIMUDataReceived(3,1:100));
varGx = var(actualIMUDataReceived(4,1:100));
varGy = var(actualIMUDataReceived(5,1:100));
varGz = var(actualIMUDataReceived(6,1:100));


ukfR = diag([varAx, varAy, varAz, varGx, varGy, varGz]); %Measurement Cov
%ukfQ = diag([3.0466e-006, 5.4396e-006, 4.2754e-005, 1e-4, 1e-4, 1e-4]); %Process Cov
ukfQ = diag([3.0466e-006, 5.4396e-006, 4.2754e-005, 0.0110, 0.0281, 6.3328e-04]); %Process Cov


%Vicon Error Found according to var(bodyLinVel(1,1:281)) etc, same for
%bodyAngVel, but that data had really noisy stuff, so...

%% Aril Tags nPoint Pose Alg:
%L.Quan, Linear N-Point Camera Pose Determination, IEEE Trans. on Pattern Analysis and Machine Intelligence, vol. 21, no.7, July 1999

visionRobot = [];
for j = 1:numSamples
  
  if ~isempty(sensorLog{j}.id)
    [vrobotPos, vrobotOrient] = nPointPose(sensorLog{j});

    visionRobot = [visionRobot, [vrobotPos(1:3); vrobotOrient]];
  end
end


figure;
plot3(visionRobot(1,:), visionRobot(2,:), visionRobot(3,:));
grid on;




%% EKF(Accel Bias Tracked, 14 Dim)


%Initialize EKF with first two frames
[robotPos, robotOrient, robotToWorld] = nPointPose(sensorLog{522});
robotPos = robotPos(1:3);

%First Control Signal preserved in prevU for use when don't have data.
U = [sensorLog{522}.accImu(1); sensorLog{522}.accImu(2); sensorLog{522}.accImu(3); sensorLog{522}.omegaImu(1); ...
        sensorLog{522}.omegaImu(2); sensorLog{522}.omegaImu(3)]; 

prevU = U; %update prevU whenever have an input signal.



initVel = robotToWorld * qdLog{522}{1}.vel_body; %The initial state should be in the world/mat frame.
Xest = [robotPos; initVel; robotOrient; 0; 0; 0; 0; 0];



P = 0*eye(14); %diag([0 0 0 0 0 0 0 0 0 ]);   % State
R = diag([0.05, 0.05, 0.09, 1e-3, 1e-3, 1e-3, 0.05, 0.05, 0.09]);  % Process
Q = diag([0.001, 0.001, 0.008, 0.001, 0.029, 0.001]); % Measurement

%6x14
H = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
     0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
     0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0; ...
     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1; ...
     0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]; %Cherry Pick the state vector components
     

stateVector = [];
stateVector = [stateVector, Xest]; %Add First State, initialization.
prevTime = qdTimeLog{522};
normP = [];
normP = [normP, norm(P)];

worldLinVel = [];
worldLinV = robotToWorld *bodyLinVel(:,522);
worldLinVel = [worldLinVel, worldLinV];


worldAngVel = [];
worldAngV = robotToWorld *bodyAngVel(:,522);
worldAngVel = [worldAngVel, worldAngV];

h = figure;
figure(h);
hold all;

%nPPpos = [];
for i = 523:(numSamples-230) %Vicon Samples and Sensor Samples are same #, but have to check if actually have a sensor packet...
   
    %disp(i);  
   
    deltaT = qdTimeLog{i} - prevTime;%timeDiff(i);
    
 
    if (~isempty(sensorLog{i}.accImu)) %Assume omegaImu also not empty.
        U = [sensorLog{i}.accImu(1); sensorLog{i}.accImu(2); sensorLog{i}.accImu(3); sensorLog{i}.omegaImu(1); ...
        sensorLog{i}.omegaImu(2); sensorLog{i}.omegaImu(3)]; 

        prevU = U; %update prevU whenever have an input signal.
        
    else
       U = prevU;
       %U = [(Xest(4:6)-prevState(4:6))/deltaT; (Xest(7:9)-prevState(7:9))/deltaT]; %Based on Previous State * change in time: m/s /deltaT =m/s^2
    end
 
    
     %EKF Prediction
     %State Vector
     Xn(1) = Xest(1); % x
     Xn(2) = Xest(2); % y
     Xn(3) = Xest(3); % z
     Xn(4) = Xest(4); % velocity x
     Xn(5) = Xest(5); % velocity y
     Xn(6) = Xest(6); % velocity z
     Xn(7) = Xest(7); % roll
     Xn(8) = Xest(8); % pitch
     Xn(9) = Xest(9); % yaw
     Xn(10) = Xest(10); %Accelerometer x bias
     Xn(11) = Xest(11); %Accelerometer y bias
     Xn(12) = Xest(12); %Accelerometer z bias
     Xn(13) = Xest(13); %Camera phi bias
     Xn(14) = Xest(14); %Camera theta bias
 
     %Control Input
     Vn(1) = U(1); %The Control input modeled with noise, (already noisy, by nature).
     Vn(2) = U(2); 
     Vn(3) = U(3);
     Vn(4) = U(4); 
     Vn(5) = U(5); 
     Vn(6) = U(6); 
  
     
     %IMU Error in Accelerometer/Gyro
     errVn(1:6) = 0; %0 Mean Noise for Process Update (Prediction)
     errVn(7:9) = sqrt((diag([varAx, varAy, varAz]))) * randn(3,1); %Gaussian Random Walk Model of accelerometer time varying bias
     
     
     %G takes X(7)-X(9) instead of X(4)-X(6) for IMU
     G   = gFunc(Vn(1), Vn(2), Vn(3), Vn(4), Vn(5), Vn(6), Xn(10), Xn(11), Xn(12), Xn(7), Xn(8), Xn(9), ...
     deltaT, errVn(1), errVn(2), errVn(3), errVn(4), errVn(5), errVn(6)); 
     %L takes X(7-9) instead also, other remains same
     L   = lFunc(Vn(4), Vn(5), Vn(6), Xn(7), Xn(8), Xn(9), deltaT, errVn(4), errVn(5), errVn(6)); %subs(LJ);
     Xest = gEqn(Vn(1), Vn(2), Vn(3), Vn(4), Vn(5), Vn(6), Xn(1), Xn(10), Xn(11), Xn(12), Xn(13), Xn(14), Xn(2), Xn(3), Xn(4), Xn(5), Xn(6), Xn(7), Xn(8), Xn(9), ...
     deltaT, errVn(1), errVn(2), errVn(3), errVn(4), errVn(5), errVn(6), errVn(7), errVn(8), errVn(9)); 
     %G is 9x9, L should be 9x6, since R is 6x6;
     P    = G * P * G' + L * R * L';

     
  % EKF Correction: Only When have an image TAG to do nPointPose
  if ~isempty(sensorLog{i}.id)
      
    [Zr Zo robotToWorld ] = nPointPose(sensorLog{i}); %nPointPose spits out [x, y, z, roll, pitch, yaw] in world/april tag mat frame  
      
    Z = [Zr(1:3); Zo];
    %nPPpos = [nPPpos, Zr(1:3)];
    
    K = P * H' * inv(H * P * H' + Q); 
    Xest = Xest + K * (Z - H*Xest); %Cherry Pick the parts of Xest
    P    = (eye(14) - K * H) * P;
 
  end
    
  
    worldLinV = robotToWorld *bodyLinVel(:,i);
    worldLinVel = [worldLinVel, worldLinV];


    worldAngV = robotToWorld *bodyAngVel(:,i);
    worldAngVel = [worldAngVel, worldAngV];

  
    stateVector = [stateVector, Xest];
    
    prevTime = qdTimeLog{i};
    
    %2-norm of a matrix => largest singuluar value => sq.rt(largest eigen value(P'P))
    normP = [normP, norm(P)]; 
    

    
    %Show State Estimate and State Coveriance during Flight with Data Given
    figure(h);
    subplot(2,3, [1 4])
    grid on
    plot3(Xest(1), Xest(2), Xest(3));
    hold on
    subplot(2,3, [2 3])
    %subplot(2,3, 3)
    plot(i, norm(P));
    hold on
    subplot(2,3, [5 6])
    %subplot(2,3, 6)
    imshow(sensorLog{i}.img);
    
     
end


%% Plot Stuff

%{
%Ground Truth Trajectory
figure(1);
plot3(viconPos(1,:), viconPos(2,:), viconPos(3,:));
%Ground Truth Orientation
figure(2);
plot(viconEuler');
figure;
plot(bodyLinVel');
figure;
plot(bodyAngVel');


%Data (Only Packets Not Dropped Shown!!)
figure;
plot(actualIMUDataReceived(1:3,:)');
figure;
plot(actualIMUDataReceived(4:6,:)'); %Gyro
%}
%{
figure;
plot3(nPPpos(1,:), nPPpos(2,:), nPPpos(3,:), 'r');
grid on;
%}



%PLOT ALL 3D
figure;
%plot3(stateVector(1,:), stateVector(2,:), stateVector(3,:), 'b');
plot3(stateVector(1,1:1493), stateVector(2,1:1493), stateVector(3,1:1493), 'b');
hold on
plot3(stateVector(1,831:938), stateVector(2,831:938), stateVector(3,831:938), 'c-'); %Blind
plot3(stateVector(1,1493:end), stateVector(2,1493:end), stateVector(3,1493:end), 'k'); %End of Tags
plot3(visionRobot(1,:), visionRobot(2,:), visionRobot(3,:), 'r'); %Measurement (Blind Data Excluded, of course)
plot3(viconPos(1,522:end)+ 1.5, viconPos(2,522:end)+1.2081, viconPos(3,522:end), 'g'); %Vicon Ground Truth
grid on; 


%EKF vs Vicon 3D
figure;
plot3(stateVector(1,1:1493), stateVector(2,1:1493), stateVector(3,1:1493), 'b');
hold on
plot3(stateVector(1,831:938), stateVector(2,831:938), stateVector(3,831:938), 'c-'); %Blind
plot3(viconPos(1,522:2015)+ 1.5, viconPos(2,522:2015)+1.2081, viconPos(3,522:2015), 'g');
grid on;


%x
figure;
subplot(3,1,1);
title('x')
plot(stateVector(1,:), 'b');
%figure;
subplot(3,1,2);
plot(visionRobot(1,:), 'r');
%figure;
subplot(3,1,3);
plot(viconPos(1,522:end)+1.5, 'g'); %1.5 for Coordinate Frame Offset of Vicon. 


%y
figure;
subplot(3,1,1);
title('y')
plot(stateVector(2,:), 'b');
%figure;
subplot(3,1,2);
plot(visionRobot(2,:), 'r');
%figure;
subplot(3,1,3);
plot(viconPos(2,522:end)+1.2081, 'g'); %1.2081 for Coordinate Frame Offset of Vicon. 


%z
figure;
subplot(3,1,1);
title('z')
plot(stateVector(3,:), 'b');
%figure;
subplot(3,1,2);
plot(visionRobot(3,:), 'r');
%figure;
subplot(3,1,3);
plot(viconPos(3,522:end), 'g');


%vx
figure;
subplot(2,1,1);
title('vx')
plot(stateVector(4,:), 'b');
%figure;
subplot(2,1,2);
plot(worldLinVel(1,:), 'g');


%vy
figure;
subplot(2,1,1);
title('vy')
plot(stateVector(5,:), 'b');
%figure;
subplot(2,1,2);
plot(worldLinVel(2,:), 'g');


%vz
figure;
subplot(2,1,1);
title('vz')
plot(stateVector(6,:), 'b');
%figure;
subplot(2,1,2);
plot(worldLinVel(3,:), 'g');




%Angles Roll
figure;
subplot(3,1,1);
title('roll')
plot(stateVector(7,:), 'b');
%figure;
subplot(3,1,2);
plot(visionRobot(4,:), 'r');
%figure;
subplot(3,1,3);
plot(viconEuler(1,522:2084), 'g');

%Angles Pitch
figure;
subplot(3,1,1);
title('pitch')
plot(stateVector(8,:), 'b');
%figure;
subplot(3,1,2);
plot(visionRobot(5,:), 'r');
%figure;
subplot(3,1,3);
plot(viconEuler(2,522:2084), 'g');


%Angles Yaw
figure;
subplot(3,1,1);
title('yaw')
plot(stateVector(9,:), 'b');
%figure;
subplot(3,1,2);
plot(visionRobot(6,:), 'r');
%figure;
subplot(3,1,3);
plot(viconEuler(3,522:2084), 'g');



%EKF and Covariance Evolution
figure;
subplot(2,1,1)
grid on
title('Position State Estimate')
plot3(stateVector(1,:), stateVector(2,:), stateVector(3,:));
hold on
plot3(stateVector(1,831:938), stateVector(2,831:938), stateVector(3,831:938), 'c-'); %Blind
hold on
plot3(stateVector(1,1493:end), stateVector(2,1493:end), stateVector(3,1493:end), 'g-'); %No Observed Tags
subplot(2,1, 2)
title('Covariance Evolution')
plot([522:2084], normP); %2314numel(normP)
hold on
plot([522+831:522+938], normP(831:938), 'c');
hold on
plot([522+1493:522+numel(normP)], normP(1493:end), 'g');

% Error
derp = bsxfun(@plus, viconPos, [1.5; 1.2081; 0]); %Coorindate Frame Offset
theError =  derp(1:3, 522:2084) - stateVector(1:3,:); %2084
fprintf('Standard Mean Error:\n');
disp(std(mean(theError)));



