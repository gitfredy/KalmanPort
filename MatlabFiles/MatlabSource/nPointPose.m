function [ robotXYZ, orient, robot2WorldOrientation] = nPointPose( data )
%NPOINTPOSE Compute Pose in world frame based on 2D-3D correspondences.
%   Fredy Monterroza
%   data.id Contains the observed tags.
%   data contains the struct with image, pi corners, etc. data.img = april
%   tag
%   robotXYZ is world frame position (april tag mat frame)
%   orient is euler angles (orientation in the mat/world frame)
%   robot2WorldOrientation is the rotation matrix of the robot orientation
%   according to the ZXY euler angle transformation.
%   L.Quan, Linear N-Point Camera Pose Determination, IEEE Trans. on Pattern Analysis and Machine Intelligence, vol. 21, no.7, July 1999

global tagIDs;
global tagWorldCoords;
global cam2RobotHomoStable;
global kMat;

camPose = [];
quadPose = [];

robotXYZ = [];
%robot2WorldOrientation = [];
orient = [];

dataPoints = [];
M = []; %2nx9 Matrix to be build and used in Brute Force R, T calculation
if ~isempty(data.id) %only perform pose estimation when at least 1 april tag is in frame

    numTags = numel(data.id);
    %Extract all the available April Tags to be used as data points in
    %the 2n x 9 A matrix in x ~ A X equation for calibrated correspondences
    %Do all the bottom left points, then br, tr, tl (order dont matter)
    for tag = 1:numTags
        [~, tagLinInd] = ismember(data.id(tag), tagIDs);
        %Bottom Left
        %for bl = 1:size(data{i}.p1, 2)
        for bl = 1:1
            %xy = kMatInv * [data{i}.p1(:,bl); 1]; %Calibrated Coords
            xy = kMat \ [data.p1(:,tag); 1]; %Calibrated Coords
            X = tagWorldCoords{tagLinInd}.blx; %World Coords
            Y = tagWorldCoords{tagLinInd}.bly;
            nextPoint = [xy(1); xy(2); X; Y];
            dataPoints = [dataPoints, nextPoint];
        end
        %Bottom Right
        %for br = 1:size(data{i}.p2, 2)
        for br = 1:1
            %xy = kMatInv * [data{i}.p2(:,br); 1]; %Calibrated Coords
            xy = kMat \ [data.p2(:,tag); 1]; %Calibrated Coords
            X = tagWorldCoords{tagLinInd}.brx; %World Coords
            Y = tagWorldCoords{tagLinInd}.bry;
            nextPoint = [xy(1); xy(2); X; Y];
            dataPoints = [dataPoints, nextPoint];
        end
        %Top Right
        %for tr = 1:size(data{i}.p3, 2)
        for tr = 1:1    
            %xy = kMatInv * [data{i}.p3(:,tr); 1]; %Calibrated Coords
            xy = kMat \ [data.p3(:,tag); 1]; %Calibrated Coords
            X = tagWorldCoords{tagLinInd}.trx; %World Coords
            Y = tagWorldCoords{tagLinInd}.try;
            nextPoint = [xy(1); xy(2); X; Y];
            dataPoints = [dataPoints, nextPoint];
        end
        %Top Left
        for tl = 1:size(data.p4, 2)
        %for tl = 1:1
            %xy = kMatInv * [data{i}.p4(:,tl); 1]; %Calibrated Coords
            xy = kMat \ [data.p4(:,tag); 1]; %Calibrated Coords
            X = tagWorldCoords{tagLinInd}.tlx; %World Coords
            Y = tagWorldCoords{tagLinInd}.tly;
            nextPoint = [xy(1); xy(2); X; Y];
            dataPoints = [dataPoints, nextPoint];
        end

    end
    %dataPoints holds the x, y, X, Y correspondences
    %M = [];
    for point = 1:size(dataPoints, 2)
        x = dataPoints(1, point);
        y = dataPoints(2, point);
        X = dataPoints(3, point);
        Y = dataPoints(4, point);
        xY = x*Y;
        yX = y*X;
        xX = x*X;
        yY = y*Y;
        %M = [M; -X, 0, 0, -Y, 0, xY -1, 0, x; 0, -X, yX, 0, -Y, 0, 0 -1, y];
        M = [M; -X, 0, xX, -Y, 0, xY -1, 0, x; 0, -X, yX, 0, -Y, yY, 0 -1, y];
    end


    %nullM = null(M);
    [~, ~, V] = svd(M);
    nullM = V(:,end);
    %Reflection
    if  nullM(end) < 0;
        nullM = nullM * -1;
    end


    %nullM = null(M);

    if ~isempty(nullM)
        %nullM = nullM(:,end); %Take the smallest eigenvalue  rightmost vector
        nullM = reshape(nullM, 3, 3);
        normR1 = norm(nullM(:,1));
        A = (1/normR1) * nullM; %[r1, r2, T] / ||r1|| to handle lambda scale factor
        %A = normc(nullM);
        r1 = A(:,1);
        r2 = A(:,2);
        r3 = cross(r1, r2);

        R = [r1, r2, r3]; %Orientation and
        T = A(:,3); %Translation (Pose) of Camera in World Coordinates

        if ~isreal(R)
            disp('NOOOOOOOOOOOOOOOOOOOOOOOOOOOO');
        end


        homoR = [R; zeros(1, 3)];
        homoT = [T; 1];
       
        world2Robot = cam2RobotHomoStable * [homoR, homoT];

        robot2World = inv(world2Robot); %pseudo inverse? A\b?
        robotXYZ = robot2World * [0; 0; 0; 1];
        

        robot2WorldOrientation = robot2World(1:3, 1:3);
        
        orient = RotMatToRPY(robot2WorldOrientation);
        roll = orient(1);
        pitch = orient(2);
        yaw = orient(3);
        newPose = [-roll; -pitch; -yaw + pi/2; robot2World(1:3,4)];

        quadPose = [quadPose, newPose];

        robot2WorldOrientation = RPYToRotMat([-roll; -pitch; -yaw]); 

    else
        disp('Empty Null Space nullM');

    end

end


end

