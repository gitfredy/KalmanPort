function [rpy] = RotMatToRPY(R)
%Rotation Matrix R (about ZXY)


phi = asin(R(2,3));
theta = atan3(-R(1,3)/cos(phi),R(3,3)/cos(phi));
psi = atan3(-R(2,1)/cos(phi),R(2,2)/cos(phi));
rpy =[phi;theta;psi];


