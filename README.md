##Kalman Filter Port
###Porting of Matlab Based Code for the State Estimation of a Quadrotor into C++/ROS framework. (UKF/EKF).

####State Vector for both Kalman Filter implementations is 14 Dimensional: 
####[position, velocity, orientation, imu accelerometer bias, roll/pitch bias]

####Implementation uses Boost 1.49 and C++11/STL

######Information
The Kalman Filter is an optimal estimator. If
the noise of the system and observations can be modeled as
Gaussian, then the Kalman Filter minimizes the mean square
error of the estimate. In addition, the filter is recursive and can
thus provide state estimates as new data becomes available. If
you have a good estimate, then combining the filter with a
pre-process step of gain learning can achieve a reliable system. 
The purpose of the project was to fly a quadrotor using
either the Extended Kalman Filter or the Unscented Kalman
Filter with an IMU and single camera serving as input to
the system. Once a good state estimator was developed, this
would be used in conjunction with a PD controller which
would use the position and velocity estimates to calculate the
required thrusts and moments to reach the desired position.
The nanoplus quadrotor has an onboard attitude controller
that runs at a higher frequency than the position and velocity
controller. This means the orientation estimate would only
be kept as an exercise of the kalman filter since the attitude
controller requires a larger rate of input than is available
(The IMU and Camera data were synchronized at 33Hz).
Having obtained a good controller and good state estimate,
the quadrotor could track a desired trajectory. If you imagine
a situation where the quadrotor would be required to follow a
desired trajectory as accurately as possible (say, surveillance
duty) then it would be necessary to learn an optimal set of
gains. Here, only the Kalman Filters are provided.

######Files
[.cpp Source] (/CPPFiles)
[Matlab Source] (/MatlabFiles/MatlabSource)

[CPP Unscented Kalman Filter] (/CPPFiles/fmUKF.cpp)
[Matlab Unscented Kalman Filter] (/MatlabFiles/MatlabSource/quadUKF.m)

[CPP UKF Loop] (/CPPFiles/quadStateEst.cpp)
[Matlab UKF Loop] (/MatlabFiles/MatlabSource/imuUKF.m)

[Matlab EKF] (/MatlabFiles/MatlabSource/final650.m)

[Matlab N-Point Pose Algorithm] (/MatlabFiles/MatlabSource/nPointPose.m)

[Symbolic Functions] (/MatlabFiles/MatlabSource/symbolicFuncs.m)
[Ouput Sensor Log Script] (/MatlabFiles/MatlabSource/outputSensorLog.m)
[Read into STL Vectors] (/CPPFiles/inputData.cpp)

[Boost LU Factorization Matrix Inversion] (/CPPFiles/InvertMatrix.hpp)
[Boost Cholesky Decomposition] (/CPPFiles/cholesky.cpp)

[Data Collected Wirelessly From Quadrotor] (/Data)




