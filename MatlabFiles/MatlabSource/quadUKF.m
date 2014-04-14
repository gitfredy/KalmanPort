
function [ x, P ] = quadUKF( x, P, deltaT, z, gEqn, errVn, Vn )
% Fredy Monterroza
% Ouputs Estimated State Vector [pos vel orientation(euler angles) bax bay baz bphi btheta]
% 14 State UKF
% Code Adapted from UKF Orientation Tracking Kraft Paper. Some of the
% assumptions maintained. Here, noise is additive and added to state covariance in Sigma Point computation
% so that they are already 'disturbed'. 
% Q is Process Covariance, R is measuerment Covariance
% References:
% Julier, SJ. and Uhlmann, J.K., Unscented Filtering and Nonlinear Estimation Proceedings of the IEEE, Vol. 92, No. 3, pp.401-422, 2004. 
% Kraft, Edgar, a Quaternion Based Unscented Kalman Filter for Orientation Tracking, University of Bonn. 

%Q = diag([8.8749e-05,  1.6563e-04, 1.4461e-04,  1.0e-3, 1.0e-3, 1.0e-3, 6.5681e-06, 6.5455e-06, 1.3567e-05, 1.0e-3, 1.0e-3, 1.0e-3, 1e-3, 1e-3]);
Q = 5e-3*eye(14);%2e-2*eye(14);%1e-2*eye(14);
%R = diag([3.0466e-006, 5.4396e-006, 4.2754e-005, 0.0110, 0.0281, 6.3328e-04]); 
R = diag([2e-1, 2e-1, 2e-1, 1e-4, 1e-4, 1e-4]);%1.5e-1*eye(6);


%Acquire W_i/Sigma Points from Cholesky of pervious state estimate error covariance.
[sqrootPQ, p] = chol(P + Q);

if p > 0 %Attempt at fixing loss of PSD (probably due to floating point? (Cov Divergence Issue present in project 2.
    
    disp('cov diverge');
    [~,D] = eig(P+Q);
    D(D <= 1e-10) = 1e-6;
    D(~isreal(D)) = 1e-6;
    sqrootPQ= chol(eye(14));
    W_i = [sqrt(14)*sqrootPQ, -sqrt(14)*sqrootPQ];
    
else    
    W_i = [sqrt(14)*sqrootPQ, -sqrt(14)*sqrootPQ];
end

%Propogate Sigma Points through non-linear process.
X = bsxfun(@plus, W_i, x); %Sigma Points are a sampling around the mean.
X_i = zeros(14, 28); %14 x 28 dimensional vectors of sigma points

for k = 1:28
   
    X_i(:,k) = gEqn(Vn(1), Vn(2), Vn(3), Vn(4), Vn(5), Vn(6), X(1,k), X(10,k), X(11,k), X(12,k), X(13,k), X(14,k), X(2,k), X(3,k), X(4,k), X(5,k), X(6,k), X(7,k), X(8,k), X(9,k), ...
    deltaT, errVn(1), errVn(2), errVn(3), errVn(4), errVn(5), errVn(6), errVn(7), errVn(8), errVn(9));
    
end

%Subtract Mean from Propogated Sigma Points and Obtain A-Priori Estimate Xhatbar_k = mean{Yi}
Y_i = X_i;
xhatbar_k = mean(Y_i, 2);


%Compute {W_i_prime} set by subtracting mean from step 4 from {Y_i}. To be used for calculating a-priori state estimate covariance.
W_i_prime = bsxfun(@minus, Y_i, xhatbar_k);

%Step 6: Compute apriori state estimate covariance (Pbar_k) from W_i_prime.  
%Average the 24 matrices:
Pbar_k = calculateCovEquation(W_i_prime, W_i_prime);
    
%End Process Update (Dynamics Update, Prediction): If there is no
%measurement, return the a-priori estiate.
if isempty(z)
    x = xhatbar_k;
    P = Pbar_k;
    return;
end


%Predicted Observation is just Z = h(x) + delta_noise, or simply, the
%estimated measured components of the state vector, that is:
Z_i = zeros(6, 28);
for d = 1:28 

    Z_i(:,d) = [Y_i(1:3,d); Y_i(7:9,d)]; %Vision (no Optical Flow), can only provide, Pose. No Linear/Angular Velocity, yet.
    
end

%Compute Zbar_k (predicted measurement) and Vk (innovation, surprise vector)
Zbar_k = mean(Z_i, 2);

Zk = z; %imuData;
Vk = Zk - Zbar_k; %6x1 vectors, [acceleration; angular velocity]


%Compute Innovation Covariance (Pvv = Pzz + R) eq.69
Z_imZbar_k = bsxfun(@minus, Z_i, Zbar_k);
Pzz = calculateCovEquation(Z_imZbar_k, Z_imZbar_k);

Pvv = Pzz + R; %Covariance of measurement: process covariance from computing Z_i up 'till now + Measurement Covariance.


%Compute Cross Correlation Pxz from W_i_prime and Z_i sets
Pxz = calculateCovEquation(W_i_prime, Z_imZbar_k);
%Pxz = (1/12) .* (W_i_prime*Z_imZbar_k');


%Step 11: Compute Kalman Gain K_k = Pxz * Pvv^-1
K_k = Pxz * inv(Pvv); %ignore matlab's advice 'cause this is matrix multiplicaton not solving a Ax=b


Xhat_k = xhatbar_k + K_k*Vk;
Pk = Pbar_k - K_k*Pvv*K_k'; %Estimate Error Covariance, 6x6
%End Measurement Update (Correction Step)

x = Xhat_k;
P = Pk;


end




