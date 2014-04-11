/*
 * Unscented Kalman Filter
 * State: Position, Velocity, Orientation, Accelerometer Bias
 * Fredy Monterroza 
 * 
*/

#include <iostream>
//#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>


#include "cholesky.hpp"
#include "InvertMatrix.hpp"

using std::cout;
using std::cin;
using std::endl;

using namespace boost::numeric;

ublas::vector<double> gEqn(ublas::vector<double>, ublas::vector<double>, ublas::vector<double>, double);
ublas::matrix<double> RPYtoRotMat(double, double, double); //Roll, Pitch, Yaw
double atanFM(double, double);
ublas::matrix<double> calculateCovEquation(ublas::matrix<double>, ublas::matrix<double>);

ublas::vector<double> sigmaPoint(14);
extern ublas::matrix<double> Q;
extern ublas::matrix<double> R;



void fmUKF (std::vector<dataVec>& stateVector, ublas::vector<double> x, ublas::matrix<double>& P, double deltaT, ublas::vector<double> z, ublas::vector<double> errVn, ublas::vector<double> Vn)

{
	cout << "fmUKF" << endl;
	
	ublas::matrix<double> sumPQ = P + Q;
	ublas::matrix<double> L(14, 14);
	size_t decompFail = cholesky_decompose(sumPQ, L);
	ublas::matrix<double> W_i(14,28);
	ublas::vector<double> aprioriStateEst(14);
	std::fill(aprioriStateEst.begin(), aprioriStateEst.end(), 0.0);
	ublas::matrix<double> X_i(14,28);
	
	if (decompFail == 0) {
		//Capture the Covariance
		subrange(W_i, 0, 14, 0, 14) = sqrt(14.0) * L;
		subrange(W_i, 0, 14, 14, 28) = -sqrt(14.0) * L;
				
		//Add the previous mean Xest to complete Sigma Points
		for(unsigned i = 0; i < W_i.size2(); ++i){ //bsxfun(@plus)
			column(W_i, i) = column(W_i, i) + x;
		}
		//cout << W_i << endl;
		
		
	} else { cout << "Require Square Symmertric Positive Definite Input" << endl; }
	
	//Propogate the Sigma Points
	ublas::vector<double> propogatedSigma(14);
	for(unsigned i = 0; i < W_i.size2(); ++i){
		sigmaPoint = column(W_i, i);
		propogatedSigma = gEqn(sigmaPoint, errVn, Vn, deltaT);
	    column(X_i, i) = propogatedSigma;
	    aprioriStateEst += propogatedSigma;
	}
	
	aprioriStateEst /= X_i.size2(); //Mean
	//cout << aprioriStateEst << endl;
	
	//W_i is W_i_prime (Propogated Sigma Points - Mean subtracted)
	for(unsigned i = 0; i < X_i.size2(); ++i){
		column(W_i, i) = column(X_i, i) - aprioriStateEst;
	}
	//cout << W_i << endl;

	ublas::matrix<double> Pbar_k = calculateCovEquation(W_i, W_i);
	
	//No Measurement Update Available
	if (z(0)+z(1)+z(2)+z(3)+z(4)+z(5) == 0){
		P = Pbar_k; 
		dataVec tempVec;
		for(unsigned i = 0; i < aprioriStateEst.size(); ++i){
		tempVec.push_back(aprioriStateEst(i));
		}
		stateVector.push_back(tempVec);
		cout << "No Measurement Update Available to Correct Apriori Estimate" << endl;
		return;
	}
	
	//Predicted Observation Zbar_k, Observation subset Z_i
	ublas::matrix<double> Z_i(6, 28); //Pos, Orient
	ublas::vector<double> Zbar_k(6);
	Zbar_k(0) = 0; Zbar_k(1) = 0; Zbar_k(2) = 0; Zbar_k(3) = 0; Zbar_k(4) = 0; Zbar_k(5) = 0;
	for(unsigned i = 0; i < X_i.size2(); ++i){
		Z_i(0, i) = X_i(0, i); Z_i(1, i) = X_i(1, i); Z_i(2, i) = X_i(2, i);
		Z_i(3, i) = X_i(6, i); Z_i(4, i) = X_i(7, i); Z_i(5, i) = X_i(8, i);
		Zbar_k += column(Z_i, i);
	}
	//Note: These will vary from matlab since this involves propogated sigma points, random walk model.
	Zbar_k /= 28;
	
	//Innovation Vector (Residual between Actual and Expected Measurement
	ublas::vector<double> Vk = z - Zbar_k;
	
	//Recycle, Reuse, Reduce: Observation subset with mean subtracted.
	for(unsigned i = 0; i < Z_i.size2(); ++i){
		column(Z_i, i) = column(Z_i, i) - Zbar_k;
	}
	
	
	//Innovation Covariance
	ublas::matrix<double> Pvv = calculateCovEquation(Z_i, Z_i);
	Pvv += R;
	//cout << Pvv << endl;
	//Cross Correlation
	ublas::matrix<double> Pxz = calculateCovEquation(W_i, Z_i);
	//cout << Pxz << endl;
	
	ublas::matrix<double> PvvInv(Pvv.size1(), Pvv.size2());
	if(InvertMatrix(Pvv, PvvInv)){ //via LU Factorization
		//cout << PvvInv << endl;
		//Kalman Gain
		ublas::matrix<double> K_k = prod(Pxz, PvvInv);
		cout << K_k << endl;
		
		//Measurement Updated State Estimate
		ublas::vector<double> aposterioriStateEst = aprioriStateEst + prod(K_k, Vk);
		cout << aposterioriStateEst << endl;
		dataVec tempVec;
		for(unsigned i = 0; i < aposterioriStateEst.size(); ++i){
		tempVec.push_back(aposterioriStateEst(i));
		}
		stateVector.push_back(tempVec);
		
		ublas::matrix<double> holdProd = prod(Pvv, trans(K_k)); 
		P = Pbar_k - prod(K_k, holdProd);
		cout << P << endl;
	}else {cout << "Pvv Approaching Singularity" << endl;}
	
	
	
}

//Non-Linear Dynamics
ublas::vector<double> gEqn(ublas::vector<double> sigmaPoint, ublas::vector<double> errVn, ublas::vector<double> Vn, double deltaT){	
	//cout << "gEqn" << endl;
	ublas::vector<double> propState(14);
	
	ublas::matrix<double> rotMat = RPYtoRotMat(sigmaPoint(6), sigmaPoint(7), sigmaPoint(8));
	//cout << rotMat << endl;
	
	//Acceleration
	ublas::vector<double> tempAccInput(3); tempAccInput(0) = Vn(0) - sigmaPoint(9); tempAccInput(1) = Vn(1) - sigmaPoint(10); tempAccInput(2) = Vn(2) - sigmaPoint(11); //IMU Accel - Tracked Bias
	ublas::vector<double> worldAcc = prod(rotMat, tempAccInput);
	worldAcc(0) = worldAcc(0) - .4109; worldAcc(1) = worldAcc(1) - .4024; worldAcc(2) = worldAcc(2) - 9.6343; //Subtract Gravity 
	//cout << worldAcc << endl;
	
	//Position
	propState(0) = sigmaPoint(0) + (sigmaPoint(3)*deltaT) + (0.5*worldAcc(0)*deltaT*deltaT);
	propState(1) = sigmaPoint(1) + (sigmaPoint(4)*deltaT) + (0.5*worldAcc(1)*deltaT*deltaT);
	propState(2) = sigmaPoint(2) + (sigmaPoint(5)*deltaT) + (0.5*worldAcc(2)*deltaT*deltaT);
	
	//Velocity
	propState(3) = sigmaPoint(3) + (worldAcc(0)*deltaT);
	propState(4) = sigmaPoint(4) + (worldAcc(1)*deltaT);
	propState(5) = sigmaPoint(5) + (worldAcc(2)*deltaT);
	
	//Orientation: Radians
	ublas::matrix<double> gyroIntegration = RPYtoRotMat(Vn(3)*deltaT, Vn(4)*deltaT, Vn(5)*deltaT);
	ublas::matrix<double> tempQuadOrientMat = prod(rotMat, gyroIntegration);
	
	propState(6) = asin(tempQuadOrientMat(1,2));
	propState(7) = atanFM(-tempQuadOrientMat(0,2)/cos(propState(6)),tempQuadOrientMat(2,2)/cos(propState(6)));
	propState(8) = atanFM(-tempQuadOrientMat(1,0)/cos(propState(6)),tempQuadOrientMat(0,0)/cos(propState(6)));
	
	
	//Accelerometer Bias (Random Walk)
	propState(9) = sigmaPoint(9) + (errVn(6)*deltaT); propState(10) = sigmaPoint(10) + (errVn(7)*deltaT); propState(11) = sigmaPoint(11) + (errVn(8)*deltaT);
	
	propState(12) = sigmaPoint(12); propState(13) = sigmaPoint(13);
	
	return propState;
	
	
}

//Return Rotation Matrix along ZXY
ublas::matrix<double> RPYtoRotMat(double phi, double theta, double psi){
	ublas::matrix<double> rotMat(3,3);
	rotMat(0,0) = cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta);
	rotMat(0,1) = cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta);
	rotMat(0,2) = -cos(phi)*sin(theta);
	
	rotMat(1,0) =  -cos(phi)*sin(psi);
	rotMat(1,1) = cos(phi)*cos(psi); 
	rotMat(1,2) = sin(phi);

	rotMat(2,0) = cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi);
	rotMat(2,1) = sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi);
	rotMat(2,2) = cos(phi)*cos(theta);
	
	return rotMat;
}

//atan used with Matlab Symbolic Explressions, replicated here for comparison.
double atanFM(double y, double x){
	return  2*atan(y / ( sqrt((x*x)+(y*y)) + x ));	
}

//Covariance Matrix calculated as weighted/averaged outer product of input points/vectors.
ublas::matrix<double> calculateCovEquation(ublas::matrix<double> A, ublas::matrix<double> B){
	
	//ublas::matrix<double> covMat(14, 14);
	ublas::matrix<double> covMat(A.size1(), B.size1());
	for(unsigned i = 0; i < covMat.size1(); ++i){
		for(unsigned j = 0; j < covMat.size2(); ++j){ 
			covMat(i, j) = 0.0;
		}
	}
	
	for(unsigned i = 0; i < A.size2(); ++i){
		covMat += outer_prod(column(A, i), column(B, i));
	}
	covMat /= 28;
	return covMat;
}





   
      
