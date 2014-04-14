/*
 * quadStateEst.cpp
 * Main Loop: Obtain Measurement, Feed into Kalman Filter, Present State
 * State Estimation of a Quadrotor for Autonomous Control of Flight
 * State: [Position, Velocity, Orientation, Accelerometer Bias, Roll/Pitch Bias at 0]
 * Fredy Monterroza 
 * 
*/

#include <iostream>
//#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <iomanip>
//#include <sstream>

//#include <algorithm>
//#include <vector>
//#include <map>
//#include <list>

#include <math.h>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/math/complex/acos.hpp>
#include <boost/math/complex/asin.hpp>
#include <boost/math/complex/atan.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include "inputData.cpp"
#include "fmUKF.cpp"
#include "outputDataUKF.cpp"

using std::cout;
using std::cin;
using std::endl;

using namespace boost::numeric;

//Data Containers from Sensor (IMU), nPointPose Measurement, Vicon
extern std::vector<dataVec> sensorLogAccImu;
extern std::vector<dataVec> sensorLogOmegaImu; 
extern std::vector<double> quadTimeLog;
extern std::vector<dataVec> visionRobotPos; 
extern std::vector<dataVec> visionRobotOrient; 
extern std::vector<dataVec> viconEuler; 
extern std::vector<dataVec> viconPos;
extern std::list<double> viconTime;

//Store State Estimates 
std::vector<dataVec> stateVector;

//Computation Variables
ublas::vector<double> Xest(14);
ublas::matrix<double> P(14,14);
double deltaT;
double prevT;
ublas::vector<double> Z(6);
ublas::vector<double> errVn(9);
ublas::vector<double> Vn(6);


//Process Covariance (Q) and Measurement Covariance (R)
ublas::matrix<double> Q(14,14);
ublas::matrix<double> R(6,6);


int main(){
	//For use with random walk model of accelerometer bias.
	boost::mt19937 rng;
	boost::normal_distribution<> xBias(0.0, 0.0094);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > xBiasSamp(rng, xBias);
    boost::normal_distribution<> yBias(0.0,  0.0129);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > yBiasSamp(rng, yBias);
    boost::normal_distribution<> zBias(0.0, 0.0120);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > zBiasSamp(rng, zBias);
	
	inputData();
	
	//Fill Process Covariance Q
	for(ublas::matrix<double>::iterator1 iter1Q = Q.begin1(); iter1Q != Q.end1(); ++iter1Q){ //Row by row
		for(ublas::matrix<double>::iterator2 iter2Q = iter1Q.begin(); iter2Q != iter1Q.end(); ++iter2Q){ //Go from col to the next col for fixed row
			if (iter1Q.index1() == iter2Q.index2()){
				*iter2Q = .005;
			} else {
				*iter2Q = 0;
			}
		}
	 } 
	 
	 //Fill Measurement Covariance R
	 int traceIndex = 1;
	 for(ublas::matrix<double>::iterator1 iter1R = R.begin1(); iter1R != R.end1(); ++iter1R){ //Row by row
		for(ublas::matrix<double>::iterator2 iter2R = iter1R.begin(); iter2R != iter1R.end(); ++iter2R){ //Go from col to the next col for fixed row
			if (iter1R.index1() == iter2R.index2()){
				if (traceIndex <=3) { *iter2R = .2; }
				else { *iter2R = .0001; }
				traceIndex++;
			}
		}
	 } 
	 
	 
	 
	/*
	 * Initialize UKF
	 */ 
	
	//First State estimate formed from known onboard sensor values and pose estimate/transforations of N-Point Pose. (Refer to the Matlab source)
	Xest(0) = -0.197612876747667;
	Xest(1) = 0.079773798179542;
	Xest(2) = 0.873867606945072;
	Xest(3) = -0.361840566716804;
	Xest(4) = 0.364435964213824;
	Xest(5) = 0.107057884736583;
	Xest(6) = 0.003720102419965;
	Xest(7) = -.0003130927657911031;
	Xest(8) = 1.539666077119250;
	Xest(9) = 0.0;
	Xest(10) = 0.0;
	Xest(11) = 0.0;
	Xest(12) = 0.0;
	Xest(13) = 0.0;
	
	 
	//Initialize prevU: Previous Control Input to be used when data dropped. (Maintain Heading when Blind). 
	//Note: All the used control inputs already collected in sensorLogAccImu, sensorLogOmegaImu, simply iterate.
	
	//Initialize prevTime: First Time Stamp
	vector<double>::iterator qtIter = quadTimeLog.begin();
	prevT = *qtIter++;
	
	
	//Save the Position Estimate Component for use with RViz/3D trajectory visualization
	dataVec tempVec;
	for(unsigned i = 0; i < Xest.size(); ++i){
		tempVec.push_back(Xest(i));
	}
	stateVector.push_back(tempVec);
	
	//TODO: 2-Norm of Matrix P
	
	//First State Covariance Matrix P
	for(ublas::matrix<double>::iterator1 iter1P = P.begin1(); iter1P != P.end1(); ++iter1P){ //Row by row
		for(ublas::matrix<double>::iterator2 iter2P = iter1P.begin(); iter2P != iter1P.end(); ++iter2P){ //Go from col to the next col for fixed row
			if (iter1P.index1() == iter2P.index2()){
				*iter2P = .01;
			} else {
				*iter2P = 0;
			}
		}
	 } 
	 
	
	//Use Accelerometer Input Iterator as Marker for End of Loop
	vector<dataVec>::iterator accIter = sensorLogAccImu.begin();
	vector<dataVec>::iterator omegaIter = sensorLogOmegaImu.begin();
	vector<dataVec>::iterator pNPPIter = visionRobotPos.begin();
	vector<dataVec>::iterator oNPPIter = visionRobotOrient.begin();
	dataVec::iterator accInput;
	dataVec::iterator gyroInput;
	dataVec::iterator nppPos;
	dataVec::iterator nppOrient;
	++accIter; ++omegaIter; ++pNPPIter; ++oNPPIter; //First input/NPP unused here, but used in Matlab
	
	/*
	 * Unscented Kalman Filter Loop 
	 */ 
	int kfIteration = 1; //0th Element in Data Vectors contains the first inputs/pose estimates
	
	while(accIter != sensorLogAccImu.end()){
		cout << kfIteration << endl;
		accInput = accIter->begin();
		gyroInput = omegaIter->begin();
		for(unsigned i = 0; i < (Vn.size()/2); ++i){
			Vn(i) = *accInput++;
			Vn(i+3) = *gyroInput++;
		}
		deltaT = *qtIter - prevT;
		errVn(0) = 0; errVn(1) = 0;  errVn(2) = 0;  errVn(3) = 0;  errVn(4) = 0;  errVn(5) = 0;  
		errVn(6) = xBiasSamp(); errVn(7) = yBiasSamp(); errVn(8) = zBiasSamp();
		
		nppPos = pNPPIter->begin();
		nppOrient = oNPPIter->begin();
		for(unsigned i = 0; i < (Z.size()/2); ++i){
			Z(i) = *nppPos++;
			Z(i+3) = *nppOrient++;
		}
		//cout << Z << endl;
		
		fmUKF(stateVector, Xest, P, deltaT, Z, errVn, Vn);
		//cout << P << endl;
	
		//Update prevTime
		prevT = *qtIter;
		
		//TODO: Save/Evaluate normP
		
		//Advance Iterators, kfIteration
		++qtIter; ++accIter; ++omegaIter; ++pNPPIter; ++oNPPIter;
		kfIteration++;
	}
	cout << "Number of Kalman Filter Iterations = " << kfIteration << endl;
	
	//Write Position Estimates to Text File
	outputDataUKF(stateVector);
	
	//Form Vector of Position for Visualization Purposes
	
	//Publish Data for use with RViz
	

	return 0;
	
}




