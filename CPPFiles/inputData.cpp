#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iomanip>
#include <sstream>

#include <algorithm>
#include <vector>
#include <map>
#include <list>

/* This file is to read in the quadData.mat cell structure containing quadLog, quadTime, and sensorLog and parse 
   it into corresponding data structures necessary for implementation of the EKF/UKF.
   Fredy Monterroza
*/

//using namespace std;
using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::string;
using std::stringstream;
using std::list;
using std::ifstream;


typedef vector<double> dataVec;

dataVec parseStringForData(string&);

std::vector<dataVec> sensorLogAccImu;
std::vector<dataVec> sensorLogOmegaImu; 
std::vector<dataVec> visionRobotPos; 
std::vector<dataVec> visionRobotOrient; 
std::vector<dataVec> viconEuler; 
std::vector<dataVec> viconPos;
std::list<double> viconTime;

//void inputData(void);
//int main() { //void inputData()
void inputData(){

 //-----Read Process Input (Control Input, IMU Accelerometer)-----
 //vector<dataVec> sensorLogAccImu; 
 string inSensorAccel;
 ifstream inFileSensorAccel ("../Data/processInputAcc.txt");
 if(inFileSensorAccel.is_open()){
  while(getline(inFileSensorAccel, inSensorAccel)){
   dataVec imuAccel;
   imuAccel = parseStringForData(inSensorAccel);
   sensorLogAccImu.push_back(imuAccel);
  }
  inFileSensorAccel.close();
 }else{
  cout << "Error Opening File" << endl;
 }

 /* 
 for(vector<dataVec>::iterator imuAccelIter = sensorLogAccImu.begin(); imuAccelIter != sensorLogAccImu.end(); ++imuAccelIter){
  for(dataVec::iterator accelImuIter = imuAccelIter->begin(); accelImuIter != imuAccelIter->end(); ++accelImuIter){
   cout << *accelImuIter << " ";
  }
  cout << endl;
 }
 */
 
 //-----Read Process Input (Control Input, IMU Gyroscope, Angular Velocity)-----
 //vector<dataVec> sensorLogOmegaImu; 
 string inSensorOmega;
 ifstream inFileSensorOmega ("../Data/processInputOmega.txt");
 if(inFileSensorOmega.is_open()){
  while(getline(inFileSensorOmega, inSensorOmega)){
   dataVec imuOmega;
   imuOmega = parseStringForData(inSensorOmega);
   sensorLogOmegaImu.push_back(imuOmega);
  }
  inFileSensorOmega.close();
 }else{
  cout << "Error Opening File" << endl;
 }
 /*
 for(vector<dataVec>::iterator imuOmegaIter = sensorLogOmegaImu.begin(); imuOmegaIter != sensorLogOmegaImu.end(); ++imuOmegaIter){
  for(dataVec::iterator omegaImuIter = imuOmegaIter->begin(); omegaImuIter != imuOmegaIter->end(); ++omegaImuIter){
   cout << *omegaImuIter << " ";
  }
  cout << endl;
 }
 */
 
 //-----Read poseNPPPos (Measurement Update, Position Output From nPointPose Algorithm)-----
 //vector<dataVec> visionRobotPos; 
 string inVisionRobotPos;
 ifstream inFileVisionRobotPos ("../Data/poseNPPPos.txt");
 if(inFileVisionRobotPos.is_open()){
  while(getline(inFileVisionRobotPos, inVisionRobotPos)){
   dataVec robotPos;
   robotPos = parseStringForData(inVisionRobotPos);
   visionRobotPos.push_back(robotPos);
  }
  inFileVisionRobotPos.close();
 }else{
  cout << "Error Opening File" << endl;
 }
 /*
 for(vector<dataVec>::iterator visionPosIter = visionRobotPos.begin(); visionPosIter != visionRobotPos.end(); ++visionPosIter){
  for(dataVec::iterator posVisionIter = visionPosIter->begin(); posVisionIter != visionPosIter->end(); ++posVisionIter){
   cout << *posVisionIter << " ";
  }
  cout << endl;
 }
*/

 //-----Read poseNPPOrient (Measurement Update, Orientation Output From nPointPose Algorithm)-----
 //vector<dataVec> visionRobotOrient; 
 string inVisionRobotOrient;
 ifstream inFileVisionRobotOrient ("../Data/poseNPPOrient.txt");
 if(inFileVisionRobotOrient.is_open()){
  while(getline(inFileVisionRobotOrient, inVisionRobotOrient)){
   dataVec robotOrient;
   robotOrient = parseStringForData(inVisionRobotOrient);
   visionRobotOrient.push_back(robotOrient);
  }
  inFileVisionRobotOrient.close();
 }else{
  cout << "Error Opening File" << endl;
 }
 /*
 for(vector<dataVec>::iterator visionOrientIter = visionRobotOrient.begin(); visionOrientIter != visionRobotOrient.end(); ++visionOrientIter){
  for(dataVec::iterator orientVisionIter = visionOrientIter->begin(); orientVisionIter != visionOrientIter->end(); ++orientVisionIter){
   cout << *orientVisionIter << " ";
  }
  cout << endl;
 }
*/


 //-----Read Vicon Euler Angles-----
 //vector<dataVec> viconEuler; 
 string inEuler;
 //ifstream inFileEuler ("quadData/viconEuler.txt");
 ifstream inFileEuler ("../Data/viconEuler.txt");
 if(inFileEuler.is_open()){
  while(getline(inFileEuler, inEuler)){
   dataVec euler;
   euler = parseStringForData(inEuler);
   viconEuler.push_back(euler);
  }
  inFileEuler.close();
 }else{
  cout << "Error Opening File" << endl;
 }
 /*
 for(vector<dataVec>::iterator eulerIter = viconEuler.begin(); eulerIter != viconEuler.end(); ++eulerIter){
  for(dataVec::iterator iterAngs = eulerIter->begin(); iterAngs != eulerIter->end(); ++iterAngs){
   cout << *iterAngs << " ";
  }
  cout << endl;
 }
 */

 //-----Read Vicon Position-----
 //vector<dataVec> viconPos; 
 string inPos;
 //ifstream inFilePos ("quadData/viconPos.txt");
 ifstream inFilePos ("../Data/viconPos.txt");
 if(inFilePos.is_open()){
  while(getline(inFilePos, inPos)){
   dataVec pos;
   pos = parseStringForData(inPos);
   viconPos.push_back(pos);
  }
  inFilePos.close();
 }else{
  cout << "Error Opening File" << endl;
 }
 
 /*
 for(vector<dataVec>::iterator posIter = viconPos.begin(); posIter != viconPos.end(); ++posIter){
  for(dataVec::iterator iterPos = posIter->begin(); iterPos != posIter->end(); ++iterPos){
   cout << *iterPos << " ";
  }
  cout << endl;
 }
 */


 //-----Read Vicon Times------
 //list<double> viconTime;
 string inTime;
 double inTimeNum;
 //ifstream inFileTime ("quadData/viconTime.txt");
 ifstream inFileTime ("../Data/viconTime.txt");
 if(inFileTime.is_open()){
  while(getline(inFileTime, inTime)){
   //inTimeNum = atof(inTime.c_str());
   //printf("%f\n", inTimeNum);
   inTimeNum = stod(inTime);
   //cout << setprecision(17) << inTimeNum << endl;
   viconTime.push_back(inTimeNum);
 
  }
  inFileTime.close();
 }else {
 cout << "Error Opening File" << endl;
 }
 /*
 for (list<double>::iterator vtIter = viconTime.begin(); vtIter != viconTime.end(); ++vtIter){
  cout << setprecision(17) << *vtIter << endl;
 }
 */
 
}

dataVec parseStringForData(string& inData){
 string dataElement;
 stringstream dataStream(inData);
 dataVec dataVector;
 double dataElementNum;
 while(dataStream >> dataElement){
  dataElementNum = stod(dataElement);
  //cout << setprecision(17) << dataElementNum << endl;
  dataVector.push_back(dataElementNum);
 }

 return dataVector;

}

/*

 //-----Read Sensor (IMU) Accelerometer-----
 vector<dataVec> sensorLogAccImu; 
 string inSensorAccel;
 ifstream inFileSensorAccel ("../Data/sensorIMUaccel.txt");
 if(inFileSensorAccel.is_open()){
  while(getline(inFileSensorAccel, inSensorAccel)){
   dataVec imuAccel;
   imuAccel = parseStringForData(inSensorAccel);
   sensorLogAccImu.push_back(imuAccel);
  }
  inFileSensorAccel.close();
 }else{
  cout << "Error Opening File" << endl;
 }
 
 for(vector<dataVec>::iterator imuAccelIter = sensorLogAccImu.begin(); imuAccelIter != sensorLogAccImu.end(); ++imuAccelIter){
  for(dataVec::iterator accelImuIter = imuAccelIter->begin(); accelImuIter != imuAccelIter->end(); ++accelImuIter){
   cout << *accelImuIter << " ";
  }
  cout << endl;
 }
 

 //-----Read Sensor (IMU) Gyroscope (Angular Velocity)-----
 vector<dataVec> sensorLogOmegaImu; 
 string inSensorOmega;
 ifstream inFileSensorOmega ("../Data/sensorIMUangVel.txt");
 if(inFileSensorOmega.is_open()){
  while(getline(inFileSensorOmega, inSensorOmega)){
   dataVec imuOmega;
   imuOmega = parseStringForData(inSensorOmega);
   sensorLogOmegaImu.push_back(imuOmega);
  }
  inFileSensorOmega.close();
 }else{
  cout << "Error Opening File" << endl;
 }
 
 for(vector<dataVec>::iterator imuOmegaIter = sensorLogOmegaImu.begin(); imuOmegaIter != sensorLogOmegaImu.end(); ++imuOmegaIter){
  for(dataVec::iterator omegaImuIter = imuOmegaIter->begin(); omegaImuIter != imuOmegaIter->end(); ++omegaImuIter){
   cout << *omegaImuIter << " ";
  }
  cout << endl;
 }

 
  //-----Read visionRobotPos (Position Output From nPointPose Algorithm)-----
 vector<dataVec> visionRobotPos; 
 string inVisionRobotPos;
 ifstream inFileVisionRobotPos ("../Data/visionRobotPos.txt");
 if(inFileVisionRobotPos.is_open()){
  while(getline(inFileVisionRobotPos, inVisionRobotPos)){
   dataVec robotPos;
   robotPos = parseStringForData(inVisionRobotPos);
   visionRobotPos.push_back(robotPos);
  }
  inFileVisionRobotPos.close();
 }else{
  cout << "Error Opening File" << endl;
 }
 
 for(vector<dataVec>::iterator visionPosIter = visionRobotPos.begin(); visionPosIter != visionRobotPos.end(); ++visionPosIter){
  for(dataVec::iterator posVisionIter = visionPosIter->begin(); posVisionIter != visionPosIter->end(); ++posVisionIter){
   cout << *posVisionIter << " ";
  }
  cout << endl;
 }
 
*/

