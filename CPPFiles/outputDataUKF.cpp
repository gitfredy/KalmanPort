/*
 * Write the resulting state estimates contained in stateVector to a text file
 * 
 */

#include <iostream>
#include <fstream>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>


void outputDataUKF(std::vector<dataVec>& stateVector) {
  std::ofstream outFile ("cppUKFData.txt");
  short position;
  if (outFile.is_open())
  {
    for(vector<dataVec>::iterator stateVectorIter = stateVector.begin(); stateVectorIter != stateVector.end(); ++stateVectorIter){
		position = 1;
		for(dataVec::iterator svIter = stateVectorIter->begin(); svIter != stateVectorIter->end(); ++svIter){
			if (position <= 3){
				outFile << *svIter << " ";
				position++;
			} else {continue;}
		}
		
		outFile << "\n";
	}
 
    outFile.close();
  }
  else std::cout << "Unable to open file";

	
}
