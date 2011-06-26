/**
* \ingroup Examples
* \file DTW.cpp
* \brief This program demonstrates the computation of Dynamic time Warping between
* two motions
*/
#include "CImg.h"
using namespace cimg_library;

#include "Smr.h"
#include "SmrDTW.h"




/// Define colors used to plot the warping path
const unsigned char red  [3] = {255,  0,  0};

int main( void )
{

	// ---------------------------------------------------------------------------------------
	// SMR Related stuff 
	// ---------------------------------------------------------------------------------------
	/// Init Smr library
	Smr::initSmr(false);

  LOG_INFO(logger, "Hello, World!");

	cout << "*** DTW test using CImg class" << endl 
		<< "-----------------------------" << endl;

	/// load two walking motions from acclaim file format
	SMRMotion motion1 = loadMotionFromAcclaim(getFileName("acclaim/08.asf"),getFileName("acclaim/08_normal_walk.amc"));
	SMRMotion motion2 = loadMotionFromAcclaim(getFileName("acclaim/08.asf"),getFileName("acclaim/08_walk_long_stride.amc"));
	/// switch every pose to absolute mode
	motion1.changeMode(ABSOLUTEMODE);
	motion2.changeMode(ABSOLUTEMODE);
	/// defines a distance matrix
	Matrix dMatrix(motion1.getNumFrames(),motion2.getNumFrames());
	/// compute it
	buildDistanceMatrix(motion1.getQuaternionicTimeSerie("lfemur"),motion2.getQuaternionicTimeSerie("lfemur"),dMatrix);
	//buildDistanceMatrix(motion1,motion2,dMatrix);
	/// compute DTW and store the warping path into two vectors of coordinates
	vector<unsigned int> alignX;vector<unsigned int> alignY;
	performDTW(dMatrix,alignX,alignY);


	// ---------------------------------------------------------------------------------------
	// Display with CImg part
	// ---------------------------------------------------------------------------------------
	//
	CImg <double> dImg;
	dImg.resize(motion1.getNumFrames(),motion2.getNumFrames(),1,3,0);	
	for (int i = 0; i<dImg.dimx(); i++)
		for (int j = 0; j<dImg.dimy(); j++)
			dImg(i,j,0,0)=dImg(i,j,0,1)=dImg(i,j,0,2)=dMatrix(i+1,j+1);
	dImg.normalize(0,255);
	for (unsigned int k=0;k<alignX.size();k++){
		dImg(alignX.at(k)-1,alignY.at(k)-1,0,0)=255;
		dImg(alignX.at(k)-1,alignY.at(k)-1,0,1)=dImg(alignX.at(k)-1,alignY.at(k)-1,0,2)=0;}
	CImgDisplay main_disp(motion1.getNumFrames()*2,motion2.getNumFrames()*2,"Distance Matrix");
	main_disp.display(dImg);
	while (!main_disp.is_closed){
		if (main_disp.is_resized) main_disp.resize().display(dImg);
		cimg::wait(40);}
	// ---------------------------------------------------------------------------------------


	Smr::shutdown();
	return 0;
}
