/// \defgroup Examples Smr examples program

/// \ingroup Examples
/// \file iva2008.cpp
/// \brief This program demontrates the use of motion player as a motion clip processing toolkit 
/// method


#include "Smr.h"
#include "argh.h"
#include "SmrPCA.h"

SmrSkeleton mySkeleton;
SmrSkeleton refPose;
std::vector<SmrMotion> motionVector;

static void loadMotions(){

  SmrMotion refMotion = loadMotionfromBvh("bindPose.bvh");
  refPose = refMotion.getSkeleton(0);

  for ( int i = 1; i<=28; i++)
  {
    ostringstream oss;
    if ( i < 10 )
      oss << 0 << i << ".bvh";
    else
      oss << i << ".bvh";
    string motionPath(oss.str());
    cout << "loading motion " + motionPath << endl;
    motionVector.push_back(loadMotionfromBvh(motionPath));
    cout << "motion " + motionPath + " loaded !" << endl;
  }
}

static void warpAll(){
  //SmrHermitePoly<double> poly(0.0,1.0,1.0,0.0);
  SmrHermitePoly<double> poly(0.0,1.0,1.0,0.0);
  int i = 1;
  std::vector<SmrMotion>::iterator it;
  for (it = motionVector.begin(); it < motionVector.end(); it++)
  {
    cout << "warping motion " << i << endl;
    (*it).warp(poly);
    i++;
  }
}

static void warpPrep(){
  //SmrHermitePoly<double> poly(0.0,1.0,1.0,0.0);
  SmrHermitePoly<double> poly(0.0,0.0,1.0,-1.0);
  cout << "warping motion 2" << endl;
  (motionVector[1]).warp(poly);
  cout << "warping motion 5" << endl;
  (motionVector[4]).warp(poly);
  cout << "warping motion 7" << endl;
  (motionVector[6]).warp(poly);
  cout << "warping motion 9" << endl;
  (motionVector[8]).warp(poly);
  cout << "warping motion 11" << endl;
  (motionVector[10]).warp(poly);
  cout << "warping motion 13" << endl;
  (motionVector[12]).warp(poly);
  cout << "warping motion 16" << endl;
  (motionVector[15]).warp(poly);
  cout << "warping motion 18" << endl;
  (motionVector[17]).warp(poly);
  cout << "warping motion 20" << endl;
  (motionVector[19]).warp(poly);
  cout << "warping motion 22" << endl;
  (motionVector[21]).warp(poly);
  cout << "warping motion 24" << endl;
  (motionVector[23]).warp(poly);
  cout << "warping motion 26" << endl;
  (motionVector[25]).warp(poly);
}

static void shortenAll(){
  std::vector<SmrMotion>::iterator it;

  double timeStep = (motionVector[0]).getTimeStep();
  int i = 1;
  for (it = motionVector.begin(); it < motionVector.end(); it++)
  {
    
    cout << "shorting motion " << i << endl;
    //(*it).resample(((*it).getTimeStep() * 1.3));
    (*it).resample(((*it).getTimeStep() * 0.85));
    (*it).setTimeStep(timeStep);
    i++;
  }

}

/**
 * \brief Exit properly 
 */
static void ShutDown(){
	cout << "Shutting down the light..." ;
	// shutdown Smr library
	Smr::shutdown();
	cout << "Done !" << endl;
}

int main(int argc, char *argv[])
{

/*********************** Parameters handling issues *********************************/


/*****************************The actual programm***********************************/

        //initialize Smr environment
  Smr::initSmr();

  loadMotions();

  cout << "warping motions" << endl;
  //warpPrep();
  warpAll();

  cout << "shortening motions" << endl;
  shortenAll();

  SmrMotion resultMotion;
  std::vector<SmrMotion>::iterator it;

  double timeStep;

  for (it = motionVector.begin(); it < motionVector.end(); it++)
  {
    cout << "merging motion" << endl;
    resultMotion + *it;
    timeStep = (*it).getTimeStep();
  }

  //read reference motion
  cout << "loading reference motion" << endl;
  //SmrMotion referenceMotion = loadMotionfromBvh("referenceMotion.bvh");
  SmrMotion referenceMotion = loadMotionfromBvh("referenceCool.bvh");

  SmrSkeleton lhandSkelTemplate,rhandSkelTemplate,bodySkelTemplate;

  lhandSkelTemplate = loadSkeletonfromBvh("lHandTemplate.bvh");
  rhandSkelTemplate = loadSkeletonfromBvh("rHandTemplate.bvh");
  bodySkelTemplate = loadSkeletonfromBvh("bodyTemplate.bvh");

  SmrMotion lhandReferenceMotion, rhandReferenceMotion, bodyReferenceMotion;
  SmrMotion lhandResultMotion, rhandResultMotion, bodyResultMotion;

  cout << "extract submotions from reference motion" << endl;
  lhandReferenceMotion = referenceMotion.extractSubSkel(lhandSkelTemplate);
  rhandReferenceMotion = referenceMotion.extractSubSkel(rhandSkelTemplate);
  bodyReferenceMotion  = referenceMotion.extractSubSkel(bodySkelTemplate);

  cout << "extract submotions from result motion" << endl;
  lhandResultMotion = resultMotion.extractSubSkel(lhandSkelTemplate);
  rhandResultMotion = resultMotion.extractSubSkel(rhandSkelTemplate);
  bodyResultMotion  = resultMotion.extractSubSkel(bodySkelTemplate);

  cout << "decomposing reference motion" << endl;
  SmrPCALinearizedQuat refPCAMotion(referenceMotion, SmrSkeleton::RELATIVEMODE);
  cout << "eigenpostures decomposition" << endl;
  refPCAMotion.computeEigenPostures();

//  cout << "dump PCA decomposition" << endl;
//  refPCAMotion.saveResult("reference_Motion",200);


  SmrSkeleton myPose = resultMotion.getSkeleton(50);
  SmrSkeleton meanPose = resultMotion.getMean(SmrSkeleton::RELATIVEMODE);
  //cout << "dump pose projection" << endl;
  //refPCAMotion.saveProjectedPose(resultMotion, meanPose, "projectedPose.dat");

  myPose = referenceMotion.getSkeleton(50);
  meanPose = referenceMotion.getMean(SmrSkeleton::RELATIVEMODE);
  //cout << "dump pose projection" << endl;
  //refPCAMotion.saveProjectedPose(referenceMotion, meanPose, "referencePose.dat");

  cout << resultMotion.getNumFrames() << endl;

  cout << "projecting and retrieving truncated projected resultMotion" << endl;
  resultMotion = refPCAMotion.getProjectedMotion(resultMotion, 50);

  bodyResultMotion  = resultMotion.extractSubSkel(bodySkelTemplate);

  cout << "re-weld result motion" << endl;
  bodyResultMotion.mergeMotionReplace(lhandResultMotion);
  bodyResultMotion.mergeMotionReplace(rhandResultMotion);

  resultMotion = bodyResultMotion;

  cout << resultMotion.getSkeleton(0) << endl;

  cout << resultMotion.getNumFrames() << endl;

  resultMotion.setTimeStep(timeStep);

  //resultMotion.insertSkeleton(refPose, 0);

  SmrMotion bindMotion = loadMotionfromBvh("bindPose.bvh");
  SmrSkeleton bindPose = bindMotion.getSkeleton(0);

  exportSkeletonToBvh("resultMotion2Weary.bvh",resultMotion,bindPose);

  // Get my scene prepared
  ShutDown();
  return 0;

}
