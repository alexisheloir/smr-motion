/**
 *  This program is a test ofr the BVH import/extport capabilities of the SMR library
 */


#include "Smr.h"

int main (int argc, char** argv)
{

  PathManager * pm = PathManager::getInstance();

  pm->addPath("../../../Data/bvh");
  pm->addPath("../../Data/bvh");

/***********************************************************************************
                      testing bvh import/export capabilities
 ***********************************************************************************/
  cout << "testing bvh import/export" << endl;

  string inputFileName = pm->getFileName("test.bvh");

  cout << "loading " + inputFileName << endl;
  // load test motion
  SMRMotion motion = loadMotionFromBVH(inputFileName);

  int point = inputFileName.rfind(".");
  string outputFileName = inputFileName.erase(point,inputFileName.length()-point);

  outputFileName += "_exported.bvh";

  // write test motion
  cout << "exporting " + outputFileName << endl; 
  exportMotionToBVH(outputFileName, motion);

  // read exported motion
  cout << "loading writen file " + outputFileName << endl;
  SMRMotion exportedMotion = loadMotionFromBVH(outputFileName);

  cout << "comparing the two motions" << endl;
  if (motion == exportedMotion)
  {
    double globalDistance = 0.0;
    for (unsigned int i = 0; i < motion.getNumFrames(); i++)
    {
      globalDistance += PointsCloudDistance(motion.getSkeleton(i), exportedMotion.getSkeleton(i));
    }
    cout << "BVH import/export test successfully passed, global distance between original and exported motion is " << globalDistance << endl;
  }else
  {
    cout << "Exported motion differs form original motion" << endl;
    return 1;
  }

/***********************************************************************************
                      testing acclaim import/export capabilities
 ***********************************************************************************/

  pm->addPath("../../../Data/acclaim");
  pm->addPath("../../Data/acclaim");

  cout << "testing acclaim import/export" << endl;

  string inputSkeletonFileName = pm->getFileName("test.asf");
  string inputMotionFileName = pm->getFileName("test.amc");

  cout << "loading " + inputSkeletonFileName << endl;
  // load test motion#
  SMRSkeleton skeleton = loadSkeletonFromAcclaim(inputSkeletonFileName);
  cout << "loading " + inputMotionFileName << endl;
  motion = loadMotionFromAcclaim(inputSkeletonFileName,inputMotionFileName);

  point = inputSkeletonFileName.rfind(".");
  string outputSkeletonFileName = inputSkeletonFileName.erase(point,inputSkeletonFileName.length()-point);
  point = inputMotionFileName.rfind(".");
  string outputMotionFileName = inputMotionFileName.erase(point,inputMotionFileName.length()-point);

  outputSkeletonFileName += "_exported.asf";
  outputMotionFileName += "_exported.amc";

  cout << "changing imported motions' bindPose " + inputMotionFileName << endl;
  motion.changeBindPose(skeleton);
  skeleton = motion.getSkeleton(0);
  //motion.changeBindPose(skeleton);
  //skeleton = motion.getSkeleton(0);
  SMRQuaternion identity;  
  identity.identity();
  for (unsigned int i = 0; i < skeleton.getNumJoints(); i++)
  {
    skeleton.getJoint(i)->setOrientation(identity);
  }

  // write test motion
  cout << "exporting " + outputMotionFileName << endl; 
  exportMotionToAcclaim(outputMotionFileName, motion);
  exportSkeletonToAcclaim(outputSkeletonFileName, skeleton);

  // read exported motion
  cout << "loading writen file " + outputMotionFileName << endl;
  exportedMotion = loadMotionFromAcclaim(outputSkeletonFileName, outputMotionFileName);

  cout << "comparing the two motions" << endl;
  if (motion == exportedMotion)
  {
    double globalDistance = 0.0;
    for (unsigned int i = 0; i < motion.getNumFrames(); i++)
    {
      globalDistance += PointsCloudDistance(motion.getSkeleton(i), exportedMotion.getSkeleton(i));
    }
    cout << "ACCLAIM import/export test successfully passed, global distance between original and exported motion is " << globalDistance << endl;
  }else
  {
    cout << "Exported motion differs form original motion" << endl;
    return 1;
  }

/***********************************************************************************
                      testing vicom import/export capabilities
 ***********************************************************************************/

  pm->addPath("../../../Data/vicon");
  pm->addPath("../../Data/vicon");

  cout << "testing Vicon import/export" << endl;

  inputSkeletonFileName = pm->getFileName("test.vsk");
  inputMotionFileName = pm->getFileName("test.V");

  cout << "loading " + inputSkeletonFileName << endl;
  // load test motion#
  skeleton = loadSkeletonFromVSK(inputSkeletonFileName);
  cout << "loading " + inputMotionFileName << endl;
  motion = loadMotionFromVSK(inputSkeletonFileName,inputMotionFileName);

  point = inputSkeletonFileName.rfind(".");
  outputSkeletonFileName = inputSkeletonFileName.erase(point,inputSkeletonFileName.length()-point);
  point = inputMotionFileName.rfind(".");
  outputMotionFileName = inputMotionFileName.erase(point,inputMotionFileName.length()-point);

  outputMotionFileName += "_exported.bvh";

  cout << "changing imported motions' bindPose " + inputMotionFileName << endl;
  motion.changeBindPose(skeleton);
  skeleton = motion.getSkeleton(0);
  //motion.changeBindPose(skeleton);
  //skeleton = motion.getSkeleton(0);
  for (unsigned int i = 0; i < skeleton.getNumJoints(); i++)
  {
    skeleton.getJoint(i)->setOrientation(identity);
  }

  // write test motion
  cout << "exporting " + outputMotionFileName << endl; 
  exportMotionToBVH(outputMotionFileName, motion);
  //exportSkeletonToAcclaim(outputSkeletonFileName, skeleton);

  // read exported motion
  cout << "loading writen file " + outputMotionFileName << endl;
  exportedMotion = loadMotionFromBVH(outputMotionFileName);

  cout << "comparing the two motions" << endl;
  if (motion == exportedMotion)
  {
    double globalDistance = 0.0;
    for (unsigned int i = 0; i < motion.getNumFrames(); i++)
    {
      globalDistance += PointsCloudDistance(motion.getSkeleton(i), exportedMotion.getSkeleton(i));
    }
    cout << "VICON import/export test successfully passed, global distance between original and exported motion is " << globalDistance << endl;
  }else
  {
    cout << "Exported motion differs form original motion" << endl;
    return 1;
  }

  return 0;
}