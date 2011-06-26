/*
*  SmrExporter.h
*  SmrCore
*
*  Created by Alexis Heloir/ Nicolas Courty on 18/10/05.
*  Copyright 2005 __MyCompanyName__. All rights reserved.
*
*/

#include "SmrExporter.h"



/**
* Write a pose into a bvh file
*/
//void exportSkeletonToBvh(string _filename, SMRSkeleton &_skeleton, SMRMotion &_motion)
//{
//
//  SMRSkeleton refSkeleton = _skeleton;
//
//  refSkeleton.setRotationOrder(SMRSkeleton::TRANSLATIONFIRST);
//
//  refSkeleton.setMode(SMRSkeleton::ABSOLUTEMODE);
//
//
//
//  string prevParent,currParent;
//  SMRJoint *prevJoint, *currJoint;
//
//  cout << "exporting skeleton to file : " << _filename << endl;
//
//  // defines a stream to be feed
//  ofstream outFile(_filename.c_str());
//
//  int jointNumber = 0;
//  prevJoint = currJoint = refSkeleton.getJoint(jointNumber);
//
//  //feed the file
//  cout << "feed the file" << endl;
//
//  outFile << "HIERARCHY" << endl;
//  outFile << "ROOT " << currJoint->getName() << endl;
//  outFile << "{" << endl;
//  outFile << "OFFSET 0 0 0" << endl;
//  outFile << "CHANNELS " << 6 << " Xrotation Yrotation Zrotation Xposition Yposition Zposition"<< endl;
//
//  int heapSize = 1;
//  for (unsigned int i=1; i < refSkeleton.getNumJoints(); i++)
//  {
//
//    currJoint = refSkeleton.getJoint(i);
//    cout << currJoint->getName() << endl;
//
//    if( currJoint->getParentName() != prevJoint->getName() )
//    {
//      cout << "branch jump at joint " << currJoint->getName() << endl;
//      cout << currJoint->getName() << " " << prevJoint->getName() << endl;
//      SMRJoint* leftParentJoint = refSkeleton.getJointByName(currJoint->getParentName());
//      SMRJoint* rightParentJoint = prevJoint;
//      do 
//      {
//        heapSize --;
//        cout << "outFile" << endl;
//        outFile << "}" << endl;
//        //leftParentJoint = _skeleton.getJointByName(leftParentJoint->getParentName());
//        rightParentJoint = refSkeleton.getJointByName(rightParentJoint->getParentName());
//      }
//      while(leftParentJoint != rightParentJoint);
//      prevJoint = rightParentJoint;
//    }
//      heapSize++;
//      outFile << "JOINT " << currJoint->getName() << endl;
//      outFile << "{" << endl;
//      outFile << "OFFSET " << currJoint->getPosition().X() - prevJoint->getPosition().X() \
//                    << " " << currJoint->getPosition().Y() - prevJoint->getPosition().Y() \
//                    << " " << currJoint->getPosition().Z() - prevJoint->getPosition().Z() << endl;
//      outFile << "CHANNELS " << 3 << " Xrotation Yrotation Zrotation"<< endl;
//      if (currJoint->isEndJoint())
//      {
//        cout << "joint " << currJoint->getName() << " is endJoint" << endl;
//        outFile << "End Site" << endl ;
//        outFile << "{" << endl;
//        outFile << "OFFSET " << currJoint->getEndLength().X() - currJoint->getPosition().X() \
//                      << " " << currJoint->getEndLength().Y() - currJoint->getPosition().Y() \
//                      << " " << currJoint->getEndLength().Z() - currJoint->getPosition().Z() << endl;
//        outFile << "}" << endl;
//      }
//      prevJoint = currJoint;
//  }
//  while (heapSize>0) { heapSize--; outFile << "}" << endl; }
//
//
//  outFile << "MOTION" << endl ;
//  outFile << "Frames: " << _motion.getNumFrames( ) << endl ;
//  outFile << "Frame Time: "<< _motion.getTimeStep( ) /1000.0 << endl ;
//
//  SMRSkeleton currentSkeleton;
//  SMRJoint *currentJoint, *refJoint;
//  SMRQuaternion currentRotation, refRotation;
//  double rX,rY,rZ;
//  SMRVector3 currentPosition;
//  for (unsigned int i = 0; i < _motion.getNumFrames( ); i++)
//  {
//    currentSkeleton = _motion.getSkeleton( i );
//    currentSkeleton.setRotationOrder(SMRSkeleton::TRANSLATIONFIRST);
//    currentJoint = currentSkeleton.getJoint(0);
//    refJoint = refSkeleton.getJoint(0);
//    currentRotation = currentJoint->getOrientation();
//    currentRotation.normalize();
//    refRotation = refJoint->getOrientation();
//    refRotation.normalize();
//    refRotation = Inverse(refRotation);
//    refRotation.normalize();
//    currentRotation = (refRotation * currentRotation).normalize();
//    currentRotation.toEulerAngles(rX,rY,rZ);
//    currentPosition = currentJoint->getPosition();
//    outFile << fixed  << rX/M_PI*180.0 << " " << rY/M_PI*180.0 << " " << rZ/M_PI*180.0 << " " \
//            << currentPosition.X() << " " << currentPosition.Y() \
//            << " " << currentPosition.Z() << " ";
////     if (i > 1400 && i << 3000)
////     {
////       cout << currentSkeleton.getNumJoints() << endl;
////     }
//    for (unsigned int j = 1; j<currentSkeleton.getNumJoints(); j++ )
//    {
//      currentJoint = currentSkeleton.getJoint(j);
//      refJoint = refSkeleton.getJoint(j);
//      currentRotation = currentJoint->getOrientation();
//      currentRotation.normalize();
//      refRotation = refJoint->getOrientation();
//      refRotation.normalize();
//      refRotation = Inverse(refRotation);
//      refRotation.normalize();
//      currentRotation = (refRotation * currentRotation).normalize();
//      currentRotation.toEulerAngles(rX,rY,rZ);
//      outFile << fixed  << rX/M_PI*180.0 << " " << rY/M_PI*180.0 << " " << rZ/M_PI*180.0 << " ";
//    }
//    outFile << endl;
//  }
//
//  outFile.close();
//
//}


void exportMotionToBVH(string _filename, SMRMotion &_motion, SMRSkeleton &_bindPose, RootRotationOrderType _rootRotOrder)
{

  SMRSkeleton refSkeleton;
  refSkeleton = _bindPose;

  SMRQuaternion id;
  id.identity();

  refSkeleton.setRotationOrder(TRANSLATIONFIRST);

  refSkeleton.setMode(ABSOLUTEMODE);

  string prevParent,currParent;
  SMRJoint *prevJoint, *currJoint;

  cout << "exporting skeleton to file : " << _filename << endl;

  // defines a stream to be feed
  ofstream outFile(_filename.c_str());

  int jointIndex = 0;
  //start with first joint
  prevJoint = currJoint = refSkeleton.getJoint(jointIndex);

  //feed the file for root joint
  cout << "feed the file" << endl;

  outFile << "HIERARCHY" << endl;
  outFile << "ROOT " << currJoint->getName() << endl;
  outFile << "{" << endl;
  outFile << "OFFSET 0 0 0" << endl;

  if (_rootRotOrder == ROOTROTATIONFIRST)
  {
    outFile << "CHANNELS " << 6 << " Xrotation Yrotation Zrotation Xposition Yposition Zposition"<< endl;
  }else
  {
    outFile << "CHANNELS " << 6 << " Xposition Yposition Zposition Xrotation Yrotation Zrotation"<< endl;
  }

  int heapSize = 1;
  for (unsigned int i=1; i < refSkeleton.getNumJoints(); i++)
  {

    currJoint = refSkeleton.getJoint(i);
    cout << currJoint->getName() << endl;

    if( currJoint->getParentName() != prevJoint->getName() )
    {
      cout << "branch jump at joint " << currJoint->getName() << endl;
      cout << currJoint->getName() << " " << prevJoint->getName() << endl;
      SMRJoint* leftParentJoint = refSkeleton.getJointByName(currJoint->getParentName());
      SMRJoint* rightParentJoint = prevJoint;
      do 
      {
        heapSize --;
        cout << "outFile" << endl;
        outFile << "}" << endl;
        //leftParentJoint = _skeleton.getJointByName(leftParentJoint->getParentName());
        rightParentJoint = refSkeleton.getJointByName(rightParentJoint->getParentName());
      }
      while(leftParentJoint != rightParentJoint);
      prevJoint = rightParentJoint;
    }
    heapSize++;
    outFile << "JOINT " << currJoint->getName() << endl;
    outFile << "{" << endl;
    outFile << "OFFSET " << currJoint->getPosition().X() - prevJoint->getPosition().X() \
      << " " << currJoint->getPosition().Y() - prevJoint->getPosition().Y() \
      << " " << currJoint->getPosition().Z() - prevJoint->getPosition().Z() << endl;
    outFile << "CHANNELS " << 3 << " Xrotation Yrotation Zrotation"<< endl;
    if (currJoint->isEndJoint())
    {
      cout << "joint " << currJoint->getName() << " is endJoint" << endl;
      outFile << "End Site" << endl ;
      outFile << "{" << endl;
      outFile << "OFFSET " << currJoint->getEndLength().X() - currJoint->getPosition().X() \
        << " " << currJoint->getEndLength().Y() - currJoint->getPosition().Y() \
        << " " << currJoint->getEndLength().Z() - currJoint->getPosition().Z() << endl;
      outFile << "}" << endl;
      /*      heapSize++;
      outFile << "JOINT " << currJoint->getName() << endl;
      outFile << "{" << endl;
      outFile << "OFFSET " << currJoint->getPosition().X() \
      << " " << currJoint->getPosition().Y() \
      << " " << currJoint->getPosition().Z() << endl;
      outFile << "CHANNELS " << 3 << " Xrotation Yrotation Zrotation"<< endl;
      if (currJoint->isEndJoint())
      {
      cout << "joint " << currJoint->getName() << " is endJoint" << endl;
      outFile << "End Site" << endl ;
      outFile << "{" << endl;
      outFile << "OFFSET " << currJoint->getEndLength().X() \
      << " " << currJoint->getEndLength().Y() \
      << " " << currJoint->getEndLength().Z() << endl;
      outFile << "}" << endl;*/
    }
    prevJoint = currJoint;
  }
  while (heapSize>0) 
  {
    heapSize--; 
    outFile << "}" << endl; 
  }

  //_motion.hemispherize();

  refSkeleton = _motion.getSkeleton(0);
  refSkeleton = _bindPose;
  refSkeleton.setRotationOrder(TRANSLATIONFIRST);

  outFile << "MOTION" << endl ;
  outFile << "Frames: " << _motion.getNumFrames( ) << endl ;
  outFile << "Frame Time: "<< _motion.getTimeStep( ) << endl ;

  SMRSkeleton currentSkeleton;
  SMRJoint *currentJoint, *refJoint;
  SMRQuaternion currentRotation, refRotation, heapRotation, invHeapRotation;
  double rX,rY,rZ;
  SMRVector3 currentPosition;
  for (unsigned int i = 0; i < _motion.getNumFrames( ); i++)
  {
    // set root joint
    std::map<string, SMRQuaternion> heapRotHeap;
    currentSkeleton = _motion.getSkeleton( i );
    currentSkeleton.setRotationOrder(TRANSLATIONFIRST);
    currentJoint = currentSkeleton.getJoint(0);
    refJoint = refSkeleton.getJoint(0);
    currentRotation = currentJoint->getOrientation();
    currentRotation.normalize();

    refRotation = refJoint->getOrientation();
    refRotation.normalize();
    refRotation = Inverse(refRotation);
    refRotation.normalize();

    currentRotation = (currentRotation * refRotation).normalize();
    currentRotation.toEulerAngles(rX,rY,rZ);
    currentPosition = currentJoint->getPosition();

    if (_rootRotOrder == ROOTROTATIONFIRST)
    {
      outFile << fixed  << rX/M_PI*180.0 << " " << rY/M_PI*180.0 << " " << rZ/M_PI*180.0 << " " \
      << currentPosition.X() << " " << currentPosition.Y()  << " " << currentPosition.Z() << " ";
    }else
    {
      outFile << fixed  << currentPosition.X() << " " << currentPosition.Y() << " " << currentPosition.Z()\
      << " "<< rX/M_PI*180.0 << " " << rY/M_PI*180.0 << " "  << rZ/M_PI*180.0 << " ";
    }

    heapRotHeap[currentJoint->getName()] = Inverse(refRotation);

    for (unsigned int j = 1; j<currentSkeleton.getNumJoints(); j++ )
    {
      heapRotation.identity();
      currentJoint = currentSkeleton.getJoint(j);
      refJoint = refSkeleton.getJoint(j);
      currentRotation = currentJoint->getOrientation();
      currentRotation.normalize();
      refRotation = refJoint->getOrientation();
      refRotation.normalize();

      if ((currentJoint->getParentName()).length() > 0) 
      {
        heapRotation = heapRotHeap[currentJoint->getParentName()];
        invHeapRotation = Inverse(heapRotation);
        invHeapRotation.normalize();
      }

      refRotation = Inverse(refRotation);
      refRotation.normalize();
      currentRotation = (heapRotation * currentRotation * refRotation * invHeapRotation ).normalize();
      currentRotation.toEulerAngles(rX,rY,rZ);
      refRotation = Inverse(refRotation);
      heapRotation = (heapRotation * refRotation);
      heapRotation.normalize();
      heapRotHeap[currentJoint->getName()] = heapRotation;

      outFile << fixed  << rX/M_PI*180.0 << " " << rY/M_PI*180.0 << " " << rZ/M_PI*180.0 << " ";
    }
    outFile << endl;
  }

  outFile.close();

}

void exportMotionToBVH(string _filename, SMRMotion &_motion, RootRotationOrderType _rootRotOrder)
{

  SMRSkeleton refSkeleton = _motion.getSkeleton(0);

  SMRQuaternion id;
  id.identity();
  for(unsigned int i = 0; i < refSkeleton.getNumJoints(); i++)
  {
    refSkeleton.getJoint(i)->setOrientation(id);
  }

  refSkeleton.setRotationOrder(TRANSLATIONFIRST);
  refSkeleton.checkEndJoints();
  refSkeleton.setMode(ABSOLUTEMODE);
  refSkeleton.checkEndJoints();

  string prevParent,currParent;
  SMRJoint *prevJoint, *currJoint;

  cout << "exporting skeleton to file : " << _filename << endl;

  // defines a stream to be feed
  ofstream outFile(_filename.c_str());

  int jointNumber = 0;
  prevJoint = currJoint = refSkeleton.getJoint(jointNumber);

  //feed the file
  cout << "feed the file" << endl;

  outFile << "HIERARCHY" << endl;
  outFile << "ROOT " << currJoint->getName() << endl;
  outFile << "{" << endl;
  outFile << "OFFSET 0 0 0" << endl;
  if (_rootRotOrder == ROOTROTATIONFIRST)
  {
    outFile << "CHANNELS " << 6 << " Xrotation Yrotation Zrotation Xposition Yposition Zposition"<< endl;
  }else
  {
    outFile << "CHANNELS " << 6 << " Xposition Yposition Zposition Xrotation Yrotation Zrotation"<< endl;
  }

  int heapSize = 1;
  for (unsigned int i=1; i < refSkeleton.getNumJoints(); i++)
  {

    currJoint = refSkeleton.getJoint(i);
    cout << currJoint->getName() << endl;

    if( currJoint->getParentName() != prevJoint->getName() )
    {
      cout << "branch jump at joint " << currJoint->getName() << endl;
      cout << currJoint->getName() << " " << prevJoint->getName() << endl;
      SMRJoint* leftParentJoint = refSkeleton.getJointByName(currJoint->getParentName());
      SMRJoint* rightParentJoint = prevJoint;
      do 
      {
        heapSize --;
        cout << "outFile" << endl;
        outFile << "}" << endl;
        //leftParentJoint = _skeleton.getJointByName(leftParentJoint->getParentName());
        rightParentJoint = refSkeleton.getJointByName(rightParentJoint->getParentName());
      }
      while(leftParentJoint != rightParentJoint);
      prevJoint = rightParentJoint;
    }
    heapSize++;
    outFile << "JOINT " << currJoint->getName() << endl;
    outFile << "{" << endl;
    outFile << "OFFSET " << currJoint->getPosition().X() - prevJoint->getPosition().X() \
      << " " << currJoint->getPosition().Y() - prevJoint->getPosition().Y() \
      << " " << currJoint->getPosition().Z() - prevJoint->getPosition().Z() << endl;
    outFile << "CHANNELS " << 3 << " Xrotation Yrotation Zrotation"<< endl;
    if (currJoint->isEndJoint())
    {
      cout << "joint " << currJoint->getName() << " is endJoint" << endl;
      outFile << "End Site" << endl ;
      outFile << "{" << endl;
      outFile << "OFFSET " << currJoint->getEndLength().X() - currJoint->getPosition().X() \
        << " " << currJoint->getEndLength().Y() - currJoint->getPosition().Y() \
        << " " << currJoint->getEndLength().Z() - currJoint->getPosition().Z() << endl;
      outFile << "}" << endl;
      /*      heapSize++;
      outFile << "JOINT " << currJoint->getName() << endl;
      outFile << "{" << endl;
      outFile << "OFFSET " << currJoint->getPosition().X() \
      << " " << currJoint->getPosition().Y() \
      << " " << currJoint->getPosition().Z() << endl;
      outFile << "CHANNELS " << 3 << " Xrotation Yrotation Zrotation"<< endl;
      if (currJoint->isEndJoint())
      {
      cout << "joint " << currJoint->getName() << " is endJoint" << endl;
      outFile << "End Site" << endl ;
      outFile << "{" << endl;
      outFile << "OFFSET " << currJoint->getEndLength().X() \
      << " " << currJoint->getEndLength().Y() \
      << " " << currJoint->getEndLength().Z() << endl;
      outFile << "}" << endl;*/
    }
    prevJoint = currJoint;
  }
  while (heapSize>0) 
  {
    heapSize--; 
    outFile << "}" << endl; 
  }

  //_motion.hemispherize();

  refSkeleton = _motion.getSkeleton(0);
  refSkeleton.setRotationOrder(TRANSLATIONFIRST);
  refSkeleton.checkEndJoints();
  for(unsigned int i = 0; i < refSkeleton.getNumJoints(); i++)
  {
    refSkeleton.getJoint(i)->setOrientation(id);
  }

  outFile << "MOTION" << endl ;
  outFile << "Frames: " << _motion.getNumFrames( ) << endl ;
  outFile << "Frame Time: "<< _motion.getTimeStep( ) << endl ;

  SMRSkeleton currentSkeleton;
  SMRJoint *currentJoint, *refJoint;
  SMRQuaternion currentRotation, refRotation, heapRotation, invHeapRotation;
  double rX,rY,rZ;
  SMRVector3 currentPosition;
  for (unsigned int i = 0; i < _motion.getNumFrames( ); i++)
  {
    // set root joint
    std::map<string, SMRQuaternion> heapRotHeap;
    currentSkeleton = _motion.getSkeleton( i );
    currentSkeleton.setRotationOrder(TRANSLATIONFIRST);
    currentJoint = currentSkeleton.getJoint(0);
    refJoint = refSkeleton.getJoint(0);
    currentRotation = currentJoint->getOrientation();
    currentRotation.normalize();

    refRotation = refJoint->getOrientation();
    refRotation.normalize();
    refRotation = Inverse(refRotation);
    refRotation.normalize();

    currentRotation = (currentRotation * refRotation).normalize();
    currentRotation.toEulerAngles(rX,rY,rZ);
    currentPosition = currentJoint->getPosition();
    if (_rootRotOrder == ROOTROTATIONFIRST)
    {
      outFile << fixed  << rX/M_PI*180.0 << " " << rY/M_PI*180.0 << " " << rZ/M_PI*180.0 << " " \
      << currentPosition.X() << " " << currentPosition.Y()  << " " << currentPosition.Z() << " ";
    }else
    {
      outFile << fixed  << currentPosition.X() << " " << currentPosition.Y() << " " << currentPosition.Z()\
      << " "<< rX/M_PI*180.0 << " " << rY/M_PI*180.0 << " "  << rZ/M_PI*180.0 << " ";
    }


    heapRotHeap[currentJoint->getName()] = Inverse(refRotation);

    for (unsigned int j = 1; j<currentSkeleton.getNumJoints(); j++ )
    {
      heapRotation.identity();
      currentJoint = currentSkeleton.getJoint(j);
      refJoint = refSkeleton.getJoint(j);
      currentRotation = currentJoint->getOrientation();
      currentRotation.normalize();
      refRotation = refJoint->getOrientation();
      refRotation.normalize();

      if ((currentJoint->getParentName()).length() > 0) 
      {
        heapRotation = heapRotHeap[currentJoint->getParentName()];
        invHeapRotation = Inverse(heapRotation);
        invHeapRotation.normalize();
      }

      refRotation = Inverse(refRotation);
      refRotation.normalize();
      currentRotation = (heapRotation * currentRotation * refRotation * invHeapRotation ).normalize();
      currentRotation.toEulerAngles(rX,rY,rZ);
      refRotation = Inverse(refRotation);
      heapRotation = (heapRotation * refRotation);
      heapRotation.normalize();
      heapRotHeap[currentJoint->getName()] = heapRotation;

      outFile << fixed  << rX/M_PI*180.0 << " " << rY/M_PI*180.0 << " " << rZ/M_PI*180.0 << " ";
    }
    outFile << endl;
  }

  outFile.close();

}


void exportPoseToBVH(string _filename, SMRSkeleton &_pose){
  SMRMotion motion; 
  motion.insertSkeleton(_pose);
  motion.insertSkeleton(_pose); 
  motion.insertSkeleton(_pose); 
  exportMotionToBVH(_filename, motion, _pose);
}