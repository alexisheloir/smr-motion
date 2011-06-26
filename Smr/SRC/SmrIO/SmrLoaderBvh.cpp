/*
*  SmrLoader.cpp
*  SmrCore
*
*  Created by Alexis Heloir/Nicolas Courty on 18/10/05.
*  Copyright 2005 __MyCompanyName__. All rights reserved.
*
*/

#include "SmrLoader.h"


SMRSkeleton loadSkeletonFromBVH(string _filename, SMRTransformationOrderType _type)
{
#ifdef TRACE
  cout << "Loading skeleton from file : " << _filename << endl;
#endif
  // defines a stream to be read from
  ifstream infile(_filename.c_str());
  if (infile.fail())
  {
    cout << "file " +  _filename + " not found" << endl;
    exit(1);
  }

  // get the bind pose skeleton
  SMRSkeleton  bindPose(RELATIVEMODE,TRANSLATIONFIRST);

  expect("HIERARCHY", infile);
  expect("ROOT", infile);

  vector<string> parentStack;
  vector<string> m_valueDescVect;
  parentStack.push_back("");

  SMRQuaternion id;
  id.identity();

  string nextToken("");
  infile >> nextToken;
  string nodeName("");
  do
  {
    nodeName = nextToken;

    expect("{", infile);
    expect("OFFSET", infile);

    double offsetX,offsetY,offsetZ;
    infile >> offsetX; infile >> offsetY; infile >> offsetZ;

    expect("CHANNELS", infile);

    int nDofs = 0;
    infile >> nDofs;

    string DofType("");
    if (nDofs == 0)
    {
      m_valueDescVect.push_back(nodeName+"+"+DofType); // not optimal
      //cout << " " << nodeName+"+"+DofType << endl;
    }
    else
    {
      for (int i=0; i<nDofs; i++)
      {
        infile >> DofType;
        m_valueDescVect.push_back(nodeName+"+"+DofType); // not optimal
        //cout << i << " " << nodeName+"+"+DofType << endl;
      }
    }
    //string nextToken;
    infile >> nextToken;

    if(nextToken == "End")
    {
      // create a final joint
      SMRJoint* currentJoint(new SMRJoint(true));

      double endOffsetX,endOffsetY,endOffsetZ;

      expect("{", infile);
      expect("OFFSET", infile);

      infile >> endOffsetX;
      infile >> endOffsetY;
      infile >> endOffsetZ;

      // set name
      currentJoint->setName(nodeName);
      // set parent name
      currentJoint->setParentName(parentStack[parentStack.size()-1]);
      // set position
      currentJoint->setPosition(SMRVector3(offsetX,offsetY,offsetZ));
      // set orientation
      currentJoint->setOrientation(id);
      // set end length
      currentJoint->setEndLength(SMRVector3(endOffsetX, endOffsetY, endOffsetZ));

      bindPose.insertJoint(currentJoint);

      expect("}", infile);
      infile >> nextToken;
      infile >> nextToken;
      while (nextToken=="}")
      {
        // pop the top of the stack of the parent name stack
        parentStack.pop_back();
        infile >> nextToken;
      }
      if (nextToken=="MOTION") break;
      infile >> nextToken;


    }
    else
    {
      // this is a standard joint
      SMRJoint* currentJoint(new SMRJoint);
      // set name
      currentJoint->setName(nodeName);
      // set parent name
      currentJoint->setParentName(parentStack[parentStack.size()-1]);
      // set position
      currentJoint->setPosition(SMRVector3(offsetX,offsetY,offsetZ));
      // set orientation
      currentJoint->setOrientation(id);
      bindPose.insertJoint(currentJoint);
      infile >> nextToken;
      if (nextToken == "}")
      {
        do
        {
          // pop the top of the stack of the parent name stack
          parentStack.pop_back();
          infile >> nextToken;
        }while (nextToken=="}");
        if (nextToken=="MOTION") break;
        infile >> nextToken;
      }
      else
      {
        // put it on top of the stack of the parent name stack
        parentStack.push_back(nodeName);
      }
    }
  }
  while (nextToken != "MOTION");

  bindPose.checkEndJoints();
  bindPose.setRotationOrder(_type);//bindPose.setRotationOrder(ROTATIONFIRST);
  bindPose.checkEndJoints();

  return bindPose;

}




SMRMotion loadMotionFromBVH(string _filename, SMRTransformationOrderType _type)
{

//#ifdef TRACE
  cout << "Loading skeleton from file : " << _filename << endl;
//#endif
  // defines a stream to be read from
  ifstream infile(_filename.c_str());
  if (infile.fail())
  {
    cout << "file " +  _filename + " not found" << endl;
    exit(1);
  }


  // get the bind pose skeleton
  SMRSkeleton  bindPose(RELATIVEMODE,TRANSLATIONFIRST);

  expect("HIERARCHY", infile);
  expect("ROOT", infile);

  vector<string> parentStack;
  vector<string> m_valueDescVect;
  parentStack.push_back("");

  SMRQuaternion id;
  id.identity();

  string nextToken("");
  infile >> nextToken;
  string nodeName("");
  do
  {
    nodeName = nextToken;

    expect("{", infile);
    expect("OFFSET", infile);

    double offsetX,offsetY,offsetZ;
    infile >> offsetX; infile >> offsetY; infile >> offsetZ;

    expect("CHANNELS", infile);

    int nDofs = 0;
    infile >> nDofs;

    string DofType("");
    if (nDofs == 0)
    {
      m_valueDescVect.push_back(nodeName+"+"+DofType); // not optimal
      //cout << " " << nodeName+"+"+DofType << endl;
    }
    else
    {
      for (int i=0; i<nDofs; i++)
      {
        infile >> DofType;
        m_valueDescVect.push_back(nodeName+"+"+DofType); // not optimal
        //cout << i << " " << nodeName+"+"+DofType << endl;
      }
    }
    //string nextToken;
    infile >> nextToken;

    if(nextToken == "End")
    {
      // create a final joint
      SMRJoint* currentJoint(new SMRJoint(true));

      double endOffsetX,endOffsetY,endOffsetZ;

      expect("{", infile);
      expect("OFFSET", infile);

      infile >> endOffsetX;
      infile >> endOffsetY;
      infile >> endOffsetZ;

      // set name
      currentJoint->setName(nodeName);
      // set parent name
      currentJoint->setParentName(parentStack[parentStack.size()-1]);
      // set position
      currentJoint->setPosition(SMRVector3(offsetX,offsetY,offsetZ));
      // set orientation
      currentJoint->setOrientation(id);
      // set end length
      currentJoint->setEndLength(SMRVector3(endOffsetX, endOffsetY, endOffsetZ));

      bindPose.insertJoint(currentJoint);

      expect("}", infile);
      infile >> nextToken;
      infile >> nextToken;
      while (nextToken=="}")
      {
        // pop the top of the stack of the parent name stack
        parentStack.pop_back();
        infile >> nextToken;
      }
      if (nextToken=="MOTION") break;
      infile >> nextToken;


    }
    else
    {
      // this is a standard joint
      SMRJoint* currentJoint(new SMRJoint);
      // set name
      currentJoint->setName(nodeName);
      // set parent name
      currentJoint->setParentName(parentStack[parentStack.size()-1]);
      // set position
      currentJoint->setPosition(SMRVector3(offsetX,offsetY,offsetZ));
      // set orientation
      currentJoint->setOrientation(id);
      bindPose.insertJoint(currentJoint);
      infile >> nextToken;
      if (nextToken == "}")
      {
        do
        {
          // pop the top of the stack of the parent name stack
          parentStack.pop_back();
          infile >> nextToken;
        }while (nextToken=="}");
        if (nextToken=="MOTION") break;
        infile >> nextToken;
      }
      else
      {
        // put it on top of the stack of the parent name stack
        parentStack.push_back(nodeName);
      }
    }
  }
  while (nextToken != "MOTION");

//#ifdef TRACE
  cout << "Loading motion from file : " << _filename << endl;
//#endif
  // instanciate a new motion
  SMRMotion motion;

  expect("Frames:", infile);
  unsigned int numFrames;
  infile >> numFrames;
//#ifdef TRACE
  cout << "Number of frames in motion: " << numFrames << endl;
//#endif

  expect("Frame", infile);
  expect("Time:", infile);
  //string nextToken;
  double alpha;
  infile >> alpha;

  motion.setTimeStep(alpha);

  vector<string>::iterator descValueIterator;
  SMRSkeleton skeleton(RELATIVEMODE,TRANSLATIONFIRST);
  SMRJoint* currentJoint;

  double value =0.0;
  double values[3]={0.0,0.0,0.0};
  string jointName = "";
  string prevJointName = "";
  SMRQuaternion quaternion;
  quaternion.identity();
  bool transjoint = false;

  for (unsigned int frame = 0 ; frame < numFrames ; frame++ )
  {
    //cout << "frame : " << frame << endl;
    skeleton = bindPose;
    for (descValueIterator=m_valueDescVect.begin(); descValueIterator < m_valueDescVect.end(); descValueIterator++)
    {

      //cout << "token lu : " <<value << endl;
      int npos = static_cast<int>((*descValueIterator).find('+',0));
      string jointName = (*descValueIterator).substr(0,npos);
      string jointType = (*descValueIterator).substr(npos+1);
      if (jointName != prevJointName && prevJointName != "")
      {
        currentJoint = skeleton.getJointByName(prevJointName);
        if (currentJoint)
        {
          //  cout << "Joint : " << jointName << " . Position :  "
          //    << SMRVector3(values[0],values[1],values[2]) << " . Quaternion :" << quaternion << endl;
          quaternion.normalize();
          currentJoint->setOrientation(quaternion);
          if (transjoint)currentJoint->setPosition(SMRVector3(values[0],values[1],values[2])); //currentJoint->getPosition() +
          values[0] = 0.0;values[1] = 0.0;values[2] = 0.0;
          quaternion.identity();
          transjoint = false;
        }
        else cout << "Problems on" << jointName << endl;
      }
      if (jointType != "")
      {
        infile >> value;

        if (jointType=="Xposition") {values[0]=value; transjoint = true;}
        if (jointType=="Yposition") {values[1]=value; transjoint = true;}
        if (jointType=="Zposition") {values[2]=value; transjoint = true;}
        if (jointType=="Xrotation")
        {
          SMRQuaternion rotX(SMRVector3(1.0,0.0,0.0),value*2*M_PI/360.0);
          quaternion = quaternion * rotX;
        }
        if (jointType=="Yrotation")
        {
          SMRQuaternion rotY(SMRVector3(0.0,1.0,0.0),value*2*M_PI/360.0);
          quaternion = quaternion * rotY;
        }
        if (jointType=="Zrotation")
        {
          SMRQuaternion rotZ(SMRVector3(0.0,0.0,1.0),value*2*M_PI/360.0);
          quaternion = quaternion * rotZ;
        }
      }
      else
      {
        //cout << *descValueIterator << endl;
      }
      prevJointName = jointName;
    }
    skeleton.checkEndJoints();
    
    skeleton.setRotationOrder(_type);//skeleton.setRotationOrder(ROTATIONFIRST);
    skeleton.checkEndJoints();
    motion.insertSkeleton( skeleton );
  }
  infile.close();
  return motion;
}