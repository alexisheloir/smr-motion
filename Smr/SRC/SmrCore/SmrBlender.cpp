/*
 *  SMRBlender.cpp
 *  SmrCore
 *
 *  Created by Alexis Heloir on 19/10/05.
 *  Copyright 2005 __MyCompanyName__. All rights reserved.
 *
 */

#include "SmrBlender.h"

void SMRBlender::addPosePtr(SMRSkeleton* _newPose)
{
  m_posesVector.push_back(_newPose);
}

void SMRBlender::removePosePtr(int _poseToRemodeInd)
{
  //m_posesVector.??
}

void SMRBlender::removePosePtr(SMRSkeleton* _poseToRemove)
{
  vector<SMRSkeleton*>::iterator pose;
  for (pose = m_posesVector.begin(); pose < m_posesVector.end(); pose++)
  {
    if (*pose == _poseToRemove) m_posesVector.erase(pose); 
  }
}

SMRSkeleton* SMRBlender::getBlendedPose()
{
return &m_blendedSkeleton;
}
