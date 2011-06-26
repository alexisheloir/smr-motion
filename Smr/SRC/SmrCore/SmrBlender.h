/**
 *  \ingroup SmrCore
 *  \file SMRBlender.h
 */
#ifndef SMRBLENDER_H
#define SMRBLENDER_H

#include "SmrSkeleton.h"

/**
 *  \class SMRBlender
 *  \brief This abstract class is in charge of storing
 *  a reference on several skeletons and to merge
 *  them according to specific rules
 */
class SMRBlender
{
public:
  virtual ~SMRBlender()=0;
  /**
   *  \brief Pure virtual blend function.
   */
  virtual void blend()=0;

  /**
   *  \brief Adds a pose pointer.
   */
  void addPosePtr(SMRSkeleton* newPose);

  /**
   *  \brief Removes a pose pointer.
   */
  void removePosePtr(int poseToRemodeInd);

  /**
   *  \brief Removes a pose pointer.
   */
  void removePosePtr(SMRSkeleton* poseToRemove);

  /**
   *  \brief Gets blended pose.
   */
  SMRSkeleton* getBlendedPose();
private:
  
protected:
  /**
   *  \brief Pose vector.
   */
  vector<SMRSkeleton*> m_posesVector;

  /**
   *  \brief Blended skeleton.
   */
  SMRSkeleton m_blendedSkeleton;
};

#endif

