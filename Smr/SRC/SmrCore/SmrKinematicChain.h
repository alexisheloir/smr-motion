/**
*  \ingroup SmrCore
*  \file SMRKinematicChain.h
*/
#ifndef SMRKINEMATICCHAIN_H
#define SMRKINEMATICCHAIN_H

#include "SmrKinematicJoint.h"
#include "SmrSkeleton.h"

/**
*  \class SMRKinematicChain
*  \brief This class is an inverse kinematic dedicated specialization of a Skeleton
*/
class SMRKinematicChain : public SMRSkeletonT<SMRKinematicJoint>
{
public:
  /**
  *  \brief Default constructor.
  */
  SMRKinematicChain():SMRSkeletonT<SMRKinematicJoint>(RELATIVEMODE,ROTATIONFIRST,"")
  {
    m_startJointIndex = 0;
  }

  ~SMRKinematicChain();

  /**
  *  \brief Parametrized constructor.
  *  \param mode is it RELATIVEMODE or ABSOLUTEMODE ?
  *  \param rotationOrder is it ROTATIONFIRST or TRANSLATIONFIRST ?
  *  \param name name for the chain
  */
  SMRKinematicChain(SMRModeType mode, SMRTransformationOrderType rotationOrder, string name=""):SMRSkeletonT<SMRKinematicJoint>(mode,rotationOrder,name)
  {
    m_startJointIndex = 0;
  }

  /**
  *  \brief \brief 
  *
  *  Replacement of the previous constructor except we don't use typeDefs for _mode and _rotationOrder
  *  (Because we could not figure out how to expose typeDefs in python)
  *  \param mode is it RELATIVEMODE or ABSOLUTEMODE ?
  *  \param rotationOrder is it ROTATIONFIRST or TRANSLATIONFIRST ?
  *  \param name name for the chain
  */
  SMRKinematicChain(bool mode, bool rotationOrder, string name=""):SMRSkeletonT<SMRKinematicJoint>(mode,rotationOrder,name)
  {
    m_startJointIndex = 0;
  }

  /**
  *  \brief \brief 
  *
  *  instanciate a new kinematicChain from a skeleton
  *  \param skeleton reference skeleton
  */
  SMRKinematicChain(const SMRSkeleton & skeleton);

  /**
  *  \brief \brief 
  *
  *  instanciate a new kinematicChain from a skeleton
  */
  SMRKinematicChain(const SMRKinematicChain & _chain);


  /**
  *  \brief \brief 
  *
  *  get the position of a joint specified using its index
  *  returns the position of the joint (absolute frame coordinate)
  */
  SMRVector3 getEndPosition(const unsigned int jointIndex) const;

  /**
  *  \brief \brief 
  *
  *  where shall IK start in the chain ?
  *  \param jointName name of the first IK joint
  */
  void setStartJointIndex(const string jointName);

  /**
  *  \brief \brief 
  *
  *  where shall IK start in the chain ?
  */
  inline void setStartJointIndexInt(unsigned int _index)
  {
    m_startJointIndex =_index;
  }

  /**
  *  \brief \brief 
  *
  *  accessor towards the first IK joint in the chain
  *  \returns index of the first IK joint
  */
  inline unsigned int getStartJointIndex()  const
  {
    return m_startJointIndex;
  }

  /**
  *  \brief \brief 
  *
  *  set up each joint DOF according to a template SMRSkeleton
  *  \returns the umber of DOFs available
  */
  unsigned int getNumDOFs();

  /**
  *  \brief \brief 
  *
  *  set up each joint DOF according to a template SMRSkeleton
  *  \returns the umber of DOFs available
  */
  //void inferDOFSFromQuats(SMRSkeleton &skel);

  /**
  *  \brief \brief 
  *
  *  copy operator
  */
  SMRKinematicChain &
  operator=(const SMRKinematicChain & _skeleton);

private:

  /**
  *  \brief Start joint index.
  */
  unsigned int m_startJointIndex;
};

#endif
