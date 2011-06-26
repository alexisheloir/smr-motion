/**
*  \ingroup SmrCore
*  \file SMRIKConstraint.h
*/
#include "SmrIKSolver.h"

#ifndef SMRIKCONSTRAINT_H
#define SMRIKCONSTRAINT_H

/**
* \class SMRIKSolver
* \brief This class represents a basic constraint for Inverse kinematics.
*/
class SMRIKConstraint
{
public:
  /**
   *  \brief Constructor.
   */
  SMRIKConstraint();

  /**
   *  \brief Constructor.
   */
  SMRIKConstraint(SMRVector3 offset, SMRVector3 position, string relatedJointName="");

  /**
   *  \brief \brief .
   */
  const string getRelatedJointName()
  {
    return m_relatedJointName;
  }

  /**
   *  \brief Destructor.
   */
  ~SMRIKConstraint();

  /**
   *  \brief \brief .
   */
  inline void setOffset(const SMRVector3 offset)
  {
    m_offset = offset;
  }

  /**
   *  \brief \brief .
   */
  inline SMRVector3 getOffset() const
  {
    return m_offset;
  }

  /**
   *  \brief \brief .
   */
  inline void setPosition(const SMRVector3 position)
  {
    m_position = position;
  }

  /**
   *  \brief \brief .
   */
  inline SMRVector3 getPosition() const
  {
    return m_position;
  }

  /**
   *  \brief \brief .
   */
  inline void setRelatedJointName(const char* relatedJointName)
  {
    m_relatedJointName = relatedJointName;
  }

  inline string getJointTarget()
  {
    return m_targetJointName;
  }

  inline void setJointTarget(const char* _jointTargetName, const SMRVector3 _targetOffset)
  {
    m_targetJointName = _jointTargetName;
    m_position = _targetOffset;
  }

  //inline bool isDynamic()
  //{
  //  return m_isDynamic;
  //}

private:
  /**
   *  \brief \brief .
   */
  SMRVector3 m_offset;

  /**
   *  \brief \brief .
   */
  string m_relatedJointName;

  /**
   *  \brief \brief .
   */
  string m_targetJointName;

  /**
   *  \brief \brief .
   */
  SMRVector3 m_position;

  //bool m_isDynamic;
};

#endif
