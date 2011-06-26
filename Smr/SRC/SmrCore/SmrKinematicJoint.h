/**
 *  \ingroup SmrCore
 *  \file SMRKinematicJoint.h
 */
#ifndef SMRKINEMATICJOINT_H
#define SMRKINEMATICJOINT_H
//#pragma once

#include "SmrJoint.h"
#include "SmrDof.h"

/**
 *  \class SMRKinematicJoint
 *  \brief This class represent an inverse kinematic dedicated specialization of a SMRJoint.
 */
class SMRKinematicJoint : public SMRJoint
{
public:
  /**
   *  \brief Constructor.
   *  \param endJoint [in] Specifies whether the joint is an end joint or not.
   */
  SMRKinematicJoint(bool endJoint=false);

  /**
   *  \brief Copy constructor.
   *  \param joint [in] Source joint to copy from.
   */
  SMRKinematicJoint(const SMRJoint & joint);

  /**
   *  \brief Copy constructor.
   *  \param joint [in] Source joint to copy from.
   */
  SMRKinematicJoint(SMRKinematicJoint & joint);

  /**
   *  \brief Destructor.
   */
  ~SMRKinematicJoint();

  /**
   *  \brief Adds a DOF.
   */
  void addDOF(const SMRDOF::SMRRotationAxisType axis, const double lowerBound = -M_PI, const double upperBound = M_PI, const double value = 0, const double gain = 0.00);
  
  /**
   *  \brief Adds a DOF.
   */
  void addDOF(SMRVector3 customAxis, const double lowerBound = -M_PI, const double upperBound = M_PI, const double value = 0, const double gain = 0.00);
  
  /**
   *  \brief Updates the rotation.
   */
  void updateRotation();

  /**
   *  \brief Gets the number of DOFs.
   */
  inline unsigned int getNumDOFs() const
  {
    return m_dofVector.size();
  }

  /**
   *  \brief Checks for a twist.
   */
  void checkTwist();


  /**
   *  \brief Gets a DOF.
   */
  SMRDOF * getDOF(const int index)
  {
    return &(m_dofVector.at(index));
  }

  /**
   *  \brief Set up the DOF according to argument joint global orientation.
   */
  void inferDOFFromQuat(const SMRJoint &joint);

protected:
  /**
   *  \brief DOF vector.
   */
  std::vector<SMRDOF> m_dofVector;

};

/**
 *  \brief Pointer to a kinematic joint.
 */
typedef SMRKinematicJoint * ptrSmrKinematicJoint;

#endif
