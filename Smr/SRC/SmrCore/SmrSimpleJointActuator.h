/**
 *  \ingroup SmrCore
 *  \file SMRSimpleJointActuator.h
 */

#ifndef SMRSIMPLEJOINTACTUATOR_H
#define SMRSIMPLEJOINTACTUATOR_H
#pragma once

#include "SmrSTL.h"
#include "SmrMath.h"
#include "SmrActuator.h"

/**
 *  \class SMRSimpleJointActuator
 *  \brief This class is a simple type of Actuator that allows
 *  you to control a joint in a given skeleton. The user has to specify 
 *  before processing a frame:
 *  - a destination skeleton
 *  - the name of the joint to control
 *  The user may also give as an alternative way of controlling the joint 
 *  a positional or rotational velocity. In this case, a simple euler
 *  integration is performed according to a given timestep
 */
class SMRSimpleJointActuator : public SMRActuator
{
  /**
   *  \brief Actuator control mode
   */
  typedef enum {direct, velocity} SMRSimpleJointActuatorControlMode;

  /**
   *  \brief Control mode
   */
  SMRSimpleJointActuatorControlMode m_mode;

public:
  /**
   *  \brief Default constructor.
   */
  SMRSimpleJointActuator();

  /**
   *  \brief Destructor.
   */
  virtual ~SMRSimpleJointActuator()
  {

  }

public:
    /**
   *  \brief Processes.
   *
   *  SMRSimpleJointActuator is an Actuator and has to define a process() mehod
   *  that updates the skeleton
   */
  void process(float relativeTime=0.0f);

  /**
   *  \brief Sets the joint to control.
   *
   *  this set of methods allows to control the actuator
   */
  inline void setJointToControl(const string & jointName)
  {
    m_jointName = jointName;
  }

  /**
   *  \brief Sets the skeleton to control.
   */
  inline void setSkeletonToControl(const SMRSkeleton & skeleton)
  {
    m_skeleton = skeleton;
  }

  /**
   *  \brief Sets the time step.
   */
  inline void setTimeStep(double timeStep)
  {
    m_timeStep = timeStep;
  }
  
  /**
   *  \brief Sets the position.
   */
  void setPosition(SMRVector3 position);

  /**
   *  \brief Sets the rotation.
   */
  void setRotation(SMRQuaternion rot);

  /**
   *  \brief Sets the translational velocity.
   */
  void setTranslationalVelocity(SMRVector3 trans);

  /**
   *  \brief Sets the rotational velocity.
   */
  void setRotationalVelocity(SMRQuaternion rot);

  /**
   *  \brief Stops the translation.
   *
   *  stop motion/positionning if needed
   */
  void stopTranslation()
  {
    m_positionFlag = false;
  }

  /**
   *  \brief Stops the rotation.
   */
  void stopRotation()
  {
    m_rotationFlag = false;
  }

  /**
   *  \brief Stops all.
   */
  void stopAll()
  {
    m_positionFlag = false;
    m_rotationFlag = false;
  }

  /**
   *  \brief Gets the skeleton.
   */
  SMRSkeleton* getSkeleton(float attenuationFactor)
  {
    return & m_skeleton;
  }

protected:
  /**
   *  \brief Skeleton.
   */
  SMRSkeleton m_skeleton;

private:
  /**
   *  \brief The name of the controlled joint.
   */
  string m_jointName;

  /**
   *  \brief \brief .
   */
  bool m_positionFlag;

  /**
   *  \brief \brief .
   */
  bool m_rotationFlag;

  /**
   *  \brief \brief .
   *
   *  position control
   */
  SMRVector3 m_position;

  /**
   *  \brief \brief .
   *
   *  rotation control
   */
  SMRQuaternion m_rotation;

  /**
   *  \brief \brief .
   *
   *  the translationnal velocity if any
   */
  SMRVector3 m_translationalVelocity;

  /**
   *  \brief \brief .
   *
   *  the rotationnal velocity if any
   */
  SMRQuaternion m_rotationalVelocity;

  /**
   *  \brief \brief .
   *
   *  timestep
   */
  double m_timeStep;
};

#endif
