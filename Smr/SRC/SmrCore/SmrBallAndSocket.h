/**
 *  \ingroup SmrCore
 *  \file SMRBallAndSocket.h
 */
#ifndef SMRBALLANDSOCKET_H
#define SMRBALLANDSOCKET_H

#include "SmrDof.h"
#include "SmrJoint.h"

/**
 *  \class SMRBallAndSocket
 *  \brief Ball and socket joint.
 */
class SMRBallAndSocket : public SMRJoint
{
public:
  /**
   *  \brief Constructor.
   */
  SMRBallAndSocket(SMRDOF::SMRRotationAxisType firstAxis, SMRDOF::SMRRotationAxisType secondAxis);

public:
  /**
   *  \brief Sets the upper twist limit.
   */
  inline void setUpperTwistLimit(const double upperTwistLimit)
  {
    m_upperTwistLimit = upperTwistLimit;
  }

  /**
   *  \brief Sets the lower twist limit.
   */
  inline void setLowerTwistLimit(const double lowerTwistLimit)
  {
    m_lowerTwistLimit = lowerTwistLimit;
  }

  /**
   *  \brief Sets the plan angle.
   */
  inline void setPlanAngle(double planAngle)
  {
    m_planAngle = planAngle;
  }

  /**
   *  \brief Sets the swing angle.
   */
  inline void setSwingAngle(double swingAngle)
  {
    m_swingAngle = swingAngle;
  }

  /**
   *  \brief Sets the twist angle.
   */
  inline void setTwistAngle(double twistAngle)
  {
    m_twistAngle = twistAngle;
  }

  /**
   *  \brief Returns the plan angle.
   */
  inline const double getPlanAngle() const
  {
    return m_planAngle;
  }

  /**
   *  \brief Returns the swing angle.
   */
  inline const double getSwingAngle() const
  {
    return m_swingAngle;
  }

  /**
   *  \brief Returns the twist angle.
   */
  inline const double getTwistAngle() const
  {
    return m_twistAngle;
  }

  /**
   *  \brief Updates the rotation.
   */
  void updateRotation();

private:
  /**
   *  \brief Swing value.
   */
  SMRQuaternion m_swing;

  /**
   *  \brief Twist value.
   */
  SMRQuaternion m_twist;

  /**
   *  \brief First axis plane component value.
   */
  SMRVector3 m_firstAxisPlaneComponent;

  /**
   *  \brief Second axis plane component value.
   */
  SMRVector3 m_secondAxisPlaneComponent;

  /**
   *  \brief Swing angle.
   */
  double m_swingAngle;

  /**
   *  \brief Twist angle.
   */
  double m_twistAngle;

  /**
   *  \brief Plan angle.
   */
  double m_planAngle;

  /**
   *  \brief Upper twist limit.
   */
  double m_upperTwistLimit;

  /**
   *  \brief Lower twist limit.
   */
  double m_lowerTwistLimit;
};

#endif

