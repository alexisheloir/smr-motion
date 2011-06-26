/**
 *  \ingroup SmrCore
 *  \file SMRDOF.h
 */
#ifndef SMRDOF_H
#define SMRDOF_H

// -- STL related stuff
#include "SmrSTL.h"
#include "logger.h"

// quaternion support
#include "SmrMath.h"
#include "SmrQuaternion.h"

/**
 *  \class SMRDOF
 *  \brief Degree-of-freedom.
 */
class SMRDOF
{
public:
  /**
   *  \brief Rotation axis type.
   */
  typedef enum {XAXIS,YAXIS,ZAXIS,CUSTOM} SMRRotationAxisType;

  /**
   *  \brief Constructor.
   */
  SMRDOF();

  /**
   *  \brief Destructor.
   */
  ~SMRDOF();

  /**
   *  \brief Sets the upper bound.
   */
  inline void setUpperBound(const double upperBound)
  {
    m_upperBound = upperBound;
  }

  /**
   *  \brief Sets the lower bound.
   */
  inline void setLowerBound(const double lowerBound)
  {
    m_lowerBound = lowerBound;
  }

  /**
   *  \brief Sets the upper bound.
   */
  inline double getUpperBound(void) const
  {
    return m_upperBound;
  }

  /**
   *  \brief Sets the lower bound.
   */
  inline double getLowerBound(void) const
  {
    return m_lowerBound;
  }


  /**
   *  \brief Returns the mean bound.
   */
  inline double getMeanBound() const
  {
    return (fabs( (m_upperBound) - (m_lowerBound) / 2.0));
  }

  /**
   *  \brief Returns the central angle.
   */
  inline double getCentralAngle() const
  {
    return ((m_lowerBound) + getMeanBound());
  }

  /**
   *  \brief Sets the gain.
   */
  inline void setGain(const double gain)
  {
    m_gain = gain;
  }

  /**
   *  \brief Gets the gain.
   */
  inline double getGain()const
  {
    return m_gain;
  }

  inline SMRVector3 getAxis()
  {
    return m_rotationAxis;
  }

  /**
   *  \brief Sets the rotation axis.
   */
  inline void setRotationAxis(const SMRRotationAxisType axis)
  {
    m_axis = axis;
    switch (axis)
    {
      case 0 :
        m_rotationAxis = SMRVector3(1.0,0.0,0.0);
        break;
      case 1 :
        m_rotationAxis = SMRVector3(0.0,1.0,0.0);
        break;
      case 2 :
        m_rotationAxis = SMRVector3(0.0,0.0,1.0);
        break;
	  default :
		m_rotationAxis = SMRVector3(1.0,0.0,0.0);
		break;	
     }
  }

  /**
   *  \brief Sets the rotation axis.
   */
  inline void setRotationAxis(const SMRVector3 axis)
  {
    m_axis = SMRDOF::CUSTOM, m_rotationAxis = axis;
  }

  /**
   *  \brief Sets the rotation angle.
   */
  void setRotationAngle(const double rotationAngle);

  /**
   *  \brief Returns the rotation angle.
   */
  inline double getRotationAngle()
  {
    return m_radRotation;
  }

  inline SMRVector3 getRotationAxis()
  {
    return m_rotationAxis;
  }

  /**
   *  \brief Returns the quaternion rotation.
   */
  inline SMRQuaternion getQuaternionRotation()
  {
    return m_quaternionRotation;
  }

  /**
   *  \brief Sets the joint name.
   */
  void setJointName(string name)
  {
    m_jointName = name;
  }

protected:
  /**
   *  \brief Gain.
   */
  double m_gain;

  /**
   *  \brief Quaternion rotation.
   */
  SMRQuaternion m_quaternionRotation;

  /**
   *  \brief Rad rotation
   */
  double m_radRotation;
  
  /**
   *  \brief Upper bound.
   */
  double m_upperBound;
  
  /**
   *  \brief Lower bound.
   */
  double m_lowerBound;

  /**
   *  \brief Axis.
   */
  SMRRotationAxisType m_axis;

  /**
   *  \brief Joint name.
   */
  string m_jointName;

  /**
   *  \brief Rotation axis.
   */
  SMRVector3 m_rotationAxis;
};

typedef SMRDOF * ptrSmrDof;

#endif
