//
// C++ Interface: smrkinematicjoint
//
// Description:
//
//
// Author: Alexis Heloir <alexis.heloir@univ-ubs.fr>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "SmrDof.h"

#define SPEED_LIMIT 0.2

SMRDOF::SMRDOF()
{
  m_quaternionRotation.identity();
  m_radRotation = 0.0f;
  m_axis = XAXIS;
  m_rotationAxis = SMRVector3(1.0,0.0,0.0);
  m_lowerBound = -2*M_PI;
  m_upperBound = 2*M_PI;
  m_gain = 5.0f;
}

SMRDOF::~SMRDOF()
{
}

void SMRDOF::setRotationAngle(const double _rotationAngle)
{
// SMRVector3 axis;
// switch(m_axis){
// case 0 :
// axis = SMRVector3(1.0f,0.0f,0.0f);
// break;
// case 1 :
// axis = SMRVector3(0.0f,1.0f,0.0f);
// break;
// case 2 :
// axis = SMRVector3(0.0f,0.0f,1.0f);
// break;
// }
//double difference = (_rotationAngle - m_radRotation)*(_rotationAngle - m_radRotation);
if( (_rotationAngle > m_lowerBound) && (_rotationAngle < m_upperBound) )
{
  m_radRotation = _rotationAngle;
  m_quaternionRotation = SMRQuaternion(m_rotationAxis,_rotationAngle);
}else if(_rotationAngle < m_lowerBound)
{
  LOG_DEBUG(logger,"rotation angle exceeds joints limit" << _rotationAngle << " < " << m_lowerBound );
  m_radRotation = m_lowerBound;
  m_quaternionRotation = SMRQuaternion(m_rotationAxis,m_lowerBound);
}else if (_rotationAngle > m_upperBound)
{
  LOG_DEBUG(logger,"rotation angle exceeds joints limit" << _rotationAngle << " > " << m_upperBound );
  m_radRotation = m_upperBound;
  m_quaternionRotation = SMRQuaternion(m_rotationAxis,m_upperBound);
}

//m_parentJointPtr->updateRotation();
}
