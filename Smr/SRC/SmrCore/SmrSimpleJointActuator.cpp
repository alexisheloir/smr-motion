#include "SmrSimpleJointActuator.h"

SMRSimpleJointActuator::SMRSimpleJointActuator()  
{
  m_mode = SMRSimpleJointActuator::direct;
  m_positionFlag = false; m_rotationFlag = false;
  m_jointName = "";
  m_position = SMRVector3(0,0,0);
  m_rotation = SMRQuaternion(1,0,0,0);
  m_timeStep = 0.1;
}

void 
SMRSimpleJointActuator::process(float relativeTime)
{
  SMRJoint* joint = m_skeleton.getJointByName(m_jointName);
  if (joint)
  {  
    // the skeleton has to be in relative system
    m_skeleton.setMode(RELATIVEMODE);

    // the two modes considered in the actuator
    if (m_mode==SMRSimpleJointActuator::direct)
    {
      if (m_positionFlag) joint->setPosition( m_position );
      if (m_rotationFlag) joint->setOrientation( m_rotation );
    }
    else if (m_mode==SMRSimpleJointActuator::velocity)
    {
      if (m_positionFlag) 
      {  
        joint->setPosition( joint->getPosition() + m_position*m_timeStep);
      }
      if (m_rotationFlag) {
        SMRQuaternion q = m_timeStep * m_rotation;
        q.normalize();
        joint->setOrientation(q*joint->getOrientation());
      }
    }
  }
  else cerr << "Unknown joint in SMRSimpleJointActuator !!" << endl;
}

void 
SMRSimpleJointActuator::setPosition(SMRVector3 _position)
{
  m_mode = SMRSimpleJointActuator::direct;
  m_positionFlag = true;
  m_position = _position;
}

void 
SMRSimpleJointActuator::setRotation(SMRQuaternion _rot)
{
  m_mode = SMRSimpleJointActuator::direct;
  m_rotationFlag = true;
  m_rotation = _rot;
}

void 
SMRSimpleJointActuator::setTranslationalVelocity(SMRVector3 _trans)
{
  m_mode = SMRSimpleJointActuator::velocity;
  m_positionFlag = true;
  m_position = _trans;
}

void 
SMRSimpleJointActuator::setRotationalVelocity(SMRQuaternion _rot)
{
  m_mode = SMRSimpleJointActuator::velocity;
  m_rotationFlag = true;
  m_rotation = _rot;
}
