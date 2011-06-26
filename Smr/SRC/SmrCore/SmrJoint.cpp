#include "SmrJoint.h"

SMRJoint::SMRJoint(bool endJoint)
{
  m_position = m_endLength = SMRVector3(0,0,0);
  m_orientation.identity();
  m_name = "Empty Joint";
  m_parent = "";
  m_endJoint = endJoint;
}

SMRJoint::SMRJoint(const SMRJoint & joint)
{
  m_position = joint.getPosition();
  m_orientation = joint.getOrientation();
  m_endLength = joint.getEndLength();  
  m_name = joint.getName();
  m_parent = joint.getParentName();
  m_endJoint = joint.isEndJoint();
}

SMRJoint::~ SMRJoint()
{

}

void SMRJoint::setPosition(double x, double y, double z)
{
  m_position.m_x=x;
  m_position.m_y=y;
  m_position.m_z=z;
}

void SMRJoint::setPosition(const SMRVector3 position)
{
  m_position=position;
}

void SMRJoint::setOrientation(double x, double y, double z)
{
  m_orientation = SMRQuaternion(x,y,z);
  m_orientation.normalize();
}

void SMRJoint::setOrientation(SMRQuaternion orientation)
{
  m_orientation = orientation;
}
