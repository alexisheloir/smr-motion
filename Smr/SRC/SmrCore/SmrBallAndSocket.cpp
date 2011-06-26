#include "SmrBallAndSocket.h"

SMRBallAndSocket::SMRBallAndSocket(SMRDOF::SMRRotationAxisType _firstAxis, SMRDOF::SMRRotationAxisType _secondAxis)
{

  switch(_firstAxis){
    case 0 :
      m_firstAxisPlaneComponent = SMRVector3(1.0f,0.0f,0.0f);
      break;
    case 1 :
      m_firstAxisPlaneComponent = SMRVector3(1.0f,0.0f,0.0f);
      break;
    case 2 :
      m_firstAxisPlaneComponent = SMRVector3(0.0f,0.0f,1.0f);
      break;
	default :
	  m_firstAxisPlaneComponent = SMRVector3(1.0f,0.0f,0.0f);
	  break;	  
  }

  switch(_secondAxis){
    case 0 :
      m_secondAxisPlaneComponent = SMRVector3(1.0f,0.0f,0.0f);
    break;
    case 1 :
      m_secondAxisPlaneComponent = SMRVector3(1.0f,0.0f,0.0f);
    break;
    case 2 :
      m_secondAxisPlaneComponent = SMRVector3(0.0f,0.0f,1.0f);
    break;
    default :
	  m_secondAxisPlaneComponent = SMRVector3(1.0f,0.0f,0.0f);
	  break;	  
  }

  m_swingAngle = 0;
  m_twistAngle = 0;
  m_planAngle = 0;

  m_upperTwistLimit = 0;
  m_lowerTwistLimit = 0;

}

void SMRBallAndSocket::updateRotation()
{
//SMRVector3 axis = cos(m_planAngle)*m_firstAxisPlaneComponent + sin(m_planAngle)*m_secondAxisPlaneComponent;

//SMRQuaternion firstRot(axis,m_swingAngle);



//double inducedRot = firstRot.getRotationAngle();
//SMRVector3 rotationAxis = firstRot.

//SMRQuaternion rotation();

}
