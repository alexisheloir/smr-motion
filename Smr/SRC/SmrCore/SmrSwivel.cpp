//
// C++ Implementation: SMRSwivel
//
// Description: cf SMRSwivel.h
//
//
// Author: Alexis Heloir <alexis.heloir@dfki.de>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "SmrSwivel.h"

SMRSwivel::SMRSwivel()
{
  m_xAxis = SMRVector3(0.0,-1.0,0.0);
  m_upVectorEndJoint = m_upVectorStartJoint = m_startJoint = m_middleJoint = m_endJoint = NULL;
}

SMRSwivel::SMRSwivel(SMRKinematicChain *_chain, const string _startJointName, const string _endJointName, const string _middleJointName)
{
  m_upVectorEndJoint = m_upVectorStartJoint = m_startJoint = m_middleJoint = m_endJoint = NULL;

  m_xAxis = SMRVector3(0.0,-1.0,0.0);
  m_skeleton = _chain;

  m_startJoint = m_skeleton->getJointByName(_startJointName);
  m_endJoint = m_skeleton->getJointByName(_endJointName);
  m_middleJoint = m_skeleton->getJointByName(_middleJointName);

  if (m_startJoint == NULL || m_endJoint == NULL || m_middleJoint == NULL)
  {
    LOG_FATAL(logger, "either " + _startJointName + ", or " + _endJointName + " or " + _middleJointName + " are not present in Chain " + m_skeleton->getName() );
    exit(1);
  }
}

SMRSwivel::~SMRSwivel()
{
}

void SMRSwivel::process(float relativeTime)
{

  //m_startJoint->updateRotation();
  m_skeleton->setMode(ABSOLUTEMODE);

  if (m_upVectorStartJoint && m_upVectorEndJoint)
  {
    m_xAxis = (m_upVectorEndJoint->getPosition() - m_upVectorStartJoint->getPosition()).normalize();
  }

  SMRVector3 vectorSW = m_endJoint->getPosition() - m_startJoint->getPosition();

  SMRVector3 vectorSE = m_middleJoint->getPosition() - m_startJoint->getPosition();
  SMRVector3 swivelAxis = vectorSW.normalize();
  swivelAxis.normalize();
  SMRVector3 vectorSEproj = (DotProduct(vectorSE,vectorSW))*vectorSW;

  SMRVector3 swivelDir = -( vectorSE - vectorSEproj );
  //swivelDir = - vectorSE;

  swivelDir.normalize();

  SMRVector3 yAxis = (CrossProduct(swivelAxis,m_xAxis)).normalize();

  double xComponent = DotProduct(swivelDir,m_xAxis);
  double yComponent = DotProduct(swivelDir,yAxis);

  double actualAngle = atan2(yComponent,xComponent);

  SMRQuaternion rotation(swivelAxis,m_swivelAngle-actualAngle);

  SMRQuaternion currentRotation = m_startJoint->getOrientation();

  m_skeleton->setMode(RELATIVEMODE);

  //actualAngle = acos(DotProduct(hauteur,referenceOrientation));
  //if (abs(_swivelAngle-actualAngle) > 1.0E-6)
  //{

    rotation = (Inverse(currentRotation) * rotation) * currentRotation;

    //m_startJoint->updateRotation();

    SMRQuaternion jointRotation = m_startJoint->getOrientation();

    if (DotProduct(jointRotation, rotation) < 0.0 )
    {
      rotation = -rotation;
    }

    rotation = jointRotation * rotation;
    rotation.normalize();

    //m_startJoint->setOrientation(rotation);

    double x,y,z;
    rotation.toEulerAnglesXZY( x, y, z );

    m_startJoint->getDOF(1)->setRotationAngle(x);
    m_startJoint->getDOF(3)->setRotationAngle(z);
    m_startJoint->getDOF(4)->setRotationAngle(y);

    m_startJoint->updateRotation();
  //}
}

SMRSkeleton* SMRSwivel::getSkeleton(float attenuationFactor)
{
  return reinterpret_cast<SMRSkeleton*>(m_skeleton);
}

void SMRSwivel::setKinematicChain(SMRKinematicChain &_chain, const string _startJointName, const string _endJointName, const string _middleJointName)
{
  m_skeleton = &_chain;
  m_startJoint = m_skeleton->getJointByName(_startJointName);
  m_endJoint = m_skeleton->getJointByName(_endJointName);
  m_middleJoint = m_skeleton->getJointByName(_middleJointName);

  m_startJoint = m_skeleton->getJointByName(_startJointName);
  m_endJoint = m_skeleton->getJointByName(_endJointName);
  m_middleJoint = m_skeleton->getJointByName(_middleJointName);

  if (m_startJoint == NULL || m_endJoint == NULL || m_middleJoint == NULL)
  {
    LOG_FATAL(logger, "Either " + _startJointName + ", or " + _endJointName + " or " + _middleJointName + " are not present in Chain " + m_skeleton->getName() );
    exit(1);
  }

}

void SMRSwivel::setUpDirection(const string _joint1, const string _joint2)
{
  m_upVectorStartJoint = m_skeleton->getJointByName(_joint1);
  m_upVectorEndJoint   = m_skeleton->getJointByName(_joint2);
  if (m_upVectorStartJoint == NULL || m_upVectorEndJoint == NULL)
  {
    LOG_FATAL(logger, "Either " + _joint1 + ", or " + _joint2 + " are not present in Chain " + m_skeleton->getName() );
    exit(1);
  }
}
