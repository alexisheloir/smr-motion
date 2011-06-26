void Tutorial01::updateKinematicChain()
{
  SMRKinematicJoint *joint;

  //rotate first joint
  joint=m_kinematicChain->getJoint(0);
  joint->setOrientation(SMRQuaternion(SMRVector3(1,0,0),sinf(m_time)));

  //rotate second joint
  joint=m_kinematicChain->getJoint(1);
  joint->setOrientation(SMRQuaternion(SMRVector3(1,0,0),sinf(m_time)));

  //rotate third joint
  joint=m_kinematicChain->getJoint(2);
  joint->setOrientation(SMRQuaternion(SMRVector3(1,0,0),sinf(m_time)));
}