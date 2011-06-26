void Tutorial03::createKinematicChain()
{
  float tx,ty,tz,rx,ry,rz;
  //currentJoint and previous joint referencing for relevant parenting when creating tentacles joints
  SMRKinematicJoint *currentJoint,*prevJoint;
  prevJoint=NULL;
  //instanciate an empty kinematic chain
  m_kinematicChain=new SMRKinematicChain(RELATIVEMODE,TRANSLATIONFIRST,"tentacle");

  //get Hordes m_kinematicChain first joint
  HordeNode firstNode=m_tentacleNode.getFirstChild();
  //will help computing Smr joints gain 
  int i=0;
  //while Horde tentacles joints are found
  while(firstNode.isValid())
  {
    //instanciate a new SMRKinematicJoint
    currentJoint=new SMRKinematicJoint();
    //get joint translation parameters in parents local frame (bone length)
    firstNode.getTransform(&tx,&ty,&tz,&rx,&ry,&rz);
    //add DOF along X axis
    currentJoint->addDOF(SMRDOF::XAXIS,-M_PI/16.0f,M_PI/16.0f,SMRUtils::degToRad(rx));
    //add DOF along Y axis
    currentJoint->addDOF(SMRDOF::YAXIS,-M_PI/3.0f,M_PI/3.0f,SMRUtils::degToRad(ry));
    //add DOF along Z axis
    currentJoint->addDOF(SMRDOF::ZAXIS,-M_PI/3.0f,M_PI/3.0f,SMRUtils::degToRad(rz));
    //set up joint gain (the more distal,the more gain)
    currentJoint->getDOF(0)->setGain(0.015*i);
    currentJoint->getDOF(1)->setGain(0.015*i);
    currentJoint->getDOF(2)->setGain(0.015*i);

    //give a name to the joint
    currentJoint->setName(firstNode.getName());
    //set up joint translation parameters in local frame (bone length)
    currentJoint->setPosition(tx,ty,tz);
    //take care of parenting issues
    if(prevJoint) 
    {
      currentJoint->setParentName(prevJoint->getName());
    }
    //and add the brand new joint into the inverse kinematics chain !
    m_kinematicChain->insertJoint(currentJoint);
    //keep a reference of the newly created joint for the next turn (parenting issue)
    prevJoint=currentJoint;
    //get the next joint according to Horde structure
    firstNode=firstNode.getFirstChild();
    //increase the gain increment
    i++;
  }
  m_kinematicChain->setStartJointIndex(m_kinematicChain->getJoint(0)->getName());

  //force the root joint not to move (tentacle basis should be fix)
  m_kinematicChain->getJoint(0)->getDOF(0)->setUpperBound(0);
  m_kinematicChain->getJoint(0)->getDOF(0)->setLowerBound(0);
  m_kinematicChain->getJoint(0)->getDOF(1)->setUpperBound(0);
  m_kinematicChain->getJoint(0)->getDOF(1)->setLowerBound(0);
  m_kinematicChain->getJoint(0)->getDOF(2)->setUpperBound(0);
  m_kinematicChain->getJoint(0)->getDOF(2)->setLowerBound(0);

  //and add the fly constraint (relative to tentacle's tip) to the tentacle
  m_ikConstraint=new SMRIKConstraint(SMRVector3(0,0,0),SMRVector3(0,0,0),currentJoint->getName());
}