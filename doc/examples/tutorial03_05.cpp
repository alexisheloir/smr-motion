void Tutorial03::updateTentacle()
{
  //set of floats storing Horde's tentacle joints state
  float tx,ty,tz,rx,ry,rz,foo;
  double rrx,rry,rrz;

  //tentacle bone node(joint)
  HordeNode tentacleBone;

  // and its corresponding node in Smr
  SMRKinematicJoint *ikJoint;

  //get the transformation parameters from the SmrTentacle
  for(unsigned int i=0;i<m_kinematicChain->getNumJoints()-1;i++)
  {
    ikJoint=m_kinematicChain->getJoint(i);

    //get Horde tentacle's corresponding joint joint (Horde) 
    tentacleBone.findJoint(ikJoint->getName().c_str());
    tentacleBone.getTransform(&tx,&ty,&tz,&rx,&ry,&rz);

    //put the rotation values into euler angles (in degrees)
    ikJoint->getOrientation().toEulerAngles(rrx,rry,rrz);

    //update tentacle's joint according to Smr kinematic chain equivalent
    tentacleBone.setTransform(tx,ty,tz,SMRUtils::radToDeg(((float)(rrx))),SMRUtils::radToDeg(((float)(rry))),SMRUtils::radToDeg(((float)(rrz))));
  }
  // orientate Horde tentacles root joint correctly
  ikJoint=m_kinematicChain->getJoint(0);
  tentacleBone.findJoint(ikJoint->getName().c_str());

  tentacleBone.getTransform(&tx,&ty,&tz,&rx,&ry,&foo);
  tentacleBone.setTransform(tx,ty,tz,rx,ry,90);
}