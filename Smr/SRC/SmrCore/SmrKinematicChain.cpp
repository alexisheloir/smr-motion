//
// C++ Implementation: smrkinematicjoint
//
// Description:
//
//
// Author: Alexis Heloir <alexis.heloir@univ-ubs.fr>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "SmrKinematicChain.h"

SMRKinematicChain::SMRKinematicChain(const SMRSkeleton & _skeleton):SMRSkeletonT<SMRKinematicJoint>(RELATIVEMODE,ROTATIONFIRST,"")
{
  m_startJointIndex=0;
  m_name = _skeleton.getName();
  this->m_rotationOrder = _skeleton.getRotationOrder();
  this->m_mode = _skeleton.getMode();
  std::vector<SMRJoint*>::const_iterator skelJointIterator;
  for ( unsigned int i=0; i < _skeleton.getNumJoints(); i++ )
  {
    this->insertJoint((new SMRKinematicJoint(*_skeleton.getJoint(i))));
  }
}

SMRKinematicChain::~SMRKinematicChain()
{
  for_each( (m_joints.begin())+m_startJointIndex, m_joints.end(), DeleteObjectPtr());
  m_joints.clear();
}

SMRKinematicChain::SMRKinematicChain(const SMRKinematicChain & _chain)
{
  *this = _chain;
}

void SMRKinematicChain::setStartJointIndex(const string _jointName)
{
  for(unsigned int i=0; i < this->getNumJoints(); i++ )
  {
    if ( this->getJoint(i)->getName() == _jointName )
    {
      m_startJointIndex = i;
    }
  }
} 

unsigned int SMRKinematicChain::getNumDOFs(void)
{
  unsigned int nDofs = 0;
  for( unsigned int i = m_startJointIndex; i < this->getNumJoints(); ++i )
  {
    SMRKinematicJoint* currentJoint = getJoint(i);
    for( unsigned int j = 0; j < currentJoint->getNumDOFs(); ++j )
    {
      ++nDofs;
    }
  }
  return nDofs;
}

SMRVector3
SMRKinematicChain::getEndPosition(const unsigned int _jointIndex) const
{
  SMRVector3 endPos;
  SMRKinematicJoint* joint;
  SMRQuaternion globalOrientation;
  globalOrientation.identity();
  for( unsigned int i = 0; i < _jointIndex; ++i )
  {
    joint = getJoint(i);
    endPos = joint->getPosition();
    if( strcmp(joint->getParentName().c_str(),""))
    {
      SMRKinematicJoint* parentJoint = getJointByName( joint->getParentName() );
      globalOrientation = parentJoint->getOrientation() * globalOrientation;
      globalOrientation.rotate( endPos );
      endPos =  (endPos + parentJoint->getPosition());
    }
  }
  return endPos;
}


/*void 
SMRKinematicChain::inferDOFSFromQuats(SMRSkeleton &skel)
{
  if (this->getMode() == ABSOLUTEMODE)
    this->setMode(RELATIVEMODE);
  if (skel.getMode() == ABSOLUTEMODE)
    skel.setMode(RELATIVEMODE);

  if (this->getRotationOrder() == TRANSLATIONFIRST)
    this->setRotationOrder(ROTATIONFIRST);
  if (skel.getRotationOrder() == TRANSLATIONFIRST)
    skel.setRotationOrder(ROTATIONFIRST);

  for (unsigned int i = 0; i < getNumJoints(); i++)
  {
    if (getJoint(i)->getName() == skel.getJoint(i)->getName())
    {
      getJoint(i)->inferDOFFromQuat(*(skel.getJoint(i)));
    }else
    {
      cout << " inferDOFSFromQuats failed : skeletons are not identical " << endl;
    }
  }
}*/

SMRKinematicChain &
SMRKinematicChain::operator=(const SMRKinematicChain & _skeleton)
{
  //for_each( m_joints.begin(), m_joints.end(), DeleteObjectPtr());
  m_joints.clear();
  m_name = _skeleton.getName();
  m_mode = _skeleton.getMode();
  m_rotationOrder = _skeleton.getRotationOrder();
  m_startJointIndex = _skeleton.getStartJointIndex();
  
  //for( unsigned int i = 0; i < m_startJointIndex; ++i)
  //  insertJoint(i,_skeleton.getJoint(i));

  for( unsigned int i = 0; i < m_startJointIndex; ++i )
    insertJoint( i, _skeleton.getJoint(i));

  for( unsigned int i = m_startJointIndex; i < _skeleton.getNumJoints(); ++i )
    insertJoint( i, (new SMRKinematicJoint( *_skeleton.getJoint(i))));

  return (*this);
}
