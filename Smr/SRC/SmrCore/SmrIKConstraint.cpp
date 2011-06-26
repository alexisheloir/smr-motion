//
// C++ Implementation: SMRIKConstraint
//
// Description: 
//
//
// Author:  <>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "SmrIKConstraint.h"

SMRIKConstraint::SMRIKConstraint(SMRVector3 _offset, SMRVector3 _position, string _relatedJointName) : m_offset(_offset),  m_relatedJointName(_relatedJointName), m_position(_position)
{
}

SMRIKConstraint::SMRIKConstraint()
{
  m_offset.init(0.0,0.0,0.0);
  m_position.init(0.0,0.0,0.0);
  m_relatedJointName = "";
}


SMRIKConstraint::~SMRIKConstraint()
{
}


