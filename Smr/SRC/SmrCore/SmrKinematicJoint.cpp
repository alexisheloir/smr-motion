#include "SmrKinematicJoint.h"

SMRKinematicJoint::SMRKinematicJoint(bool _endJoint): SMRJoint(_endJoint)
{
}

SMRKinematicJoint::~SMRKinematicJoint()
{
}

SMRKinematicJoint::SMRKinematicJoint(const SMRJoint & _joint): SMRJoint(_joint)
{
  double x,y,z;
  _joint.getOrientation().toEulerAnglesXZY( x, y, z );
  addDOF( SMRDOF::YAXIS, 0, 0, 0 );
  addDOF( SMRDOF::XAXIS, x, x, x );
  addDOF( SMRDOF::YAXIS, 0, 0, 0 );
  addDOF( SMRDOF::ZAXIS, z, z, z );
  addDOF( SMRDOF::YAXIS, y, y, y );
  this->updateRotation();
}

SMRKinematicJoint::SMRKinematicJoint(SMRKinematicJoint & _joint): SMRJoint(_joint)
{
  for (unsigned int i=0; i<_joint.getNumDOFs(); i++)
  {
    m_dofVector.push_back(*(_joint.getDOF(i)));
  }

}

void SMRKinematicJoint::addDOF(const SMRDOF::SMRRotationAxisType _axis, const double _lowerBound, const double _upperBound, const double _value, const double gain)
{
  SMRDOF dof;
  dof.setRotationAxis(_axis);
  dof.setRotationAngle(_value);
  dof.setUpperBound(_upperBound);
  dof.setLowerBound(_lowerBound);
  dof.setGain(gain);
  dof.setJointName(m_name);
  m_dofVector.push_back(dof);
}

void SMRKinematicJoint::addDOF(SMRVector3 customAxis, const double _lowerBound, const double _upperBound, const double _value, const double gain)
{
  SMRDOF dof;
  dof.setRotationAxis(customAxis);
  dof.setRotationAngle(_value);
  dof.setUpperBound(_upperBound);
  dof.setLowerBound(_lowerBound);
  dof.setGain(gain);
  dof.setJointName(m_name);
  m_dofVector.push_back(dof);
}

void SMRKinematicJoint::updateRotation()
{
  SMRQuaternion orient;
  SMRVector3 finalOrientAxis;

  //orient.identity();

  std::vector<SMRDOF>::iterator dofIt;
  for (dofIt=m_dofVector.begin(); dofIt < m_dofVector.end(); dofIt++ )
  {
    orient = orient * ((*dofIt).getQuaternionRotation());
  }
  m_orientation = orient;

  // apply the two first angle parametrization
  //dofIt = m_dofVector.begin();
  //firstone
  //orient = orient * (*dofIt).getQuaternionRotation();
  //dofIt++;
  //firstone = secondone.rot(firstone)
  //orient = orient * ((*dofIt).getQuaternionRotation()) ;
  //dofIt++;
  //m_orientation = orient * ((*dofIt).getQuaternionRotation()) ;

  /*
  orientAngle = orient.getRotationAngle();

  //m_orientation = orient;

  //get finalaxis
  //finalOrientAxis = orient.getRotationAxis();
  finalOrientAxis = getPosition();
  finalOrientAxis.normalize();
  //orient.rotate(finalOrientAxis);

  dofIt++;

  double phi,theta,twista,twistaCor;
  phi = getDOF(0)->getRotationAngle();
  theta = getDOF(1)->getRotationAngle();
  twista = getDOF(2)->getRotationAngle();

  //tweak rotation
  twistaCor = (phi*theta*2.0/M_PI);
  if (twistaCor >=  3.14) twistaCor -= M_PI;
  if (twistaCor <= -3.14) twistaCor += M_PI;

  //twistaCor is the opposite of induced rotation.

  SMRQuaternion twistRot(finalOrientAxis,twistaCor+twista);
  //SMRQuaternion twistRot(finalOrientAxis,(M_PI));
  //twistRot.identity();

  //finalOrientAngle = (*dofIt)->getQuaternionRotation().getRotationAngle();
  //m_orientation = (SMRQuaternion(finalOrientAxis,orientAngle+finalOrientAngle));

  m_orientation = orient * twistRot ;
  //*/
}

void SMRKinematicJoint::checkTwist()
{
  double cx,cy,rx,ry,Sx,Sy,Rx,Ry;

  cx = getDOF( 0 )->getCentralAngle();
  cy = getDOF( 1 )->getCentralAngle();

  rx = getDOF( 0 )->getMeanBound();
  ry = getDOF( 1 )->getMeanBound();

  rx = fabs(rx);
  ry = fabs(ry);

  if (rx == 0.0) rx = 1000000.0;
  if (ry == 0.0) ry = 1000000.0;

  Sx = getDOF( 0 )->getRotationAngle() - cx;
  Sy = getDOF( 1 )->getRotationAngle() - cy;

  Rx = Sx/rx;
  Ry = Sy/ry;

  //cout << cx << " " << cy << " " << rx << "  " << ry << " " << Rx << " " << Ry << " " << (Rx)*(Rx) + (Ry)*(Ry) << endl;

  if ( (Rx)*(Rx) + (Ry)*(Ry) > 1)
  {
    if(Rx > 0) getDOF( 0 )->setRotationAngle(cx + Sx - (fabs(Ry) * 0.02928 * M_PI));
    else  getDOF( 0 )->setRotationAngle(cx + Sx + (fabs(Ry) * 0.02928 * M_PI));
    if(Ry > 0) getDOF( 1 )->setRotationAngle(cy + Sy - (fabs(Rx) * 0.02928 * M_PI));
    else  getDOF( 1 )->setRotationAngle(cy + Sy + (fabs(Rx) * 0.02928 * M_PI));
  }
}
