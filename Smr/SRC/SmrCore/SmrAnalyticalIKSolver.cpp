#include "SmrAnalyticalIKSolver.h"


/**
* base constructor
*/
SMRAnalyticalIKSolver::SMRAnalyticalIKSolver()
{
  m_numConstraints = 0;
  m_numDOFs = 0;
}

SMRAnalyticalIKSolver::SMRAnalyticalIKSolver(const SMRSkeleton & _IKChain)//:SMRIKSolver(_IKChain)
{
/*  //check if is HAL
  if(!isHAL(_IKChain))return;
  setKinematicChain(_IKChain);*/
}

/**
* 
*/
SMRAnalyticalIKSolver::SMRAnalyticalIKSolver(SMRKinematicChain * _IKChain)//:SMRIKSolver(_IKChain)
{
  //check if is HAL
  if(!isHAL(_IKChain))
  {
    LOG_ERROR(logger, "impossible to instanciate the solver: chain passed in parameter is not Human Arm Like !");
    return;
  }
  setKinematicChain(*_IKChain);
}

/**
* update the kinematic chain
*/

void SMRAnalyticalIKSolver::process(float relativeTime)
{

  unsigned int numJoints = m_skeleton->getNumJoints();

  //compute theta
  //float theta=computeTheta();
  
  //get joints of HAL
  //SMRKinematicJoint* firstJoint=m_skeleton->getJoint(numJoints-3);
  SMRKinematicJoint* secondJoint=m_skeleton->getJoint(numJoints-2);
  //SMRKinematicJoint* thirdJoint=m_skeleton->getJoint(numJoints-1);

  float theta4=computeTheta();
  //secondJoint->setOrientation(SMRQuaternion(SMRVector3(0,0,1),theta4));
  if (theta4 > secondJoint->getDOF(3)->getLowerBound() && theta4 < secondJoint->getDOF(3)->getUpperBound())
  {
    secondJoint->getDOF(3)->setRotationAngle(theta4);
  }else if (-theta4 > secondJoint->getDOF(3)->getLowerBound() && -theta4 < secondJoint->getDOF(3)->getUpperBound())
  {
    secondJoint->getDOF(3)->setRotationAngle(-theta4);
  }
  //firstJoint->updateRotation();
  secondJoint->updateRotation();

  orientToTarget();
}
 
bool SMRAnalyticalIKSolver::isHAL(SMRKinematicChain *ikChain)
{
  unsigned int numJoints = ikChain->getNumJoints();
  //check number of joints
  if(numJoints<3)return false;

  //get joints of HAL
  SMRKinematicJoint* firstJoint=ikChain->getJoint(numJoints-1);
  if(!firstJoint)return false;
  SMRKinematicJoint* secondJoint=ikChain->getJoint(numJoints-2);
  if(!secondJoint)return false;
  SMRKinematicJoint* thirdJoint=ikChain->getJoint(numJoints-3);
  if(!thirdJoint)return false;

  //if (firstJoint->getNumDOFs() != 3) return false;
  //if (secondJoint->getNumDOFs() != 1) return false;

  //is HAL
  return true;
}

float SMRAnalyticalIKSolver::computeTheta()
{
  
  unsigned int numJoints = m_skeleton->getNumJoints();

  //get joints of HAL
  SMRKinematicJoint* firstJoint=m_skeleton->getJoint(numJoints-3);
  SMRKinematicJoint* secondJoint=m_skeleton->getJoint(numJoints-2);
  SMRKinematicJoint* thirdJoint=m_skeleton->getJoint(numJoints-1);

  m_skeleton->setMode(ABSOLUTEMODE);

  //get target position
  SMRIKConstraint *constraint=getConstraintPtr(0);

  //check if constraint relates to the right joint (the last one) 
  if (constraint->getRelatedJointName() != thirdJoint->getName())
  {
    LOG_WARN(logger, "Analytical IK cannot process this constraint " << \
                      "because it relates to the wrong joint (" << \
                       constraint->getRelatedJointName() << \
                      " instead of " <<  thirdJoint->getName() << ")");
    return 0.0f;
  }

  SMRVector3 targetPosition=constraint->getPosition();
  SMRVector3 firstPosition=firstJoint->getPosition();
  SMRVector3 secondPosition=secondJoint->getPosition();
  SMRVector3 thirdPosition=thirdJoint->getPosition();

  float targetDistance=(float)(targetPosition-firstPosition).norm();

  //get link lengths from relative positions
  float length1=(float)(secondPosition-firstPosition).norm();
  float length2=(float)(thirdPosition-secondPosition).norm();

  m_skeleton->setMode(RELATIVEMODE);

  //check if reachable
  if(targetDistance>(length1+length2))
  {
    LOG_WARN(logger, "Analytical IK cannot completly process this constraint \
                    because its related constraint is too far away" );
    return 0.0f;
  }

  //compute angle (law of cosines)
  float theta=static_cast<float>(M_PI)-acosf((length1*length1+length2*length2-targetDistance*targetDistance)/(2*length1*length2));
  return theta;
}

Matrix computePseudoInverse(const Matrix &matrix)
{
  DiagonalMatrix diag ;
  Matrix U;
  Matrix V;
  Matrix JPlus;
  try
  {
    SVD(matrix.t(),diag,U,V);
    for(int i = 0; i < diag.nrows(); i++)
    {
      (diag(i+1) > 0.0000001) ?  diag(i+1) = 1.0/diag(i+1) : diag(i+1) = 0;  
    }
    JPlus = V * diag * U.t();
  }
  catch(BaseException) 
  { 
    cout << "Error in decomposition. Reason: ";
    cout << BaseException::what(); 
  }
  return JPlus.t();
}

void updateDOFValues(SMRKinematicJoint* _joint, const ColumnVector &_deltas)
{
  for ( unsigned int i=0; i < _joint->getNumDOFs(); i++)
  {
    _joint->getDOF(i)->setRotationAngle(_deltas(i+1));
  }
  _joint->updateRotation();
}

Matrix computeJacobian(SMRKinematicJoint* _joint)
{
  Matrix jacobian(4,_joint->getNumDOFs());
  jacobian = 0.0;

  SMRQuaternion orientation = _joint->getOrientation();
  for ( unsigned int i=0; i< _joint->getNumDOFs(); i++)
  {
    jacobian(1,i+1) = orientation.m_x;
    jacobian(2,i+1) = orientation.m_y;
    jacobian(3,i+1) = orientation.m_z;
    jacobian(4,i+1) = orientation.m_w;

  }

  //cout << jacobian << endl;

  float delta = 0.001f;
  for ( unsigned int i=0; i < _joint->getNumDOFs(); i++)
  {
    SMRDOF* currentDof = _joint->getDOF(i);
    double currentAngle = currentDof->getRotationAngle();
    currentDof->setRotationAngle(currentAngle - delta);
    _joint->updateRotation();
    SMRQuaternion orientation = _joint->getOrientation();
    //cout << jacobian << endl;
    jacobian(1,i+1) -= orientation.m_x;
    jacobian(2,i+1) -= orientation.m_y;
    jacobian(3,i+1) -= orientation.m_z;
    jacobian(4,i+1) -= orientation.m_w;
    currentDof->setRotationAngle(currentAngle + delta);
    _joint->updateRotation();
  }

  //cout << jacobian << endl;

  return jacobian;
}

ColumnVector getError (const SMRQuaternion &rot1, const SMRQuaternion &rot2)
{
  ColumnVector error(4);
  error = 0.0;
  error(1) = rot2.m_x - rot1.m_x;
  error(2) = rot2.m_y - rot1.m_y;
  error(3) = rot2.m_z - rot1.m_z;
  error(4) = rot2.m_w - rot1.m_w;

  return error;
}

/*void dof2Quat(SMRKinematicJoint* _joint, const SMRQuaternion &_rot)
{
  for ( int i=0; i < 10; i++)
  {
    ColumnVector error = getError(_joint->getOrientation(),_rot);
    Matrix jacobian = computeJacobian(_joint);
    //cout << jacobian << endl;
    Matrix jacobianPInv = computePseudoInverse(jacobian);
    //cout << jacobian.t() << endl;
    ColumnVector deltas = 2000*jacobian.t()*error;
    //cout << error << endl;
    //cout << deltas << endl;
    updateDOFValues(_joint,deltas);
  }
}*/

void SMRAnalyticalIKSolver::orientToTarget()
{
  unsigned int numJoints = m_skeleton->getNumJoints();

  //get joints of HAL
  SMRKinematicJoint* firstJoint=m_skeleton->getJoint(numJoints-3);
  SMRKinematicJoint* secondJoint=m_skeleton->getJoint(numJoints-2);
  SMRKinematicJoint* thirdJoint=m_skeleton->getJoint(numJoints-1);

  firstJoint->setOrientation(0,0,0);

  m_skeleton->setMode(ABSOLUTEMODE);

  //get target position
  SMRIKConstraint *constraint=getConstraintPtr(0);

  SMRVector3 targetPosition=constraint->getPosition();
  SMRVector3 firstPosition=firstJoint->getPosition();
  SMRVector3 secondPosition=secondJoint->getPosition();
  SMRVector3 thirdPosition=thirdJoint->getPosition();

  SMRQuaternion firstAbsoluteOrientation = firstJoint->getOrientation();

  m_skeleton->setMode(RELATIVEMODE);

  SMRVector3 goalVector=(targetPosition-firstPosition).normalize();
  SMRVector3 chainVector=(thirdPosition-firstPosition).normalize();

  //compute axis-angle
  SMRVector3 axis=CrossProduct(chainVector,goalVector);
  if(axis.norm()>0.001f)
  {
    float angle=acosf(static_cast<float>(DotProduct(chainVector,goalVector)));

    SMRQuaternion rotation(axis,angle);

    rotation = (Inverse(firstAbsoluteOrientation)) * rotation * firstAbsoluteOrientation;

    firstJoint->updateRotation();

    SMRQuaternion jointRotation = firstJoint->getOrientation(); 
    LOG_DEBUG(logger,"original shoulder orientation: " << jointRotation);
    LOG_DEBUG(logger,"new shoulder orientation: " << rotation);
    LOG_DEBUG(logger,"1-dotProduct between orientations: " << DotProduct(jointRotation, rotation));

    if (DotProduct(jointRotation, rotation) < 0.0 )
    {
      rotation = -rotation;
    }

    double x,y,z;
    //SMRQuaternion orientation = (firstJoint->getOrientation());
    rotation.toEulerAnglesXZY( x, y, z );
    firstJoint->getDOF(1)->setRotationAngle(x);
    firstJoint->getDOF(3)->setRotationAngle(z);
    firstJoint->getDOF(4)->setRotationAngle(y);

    firstJoint->updateRotation();

  }
}
