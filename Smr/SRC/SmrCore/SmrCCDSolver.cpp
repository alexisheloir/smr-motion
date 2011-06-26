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

#include "SmrCCDSolver.h"
#define GSMM_CCD_ANGULAR_EPSILON 0.0001f
#define GSMM_POTENTIEL 0.1f

/**
* base constructor
*/
SmrCCDSolver::SmrCCDSolver()
{
	m_jointProcessed = 0;
}

/**
* copy constructor
*/
SmrCCDSolver::SmrCCDSolver(SmrKinematicChain * _IKChain):SmrIKSolver()
{
  m_IKChainPtr = _IKChain;
  m_nDofs = m_IKChainPtr->getNumDofs();
  m_nConstraints = 0;
  m_jointProcessed = 0;
}

/**
* add a constraint to the kinematic chain 
*/

void SmrCCDSolver::addConstraintPtr( SmrIKConstraint* _contraintPtr )
{
  SmrIKSolver::addConstraintPtr( _contraintPtr );
  m_error.resize(m_nConstraints);
  m_jacobianT.resize(3,3);
}

/**
* Compute the jacobian.
* [...] The Jacobian matrix is the matrix of all first-order partial derivatives of a vector-valued
* function. Its importance lies in the fact that it represents the best linear approximation
* to a differentiable function near a given point. In this sense, the Jacobian is akin to a 
* derivative of a multivariate function. [...] Wikipedia
*
* The multivariate derivative is computed vias the well known half step method. The accuracy of the methods
* depends on the resolution of the angular delta (GSMM_ANGULAR_EPSILON)
*
*/

void SmrCCDSolver::computeJacobian(void)
{
  int dofHandled = 0;
  //go through every joint of the kinematic chain

  if ( m_jointProcessed == m_IKChainPtr->getNumJoints() ) m_jointProcessed = 0;

  SmrKinematicJoint *currentJoint = m_IKChainPtr->getJoint(m_jointProcessed);

  for (unsigned int j=0; j<currentJoint->getNumDofs(); j++)
  {
    //first step of the half step derivative calculation
    SmrDof * currentDof = currentJoint->getDof(j);
    currentDof->setRotationAngle(currentDof->getRotationAngle()+GSMM_CCD_ANGULAR_EPSILON);
    currentJoint->updateRot();

    //get the dF for for (\theta + d\theta)
	m_IKChainPtr->setMode( ABSOLUTEMODE );
    //for each cartesian constraint

	//Update the cartesian position and feed the matrix with the cartesian position
    SmrIKConstraint* currentConstraint = getConstraintPtr(0);
    SmrJoint* relatedJoint = currentConstraint->getRelatedJointPtr();
    SmrQuaternion relatedOrient = relatedJoint->getOrient();
    SmrVector3 currentOffset = currentConstraint->getOffset();
    relatedOrient.rotate(currentOffset);
    m_jacobianT(j+1,1) = -(relatedJoint->getPos()).X();
    m_jacobianT(j+1,2) = -(relatedJoint->getPos()).Y();
    m_jacobianT(j+1,3) = -(relatedJoint->getPos()).Z();

	m_IKChainPtr->setMode( RELATIVEMODE );

    currentDof->setRotationAngle(currentDof->getRotationAngle()-2*GSMM_CCD_ANGULAR_EPSILON);
    currentJoint->updateRot();

    //second step of the half step derivative calculation
	m_IKChainPtr->setMode( ABSOLUTEMODE );
    //Update the cartesian position and feed the matrix with the cartesian position

	relatedOrient.rotate(currentOffset);
    m_jacobianT(j+1,1) += (relatedJoint->getPos()).X();
    m_jacobianT(j+1,2) += (relatedJoint->getPos()).Y();
    m_jacobianT(j+1,3) += (relatedJoint->getPos()).Z();

    m_IKChainPtr->setMode( RELATIVEMODE );

    //reset the angle coordinate to its initial value
    currentDof->setRotationAngle(currentDof->getRotationAngle()-GSMM_CCD_ANGULAR_EPSILON);
    currentJoint->updateRot();

	if (m_jacobianT(1,1) == 0) m_jacobianT(j+1,1) = 0.01; // add some noise to enable amtrix inversion;

  }
  //cout << m_jacobianT << endl;
  m_jacobianT = m_jacobianT / (2*GSMM_CCD_ANGULAR_EPSILON);
}

/**
 * Compute the error vector.
 * gothrough every DOF of every joint. for each DOF, get the position of the constraint
 * and compare it to the position of the constraint's related joind according to the
 * cartesian offset of the chain.
 */

void SmrCCDSolver::computeErrorVect(void)
{

  if ( m_jointProcessed == m_IKChainPtr->getNumJoints() ) m_jointProcessed = 0;

  SmrKinematicJoint *currentJoint = m_IKChainPtr->getJoint(m_jointProcessed);

  //change this according to constraints.
  // Build target vector
  ColumnVector targetVector(3);
  ColumnVector currentVector(3);


  m_IKChainPtr->setMode( ABSOLUTEMODE );

  SmrIKConstraint* currentConstraint = getConstraintPtr(0);
  //get the related joint for every constraint
  SmrJoint* relatedJoint = currentConstraint->getRelatedJointPtr();
  SmrQuaternion relatedOrient = relatedJoint->getOrient();
  SmrVector3 currentOffset = currentConstraint->getOffset();
  //apply the local offsets defined in the constraint
  relatedOrient.rotate(currentOffset);
  // update error vector
  currentVector(1) = (relatedJoint->getPos()+currentOffset).X();
  currentVector(2) = (relatedJoint->getPos()+currentOffset).Y();
  currentVector(3) = (relatedJoint->getPos()+currentOffset).Z();

  targetVector(1) = (currentConstraint->getPosition()).X();
  targetVector(2) = (currentConstraint->getPosition()).Y();
  targetVector(3) = (currentConstraint->getPosition()).Z();
  //cout << currentOffset << endl;

  m_IKChainPtr->setMode( RELATIVEMODE );
  //cout << m_error << endl;// << currentVector << endl << targetVector << endl;
  currentVector-=targetVector;
  m_error=currentVector;
  //cout << m_IKChainPtr->getName() << " errorCalculated" << endl;
}

/**
* update the kinematic chain
* Apply the GSMM fundamental relation : dTheta = (Jacob)^T * E 
* Where dTheta is the angular offset to be applied, Jabob^T is the transpose
* of the jacobian matrix and E is the error vector
*/

void SmrCCDSolver::process( )
{
  ColumnVector dQ(3);
  ColumnVector dQ1(3);
  computeErrorVect();
  computeJacobian();

  //cout << m_error << endl;

  SmrKinematicJoint *currentJoint = m_IKChainPtr->getJoint(m_jointProcessed);
  //cout << m_jacobianT;
  try
  {
	 dQ = m_jacobianT*m_jointProcessed*m_error;
  }
  catch(BaseException)
  {
	 dQ = m_jacobianT*m_jointProcessed*m_error;
	 //cout << BaseException::what() << endl; 
  }
	
  float max = dQ.Maximum();
  if (max > (0.01))
	  dQ = 0.01 / max * dQ ;

  unsigned int k = 0;
  for (unsigned int j=0; j<currentJoint->getNumDofs(); j++)
  {
    SmrDof * currentDof = currentJoint->getDof(j);
    currentDof->setRotationAngle(currentDof->getRotationAngle()+dQ(k+1));
    ++k;
  }
  //currentJoint->checkTwist();
  currentJoint->updateRot();
  m_jointProcessed ++;
}


SmrVector3 SmrCCDSolver::getEndPos()
{
  m_IKChainPtr->setMode( ABSOLUTEMODE );
  SmrJoint* lastJoint = m_IKChainPtr->getJoint(m_IKChainPtr->getNumJoints()-1);
  m_IKChainPtr->setMode( RELATIVEMODE );
  return lastJoint->getPos();
}
