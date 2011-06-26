//
// C++ Implementation: SMRIKSolver
//
// Description: cf SMRIKSolver.h
//
//
// Author: Alexis Heloir <alexis.heloir@univ-ubs.fr>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "SmrIKSolver.h"

SMRIKSolver::SMRIKSolver(const SMRSkeleton & _IKChain):SMRActuator()
{
  m_skeleton = new SMRKinematicChain(_IKChain);
  m_numDOFs = m_skeleton->getNumDOFs();
  m_numConstraints = 0;
}

/**
* 
*/
SMRIKSolver::SMRIKSolver(SMRKinematicChain * _IKChain):SMRActuator()
{
  m_skeleton = _IKChain;
  m_numDOFs = m_skeleton->getNumDOFs();
  m_numConstraints = 0;
}

/**
* add a constraint to the kinematic chain 
*/

void SMRIKSolver::addConstraintPtr( SMRIKConstraint* _contraintPtr )
{
  m_constraintPtrVector.push_back(_contraintPtr);
  m_numConstraints += 3;
  m_error.resize(m_numConstraints);
  m_jacobianT.resize(m_numDOFs,m_numConstraints);
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

void SMRIKSolver::computeJacobian(void)
{
  int dofHandled = 0;
  //go through every joint of the kinematic chain

  for (unsigned int i=m_skeleton->getStartJointIndex(); i<m_skeleton->getNumJoints(); i++)
  {
    SMRKinematicJoint* currentJoint = m_skeleton->getJoint(i);
    //go through every DOF of the kinematic chain
    for (unsigned int j=0; j<currentJoint->getNumDOFs(); j++)
    {
      //first step of the half step derivative calculation
      SMRDOF * currentDof = currentJoint->getDOF(j);
      if ( currentDof->getMeanBound() > 0.01f )
      {
        currentDof->setRotationAngle(currentDof->getRotationAngle()+IK_ANGULAR_EPSILON);
        currentJoint->updateRotation();

        //get the dF for for (\theta + d\theta)
        m_skeleton->setMode( ABSOLUTEMODE );
        //for each cartesian constraint
        for (unsigned int k = 0; k < m_constraintPtrVector.size(); k++)
        {
          //Update the cartesian position and feed the matrix with the cartesian position
          SMRIKConstraint* currentConstraint = getConstraintPtr(k);
          SMRKinematicJoint* relatedJoint = m_skeleton->getJointByName(currentConstraint->getRelatedJointName());
          SMRQuaternion relatedOrient = relatedJoint->getOrientation();
          SMRVector3 currentOffset = currentConstraint->getOffset();
          relatedOrient.rotate(currentOffset);
          if ( i == (m_skeleton->getNumJoints()-1) && currentOffset.isZero() )
          {
            m_jacobianT(dofHandled+1,k*3+1) = 0;
            m_jacobianT(dofHandled+1,k*3+2) = 0;
            m_jacobianT(dofHandled+1,k*3+3) = 0;
          }else
          {
            m_jacobianT(dofHandled+1,k*3+1) = -(relatedJoint->getPosition()+currentOffset).X();
            m_jacobianT(dofHandled+1,k*3+2) = -(relatedJoint->getPosition()+currentOffset).Y();
            m_jacobianT(dofHandled+1,k*3+3) = -(relatedJoint->getPosition()+currentOffset).Z();
          }
        }

        m_skeleton->setMode( RELATIVEMODE );

        currentDof->setRotationAngle(currentDof->getRotationAngle()-2*IK_ANGULAR_EPSILON);
        currentJoint->updateRotation();
      
        //second step of the half step derivative calculation
        m_skeleton->setMode( ABSOLUTEMODE );
        for (unsigned int k = 0; k < m_constraintPtrVector.size(); k++)
        {
          //Update the cartesian position and feed the matrix with the cartesian position
          SMRIKConstraint* currentConstraint = getConstraintPtr(k);
          SMRKinematicJoint* relatedJoint = m_skeleton->getJointByName(currentConstraint->getRelatedJointName());
          SMRQuaternion relatedOrient = relatedJoint->getOrientation();
          SMRVector3 currentOffset = currentConstraint->getOffset();
          relatedOrient.rotate(currentOffset);

          if ( i == (m_skeleton->getNumJoints()-1) && currentOffset.isZero() )
          {
          }else
          {
            m_jacobianT(dofHandled+1,k*3+1) += (relatedJoint->getPosition()+currentOffset).X();
            m_jacobianT(dofHandled+1,k*3+2) += (relatedJoint->getPosition()+currentOffset).Y();
            m_jacobianT(dofHandled+1,k*3+3) += (relatedJoint->getPosition()+currentOffset).Z();
          }
        }
      }else
      {
        for (unsigned int k = 0; k < m_constraintPtrVector.size(); k++)
        {
          m_jacobianT(dofHandled+1,k*3+1) = 0.0;
          m_jacobianT(dofHandled+1,k*3+2) = 0.0;
          m_jacobianT(dofHandled+1,k*3+3) = 0.0;
        }
      }

      m_skeleton->setMode( RELATIVEMODE );

      //reset the angle coordinate to its initial value
      currentDof->setRotationAngle(currentDof->getRotationAngle()+IK_ANGULAR_EPSILON);
      currentJoint->updateRotation();
      dofHandled++;
    }
  }
  m_jacobianT = m_jacobianT / (2*IK_ANGULAR_EPSILON);
}

/**
* Compute the error vector.
* gothrough every DOF of every joint. for each DOF, get the position of the constraint
* and compare it to the position of the constraint's related joind according to the
* cartesian offset of the chain.
*/

void SMRIKSolver::computeErrorVector(void)
{

  //change this according to constraints.
  // Build target vector
  ColumnVector targetVector(m_numConstraints);
  ColumnVector currentVector(m_numConstraints);


  m_skeleton->setMode( ABSOLUTEMODE );

  //go through every registered constraint
  for (unsigned int i = 0; i < m_constraintPtrVector.size(); i++)
  {
    SMRIKConstraint* currentConstraint = getConstraintPtr(i);
    //cout << currentConstraint->getPosition();
    //get the related joint for every constraint
    SMRKinematicJoint* relatedJoint = m_skeleton->getJointByName(currentConstraint->getRelatedJointName());
    if (relatedJoint == NULL)
      LOG_FATAL(logger,"SmrIKSolver.cpp, in computeErrorVector(): the joint " + currentConstraint->getRelatedJointName() + " was not found");
    SMRQuaternion relatedOrient = relatedJoint->getOrientation();
    SMRVector3 currentOffset = currentConstraint->getOffset();
    //apply the local offsets defined in the constraint
    relatedOrient.rotate(currentOffset);
    // update error vector
    currentVector(i*3+1) = (relatedJoint->getPosition()+currentOffset).X();
    currentVector(i*3+2) = (relatedJoint->getPosition()+currentOffset).Y();
    currentVector(i*3+3) = (relatedJoint->getPosition()+currentOffset).Z();

    if (currentConstraint->getJointTarget() != "")
    {
      //m_referenceSkeleton->setMode(ABSOLUTEMODE);
      SMRKinematicJoint* targetJoint = m_skeleton->getJointByName(currentConstraint->getJointTarget());
      //cout << targetJoint->getPosition() << endl;
      targetVector(i*3+1) = (targetJoint->getPosition()+currentConstraint->getPosition()).X();
      targetVector(i*3+2) = (targetJoint->getPosition()+currentConstraint->getPosition()).Y();
      targetVector(i*3+3) = (targetJoint->getPosition()+currentConstraint->getPosition()).Z();
      //currentConstraint->setPosition(targetJoint->getPosition());
      //currentConstraint->setJointTarget("");
      //m_referenceSkeleton->setMode(RELATIVEMODE);

    }else
    {
      targetVector(i*3+1) = (currentConstraint->getPosition()).X();
      targetVector(i*3+2) = (currentConstraint->getPosition()).Y();
      targetVector(i*3+3) = (currentConstraint->getPosition()).Z();
      //cout << currentOffset << endl;
    }
  }
  m_skeleton->setMode( RELATIVEMODE );
  //cout << m_error << endl;// << currentVector << endl << targetVector << endl;
  currentVector-=targetVector;
  m_error=currentVector;
  //cout << m_skeleton->getName() << " errorCalculated" << endl; 
}

SMRVector3 SMRIKSolver::getEndPosition()
{
  return m_skeleton->getEndPosition(m_skeleton->getNumJoints()-1);
}
