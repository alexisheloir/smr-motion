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

#include "SmrGSMMSolver.h"


/**
* base constructor
*/
SMRGSMMSolver::SMRGSMMSolver():SMRIKSolver()
{
  m_globalGain = 1.0f;
}

SMRGSMMSolver::SMRGSMMSolver(const SMRSkeleton & _IKChain):SMRIKSolver(_IKChain)
{
  m_globalGain = 1.0f;
}

/**
* 
*/
SMRGSMMSolver::SMRGSMMSolver(SMRKinematicChain * _IKChain):SMRIKSolver(_IKChain)
{
  m_globalGain = 1.0f;
}

/**
* update the kinematic chain
* Apply the GSMM fundamental relation : dTheta = (Jacob)^T * E 
* Where dTheta is the angular offset to be applied, Jabob^T is the transpose
* of the jacobian matrix and E is the error vector
*/

void SMRGSMMSolver::process(float relativeTime)
{
  //unsigned int iterations = 0;
  ColumnVector dQ;
  //clock_t startTime = clock();

  //do
  //{
    computeErrorVector();
    computeJacobian();

    DiagonalMatrix ggain(m_numDOFs);

    //double error = m_error.norm1();

    int dofIndex = 1;

    //feed the gain matrix (ggain)
    for (unsigned int i = m_skeleton->getStartJointIndex(); i < m_skeleton->getNumJoints() ; i++)
    {
      SMRKinematicJoint* currentJoint = m_skeleton->getJoint(i);
      for (unsigned int j = 0; j < currentJoint->getNumDOFs(); j++)
      {
        SMRDOF * currentDof = currentJoint->getDOF(j);
        //ggain(dofIndex) = currentDof->getGain()*exp(-(SQrealError)/Dsigma2);
        double currentGain = currentDof->getGain();
        ggain(dofIndex) = currentGain;
        dofIndex++;
      }
    }

    m_jacobianT = (ggain*m_jacobianT);
    dQ = (m_jacobianT*(m_error));//*gain;
    double deltaQ; // DELTA T = M'(Q).DELTA(Q)
    unsigned int k = 0; //(DOF increment)
    for (unsigned int i = m_skeleton->getStartJointIndex(); i<m_skeleton->getNumJoints(); i++)
    {
      SMRKinematicJoint* currentJoint = m_skeleton->getJoint(i);
      for (unsigned int j=0; j<currentJoint->getNumDOFs(); j++)
      {
        SMRDOF * currentDof = currentJoint->getDOF(j);
        deltaQ = dQ(k+1)*m_globalGain;
        //if ( deltaQ > 0.0001 && deltaQ < IK_ANGULAR_EPSILON) deltaQ = 0.01;

        currentDof->setRotationAngle(currentDof->getRotationAngle()+deltaQ);
        ++k;
      }
      //currentJoint->checkTwist();
      currentJoint->updateRotation();
    }
}
 
