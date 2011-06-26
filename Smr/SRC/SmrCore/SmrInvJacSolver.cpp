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

#include "SmrInvJacSolver.h"
#define GSMM_GSMM_ANGULAR_EPSILON 0.001f
#define GSMM_POTENTIEL 0.1f

SMRInvJacSolver::SMRInvJacSolver()
{
}

SMRInvJacSolver::SMRInvJacSolver(const SMRSkeleton & _IKChain):SMRIKSolver(_IKChain)
{
}

/**
* 
*/
SMRInvJacSolver::SMRInvJacSolver(SMRKinematicChain * _IKChain):SMRIKSolver(_IKChain)
{
}

void SMRInvJacSolver::process(float relativeTime)
{
  unsigned int iterations = 0;
  ColumnVector dQ;
  //do
  //{
    iterations++;
    computeErrorVector();
    computeJacobian();

    //int dofIndex = 1;
    Matrix jiTji = m_jacobianT * m_jacobianT.t();


    DiagonalMatrix diag ;
    Matrix U;
    Matrix V;
    Matrix JPlus;

    // perform SVD on data
    try
    {
      SVD(m_jacobianT,diag,U,V);
      for(int i = 0; i < diag.nrows(); i++)
      {
        (diag(i+1) > 0.0000001) ?  diag(i+1) = 1.0/diag(i+1) : diag(i+1) = 0;  
      }
      JPlus = V * diag * U.t();
      dQ = (JPlus.t()*(m_error));
    }
    catch(BaseException) 
    { 
      cout << "Error in decomposition. Reason : " << endl;
      cout << BaseException::what() << endl; 
      dQ = 0;
    }
    //Real max = dQ.Maximum();
    //if (max > (0.01))
    //  dQ = 0.01 / max * dQ ;

    unsigned int k = 0;
    for (unsigned int i=m_skeleton->getStartJointIndex(); i<m_skeleton->getNumJoints(); i++)
    {
      SMRKinematicJoint* currentJoint = m_skeleton->getJoint(i);
      for (unsigned int j=0; j<currentJoint->getNumDOFs(); j++)
      {
        SMRDOF * currentDof = currentJoint->getDOF(j);
        currentDof->setRotationAngle(currentDof->getRotationAngle()+dQ(k+1));
        ++k;
      }
      //currentJoint->checkTwist();
      currentJoint->updateRotation();
    }
  //}while((m_error.norm1() > IK_ERROR_TRESHOLD) && iterations < 20 );
}
