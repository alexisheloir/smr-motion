//
// C++ Interface: smrkinematicjoint
//
// Description:
//
//
// Author: Alexis Heloir <alexis.heloir@univ-ubs.fr>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef SMRCCDSOLVER_H
#define SMRCCDSOLVER_H 

#include "SmrIKSolver.h"
#include "newmat.h"
#include "newmatio.h"           // for output functions
#include "newmatap.h"           // for SVD

class SmrCCDSolver : public SmrIKSolver
{
public :

  //base constructor
  SmrCCDSolver(void);

  //copy constructor
  SmrCCDSolver(SmrKinematicChain * _IKChaIn);

  ~SmrCCDSolver(){};

  //update chain state, according to error and jacobian
  void process(void);

  //add a cartesian constraint ptr for the chain
  void addConstraintPtr( SmrIKConstraint* _contraintPtr );

  //compute jacobian matrix
  void computeJacobian(void);

  //compute error vector as a columns vector
  void computeErrorVect(void);

  SmrVector3 getEndPos();

protected :

  //how many Dof for the chain ?
  unsigned int m_nDofs;

  //a pointer on the kinematic chain
  SmrKinematicChain *  m_IKChainPtr;

  //a column vector representing the error
  ColumnVector m_error;

  //a matric storing the jacobian values
  Matrix m_jacobianT;

  unsigned int m_jointProcessed;
};

#endif
