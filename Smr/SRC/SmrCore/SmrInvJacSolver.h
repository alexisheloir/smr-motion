/**
 *  \ingroup SmrCore
 *  \file SMRInvJacSolver.h
 */
#ifndef SMRINVJACSOLVER_H
#define SMRINVJACSOLVER_H 

#include "SmrIKSolver.h"

/**
 *  \class SMRInvJacSolver
 *  \brief Inverse Jacobian solver.
 */
class SMRInvJacSolver : public SMRIKSolver
{
public :
  /**
   *  \brief Default constructor.
   */
  SMRInvJacSolver();

  /**
   *  \brief Copy constructor.
   */
  SMRInvJacSolver(const SMRSkeleton & IKChain);

  /**
   *  \brief Copy constructor.
   */
  SMRInvJacSolver(SMRKinematicChain * IKChain);

  /**
   *  \brief Destructor.
   */
  ~SMRInvJacSolver()
  {

  }

  /**
   *  \brief Updates the chain state according to error and Jacobian.
   */
  void process(float relativeTime=0.0f);
};

#endif
