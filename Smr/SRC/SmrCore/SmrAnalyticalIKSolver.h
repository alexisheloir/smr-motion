/**
 *  \ingroup SmrCore
 *  \file SMRAnalyticalIKSolver.h
 */
#ifndef SMRANALYTICALIKSOLVER_H
#define SMRANALYTICALIKSOLVER_H 

#include "SmrIKSolver.h"

/**
 *  \class SMRAnalyticalIKSolver
 *  \brief Analytical IK solver.
 */
class SMRAnalyticalIKSolver : public SMRIKSolver
{
public :
  /**
   *  \brief Constructor.
   */
  SMRAnalyticalIKSolver();

  /**
   *  \brief Copy constructor.
   */
  SMRAnalyticalIKSolver(const SMRSkeleton & IKChain);

  /**
   *  \brief Copy constructor.
   */
  SMRAnalyticalIKSolver(SMRKinematicChain * IKChain);

  /**
   *  \brief Destructor.
   */
  ~SMRAnalyticalIKSolver()
  {

  }

  /**
   *  \brief Updates the chain state according to the HAL (human-arm-like) approach.
   */
  void process(float relativeTime=0.0f);

private:
  bool isHAL(SMRKinematicChain *ikChain);
  float computeTheta();
  void orientToTarget();
};

#endif
