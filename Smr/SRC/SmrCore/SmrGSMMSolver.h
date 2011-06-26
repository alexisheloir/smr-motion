/**
 *  \ingroup SmrCore
 *  \file SMRGSMMSolver.h
 */
#ifndef SMRGSMMIKSOLVER_H
#define SMRGSMMIKSOLVER_H 

#include "SmrIKSolver.h"

/**
 *  \class SMRGSMMSolver
 *  \brief GSMM solver.
 */
class SMRGSMMSolver : public SMRIKSolver
{
public :
  /**
   *  \brief Constructor.
   */
  SMRGSMMSolver();

  /**
   *  \brief Copy constructor.
   */
  SMRGSMMSolver(const SMRSkeleton & IKChain);

  /**
   *  \brief Copy constructor.
   */
  SMRGSMMSolver(SMRKinematicChain * IKChain);

  /**
   *  \brief Destructor.
   */
  ~SMRGSMMSolver()
  {

  }

  inline void setGlobalGain(float _globalGain){m_globalGain = _globalGain;}

  /**
   *  \brief Updates the chain state according to error and Jacobian.
   */
  void process(float relativeTime=0.0f);

protected :

  float m_globalGain;

};

#endif
