/**
*  \ingroup SmrCore
*  \file SMRIKSolver.h
*/
#ifndef SMRIKSOLVER_H
#define SMRIKSOLVER_H

#define IK_ANGULAR_EPSILON 0.0001f
#define IK_POTENTIEL 0.1f
#define IK_ERROR_TRESHOLD 0.001f

#include "SmrActuator.h"
#include "SmrKinematicChain.h"
#include "SmrIKConstraint.h"
#include "newmat.h"
#include "newmatio.h"           // for output functions
#include "newmatap.h"           // for SVD

/**
*  \class SMRIKSolver
*  \brief This class is an inverse kinematic dedicated specialization of a SMRJoint.
*/
class SMRIKSolver : public SMRActuator
{
public:
  /**
  *  \brief Default constructor.
  */
  SMRIKSolver()
  {
  }

  /**
  *  \brief Copy constructor.
  */
  SMRIKSolver(const SMRSkeleton & IKChain);

  /**
  *  \brief Copy constructor.
  */
  SMRIKSolver(SMRKinematicChain * IKChain);

  /**
  *  \brief Destructor.
  */
  ~SMRIKSolver()
  {
  }

  /**
  *  \brief Adds a constraint.
  */
  void addConstraintPtr( SMRIKConstraint* _contraintPtr );

  /**
  *  \brief Flushes the constraints.
  */
  void flushConstraintPtrs()
  {
    m_numConstraints = 0;
    m_constraintPtrVector.clear();
  }

  /**
  *  \brief Returns a constraint pointer.
  */
  inline SMRIKConstraint* getConstraintPtr(const unsigned int index) const
  {
    return m_constraintPtrVector.at(index);
  }

  /**
  *  \brief Returns the number of constraints.
  */
  inline unsigned int getNumConstraints(void) const
  {
    return m_constraintPtrVector.size();
  }


  /**
  *  \brief Refreshes the skeleton.
  */
  inline void refreshSkeleton()
  {
    //SMRSkeleton* subSkeleton = reinterpret_cast<SMRSkeleton*>(&m_skeleton);
    //SMRSkeleton refSubSkeleton = m_referenceSkeleton->getSubSkeleton(*subSkeleton);
    //unsigned int startIndex = m_skeleton->getStartJointIndex();
    //m_skeleton = SMRKinematicChain(refSubSkeleton);
    //m_skeleton->setStartJointIndexInt(startIndex);
  }

  /**
  *  \brief Computes the Jacobian matrix.
  */
  void computeJacobian();

  /**
  *  \brief Computes the error vector as a column vector.
  */
  void computeErrorVector();

  /**
  *  \brief Returns the end position.
  */
  SMRVector3 getEndPosition();

  /**
  *  \brief Sets the kinematic chain.
  */
  void setKinematicChain(SMRKinematicChain &chain)
  {
    m_skeleton = &chain;
    m_numDOFs = m_skeleton->getNumDOFs();
    m_numConstraints = 0;
  }

  /**
  *  \brief Returns the skeleton.
  */
  SMRSkeleton * getSkeleton(float attenuationFactor)
  {
    return reinterpret_cast<SMRSkeleton*>(& m_skeleton);
  }

  inline Real getErrorNorm(){return m_error.norm1();}

protected:

  /**
  *  \brief Number of degrees of freedom for chain.
  */
  unsigned int m_numDOFs;

  //a pointer on the kinematic chain
  //SMRKinematicChain *  m_IKChainPtr;

  /**
  *  \brief Column vector representing the error.
  */
  ColumnVector m_error;

  /**
  *  \brief Matrix storing the jacobian values.
  */
  Matrix m_jacobianT;

  /**
  *  \brief Constraint vector.
  */
  std::vector<SMRIKConstraint*> m_constraintPtrVector;

  /**
  *  \brief Number of constraints.
  */
  int m_numConstraints;

  /**
  *  \brief Skeleton
  */
  SMRKinematicChain *m_skeleton;
};

#endif
