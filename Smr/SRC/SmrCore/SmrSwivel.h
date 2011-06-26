/**
*  \ingroup SmrCore
*  \file SMRIKSolver.h
*/

#ifndef SMRSWIVEL_H
#define SMRSWIVEL_H

#include "SmrActuator.h"
#include "SmrKinematicChain.h"


class SMRSwivel : public SMRActuator
{
	public:
	 /**
	  *  \brief Default constructor.
	  */
	  SMRSwivel();

	 /**
	  *  \brief from skeleton constructor.
	  */
    SMRSwivel(SMRKinematicChain *_chain, const string _startJointName, const string _endJointName, const string _middleJointName);


 	 /**
	  *  \brief Default destructor
	  */
	  ~SMRSwivel();

    inline void setAngle(float _angle)
    {
      m_swivelAngle = _angle;
    }
   /**
    *  \brief Updates the chain state according to error and Jacobian.
    */
    void process(float relativeTime=0.0f);

    SMRSkeleton* getSkeleton(float attenuationFactor=1.0f);

    void setKinematicChain(SMRKinematicChain &_chain, const string _startJointName, const string _endJointName, const string _middleJointName);

    void setXAxis(SMRVector3 _xAxis)
    {
      m_xAxis = _xAxis;
    }

    void setUpDirection(const string _joint1, const string _joint2);

  protected:
   /**
    *  \brief Skeleton
    */
    SMRVector3 m_xAxis;
    SMRKinematicChain *m_skeleton;
    SMRKinematicJoint *m_startJoint, *m_endJoint, *m_middleJoint, *m_upVectorStartJoint, *m_upVectorEndJoint;
    float m_swivelAngle;

};

#endif
