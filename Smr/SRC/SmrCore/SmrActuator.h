/**
 *  \ingroup SmrCore
 *  \file SMRActuator.h
 */
#ifndef SMRACTUATOR_H
#define SMRACTUATOR_H
#pragma once

#include "SmrSkeleton.h"
/**
 *  \class SMRActuator
 *  \brief Abstract class in charge of updating skeletons.
 *
 *  SMRActuator is an abstract class in charge of 
 *  computing an up-to-date skeleton thanks to \ref SmrActuator_Process "process()" .
 *  The behavior of the class Actuator is driven by a 
 *  set of instructions/order/constraints that have to 
 *  be updtated each time animation goal may change.
 */
class SMRActuator
{
public:
  /**
   *  \brief virtual destructor (if implemented vitual functions mess 
   *  up with memory)
   */
  virtual ~SMRActuator(){;}
  /**
   *  \brief This function is the pure virtual function in charge of
   *  updating the skeleton.
   *  \anchor SmrActuator_Process
   */
  virtual void process(float relativeTime=0.0f)=0;

  /**
   *  \brief Gets the skeleton.
   */
  virtual SMRSkeleton * getSkeleton(float attenuationFactor = 1.0f) = 0;

protected:
 /**
  *  \brief Reference towards the reference skeleton.
  */
  const SMRSkeleton *m_referenceSkeleton;
};

#endif


