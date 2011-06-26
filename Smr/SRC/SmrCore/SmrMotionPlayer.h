/**
 *  \ingroup SmrCore
 *  \file SMRMotionPlayer.h
 */

#ifndef SMRMOTIONPLAYER_H
#define SMRMOTIONPLAYER_H
#pragma once

#include "SmrSTL.h"
#include "SmrMath.h"
#include "SmrMotion.h"
#include "SmrActuator.h"

/**
 *  \brief A bounding box structure.
 */
typedef struct {
  double mx;
  double Mx;
  double my;
  double My;
  double mz;
  double Mz;
} BoundingBox;


/**
 *  \class SMRMotionPlayer
 *  \brief this class is a type of Actuator that allows
 *  you to play an SMRMotion. It can be controlled
 *  by setting :
 *  - a frame
 *  - a time in the motion
 *  before processing.
 *  It is possible to have an automatic interpolation between 
 *  skeletons in the specified number of frame is not an integer.
 *  This is explicitely set by the user
 */
class SMRMotionPlayer : public SMRActuator
{
public:
  // an SMRMotionPlayer can not only be constructed with an existing motion
  //SMRMotionPlayer(const SMRMotion & _motion);

  /**
   *  \brief Copy constructor.
   */
  SMRMotionPlayer(SMRMotion* motion);

  /**
   *  \brief Default constructor.
   */
  SMRMotionPlayer();

  /**
   *  \brief Destructor.
   */
  ~SMRMotionPlayer()
  {
  }

public:
  /**
   *  \brief Sets a motion.
   */
  inline void setMotion(SMRMotion* motion)
  {
     m_motion = motion; 
     m_frame = 0.0f; 
     computeBoundingBox();
  }

  /**
   *  \brief Processes.
   *
   *  SMRMotionPlayer is an Actuator and has to define a process() mehod
   *  that updates the skeleton
   */
  void process(float relativeTime=0.0f);

  /**
   *  \brief Sets the frame to display.
   *
   *  this set of methods allows to control the actuator
   */
  void setFrameToDisplay(float numFrame);

  /**
   *  \brief Sets the frame to display.
   */
  void setFrameToDisplay(unsigned int numFrame);

  /**
   *  \brief Sets the time to display.
   */
  void setTimeToDisplay(float time);

  /**
   *  \brief Returns the total time.
   *
   *  accessor to total time of animation
   */
  float getTotalTime();

  /**
   *  \brief Sets the interpolation mode.
   *
   *  control interpolation mode
   */
  inline void setInterpolationMode(bool b)
  {
    m_interpolation = b;
  }

  /**
   *  \brief Returns the interpolation mode.
   */
  inline bool getInterpolationMode() const
  {
    return m_interpolation;
  }

  /**
   *  \brief Returns the bounding box diagonal.
   */
  double getBoundingBoxDiagonal() const
  {
    return m_boundingBoxDiagonal;
  }

  /**
   *  \brief Returns the skeleton.
   */
  SMRSkeleton* getSkeleton(float attennuationFactor=1.0f)
  {
    return & m_skeleton;
  }

protected:
  /**
   *  \brief Skeleton.
   */
  SMRSkeleton m_skeleton;

  /**
   *  \brief Motion.
   *
   *  The SMRMotionPlayer contains an SMRMotion
   */
  SMRMotion* m_motion;

private:

  /**
   *  \brief Interpolation.
   *
   *  Interpolating or not ?
   */
  bool m_interpolation;

  /**
   *  \brief Frame.
   *
   *  frame to display
   */
  float m_frame;

  /**
   *  \brief Bounding box.
   *
   *  bounding box
   */
  BoundingBox m_boundingBox;

  /**
   *  \brief Bounding box diagonal.
   *
   *  bounding box diagonal
   */
  double m_boundingBoxDiagonal; 

  /**
   *  \brief Computes the bounding box.
   *
   *  computeBoundingBox
   */
  void computeBoundingBox();
};

#endif
