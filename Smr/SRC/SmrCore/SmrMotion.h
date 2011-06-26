/**
 *  \ingroup SmrCore
 *  \file SMRMotion.h
 */

#ifndef SMRMOTION_H
#define SMRMOTION_H
#pragma once 

#include "SmrSTL.h"
#include "SmrSkeleton.h"
#include "SmrMath.h"
#include <float.h> // DBL_MAX definition
#include <cstring>

/**
 *  \class SMRMotion
 *  \brief This class represents a generic motion structure
 *  which is composed by several SMRSkeleton which are
 *  equivalent to postures for a given skeleton.
 *  Warning ! An SMRMotion can be composed of different 
 *  hierarchy for skeletons. 
 */
class SMRMotion
{
public:
  /**
   *  \brief Default constructor.
   */
  SMRMotion()
  {
    m_timeStep=0.0;
  }

  /**
   *  \brief Copy constructor.
   */
  SMRMotion(const SMRMotion &motion);

  /**
   *  \brief Destructor.
   */
  virtual ~SMRMotion()
  {

  }

private:
  /**
   *  \brief Skeleton vector.
   *
   *  An SMRMotion is composed of several skeletons
   */
  vector<SMRSkeleton> m_skeletonTab;
  /**
   *  \brief Time step.
   *
   *  and is caracterized by a frequency, here represented as
   *  a timestep between two frames (i.e. skeletons)
   */
  double m_timeStep;

  /**
   *  \brief Diagonal
   *
   *  diagonal of motions boundig box
   */
  double m_diagonal;

public:
  /**
   *  \brief Returns the time step.
   *
   *  simple accessor to timestep
   */
  inline double getTimeStep() const
  {
    return m_timeStep;
  }

  /**
   *  \brief Sets the time step.
   *
   *  set timestep, this operation is done without resampling
   */
  inline void setTimeStep(double dt)
  {
    m_timeStep = dt;
  }

  /**
   *  \brief Returns the number of frames.
   *
   *  get number of frames (i.e. skeletons) in motion
   */
  inline unsigned int getNumFrames() const
  {
    return m_skeletonTab.size();
  }

  /**
   *  \brief Returns a skeleton.
   *
   *  get skeleton at frame i
   */
  const SMRSkeleton & getSkeleton (unsigned int num) const;

  /**
   *  \brief Returns the bounding box diagonal.
   *
   *  get bounding box dagonal
   */
  inline const double & getBoundingDiagonal () const
  {
    return m_diagonal;
  }

  /**
   *  \brief Inserts a skeleton.
   *
   *  add a skeleton to the motion
   */
  inline void insertSkeleton(const SMRSkeleton & skeleton)
  {
    m_skeletonTab.push_back(skeleton);
  }

  /**
   *  \brief Inserts a skeleton.
   */
  inline void insertSkeleton(const SMRSkeleton & skeleton, int pos)
  {
    m_skeletonTab.insert(m_skeletonTab.begin(),skeleton);
  }

  /**
   *  \brief Removes a skeleton.
   *
   *  remove skeleton at frame _num
   */
  void removeSkeleton(unsigned int num);
  
  /**
   *  \brief Returns a mean skeleton.
   *
   *  get mean skeleton
   */
  SMRSkeleton getMean(SMRModeType mode = ABSOLUTEMODE) const;

  /**
   *  \brief Returns the quaternionic mean.
   *
   *  get mean skeleton using quaternion averaging technique
   */
  SMRSkeleton getQuaternionicMean();

  /**
   *  \brief Returns an approximate quaternionic mean.
   *
   *  some private methods to implement the quaternionic mean
   */
  SMRSkeleton getApproximateQuaternionicMean();

  /**
   *  \brief Gradient descent method.
   *
   *  some private methods to implement the quaternionic mean TODO: what's this ?
   */
  void oneStepGradientDescentMethod(SMRSkeleton & skeleton);

  /**
   *  \brief Dumps quaternions to a file.
   *
   *  dump quaternions into a file that can be used in Matlab for 
   *  instance
   *  \param filename is the name of the file
   *  \param mode give the quaternions either in absolute or relative mode
   *          (see SMRSkeleton for details)
   */
  bool dumpQuaternionsToFile(const string filename, SMRModeType mode = ABSOLUTEMODE);

  /**
   *  \brief Dumps positions to a file.
   *
   *  dump positions into a file that can be used in Matlab for 
   *  instance
   *  \param filename is the name of the file
   *  \param mode give the quaternions either in absolute or relative mode
   *          (see SMRSkeleton for details)
   */
  bool dumpPositionsToFile(const string filename, SMRModeType mode = ABSOLUTEMODE);

  /**
   *  \brief Dumps Euler angles to a file.
   *
   *  dump euler angles a file that can be used in Matlab for 
   *  instance
   *  \param filename is the name of the file
   *  \param mode: give the quaternions either in absolute or relative mode
   *          (see SMRSkeleton for details)
   */
  bool dumpEulerAnglesToFile(const string filename, SMRModeType mode = ABSOLUTEMODE);

  /**
   *  \brief Hemispherizes.
   *
   *  Put all the quaternions of all the joints on the same hemisphere of S^3
   *  Warning ! This method modifies the motion, while it looks exactly the same
   */
  void hemispherize();

  /**
   *  \brief Returns a quaternionic time serie.
   *
   *  Returns an SMRTimeSerie of quaternions corresponding to a joint
   */
  SMRTimeSerie<SMRQuaternion> getQuaternionicTimeSerie(unsigned int numJoint);

  /**
   *  \brief Returns quaternionic time serie.
   */
  SMRTimeSerie<SMRQuaternion> getQuaternionicTimeSerie(string name);

  /**
   *  \brief Returns the joint trajectory.
   *
   *  Returns an SMRTimeSerie of SMRVector3 corresponding to the joint position
   */
  SMRTimeSerie<SMRVector3> getJointTrajectory(unsigned int numJoint);

  /**
   *  \brief Returns the joint trajectory.
   */
  SMRTimeSerie<SMRVector3> getJointTrajectory(string name);

  /**
   *  \brief Changes the mode.
   *
   *  Change representation mode of all postures
   */
  void changeMode(SMRModeType mode);

  void rotate(const SMRQuaternion &rotation)
  {
    std::vector<SMRSkeleton>::iterator it;
    for (it = m_skeletonTab.begin(); it < m_skeletonTab.end(); it++)
    {
      (*it).rotate(rotation);
    }
  }

  void changeBindPose(const SMRSkeleton &bindPose)
  {
    std::vector<SMRSkeleton>::iterator it;
    for (it = m_skeletonTab.begin(); it < m_skeletonTab.end(); it++)
    {
      (*it).changeBindPose(bindPose);
    }
  }

public:
  // useful methods on an SMRMotion
  // Warning ! those methods are destructive

  /**
   *  \brief Warps.
   *
   *  warp motion according to hermite poly _poly
   *  the size of the vector will not be changed
   */
  void warp(SMRHermitePoly<double> poly);

  /**
   *  \brief Resamples.
   *
   *  resample motion with new timestep
   *  the size of the vector will be changed
   */
  void resample(double dt);

  /**
   *  \brief Cuts the motion from frame.
   *
   *  cut motion : keep frames until/from a certain frame 
   */
  void cutMotionFromFrame(unsigned int num);

  /**
   *  \brief Cuts the motion until frame.
   */
  void cutMotionUntilFrame(unsigned int num);

  /**
   *  \brief Cuts the motion.
   *
   *  cut a certain portion of the motion
   */
  void cutMotion(unsigned int begin, unsigned int duration);

  /**
   *  \brief Crops the motion.
   *
   *  crop Motion : keep everything between _begin and _end
   */
  void cropMotion(unsigned int begin, unsigned int end);

  /**
   *  \brief Decimates the motion.
   *
   *  Decimate motion : keep every nth frame
   */
  void decimateMotion(unsigned int n);

  /**
   *  \brief Add motion operator.
   *
   *  add motion to the current one, the motion will be concatenated
   *  at the end of the first one
   */
  SMRMotion & operator+ (const SMRMotion & motion);

  /**
   *  \brief Assign motion operator.
   *
   *  operator = declaration
   */
  SMRMotion & operator= (const SMRMotion & motion);

  /**
   *  \brief Compare motion operator.
   *
   *  operator == declaration
   */
  bool  operator== (const SMRMotion & _motion);

  /**
   *  \brief Merges motion.
   *
   *  Merge motion. Resample every motions at the same frequency. 
   *  This frequency is chosen to be the highest one (smallest timeStep)
   *  Several versions :
   *    - add : find common nodes in skeletons then merge skeletons from there
   *    - replace : find common nodes in skeletons then replace subbranches  from there
   */
  void mergeMotionAdd(const SMRMotion & motion);

  /**
   *  \brief Merge motion replace.
   */
  void mergeMotionReplace(SMRMotion & motion);

  /**
   *  \brief Extracts a sub skeleton.
   *
   *  Extract a sub-squeleton for reach posture of motion
   *  \param subSkeleton the skeleton template to be extraced
   *  \returns the resulting sub-motion
   */
  SMRMotion extractSubSkeleton(const SMRSkeleton & subSkeleton);

  SMRMotion derivate();

};

#endif

