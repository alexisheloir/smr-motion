/**
 *  \ingroup SmrMath
 *  \file SMRPCA.h
 *  A set of functions to compute PCA for motion data
 */

#ifndef SMRPCA_H
#define SMRPCA_H
//#pragma once

#define WANT_STREAM             // 
#include "newmat.h"             // 
#include "newmatio.h"           // for output functions
#include "newmatap.h"           // for SVD

#include "SmrMotion.h"

/**
 *  \class SMRPCA
 *  \brief this class provides a generic interface for performing different types of
 *  principal components decomposition
 */
class SMRPCA
{
public:
  /**
   *  \brief Constructor.
   *
   *  a PCA can be constructed from a motion and a boolean which specifies the mode (absolute or relative) of the motion
   */
  SMRPCA(const SMRMotion & motion, SMRModeType mode);

  /**
   *  \brief Destructor.
   */
  virtual ~SMRPCA()
  {
  
  }

// data members
protected:
  /**
   *  \brief Mode
   *
   *  the mode of the motion
   */
  SMRModeType m_mode;

  /**
   *  \brief Motion
   *
   *  a copy of the original motion is kept as a data member
   */
  SMRMotion m_motion;

  /**
   *  \brief Mean skeleton
   *
   *  the mean posture of the motion
   */
  SMRSkeleton meanSkeleton;
  /// eigen postures represented as a vector of skeletons
  vector <SMRSkeleton> m_eigenPostures; 
  /// eigen values vector
  SMRTimeSerie<> m_eigenValues; 
  /// motion curves vector (represented as time series)
  vector < SMRTimeSerie<> > m_motionCurves ; 
public:
  /// get the num th eigenPosture
  SMRSkeleton & getEigenPosture (unsigned int _num){return m_eigenPostures.at(_num);} 
  /// save results, i.e. one file for eigenValues, n files for motion curves
  void saveResult(string _fileprefix, unsigned int _numMotionCurves = 0); 
  /// get the num th motion curve
  SMRTimeSerie<> getMotionCurve(unsigned int _num){return m_motionCurves.at(_num);}

  /// pure virtual method which computes eigenpostures
  virtual void computeEigenPostures() = 0;
  /**
   * get a new motion as the projection of the original motion on
   * the _num th principal components
   */
  virtual SMRMotion getProjectedMotion(unsigned int _num) = 0; 
};

/**
 * \class SMRPCAPosition
 * \brief this class provides support for PCA decomposition on the position values 
 * of the joints of skeletons
 */

class SMRPCAPosition : public SMRPCA
{
public:
  SMRPCAPosition (const SMRMotion & _motion, SMRModeType _mode = ABSOLUTEMODE); // mode is absolute or relative 
  ~SMRPCAPosition (){};

private:
  Matrix m_positionData;
  SMRSkeleton toEigenPosture(ColumnVector _v); // transform a column vector in a  posture
  SMRTimeSerie<> toMotionCurve(RowVector _v); // transform a row vector in a motion curve
public:
  void computeEigenPostures();
  SMRMotion getProjectedMotion(unsigned int _num);
};

/**
 * \class SMRPCALinearizedQuaternion
 * \brief this class provides support for PCA decomposition on linearized quaternions 
 * of the joints of skeletons
 */

class SMRPCALinearizedQuaternion : public SMRPCA
{
public:
  SMRPCALinearizedQuaternion (const SMRMotion & _motion,  SMRModeType _mode = ABSOLUTEMODE); // mode is absolute or relative 
  ~SMRPCALinearizedQuaternion (){};

private:
  Matrix m_positionData;
  SMRSkeleton toEigenPosture(ColumnVector _v); // transform a column vector in a  posture
  SMRTimeSerie<> toMotionCurve(RowVector _v); // transform a row vector in a motion curve
public:
  void computeEigenPostures();
  /**
   * \brief get the projected pose in the low dim space corresponding to the
   * coefs vector
   */
  SMRSkeleton getProjectedPose(const vector<double> & coefs);
  
  /**
   * \brief get the projected motion along the _num th first dimension
   */  
  SMRMotion getProjectedMotion(unsigned int _num);
  /**
   * \brief project one pose in the PCA space
   */  
  SMRSkeleton getProjectedPose(const SMRSkeleton & _pose, unsigned int _num, const SMRSkeleton & meanPose);

        /**
         * \brief project one pose in the PCA space
         */
        void saveProjectedPose(const SMRMotion & _motion, const SMRSkeleton & meanPose, char* fileName);

  /**
   * \brief project one motion in the PCA space
   * \return a _num dimensional vector of motion curves
   */  
  SMRMotion getProjectedMotion(SMRMotion & _motion, unsigned int _num);


};


#endif
