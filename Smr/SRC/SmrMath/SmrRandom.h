/** 
 * \ingroup SmrMath
 * \file SMRRandom.h
 * 
 * Set of functions to generate pseudo-random numbers
 *    - allows to sample upon uniform distribution
 *    - allows to sample upon gaussian distribution
 *    - efficient sampling for elements on SO(3)
 *
 * Those functions are accessible through a singleton class SMRRandom
 *
 * The random generator used here is rand()/RAND_MAX
 *
 */

#ifndef __SMRRANDOM__H__
#define __SMRRANDOM__H__
//#pragma once

#include "SmrMath.h"

#include "SmrVector3.h"
#include "SmrQuaternion.h"

#include <cstdlib> // definition of rand(), srand() and RAND_MAX
#include <ctime>  // access to time()


 /*! \class SMRRandom
  *  \brief Singleton. Access to random generator routines
  */


class SMRRandom
{
public:
  /// init seed with time
  static void randomize();

  // ------------- Methods on numerical values -------------------------

  /// generates a uniformly distributed double between 0 and 1
  static double getNormalizedRandom(); 
  /// generates a uniformly distributed double between _min and and _max
  static double getRandom(double _min, double _max);
  /**
   * Returns a double taken on a Gaussian with specified mean and standard dev.
   * Default behaviour is the Normal law with mean=0, std dev=1
   */
  static double getRandomGaussian(double _mean=0, double _std=1);

  // ------------- Methods on elements of R^3-------------------------
  /// generates a 3D vector with random values
  static SMRVector3 getRandomVector3(const SMRVector3 _mean  = SMRVector3(0,0,0),
                                   double _min1 = 0, double _max1 = 1,
                     double _min2 = 0, double _max2 = 1,
                     double _min3 = 0, double _max3 = 1);

  /// generates a 3D vector with random gaussian values defined by their stdv
  static SMRVector3 getRandomGaussianVector3(const SMRVector3 _mean = SMRVector3(0,0,0),
                           double _sigma1 = 1.0, double  _sigma2 = 1.0, double _sigma3 = 1.0);

  // ------------- Methods on elements of SO(3)-------------------------

  /// generates a uniformly distributed random unit quaternion
  static SMRQuaternion getRandomQuaternion(); 
  /// generates a random unit quaternion taken on a Gaussian with sigma1, sigma2, sigma3 
  /// the eigenvalues of the covariance matrix ( sqr(variance) ) and centered on 
  /// the quaternion _mean.
  /// sigmaAngle is the variance of the angle
  static SMRQuaternion getRandomGaussianQuaternion(const SMRQuaternion _mean = SMRQuaternion().identity(), 
                           double sigma1 = 1,
                           double sigma2 = 1,
                           double sigma3 = 1,
                                                     double sigmaAngle = 1); 


private:
  static SMRRandom m_singleton;

};

#endif
