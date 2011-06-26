#include "SmrRandom.h"

// change seed
void 
SMRRandom::randomize()
{
  // seed the generator with time
  srand( ((unsigned int)( time( NULL ) )) );
}


// generates a uniformly distributed double between 0 and 1
double 
SMRRandom::getNormalizedRandom(){
  return static_cast<double>(rand())/RAND_MAX;
}

// generates a uniformly distributed double between _min and and _max
double 
SMRRandom::getRandom(double _min, double _max)
{
  return (_min+getNormalizedRandom()*(_max-_min));
}

// Normal Law with mean and standard deviation
double 
SMRRandom::getRandomGaussian(double _mean, double _stddev)
{
  const int nbSamples = 12; //increase for better precision. 
  double v = 0;
  for(int i=0 ; i < nbSamples ; ++i)
    v += getNormalizedRandom();
  v -= nbSamples/2;
  v *= (nbSamples == 12) ? _stddev : sqrt(12/static_cast<double>(nbSamples))*_stddev; // spreading using standard deviation
  v += _mean; // centering 
  return v;
}

// random Vector
SMRVector3 
SMRRandom::getRandomVector3(const SMRVector3 _mean,
              double _min1, double _max1,double _min2, double _max2,double _min3, double _max3)
{
  return (_mean + SMRVector3( getRandom(_min1,_max1), getRandom(_min2,_max2), getRandom(_min3,_max3)) ); 
}

// random Gaussian Vector
SMRVector3 
SMRRandom::getRandomGaussianVector3(const SMRVector3 _mean,
                      double _sigma1, double  _sigma2, double _sigma3)
{
  return (SMRVector3( getRandomGaussian(_mean.m_x,_sigma1), getRandomGaussian(_mean.m_y,_sigma2), getRandomGaussian(_mean.m_z,_sigma3) ) ); 
}

// generates a uniformly distributed random unit quaternion
// from Kuffner ICRA04 "Effective Sampling.."

SMRQuaternion 
SMRRandom::getRandomQuaternion()
{
  SMRQuaternion result;
  double s = getNormalizedRandom();
  double sigma1 = sqrt(1-s);
  double sigma2 = sqrt(s);
  double theta1 = M_2PI * getNormalizedRandom();
  double theta2 = M_2PI * getNormalizedRandom();
  result.m_w = cos(theta2)*sigma2;
  result.m_x = sin(theta1)*sigma1;
  result.m_y = cos(theta1)*sigma1;
  result.m_z = sin(theta2)*sigma2;
  return result;
}

// random quaternion on a gaussian
SMRQuaternion 
SMRRandom::getRandomGaussianQuaternion(const SMRQuaternion _mean,
                     double _sigma1,double _sigma2,double _sigma3,
                     double _sigmaAngle)
{
  SMRQuaternion result;
  SMRVector3 axis = (getRandomGaussianVector3()).normalize();
  axis.m_x *= _sigma1;
  axis.m_y *= _sigma2;
  axis.m_z *= _sigma3;
  double angle = getRandomGaussian(0,_sigmaAngle);
  result = Exp(axis * angle); 
  return _mean*result;
}
