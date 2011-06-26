#include <stdio.h>

#include <gtest/gtest.h>


#include "SmrQuaternion.h"
#include "SmrMath.h"

#define ROUNDING_ERROR_BOUND 0.0000001

TEST(SMRQuaternion, Constructor_Empty) {
  SMRQuaternion quat;
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Constructor_WXYZ) {
  SMRQuaternion quat(1,2,3,4);
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 2);
  ASSERT_DOUBLE_EQ(quat.m_y, 3);
  ASSERT_DOUBLE_EQ(quat.m_z, 4);
}

TEST(SMRQuaternion, Constructor_XYZ) {
  SMRQuaternion quat(0,0,0);
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Constructor_AxisAngle) {
  SMRQuaternion quat( SMRVector3(1,0,0), 0 );
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Identity) {
  SMRQuaternion quat;
  quat.identity();
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Normalize) {
  SMRQuaternion quat(4,0,0,0);
  quat.normalize();
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Rotate) {
  SMRQuaternion quat( SMRVector3(1,0,0), M_PI/2 );
  SMRVector3 v(0,1,0);
  quat.rotate(v);
  ASSERT_DOUBLE_EQ(v.m_x, 0);
  ASSERT_DOUBLE_EQ(v.m_y, 0);
  ASSERT_DOUBLE_EQ(v.m_z, 1);
}

TEST(SMRQuaternion, SquareNorm) {
  SMRQuaternion quat(4,0,0,0);
  ASSERT_DOUBLE_EQ(quat.squareNorm(), 16);
}

TEST(SMRQuaternion, Norm) {
  SMRQuaternion quat(4,0,0,0);
  ASSERT_DOUBLE_EQ(quat.norm(), 4);
}

TEST(SMRQuaternion, EulerAngles_From) {
  SMRQuaternion quat;
  quat.fromEulerAngles(M_PI/2,0,0);
  ASSERT_NEAR(quat.m_w, 0.707106781, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, 0.707106781, ROUNDING_ERROR_BOUND);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, EulerAngles_ToXYZ) {
  SMRQuaternion quat( SMRVector3(1,0,0), M_PI/2 );
  double x,y,z;
  quat.toEulerAngles(x,y,z);
  ASSERT_DOUBLE_EQ(x, M_PI/2);
  ASSERT_DOUBLE_EQ(y, 0);
  ASSERT_DOUBLE_EQ(z, 0);
}


TEST(SMRQuaternion, eulerConversion) {
  double alpha,beta,gamma;
  alpha = rand();
  beta = rand();
  gamma = rand();

  SMRQuaternion quat, quat2,quat3;
  quat.fromEulerAngles(alpha,beta,gamma);
 
  double resultAlpha = 0.0;
  double resultBeta = 0.0;
  double resultGamma = 0.0;
  quat.normalize();
  quat.toEulerAngles(resultAlpha,resultBeta,resultGamma);

  ASSERT_NEAR(resultAlpha, alpha , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);

  SMRQuaternion resultQuat;
  resultQuat.fromEulerAngles(resultAlpha,resultBeta,resultGamma);
  resultQuat.normalize();
  
  ASSERT_NEAR(quat.m_w, resultQuat.m_w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, resultQuat.m_x, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, resultQuat.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, resultQuat.m_z , ROUNDING_ERROR_BOUND);
}

TEST(SMRQuaternion, eulerConversionAlphaPiHalf) {
  double alpha,beta,gamma;
  alpha = M_PI/ 2.0;
  beta = 0.0;
  gamma =0.0;

  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);
 
  double resultAlpha = 0.0;
  double resultBeta = 0.0;
  double resultGamma = 0.0;
  quat.normalize();
  quat.toEulerAngles(resultAlpha,resultBeta,resultGamma);

  ASSERT_NEAR(resultAlpha, alpha , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);

  SMRQuaternion resultQuat;
  resultQuat.fromEulerAngles(resultAlpha,resultBeta,resultGamma);
  resultQuat.normalize();
  
  ASSERT_NEAR(quat.m_w, resultQuat.m_w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, resultQuat.m_x, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, resultQuat.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, resultQuat.m_z , ROUNDING_ERROR_BOUND);
}

TEST(SMRQuaternion, eulerConversionBetaPiHalf) {
  double alpha,beta,gamma;
  alpha = 0.0;
  beta = M_PI/ 2.0;
  gamma =0.0;

  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);
 
  double resultAlpha = 0.0;
  double resultBeta = 0.0;
  double resultGamma = 0.0;
  quat.normalize();
  quat.toEulerAngles(resultAlpha,resultBeta,resultGamma);

  ASSERT_NEAR(resultAlpha, alpha , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);

  SMRQuaternion resultQuat;
  resultQuat.fromEulerAngles(resultAlpha,resultBeta,resultGamma);
  resultQuat.normalize();
  
  ASSERT_NEAR(quat.m_w, resultQuat.m_w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, resultQuat.m_x, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, resultQuat.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, resultQuat.m_z , ROUNDING_ERROR_BOUND);
}
TEST(SMRQuaternion, eulerConversionGammaPiHalf) {
  double alpha,beta,gamma;
  alpha = 0.0;
  beta = 0.0;
  gamma =M_PI/ 2.0;

  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);
 
  double resultAlpha = 0.0;
  double resultBeta = 0.0;
  double resultGamma = 0.0;
  quat.normalize();
  quat.toEulerAngles(resultAlpha,resultBeta,resultGamma);

  ASSERT_NEAR(resultAlpha, alpha , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);

  SMRQuaternion resultQuat;
  resultQuat.fromEulerAngles(resultAlpha,resultBeta,resultGamma);
  resultQuat.normalize();
  
  ASSERT_NEAR(quat.m_w, resultQuat.m_w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, resultQuat.m_x, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, resultQuat.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, resultQuat.m_z , ROUNDING_ERROR_BOUND);
}
TEST(SMRQuaternion, eulerConversionPiHalf) {
  double alpha,beta,gamma;
  alpha = M_PI/ 2.0;
  beta = M_PI/ 2.0;
  gamma =M_PI/ 2.0;

  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);
 
  double resultAlpha = 0.0;
  double resultBeta = 0.0;
  double resultGamma = 0.0;
  quat.normalize();
  quat.toEulerAngles(resultAlpha,resultBeta,resultGamma);

  ASSERT_NEAR(resultAlpha, alpha , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);

  SMRQuaternion resultQuat;
  resultQuat.fromEulerAngles(resultAlpha,resultBeta,resultGamma);
  //resultQuat.normalize();
  
  ASSERT_NEAR(quat.m_w, resultQuat.m_w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, resultQuat.m_x, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, resultQuat.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, resultQuat.m_z , ROUNDING_ERROR_BOUND);
}
TEST(SMRQuaternion, eulerConversionDouble) {
  double alpha,beta,gamma;
  alpha = M_PI/ 8.0;
  beta = M_PI/ 8.0;
  gamma = M_PI/ 8.0;

  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);
 
  double resultAlpha = 0.0;
  double resultBeta = 0.0;
  double resultGamma = 0.0;
  quat.normalize();
  quat.toEulerAngles(resultAlpha,resultBeta,resultGamma);


  SMRQuaternion resultQuat;
  resultQuat.fromEulerAngles(resultAlpha,resultBeta,resultGamma);
  resultQuat.normalize();
  //resultQuat.toEulerAngles(alpha,beta,gamma);
  
 // ASSERT_NEAR(resultAlpha, alpha , ROUNDING_ERROR_BOUND);
 // ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);
 // ASSERT_NEAR(resultBeta, beta, ROUNDING_ERROR_BOUND);
  
  ASSERT_NEAR(quat.m_w, resultQuat.m_w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, resultQuat.m_x, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, resultQuat.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, resultQuat.m_z , ROUNDING_ERROR_BOUND);
}

TEST(SMRQuaternion, compEulerQuatRotationWithConversion) {
  double alpha,beta,gamma;
  alpha = rand();
  beta = rand();
  gamma = rand();

  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);

  double resultAlpha = 0.0;
  double resultBeta = 0.0;
  double resultGamma = 0.0;
  quat.normalize();
  quat.toEulerAngles(resultAlpha,resultBeta,resultGamma);
  SMRVector3 rotPoint(1.0,0.0,0.0);
  quat.rotate(rotPoint);
  double cosAlpha = cos(alpha);
  double sinAlpha = sin(alpha);
  double cosBeta = cos(beta);
  double sinBeta = sin(beta);
  double cosGamma = cos(gamma);
  double sinGamma = sin(gamma); 
  double a11 = cosGamma * cosAlpha - cosBeta * sinAlpha * sinGamma;  
  double a12 = cosGamma * sinAlpha + cosBeta * cosAlpha * sinGamma;
  double a13 = sinGamma * sinBeta;
  double a21 = -sinGamma * cosAlpha -cosBeta * sinAlpha * cosGamma;  
  double a22 = -sinGamma *sinAlpha +cosBeta *cosAlpha *cosGamma;
  double a23 = cosGamma * sinBeta;
  double a31 = sinBeta * cosAlpha;
  double a32 = -sinBeta * cosAlpha;
  double a33 = cosBeta;
  
  SMRVector3 matrRot(rotPoint.m_x * a11 + rotPoint.m_y * a12 + rotPoint.m_z* a13, 
                    rotPoint.m_x * a21 + rotPoint.m_y * a22 + rotPoint.m_z* a23,
                    rotPoint.m_x * a31 + rotPoint.m_y * a32 + rotPoint.m_z* a33); 
  ASSERT_NEAR(matrRot.m_x, rotPoint.m_x , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(matrRot.m_y, rotPoint.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(matrRot.m_z, rotPoint.m_z , ROUNDING_ERROR_BOUND);
  
}
TEST(SMRQuaternion, compEulerQuatRotation) {
  double alpha,beta,gamma;

  SMRQuaternion quat( SMRVector3(1,0,0), M_PI/2 );
  quat.toEulerAngles(alpha,beta,gamma);

  SMRVector3 rotPoint(1.0,0.0,0.0);
  quat.rotate(rotPoint);
  double cosAlpha = cos(alpha);
  double sinAlpha = sin(alpha);
  double cosBeta = cos(beta);
  double sinBeta = sin(beta);
  double cosGamma = cos(gamma);
  double sinGamma = sin(gamma); 
  double a11 = cosGamma * cosAlpha - cosBeta * sinAlpha * sinGamma;  
  double a12 = cosGamma * sinAlpha + cosBeta * cosAlpha * sinGamma;
  double a13 = sinGamma * sinBeta;
  double a21 = -sinGamma * cosAlpha -cosBeta * sinAlpha * cosGamma;  
  double a22 = -sinGamma *sinAlpha +cosBeta *cosAlpha *cosGamma;
  double a23 = cosGamma * sinBeta;
  double a31 = sinBeta * cosAlpha;
  double a32 = -sinBeta * cosAlpha;
  double a33 = cosBeta;
  
  SMRVector3 matrRot(rotPoint.m_x * a11 + rotPoint.m_y * a12 + rotPoint.m_z* a13, 
                    rotPoint.m_x * a21 + rotPoint.m_y * a22 + rotPoint.m_z* a23,
                    rotPoint.m_x * a31 + rotPoint.m_y * a32 + rotPoint.m_z* a33); 
  ASSERT_NEAR(matrRot.m_x, rotPoint.m_x , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(matrRot.m_y, rotPoint.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(matrRot.m_z, rotPoint.m_z , ROUNDING_ERROR_BOUND);
  
}  
  
TEST(SMRQuaternion, eulerConversionRand)
{
  double alpha,beta,gamma;
  alpha = rand();
  beta = rand() ;
  gamma = rand() ;

  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);
  
  double cosAlpha = cos(alpha/2.0);
  double sinAlpha = sin(alpha/2.0);
  double cosBeta = cos(beta/2.0);
  double sinBeta = sin(beta/2.0);
  double cosGamma = cos(gamma/2.0);
  double sinGamma = sin(gamma/2.0);  
  double w = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  
  w =  cosAlpha * cosBeta * cosGamma + sinAlpha * sinBeta * sinGamma; 
  x =  sinAlpha * cosBeta * cosGamma - cosAlpha * sinBeta * sinGamma;
	y =  cosAlpha * sinBeta * cosGamma + sinAlpha * cosBeta * sinGamma;
	z =  cosAlpha * cosBeta * sinGamma - sinAlpha * sinBeta * cosGamma; 
	
  ASSERT_NEAR(quat.m_w, w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, x , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, z , ROUNDING_ERROR_BOUND);
}

TEST(SMRQuaternion, quaternionToEuler180degreeY)
{
  double alpha,beta,gamma;
  alpha = 0.0;
  beta = 0.0;
  gamma = 0.0;

  SMRQuaternion quat(0.0,0.0,1.0,0.0);
  quat.toEulerAngles(alpha,beta,gamma);
  std::cerr<<"alpha: "<<alpha<<" . beta: "<<beta<<" . gamma: "<<gamma<<"\n";
  ASSERT_DOUBLE_EQ(alpha, 0.0);
  ASSERT_DOUBLE_EQ(beta,  M_PI);
  ASSERT_DOUBLE_EQ(gamma, 0.0);  
}
TEST(SMRQuaternion, quaternionToEuler45degreeY45degreeZ)
{
  double alpha,beta,gamma;
  alpha = 0.0;
  beta = 0.0;
  gamma = 0.0;

  SMRQuaternion quat(0.8535533905932737,0.3535533905932738,0.14644660940672624,0.3535533905932738);
  quat.normalize();
  quat.toEulerAngles(alpha,beta,gamma);
  std::cerr<<"alpha: "<<alpha<<" . beta: "<<beta<<" . gamma: "<<gamma<<"\n";
  ASSERT_DOUBLE_EQ(alpha, 0.0);
  ASSERT_DOUBLE_EQ(beta,  M_PI/4.0);
  ASSERT_DOUBLE_EQ(gamma, M_PI/4.0);  
}

TEST(SMRQuaternion, quaternionToEuler90degreeX)
{
  double alpha,beta,gamma;
  alpha = 0.0;
  beta = 0.0;
  gamma = 0.0;

  SMRQuaternion quat(0.7071067811865476,0.7071067811865476,0.0,0.0);
  quat.toEulerAngles(alpha,beta,gamma);
  std::cerr<<"alpha: "<<alpha<<" . beta: "<<beta<<" . gamma: "<<gamma<<"\n";
  ASSERT_DOUBLE_EQ(alpha, M_PI *0.5);
  ASSERT_DOUBLE_EQ(beta,  0.0);
  ASSERT_DOUBLE_EQ(gamma,0.0);  
}

TEST(SMRQuaternion,EulerToQuaternion_Alpha90) {
  double alpha,beta,gamma;
  alpha = M_PI *0.5;
  beta = 0.0;
  gamma = 0.0;
  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);
  std::cerr<<"quat.m_w: "<<quat.m_w<<" .quat.m_x: "<<quat.m_x<<" . quat.m_y: "<<quat.m_y<<" . quat.m_z: "<<quat.m_z<<"\n";
  ASSERT_DOUBLE_EQ( quat.m_w,0.7071067811865476);
  ASSERT_DOUBLE_EQ( quat.m_x,0.7071067811865476);
  ASSERT_DOUBLE_EQ( quat.m_y,0.0);
  ASSERT_DOUBLE_EQ( quat.m_z,0.0); 
}
TEST(SMRQuaternion,EulerToQuaternion_Beta90) {
  double alpha,beta,gamma;
  alpha = 0.0;        
  beta = M_PI *0.5;
  gamma = 0.0;
  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);
  std::cerr<<"quat.m_w: "<<quat.m_w<<" .quat.m_x: "<<quat.m_x<<" . quat.m_y: "<<quat.m_y<<" . quat.m_z: "<<quat.m_z<<"\n";
  ASSERT_DOUBLE_EQ( quat.m_w,0.7071067811865476);
  ASSERT_DOUBLE_EQ( quat.m_x,0.0);
  ASSERT_DOUBLE_EQ( quat.m_y,0.7071067811865476);
  ASSERT_DOUBLE_EQ( quat.m_z,0.0); 
}

TEST(SMRQuaternion,EulerToQuaternion_gamma135) {
  double alpha,beta,gamma;
  alpha = 0.0;        
  beta = 0.0;
  gamma = M_PI *0.75;
  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);
  std::cerr<<"quat.m_w: "<<quat.m_w<<" .quat.m_x: "<<quat.m_x<<" . quat.m_y: "<<quat.m_y<<" . quat.m_z: "<<quat.m_z<<"\n";
  ASSERT_DOUBLE_EQ( quat.m_w,0.38268343236508984);
  ASSERT_DOUBLE_EQ( quat.m_x,0.0);
  ASSERT_DOUBLE_EQ( quat.m_y,0.0);
  ASSERT_DOUBLE_EQ( quat.m_z,0.9238795325112867); 
}

TEST(SMRQuaternion,EulerToQuaternion_alpha45gamma135) {
  double alpha,beta,gamma;
  alpha = M_PI*0.5;        
  beta = 0.0;
  gamma = M_PI *0.75;
  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);
  std::cerr<<"quat.m_w: "<<quat.m_w<<" .quat.m_x: "<<quat.m_x<<" . quat.m_y: "<<quat.m_y<<" . quat.m_z: "<<quat.m_z<<"\n";
  quat.toEulerAngles(alpha,beta,gamma);
  std::cerr<<"alpha: "<<alpha<<" . beta: "<<beta<<" . gamma: "<<gamma<<"\n";
  ASSERT_DOUBLE_EQ( quat.m_w,0.35355339059327384);
  ASSERT_DOUBLE_EQ( quat.m_x,0.8535533905932737);
  ASSERT_DOUBLE_EQ( quat.m_y,0.14644660940672627);
  ASSERT_DOUBLE_EQ( quat.m_z,-0.3535533905932738); 
}

TEST(SMRQuaternion,EulerToQuaternion_Alpha45Beta45) {
  double alpha,beta,gamma;
  alpha = M_PI *0.25;
  beta = M_PI *0.25;
  gamma = 0.0;
  SMRQuaternion quat;
  quat.fromEulerAngles(alpha,beta,gamma);
  std::cerr<<"quat.m_w: "<<quat.m_w<<" .quat.m_x: "<<quat.m_x<<" . quat.m_y: "<<quat.m_y<<" . quat.m_z: "<<quat.m_z<<"\n";
  quat.toEulerAngles(alpha,beta,gamma);
  std::cerr<<"alpha: "<<alpha<<" . beta: "<<beta<<" . gamma: "<<gamma<<"\n";
  ASSERT_DOUBLE_EQ( quat.m_w,0.8535533905932737);
  ASSERT_DOUBLE_EQ( quat.m_x,0.3535533905932738);
  ASSERT_DOUBLE_EQ( quat.m_y,0.3535533905932738);
  ASSERT_DOUBLE_EQ( quat.m_z,0.14644660940672624); 
}

TEST(SMRQuaternion,EulerToQuaternion_Alpha) {
  
  SMRQuaternion quat;
   
  double alpha,beta,gamma;
  alpha = M_PI/2.0;   //x
  beta = 0.0;         //y
  gamma = 0.0;        //z
  
  quat.fromEulerAngles(alpha,beta,gamma);
  
  double cosAlpha = cos(alpha/2.0);
  double sinAlpha = sin(alpha/2.0);
  double cosBeta = cos(beta/2.0);
  double sinBeta = sin(beta/2.0);
  double cosGamma = cos(gamma/2.0);
  double sinGamma = sin(gamma/2.0);  
  double w = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  
  w =  cosAlpha * cosBeta * cosGamma + sinAlpha * sinBeta * sinGamma; 
  x =  sinAlpha * cosBeta * cosGamma - cosAlpha * sinBeta * sinGamma;
	y =  cosAlpha * sinBeta * cosGamma + sinAlpha * cosBeta * sinGamma;
	z =  cosAlpha * cosBeta * sinGamma - sinAlpha * sinBeta * cosGamma; 
	
  ASSERT_NEAR(quat.m_w, w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, x , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, z , ROUNDING_ERROR_BOUND);
}
     
TEST(SMRQuaternion,EulerToQuaternion_Beta) {
  
  SMRQuaternion quat;
   
  double alpha,beta,gamma;
  alpha = 0.0;        
  beta = M_PI/2.0;    
  gamma = 0.0;         
  
  quat.fromEulerAngles(alpha,beta,gamma);
  
  double cosAlpha = cos(alpha/2.0);
  double sinAlpha = sin(alpha/2.0);
  double cosBeta = cos(beta/2.0);
  double sinBeta = sin(beta/2.0);
  double cosGamma = cos(gamma/2.0);
  double sinGamma = sin(gamma/2.0);
  double w = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  
  w =  (cosAlpha * cosBeta * cosGamma) + (sinAlpha * sinBeta * sinGamma); 
  x =  sinAlpha * cosBeta * cosGamma - cosAlpha * sinBeta * sinGamma;
	y =  cosAlpha * sinBeta * cosGamma + sinAlpha * cosBeta * sinGamma;
	z =  cosAlpha * cosBeta * sinGamma - sinAlpha * sinBeta * cosGamma; 
	
  ASSERT_NEAR(quat.m_w, w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, x , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, z , ROUNDING_ERROR_BOUND);
}

TEST(SMRQuaternion,EulerToQuaternion_Gamma) {
  
  SMRQuaternion quat;
   
  double alpha,beta,gamma;
  alpha = 0.0;         //x
  beta  = 0.0;         //y
  gamma = M_PI/2.0;    //z
  
  quat.fromEulerAngles(alpha,beta,gamma);
  
  double cosAlpha = cos(alpha/2.0);
  double sinAlpha = sin(alpha/2.0);
  double cosBeta = cos(beta/2.0);
  double sinBeta = sin(beta/2.0);
  double cosGamma = cos(gamma/2.0);
  double sinGamma = sin(gamma/2.0);
  double w = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  
  w =  cosAlpha * cosBeta * cosGamma + sinAlpha * sinBeta * sinGamma; 
  x =  sinAlpha * cosBeta * cosGamma - cosAlpha * sinBeta * sinGamma;
	y =  cosAlpha * sinBeta * cosGamma + sinAlpha * cosBeta * sinGamma;
	z =  cosAlpha * cosBeta * sinGamma - sinAlpha * sinBeta * cosGamma; 
	
  ASSERT_NEAR(quat.m_w, w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, x , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, z , ROUNDING_ERROR_BOUND);
}
TEST(SMRQuaternion, QuatToEulerAndBackX) {
  SMRQuaternion quat( SMRVector3(1,0,0), M_PI/2.0 );
  double x,y,z;
  quat.toEulerAnglesZYX(x,y,z);
  SMRQuaternion resultQuat;
  resultQuat.fromEulerAngles(x,y,z);
  ASSERT_DOUBLE_EQ(x, M_PI/2.0);
  ASSERT_DOUBLE_EQ(y, 0);
  ASSERT_DOUBLE_EQ(z, 0);
  ASSERT_NEAR(quat.m_w, resultQuat.m_w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, resultQuat.m_x , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, resultQuat.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, resultQuat.m_z , ROUNDING_ERROR_BOUND);
}

TEST(SMRQuaternion, QuatToEulerAndBackY) {
  SMRQuaternion quat( SMRVector3(0,1,0), M_PI/2.0 );
  double x,y,z;
  quat.toEulerAnglesZYX(x,y,z);
  SMRQuaternion resultQuat;
  resultQuat.fromEulerAngles(x,y,z);
  ASSERT_DOUBLE_EQ(x, 0);
  ASSERT_DOUBLE_EQ(y, M_PI/2.0);
  ASSERT_DOUBLE_EQ(z, 0);
  ASSERT_NEAR(quat.m_w, resultQuat.m_w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, resultQuat.m_x , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, resultQuat.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, resultQuat.m_z , ROUNDING_ERROR_BOUND);
}

TEST(SMRQuaternion, QuatToEulerAndBackZ) {
  SMRQuaternion quat( SMRVector3(0,1,0), M_PI/2.0 );
  double x,y,z;
  quat.toEulerAnglesZYX(x,y,z);
  SMRQuaternion resultQuat;
  resultQuat.fromEulerAngles(x,y,z);
  ASSERT_DOUBLE_EQ(x, 0);
  ASSERT_DOUBLE_EQ(y, M_PI/2.0);
  ASSERT_DOUBLE_EQ(z, 0);
  ASSERT_NEAR(quat.m_w, resultQuat.m_w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, resultQuat.m_x , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, resultQuat.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, resultQuat.m_z , ROUNDING_ERROR_BOUND);
}

TEST(SMRQuaternion, QuatToEulerAndBackXY) {
  SMRQuaternion quat( SMRVector3(1,1,0), M_PI/2.0 );
  double x,y,z;
  quat.toEulerAngles(x,y,z);
  SMRQuaternion resultQuat;
  resultQuat.fromEulerAngles(x,y,z);

  ASSERT_NEAR(quat.m_w, resultQuat.m_w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, resultQuat.m_x , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, resultQuat.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, resultQuat.m_z , ROUNDING_ERROR_BOUND);
}

TEST(SMRQuaternion, QuatToEulerAndBackXYZ) {
  SMRQuaternion quat( SMRVector3(1,1,1), M_PI/4.0 );
  double x,y,z,a,b,c;
  quat.normalize();
  quat.toEulerAngles(x,y,z);
  SMRQuaternion resultQuat;
  resultQuat.fromEulerAngles(x,y,z);
  resultQuat.toEulerAngles(a,b,c);
  std::cerr<<quat.m_w<<","<<resultQuat.m_w<<"\n"<<quat.m_x<<","<<resultQuat.m_x<<"\n"<<quat.m_y<<","<<resultQuat.m_y<<"\n"<<quat.m_z<<","<<resultQuat.m_z<<"\n";
  ASSERT_NEAR(quat.m_w, resultQuat.m_w, ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_x, resultQuat.m_x , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_y, resultQuat.m_y , ROUNDING_ERROR_BOUND);
  ASSERT_NEAR(quat.m_z, resultQuat.m_z , ROUNDING_ERROR_BOUND);
  ASSERT_DOUBLE_EQ(x,a);
  ASSERT_DOUBLE_EQ(y, b);
  ASSERT_DOUBLE_EQ(z, c);

}
TEST(SMRQuaternion, EulerAngles_ToZYX) {
  SMRQuaternion quat( SMRVector3(1,0,0), M_PI/2.0 );
  double x,y,z;
  quat.toEulerAnglesZYX(x,y,z);
  
  ASSERT_DOUBLE_EQ(x, M_PI/2);
  ASSERT_DOUBLE_EQ(y, 0);          
  ASSERT_DOUBLE_EQ(z, 0);
}

TEST(SMRQuaternion, EulerAngles_ToYXZ) {
  SMRQuaternion quat( SMRVector3(1,0,0), M_PI/2 );
  double x,y,z;
  quat.toEulerAnglesYXZ(x,y,z);
  ASSERT_DOUBLE_EQ(x, M_PI/2);
  ASSERT_DOUBLE_EQ(y, 0);
  ASSERT_DOUBLE_EQ(z, 0);
}

TEST(SMRQuaternion, EulerAngles_ToXZY) {
  SMRQuaternion quat( SMRVector3(1,0,0), M_PI/2 );
  double x,y,z;
  quat.toEulerAnglesXZY(x,y,z);
  ASSERT_DOUBLE_EQ(x, M_PI/2);
  ASSERT_DOUBLE_EQ(y, 0);
  ASSERT_DOUBLE_EQ(z, 0);
}

TEST(SMRQuaternion, GetRotationAngle) {
  SMRQuaternion quat( SMRVector3(1,0,0), M_PI/2 );
  ASSERT_DOUBLE_EQ(quat.getRotationAngle(), M_PI/2);
}

TEST(SMRQuaternion, GetRotationAxis) {
  SMRQuaternion quat( SMRVector3(1,0,0), M_PI/2 );
  SMRVector3 axis=quat.getRotationAxis();
  ASSERT_DOUBLE_EQ(axis.m_x, 1);
  ASSERT_DOUBLE_EQ(axis.m_y, 0);
  ASSERT_DOUBLE_EQ(axis.m_z, 0);
}

TEST(SMRQuaternion, Operator_Addition) {
  SMRQuaternion quat1( 1, 2, 3, 4 );
  SMRQuaternion quat2( 4, 3, 2, 1 );
  SMRQuaternion quat = quat1 + quat2;
  ASSERT_DOUBLE_EQ(quat.m_w, 5);
  ASSERT_DOUBLE_EQ(quat.m_x, 5);
  ASSERT_DOUBLE_EQ(quat.m_y, 5);
  ASSERT_DOUBLE_EQ(quat.m_z, 5);
}

TEST(SMRQuaternion, Operator_AssignmentByAddition) {
  SMRQuaternion quat1( 1, 2, 3, 4 );
  SMRQuaternion quat2( 4, 3, 2, 1 );
  quat1 += quat2;
  ASSERT_DOUBLE_EQ(quat1.m_w, 5);
  ASSERT_DOUBLE_EQ(quat1.m_x, 5);
  ASSERT_DOUBLE_EQ(quat1.m_y, 5);
  ASSERT_DOUBLE_EQ(quat1.m_z, 5);
}

TEST(SMRQuaternion, Operator_Subtraction) {
  SMRQuaternion quat1( 4, 3, 2, 1 );
  SMRQuaternion quat2( 4, 3, 2, 1 );
  SMRQuaternion quat = quat1 - quat2;
  ASSERT_DOUBLE_EQ(quat.m_w, 0);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Operator_AssignmentBySubtraction) {
  SMRQuaternion quat1( 4, 3, 2, 1 );
  SMRQuaternion quat2( 4, 3, 2, 1 );
  quat1 -= quat2;
  ASSERT_DOUBLE_EQ(quat1.m_w, 0);
  ASSERT_DOUBLE_EQ(quat1.m_x, 0);
  ASSERT_DOUBLE_EQ(quat1.m_y, 0);
  ASSERT_DOUBLE_EQ(quat1.m_z, 0);
}

TEST(SMRQuaternion, Operator_Multiplication) {
  SMRQuaternion quat1( 4, 3, 2, 1 );
  SMRQuaternion quat2( 4, 3, 2, 1 );
  SMRQuaternion quat = quat1 * quat2;
  ASSERT_DOUBLE_EQ(quat.m_w, 2);
  ASSERT_DOUBLE_EQ(quat.m_x, 24);
  ASSERT_DOUBLE_EQ(quat.m_y, 16);
  ASSERT_DOUBLE_EQ(quat.m_z, 8);
}

TEST(SMRQuaternion, Operator_AssignmentByMultiplication) {
  SMRQuaternion quat1( 4, 3, 2, 1 );
  SMRQuaternion quat2( 4, 3, 2, 1 );
  quat1 *= quat2;
  ASSERT_DOUBLE_EQ(quat1.m_w, 2);
  ASSERT_DOUBLE_EQ(quat1.m_x, 24);
  ASSERT_DOUBLE_EQ(quat1.m_y, 16);
  ASSERT_DOUBLE_EQ(quat1.m_z, 8);
}

TEST(SMRQuaternion, Operator_Multiplication_Scalar) {
  SMRQuaternion quat1( 4, 3, 2, 1 );
  SMRQuaternion quat = quat1 * 2;
  ASSERT_DOUBLE_EQ(quat.m_w, 8);
  ASSERT_DOUBLE_EQ(quat.m_x, 6);
  ASSERT_DOUBLE_EQ(quat.m_y, 4);
  ASSERT_DOUBLE_EQ(quat.m_z, 2);
}

TEST(SMRQuaternion, Operator_AssignmentByMultiplication_Scalar) {
  SMRQuaternion quat( 4, 3, 2, 1 );
  quat *= 2;
  ASSERT_DOUBLE_EQ(quat.m_w, 8);
  ASSERT_DOUBLE_EQ(quat.m_x, 6);
  ASSERT_DOUBLE_EQ(quat.m_y, 4);
  ASSERT_DOUBLE_EQ(quat.m_z, 2);
}

TEST(SMRQuaternion, Operator_Multiplication_Vector) {
  SMRQuaternion quat1( 4, 3, 2, 1 );
  SMRVector3 v( 3, 2, 1 );
  SMRQuaternion quat = quat1 * v;
  ASSERT_DOUBLE_EQ(quat.m_w, -14);
  ASSERT_DOUBLE_EQ(quat.m_x, 12);
  ASSERT_DOUBLE_EQ(quat.m_y, 8);
  ASSERT_DOUBLE_EQ(quat.m_z, 4);
}

TEST(SMRQuaternion, Operator_AssignmentByMultiplication_Vector) {
  SMRQuaternion quat( 4, 3, 2, 1 );
  SMRVector3 v( 3, 2, 1 );
  quat *= v;
  ASSERT_DOUBLE_EQ(quat.m_w, -14);
  ASSERT_DOUBLE_EQ(quat.m_x, 12);
  ASSERT_DOUBLE_EQ(quat.m_y, 8);
  ASSERT_DOUBLE_EQ(quat.m_z, 4);
}

TEST(SMRQuaternion, Operator_Division) {
  SMRQuaternion quat1( 8, 6, 4, 2 );
  SMRQuaternion quat = quat1 / 2;
  ASSERT_DOUBLE_EQ(quat.m_w, 4);
  ASSERT_DOUBLE_EQ(quat.m_x, 3);
  ASSERT_DOUBLE_EQ(quat.m_y, 2);
  ASSERT_DOUBLE_EQ(quat.m_z, 1);
}

TEST(SMRQuaternion, Operator_AssignmentByDivision) {
  SMRQuaternion quat( 8, 6, 4, 2 );
  quat /= 2;
  ASSERT_DOUBLE_EQ(quat.m_w, 4);
  ASSERT_DOUBLE_EQ(quat.m_x, 3);
  ASSERT_DOUBLE_EQ(quat.m_y, 2);
  ASSERT_DOUBLE_EQ(quat.m_z, 1);
}

TEST(SMRQuaternion, Operator_Conjugate) {
  SMRQuaternion quat1( 4, 3, 2, 1 );
  SMRQuaternion quat = ~quat1;
  ASSERT_DOUBLE_EQ(quat.m_w, 4);
  ASSERT_DOUBLE_EQ(quat.m_x, -3);
  ASSERT_DOUBLE_EQ(quat.m_y, -2);
  ASSERT_DOUBLE_EQ(quat.m_z, -1);
}

TEST(SMRQuaternion, Operator_Negation) {
  SMRQuaternion quat1( 4, 3, 2, 1 );
  SMRQuaternion quat = -quat1;
  ASSERT_DOUBLE_EQ(quat.m_w, -4);
  ASSERT_DOUBLE_EQ(quat.m_x, -3);
  ASSERT_DOUBLE_EQ(quat.m_y, -2);
  ASSERT_DOUBLE_EQ(quat.m_z, -1);
}

TEST(SMRQuaternion, GeodesicDistance) {
  SMRQuaternion quat1( 1, 0, 0, 0 );
  SMRQuaternion quat2( 1, 0, 0, 0 );
  ASSERT_DOUBLE_EQ(GeodesicDistance(quat1,quat2), 0);
}

TEST(SMRQuaternion, Product) {
  SMRQuaternion quat1( 4, 3, 2, 1 );
  SMRQuaternion quat = quat1 * 2;
  ASSERT_DOUBLE_EQ(quat.m_w, 8);
  ASSERT_DOUBLE_EQ(quat.m_x, 6);
  ASSERT_DOUBLE_EQ(quat.m_y, 4);
  ASSERT_DOUBLE_EQ(quat.m_z, 2);
}

TEST(SMRQuaternion, DotProduct) {
  SMRQuaternion quat1( 1, 0, 0, 0 );
  SMRQuaternion quat2( 1, 0, 0, 0 );
  ASSERT_DOUBLE_EQ(DotProduct(quat1,quat2), 1);
}

TEST(SMRQuaternion, Conjugate) {
  SMRQuaternion quat1( 4, 3, 2, 1 );
  SMRQuaternion quat = Conjugate(quat1);
  ASSERT_DOUBLE_EQ(quat.m_w, 4);
  ASSERT_DOUBLE_EQ(quat.m_x, -3);
  ASSERT_DOUBLE_EQ(quat.m_y, -2);
  ASSERT_DOUBLE_EQ(quat.m_z, -1);
}

TEST(SMRQuaternion, Inverse) {
  SMRQuaternion quat1( 4, 3, 2, 1 );
  SMRQuaternion quat = Inverse(quat1);
  ASSERT_DOUBLE_EQ(quat.m_w, 4);
  ASSERT_DOUBLE_EQ(quat.m_x, -3);
  ASSERT_DOUBLE_EQ(quat.m_y, -2);
  ASSERT_DOUBLE_EQ(quat.m_z, -1);
}

TEST(SMRQuaternion, RotationBetweenVectors) {
//  SMRQuaternion quat = RotationBetweenVectors( SMRVector3(1,0,0), SMRVector3(0,1,0) );
  // VectorMag?
  ASSERT_DOUBLE_EQ(1, 0);
}

TEST(SMRQuaternion, Log) {
  SMRQuaternion quat( 4, 3, 2, 1 );
  SMRVector3 v = Log(quat);
  ASSERT_DOUBLE_EQ(v.m_x, 3);
  ASSERT_DOUBLE_EQ(v.m_y, 2);
  ASSERT_DOUBLE_EQ(v.m_z, 1);
}

TEST(SMRQuaternion, Exp) {
  SMRQuaternion quat1( 1, 0, 0, 0 );
  SMRQuaternion quat = Exp(quat1);
  ASSERT_DOUBLE_EQ(quat.m_w, 0.54030230586813977);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Exp_Vector) {
  SMRQuaternion quat = Exp( SMRVector3(1,0,0) );
  ASSERT_DOUBLE_EQ(quat.m_w, 0.54030230586813977);
  ASSERT_DOUBLE_EQ(quat.m_x, 0.84147098480789650);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Pow) {
  //SMRQuaternion quat1( 4, 3, 2, 1 );
  //double d=2.0;
  //SMRQuaternion quat = Pow(quat1,d);
  ASSERT_DOUBLE_EQ(1, 0);
}

TEST(SMRQuaternion, Slerp) {
  SMRQuaternion quat1( 1, 0, 0, 0 );
  SMRQuaternion quat2( 1, 0, 0, 0 );
  SMRQuaternion quat = Slerp(quat1,quat2,0.0);
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Squad) {
  SMRQuaternion quat1( 1, 0, 0, 0 );
  SMRQuaternion quatA( 1, 0, 0, 0 );
  SMRQuaternion quatB( 1, 0, 0, 0 );
  SMRQuaternion quat2( 1, 0, 0, 0 );
  
  SMRQuaternion quat = Squad(quat1,quatA,quatB,quat2,0.0);
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Intermediate) {
  SMRQuaternion qPrev( 1, 0, 0, 0 );
  SMRQuaternion qCurr( 1, 0, 0, 0 );
  SMRQuaternion qNext( 1, 0, 0, 0 );
  SMRQuaternion qA( 1, 0, 0, 0 );
  SMRQuaternion qB( 1, 0, 0, 0 );
  
  Intermediate(qPrev,qCurr,qNext,qA,qB,0,0,0,0,0,0);
  ASSERT_DOUBLE_EQ(qA.m_w, 1);
  ASSERT_DOUBLE_EQ(qA.m_x, 0);
  ASSERT_DOUBLE_EQ(qA.m_y, 0);
  ASSERT_DOUBLE_EQ(qA.m_z, 0);
  ASSERT_DOUBLE_EQ(qB.m_w, 1);
  ASSERT_DOUBLE_EQ(qB.m_x, 0);
  ASSERT_DOUBLE_EQ(qB.m_y, 0);
  ASSERT_DOUBLE_EQ(qB.m_z, 0);
}

TEST(SMRQuaternion, ComputeTangentsAB) {
  SMRQuaternion qPrev( 1, 0, 0, 0 );
  SMRQuaternion q0( 1, 0, 0, 0 );
  SMRQuaternion q1( 1, 0, 0, 0 );
  SMRQuaternion qNext( 1, 0, 0, 0 );
  SMRQuaternion qA( 1, 0, 0, 0 );
  SMRQuaternion qB( 1, 0, 0, 0 );
  
  /*ComputeTangentsAB(qPrev,q0,q1,qNext,qA,qB);
  ASSERT_DOUBLE_EQ(qA.m_w, 1);
  ASSERT_DOUBLE_EQ(qA.m_x, 0);
  ASSERT_DOUBLE_EQ(qA.m_y, 0);
  ASSERT_DOUBLE_EQ(qA.m_z, 0);
  ASSERT_DOUBLE_EQ(qB.m_w, 1);
  ASSERT_DOUBLE_EQ(qB.m_x, 0);
  ASSERT_DOUBLE_EQ(qB.m_y, 0);
  ASSERT_DOUBLE_EQ(qB.m_z, 0);*/
  ASSERT_DOUBLE_EQ(0, 1);
}

TEST(SMRQuaternion, Interpolate) {
  SMRQuaternion q1( 1, 0, 0, 0 );
  SMRQuaternion q2( 1, 0, 0, 0 );
  SMRQuaternion q3( 1, 0, 0, 0 );
  SMRQuaternion q4( 1, 0, 0, 0 );
  SMRQuaternion quat = Interpolate(q1,q2,q3,q4,0.0);
  
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Filter) {
  vector<SMRQuaternion> window(1);
  window[0]=SMRQuaternion(1, 0, 0, 0);
  vector<double> coef( 1 );
  coef[0] = 1.0;
  SMRQuaternion quat = filterQuaternions(window,coef);
  
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Smooth) {
  SMRQuaternion q0( 1, 0, 0, 0 );
  SMRQuaternion q1( 1, 0, 0, 0 );
  SMRQuaternion q2( 1, 0, 0, 0 );
  SMRQuaternion q3( 1, 0, 0, 0 );
  SMRQuaternion quat = smoothQuaternions(q0,q1,q2,q3);
  
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

TEST(SMRQuaternion, Blur) {
  SMRQuaternion q0( 1, 0, 0, 0 );
  SMRQuaternion q1( 1, 0, 0, 0 );
  SMRQuaternion q2( 1, 0, 0, 0 );
  SMRQuaternion q3( 1, 0, 0, 0 );
  SMRQuaternion quat = blurQuaternions(q0,q1,q2,q3);
  
  ASSERT_DOUBLE_EQ(quat.m_w, 1);
  ASSERT_DOUBLE_EQ(quat.m_x, 0);
  ASSERT_DOUBLE_EQ(quat.m_y, 0);
  ASSERT_DOUBLE_EQ(quat.m_z, 0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

