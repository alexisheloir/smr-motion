/**
* \ingroup SmrMath
* \file SMRQuaternion.h
* 
*/

#ifndef __SMRQUATERNION__H__
#define __SMRQUATERNION__H__
//#pragma once

#include "SmrMath.h"

#include "SmrSTL.h"
#include "SmrVector3.h"
#include "SmrTimeSerie.h"

/*! \class SMRQuaternionT
*  \brief This class defines a quaternionic value, usually used to represent
*         a rotation in 3D
*/

template <typename Real>
class SMRQuaternionT
{
  // public data
public:
  ///real part
  Real m_w; 
  ///imaginary part
  Real m_x; 
  Real m_y; 
  Real m_z; 

public:
  ///quaternion empty constructor
  SMRQuaternionT(void):m_w(1.0),m_x(0.0),m_y(0.0),m_z(0.0){};
  SMRQuaternionT(Real w, Real x, Real y, Real z)
    : m_w(w), m_x(x), m_y(y), m_z(z) { };
  ///built an unit quaternion from x,y,and z Caution! this is not from euler angles, use fromEulerAngles() methods instead
  SMRQuaternionT(Real x, Real y, Real z)
    :  m_x(x), m_y(y), m_z(z) {computeW(); };
  ///quaternion construction from axis + angle
  SMRQuaternionT(SMRVector3 axis, Real angle);
  ///quaternion destructor
  ~SMRQuaternionT(){};


  // public methods
public:
  SMRQuaternionT<Real> & identity( void );
  SMRQuaternionT<Real> & normalize( void );

  void computeW( void );
  void rotate( SMRVector3T<Real> &v ) const;

  /// return norm and square Norm of the quaternion
  Real squareNorm( void );
  Real norm( void );

  /// Quaternion <- Euler conversions; XYZ rotation order; angles in radians
  void fromEulerAngles( Real x, Real y, Real z );
  /// Quaternion -> Euler conversions; XYZ rotation order; angles in radians
  void toEulerAngles( Real &x, Real &y, Real &z ) const;
  void toEulerAnglesZYX( Real &z, Real &y, Real &x );
  void toEulerAnglesYXZ( Real &x, Real &y, Real &z ) const;
  void toEulerAnglesXZY( Real &x, Real &y, Real &z ) const;



  Real getRotationAngle( void ) const;
  SMRVector3 getRotationAxis( void ) const;

  // Quaternion operations
  SMRQuaternionT<Real> operator+( const SMRQuaternionT<Real> &q ) const;
  SMRQuaternionT<Real> &operator+=( const SMRQuaternionT<Real> &q );
  SMRQuaternionT<Real> operator-( const SMRQuaternionT<Real> &q ) const;
  SMRQuaternionT<Real> &operator-=( const SMRQuaternionT<Real> &q );
  SMRQuaternionT<Real> operator*( const SMRQuaternionT<Real> &q ) const;
  SMRQuaternionT<Real> &operator*=( const SMRQuaternionT<Real> &q );
  SMRQuaternionT<Real> operator*( Real k ) const;
  SMRQuaternionT<Real> &operator*=( Real k );
  SMRQuaternionT<Real> operator*( const SMRVector3T<Real> &v ) const;
  SMRQuaternionT<Real> &operator*=( const SMRVector3T<Real> &v );
  SMRQuaternionT<Real> operator/( Real k ) const;
  SMRQuaternionT<Real> &operator/=( Real k );
  ///quaternion conjugate
  SMRQuaternionT<Real> operator~( void ) const; 
  ///quaternion negation
  SMRQuaternionT<Real> operator-( void ) const; 

  ///quaternion streaming opreation
  friend ostream & operator<<(ostream& os, const SMRQuaternionT<Real> & _q){
    os << "Quaternion (w, x,y,z) : " << _q.m_w << "   " << 
      _q.m_x << " " << 
      _q.m_y << " " <<
      _q.m_z << " " << endl;
    return os;
  }
};

// -------------------- Nonmember SMRQuaternionT functions

///compute the geodesic distance between two quaternions
template <typename Real> Real GeodesicDistance( const SMRQuaternionT<Real> &a, const SMRQuaternionT<Real> &b );
///scalar quaternion product
template <typename Real> SMRQuaternionT<Real> operator*( Real k, const SMRQuaternionT<Real> &q );
///quaternions dotproduct
template <typename Real> Real DotProduct( const SMRQuaternionT<Real> &a, const SMRQuaternionT<Real> &b );
///quaternion conjugate
template <typename Real> SMRQuaternionT<Real> Conjugate( const SMRQuaternionT<Real> &q );
///quaternion inverse
template <typename Real> SMRQuaternionT<Real> Inverse( const SMRQuaternionT<Real> &q );
///evaluate rotation (quaternion) between two vectors
template <typename Real> SMRQuaternionT<Real> RotationBetweenVectors( const SMRVector3T<Real> &axis1,const SMRVector3T<Real> &axis2);
///quaternion Log map
template <typename Real> SMRVector3T<Real> Log( const SMRQuaternionT<Real> &q );
///quaternion exp map
template <typename Real> SMRQuaternionT<Real> Exp( const SMRQuaternionT<Real> &q );
///quaternion exp map from a 3D vector expressed in tangent space
template <typename Real> SMRQuaternionT<Real> Exp( const SMRVector3T<Real> &v);
///quaternion exponentiation
template <typename Real> SMRQuaternionT<Real> Pow( const SMRQuaternionT<Real> &q, Real exponent );

// -------------------- Interpolation ---------------------------------------
///quaternions spherical interpolation
template <typename Real> SMRQuaternionT<Real> Slerp( const SMRQuaternionT<Real> &q0, const SMRQuaternionT<Real> &q1, Real t, bool _hemispherize=true );
///quaternions cubical interpolation
template <typename Real> SMRQuaternionT<Real> Squad( const SMRQuaternionT<Real> &q0, const SMRQuaternionT<Real> &qa, const SMRQuaternionT<Real> &qb, const SMRQuaternionT<Real> &q1, Real t );
///quaternions interpolation const double & _sampleTime)
///compute tangents for quaternion interpolation
template <typename Real> void Intermediate( const SMRQuaternionT<Real> &qprev, const SMRQuaternionT<Real> &qcurr,const SMRQuaternionT<Real> &qnext, SMRQuaternionT<Real> &qa,SMRQuaternionT<Real> &qb,const double &_tension, const double &_bias, const double &_continuity, const double &_tqprev,const double &_tqcurr,const double &_tqnext );
///compute tangents
template <typename Real> void ComputeTangentsAB(const SMRQuaternionT<Real> & qprev, const SMRQuaternionT<Real> & q0, const SMRQuaternionT<Real> & q1,const SMRQuaternionT<Real> & qnext, SMRQuaternionT<Real> & a, SMRQuaternionT<Real> & b);

template <typename Real> inline SMRQuaternionT<Real> Interpolate( const SMRQuaternionT<Real> &firstQuat, const SMRQuaternionT<Real> &secondQuat, \
                                                                 const SMRQuaternionT<Real> &thirdQuat, const SMRQuaternionT<Real> &fourthQuat, const double &t, const double &_tension=0.0, const double &_bias=0.0, const double &_continuity=0.0);



// -------------------- filtering quaternion values -------- from Lee 2002 TVCG
/// performs convolution of a quaternionic signal around the quaternionic window qwindow and with coefficients coef
template <typename Real> SMRQuaternionT<Real> filterQuaternions( const vector<SMRQuaternionT<Real> > & qwindow, const vector<double> & coef);
///quaternions smoothing operation
template <typename Real> SMRQuaternionT<Real> smoothQuaternions( const SMRQuaternionT<Real> & q0, const SMRQuaternionT<Real> & q1, const SMRQuaternionT<Real> & q2, const SMRQuaternionT<Real> & q3,double smooth = 0.1);
///quaternions blurring operation
template <typename Real> SMRQuaternionT<Real> blurQuaternions( const SMRQuaternionT<Real> & q0, const SMRQuaternionT<Real> & q1, const SMRQuaternionT<Real> & q2, const SMRQuaternionT<Real> & q3);

/// Typical use of a double-precision quaternion
typedef SMRQuaternionT<double> SMRQuaternion;

#include "SmrQuaternion.inl"

#endif

