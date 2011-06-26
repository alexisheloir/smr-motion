/**
 * \ingroup SmrMath
 * \file SMRVector3.h
 *  This file contains all the definition related to a 
 *  SMRVector3
 */

#ifndef __SMRVECTOR3__H__
#define __SMRVECTOR3__H__
#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "newmat.h"

 /*! \class SMRVector3T
  *  \brief This class defines a templated vector in 3D space
  */
template <typename Real>
class SMRVector3T
{
public:
  // Constructors
  SMRVector3T( void ) { }
  SMRVector3T( Real _x, Real _y, Real _z )
    : m_x(_x), m_y(_y), m_z(_z) { }

public:
  // accessors
  inline const Real X() const {return m_x;};
  inline const Real Y() const {return m_y;};
  inline const Real Z() const {return m_z;};

  /// initialization
  inline void init(Real x, Real y, Real z ){m_x=x;m_y=y;m_z=z;};
public:
  /// vector comparison
  bool operator==( const SMRVector3T<Real> &v ) const;
  bool operator!=( const SMRVector3T<Real> &v ) const;

  /// vector negation
  SMRVector3T<Real> operator-( void ) const;

  // Vector operations
  SMRVector3T<Real> operator+( const SMRVector3T<Real> &v ) const;
  SMRVector3T<Real> operator-( const SMRVector3T<Real> &v ) const;
  SMRVector3T<Real> operator*( Real s ) const;
  SMRVector3T<Real> operator/( Real s ) const;
  SMRVector3T<Real> &operator+=( const SMRVector3T<Real> &v );
  SMRVector3T<Real> &operator-=( const SMRVector3T<Real> &v );
  SMRVector3T<Real> &operator*=( Real s );
  SMRVector3T<Real> &operator/=( Real s );

  ///SMRVector3 streaming opreation
  friend std::ostream & operator<<(std::ostream & os , const SMRVector3T<Real> &v)
  {
    os << v.m_x << " " << v.m_y << " " << v.m_z << std::endl;
    return os;
  }

public:
  ///test if vector is null vector
  bool isZero( void );
  ///Normalization operation
  SMRVector3T<Real> & normalize( void );
  ///returns vector norm 
  Real norm() const;
  
public:
  // Member variables
  Real m_x;
  Real m_y;
  Real m_z;
};


//
// Nonmember Vector3 functions
//

/// scalar-vector product
template <typename Real> SMRVector3T<Real> operator*( Real k, SMRVector3T<Real> v );
/// vector-vector dot product
template <typename Real> Real DotProduct( const SMRVector3T<Real> &a, const SMRVector3T<Real> &b );
/// vector-vector cross product
template <typename Real> SMRVector3T<Real> CrossProduct( const SMRVector3T<Real> &a, const SMRVector3T<Real> &b );
/// compute a plane normal from 3 points
template <typename Real> SMRVector3T<Real> ComputeNormal( const SMRVector3T<Real> &p1,const SMRVector3T<Real> &p2, const SMRVector3T<Real> &p3 );
/// compute distance between two points
template <typename Real> Real Distance( const SMRVector3T<Real> &a, const SMRVector3T<Real> &b );
/// vector norm
template <typename Real> Real Norm( const SMRVector3T<Real> &a );
/// compute squared distance between two points
template <typename Real> Real DistanceSquared( const SMRVector3T<Real> &a, const SMRVector3T<Real> &b );

template <typename Real>
SMRVector3T<Real> TBCInterpolation(const SMRVector3T<Real> &_Xim1,
                                   const SMRVector3T<Real> &Xi,
                                   const SMRVector3T<Real> &Xip1,
                                   const SMRVector3T<Real> &Xip2,
                                   const double _tension,
                                   const double _bias, 
                                   const double _continuity,
                                   const double alpha);

/// Typical use of a double-precision vector
typedef SMRVector3T<double> SMRVector3;


#include "SmrVector3.inl"

#endif
