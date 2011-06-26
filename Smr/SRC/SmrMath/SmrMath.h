/**
 *  \defgroup SmrMath Smr Math routines
 *  \file SmrMath.h
 *  \brief Definition file for Smr Math library.
 */
#ifndef SMRMATH_H
#define SMRMATH_H
//#pragma once

#include <math.h>
#include <assert.h>

//#include "SmrVector3.h"
//#include "SmrQuaternion.h"
//#include "SmrTimeSerie.h"
//#include "SmrRandom.h"
//#include "SmrPCA.h"
//#include "SmrDTW.h"



#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

#ifndef M_PIover2
#define M_PIover2 M_PI/2.0
#endif

#ifndef M_2PI
#define M_2PI 2.0*M_PI
#endif

#ifndef invM_PI
#define invM_PI 1.0/M_PI
#endif

#ifndef invM_2PI
#define invM_2PI 1.0/M_2PI
#endif

#ifdef SQR
#undef SQR
#endif
#define SQR(x) ((x)*(x))


#ifdef MIN
#undef MIN
#endif
#define MIN(x,y) ((x<y)?x:y)

#ifdef MAX
#undef MAX
#endif
#define MAX(x,y) ((x>y)?x:y)

#ifdef PI
#undef PI
#endif
#define PI 3.1415927


#ifdef deg
#undef deg
#endif
#define deg(x) ((x)*180.0/M_PI)


#ifdef rad
#undef rad
#endif
#define rad(x) ((x)*M_PI/180.0)

/**
 *  \brief Sign function.
 *  sign function
 *  \param theta is a number
 *  \return 1 or -1 given the sign of the number
 */
template <typename Real> Real sign( Real theta )
{
  if (theta<0) return -1; 
  if (theta>0) return 1; 
  else return 0; 
}

/**
 *  \brief Wrapping function.
 *
 *  wrapping function
 *  \param theta is a number
 *  \return wrap an angle between \f$-\pi\f$ and \f$\pi\f$
 */
template <typename Real> Real wrapPi( Real theta )
{
  theta += M_PI;
  theta -= floor( theta * invM_2PI ) * M_2PI;
  theta -= M_PI;
  return theta;
}

/**
 *  \brief Safe acos function.
 *
 *  safe acos function
 *  \param x is a number
 *  \return returns acos(x) along a test on x
 */
template <typename Real> Real safeAcos( Real x )
{
  // Check limit conditions
  if( x <= -1.0 ) {
  return M_PI;
  }
  if( x >= 1.0 ) {
  return 0.0;
  }
  // value is in the domain - use standard C function
  return acos(x);
}

/**
 *  \brief Check if Real is NaN.
 *
 *  check if Real is NaN
 */
template <typename Real> bool isNaN(const Real &a)
{
  bool b1  = (a < 0.0);
  bool b2  = (a >= 0.0);
  bool b3  = !(b1 || b2);
  return b3;
}

template <typename Real> Real CubicInterpolate( Real y0, Real y1, Real y2, Real y3, Real mu)
{
   Real a0,a1,a2,a3,mu2;

   mu2 = mu*mu;
   a0 = y3 - y2 - y0 + y1;
   a1 = y0 - y1 - a0;
   a2 = y2 - y0;
   a3 = y1;

   return(a0*mu*mu2+a1*mu2+a2*mu+a3);
}

template <typename Real>
class SMRTimeWarpPath
{
public:
  SMRTimeWarpPath():m_y1(0.33),m_y2(0.66)
  {
  }
  SMRTimeWarpPath(Real _y1, Real _y2):m_y1(_y1),m_y2(_y2)
  {
  }
private:
  Real m_y1;
  Real m_y2;
public:
  Real evaluate(double _mu)
  {
    assert(_mu > -0.001);
    assert(_mu < 1.0001);

    if (_mu < 0.333333)
    {
      return CubicInterpolate(0.0,0.0,m_y1,m_y2,_mu*3);
    }else if (_mu <0.666666)
    {
      return CubicInterpolate(0.0,m_y1,m_y2,1.0,(_mu-0.333333)*3);
    }else if (_mu < 1.0)
    {
      return CubicInterpolate(m_y1,m_y2,1.0,1.0,(_mu-0.666666)*3);
    }else
    {
      return 1.0;
    }
  }

};

/**
 *  \brief Hermite polynomial.
 *
 *  basic Hermite polynomial: http://local.wasp.uwa.edu.au/~pbourke/miscellaneous/interpolation/
 */
template <typename Real>
class SMRHermitePoly
{
public:  
  /**
   *  \brief Default constructor.
   */
  SMRHermitePoly(): m_p0(-1), m_p1(0), m_p2(1), m_p3(2),m_bias(0),m_tension(0), m_continuity(0)
  {

  }

  /**
   *  \brief Constructor.
   */
  SMRHermitePoly(const Real p0, const Real p1, const Real p2, const Real p3): m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3), m_bias(0), m_tension(0), m_continuity(0)
  {
  }

  /**
   *  \brief Destructor.
   */
  ~SMRHermitePoly()
  {

  }

private:
  /**
   *  \brief p0
   */
  Real m_p0;

  /**
   *  \brief p1
   */
  Real m_p1;

  /**
   *  \brief p2
   */
  Real m_p2;

  /**
   *  \brief p3
   */
  Real m_p3;

  /**
   *  \brief Bias
   */
  Real m_bias;

  /**
   *  \brief Tension
   */

  Real m_tension;

  /**
   * \brief Continuity
   */

  Real m_continuity;

public:
  /**
   *  \brief Evalutes.
   */
  Real evaluate(const Real _mu)
  {
    Real m0,m1,mu2,mu3;
    double a0,a1,a2,a3;

    mu2 =  _mu * _mu;
    mu3 =  mu2 * _mu;
    m0  = (m_p1-m_p0)*(1+m_bias)*(1-m_tension)/2;
    m0 += (m_p2-m_p1)*(1-m_bias)*(1-m_tension)/2;
    m1  = (m_p2-m_p1)*(1+m_bias)*(1-m_tension)/2;
    m1 += (m_p3-m_p2)*(1-m_bias)*(1-m_tension)/2;
    a0  =  2*mu3 - 3*mu2 + 1;
    a1  =    mu3 - 2*mu2 + _mu;
    a2  = -2*mu3 + 3*mu2;
    a3  =    mu3 -   mu2;

    return(a0*m_p1+a1*m0+a2*m1+a3*m_p2);
  }

  /**
   *  \brief Sets the tension.
   */
  void setTension(const Real _tension)
  {
    m_tension = _tension;
  }

  void setBias(const Real _bias)
  {
    m_bias = _bias;
  }

  void setContinuity(const Real _continuity)
  {
    m_continuity = _continuity;
  }

};

class QMatrix4f
{
private:

	QMatrix4f( bool /*noInit*/ )
	{
		// Don't initialize the matrix
	}

public:
	
	union
	{
		float c[4][4];	// Column major order for OpenGL: c[column][row]
		float x[16];
	};
	
	// --------------
	// Static methods
	// --------------
	static QMatrix4f TransMat( float x, float y, float z )
	{
		QMatrix4f m;

		m.c[3][0] = x;
		m.c[3][1] = y;
		m.c[3][2] = z;

		return m;
	}

	//static QMatrix4f TransMat( const QVec3f& trans )
	//{
	//	QMatrix4f m;

	//	m.c[3][0] = trans.X;
	//	m.c[3][1] = trans.Y;
	//	m.c[3][2] = trans.Z;

	//	return m;
	//}

	static QMatrix4f ScaleMat( float x, float y, float z )
	{
		QMatrix4f m;
		
		m.c[0][0] = x;
		m.c[1][1] = y;
		m.c[2][2] = z;

		return m;
	}

	//static QMatrix4f ScaleMat( const QVec3f& scale )
	//{
	//	QMatrix4f m;
	//	
	//	m.c[0][0] = scale.X;
	//	m.c[1][1] = scale.Y;
	//	m.c[2][2] = scale.Z;

	//	return m;
	//}

	//static QMatrix4f RotMat( float x, float y, float z )
	//{
	//	// Rotation order: YXZ [* Vector]
	//	return QMatrix4f( QQuaternion( x, y, z ) );
	//}

	//static QMatrix4f RotMat( QVec3f axis, float angle )
	//{
	//	axis = axis * sinf( angle / 2 );
	//	return QMatrix4f( QQuaternion( axis.X, axis.Y, axis.Z, cosf( angle / 2 ) ) );
	//}

	//static QMatrix4f RotMat( QVec3f angles )
	//{		
	//	return QMatrix4f( QQuaternion( angles.X, angles.Y, angles.Z ) );
	//}

	// ------------
	// Constructors
	// ------------
	QMatrix4f()
	{
		c[0][0] = 1; c[1][0] = 0; c[2][0] = 0; c[3][0] = 0;
		c[0][1] = 0; c[1][1] = 1; c[2][1] = 0; c[3][1] = 0;
		c[0][2] = 0; c[1][2] = 0; c[2][2] = 1; c[3][2] = 0;
		c[0][3] = 0; c[1][3] = 0; c[2][3] = 0; c[3][3] = 1;
	}

	QMatrix4f( const float *floatArray16 )
	{
		for( unsigned int i = 0; i < 4; ++i )
		{
			for( unsigned int j = 0; j < 4; ++j )
			{
				c[i][j] = floatArray16[i * 4 + j];
			}
		}
	}

	//QMatrix4f( const QQuaternion &q )
	//{
	//	float wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;

	//	// Calculate coefficients
	//	x2 = q.x + q.x;	y2 = q.y + q.y;	z2 = q.z + q.z;
	//	xx = q.x * x2;	xy = q.x * y2;	xz = q.x * z2;
	//	yy = q.y * y2;	yz = q.y * z2;	zz = q.z * z2;
	//	wx = q.w * x2;		wy = q.w * y2;		wz = q.w * z2;


	//	c[0][0] = 1 - (yy + zz);	c[1][0] = xy - wz;	
	//	c[2][0] = xz + wy;			c[3][0] = 0;
	//	c[0][1] = xy + wz;			c[1][1] = 1 - (xx + zz);
	//	c[2][1] = yz - wx;			c[3][1] = 0;
	//	c[0][2] = xz - wy;			c[1][2] = yz + wx;
	//	c[2][2] = 1 - (xx + yy);	c[3][2] = 0;
	//	c[0][3] = 0;				c[1][3] = 0;
	//	c[2][3] = 0;				c[3][3] = 1;
	//}

	// ---------------------
	// Matrix multiplication
	// ---------------------
	QMatrix4f operator*( const QMatrix4f &m ) const 
	{
		QMatrix4f mf( false );
		
		mf.x[0] = x[0] * m.x[0] + x[4] * m.x[1] + x[8] * m.x[2] + x[12] * m.x[3];
		mf.x[1] = x[1] * m.x[0] + x[5] * m.x[1] + x[9] * m.x[2] + x[13] * m.x[3];
		mf.x[2] = x[2] * m.x[0] + x[6] * m.x[1] + x[10] * m.x[2] + x[14] * m.x[3];
		mf.x[3] = x[3] * m.x[0] + x[7] * m.x[1] + x[11] * m.x[2] + x[15] * m.x[3];

		mf.x[4] = x[0] * m.x[4] + x[4] * m.x[5] + x[8] * m.x[6] + x[12] * m.x[7];
		mf.x[5] = x[1] * m.x[4] + x[5] * m.x[5] + x[9] * m.x[6] + x[13] * m.x[7];
		mf.x[6] = x[2] * m.x[4] + x[6] * m.x[5] + x[10] * m.x[6] + x[14] * m.x[7];
		mf.x[7] = x[3] * m.x[4] + x[7] * m.x[5] + x[11] * m.x[6] + x[15] * m.x[7];

		mf.x[8] = x[0] * m.x[8] + x[4] * m.x[9] + x[8] * m.x[10] + x[12] * m.x[11];
		mf.x[9] = x[1] * m.x[8] + x[5] * m.x[9] + x[9] * m.x[10] + x[13] * m.x[11];
		mf.x[10] = x[2] * m.x[8] + x[6] * m.x[9] + x[10] * m.x[10] + x[14] * m.x[11];
		mf.x[11] = x[3] * m.x[8] + x[7] * m.x[9] + x[11] * m.x[10] + x[15] * m.x[11];

		mf.x[12] = x[0] * m.x[12] + x[4] * m.x[13] + x[8] * m.x[14] + x[12] * m.x[15];
		mf.x[13] = x[1] * m.x[12] + x[5] * m.x[13] + x[9] * m.x[14] + x[13] * m.x[15];
		mf.x[14] = x[2] * m.x[12] + x[6] * m.x[13] + x[10] * m.x[14] + x[14] * m.x[15];
		mf.x[15] = x[3] * m.x[12] + x[7] * m.x[13] + x[11] * m.x[14] + x[15] * m.x[15];

		return mf;
	}

	QMatrix4f operator*( const float f ) const
	{
		QMatrix4f m( *this );
		
		for( unsigned int y = 0; y < 4; ++y )
		{
			for( unsigned int x = 0; x < 4; ++x ) 
			{
				m.c[x][y] *= f;
			}
		}

		return m;
	}

	// ----------------------------
	// Vector-Matrix multiplication
	// ----------------------------
	//QVec3f operator*( const QVec3f &v ) const
	//{
	//	return QVec3f( v.X * c[0][0] + v.Y * c[1][0] + v.Z * c[2][0] + c[3][0],
	//				  v.X * c[0][1] + v.Y * c[1][1] + v.Z * c[2][1] + c[3][1],
	//				  v.X * c[0][2] + v.Y * c[1][2] + v.Z * c[2][2] + c[3][2] );
	//}

	//Vec4f operator*( const Vec4f &v ) const
	//{
	//	return Vec4f( v.x * c[0][0] + v.y * c[1][0] + v.z * c[2][0] + c[3][0],
	//				  v.x * c[0][1] + v.y * c[1][1] + v.z * c[2][1] + c[3][1],
	//				  v.x * c[0][2] + v.y * c[1][2] + v.z * c[2][2] + c[3][2],
	//				  v.x * c[0][3] + v.y * c[1][3] + v.z * c[2][3] + c[3][3] );
	//}
	
	// ---------------
	// Transformations
	// ---------------
	void translate( const float x, const float y, const float z )
	{
		*this = TransMat( x, y, z ) * *this;
	}

	//void translate( const QVec3f& trans )
	//{
	//	*this = TransMat( trans.X, trans.Y, trans.Z ) * *this;
	//}

	void scale( const float x, const float y, const float z )
	{
		*this = ScaleMat( x, y, z ) * *this;
	}

	//void scale( const QVec3f& scale )
	//{
	//	*this = ScaleMat( scale.X, scale.Y, scale.Z ) * *this;
	//}

	//void rotate( const float x, const float y, const float z )
	//{
	//	*this = RotMat( x, y, z ) * *this;
	//}

	//void rotate( const QVec3f& rot )
	//{
	//	*this = RotMat( rot.X, rot.Y, rot.Z) * *this;
	//}

	// ---------------
	// Other
	// ---------------

	QMatrix4f transposed() const
	{
		QMatrix4f m( *this );
		
		for( unsigned int y = 0; y < 4; ++y )
		{
			for( unsigned int x = y + 1; x < 4; ++x ) 
			{
				float tmp = m.c[x][y];
				m.c[x][y] = m.c[y][x];
				m.c[y][x] = tmp;
			}
		}

		return m;
	}

	float determinant() const
	{
		return 
			c[0][3]*c[1][2]*c[2][1]*c[3][0] - c[0][2]*c[1][3]*c[2][1]*c[3][0] - c[0][3]*c[1][1]*c[2][2]*c[3][0] + c[0][1]*c[1][3]*c[2][2]*c[3][0] +
			c[0][2]*c[1][1]*c[2][3]*c[3][0] - c[0][1]*c[1][2]*c[2][3]*c[3][0] - c[0][3]*c[1][2]*c[2][0]*c[3][1] + c[0][2]*c[1][3]*c[2][0]*c[3][1] +
			c[0][3]*c[1][0]*c[2][2]*c[3][1] - c[0][0]*c[1][3]*c[2][2]*c[3][1] - c[0][2]*c[1][0]*c[2][3]*c[3][1] + c[0][0]*c[1][2]*c[2][3]*c[3][1] +
			c[0][3]*c[1][1]*c[2][0]*c[3][2] - c[0][1]*c[1][3]*c[2][0]*c[3][2] - c[0][3]*c[1][0]*c[2][1]*c[3][2] + c[0][0]*c[1][3]*c[2][1]*c[3][2] +
			c[0][1]*c[1][0]*c[2][3]*c[3][2] - c[0][0]*c[1][1]*c[2][3]*c[3][2] - c[0][2]*c[1][1]*c[2][0]*c[3][3] + c[0][1]*c[1][2]*c[2][0]*c[3][3] +
			c[0][2]*c[1][0]*c[2][1]*c[3][3] - c[0][0]*c[1][2]*c[2][1]*c[3][3] - c[0][1]*c[1][0]*c[2][2]*c[3][3] + c[0][0]*c[1][1]*c[2][2]*c[3][3];
	}

	QMatrix4f inverted() const
	{
		QMatrix4f m( false );

		float d = determinant();
		if( d == 0 ) return m;
		d = 1 / d;
		
		m.c[0][0] = d * (c[1][2]*c[2][3]*c[3][1] - c[1][3]*c[2][2]*c[3][1] + c[1][3]*c[2][1]*c[3][2] - c[1][1]*c[2][3]*c[3][2] - c[1][2]*c[2][1]*c[3][3] + c[1][1]*c[2][2]*c[3][3]);
		m.c[0][1] = d * (c[0][3]*c[2][2]*c[3][1] - c[0][2]*c[2][3]*c[3][1] - c[0][3]*c[2][1]*c[3][2] + c[0][1]*c[2][3]*c[3][2] + c[0][2]*c[2][1]*c[3][3] - c[0][1]*c[2][2]*c[3][3]);
		m.c[0][2] = d * (c[0][2]*c[1][3]*c[3][1] - c[0][3]*c[1][2]*c[3][1] + c[0][3]*c[1][1]*c[3][2] - c[0][1]*c[1][3]*c[3][2] - c[0][2]*c[1][1]*c[3][3] + c[0][1]*c[1][2]*c[3][3]);
		m.c[0][3] = d * (c[0][3]*c[1][2]*c[2][1] - c[0][2]*c[1][3]*c[2][1] - c[0][3]*c[1][1]*c[2][2] + c[0][1]*c[1][3]*c[2][2] + c[0][2]*c[1][1]*c[2][3] - c[0][1]*c[1][2]*c[2][3]);
		m.c[1][0] = d * (c[1][3]*c[2][2]*c[3][0] - c[1][2]*c[2][3]*c[3][0] - c[1][3]*c[2][0]*c[3][2] + c[1][0]*c[2][3]*c[3][2] + c[1][2]*c[2][0]*c[3][3] - c[1][0]*c[2][2]*c[3][3]);
		m.c[1][1] = d * (c[0][2]*c[2][3]*c[3][0] - c[0][3]*c[2][2]*c[3][0] + c[0][3]*c[2][0]*c[3][2] - c[0][0]*c[2][3]*c[3][2] - c[0][2]*c[2][0]*c[3][3] + c[0][0]*c[2][2]*c[3][3]);
		m.c[1][2] = d * (c[0][3]*c[1][2]*c[3][0] - c[0][2]*c[1][3]*c[3][0] - c[0][3]*c[1][0]*c[3][2] + c[0][0]*c[1][3]*c[3][2] + c[0][2]*c[1][0]*c[3][3] - c[0][0]*c[1][2]*c[3][3]);
		m.c[1][3] = d * (c[0][2]*c[1][3]*c[2][0] - c[0][3]*c[1][2]*c[2][0] + c[0][3]*c[1][0]*c[2][2] - c[0][0]*c[1][3]*c[2][2] - c[0][2]*c[1][0]*c[2][3] + c[0][0]*c[1][2]*c[2][3]);
		m.c[2][0] = d * (c[1][1]*c[2][3]*c[3][0] - c[1][3]*c[2][1]*c[3][0] + c[1][3]*c[2][0]*c[3][1] - c[1][0]*c[2][3]*c[3][1] - c[1][1]*c[2][0]*c[3][3] + c[1][0]*c[2][1]*c[3][3]);
		m.c[2][1] = d * (c[0][3]*c[2][1]*c[3][0] - c[0][1]*c[2][3]*c[3][0] - c[0][3]*c[2][0]*c[3][1] + c[0][0]*c[2][3]*c[3][1] + c[0][1]*c[2][0]*c[3][3] - c[0][0]*c[2][1]*c[3][3]);
		m.c[2][2] = d * (c[0][1]*c[1][3]*c[3][0] - c[0][3]*c[1][1]*c[3][0] + c[0][3]*c[1][0]*c[3][1] - c[0][0]*c[1][3]*c[3][1] - c[0][1]*c[1][0]*c[3][3] + c[0][0]*c[1][1]*c[3][3]);
		m.c[2][3] = d * (c[0][3]*c[1][1]*c[2][0] - c[0][1]*c[1][3]*c[2][0] - c[0][3]*c[1][0]*c[2][1] + c[0][0]*c[1][3]*c[2][1] + c[0][1]*c[1][0]*c[2][3] - c[0][0]*c[1][1]*c[2][3]);
		m.c[3][0] = d * (c[1][2]*c[2][1]*c[3][0] - c[1][1]*c[2][2]*c[3][0] - c[1][2]*c[2][0]*c[3][1] + c[1][0]*c[2][2]*c[3][1] + c[1][1]*c[2][0]*c[3][2] - c[1][0]*c[2][1]*c[3][2]);
		m.c[3][1] = d * (c[0][1]*c[2][2]*c[3][0] - c[0][2]*c[2][1]*c[3][0] + c[0][2]*c[2][0]*c[3][1] - c[0][0]*c[2][2]*c[3][1] - c[0][1]*c[2][0]*c[3][2] + c[0][0]*c[2][1]*c[3][2]);
		m.c[3][2] = d * (c[0][2]*c[1][1]*c[3][0] - c[0][1]*c[1][2]*c[3][0] - c[0][2]*c[1][0]*c[3][1] + c[0][0]*c[1][2]*c[3][1] + c[0][1]*c[1][0]*c[3][2] - c[0][0]*c[1][1]*c[3][2]);
		m.c[3][3] = d * (c[0][1]*c[1][2]*c[2][0] - c[0][2]*c[1][1]*c[2][0] + c[0][2]*c[1][0]*c[2][1] - c[0][0]*c[1][2]*c[2][1] - c[0][1]*c[1][0]*c[2][2] + c[0][0]*c[1][1]*c[2][2]);
		
		return m;
	}

	//void decompose( QVec3f &trans, QVec3f &rot, QVec3f &scale ) const
	//{
	//	// Getting translation is trivial
	//	trans = QVec3f( c[3][0], c[3][1], c[3][2] );

	//	// Scale is length of columns
	//	scale.X = sqrt( c[0][0] * c[0][0] + c[0][1] * c[0][1] + c[0][2] * c[0][2] );
	//	scale.Y = sqrt( c[1][0] * c[1][0] + c[1][1] * c[1][1] + c[1][2] * c[1][2] );
	//	scale.Z = sqrt( c[2][0] * c[2][0] + c[2][1] * c[2][1] + c[2][2] * c[2][2] );

	//	if( scale.X == 0 || scale.Y == 0 || scale.Z == 0 ) return;

	//	// Detect negative scale with determinant and flip one arbitrary axis
	//	if( determinant() < 0 ) scale.X = -scale.X;

	//	// Combined rotation matrix YXZ
	//	//
	//	// Cos[y]*Cos[z]+Sin[x]*Sin[y]*Sin[z]	Cos[z]*Sin[x]*Sin[y]-Cos[y]*Sin[z]	Cos[x]*Sin[y]	
	//	// Cos[x]*Sin[z]						Cos[x]*Cos[z]						-Sin[x]
	//	// -Cos[z]*Sin[y]+Cos[y]*Sin[x]*Sin[z]	Cos[y]*Cos[z]*Sin[x]+Sin[y]*Sin[z]	Cos[x]*Cos[y]

	//	rot.X = asinf( -c[2][1] / scale.Z );
	//	
	//	// Special case: Cos[x] == 0 (when Sin[x] is +/-1)
	//	float f = fabsf( c[2][1] / scale.Z );
	//	if( f > 0.999f && f < 1.001f )
	//	{
	//		// Pin arbitrarily one of y or z to zero
	//		// Mathematical equivalent of gimbal lock
	//		rot.Y = 0;
	//		
	//		// Now: Cos[x] = 0, Sin[x] = +/-1, Cos[y] = 1, Sin[y] = 0
	//		// => m[0][0] = Cos[z] and m[1][0] = Sin[z]
	//		rot.Z = atan2f( -c[1][0] / scale.Y, c[0][0] / scale.X );
	//	}
	//	// Standard case
	//	else
	//	{
	//		rot.Y = atan2f( c[2][0] / scale.Z, c[2][2] / scale.Z );
	//		rot.Z = atan2f( c[0][1] / scale.X, c[1][1] / scale.Y );
	//	}
	//}

	//QVec3f getTranslation() const
	//{
	//	return QVec3f( x[12], x[13], x[14] );
	//}

	//QVec3f getRotation() const
	//{
	//	QVec3f scale;
	//	// Scale is length of columns
	//	scale.X = sqrt( c[0][0] * c[0][0] + c[0][1] * c[0][1] + c[0][2] * c[0][2] );
	//	scale.Y = sqrt( c[1][0] * c[1][0] + c[1][1] * c[1][1] + c[1][2] * c[1][2] );
	//	scale.Z = sqrt( c[2][0] * c[2][0] + c[2][1] * c[2][1] + c[2][2] * c[2][2] );

	//	if( scale.X == 0 || scale.Y == 0 || scale.Z == 0 ) return QVec3f();

	//	// Detect negative scale with determinant and flip one arbitrary axis
	//	if( determinant() < 0 ) scale.X = -scale.X;

	//	QVec3f rot;
	//	rot.X = asinf( -c[2][1] / scale.Z );
	//	
	//	// Special case: Cos[x] == 0 (when Sin[x] is +/-1)
	//	float f = fabsf( c[2][1] / scale.Z );
	//	if( f > 0.999f && f < 1.001f )
	//	{
	//		// Pin arbitrarily one of y or z to zero
	//		// Mathematical equivalent of gimbal lock
	//		rot.Y = 0;
	//		
	//		// Now: Cos[x] = 0, Sin[x] = +/-1, Cos[y] = 1, Sin[y] = 0
	//		// => m[0][0] = Cos[z] and m[1][0] = Sin[z]
	//		rot.Z = atan2f( -c[1][0] / scale.Y, c[0][0] / scale.X );
	//	}
	//	// Standard case
	//	else
	//	{
	//		rot.Y = atan2f( c[2][0] / scale.Z, c[2][2] / scale.Z );
	//		rot.Z = atan2f( c[0][1] / scale.X, c[1][1] / scale.Y );
	//	}
	//	return rot;
	//}

//	QVec3f getScale() const
//	{
//
//		// Scale is length of columns
//		return QVec3f(
//			sqrt( c[0][0] * c[0][0] + c[0][1] * c[0][1] + c[0][2] * c[0][2] ),
//			sqrt( c[1][0] * c[1][0] + c[1][1] * c[1][1] + c[1][2] * c[1][2] ),
//			sqrt( c[2][0] * c[2][0] + c[2][1] * c[2][1] + c[2][2] * c[2][2] ));		
//				
//	}
//
//	QMatrix4f getRotationMatrix() const
//	{
//		QVec3f scale = getScale();
//		QMatrix4f rot;
//		rot.c[0][0] = c[0][0] / scale.X; rot.c[0][1] = c[0][1] / scale.X; rot.c[0][2] = c[0][2] / scale.X;
//		rot.c[1][0] = c[1][0] / scale.X; rot.c[1][1] = c[1][1] / scale.X; rot.c[1][2] = c[1][2] / scale.Y;
//		rot.c[2][0] = c[2][0] / scale.X; rot.c[2][1] = c[2][1] / scale.X; rot.c[2][2] = c[2][2] / scale.Z;
//		return rot;
//	}
//
//	//Vec4f getCol( unsigned int col )
//	//{
//	//	return Vec4f( x[col * 4 + 0], x[col * 4 + 1], x[col * 4 + 2], x[col * 4 + 3] );
//	//}
//
//	//Vec4f getRow( unsigned int row )
//	//{
//	//	return Vec4f( x[row + 0], x[row + 4], x[row + 8], x[row + 12] );
//	//}
};

#endif
