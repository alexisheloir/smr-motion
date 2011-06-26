/////////////////////////////////////////////////////////////////////////////
//
// class SMRQuaternionT<Real> implementation.
//
/////////////////////////////////////////////////////////////////////////////

// --------------------------------------------------------------------------
// SMRQuaternionT::SMRQuaternionT(SMRVector3T vect, Real angle)
//
// Constructor based on rotation axis and angle
// --------------------------------------------------------------------------

//#ifndef M_PI
//  #define M_PI 3.141592653
//#endif

template <typename Real>
inline SMRQuaternionT<Real>::SMRQuaternionT(SMRVector3 axis, Real angle)
{
  axis.normalize();
  double sina = sin(angle/2);
  double cosa = cos(angle/2);
  m_x = axis.X() * sina;
  m_y = axis.Y() * sina;
  m_z = axis.Z() * sina;
  m_w = cosa;
  normalize();

}
// --------------------------------------------------------------------------
// SMRQuaternionT::identity
//
// Set to identity
// --------------------------------------------------------------------------

template <typename Real>
inline SMRQuaternionT<Real> & SMRQuaternionT<Real>::identity( void )
{
  m_w = 1.0; m_x = m_y = m_z = 0.0;
  return *this;
}


// --------------------------------------------------------------------------
// SMRQuaternionT::normalize
//
// "Normalize" a quaternion.  Note that normally, quaternions
// are always normalized (within limits of numerical precision).
//
// This function is provided primarily to combat floating point "error
// creep", which can occur when many successive quaternion operations
// are applied.
// --------------------------------------------------------------------------

template <typename Real>
inline SMRQuaternionT<Real> & SMRQuaternionT<Real>::normalize( void )
{
  // Compute magnitude of the quaternion
  Real mag = (m_w * m_w) + (m_x * m_x) + (m_y * m_y) + (m_z * m_z);

  if ( mag != 1.0 ) 
  {
    mag = sqrt(mag);
    // Normalize it
    Real oneOverMag = 1.0 / mag;

    m_w *= oneOverMag;
    m_x *= oneOverMag;
    m_y *= oneOverMag;
    m_z *= oneOverMag;
  }

  return *this; 
}

template <typename Real>
inline Real SMRQuaternionT<Real>::norm( void )
{
  // return magnitude of the quaternion
  return sqrt( (m_w * m_w) + (m_x * m_x) + (m_y * m_y) + (m_z * m_z) ) ;
}

template <typename Real>
inline Real SMRQuaternionT<Real>::squareNorm( void )
{
  // return square magnitude of the quaternion
  return (m_w * m_w) + (m_x * m_x) + (m_y * m_y) + (m_z * m_z);
}


// --------------------------------------------------------------------------
// Quaternion<Real>::computeW
//
// Compute the W component of a unit quaternion given its x,y,z components.
// --------------------------------------------------------------------------

template <typename Real>
inline void SMRQuaternionT<Real>::computeW( void )
{
  Real t = 1.0 - (m_x * m_x) - (m_y * m_y) - (m_z * m_z);

  if( t < 0.0 ) {
    m_w = 0.0;
  }
  else {
    m_w = -sqrt( t );
  }
}


// --------------------------------------------------------------------------
// Quaternion<Real>::rotate
//
// Rotate a point by quaternion.  v' = q * qv * invSmr(q), where qv = <0, v>.
// --------------------------------------------------------------------------

template <typename Real>
void SMRQuaternionT<Real>::rotate( SMRVector3T<Real> &v ) const
{
  SMRQuaternionT<Real> qf = *this * v * Inverse( *this );
  v.m_x = qf.m_x;
  v.m_y = qf.m_y;
  v.m_z = qf.m_z;
}



// --------------------------------------------------------------------------
// Quaternion::fromEulerAngles
//
// Setup the quaternion to perform an object->inertial rotation, given the
// orientation in XYZ-Euler angles format.  x,y,z parameters must be in
// radians.
// --------------------------------------------------------------------------

template <typename Real>
inline void SMRQuaternionT<Real>::fromEulerAngles( Real x, Real y, Real z )
{
  // Compute sine and cosine of the half angles
  Real sr = sin( x * 0.5 );
  Real cr = cos( x * 0.5 );
  Real sp = sin( y * 0.5 );
  Real cp = cos( y * 0.5 );
  Real sy = sin( z * 0.5 );
  Real cy = cos( z * 0.5 );

  // Compute values
  m_w =  (cy * cp * cr) + (sy * sp * sr);
  m_x = -(sy * sp * cr) + (cy * cp * sr);
  m_y =  (cy * sp * cr) + (sy * cp * sr);
  m_z = -(cy * sp * sr) + (sy * cp * cr);

  //SMRQuaternion rX(SMRVector3(1.0,0.0,0.0),x);
  //SMRQuaternion rY(SMRVector3(0.0,1.0,0.0),y);
  //SMRQuaternion rZ(SMRVector3(0.0,0.0,1.0),z);

  //*this = rZ*rY*rX;

}


// --------------------------------------------------------------------------
// Quaternion::toEulerAngles
//
// Setup the Euler angles, given an object->inertial rotation quaternion.
// Returned x,y,z are in radians.
// --------------------------------------------------------------------------

template <typename Real>
inline void SMRQuaternionT<Real>::toEulerAngles( Real &x, Real &y, Real &z ) const
{

  //double test = m_x * m_y + m_z * m_w;
  //if (test > 0.499)
  //{
  //  x = 2 * atan2(m_x,m_w);
  //  y = M_PI / 2.0;
  //  z = 0;
  //}else if (test < -0.499)
  //{
  //  x = -2 * atan2(m_x,m_w);
  //  y = M_PI / 2.0;
  //  z = 0;
  //}else
  //{
  //  x = atan2(2*m_y*m_w - 2*m_x*m_z, 1 - 2*m_y*m_y - 2*m_z*m_z);
  //  y = asin(2*test);
  //  z = atan2(2*m_x*m_w - 2*m_y*m_z, 1 - 2*m_x*m_x - 2*m_z*m_z);
  //}


  // Compute Y-axis angle
  y = asin( 2.0 * ((m_x * m_z) + (m_w * m_y)) );

  // Compute cos and one over cos for optimization
  Real cy = cos( y );
  //Real oneOverCosY = 1.0 / cy;

  if( fabs( cy ) > 1E-8 ) {
    // No gimball lock
    x = atan2( 2.0 * ((m_w * m_x) - (m_y * m_z)),
      (1.0 - 2.0 * (m_x*m_x + m_y*m_y)));
    z = atan2( 2.0 * ((m_w * m_z) - (m_x * m_y)),
      (1.0 - 2.0 * (m_y*m_y + m_z*m_z)));
  }
  else {
    // Gimbal lock case
    x = 0.0;
    z = atan2( 2.0 * ((m_x * m_y) + (m_w * m_z)),
      1.0 - 2.0 * (m_x*m_x + m_z*m_z) );
  }

  if (isNaN(x)) x = 0.0;
  if (isNaN(y)) y = 0.0;
  if (isNaN(z)) z = 0.0;
}

// --------------------------------------------------------------------------
// Quaternion::toEulerAngles
//
// Setup the Euler angles, given an object->inertial rotation quaternion.
// Returned x,y,z are in radians.
// --------------------------------------------------------------------------

template <typename Real>
inline void SMRQuaternionT<Real>::toEulerAnglesZYX( Real &x, Real &y, Real &z )
{
  Real transfert;
  transfert = m_z;
  m_z = m_x;
  m_x = transfert;

  // Compute Y-axis angle
  y = asin( 2.0 * ( -1*(m_x * m_z) + (m_w * m_y)) );

  // Compute cos and one over cos for optimization
  Real cy = cos( y );
  //Real oneOverCosY = 1.0 / cy;

  if( fabs( cy ) > 1E-8 ) {
    // No gimball lock
    z = atan2( 2.0 * ((m_w * m_x) + (m_y * m_z)),
      (1.0 - 2.0 * (m_x*m_x + m_y*m_y)));
    x = atan2( 2.0 * ((m_w * m_z) + (m_x * m_y)),
      (1.0 - 2.0 * (m_y*m_y + m_z*m_z)));

  }
  else {
    // Gimbal lock case
    z = 0.0;
    x = atan2( 2.0 * ((m_x * m_y) + (m_z * m_w)),
      (m_x*m_x - m_y*m_y - m_z*m_z + m_w*m_w) );
  }

  if (isNaN(x)) x = 0.0;
  if (isNaN(y)) y = 0.0;
  if (isNaN(z)) z = 0.0;

  transfert = m_z;
  m_z = m_x;
  m_x = transfert;
}

template <typename Real>
inline void SMRQuaternionT<Real>::toEulerAnglesXZY( Real &x, Real &y, Real &z ) const
{
  double p0=m_w;
  double p1=m_x;
  double p2=m_z;
  double p3=m_y;
  double e=-1; // given by  e3 (vec. product) e2 == e * e1

  double tanTheta1N=2*(p0*p1-e*p2*p3);
  double tanTheta1D=(1-2*(p1*p1+p2*p2));
  double sinTheta2=2*(p0*p2+e*p1*p3);
  double tanTheta3N=2*(p0*p3-e*p1*p2);
  double tanTheta3D=(1-2*(p2*p2+p3*p3));

  double theta1=0.0;
  double theta2=asin(sinTheta2);
  double theta3=0.0;

  if(fabs(fabs(theta2)-M_PI/2)<1E-8)
  {
    theta3=0.0;
    theta1=atan2(p1,p0);
  }else{
    theta1=atan2(tanTheta1N,tanTheta1D);
    theta3=atan2(tanTheta3N,tanTheta3D);
  }

  x=theta1;
  y=theta3;
  z=theta2;
}


// --------------------------------------------------------------------------
// Quaternion::toEulerAngles
//
// Setup the Euler angles, given an object->inertial rotation quaternion.
// Returned x,y,z are in radians.
// --------------------------------------------------------------------------

template <typename Real>
inline void SMRQuaternionT<Real>::toEulerAnglesYXZ( Real &x, Real &y, Real &z ) const
{
  double p0=m_w;
  double p1=m_y;
  double p2=m_x;
  double p3=m_z;
  double e=-1;

  double tanTheta1N=2*(p0*p1-e*p2*p3);
  double tanTheta1D=(1-2*(p1*p1+p2*p2));
  double sinTheta2=2*(p0*p2+e*p1*p3);
  double tanTheta3N=2*(p0*p3-e*p1*p2);
  double tanTheta3D=(1-2*(p2*p2+p3*p3));

  double theta1=0.0;
  double theta2=asin(sinTheta2);
  double theta3=0.0;

  if(fabs(fabs(theta2)-M_PI/2)<1E-8f)
  {
    theta3=0.0;
    theta1=atan2(p1,p0);
  }else{
    theta1=atan2(tanTheta1N,tanTheta1D);
    theta3=atan2(tanTheta3N,tanTheta3D);
  }

  x=theta2;
  y=theta1;
  z=theta3;
}

// --------------------------------------------------------------------------
// Quaternion::getRotationAngle
//
// Return the rotation angle theta (in radians).
// --------------------------------------------------------------------------

template <typename Real>
inline Real SMRQuaternionT<Real>::getRotationAngle( void ) const
{
  // Compute the half angle.  Remember that w = cos(theta / 2)
  Real thetaOver2 = safeAcos(m_w);

  // Return the rotation angle
  return thetaOver2 * 2.0;
}


// --------------------------------------------------------------------------
// Quaternion::getRotationAxis
//
// Return the rotation axis.
// --------------------------------------------------------------------------


template <typename Real>
inline SMRVector3 SMRQuaternionT<Real>::getRotationAxis( void ) const
{
  Real sinThetaOver2Sq = 1.0 - (m_w * m_w);

  // Protect against numerical imprecision
  if( sinThetaOver2Sq <= 0.0 ) {
    // Identity quaternion, or numerical imprecision.  Just
    // return any valid vector, since it doesn't matter

    return SMRVector3( 1.0, 0.0, 0.0 );
  }

  // Compute 1 / sin(theta/2)
  Real oneOverSinThetaOver2 = 1.0 / sqrt( sinThetaOver2Sq );

  // Return axis of rotation
  return SMRVector3(
    m_x * oneOverSinThetaOver2,
    m_y * oneOverSinThetaOver2,
    m_z * oneOverSinThetaOver2
    );
}


// --------------------------------------------------------------------------
// Quaternion operators
//
// Operator overloading for basic quaternion operations.
// --------------------------------------------------------------------------

template <typename Real>
inline SMRQuaternionT<Real> SMRQuaternionT<Real>::operator+( const SMRQuaternionT<Real> &q ) const
{
  return SMRQuaternionT<Real>( m_w + q.m_w, m_x + q.m_x, m_y + q.m_y, m_z + q.m_z );
}

template <typename Real>
inline SMRQuaternionT<Real> &SMRQuaternionT<Real>::operator+=( const SMRQuaternionT<Real> &q )
{
  m_w += q.m_w; m_x += q.m_x; m_y += q.m_y; m_z += q.m_z;
  return *this;
}

template <typename Real>
inline SMRQuaternionT<Real> SMRQuaternionT<Real>::operator-( const SMRQuaternionT<Real> &q ) const
{
  return SMRQuaternionT<Real>( m_w - q.m_w, m_x - q.m_x, m_y - q.m_y, m_z - q.m_z );
}

template <typename Real>
inline SMRQuaternionT<Real> &SMRQuaternionT<Real>::operator-=( const SMRQuaternionT<Real> &q )
{
  m_w -= q.m_w; m_x -= q.m_x; m_y -= q.m_y; m_z -= q.m_z;
  return *this;
}

// SMRQuaternionT concatenation.  The order of multiplication, from left
// to right, corresponds to the inverse of the order of rotations, i.e.
// p * q is first a rotation of q and then, a rotation of p.
// NOTE: p * q != q * p
template <typename Real>
inline SMRQuaternionT<Real> SMRQuaternionT<Real>::operator*( const SMRQuaternionT<Real> &q ) const
{
  return SMRQuaternionT<Real>(
    (m_w * q.m_w) - (m_x * q.m_x) - (m_y * q.m_y) - (m_z * q.m_z),
    (m_w * q.m_x) + (m_x * q.m_w) + (m_y * q.m_z) - (m_z * q.m_y),
    (m_w * q.m_y) + (m_y * q.m_w) + (m_z * q.m_x) - (m_x * q.m_z),
    (m_w * q.m_z) + (m_z * q.m_w) + (m_x * q.m_y) - (m_y * q.m_x)
    );
}

template <typename Real>
inline SMRQuaternionT<Real> &SMRQuaternionT<Real>::operator*=( const SMRQuaternionT<Real> &q )
{
  *this = *this * q;
  return *this;
}

template <typename Real>
inline SMRQuaternionT<Real> SMRQuaternionT<Real>::operator*( const SMRVector3T<Real> &v ) const
{
  // q * v = q * p where p = <0,v>
  // Thus, we can simplify the operations.
  return SMRQuaternionT<Real>(
    - (m_x * v.m_x) - (m_y * v.m_y) - (m_z * v.m_z),
    (m_w * v.m_x) + (m_y * v.m_z) - (m_z * v.m_y),
    (m_w * v.m_y) + (m_z * v.m_x) - (m_x * v.m_z),
    (m_w * v.m_z) + (m_x * v.m_y) - (m_y * v.m_x)
    );
}

template <typename Real>
inline SMRQuaternionT<Real> &SMRQuaternionT<Real>::operator*=( const SMRVector3T<Real> &v )
{
  *this = *this * v;
  return *this;
}

template <typename Real>
inline SMRQuaternionT<Real> SMRQuaternionT<Real>::operator*( Real k ) const
{
  return SMRQuaternionT<Real>( m_w * k, m_x * k, m_y * k, m_z * k );
}

template <typename Real>
inline SMRQuaternionT<Real> &SMRQuaternionT<Real>::operator*=( Real k )
{
  m_w *= k; m_x *= k; m_y *= k; m_z *= k;
  return *this;
}

template <typename Real>
inline SMRQuaternionT<Real> SMRQuaternionT<Real>::operator/( Real k ) const
{
  Real oneOverK = 1.0 / k;
  return SMRQuaternionT<Real>( m_w * oneOverK, m_x * oneOverK, m_y * oneOverK, m_z * oneOverK );
}

template <typename Real>
inline SMRQuaternionT<Real> &SMRQuaternionT<Real>::operator/=( Real k )
{
  Real oneOverK = 1.0 / k;
  m_w *= oneOverK; m_x *= oneOverK; m_y *= oneOverK; m_z *= oneOverK;
  return *this;
}

// SMRQuaternionT conjugate
template <typename Real>
inline SMRQuaternionT<Real> SMRQuaternionT<Real>::operator~( void ) const
{
  return SMRQuaternionT<Real>( m_w, -m_x, -m_y, -m_z );
}


// SMRQuaternionT negation
template <typename Real>
inline SMRQuaternionT<Real> SMRQuaternionT<Real>::operator-( void ) const
{
  return SMRQuaternionT<Real>( -m_w, -m_x, -m_y, -m_z );
}

// --------------------------------------------------------------------------
//
// Nonmember Quaternion functions
//
// --------------------------------------------------------------------------

// geodesic distance between 2 quaternions
template <typename Real>
Real GeodesicDistance( const SMRQuaternionT<Real> &a, const SMRQuaternionT<Real> &b )
{
  return (Log(Inverse(a) * b )).norm();
}

// Scalar on left multiplication
template <typename Real>
inline SMRQuaternionT<Real> operator*( Real k, const SMRQuaternionT<Real> &q )
{
  return SMRQuaternionT<Real>( q.m_w * k, q.m_x * k, q.m_y * k, q.m_z * k );
}

// SMRQuaternionT dot product
template <typename Real>
inline Real DotProduct( const SMRQuaternionT<Real> &a, const SMRQuaternionT<Real> &b )
{
  return ((a.m_w * b.m_w) + (a.m_x * b.m_x) + (a.m_y * b.m_y) + (a.m_z * b.m_z));
}

// Compute the quaternion conjugate.  This is the quaternian
// with the opposite rotation as the original quaternion.
template <typename Real>
inline SMRQuaternionT<Real> Conjugate( const SMRQuaternionT<Real> &q )
{
  return SMRQuaternionT<Real>( q.m_w, -q.m_x, -q.m_y, -q.m_z );
}


// Compute the inverse quaternion (for unit quaternion only).
template <typename Real>
inline SMRQuaternionT<Real> Inverse( const SMRQuaternionT<Real> &q )
{
  // Assume this is a unit quaternion! No check for this!
  SMRQuaternionT<Real> res( q.m_w, -q.m_x, -q.m_y, -q.m_z );
  //res.normalize();
  return res;
}

// compute the rotation bewteen 2 vectors
template <typename Real>
SMRQuaternionT<Real> RotationBetweenVectors( const SMRVector3T<Real> &axis1,
                                            const SMRVector3T<Real> &axis2)
{
  SMRVector3T<Real> axis = CrossProduct(axis1,axis2);
  Real angle = safeAcos(DotProduct(axis1,axis2)/(VectorMag(axis1)*VectorMag(axis2)));
  return SMRQuaternionT<Real>(axis,angle);
}

// --------------------------------------------------------------------------
// Log
//
// Unit quaternion logarithm. log(q) = log(cos(theta) + n*sin(theta))
// --------------------------------------------------------------------------

template <typename Real>
inline SMRVector3T<Real> Log( const SMRQuaternionT<Real> &q )
{
  SMRVector3T<Real> res;

  if( fabs( q.m_w ) < 1.0 ) {
    Real theta = acos( q.m_w );
    Real sin_theta = sin( theta );

    if( fabs( sin_theta ) > 0.00001 ) {
      Real thetaOverSinTheta = theta / sin_theta;
      res.m_x = q.m_x * thetaOverSinTheta;
      res.m_y = q.m_y * thetaOverSinTheta;
      res.m_z = q.m_z * thetaOverSinTheta;
      return res;
    }
  }

  res.m_x = q.m_x;
  res.m_y = q.m_y;
  res.m_z = q.m_z;

  return res;
}


// --------------------------------------------------------------------------
// Exp
//
// Quaternion exponential.
// --------------------------------------------------------------------------

template <typename Real>
inline SMRQuaternionT<Real> Exp( const SMRQuaternionT<Real> &q )
{
  Real theta = sqrt( DotProduct( q, q ) );
  Real sin_theta = sin( theta );

  SMRQuaternionT<Real> res;
  res.m_w = cos( theta );

  if( fabs( sin_theta ) > 0.00001 ) {
    Real sinThetaOverTheta = sin_theta / theta;
    res.m_x = q.m_x * sinThetaOverTheta;
    res.m_y = q.m_y * sinThetaOverTheta;
    res.m_z = q.m_z * sinThetaOverTheta;
  }
  else {
    res.m_x = q.m_x;
    res.m_y = q.m_y;
    res.m_z = q.m_z;
  }

  return res;
}

template <typename Real>
inline SMRQuaternionT<Real> Exp( const SMRVector3T<Real> &v)
{
  Real theta = v.norm();
  Real sin_theta = sin( theta );

  SMRQuaternionT<Real> res;
  res.m_w = cos( theta );

  if( fabs( sin_theta ) > 0.00001 ) {
    Real sinThetaOverTheta = sin_theta / theta;
    res.m_x = v.m_x * sinThetaOverTheta;
    res.m_y = v.m_y * sinThetaOverTheta;
    res.m_z = v.m_z * sinThetaOverTheta;
  }
  else {
    res.m_x = v.m_x;
    res.m_y = v.m_y;
    res.m_z = v.m_z;
  }

  return res;
}


// --------------------------------------------------------------------------
// Pow
//
// Quaternion exponentiation.
// --------------------------------------------------------------------------

template <typename Real>
inline SMRQuaternionT<Real> Pow( const SMRQuaternionT<Real> &q, Real exponent )
{
  // Check for the case of an identity quaternion.
  // This will protect against divide by zero
  if( fabs( q.m_w ) > 0.9999 ) {
    return q;
  }

  // Extract the half angle alpha (alpha = theta/2)
  Real alpha = acos( q.m_w );

  // Compute new alpha value
  Real newAlpha = alpha * exponent;

  // Compute new quaternion
  SMRVector3T<Real> n( q.m_x, q.m_y, q.m_z );
  n *= sin( newAlpha ) / sin( alpha );

  return SMRQuaternionT<Real>( cos( newAlpha ), n );
}


// --------------------------------------------------------------------------
// Slerp
//
// Spherical linear interpolation.
// --------------------------------------------------------------------------

template <typename Real>
inline SMRQuaternionT<Real> Slerp( const SMRQuaternionT<Real> &q0, const SMRQuaternionT<Real> &q1, Real t, bool _hemispherize )
{
  // Check for out-of range parameter and return edge points if so
  //if( t <= 0.0 ) return q0;
  //if( t >= 1.0 ) return q1;

  // Compute "cosine of angle between quaternions" using dot product
  Real cosOmega = DotProduct( q0, q1 );

  // If negative dot, use -q1.  Two quaternions q and -q
  // represent the same rotation, but may produce
  // different slerp.  We chose q or -q to rotate using
  // the acute angle.
  Real q1w = q1.m_w;
  Real q1x = q1.m_x;
  Real q1y = q1.m_y;
  Real q1z = q1.m_z;

  if ( _hemispherize)
  { 
    if( cosOmega < 0.0 ) 
    {
      //std::cout<<" inverting quat " << std::endl;
      q1w = -q1w;
      q1x = -q1x;
      q1y = -q1y;
      q1z = -q1z;
      cosOmega = -cosOmega;
    }
  }
  // We should have two unit quaternions, so dot should be <= 1.0
  // assert( cosOmega < 1.1 );

  // Compute interpolation fraction, checking for quaternions
  // almost exactly the same
  Real k0, k1;

  if( cosOmega > 1-1E-8 ) 
  {
    // Very close - just use linear interpolation,
    // which will protect againt a divide by zero
    k0 = 1.0 - t;
    k1 = t;
  }
  else 
  {
    // Compute the sin of the angle using the
    // trig identity sin^2(omega) + cos^2(omega) = 1
    Real sinOmega = sqrt( 1.0 - (cosOmega * cosOmega) );

    // Compute the angle from its sin and cosine
    Real omega = atan2( sinOmega, cosOmega );

    // Compute inverse of denominator, so we only have
    // to divide once
    Real oneOverSinOmega = 1.0 / sinOmega;

    // Compute interpolation parameters
    k0 = sin( (1.0 - t) * omega ) * oneOverSinOmega;
    k1 = sin( t * omega ) * oneOverSinOmega;
  }

  // Interpolate and return new quaternion
  return SMRQuaternionT<Real>(
    (k0 * q0.m_w) + (k1 * q1w),
    (k0 * q0.m_x) + (k1 * q1x),
    (k0 * q0.m_y) + (k1 * q1y),
    (k0 * q0.m_z) + (k1 * q1z)
    );
}


// --------------------------------------------------------------------------
// Squad
//
// Spherical cubic interpolation.
// squad = slerp( slerp( q0, q1, t ), slerp( qa, qb, t ), 2t(1 - t) ).
// --------------------------------------------------------------------------

template <typename Real>
inline SMRQuaternionT<Real> Squad( const SMRQuaternionT<Real> &q0,
                                  const SMRQuaternionT<Real> &qa,
                                  const SMRQuaternionT<Real> &qb,
                                  const SMRQuaternionT<Real> &q1,
                                  Real t )
{
  Real slerp_t = 2.0 * t * (1.0 - t);

  SMRQuaternionT<Real> slerp_q0 = Slerp( q0, q1, t, false );
  SMRQuaternionT<Real> slerp_q1 = Slerp( qa, qb, t,false );

  return Slerp( slerp_q0, slerp_q1, slerp_t ,false);
}

//---------------------------------------------------------------------------
// Interpolate
//
// Cubic interpolation between secondQuat and ThirdQuat, use FirstQuat anf FourthQuat to compute a and b
//
//---------------------------------------------------------------------------
template <typename Real>
inline SMRQuaternionT<Real> Interpolate( const SMRQuaternionT<Real> &firstQuat,
                                        const SMRQuaternionT<Real> &secondQuat,
                                        const SMRQuaternionT<Real> &thirdQuat,
                                        const SMRQuaternionT<Real> &fourthQuat,
                                        const double &t,
                                        const double &_tension,
                                        const double &_bias,
                                        const double &_continuity
                                        )
{
  SMRQuaternionT<Real> a, bPrev, b;
  Intermediate(firstQuat,secondQuat,thirdQuat,a,bPrev,_tension,_bias,_continuity,1 ,11,21);

  Intermediate(secondQuat,thirdQuat,fourthQuat,a,b,_tension,_bias,_continuity,1 ,11,21);

  return Squad(secondQuat,bPrev,a,thirdQuat,t).normalize();
}



// --------------------------------------------------------------------------
// Intermediate
//
// Compute intermediate quaternions for building spline segments.
// --------------------------------------------------------------------------

template <typename Real>
inline void Intermediate( const SMRQuaternionT<Real> &qprev, const SMRQuaternionT<Real> &qcurr,
                         const SMRQuaternionT<Real> &qnext, SMRQuaternionT<Real> &qa,
                         SMRQuaternionT<Real> &qb,
                         const double &_tension, const double &_bias, const double &_continuity,
                         const double &_tqprev, const double &_tqcurr, const double &_tqnext)
{
  /* old version

  // We should have unit quaternions
  assert( DotProduct( qprev, qprev ) <= 1.0001 );
  assert( DotProduct( qcurr, qcurr ) <= 1.0001 );

  SMRQuaternionT<Real> inv_prev = Conjugate( qprev );
  SMRQuaternionT<Real> inv_curr = Conjugate( qcurr );

  //SMRQuaternionT<Real> p0 = inv_prev * qcurr;
  //SMRQuaternionT<Real> p1 = inv_curr * qnext;
  SMRQuaternionT<Real> p0 = inv_curr * qprev;
  SMRQuaternionT<Real> p1 = inv_curr * qnext;



  SMRVector3T<Real> v = - ( Log( p0 ) + Log( p1 ) );
  SMRQuaternionT<Real> arg = (SMRQuaternionT<Real>(0,v.m_x,v.m_y,v.m_z)) * 0.25;

  qa = qcurr * Exp(  arg );
  qb = qa;// qcurr * Exp(- arg );

  */ 
  //here comes the new one, supporting tension, bias and continuity:


  double p0 = (1-_tension)*(1-_continuity)*(1-_bias)* 0.5;
  double p1 = (1-_tension)*(1+_continuity)*(1+_bias)* 0.5;
  double p2 = (1-_tension)*(1+_continuity)*(1-_bias)* 0.5;
  double p3 = (1-_tension)*(1-_continuity)*(1+_bias)* 0.5;

  double deltaN   = _tqnext - _tqcurr;
  double deltaNm1 = _tqcurr - _tqprev;
  double f0 = 2.0 * (deltaNm1 / (deltaNm1 + deltaN));
  double f1 = 2.0 * (deltaN / (deltaNm1 + deltaN));
  assert( DotProduct( qprev, qprev ) <= 1.0001 );
  assert( DotProduct( qcurr, qcurr ) <= 1.0001 );

  SMRQuaternionT<Real> inv_prev = Conjugate( qprev );
  SMRQuaternionT<Real> inv_curr = Conjugate( qcurr );

  SMRQuaternionT<Real> s0 = inv_prev * qcurr;
  SMRQuaternionT<Real> s1 = inv_curr * qnext;


  SMRVector3T<Real> T0 = p0 * (Log (s1)) + p1 * (Log(s0));
  SMRVector3T<Real> T1 = p2 * (Log (s1)) + p3 * (Log(s0));

  SMRVector3T<Real> v =  (f0 * T0) -  Log( s1 );
  SMRVector3T<Real> w =  Log(s0) - (f1 * T1);

  SMRQuaternionT<Real> arg1 = (SMRQuaternionT<Real>(0,v.m_x,v.m_y,v.m_z)) * 0.5;
  SMRQuaternionT<Real> arg2 = (SMRQuaternionT<Real>(0,w.m_x,w.m_y,w.m_z)) * 0.5;


  qa = qcurr * Exp( arg1 );
  qa.normalize();
  qb = qcurr * Exp( arg2 );
  qb.normalize();

}


// --------------------------------------------------------------------------
// filterQuaternions
//
// performs convolution of a quaternionic signal around the quaternionic window qwindow and with coefficients coef
// from Lee 2002 TVCG "General Construction of Time-Domain Filters for Orientation Data"
// --------------------------------------------------------------------------
template <typename Real>
SMRQuaternionT<Real> filterQuaternions( const vector< SMRQuaternionT<Real> > & qwindow, const vector<double> & coef)
{
  // first of all, we need to check that the number of coef = size of window
  assert(qwindow.size()==coef.size());
  SMRQuaternionT<Real> qi = qwindow.at(qwindow.size()/2);
  SMRQuaternionT<Real> iqi = Inverse(qi);
  SMRVector3 wsum(0,0,0);
  for (unsigned int i = 0; i < coef.size() ; i++)
    wsum+= coef.at(i) * Log (iqi * qwindow.at(i));
  // we assume here that the filter is normalized, i.e. sum(coef)=1
  return qi * Exp(wsum);
}

// --------------------------------------------------------------------------
// smoothQuaternions
//
// smoothing operations on quaternionic values
// from Lee 2002 TVCG "General Construction of Time-Domain Filters for Orientation Data"
// --------------------------------------------------------------------------
template <typename Real>
SMRQuaternionT<Real> smoothQuaternions( const SMRQuaternionT<Real> & q0, const SMRQuaternionT<Real> & q1, const SMRQuaternionT<Real> & q2, const SMRQuaternionT<Real> & q3,
                                       double smooth)
{
  vector< SMRQuaternionT<Real> > qw;
  qw.push_back(q0);qw.push_back(q1);qw.push_back(q2);qw.push_back(q3);
  vector<double> coef;
  coef.push_back(smooth/24.0);
  coef.push_back(-3.0*smooth/24.0);
  coef.push_back(3.0*smooth/24.0);
  coef.push_back(-smooth/24.0);
  return filterQuaternions(qw,coef);
}

// --------------------------------------------------------------------------
// blurQuaternions
//
// bluring processing on quaternionic signal
// from Lee 2002 TVCG "General Construction of Time-Domain Filters for Orientation Data"
// --------------------------------------------------------------------------
template <typename Real>
SMRQuaternionT<Real> blurQuaternions( const SMRQuaternionT<Real> & q0, const SMRQuaternionT<Real> & q1, const SMRQuaternionT<Real> & q2, const SMRQuaternionT<Real> & q3)
{
  vector< SMRQuaternionT<Real> > qw;
  qw.push_back(q0);qw.push_back(q1);qw.push_back(q2);qw.push_back(q3);
  vector<double> coef;
  coef.push_back(-1/16.0);
  coef.push_back(-5/16.0);
  coef.push_back(5/16.0);
  coef.push_back(1/16.0);
  return filterQuaternions(qw,coef);
}

template <typename Real>
SMRQuaternionT<Real> computeTangentsAB(const SMRQuaternionT<Real> & qprev, const SMRQuaternionT<Real> & q0, const SMRQuaternionT<Real> & q1,const SMRQuaternionT<Real> & qnext, SMRQuaternionT<Real> & a, SMRQuaternionT<Real> & b)
{}/*
  assert( DotProduct( qprev, qprev ) <= 1.0001 );
  assert( DotProduct( qcurr, qcurr ) <= 1.0001 );
  assert( DotProduct( qnext, qnext ) <= 1.0001 );
  SMRQuaternionT<Real> inv_q0 = Conjugate( q0 );
  SMRQuaternionT<Real> inv_q1 = Conjugate( q1 );

  SMRQuaternionT<Real> p0 = inv_q0 * ;
  SMRQuaternionT<Real> p1 = inv_q0 * q;

  SMRQuaternionT<Real> p0 = inv_next * qprev;
  SMRQuaternionT<Real> p1 = inv_next * qnext;
  }*/
