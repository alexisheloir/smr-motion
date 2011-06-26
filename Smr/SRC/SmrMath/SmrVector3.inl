template <typename Real>
inline bool SMRVector3T<Real>::isZero( void )
{
  return (m_x == 0.0) && (m_y == 0.0) && (m_z == 0.0);
}

template <typename Real>
inline SMRVector3T<Real> & SMRVector3T<Real>::normalize( void )
{
  Real magSq = (m_x * m_x) + (m_y * m_y) + (m_z * m_z);

  if( magSq > 0.0 ) { // check for divide-by-zero
	Real oneOverMag = 1.0 / sqrt( magSq );
	m_x *= oneOverMag;
	m_y *= oneOverMag;
	m_z *= oneOverMag;
  }

  return *this;
}

template <typename Real>
inline Real SMRVector3T<Real>::norm() const
{
  return sqrt( (m_x * m_x) +  (m_y * m_y) +  (m_z * m_z) );
}

template <typename Real>
inline bool SMRVector3T<Real>::operator==( const SMRVector3T<Real> &v ) const
{
  return ((m_x == v.m_x) && (m_y == v.m_y) && (m_z == v.m_z));
}

template <typename Real>
inline bool SMRVector3T<Real>::operator!=( const SMRVector3T<Real> &v ) const
{
  return ((m_x != v.m_x) || (m_y != v.m_y) || (m_z != v.m_z));
}

template <typename Real>
inline SMRVector3T<Real> SMRVector3T<Real>::operator-( void ) const
{
  return SMRVector3T<Real>( -m_x, -m_y, -m_z );
}

template <typename Real>
inline SMRVector3T<Real> SMRVector3T<Real>::operator+( const SMRVector3T<Real> &v ) const
{
  return SMRVector3T<Real>( m_x + v.m_x, m_y + v.m_y, m_z + v.m_z );
}

template <typename Real>
inline SMRVector3T<Real> SMRVector3T<Real>::operator-( const SMRVector3T<Real> &v ) const
{
  return SMRVector3T<Real>( m_x - v.m_x, m_y - v.m_y, m_z - v.m_z );
}

template <typename Real>
inline SMRVector3T<Real> SMRVector3T<Real>::operator*( Real s ) const
{
  return SMRVector3T<Real>( m_x * s, m_y * s, m_z * s );
}

template <typename Real>
inline SMRVector3T<Real> SMRVector3T<Real>::operator/( Real s ) const
{
  Real oneOverS = 1.0 / s; // Note: no check for divide by zero
  return SMRVector3T<Real>( m_x * oneOverS, m_y * oneOverS, m_z * oneOverS );
}

template <typename Real>
inline SMRVector3T<Real> &SMRVector3T<Real>::operator+=( const SMRVector3T<Real> &v )
{
  m_x += v.m_x; m_y += v.m_y; m_z += v.m_z;
  return *this;
}

template <typename Real>
inline SMRVector3T<Real> &SMRVector3T<Real>::operator-=( const SMRVector3T<Real> &v )
{
  m_x -= v.m_x; m_y -= v.m_y; m_z -= v.m_z;
  return *this;
}

template <typename Real>
inline SMRVector3T<Real> &SMRVector3T<Real>::operator*=( Real s )
{
  m_x *= s; m_y *= s; m_z *= s;
  return *this;
}

template <typename Real>
inline SMRVector3T<Real> &SMRVector3T<Real>::operator/=( Real s )
{
  Real oneOverS = 1.0 / s; // Note: no check for divide by zero
  m_x *= oneOverS; m_y *= oneOverS ; m_z *= oneOverS;
  return *this;
}

// Scalar on the left multiplication, for symmetry
template <typename Real>
inline SMRVector3T<Real> operator*( Real k, SMRVector3T<Real> v )
{
  return SMRVector3T<Real>( k * v.m_x, k * v.m_y, k * v.m_z );
}

// Vector3 dot product
template <typename Real>
inline Real DotProduct( const SMRVector3T<Real> &a, const SMRVector3T<Real> &b )
{
  return ((a.m_x * b.m_x) +  (a.m_y * b.m_y) +  (a.m_z * b.m_z));
}

// Vector3 cross product
template <typename Real>
inline SMRVector3T<Real> CrossProduct( const SMRVector3T<Real> &a, const SMRVector3T<Real> &b )
{
  return SMRVector3T<Real>(
		(a.m_y * b.m_z) - (a.m_z * b.m_y),
		(a.m_z * b.m_x) - (a.m_x * b.m_z),
		(a.m_x * b.m_y) - (a.m_y * b.m_x)
	);
}

// Compute normal plane given three points
template <typename Real>
inline SMRVector3T<Real> ComputeNormal( const SMRVector3T<Real> &p1,
										const SMRVector3T<Real> &p2,
										const SMRVector3T<Real> &p3 )
{
  SMRVector3T<Real> vec1( p1 - p2 );
  SMRVector3T<Real> vec2( p1 - p3 );
  SMRVector3T<Real> result( CrossProduct( vec1, vec2 ) );
  result.normalize();

  return result;
}

// Compute distance between two points
template <typename Real>
inline Real Distance( const SMRVector3T<Real> &a, const SMRVector3T<Real> &b )
{
  Real dx = a.m_x - b.m_x;
  Real dy = a.m_y - b.m_y;
  Real dz = a.m_z - b.m_z;
  return sqrt( (dx * dx) + (dy * dy) + (dz * dz) );
}

// get vector's norm
template <typename Real>
inline Real Norm( const SMRVector3T<Real> &a )
{
	return a.norm();
}

// Compute squared distance between two points.
// Useful when comparing distances, since we don't need
// to square the result.
template <typename Real>
inline Real DistanceSquared( const SMRVector3T<Real> &a, const SMRVector3T<Real> &b )
{
  Real dx = a.m_x - b.m_x;
  Real dy = a.m_y - b.m_y;
  Real dz = a.m_z - b.m_z;
  return ((dx * dx) + (dy * dy) + (dz * dz));
}

template <typename Real>
SMRVector3T<Real> TBCInterpolation(const SMRVector3T<Real> & _Xim1, const SMRVector3T<Real> & _Xi, const SMRVector3T<Real> & _Xip1, const SMRVector3T<Real> & _Xip2, const double _tension, const double _bias, const double _continuity, const double _alpha)
{
  SMRVector3T<Real> ds0 = _Xip1 - _Xi;
  SMRVector3T<Real> ds1 = _Xip2 - _Xip1;
  SMRVector3T<Real> dd0 = _Xi   - _Xim1;
  SMRVector3T<Real> dd1 = _Xip1 - _Xi;
  
  Real dsFac0 = ( 1.0 - _tension ) * ( 1.0 - _continuity ) * ( 1.0 + _bias) / 2.0;
  Real dsFac1 = ( 1.0 - _tension ) * ( 1.0 + _continuity ) * ( 1.0 - _bias) / 2.0;
  Real ddFac0 = ( 1.0 - _tension ) * ( 1.0 + _continuity ) * ( 1.0 + _bias) / 2.0;
  Real ddFac1 = ( 1.0 - _tension ) * ( 1.0 - _continuity ) * ( 1.0 - _bias) / 2.0;  
  
  SMRVector3T<Real> DSip1 =  dsFac0 * ds0 + dsFac1 * ds1;
  SMRVector3T<Real> DDi   =  ddFac0 * dd0 + ddFac1 * dd1;

  Matrix S(1,4);
  S = 0.0;
  S.element(0,3) = 1.0;
  S.element(0,2) = _alpha;
  double help = _alpha*_alpha;
  S.element(0,1) = help;
  S.element(0,0) = help * _alpha;

  Matrix h(4,4);
  h = 0.0;
  h.element(0,0) = 2.0;
  h.element(1,0) = -3.0;
  h.element(3,0) = 1.0;
  h.element(0,1) = -2.0;
  h.element(1,1) = 3.0;
  h.element(0,2) = 1.0;
  h.element(1,2) = -2.0;
  h.element(2,2) = 1.0;
  h.element(0,3) = 1.0;
  h.element(1,3) = -1.0;

  Matrix pX(4,1);
  Matrix pY(4,1);
  Matrix pZ(4,1);
  pX.element(0,0) = _Xi.X();
  pY.element(0,0) = _Xi.Y();
  pZ.element(0,0) = _Xi.Z();
  pX.element(1,0) = _Xip1.X();
  pY.element(1,0) = _Xip1.Y();
  pZ.element(1,0) = _Xip1.Z();
  pX.element(2,0) = DDi.X();
  pY.element(2,0) = DDi.Y();
  pZ.element(2,0) = DDi.Z();
  pX.element(3,0) = DSip1.X();
  pY.element(3,0) = DSip1.Y();
  pZ.element(3,0) = DSip1.Z();
  
  Matrix newX;
  newX = (S * h) * pX;
  Matrix newY;
  newY= (S * h) * pY;
  Matrix newZ;
  newZ= (S * h) * pZ;
  SMRVector3T<Real> result(newX.element(0,0),newY.element(0,0),newZ.element(0,0));
  return result;
}

